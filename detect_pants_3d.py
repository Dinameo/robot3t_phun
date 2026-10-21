import os
import glob
import numpy as np
import cv2
from sklearn.linear_model import RANSACRegressor, LinearRegression

class PantsDetector3D:
    """
    3D Point Cloud-based Garment Detector for Zivid 2 M70 datasets.
    
    Tự động cô lập vùng quần (Dynamic Isolation):
    1. Không dùng bounding box cố định (table_roi_box).
    2. RANSAC tự động tìm mặt phẳng bàn làm việc thống trị (Dominant Working Plane).
    3. Trích xuất các đối tượng nhô lên khỏi mặt bàn (height >= 1.8mm).
    4. Tự động nhận diện cụm quần dựa trên đặc trưng:
       - Là khối liên thông lớn nhất (diện tích > 100,000 pixel, tương đương > 150 cm^2).
       - Nằm hoàn toàn bên trong vùng sàn/bàn trống (không dính viền ảnh ngoại cảnh).
    5. Tinh chỉnh biên cục bộ (Edge Refinement) để ôm trọn 100% gấu quần và nếp gấp.
    6. Trích xuất thông số 3D, xuất mask, overlay, nhãn YOLO-seg và báo cáo CSV.
    """
    def __init__(self, height_threshold_mm=3, max_height_mm=120.0, min_pants_pixels=100000):
        self.height_threshold_mm = height_threshold_mm
        self.max_height_mm = max_height_mm
        self.min_pants_pixels = min_pants_pixels

    def load_ply(self, ply_path):
        """Loads organized Zivid point cloud from binary PLY file."""
        dt = np.dtype([('x', '<f4'), ('y', '<f4'), ('z', '<f4'), ('r', 'u1'), ('g', 'u1'), ('b', 'u1')])
        with open(ply_path, 'rb') as f:
            while True:
                line = f.readline()
                if line.strip() == b'end_header':
                    break
            data = np.fromfile(f, dtype=dt, count=2332800)

        h, w = 1200, 1944
        xyz = np.stack([data['x'].reshape(h, w), data['y'].reshape(h, w), data['z'].reshape(h, w)], axis=-1)
        bgr = np.stack([data['b'].reshape(h, w), data['g'].reshape(h, w), data['r'].reshape(h, w)], axis=-1)
        rgb = np.stack([data['r'].reshape(h, w), data['g'].reshape(h, w), data['b'].reshape(h, w)], axis=-1)
        return xyz, bgr, rgb

    def fit_dominant_plane(self, xyz):
        """
        Tìm mặt phẳng làm việc lớn nhất (Mặt bàn/sàn) bằng RANSAC trên toàn bộ FOV.
        Không phụ thuộc vào bất kỳ vùng ROI cứng nào.
        """
        x = xyz[:, :, 0]
        y = xyz[:, :, 1]
        z = xyz[:, :, 2]

        # Lọc các điểm 3D hợp lệ trong dải làm việc của camera Zivid (1.0m - 2.2m)
        valid = (z > 1000.0) & (z < 2200.0)
        pts_xy = np.stack([x[valid], y[valid]], axis=-1)
        pts_z = z[valid]

        if len(pts_xy) < 5000:
            raise ValueError("Không đủ điểm 3D hợp lệ để xác định mặt bàn.")

        # Lấy mẫu ngẫu nhiên để RANSAC chạy siêu nhanh (< 50ms)
        n_samples = min(40000, len(pts_xy))
        idx = np.random.choice(len(pts_xy), size=n_samples, replace=False)

        ransac = RANSACRegressor(LinearRegression(), min_samples=3, residual_threshold=2.5, max_trials=100)
        ransac.fit(pts_xy[idx], pts_z[idx])

        a, b_coef = ransac.estimator_.coef_
        c = ransac.estimator_.intercept_
        norm = np.sqrt(a * a + b_coef * b_coef + 1.0)
        normal = np.array([-a / norm, -b_coef / norm, 1.0 / norm])

        plane_params = {'a': float(a), 'b': float(b_coef), 'c': float(c), 'normal': normal}
        return plane_params

    def segment_pants(self, xyz, bgr, plane_params):
        """
        Tự động cô lập vùng quần dựa trên đặc trưng:
        - Nhô cao hơn mặt bàn (height >= 1.8mm).
        - Khối liền mạch lớn nhất nằm trọn giữa không gian sàn trống bao quanh.
        - Tinh chỉnh biên tự động ôm trọn gấu quần.
        """
        h, w = xyz.shape[:2]
        x, y, z = xyz[:, :, 0], xyz[:, :, 1], xyz[:, :, 2]
        r = bgr[:, :, 2].astype(np.float32)
        g = bgr[:, :, 1].astype(np.float32)
        b = bgr[:, :, 0].astype(np.float32)

        a = plane_params['a']
        b_coef = plane_params['b']
        c = plane_params['c']

        # Tính độ cao của từng điểm so với mặt phẳng bàn
        z_table = a * x + b_coef * y + c
        height = z_table - z

        valid_z = (z > 1000.0) & (z < 2200.0)
        
        # Mặt bàn trống là nơi có khoảng cách tới mặt phẳng <= 2.0mm
        is_table_plane = np.abs(height) <= 2.0

        # Bước 1: Lọc tất cả các vật thể nhô lên khỏi mặt bàn
        objects_raw = (height >= self.height_threshold_mm) & (height <= self.max_height_mm) & valid_z
        
        k_morph = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (9, 9))
        objects_clean = cv2.morphologyEx(objects_raw.astype(np.uint8) * 255, cv2.MORPH_CLOSE, k_morph)
        objects_clean = cv2.morphologyEx(objects_clean, cv2.MORPH_OPEN, k_morph)

        # Bước 2: Phân tích thành phần liên thông để cô lập khối quần
        num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(objects_clean)
        
        valid_candidates = []
        for lbl in range(1, num_labels):
            area = stats[lbl, cv2.CC_STAT_AREA]
            left = stats[lbl, cv2.CC_STAT_LEFT]
            top = stats[lbl, cv2.CC_STAT_TOP]
            box_w = stats[lbl, cv2.CC_STAT_WIDTH]
            box_h = stats[lbl, cv2.CC_STAT_HEIGHT]

            # Loại bỏ các đối tượng dính sát viền khung hình (tủ bên dưới, tường/sàn ngoài rìa)
            touches_border = (left <= 3) or (top <= 3) or (left + box_w >= w - 3) or (top + box_h >= h - 3)
            
            # Quần luôn là cụm độc lập nằm giữa sàn trống và có diện tích lớn
            if not touches_border and area >= self.min_pants_pixels:
                valid_candidates.append((lbl, area))

        if not valid_candidates:
            # Fallback nếu quần nằm hơi sát mép: lấy cụm lớn nhất
            largest_label = 1 + np.argmax(stats[1:, cv2.CC_STAT_AREA])
            pants_seed = (labels == largest_label).astype(np.uint8) * 255
        else:
            best_lbl = max(valid_candidates, key=lambda it: it[1])[0]
            pants_seed = (labels == best_lbl).astype(np.uint8) * 255

        # Bước 3: Tinh chỉnh bám cạnh chuẩn xác (3D-Guided GrabCut Edge Snapping)
        # Giúp mask ôm sát từng thớ vải/đường chỉ viền, triệt tiêu hoàn toàn phần lem ra sàn nhà
        bx, by, bw, bh = cv2.boundingRect(pants_seed)
        margin = 35
        bx0 = max(0, bx - margin)
        by0 = max(0, by - margin)
        bx1 = min(w, bx + bw + margin)
        by1 = min(h, by + bh + margin)

        crop_bgr = bgr[by0:by1, bx0:bx1]
        crop_h = height[by0:by1, bx0:bx1]
        crop_fg = pants_seed[by0:by1, bx0:bx1]

        # Khởi tạo ma trận nhãn GrabCut trong vùng ROI của quần:
        # 0 = Chắc chắn là nền (BGD), 1 = Chắc chắn là quần (FGD)
        # 2 = Khả năng là nền (PR_BGD), 3 = Khả năng là quần (PR_FGD)
        gc_mask = np.full(crop_bgr.shape[:2], cv2.GC_PR_BGD, dtype=np.uint8)

        # Mặt sàn thực tế (độ cao <= 1.2mm hoặc sàn phẳng sáng màu) chắc chắn là Background
        brightness_crop = (crop_bgr[:, :, 0].astype(np.float32) + crop_bgr[:, :, 1].astype(np.float32) + crop_bgr[:, :, 2].astype(np.float32)) / 3.0
        is_floor = (crop_h <= 1.2) | ((crop_h <= 2.2) & (brightness_crop > 150.0))
        gc_mask[is_floor] = cv2.GC_BGD

        # Lõi độ cao 3D (h >= 4.5mm) chắc chắn là Foreground
        sure_fg = (crop_fg > 0) & (crop_h >= 4.5)
        gc_mask[sure_fg] = cv2.GC_FGD

        # Vùng chuyển tiếp mép gấu (1.5mm - 4.5mm) hoặc nếp vải là Probable Foreground
        probable_fg = (crop_fg > 0) & (crop_h >= 1.5) & (~is_floor)
        gc_mask[probable_fg] = cv2.GC_PR_FGD

        # Chạy GrabCut tối ưu hóa năng lượng viền cắt
        bgd_model = np.zeros((1, 65), np.float64)
        fgd_model = np.zeros((1, 65), np.float64)
        cv2.grabCut(crop_bgr, gc_mask, None, bgd_model, fgd_model, 4, cv2.GC_INIT_WITH_MASK)

        crop_snapped = np.where((gc_mask == cv2.GC_FGD) | (gc_mask == cv2.GC_PR_FGD), 255, 0).astype(np.uint8)

        # Bước 4: Làm mịn và điền kín các nếp gấp bên trong (Fill holes)
        contours, _ = cv2.findContours(crop_snapped, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        crop_final = np.zeros_like(crop_snapped)
        cv2.drawContours(crop_final, contours, -1, 255, thickness=cv2.FILLED)

        mask_final = np.zeros(bgr.shape[:2], dtype=np.uint8)
        mask_final[by0:by1, bx0:bx1] = crop_final

        return mask_final, height

    def compute_3d_metrics(self, xyz, mask, height_map):
        """Tính toán kích thước hình học 3D, tâm thực tế và diện tích."""
        mask_bool = (mask > 0)
        pts_x = xyz[:, :, 0][mask_bool]
        pts_y = xyz[:, :, 1][mask_bool]
        pts_z = xyz[:, :, 2][mask_bool]
        pts_h = height_map[mask_bool]

        if len(pts_x) == 0:
            return None

        centroid_3d = [float(np.mean(pts_x)), float(np.mean(pts_y)), float(np.mean(pts_z))]
        
        bbox_3d = {
            'x_min': float(np.min(pts_x)), 'x_max': float(np.max(pts_x)), 'length_x': float(np.max(pts_x) - np.min(pts_x)),
            'y_min': float(np.min(pts_y)), 'y_max': float(np.max(pts_y)), 'length_y': float(np.max(pts_y) - np.min(pts_y)),
            'z_min': float(np.min(pts_z)), 'z_max': float(np.max(pts_z)),
            'height_min': float(np.min(pts_h)), 'height_max': float(np.max(pts_h)), 'height_mean': float(np.mean(pts_h))
        }

        y_indices, x_indices = np.where(mask_bool)
        bbox_2d = {
            'x': int(np.min(x_indices)), 'y': int(np.min(y_indices)),
            'w': int(np.max(x_indices) - np.min(x_indices)),
            'h': int(np.max(y_indices) - np.min(y_indices))
        }

        area_pixels = int(np.sum(mask_bool))
        approx_area_mm2 = area_pixels * (0.387 ** 2)

        return {
            'centroid_3d_mm': centroid_3d,
            'bbox_3d_mm': bbox_3d,
            'bbox_2d_pixels': bbox_2d,
            'pixel_count': area_pixels,
            'approx_area_cm2': float(approx_area_mm2 / 100.0)
        }

    def export_yolo_seg_label(self, mask, out_txt_path, class_id=0):
        """Xuất nhãn đa giác chuẩn YOLO11-seg."""
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return
        
        c = max(contours, key=cv2.contourArea)
        epsilon = 0.0015 * cv2.arcLength(c, True)
        approx = cv2.approxPolyDP(c, epsilon, True)
        
        h, w = mask.shape
        poly = approx.reshape(-1, 2).astype(np.float32)
        poly[:, 0] /= w
        poly[:, 1] /= h
        
        coords_str = " ".join([f"{pt[0]:.6f} {pt[1]:.6f}" for pt in poly])
        with open(out_txt_path, 'w') as f:
            f.write(f"{class_id} {coords_str}\n")

    def generate_height_overlay(self, xyz, bgr, mask, height_map, max_display_h=100.0):
        """
        Tô màu vùng quần theo độ cao 3D (Elevation Heatmap) kèm thanh Colorbar chuẩn milimet.
        Thang màu Turbo:
        - Tím / Xanh dương: Sát mặt bàn (0 - 15mm, gấu quần)
        - Xanh lục: Thân ống (20 - 50mm)
        - Vàng / Cam: Nếp gấp, đùi (50 - 75mm)
        - Đỏ: Điểm cao nhất (75 - 100mm+, cạp, túi, đũng quần)
        """
        z = xyz[:, :, 2]
        valid_z = (z > 500) & (mask > 0)

        h_map = np.clip(height_map, 0.0, 120.0)
        invalid_inside = (mask > 0) & (~valid_z)
        if np.any(invalid_inside):
            h_norm = (h_map / 120.0 * 255).astype(np.uint8)
            inpaint_mask = invalid_inside.astype(np.uint8) * 255
            h_inp = cv2.inpaint(h_norm, inpaint_mask, 5, cv2.INPAINT_TELEA)
            h_map = h_inp.astype(np.float32) / 255.0 * 120.0

        # Chuẩn hóa về [0, 1] theo dải 0 - max_display_h (mm)
        h_norm = np.clip(h_map / max_display_h, 0.0, 1.0)
        h_uint8 = (h_norm * 255).astype(np.uint8)

        # Áp dụng bảng màu Turbo trực quan
        colormap = cv2.applyColorMap(h_uint8, cv2.COLORMAP_TURBO)

        # Hòa trộn với ảnh gốc để vừa thấy độ cao vừa thấy chi tiết vải
        height_overlay = bgr.copy()
        blended = cv2.addWeighted(bgr, 0.35, colormap, 0.65, 0)
        height_overlay[mask > 0] = blended[mask > 0]

        # Vẽ đường viền trắng tinh tế quanh quần
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(height_overlay, contours, -1, (255, 255, 255), 2)

        # Vẽ thanh Colorbar chú thích ở góc phải
        cb_x, cb_y, cb_w, cb_h = 1820, 200, 30, 400
        grad = np.linspace(255, 0, cb_h).astype(np.uint8).reshape(-1, 1)
        grad_img = np.tile(grad, (1, cb_w))
        cb_colors = cv2.applyColorMap(grad_img, cv2.COLORMAP_TURBO)

        cv2.rectangle(height_overlay, (cb_x - 12, cb_y - 32), (cb_x + cb_w + 80, cb_y + cb_h + 30), (30, 30, 30), -1)
        cv2.rectangle(height_overlay, (cb_x - 12, cb_y - 32), (cb_x + cb_w + 80, cb_y + cb_h + 30), (200, 200, 200), 1)
        height_overlay[cb_y:cb_y+cb_h, cb_x:cb_x+cb_w] = cb_colors

        cv2.putText(height_overlay, 'Height', (cb_x - 5, cb_y - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1, cv2.LINE_AA)
        steps = [0, 25, 50, 75, 100]
        for s in steps:
            y_pos = int(cb_y + cb_h - (s / max_display_h) * cb_h)
            cv2.line(height_overlay, (cb_x + cb_w, y_pos), (cb_x + cb_w + 6, y_pos), (255, 255, 255), 1)
            cv2.putText(height_overlay, f'{s}mm', (cb_x + cb_w + 10, y_pos + 4), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1, cv2.LINE_AA)

        return height_overlay

    def process_file(self, ply_path, output_dir):
        """Xử lý đơn lẻ 1 file PLY."""
        os.makedirs(output_dir, exist_ok=True)
        base_name = os.path.splitext(os.path.basename(ply_path))[0]

        xyz, bgr, rgb = self.load_ply(ply_path)
        plane_params = self.fit_dominant_plane(xyz)
        mask, height_map = self.segment_pants(xyz, bgr, plane_params)
        metrics = self.compute_3d_metrics(xyz, mask, height_map)

        # 1. Tạo ảnh hiển thị trực quan mask xanh lá
        overlay = bgr.copy()
        overlay[mask > 0] = [0, 255, 0]  # Phủ màu xanh lá lên quần
        blend = cv2.addWeighted(bgr, 0.65, overlay, 0.35, 0)
        
        if metrics:
            bb = metrics['bbox_2d_pixels']
            cv2.rectangle(blend, (bb['x'], bb['y']), (bb['x'] + bb['w'], bb['y'] + bb['h']), (0, 0, 255), 2)
            c3d = metrics['centroid_3d_mm']
            info_str = f"Centroid: [{c3d[0]:.1f}, {c3d[1]:.1f}, {c3d[2]:.1f}] mm | Area: {metrics['approx_area_cm2']:.1f} cm2"
            cv2.putText(blend, info_str, (bb['x'], max(30, bb['y'] - 10)), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

        # 2. Tạo ảnh tô màu theo độ cao 3D (Elevation Colormap)
        height_overlay = self.generate_height_overlay(xyz, bgr, mask, height_map)

        mask_path = os.path.join(output_dir, f"{base_name}_mask.png")
        overlay_path = os.path.join(output_dir, f"{base_name}_overlay.png")
        height_overlay_path = os.path.join(output_dir, f"{base_name}_height_overlay.png")
        yolo_path = os.path.join(output_dir, f"{base_name}_yolo.txt")
        json_path = os.path.join(output_dir, f"{base_name}_metrics.json")

        cv2.imwrite(mask_path, mask)
        cv2.imwrite(overlay_path, blend)
        cv2.imwrite(height_overlay_path, height_overlay)
        self.export_yolo_seg_label(mask, yolo_path)

        import json
        with open(json_path, 'w') as f:
            json.dump(metrics, f, indent=2)

        return metrics, overlay_path

    def process_all(self, input_dir, output_dir):
        """Xử lý hàng loạt toàn bộ thư mục 3D PLY và xuất báo cáo CSV."""
        import glob
        import csv

        os.makedirs(output_dir, exist_ok=True)
        ply_files = sorted(glob.glob(os.path.join(input_dir, "*.ply")))
        print(f"Found {len(ply_files)} PLY files in {input_dir}. Starting dynamic isolation batch...")

        results = []
        for idx, ply_p in enumerate(ply_files, 1):
            base_name = os.path.basename(ply_p)
            print(f"[{idx}/{len(ply_files)}] Processing {base_name}...")
            try:
                metrics, overlay_p = self.process_file(ply_p, output_dir)
                c3d = metrics['centroid_3d_mm']
                b3d = metrics['bbox_3d_mm']
                results.append({
                    'file': base_name,
                    'centroid_x_mm': round(c3d[0], 2),
                    'centroid_y_mm': round(c3d[1], 2),
                    'centroid_z_mm': round(c3d[2], 2),
                    'length_x_mm': round(b3d['length_x'], 2),
                    'length_y_mm': round(b3d['length_y'], 2),
                    'height_mean_mm': round(b3d['height_mean'], 2),
                    'height_max_mm': round(b3d['height_max'], 2),
                    'area_cm2': round(metrics['approx_area_cm2'], 2),
                    'pixel_count': metrics['pixel_count'],
                    'status': 'SUCCESS'
                })
            except Exception as e:
                print(f"  [ERROR] Failed processing {base_name}: {e}")
                results.append({'file': base_name, 'status': f'FAILED: {str(e)}'})

        csv_path = os.path.join(output_dir, "batch_3d_detection_summary.csv")
        if results:
            keys = results[0].keys()
            with open(csv_path, 'w', newline='', encoding='utf-8') as f:
                writer = csv.DictWriter(f, fieldnames=keys)
                writer.writeheader()
                writer.writerows(results)
            print(f"\n[DONE] Summary report saved to: {csv_path}")

        return results

if __name__ == '__main__':
    import argparse

    parser = argparse.ArgumentParser(description="Zivid 3D Garment Detector (Dynamic Table Isolation)")
    parser.add_argument('--input', type=str, default=r'dataset\_3D', help="Path to .ply file or folder")
    parser.add_argument('--output', type=str, default='output_detection', help="Output directory")
    parser.add_argument('--height_thresh', type=float, default=1.8, help="Min height above table in mm (default: 1.8)")
    args = parser.parse_args()

    detector = PantsDetector3D(height_threshold_mm=args.height_thresh)

    if os.path.isfile(args.input):
        print(f"Processing single file: {args.input}")
        metrics, overlay_p = detector.process_file(args.input, args.output)
        print("Completed single file detection.")
    elif os.path.isdir(args.input):
        detector.process_all(args.input, args.output)
    else:
        print(f"Input path not found: {args.input}")
