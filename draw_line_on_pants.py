import os
import sys
import glob
import math
import argparse

# Đảm bảo in UTF-8 trơn tru trên Windows console
if sys.platform.startswith('win'):
    try:
        sys.stdout.reconfigure(encoding='utf-8')
        sys.stderr.reconfigure(encoding='utf-8')
    except Exception:
        pass

import numpy as np
import cv2
import trimesh
from scipy.spatial import cKDTree
from sklearn.linear_model import RANSACRegressor, LinearRegression

class VirtualPenDrawer3D:
    """
    Chương trình lấy file PLY bất kỳ, xác định vị trí nhô cao ở trung tâm ảnh (vùng quần),
    và vẽ một đường thẳng độ dài 100mm trên bề mặt vi mô.
    Tại mỗi vị trí vẽ, hướng bút ảo luôn luôn vuông góc với mặt phẳng vi mô cục bộ.
    """

    def __init__(self, line_length_mm=100.0, step_mm=1.0, micro_radius_mm=6.0, pen_length_mm=35.0):
        self.line_length_mm = float(line_length_mm)
        self.step_mm = float(step_mm)
        self.micro_radius_mm = float(micro_radius_mm)
        self.pen_length_mm = float(pen_length_mm)

    def load_ply(self, ply_path):
        """
        Đọc file PLY: Hỗ trợ cả file cấu trúc Zivid chuẩn (1200x1944) 
        lẫn file PLY dạng point cloud bất kỳ.
        """
        if not os.path.exists(ply_path):
            raise FileNotFoundError(f"Không tìm thấy file PLY: {ply_path}")

        print(f"[1/6] Đang đọc file PLY: {os.path.basename(ply_path)}...")
        
        # Thử đọc header để kiểm tra định dạng
        is_zivid_organized = False
        num_vertices = 0
        is_binary = False
        header_lines = []

        with open(ply_path, 'rb') as f:
            while True:
                line = f.readline()
                if not line:
                    break
                header_lines.append(line.decode('latin1', errors='ignore').strip())
                if b'element vertex' in line:
                    num_vertices = int(line.split()[-1])
                if b'format binary' in line:
                    is_binary = True
                if line.strip() == b'end_header':
                    break
            
            header_bytes = f.tell()

        if num_vertices == 2332800:  # 1200 * 1944
            is_zivid_organized = True

        if is_zivid_organized:
            dt = np.dtype([('x', '<f4'), ('y', '<f4'), ('z', '<f4'), ('r', 'u1'), ('g', 'u1'), ('b', 'u1')])
            with open(ply_path, 'rb') as f:
                f.seek(header_bytes)
                data = np.fromfile(f, dtype=dt, count=num_vertices)
            
            h, w = 1200, 1944
            xyz = np.stack([data['x'].reshape(h, w), data['y'].reshape(h, w), data['z'].reshape(h, w)], axis=-1)
            bgr = np.stack([data['b'].reshape(h, w), data['g'].reshape(h, w), data['r'].reshape(h, w)], axis=-1)
            rgb = np.stack([data['r'].reshape(h, w), data['g'].reshape(h, w), data['b'].reshape(h, w)], axis=-1)
            return xyz, bgr, rgb, True
        else:
            # Fallback đọc PLY dạng điểm tự do bằng trimesh hoặc numpy
            import trimesh
            mesh_or_pc = trimesh.load(ply_path, process=False)
            vertices = np.array(mesh_or_pc.vertices, dtype=np.float32)
            if hasattr(mesh_or_pc, 'visual') and hasattr(mesh_or_pc.visual, 'vertex_colors') and mesh_or_pc.visual.vertex_colors is not None:
                colors = np.array(mesh_or_pc.visual.vertex_colors[:, :3], dtype=np.uint8)
                rgb = colors
                bgr = colors[:, ::-1]
            else:
                rgb = np.full((len(vertices), 3), 180, dtype=np.uint8)
                bgr = rgb.copy()
            
            return vertices, bgr, rgb, False

    def detect_table_plane(self, xyz_img):
        """Fit mặt bàn bằng RANSAC nếu là ảnh Zivid."""
        x = xyz_img[:, :, 0]
        y = xyz_img[:, :, 1]
        z = xyz_img[:, :, 2]

        valid = (z > 1000.0) & (z < 2200.0)
        pts_xy = np.stack([x[valid], y[valid]], axis=-1)
        pts_z = z[valid]

        if len(pts_xy) < 5000:
            raise ValueError("Không đủ điểm 3D để xác định mặt phẳng bàn.")

        n_samples = min(40000, len(pts_xy))
        idx = np.random.choice(len(pts_xy), size=n_samples, replace=False)
        ransac = RANSACRegressor(LinearRegression(), min_samples=3, residual_threshold=2.5, max_trials=100)
        ransac.fit(pts_xy[idx], pts_z[idx])

        a, b_coef = ransac.estimator_.coef_
        c = ransac.estimator_.intercept_
        norm = np.sqrt(a * a + b_coef * b_coef + 1.0)
        normal = np.array([-a / norm, -b_coef / norm, 1.0 / norm], dtype=np.float32)

        return {'a': float(a), 'b': float(b_coef), 'c': float(c), 'normal': normal}

    def segment_pants_region(self, xyz_img, bgr_img, plane_params, height_thresh=2.5):
        """Tự động phân đoạn khối quần nhô cao hơn mặt bàn."""
        h, w = xyz_img.shape[:2]
        x, y, z = xyz_img[:, :, 0], xyz_img[:, :, 1], xyz_img[:, :, 2]
        z_table = plane_params['a'] * x + plane_params['b'] * y + plane_params['c']
        height_map = z_table - z

        valid_z = (z > 1000.0) & (z < 2200.0)
        objects = (height_map >= height_thresh) & (height_map <= 120.0) & valid_z

        k_morph = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (9, 9))
        objects_clean = cv2.morphologyEx(objects.astype(np.uint8) * 255, cv2.MORPH_CLOSE, k_morph)
        objects_clean = cv2.morphologyEx(objects_clean, cv2.MORPH_OPEN, k_morph)

        num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(objects_clean)
        candidates = []
        for lbl in range(1, num_labels):
            area = stats[lbl, cv2.CC_STAT_AREA]
            left, top = stats[lbl, cv2.CC_STAT_LEFT], stats[lbl, cv2.CC_STAT_TOP]
            bw, bh = stats[lbl, cv2.CC_STAT_WIDTH], stats[lbl, cv2.CC_STAT_HEIGHT]
            touches_border = (left <= 5) or (top <= 5) or (left + bw >= w - 5) or (top + bh >= h - 5)
            if not touches_border and area >= 50000:
                candidates.append((lbl, area))

        if candidates:
            best_lbl = max(candidates, key=lambda it: it[1])[0]
            mask = (labels == best_lbl).astype(np.uint8) * 255
        else:
            if num_labels > 1:
                largest = 1 + np.argmax(stats[1:, cv2.CC_STAT_AREA])
                mask = (labels == largest).astype(np.uint8) * 255
            else:
                mask = (objects_clean > 0).astype(np.uint8) * 255

        return mask, height_map

    def find_center_high_peak(self, xyz_img, mask, height_map, center_window_size=350):
        """
        Tìm phần nhô cao nhất ở vùng center của ảnh (vị trí quần ở center ảnh).
        """
        h, w = mask.shape
        cy, cx = h // 2, w // 2
        half_w = center_window_size // 2

        y0, y1 = max(0, cy - half_w), min(h, cy + half_w)
        x0, x1 = max(0, cx - half_w), min(w, cx + half_w)

        # Lọc các điểm thuộc quần trong vùng center
        center_mask = (mask[y0:y1, x0:x1] > 0)
        center_h = height_map[y0:y1, x0:x1].copy()
        center_z = xyz_img[y0:y1, x0:x1, 2]
        valid_pts = center_mask & (center_z > 1000.0) & (center_z < 2200.0) & (~np.isnan(center_h))

        if not np.any(valid_pts):
            # Nếu trung tâm chính xác không có quần, mở rộng ra toàn bộ mask quần
            valid_pts = (mask > 0) & (xyz_img[:, :, 2] > 1000.0) & (~np.isnan(height_map))
            y0, y1, x0, x1 = 0, h, 0, w
            center_h = height_map

        masked_h = np.where(valid_pts, center_h, -9999.0)
        
        # Để tránh nhiễu điểm outlier đơn lẻ, làm mượt cục bộ vùng độ cao bằng Gaussian Filter
        blurred_h = cv2.GaussianBlur(np.maximum(masked_h, 0.0), (15, 15), 3.0)
        blurred_h[~valid_pts] = -9999.0

        max_idx = np.unravel_index(np.argmax(blurred_h), blurred_h.shape)
        peak_y = y0 + max_idx[0]
        peak_x = x0 + max_idx[1]
        peak_point_3d = xyz_img[peak_y, peak_x].astype(np.float64)
        peak_height_mm = float(height_map[peak_y, peak_x])

        return (peak_y, peak_x), peak_point_3d, peak_height_mm

    def compute_local_micro_normal(self, query_pt, kdtree, pts_3d, radius_mm=6.0):
        """
        Tính mặt phẳng vi mô cục bộ (Local micro-plane) tại điểm vẽ bằng PCA/SVD
        và trả về vector pháp tuyến vi mô (Micro-surface Normal Vector).
        """
        neighbor_indices = kdtree.query_ball_point(query_pt, r=radius_mm)
        if len(neighbor_indices) < 5:
            # Mở rộng bán kính nếu điểm quá thưa
            neighbor_indices = kdtree.query_ball_point(query_pt, r=radius_mm * 2.0)
        
        if len(neighbor_indices) < 3:
            # Fallback: Pháp tuyến thẳng đứng nhìn lên
            return np.array([0.0, 0.0, -1.0], dtype=np.float64)

        neighbors = pts_3d[neighbor_indices]
        centroid = np.mean(neighbors, axis=0)
        centered = neighbors - centroid

        # SVD để tìm ma trận hiệp phương sai và vector riêng nhỏ nhất
        _, _, vh = np.linalg.svd(centered)
        normal = vh[2, :]  # Vector pháp tuyến vi mô

        # Chuẩn hóa độ dài
        norm = np.linalg.norm(normal)
        if norm > 1e-6:
            normal = normal / norm
        else:
            normal = np.array([0.0, 0.0, -1.0], dtype=np.float64)

        # Định hướng pháp tuyến: luôn hướng về phía camera / robot (hướng lên trên)
        # Trong hệ tọa độ Zivid: trục Z hướng từ camera xuống bàn, mặt bàn Z ~ 1660mm
        # Vật thể nằm trên bàn thì bề mặt hướng ngược về camera, tức n_z < 0
        if normal[2] > 0:
            normal = -normal

        return normal

    def generate_surface_line_waypoints(self, start_peak_3d, pts_3d, angle_deg=0.0):
        """
        Tạo quỹ đạo đường thẳng dài 100mm nằm CHÍNH XÁC trên các điểm THẬT của quần:
        - Mỗi waypoint là một ĐIỂM THỰC TẾ 100% được lấy trực tiếp từ đám mây điểm của quần.
        - Không dùng tọa độ giả lập hay điểm nội suy lơ lửng ngoài bề mặt.
        - Tại mỗi điểm thật, tính mặt phẳng vi mô cục bộ và vector pháp tuyến vuông góc.
        """
        angle_rad = math.radians(angle_deg)
        dir_xy = np.array([math.cos(angle_rad), math.sin(angle_rad)], dtype=np.float64)

        half_len = self.line_length_mm / 2.0
        num_steps = int(round(self.line_length_mm / self.step_mm)) + 1
        s_values = np.linspace(-half_len, half_len, num_steps)

        # KDTree 2D trên mặt phẳng XY của các điểm thật trên quần
        kdtree_2d = cKDTree(pts_3d[:, :2])
        # KDTree 3D để tính mặt phẳng vi mô
        kdtree_3d = cKDTree(pts_3d)

        waypoints = []
        selected_real_indices = set()

        for idx, s in enumerate(s_values):
            # Tọa độ mục tiêu cách tâm đỉnh cao s mm theo hướng vẽ
            target_xy = start_peak_3d[:2] + s * dir_xy

            # Tìm điểm THẬT gần nhất trên bề mặt quần
            dist, nearest_idx = kdtree_2d.query(target_xy)
            selected_real_indices.add(int(nearest_idx))
            
            # ĐIỂM THẬT 100% TRÊN QUẦN (Không sửa đổi tọa độ)
            real_surface_pt = pts_3d[nearest_idx].astype(np.float64)

            # Tính mặt phẳng vi mô cục bộ từ các điểm lân cận vi mô quanh điểm thật này
            micro_normal = self.compute_local_micro_normal(real_surface_pt, kdtree_3d, pts_3d, self.micro_radius_mm)

            # --- TÍNH TƯ THẾ BÚT ẢO (VIRTUAL PEN POSE) ---
            # Hướng của bút ảo luôn luôn vuông góc với mặt phẳng vi mô:
            # Ngòi bút đặt đúng tại điểm thật trên quần: real_surface_pt
            # Thân bút ảo vươn ngược lên không gian theo vector pháp tuyến vi mô:
            pen_direction = -micro_normal
            pen_top_3d = real_surface_pt - micro_normal * self.pen_length_mm

            # Xây dựng hệ tọa độ cục bộ của bút (Tool Frame: X_tool, Y_tool, Z_tool)
            z_tool = pen_direction / np.linalg.norm(pen_direction)
            
            t_vec = np.array([dir_xy[0], dir_xy[1], 0.0])
            x_proj = t_vec - np.dot(t_vec, z_tool) * z_tool
            norm_x = np.linalg.norm(x_proj)
            if norm_x > 1e-4:
                x_tool = x_proj / norm_x
            else:
                x_tool = np.array([1.0, 0.0, 0.0])
            
            y_tool = np.cross(z_tool, x_tool)
            y_tool = y_tool / np.linalg.norm(y_tool)

            R = np.column_stack([x_tool, y_tool, z_tool])

            qw = math.sqrt(max(0.0, 1.0 + R[0, 0] + R[1, 1] + R[2, 2])) / 2.0
            qx = math.sqrt(max(0.0, 1.0 + R[0, 0] - R[1, 1] - R[2, 2])) / 2.0
            qy = math.sqrt(max(0.0, 1.0 - R[0, 0] + R[1, 1] - R[2, 2])) / 2.0
            qz = math.sqrt(max(0.0, 1.0 - R[0, 0] - R[1, 1] + R[2, 2])) / 2.0
            qx = math.copysign(qx, R[2, 1] - R[1, 2])
            qy = math.copysign(qy, R[0, 2] - R[2, 0])
            qz = math.copysign(qz, R[1, 0] - R[0, 1])

            tilt_deg = math.degrees(math.acos(np.clip(abs(micro_normal[2]), 0.0, 1.0)))

            waypoints.append({
                'index': idx,
                'point_index_in_cloud': int(nearest_idx),
                'distance_mm': float(s + half_len),
                'x': float(real_surface_pt[0]),
                'y': float(real_surface_pt[1]),
                'z': float(real_surface_pt[2]),
                'normal_x': float(micro_normal[0]),
                'normal_y': float(micro_normal[1]),
                'normal_z': float(micro_normal[2]),
                'pen_top_x': float(pen_top_3d[0]),
                'pen_top_y': float(pen_top_3d[1]),
                'pen_top_z': float(pen_top_3d[2]),
                'qx': float(qx),
                'qy': float(qy),
                'qz': float(qz),
                'qw': float(qw),
                'tilt_deg': float(tilt_deg)
            })

        return waypoints, selected_real_indices

    def export_visualization_ply(self, output_ply_path, pants_pts, pants_colors, waypoints, start_peak_3d, angle_deg=0.0, line_width_mm=1.8):
        """
        Xuất file 3D PLY:
        - ĐƯỜNG THẲNG MÀU ĐỎ NẰM ĐÚNG TRÊN CÁC ĐIỂM THẬT CỦA BỀ MẶT QUẦN:
          Các điểm thật của quần nằm trong dải đường vẽ 100mm được tô màu ĐỎ RỰC RỠ [255, 0, 0].
          KHÔNG dùng điểm nhân tạo hay điểm lơ lửng.
        - CÁC THÂN BÚT ẢO MÀU VÀNG:
          Cắm vuông góc từ chính các điểm thật trên bề mặt quần theo vector pháp tuyến vi mô.
        """
        # 1. Xác định các điểm THẬT của quần nằm trên dải đường vẽ 100mm
        angle_rad = math.radians(angle_deg)
        u_dir = np.array([math.cos(angle_rad), math.sin(angle_rad)], dtype=np.float32)

        # Tính khoảng cách dọc (s) và khoảng cách vuông góc (d_perp) của từng điểm thật
        delta_xy = pants_pts[:, :2] - start_peak_3d[:2].astype(np.float32)
        s_proj = delta_xy[:, 0] * u_dir[0] + delta_xy[:, 1] * u_dir[1]
        d_perp = np.abs(-delta_xy[:, 0] * u_dir[1] + delta_xy[:, 1] * u_dir[0])

        half_len = self.line_length_mm / 2.0
        half_width = line_width_mm / 2.0

        is_on_line = (np.abs(s_proj) <= half_len) & (d_perp <= half_width)

        # 2. Tạo bản sao màu sắc của quần: Tô đỏ trực tiếp lên các điểm thật của quần
        colored_pants = pants_colors.copy()
        colored_pants[is_on_line] = [255, 0, 0]  # MÀU ĐỎ TRÊN CÁC ĐIỂM THẬT CỦA QUẦN

        # 3. Tạo các đoạn thân bút ảo (Virtual Pens) cắm vuông góc tại các waypoint thật
        pen_pts = []
        pen_cols = []
        draw_pen_step = max(1, len(waypoints) // 15)  # Hiển thị ~15 bút ảo dọc theo 100mm

        for i in range(0, len(waypoints), draw_pen_step):
            wp = waypoints[i]
            # Điểm gốc bút là ĐIỂM THẬT trên bề mặt quần
            p_tip = np.array([wp['x'], wp['y'], wp['z']], dtype=np.float32)
            p_top = np.array([wp['pen_top_x'], wp['pen_top_y'], wp['pen_top_z']], dtype=np.float32)

            # Vẽ đoạn thân bút màu vàng cắm vuông góc từ điểm thật vươn lên
            t_vals = np.linspace(0.0, 1.0, 35)
            for t in t_vals:
                pos = (1.0 - t) * p_tip + t * p_top
                pen_pts.append(pos)
                pen_cols.append([255, 255, 0])  # Màu vàng
                # Làm nét thân bút rõ nét
                pen_pts.append(pos + np.array([0.3, 0.0, 0.0], dtype=np.float32))
                pen_cols.append([0, 255, 255])
                pen_pts.append(pos + np.array([0.0, 0.3, 0.0], dtype=np.float32))
                pen_cols.append([0, 255, 255])

        # Hợp nhất: Toàn bộ điểm thật của quần (đã nhuộm đỏ đường vẽ) + Thân bút ảo
        if pen_pts:
            merged_pts = np.vstack([pants_pts, np.array(pen_pts, dtype=np.float32)])
            merged_cols = np.vstack([colored_pants, np.array(pen_cols, dtype=np.uint8)])
        else:
            merged_pts = pants_pts
            merged_cols = colored_pants

        # Ghi file PLY chuẩn
        num_v = len(merged_pts)
        with open(output_ply_path, 'wb') as f:
            header = (
                f"ply\n"
                f"format binary_little_endian 1.0\n"
                f"element vertex {num_v}\n"
                f"property float x\n"
                f"property float y\n"
                f"property float z\n"
                f"property uchar red\n"
                f"property uchar green\n"
                f"property uchar blue\n"
                f"end_header\n"
            )
            f.write(header.encode('latin1'))

            dt = np.dtype([('x', '<f4'), ('y', '<f4'), ('z', '<f4'), ('r', 'u1'), ('g', 'u1'), ('b', 'u1')])
            out_data = np.empty(num_v, dtype=dt)
            out_data['x'] = merged_pts[:, 0]
            out_data['y'] = merged_pts[:, 1]
            out_data['z'] = merged_pts[:, 2]
            out_data['r'] = merged_cols[:, 0]
            out_data['g'] = merged_cols[:, 1]
            out_data['b'] = merged_cols[:, 2]
            out_data.tofile(f)

        print(f"[5/6] Đã xuất file 3D PLY: {output_ply_path}")
        print(f"      -> Tổng số điểm thật trên quần được nhuộm đỏ cho đường vẽ 100mm: {np.sum(is_on_line):,} điểm")

    def export_trajectory_csv(self, output_csv_path, waypoints):
        """Xuất danh sách waypoint quỹ đạo của bút ảo ra CSV."""
        import csv
        with open(output_csv_path, 'w', newline='', encoding='utf-8') as f:
            fieldnames = [
                'index', 'distance_mm', 'x_mm', 'y_mm', 'z_mm',
                'normal_x', 'normal_y', 'normal_z',
                'pen_top_x', 'pen_top_y', 'pen_top_z',
                'qx', 'qy', 'qz', 'qw', 'tilt_deg'
            ]
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            for wp in waypoints:
                writer.writerow({
                    'index': wp['index'],
                    'distance_mm': round(wp['distance_mm'], 2),
                    'x_mm': round(wp['x'], 3),
                    'y_mm': round(wp['y'], 3),
                    'z_mm': round(wp['z'], 3),
                    'normal_x': round(wp['normal_x'], 5),
                    'normal_y': round(wp['normal_y'], 5),
                    'normal_z': round(wp['normal_z'], 5),
                    'pen_top_x': round(wp['pen_top_x'], 3),
                    'pen_top_y': round(wp['pen_top_y'], 3),
                    'pen_top_z': round(wp['pen_top_z'], 3),
                    'qx': round(wp['qx'], 5),
                    'qy': round(wp['qy'], 5),
                    'qz': round(wp['qz'], 5),
                    'qw': round(wp['qw'], 5),
                    'tilt_deg': round(wp['tilt_deg'], 2)
                })
        print(f"[6/6] Đã xuất file quỹ đạo Robot CSV: {output_csv_path}")

    def generate_2d_overlay_image(self, bgr_img, mask, peak_px, waypoints, output_img_path):
        """Tạo ảnh 2D Overlay hiển thị đường vẽ 100mm và các vector pháp tuyến bút ảo."""
        overlay = bgr_img.copy()

        # Làm mờ nhẹ vùng nền để vùng quần nổi bật
        bg_mask = (mask == 0)
        overlay[bg_mask] = (overlay[bg_mask].astype(np.float32) * 0.4).astype(np.uint8)

        # Vẽ tâm đỉnh nhô cao
        py, px = peak_px
        cv2.circle(overlay, (px, py), 9, (0, 0, 255), -1)
        cv2.circle(overlay, (px, py), 13, (255, 255, 255), 2)
        cv2.putText(overlay, "Center High Peak", (px + 15, py - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

        # Tính tọa độ 2D của các waypoint trên ảnh
        # Trong hệ tọa độ Zivid: ta có thể chiếu ngược hoặc nội suy từ lưới ảnh
        # Để chính xác, ta tìm pixel gần nhất cho từng waypoint
        wp_pixels = []
        for wp in waypoints:
            # Ước lượng vị trí pixel tương đối từ peak_px và khoảng cách (dùng x_mm, y_mm)
            # Với Zivid: 1 pixel ~ 0.387mm ở khoảng cách 1600mm
            dx_mm = wp['x'] - waypoints[len(waypoints)//2]['x']
            dy_mm = wp['y'] - waypoints[len(waypoints)//2]['y']
            u = int(round(px + dx_mm / 0.387))
            v = int(round(py + dy_mm / 0.387))
            wp_pixels.append((u, v))

        # Vẽ đường thẳng 100mm trên ảnh (Màu đỏ dày)
        for i in range(len(wp_pixels) - 1):
            cv2.line(overlay, wp_pixels[i], wp_pixels[i + 1], (0, 0, 255), 4)

        # Vẽ các mũi tên chỉ vector bút ảo vuông góc tại các điểm
        draw_step = max(1, len(waypoints) // 12)
        for i in range(0, len(waypoints), draw_step):
            u, v = wp_pixels[i]
            norm_x = waypoints[i]['normal_x']
            norm_y = waypoints[i]['normal_y']
            
            # Chiếu vector pháp tuyến vi mô lên 2D
            arrow_len = 35.0
            u2 = int(round(u + norm_x * arrow_len))
            v2 = int(round(v + norm_y * arrow_len))

            cv2.arrowedLine(overlay, (u, v), (u2, v2), (0, 255, 255), 2, tipLength=0.3)
            cv2.circle(overlay, (u, v), 4, (255, 0, 255), -1)

        # Vẽ thanh thông tin chi tiết
        info_panel = np.zeros((120, overlay.shape[1], 3), dtype=np.uint8)
        info_panel[:] = (35, 35, 35)
        
        cv2.putText(info_panel, f"QUY DAO VE BUT AO 3D (CHIEU DAI = {self.line_length_mm:.1f} mm)",
                    (25, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 255), 2)
        cv2.putText(info_panel, f"Tam ve (Peak): X={waypoints[len(waypoints)//2]['x']:.1f}mm, Y={waypoints[len(waypoints)//2]['y']:.1f}mm, Z={waypoints[len(waypoints)//2]['z']:.1f}mm",
                    (25, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 1)
        cv2.putText(info_panel, f"Huong but: VUONG GOC VOI MAT PHANG VI MO TAI MOI DIEM (Micro Normal Alignment)",
                    (25, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 0), 1)

        result_img = np.vstack([overlay, info_panel])
        cv2.imwrite(output_img_path, result_img)
        print(f"[4/6] Đã lưu ảnh minh họa 2D: {output_img_path}")

    def build_3d_line_tube_mesh(self, waypoints, radius=0.85):
        """
        DỰNG ĐƯỜNG NỐI 3D LIỀN MẠCH (Solid 3D Tube Mesh):
        Nối liên tục từng cặp waypoint liền kề P_0 -> P_1 -> ... -> P_100
        thành một đường ống 3D đặc màu ĐỎ bám sát 100% trên bề mặt quần.
        """
        elements = []
        for i in range(len(waypoints) - 1):
            p1 = np.array([waypoints[i]['x'], waypoints[i]['y'], waypoints[i]['z']], dtype=np.float64)
            p2 = np.array([waypoints[i + 1]['x'], waypoints[i + 1]['y'], waypoints[i + 1]['z']], dtype=np.float64)
            
            # Khối trụ nối 2 điểm liên tiếp
            cyl = trimesh.creation.cylinder(radius=radius, segment=[p1, p2], sections=12)
            elements.append(cyl)
            
            # Khớp cầu tại waypoint giúp đường vẽ mượt mà không bị gấp khúc
            sphere = trimesh.creation.icosphere(radius=radius * 1.05, subdivisions=1)
            sphere.apply_translation(p1)
            elements.append(sphere)

        # Cầu tại điểm cuối
        p_last = np.array([waypoints[-1]['x'], waypoints[-1]['y'], waypoints[-1]['z']], dtype=np.float64)
        sphere_last = trimesh.creation.icosphere(radius=radius * 1.05, subdivisions=1)
        sphere_last.apply_translation(p_last)
        elements.append(sphere_last)

        line_mesh = trimesh.util.concatenate(elements)
        line_mesh.visual.vertex_colors = [255, 0, 0, 255]  # MÀU ĐỎ RỰC RỠ
        return line_mesh

    def build_3d_virtual_pens_mesh(self, waypoints, radius=0.45, pen_step=7):
        """
        DỰNG CÁC BÚT ẢO 3D (Virtual Pens Mesh):
        Tạo các thân bút 3D dạng ống trụ màu VÀNG cắm vuông góc với mặt phẳng vi mô
        từ chính các điểm thật trên bề mặt quần vươn lên không gian.
        """
        elements = []
        for i in range(0, len(waypoints), pen_step):
            wp = waypoints[i]
            p_tip = np.array([wp['x'], wp['y'], wp['z']], dtype=np.float64)
            p_top = np.array([wp['pen_top_x'], wp['pen_top_y'], wp['pen_top_z']], dtype=np.float64)

            # Thân bút màu vàng
            cyl = trimesh.creation.cylinder(radius=radius, segment=[p_tip, p_top], sections=10)
            cyl.visual.vertex_colors = [255, 255, 0, 255]
            elements.append(cyl)

            # Ngòi bút màu xanh cyan tiếp xúc với bề mặt
            tip_sphere = trimesh.creation.icosphere(radius=radius * 1.3, subdivisions=1)
            tip_sphere.apply_translation(p_tip)
            tip_sphere.visual.vertex_colors = [0, 255, 255, 255]
            elements.append(tip_sphere)

            # Đỉnh cán bút màu cam
            top_sphere = trimesh.creation.icosphere(radius=radius * 1.4, subdivisions=1)
            top_sphere.apply_translation(p_top)
            top_sphere.visual.vertex_colors = [255, 140, 0, 255]
            elements.append(top_sphere)

        pen_mesh = trimesh.util.concatenate(elements)
        return pen_mesh

    def reconstruct_pants_surface_mesh(self, xyz_img, rgb_img, mask, step=3, max_edge_mm=5.5):
        """
        DỰNG LẠI FILE PLY SANG MÔ HÌNH 3D MESH (Surface Reconstruction):
        Tái tạo mặt lưới tam giác 3D từ lưới độ sâu của ảnh Zivid, giữ nguyên màu sắc gốc.
        """
        h, w = mask.shape
        sub_y = slice(0, h, step)
        sub_x = slice(0, w, step)

        pts_sub = xyz_img[sub_y, sub_x]
        col_sub = rgb_img[sub_y, sub_x]
        mask_sub = (mask[sub_y, sub_x] > 0) & (pts_sub[:, :, 2] > 1000.0) & (pts_sub[:, :, 2] < 2200.0)

        h_s, w_s = mask_sub.shape
        idx_grid = np.full((h_s, w_s), -1, dtype=np.int32)
        valid_indices = np.where(mask_sub)
        idx_grid[valid_indices] = np.arange(len(valid_indices[0]))

        vertices = pts_sub[mask_sub]
        colors = np.pad(col_sub[mask_sub], ((0, 0), (0, 1)), constant_values=255)

        v00 = idx_grid[:-1, :-1]
        v01 = idx_grid[:-1, 1:]
        v10 = idx_grid[1:, :-1]
        v11 = idx_grid[1:, 1:]

        valid_t1 = (v00 >= 0) & (v10 >= 0) & (v01 >= 0)
        t1 = np.stack([v00[valid_t1], v10[valid_t1], v01[valid_t1]], axis=-1)

        valid_t2 = (v01 >= 0) & (v10 >= 0) & (v11 >= 0)
        t2 = np.stack([v01[valid_t2], v10[valid_t2], v11[valid_t2]], axis=-1)

        all_triangles = np.vstack([t1, t2])

        pA = vertices[all_triangles[:, 0]]
        pB = vertices[all_triangles[:, 1]]
        pC = vertices[all_triangles[:, 2]]
        dAB = np.linalg.norm(pA - pB, axis=-1)
        dBC = np.linalg.norm(pB - pC, axis=-1)
        dCA = np.linalg.norm(pC - pA, axis=-1)
        valid_edges = (dAB <= max_edge_mm) & (dBC <= max_edge_mm) & (dCA <= max_edge_mm)
        clean_faces = all_triangles[valid_edges]

        surface_mesh = trimesh.Trimesh(vertices=vertices, faces=clean_faces, vertex_colors=colors, process=False)
        return surface_mesh

    def render_3d_perspective_snapshot(self, pts_pants, rgb_pants, waypoints, out_img_path, img_w=1600, img_h=1000):
        """
        RENDER ẢNH PHỐI CẢNH 3D TRỰC QUAN (3D Perspective Snapshot):
        Chụp mô hình từ góc nghiêng nhìn xuống ~35 độ, hiển thị rõ nét:
        - Khối bề mặt quần 3D.
        - Đường nối 3D màu ĐỎ nối liền mạch toàn bộ các điểm thật từ đầu đến cuối 100mm.
        - Các thân bút ảo màu VÀNG cắm vuông góc với mặt phẳng vi mô vươn lên không gian.
        """
        center_pt = np.array([waypoints[len(waypoints)//2]['x'], waypoints[len(waypoints)//2]['y'], waypoints[len(waypoints)//2]['z']])

        elev = math.radians(38.0)
        azim = math.radians(-32.0)
        cam_dist = 480.0  # mm

        cam_pos = center_pt + np.array([
            cam_dist * math.cos(elev) * math.sin(azim),
            cam_dist * math.cos(elev) * math.cos(azim),
            -cam_dist * math.sin(elev)
        ])

        forward = center_pt - cam_pos
        forward /= np.linalg.norm(forward)
        up_ref = np.array([0.0, 0.0, -1.0])
        right = np.cross(forward, up_ref)
        right /= np.linalg.norm(right)
        cam_up = np.cross(right, forward)

        R = np.vstack([right, cam_up, forward])

        sub_pts = pts_pants[::2]
        sub_cols = rgb_pants[::2]
        pts_cam = (sub_pts - cam_pos) @ R.T

        fx = fy = 1450.0
        cx = img_w / 2.0
        cy = img_h / 2.0 + 30.0

        valid_cam = pts_cam[:, 2] > 10.0
        u = (fx * pts_cam[valid_cam, 0] / pts_cam[valid_cam, 2] + cx).astype(np.int32)
        v = (fy * pts_cam[valid_cam, 1] / pts_cam[valid_cam, 2] + cy).astype(np.int32)
        z_depth = pts_cam[valid_cam, 2]
        c_rgb = sub_cols[valid_cam]

        canvas = np.full((img_h, img_w, 3), 32, dtype=np.uint8)

        sort_idx = np.argsort(-z_depth)
        u_s, v_s, z_s, c_s = u[sort_idx], v[sort_idx], z_depth[sort_idx], c_rgb[sort_idx]

        in_screen = (u_s >= 1) & (u_s < img_w - 1) & (v_s >= 1) & (v_s < img_h - 1)
        u_valid, v_valid, c_valid = u_s[in_screen], v_s[in_screen], c_s[in_screen]

        bgr_colors = c_valid[:, ::-1]
        for du in [-1, 0]:
            for dv in [-1, 0]:
                canvas[v_valid + dv, u_valid + du] = bgr_colors

        # Vẽ ĐƯỜNG NỐI 3D MÀU ĐỎ NỐI CÁC ĐIỂM
        wp_3d = np.array([[wp['x'], wp['y'], wp['z']] for wp in waypoints], dtype=np.float64)
        wp_cam = (wp_3d - cam_pos) @ R.T
        wp_u = (fx * wp_cam[:, 0] / wp_cam[:, 2] + cx).astype(np.int32)
        wp_v = (fy * wp_cam[:, 1] / wp_cam[:, 2] + cy).astype(np.int32)

        pts_poly = np.stack([wp_u, wp_v], axis=-1)
        cv2.polylines(canvas, [pts_poly], isClosed=False, color=(0, 0, 255), thickness=6, lineType=cv2.LINE_AA)
        cv2.polylines(canvas, [pts_poly], isClosed=False, color=(20, 100, 255), thickness=2, lineType=cv2.LINE_AA)

        for pt in pts_poly:
            cv2.circle(canvas, tuple(pt), 4, (255, 255, 255), -1)
            cv2.circle(canvas, tuple(pt), 2, (0, 0, 255), -1)

        # Vẽ CÁC BÚT ẢO MÀU VÀNG CẮM VUÔNG GÓC
        pen_step = max(1, len(waypoints) // 14)
        for i in range(0, len(waypoints), pen_step):
            wp = waypoints[i]
            p_tip = np.array([wp['x'], wp['y'], wp['z']], dtype=np.float64)
            p_top = np.array([wp['pen_top_x'], wp['pen_top_y'], wp['pen_top_z']], dtype=np.float64)

            tip_cam = (p_tip - cam_pos) @ R.T
            top_cam = (p_top - cam_pos) @ R.T

            u_tip = int(round(fx * tip_cam[0] / tip_cam[2] + cx))
            v_tip = int(round(fy * tip_cam[1] / tip_cam[2] + cy))
            u_top = int(round(fx * top_cam[0] / top_cam[2] + cx))
            v_top = int(round(fy * top_cam[1] / top_cam[2] + cy))

            cv2.line(canvas, (u_tip, v_tip), (u_top, v_top), (0, 255, 255), 3, lineType=cv2.LINE_AA)
            cv2.circle(canvas, (u_tip, v_tip), 5, (255, 255, 0), -1)
            cv2.circle(canvas, (u_top, v_top), 6, (0, 165, 255), -1)

        header_bar = np.zeros((90, img_w, 3), dtype=np.uint8)
        header_bar[:] = (20, 20, 20)
        cv2.putText(header_bar, f"3D MODEL: DUONG NOI 100mm TREN QUAN & BUT AO VUONG GOC VI MO",
                    (25, 38), cv2.FONT_HERSHEY_SIMPLEX, 0.85, (0, 255, 255), 2)
        cv2.putText(header_bar, f"Goc nhin 3D Isometric (Elevation: 38 deg) | Duong noi 3D mau DO (101 points) | But ao mau VANG vuong goc",
                    (25, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1)

        result_view = np.vstack([header_bar, canvas])
        cv2.imwrite(out_img_path, result_view)
        print(f"[4/6] Đã lưu ảnh phối cảnh 3D trực quan: {out_img_path}")

    def process_file(self, ply_path, output_dir="output_drawing", angle_deg=0.0):
        """Quy trình hoàn chỉnh xử lý 1 file PLY."""
        os.makedirs(output_dir, exist_ok=True)
        base_name = os.path.splitext(os.path.basename(ply_path))[0]

        xyz, bgr, rgb, is_organized = self.load_ply(ply_path)

        if is_organized:
            print("[2/6] Đang phân tích mặt bàn và phân đoạn khối quần...")
            plane_params = self.detect_table_plane(xyz)
            mask, height_map = self.segment_pants_region(xyz, bgr, plane_params)

            print("[3/6] Đang tìm vị trí nhô cao ở trung tâm ảnh...")
            peak_px, peak_3d, peak_h = self.find_center_high_peak(xyz, mask, height_map)
            print(f"      -> Vị trí nhô cao tại pixel: {peak_px}, Độ cao so với bàn: {peak_h:.2f} mm")
            print(f"      -> Tọa độ 3D: X={peak_3d[0]:.2f}mm, Y={peak_3d[1]:.2f}mm, Z={peak_3d[2]:.2f}mm")

            # Chuẩn bị tập hợp điểm 3D của quần để xây dựng KDTree vi mô
            pants_mask_bool = (mask > 0) & (xyz[:, :, 2] > 1000.0)
            pts_pants = xyz[pants_mask_bool]
            col_pants = rgb[pants_mask_bool]
        else:
            # File dạng điểm tự do
            pts_pants = xyz
            col_pants = rgb
            center_3d = np.mean(pts_pants, axis=0)
            # Tìm điểm cao nhất gần tâm XY
            dists_xy = np.linalg.norm(pts_pants[:, :2] - center_3d[:2], axis=1)
            near_center = dists_xy < np.percentile(dists_xy, 20)
            peak_idx = np.argmin(pts_pants[near_center, 2])  # Điểm gần camera nhất
            peak_3d = pts_pants[near_center][peak_idx]
            peak_px = (600, 972)
            bgr = np.full((1200, 1944, 3), 150, dtype=np.uint8)
            mask = np.ones((1200, 1944), dtype=np.uint8) * 255

        print(f"[*] Tính toán quỹ đạo vẽ {self.line_length_mm:.1f}mm nằm CHÍNH XÁC trên các điểm thật của quần...")
        waypoints, selected_indices = self.generate_surface_line_waypoints(peak_3d, pts_pants, angle_deg=angle_deg)

        # 1. Render ảnh phối cảnh 3D trực quan hóa đường nối các điểm
        out_3d_img = os.path.join(output_dir, f"{base_name}_3d_rendered_view.png")
        self.render_3d_perspective_snapshot(pts_pants, col_pants, waypoints, out_3d_img)

        # 2. Xuất ảnh 2D minh họa
        out_2d_img = os.path.join(output_dir, f"{base_name}_drawn_line_2d.png")
        if is_organized:
            self.generate_2d_overlay_image(bgr, mask, peak_px, waypoints, out_2d_img)

        # 3. DỰNG MÔ HÌNH 3D: Nối các điểm thành đường ống 3D (Tube Mesh) và Bút ảo 3D
        print("[*] Đang dựng mô hình 3D (đường ống 3D nối các điểm + bút ảo + bề mặt quần)...")
        line_tube_mesh = self.build_3d_line_tube_mesh(waypoints, radius=0.85)
        pen_mesh = self.build_3d_virtual_pens_mesh(waypoints, radius=0.45, pen_step=7)

        if is_organized:
            surface_mesh = self.reconstruct_pants_surface_mesh(xyz, rgb, mask, step=3)
            combined_3d = trimesh.util.concatenate([surface_mesh, line_tube_mesh, pen_mesh])
        else:
            combined_3d = trimesh.util.concatenate([line_tube_mesh, pen_mesh])

        # 4. Xuất file 3D OBJ và PLY hoàn chỉnh
        out_obj_path = os.path.join(output_dir, f"{base_name}_model_3d_with_line.obj")
        out_3d_ply = os.path.join(output_dir, f"{base_name}_model_3d_with_line.ply")
        combined_3d.export(out_obj_path)
        combined_3d.export(out_3d_ply)
        print(f"[5/6] Đã xuất file 3D Mesh OBJ: {out_obj_path}")
        print(f"      Đã xuất file 3D Mesh PLY: {out_3d_ply}")

        # 5. Xuất file PLY điểm thật nhuộm đỏ
        out_ply_path = os.path.join(output_dir, f"{base_name}_virtual_pen_3d.ply")
        self.export_visualization_ply(out_ply_path, pts_pants, col_pants, waypoints, peak_3d, angle_deg=angle_deg)

        # 6. Xuất file CSV quỹ đạo
        out_csv_path = os.path.join(output_dir, f"{base_name}_pen_trajectory.csv")
        self.export_trajectory_csv(out_csv_path, waypoints)

        print("\n" + "=" * 60)
        print("HOÀN THÀNH DỰNG 3D VÀ VẼ NỐI ĐƯỜNG THẲNG TRÊN QUẦN")
        print(f"  - Chiều dài đường vẽ: {self.line_length_mm:.1f} mm")
        print(f"  - Số điểm waypoint:   {len(waypoints)} điểm (ĐÃ NỐI THÀNH ĐƯỜNG 3D)")
        print(f"  - File 3D OBJ:        {out_obj_path}")
        print(f"  - File 3D PLY Mesh:   {out_3d_ply}")
        print(f"  - File Quỹ đạo CSV:   {out_csv_path}")
        print(f"  - Ảnh Render 3D:      {out_3d_img}")
        if is_organized:
            print(f"  - Ảnh 2D minh họa:    {out_2d_img}")
        print("=" * 60)

        return waypoints


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="Chương trình vẽ đường thẳng 100mm trên vị trí quần với bút ảo vuông góc vi mô")
    parser.add_argument('--input', type=str, default=None, help="Đường dẫn file .ply bất kỳ hoặc thư mục chứa ply")
    parser.add_argument('--output', type=str, default="output_drawing", help="Thư mục xuất kết quả")
    parser.add_argument('--length', type=float, default=100.0, help="Độ dài đường vẽ tính bằng mm (mặc định: 100.0mm)")
    parser.add_argument('--step', type=float, default=1.0, help="Khoảng cách giữa các waypoint mm (mặc định: 1.0mm)")
    parser.add_argument('--angle', type=float, default=0.0, help="Góc nghiêng đường vẽ theo mặt phẳng XY (độ, mặc định 0 = nằm ngang)")
    parser.add_argument('--radius', type=float, default=6.0, help="Bán kính xét mặt phẳng vi mô cục bộ mm (mặc định: 6.0mm)")
    args = parser.parse_args()

    input_path = args.input
    if input_path is None:
        # Tự động tìm 1 file ply bất kỳ trong dataset/_3D nếu người dùng không truyền
        sample_files = glob.glob(r'dataset\_3D\*.ply')
        if sample_files:
            input_path = sample_files[0]
            print(f"[!] Không truyền --input, tự động chọn file mẫu: {input_path}")
        else:
            print("[X] Không tìm thấy file PLY nào. Vui lòng truyền --input <đường dẫn file .ply>")
            sys.exit(1)

    drawer = VirtualPenDrawer3D(
        line_length_mm=args.length,
        step_mm=args.step,
        micro_radius_mm=args.radius
    )

    if os.path.isfile(input_path):
        drawer.process_file(input_path, output_dir=args.output, angle_deg=args.angle)
    elif os.path.isdir(input_path):
        ply_list = glob.glob(os.path.join(input_path, "*.ply"))
        print(f"Tìm thấy {len(ply_list)} file PLY trong thư mục: {input_path}")
        for p in ply_list:
            drawer.process_file(p, output_dir=args.output, angle_deg=args.angle)
    else:
        print(f"[X] Đường dẫn không tồn tại: {input_path}")
