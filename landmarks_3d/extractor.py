import os
import json
import numpy as np
import cv2
import sys

# Import core detector from root workspace
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from detect_pants_3d import PantsDetector3D


class PantsLandmarks3D:
    """
    Trích xuất Bộ mốc hình học 3D (3D Geometric Landmarks) & Lưới tam giác giải phẫu (Mesh Topology)
    cho quần áo từ dữ liệu camera Zivid 2 M70.
    
    Phục vụ:
    - Phương pháp P3 (Đề cương & Góp ý Mentor): Quỹ đạo mẫu theo mốc & Biến đổi Affine từng mảnh.
    - Đổi cỡ tự động (Size S <-> M <-> L) không cần dạy tay lại robot.
    - Xuất nhãn chuẩn YOLO11-pose và tọa độ không gian 3D cho Robot TCP.
    """

    LANDMARK_NAMES = [
        "waist_left",
        "waist_center",
        "waist_right",
        "crotch",
        "thigh_left",
        "thigh_right",
        "knee_left",
        "knee_right",
        "cuff_left_outer",
        "cuff_left_inner",
        "cuff_right_inner",
        "cuff_right_outer"
    ]

    # Danh sách các cạnh kết nối tạo khung lưới giải phẫu
    SKELETON_EDGES = [
        ("waist_left", "waist_center"),
        ("waist_center", "waist_right"),
        ("waist_left", "crotch"),
        ("waist_right", "crotch"),
        ("crotch", "thigh_left"),
        ("crotch", "thigh_right"),
        ("thigh_left", "knee_left"),
        ("thigh_right", "knee_right"),
        ("knee_left", "cuff_left_outer"),
        ("knee_left", "cuff_left_inner"),
        ("knee_right", "cuff_right_outer"),
        ("knee_right", "cuff_right_inner"),
        ("cuff_left_outer", "cuff_left_inner"),
        ("cuff_right_inner", "cuff_right_outer"),
        ("waist_left", "thigh_left"),
        ("waist_right", "thigh_right")
    ]

    def __init__(self, detector=None):
        self.detector = detector if detector is not None else PantsDetector3D()

    def get_valid_3d_point(self, xyz, u, v, window=9):
        """Lấy tọa độ 3D tại (u, v); nếu dính điểm khuyết (Z=0) thì tìm điểm lân cận hợp lệ gần nhất."""
        h, w = xyz.shape[:2]
        u_int, v_int = int(round(u)), int(round(v))
        u_min, u_max = max(0, u_int - window), min(w, u_int + window + 1)
        v_min, v_max = max(0, v_int - window), min(h, v_int + window + 1)

        sub = xyz[v_min:v_max, u_min:u_max]
        z_sub = sub[:, :, 2]
        valid = z_sub > 500.0

        if np.any(valid):
            yy, xx = np.indices(z_sub.shape)
            dists = (yy - (v_int - v_min)) ** 2 + (xx - (u_int - u_min)) ** 2
            dists[~valid] = 1e9
            min_idx = np.unravel_index(np.argmin(dists), dists.shape)
            return sub[min_idx]
        return xyz[v_int, u_int]

    def estimate_normal(self, xyz, u, v, window=5):
        """Ước tính vector pháp tuyến bề mặt 3D tại (u, v) hướng về phía camera."""
        h, w = xyz.shape[:2]
        u_int, v_int = int(round(u)), int(round(v))
        u_min, u_max = max(0, u_int - window), min(w, u_int + window + 1)
        v_min, v_max = max(0, v_int - window), min(h, v_int + window + 1)

        sub = xyz[v_min:v_max, u_min:u_max]
        pts = sub.reshape(-1, 3)
        valid = pts[:, 2] > 500.0
        pts_valid = pts[valid]

        if len(pts_valid) >= 5:
            # PCA trên đám mây điểm lân cận để tìm pháp tuyến
            centroid = np.mean(pts_valid, axis=0)
            pts_centered = pts_valid - centroid
            cov = np.dot(pts_centered.T, pts_centered)
            eigenvalues, eigenvectors = np.linalg.eigh(cov)
            normal = eigenvectors[:, 0]  # Vector ứng với trị riêng nhỏ nhất
            # Đảm bảo pháp tuyến hướng về phía camera (Z âm trong camera frame)
            if normal[2] > 0:
                normal = -normal
            return (normal / np.linalg.norm(normal)).tolist()
        return [0.0, 0.0, -1.0]

    def find_cross_section_center(self, mask, xyz, seed_pt, perp_dir, max_radius=200.0, nominal_half_width=60.0):
        """
        Bắn tia cắt ngang vuông góc với trục ống chân để tìm 2 mép vải thực tế và xác định chính xác Tim Ống Vải.
        Tích hợp Fallback đa tầng khi tim bị che khuất:
        1. Tìm 2 mép vải trên mask nhị phân (P_outer, P_inner) và lấy trung điểm.
        2. Nếu một mép bị che khuất (do gá kẹp, gập mép, hoặc dính ống), dùng mép còn lại + nominal_half_width.
        3. Nếu cả hai mép không xác định được, fallback về vị trí khung xương dự kiến (seed_pt).
        """
        h_img, w_img = mask.shape
        # Tạo chuỗi các bước mẫu dọc theo vector vuông góc
        ts = np.linspace(-max_radius, max_radius, int(2 * max_radius + 1))
        coords = (seed_pt[None, :] + ts[:, None] * perp_dir[None, :]).astype(int)

        # Lọc các điểm nằm trong khung ảnh
        valid_bounds = (coords[:, 0] >= 0) & (coords[:, 0] < w_img) & \
                       (coords[:, 1] >= 0) & (coords[:, 1] < h_img)

        # Kiểm tra xem điểm nào nằm trên vải (mask > 0)
        inside_mask = np.zeros(len(ts), dtype=bool)
        inside_mask[valid_bounds] = mask[coords[valid_bounds, 1], coords[valid_bounds, 0]] > 0

        center_idx = len(ts) // 2

        # Trường hợp 1: Tâm dự kiến seed_pt nằm trong vải -> Loang tìm 2 mép
        if inside_mask[center_idx]:
            left = center_idx
            while left > 0 and inside_mask[left - 1]:
                left -= 1
            right = center_idx
            while right < len(ts) - 1 and inside_mask[right + 1]:
                right += 1

            p_outer = coords[left].astype(np.float32)
            p_inner = coords[right].astype(np.float32)
            centerline_pt = (p_outer + p_inner) / 2.0
            return centerline_pt

        # Trường hợp 2: Tâm dự kiến bị rơi vào vùng che khuất / nếp lõm / mất depth
        # Tìm đoạn vải liên tục gần tâm dự kiến nhất
        fabric_indices = np.where(inside_mask)[0]
        if len(fabric_indices) > 0:
            # Nhóm các đoạn vải liên tục
            diffs = np.diff(fabric_indices)
            splits = np.where(diffs > 1)[0] + 1
            segments = np.split(fabric_indices, splits)

            # Chọn đoạn vải có tâm gần center_idx nhất
            best_seg = min(segments, key=lambda seg: abs(np.mean(seg) - center_idx))
            p_outer = coords[best_seg[0]].astype(np.float32)
            p_inner = coords[best_seg[-1]].astype(np.float32)
            return (p_outer + p_inner) / 2.0

        # Trường hợp 3: Che khuất hoàn toàn dọc theo đường quét -> Fallback về mốc xương danh nghĩa
        return seed_pt

    def extract_landmarks(self, xyz, bgr, mask, height_map, plane_params=None):
        """
        Trích xuất 12 mốc giải phẫu từ contour và hình học đám mây điểm.
        """
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None

        c = max(contours, key=cv2.contourArea)

        a = plane_params['a'] if plane_params else 0.0
        b_coef = plane_params['b'] if plane_params else 0.0
        c_plane = plane_params['c'] if plane_params else 1660.0

        # 1. Phát hiện đáy đũng (Crotch) bằng Convexity Defects
        hull = cv2.convexHull(c, returnPoints=False)
        defects = cv2.convexityDefects(c, hull)

        if defects is None:
            return None

        # Sắp xếp các khuyết lõm theo khoảng cách sâu nhất
        defects_sorted = sorted(defects, key=lambda d: d[0, 3], reverse=True)
        s, e, f, d = defects_sorted[0][0]
        crotch_pt = np.array(c[f][0], dtype=np.float32)

        # Hai đỉnh gấu ống quần (Hull endpoints)
        cuff_center1 = np.array(c[s][0], dtype=np.float32)
        cuff_center2 = np.array(c[e][0], dtype=np.float32)

        # 2. Xác định hướng ống quần và hướng cạp quần
        leg_vec = ((cuff_center1 + cuff_center2) / 2.0) - crotch_pt
        leg_norm = np.linalg.norm(leg_vec)
        if leg_norm > 0:
            leg_dir = leg_vec / leg_norm
        else:
            leg_dir = np.array([-1.0, 0.0], dtype=np.float32)

        waist_dir = -leg_dir
        perp_dir = np.array([-waist_dir[1], waist_dir[0]], dtype=np.float32)

        # Phân biệt ống 1 (trên/trái) và ống 2 (dưới/phải)
        proj_perp1 = np.dot(cuff_center1 - crotch_pt, perp_dir)
        proj_perp2 = np.dot(cuff_center2 - crotch_pt, perp_dir)
        if proj_perp1 > proj_perp2:
            tip_leg1 = cuff_center2
            tip_leg2 = cuff_center1
        else:
            tip_leg1 = cuff_center1
            tip_leg2 = cuff_center2

        # 3. Mốc cạp quần (Waist corners & center)
        pts_all = c.reshape(-1, 2).astype(np.float32)
        waist_projections = np.dot(pts_all - crotch_pt, waist_dir)
        waist_cutoff = np.percentile(waist_projections, 97)
        waist_pts = pts_all[waist_projections >= waist_cutoff]

        waist_perps = np.dot(waist_pts - crotch_pt, perp_dir)
        waist_left = waist_pts[np.argmin(waist_perps)]
        waist_right = waist_pts[np.argmax(waist_perps)]
        waist_center = (waist_left + waist_right) / 2.0

        # 4. Tìm 4 góc gấu quần (Cuff corners: inner & outer cho mỗi ống)
        # Ống 1
        dists_to_tip1 = np.linalg.norm(pts_all - tip_leg1, axis=1)
        cuff1_candidates = pts_all[dists_to_tip1 <= 70.0]
        cuff1_perps = np.dot(cuff1_candidates - crotch_pt, perp_dir)
        cuff_left_outer = cuff1_candidates[np.argmin(cuff1_perps)]
        cuff_left_inner = cuff1_candidates[np.argmax(cuff1_perps)]

        # Ống 2
        dists_to_tip2 = np.linalg.norm(pts_all - tip_leg2, axis=1)
        cuff2_candidates = pts_all[dists_to_tip2 <= 70.0]
        cuff2_perps = np.dot(cuff2_candidates - crotch_pt, perp_dir)
        cuff_right_inner = cuff2_candidates[np.argmin(cuff2_perps)]
        cuff_right_outer = cuff2_candidates[np.argmax(cuff2_perps)]

        cuff1_mid = (cuff_left_outer + cuff_left_inner) / 2.0
        cuff2_mid = (cuff_right_outer + cuff_right_inner) / 2.0

        # Vector trục và vector vuông góc riêng của từng ống chân
        vec_leg1 = cuff1_mid - crotch_pt
        norm_leg1 = np.linalg.norm(vec_leg1)
        dir_leg1 = vec_leg1 / norm_leg1 if norm_leg1 > 0 else leg_dir
        perp_leg1 = np.array([-dir_leg1[1], dir_leg1[0]], dtype=np.float32)

        vec_leg2 = cuff2_mid - crotch_pt
        norm_leg2 = np.linalg.norm(vec_leg2)
        dir_leg2 = vec_leg2 / norm_leg2 if norm_leg2 > 0 else leg_dir
        perp_leg2 = np.array([-dir_leg2[1], dir_leg2[0]], dtype=np.float32)

        # 5. Xác định Mốc Gối (Knee) & Đùi (Thigh) theo TIM ỐNG VẢI THỰC TẾ (Centerline Ray-Casting)
        # Seed sơ bộ theo tỷ lệ nhân trắc học may mặc: Đùi 28%, Gối 55%
        seed_knee1 = (1.0 - 0.55) * crotch_pt + 0.55 * cuff1_mid
        seed_knee2 = (1.0 - 0.55) * crotch_pt + 0.55 * cuff2_mid
        seed_thigh1 = (1.0 - 0.28) * crotch_pt + 0.28 * cuff1_mid
        seed_thigh2 = (1.0 - 0.28) * crotch_pt + 0.28 * cuff2_mid

        # Bắn tia tìm tim ống vải thực tế (tự động điều chỉnh khi ống bị cong / lệch / che khuất)
        knee_left = self.find_cross_section_center(mask, xyz, seed_knee1, perp_leg1, max_radius=180.0)
        knee_right = self.find_cross_section_center(mask, xyz, seed_knee2, perp_leg2, max_radius=180.0)
        thigh_left = self.find_cross_section_center(mask, xyz, seed_thigh1, perp_leg1, max_radius=220.0)
        thigh_right = self.find_cross_section_center(mask, xyz, seed_thigh2, perp_leg2, max_radius=220.0)

        raw_landmarks_2d = {
            "waist_left": waist_left,
            "waist_center": waist_center,
            "waist_right": waist_right,
            "crotch": crotch_pt,
            "thigh_left": thigh_left,
            "thigh_right": thigh_right,
            "knee_left": knee_left,
            "knee_right": knee_right,
            "cuff_left_outer": cuff_left_outer,
            "cuff_left_inner": cuff_left_inner,
            "cuff_right_inner": cuff_right_inner,
            "cuff_right_outer": cuff_right_outer
        }

        # 7. Tính toán tọa độ 3D và vector pháp tuyến cho từng mốc
        landmarks_data = {}
        for name, pt in raw_landmarks_2d.items():
            u = int(round(pt[0]))
            v = int(round(pt[1]))
            pt_3d = self.get_valid_3d_point(xyz, u, v)
            
            # Tính độ cao chính xác từ tọa độ 3D hợp lệ
            z_plane_at_pt = a * pt_3d[0] + b_coef * pt_3d[1] + c_plane
            h_val = float(z_plane_at_pt - pt_3d[2])
            normal = self.estimate_normal(xyz, u, v)

            landmarks_data[name] = {
                "pixel_u": u,
                "pixel_v": v,
                "coord_3d_mm": [float(round(pt_3d[0], 2)), float(round(pt_3d[1], 2)), float(round(pt_3d[2], 2))],
                "height_above_table_mm": float(round(h_val, 2)),
                "normal_vector": normal
            }

        return landmarks_data

    def render_landmarks_overlay(self, bgr, mask, landmarks_data):
        """Vẽ trực quan bộ mốc hình học, khung lưới và nhãn tên mốc lên ảnh màu."""
        overlay = bgr.copy()

        # Tô mờ vùng quần
        green_mask = bgr.copy()
        green_mask[mask > 0] = [0, 230, 0]
        overlay = cv2.addWeighted(overlay, 0.75, green_mask, 0.25, 0)

        # Vẽ đường biên bao
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(overlay, contours, -1, (255, 255, 255), 2)

        # 1. Vẽ các cạnh của khung lưới (Mesh Topology Edges)
        for name1, name2 in self.SKELETON_EDGES:
            if name1 in landmarks_data and name2 in landmarks_data:
                p1 = (landmarks_data[name1]["pixel_u"], landmarks_data[name1]["pixel_v"])
                p2 = (landmarks_data[name2]["pixel_u"], landmarks_data[name2]["pixel_v"])
                cv2.line(overlay, p1, p2, (0, 255, 255), 2, cv2.LINE_AA)

        # 2. Vẽ các điểm mốc (Keypoints)
        for name, data in landmarks_data.items():
            u, v = data["pixel_u"], data["pixel_v"]
            c3d = data["coord_3d_mm"]

            # Vùng đùi (màu đỏ - vùng phun PP)
            if "thigh" in name:
                color = (0, 0, 255)
                radius = 8
            # Đáy đũng (màu cam)
            elif "crotch" in name:
                color = (0, 140, 255)
                radius = 8
            # Cạp lưng (màu tím)
            elif "waist" in name:
                color = (255, 0, 255)
                radius = 7
            # Gấu quần (màu xanh dương)
            else:
                color = (255, 200, 0)
                radius = 7

            cv2.circle(overlay, (u, v), radius, color, -1, cv2.LINE_AA)
            cv2.circle(overlay, (u, v), radius + 2, (255, 255, 255), 2, cv2.LINE_AA)

            # Vẽ tên mốc và tọa độ Z
            label_text = f"{name} (Z:{c3d[2]:.0f})"
            cv2.putText(overlay, label_text, (u + 10, v - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 0, 0), 3, cv2.LINE_AA)
            cv2.putText(overlay, label_text, (u + 10, v - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (255, 255, 255), 1, cv2.LINE_AA)

        # Thêm bảng chú giải (Legend)
        cv2.rectangle(overlay, (20, 20), (320, 110), (20, 20, 20), -1)
        cv2.rectangle(overlay, (20, 20), (320, 110), (180, 180, 180), 1)
        cv2.putText(overlay, "3D Geometric Landmarks (P3)", (30, 42), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 1, cv2.LINE_AA)
        cv2.circle(overlay, (40, 62), 5, (0, 0, 255), -1)
        cv2.putText(overlay, "Thigh PP Centers", (55, 66), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
        cv2.circle(overlay, (40, 82), 5, (0, 140, 255), -1)
        cv2.putText(overlay, "Crotch Bifurcation", (55, 86), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
        cv2.circle(overlay, (40, 100), 5, (255, 200, 0), -1)
        cv2.putText(overlay, "Cuff / Waist Edges", (55, 104), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)

        return overlay

    def export_yolo_pose_label(self, landmarks_data, out_txt_path, img_w=1944, img_h=1200, class_id=0):
        """Xuất nhãn chuẩn định dạng YOLO11-pose (BBox + Keypoints [x, y, v])."""
        xs = [data["pixel_u"] for data in landmarks_data.values()]
        ys = [data["pixel_v"] for data in landmarks_data.values()]

        x_min, x_max = min(xs), max(xs)
        y_min, y_max = min(ys), max(ys)

        bbox_center_x = ((x_min + x_max) / 2.0) / img_w
        bbox_center_y = ((y_min + y_max) / 2.0) / img_h
        bbox_w = (x_max - x_min) / img_w
        bbox_h = (y_max - y_min) / img_h

        kpts_str = []
        for name in self.LANDMARK_NAMES:
            if name in landmarks_data:
                u_norm = landmarks_data[name]["pixel_u"] / img_w
                v_norm = landmarks_data[name]["pixel_v"] / img_h
                # v=2: visible and labeled
                kpts_str.append(f"{u_norm:.6f} {v_norm:.6f} 2")
            else:
                kpts_str.append("0.0 0.0 0")

        line = f"{class_id} {bbox_center_x:.6f} {bbox_center_y:.6f} {bbox_w:.6f} {bbox_h:.6f} " + " ".join(kpts_str) + "\n"
        with open(out_txt_path, "w") as f:
            f.write(line)

    def process_file(self, ply_path, output_dir):
        """Xử lý đơn lẻ 1 file PLY: nhận diện quần, trích xuất mốc 3D và lưu file."""
        os.makedirs(output_dir, exist_ok=True)
        base_name = os.path.splitext(os.path.basename(ply_path))[0]

        xyz, bgr, rgb = self.detector.load_ply(ply_path)
        plane_params = self.detector.fit_dominant_plane(xyz)
        mask, height_map = self.detector.segment_pants(xyz, bgr, plane_params)

        landmarks_data = self.extract_landmarks(xyz, bgr, mask, height_map, plane_params)
        if landmarks_data is None:
            print(f"[WARN] Không trích xuất được mốc cho file {base_name}")
            return None

        # Tạo và lưu ảnh overlay
        overlay = self.render_landmarks_overlay(bgr, mask, landmarks_data)
        overlay_path = os.path.join(output_dir, f"{base_name}_landmarks_overlay.png")
        cv2.imwrite(overlay_path, overlay)

        # Lưu JSON
        json_path = os.path.join(output_dir, f"{base_name}_landmarks_3d.json")
        plane_serializable = {
            "a": float(plane_params["a"]),
            "b": float(plane_params["b"]),
            "c": float(plane_params["c"]),
            "normal": [float(v) for v in plane_params["normal"]]
        }
        result_payload = {
            "file": os.path.basename(ply_path),
            "camera": "Zivid 2 M70",
            "plane_equation": plane_serializable,
            "landmarks_count": len(landmarks_data),
            "landmarks": landmarks_data,
            "skeleton_edges": self.SKELETON_EDGES
        }
        with open(json_path, "w") as f:
            json.dump(result_payload, f, indent=2)

        # Lưu YOLO-pose label
        pose_label_path = os.path.join(output_dir, f"{base_name}_yolo_pose.txt")
        self.export_yolo_pose_label(landmarks_data, pose_label_path)

        return result_payload, overlay_path
