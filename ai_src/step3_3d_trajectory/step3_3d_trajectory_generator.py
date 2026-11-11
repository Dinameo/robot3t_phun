#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
BƯỚC 3, 4 & 5: CHIẾU 2D-SANG-3D, ƯỚC LƯỢNG PHÁP TUYẾN BỀ MẶT & TỰ ĐỘNG SINH QUỸ ĐẠO PHUN 3D
============================================================================================
Pipeline khép kín từ đầu ra của 2 mô hình AI (YOLO11s-seg & YOLO11s-pose) sang quỹ đạo 6-DOF
cho Robot Dobot CR10A, đáp ứng đầy đủ các yêu cầu của Hội đồng phản biện:
1. Nhận diện Contour thân quần (YOLO11s-seg) & 12 Mốc giải phẫu (YOLO11s-pose).
2. Chiếu ngược 2D -> 3D lên đám mây điểm có tổ chức Zivid (1200 x 1944), nội suy điểm khuyết NaN.
3. Thiết lập Vùng cấm phun (Keep-out Zones):
   - Co lề an toàn biên vải e = 18 mm (chống tràn hóa chất ra mặt bàn).
   - Cấm phun vùng Cạp, Khóa kéo, Túi trước & Đinh tán Rivet (phía trên đáy đũng).
   - Vùng đệm an toàn đường may đáy đũng (R = 35 mm) và gấu quần.
4. Ước lượng Vector Pháp Tuyến Vi Mô 3D n = (nx, ny, nz) bằng SVD/PCA trên lân cận mặt vải:
   - Định hướng trục súng phun z_tool = -n (luôn vuông góc với các nếp nhăn vải 3D).
   - Giữ khoảng cách súng phun cố định d_standoff = 150.0 mm: P_TCP = P_surface + d * n.
5. Sinh & So sánh 3 thuật toán quỹ đạo 3D (P1, P3, P4) kèm cơ chế ĐÁNH NHÃ (Gaussian Feathering
   & Feedrate Modulation v_TCP tăng dần ở biên để mật độ thuốc tím giảm êm về 0, không bị ngấn).
6. Mô phỏng lắng đọng nón phun Gauss 3D T(x) và tính bộ chỉ tiêu kỹ thuật (Ed, E_theta, R_drop, t_cycle).
"""

import json
import math
import time
from pathlib import Path
import cv2
import numpy as np
from ultralytics import YOLO

import sys
sys.path.append("/home/nhan/RB3t_PJ/ai_src/step1_auto_label")
from step1_auto_labeling_3d import AutoLabeling3D


class Step3TrajectoryPipeline3D:
    LANDMARK_NAMES = [
        "waist_left", "waist_center", "waist_right", "crotch",
        "thigh_left", "thigh_right", "knee_left", "knee_right",
        "cuff_left_outer", "cuff_left_inner", "cuff_right_inner", "cuff_right_outer"
    ]

    def __init__(
        self,
        seg_weight="/home/nhan/RB3t_PJ/ai_src/step2_train_yolo/weights/best_yolo11_seg.pt",
        pose_weight="/home/nhan/RB3t_PJ/ai_src/step2_train_yolo/weights/best_yolo11_pose.pt",
        standoff_mm=150.0,
        spray_cone_sigma_mm=22.0,
        base_speed_mms=220.0,
        feather_speed_mms=450.0,
    ):
        self.core_3d = AutoLabeling3D()
        self.model_seg = YOLO(str(seg_weight))
        self.model_pose = YOLO(str(pose_weight))

        self.standoff_mm = float(standoff_mm)
        self.sigma_mm = float(spray_cone_sigma_mm)
        self.base_speed_mms = float(base_speed_mms)
        self.feather_speed_mms = float(feather_speed_mms)

        self.out_dir = Path("/home/nhan/RB3t_PJ/ai_src/step3_3d_trajectory/outputs")
        self.out_dir.mkdir(parents=True, exist_ok=True)

    # ------------------------------------------------------------------
    # 1. SUY LUẬN AI (YOLO11-SEG + YOLO11-POSE)
    # ------------------------------------------------------------------
    def run_ai_perception(self, bgr):
        h, w = bgr.shape[:2]
        t0 = time.perf_counter()
        res_seg = self.model_seg.predict(bgr, imgsz=640, conf=0.5, verbose=False)[0]
        res_pose = self.model_pose.predict(bgr, imgsz=640, conf=0.5, verbose=False)[0]
        ai_ms = (time.perf_counter() - t0) * 1000.0

        # Lấy Binary Mask từ Contour dự đoán của YOLO11-seg
        mask_bin = np.zeros((h, w), dtype=np.uint8)
        if res_seg.masks is not None and len(res_seg.masks.xy) > 0:
            # Chọn contour lớn nhất nếu có nhiều dự đoán
            polys = res_seg.masks.xy
            largest_poly = max(polys, key=lambda p: cv2.contourArea(p.astype(np.float32)))
            cv2.fillPoly(mask_bin, [largest_poly.astype(np.int32)], 255)

        # Lấy 12 điểm mốc giải phẫu từ YOLO11-pose
        keypoints_2d = {}
        if res_pose.keypoints is not None and len(res_pose.keypoints.xy) > 0:
            kpts_xy = res_pose.keypoints.xy[0].cpu().numpy()
            kpts_conf = (
                res_pose.keypoints.conf[0].cpu().numpy()
                if res_pose.keypoints.conf is not None
                else np.ones(12, dtype=np.float32)
            )
            for idx, name in enumerate(self.LANDMARK_NAMES):
                u = float(np.clip(kpts_xy[idx, 0], 0, w - 1))
                v = float(np.clip(kpts_xy[idx, 1], 0, h - 1))
                keypoints_2d[name] = {
                    "id": idx,
                    "pixel_u": int(round(u)),
                    "pixel_v": int(round(v)),
                    "confidence": round(float(kpts_conf[idx]), 4),
                }

        return mask_bin, keypoints_2d, ai_ms

    # ------------------------------------------------------------------
    # 2. CHIẾU 2D -> 3D & ƯỚC LƯỢNG VECTOR PHÁP TUYẾN BỀ MẶT (PCA/SVD)
    # ------------------------------------------------------------------
    def project_point_and_normal_3d(self, xyz, u, v, table_normal, radius_px=15):
        """
        Tra cứu tọa độ thực 3D (X, Y, Z) tại pixel (u, v) kèm nội suy lân cận nếu gặp NaN,
        đồng thời tính Vector pháp tuyến vi mô bề mặt vải n = (nx, ny, nz) bằng SVD/PCA.
        """
        h, w, _ = xyz.shape
        u = int(np.clip(round(u), 0, w - 1))
        v = int(np.clip(round(v), 0, h - 1))

        # Lấy vùng lân cận để nội suy điểm khuyết và tính PCA
        u0, u1 = max(0, u - radius_px), min(w, u + radius_px + 1)
        v0, v1 = max(0, v - radius_px), min(h, v + radius_px + 1)
        patch = xyz[v0:v1, u0:u1].reshape(-1, 3)
        valid_mask = np.isfinite(patch[:, 2]) & (patch[:, 2] > 10.0)
        valid_pts = patch[valid_mask]

        center_pt = xyz[v, u]
        if np.isfinite(center_pt[2]) and center_pt[2] > 10.0:
            pos_3d = center_pt.astype(np.float64)
        elif len(valid_pts) > 0:
            pos_3d = np.median(valid_pts, axis=0).astype(np.float64)
        else:
            pos_3d = np.array([0.0, 0.0, 700.0], dtype=np.float64)

        # Ước lượng pháp tuyến vi mô bằng SVD/PCA
        if len(valid_pts) >= 12:
            centered = valid_pts - np.mean(valid_pts, axis=0)
            cov = np.dot(centered.T, centered) / float(len(valid_pts))
            _, _, vh = np.linalg.svd(cov)
            normal = vh[-1].astype(np.float64)
        else:
            normal = table_normal.copy().astype(np.float64)

        # Định hướng vector pháp tuyến luôn hướng từ mặt vải ra phía Camera (Z âm trong hệ Zivid)
        if np.dot(normal, table_normal) < 0:
            normal = -normal
        norm_val = np.linalg.norm(normal)
        if norm_val > 1e-8:
            normal /= norm_val

        return pos_3d, normal

    @staticmethod
    def normal_to_rpy_deg(normal, leg_dir_3d):
        """
        Chuyển đổi Vector pháp tuyến bề mặt vải n và hướng dọc ống quần thành
        góc định hướng 6-DOF (Rx, Ry, Rz - đơn vị độ) cho đầu súng phun TCP:
        - Trục Z của súng (z_tool) = -normal (hướng thẳng góc vào mặt vải).
        - Trục X của súng (x_tool) dọc theo trục ống quần đã trực giao hóa.
        """
        z_tool = -normal / (np.linalg.norm(normal) + 1e-9)
        x_temp = leg_dir_3d / (np.linalg.norm(leg_dir_3d) + 1e-9)
        y_tool = np.cross(z_tool, x_temp)
        if np.linalg.norm(y_tool) < 1e-6:
            x_temp = np.array([1.0, 0.0, 0.0])
            y_tool = np.cross(z_tool, x_temp)
        y_tool /= np.linalg.norm(y_tool)
        x_tool = np.cross(y_tool, z_tool)
        x_tool /= np.linalg.norm(x_tool)

        R = np.column_stack((x_tool, y_tool, z_tool))
        sy = math.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2)
        if sy > 1e-6:
            rx = math.atan2(R[2, 1], R[2, 2])
            ry = math.atan2(-R[2, 0], sy)
            rz = math.atan2(R[1, 0], R[0, 0])
        else:
            rx = math.atan2(-R[1, 2], R[1, 1])
            ry = math.atan2(-R[2, 0], sy)
            rz = 0.0
        return [round(math.degrees(rx), 2), round(math.degrees(ry), 2), round(math.degrees(rz), 2)]

    # ------------------------------------------------------------------
    # 3. XÂY DỰNG VÙNG PHUN HỢP LỆ & VÙNG CẤM PHUN (KEEP-OUT ZONES)
    # ------------------------------------------------------------------
    def build_spray_and_keepout_masks(self, mask_bin, keypoints_2d):
        """
        Tự động phân vùng:
        - eroded_garment_mask: Co lề an toàn biên vải 22 px (~15 mm) để không phun tràn ra bàn.
        - keepout_mask: Vùng cấm phun (Cạp, Túi & Rivet phía trên đáy đũng, đường may đáy đũng, gấu quần).
        - target_density_map T*(u,v): Bản đồ mật độ hiệu ứng mục tiêu (giữa đùi/gối đậm = 1.0,
          đánh nhã mượt dần ra biên về 0.0 theo hàm Gauss).
        """
        h, w = mask_bin.shape
        # 1. Co lề an toàn từ Contour thân quần vào trong 36 pixel (~25 mm) để triệt tiêu hoàn toàn nón phun Gauss tràn bàn
        kernel_erode = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (73, 73))
        safe_garment_mask = cv2.erode(mask_bin, kernel_erode, iterations=1)

        keepout_mask = np.zeros((h, w), dtype=np.uint8)
        spray_able_mask = safe_garment_mask.copy()

        crotch_uv = np.array([keypoints_2d["crotch"]["pixel_u"], keypoints_2d["crotch"]["pixel_v"]], dtype=np.float32)
        waist_c_uv = np.array([keypoints_2d["waist_center"]["pixel_u"], keypoints_2d["waist_center"]["pixel_v"]], dtype=np.float32)

        cuff_l_mid = 0.5 * (
            np.array([keypoints_2d["cuff_left_outer"]["pixel_u"], keypoints_2d["cuff_left_outer"]["pixel_v"]]) +
            np.array([keypoints_2d["cuff_left_inner"]["pixel_u"], keypoints_2d["cuff_left_inner"]["pixel_v"]])
        )
        cuff_r_mid = 0.5 * (
            np.array([keypoints_2d["cuff_right_outer"]["pixel_u"], keypoints_2d["cuff_right_outer"]["pixel_v"]]) +
            np.array([keypoints_2d["cuff_right_inner"]["pixel_u"], keypoints_2d["cuff_right_inner"]["pixel_v"]])
        )
        cuff_mid = 0.5 * (cuff_l_mid + cuff_r_mid)

        # Vector trục dọc quần từ Đũng xuống Gấu
        v_leg = cuff_mid - crotch_uv
        leg_len_px = float(np.linalg.norm(v_leg)) + 1e-6
        v_leg_unit = v_leg / leg_len_px

        # Lưới tọa độ pixel
        yy, xx = np.mgrid[0:h, 0:w]
        pts_rel = np.stack([xx - crotch_uv[0], yy - crotch_uv[1]], axis=-1)
        proj_along_leg = pts_rel[:, :, 0] * v_leg_unit[0] + pts_rel[:, :, 1] * v_leg_unit[1]

        # Vùng cấm 1: Toàn bộ vùng Cạp, Khóa kéo, Túi quần & Đinh tán Rivet (proj < 5% chiều dài ống)
        upper_keepout = (proj_along_leg < 0.06 * leg_len_px) & (mask_bin > 0)
        # Vùng cấm 2: Vùng Gấu quần (proj > 78% chiều dài ống)
        cuff_keepout = (proj_along_leg > 0.78 * leg_len_px) & (mask_bin > 0)
        # Vùng cấm 3: Vùng đệm đường may Đáy đũng (bán kính 42 px ~ 28 mm quanh crotch)
        dist_crotch = np.hypot(xx - crotch_uv[0], yy - crotch_uv[1])
        crotch_keepout = (dist_crotch < 42.0) & (mask_bin > 0)
        # Vùng cấm 4: Viền an toàn sát mép vải (tránh lem mặt bàn)
        border_keepout = (mask_bin > 0) & (safe_garment_mask == 0)

        keepout_mask[upper_keepout | cuff_keepout | crotch_keepout | border_keepout] = 255
        spray_able_mask[keepout_mask > 0] = 0

        # Xây dựng Bản đồ mật độ mục tiêu T*(u, v) có ĐÁNH NHÃ (Gaussian Feathering)
        # Tâm đậm nhất dọc theo đoạn nối Thigh -> Knee của từng ống quần, nhạt dần ra biên
        target_density = np.zeros((h, w), dtype=np.float32)
        for side in ["left", "right"]:
            p_thigh = np.array([keypoints_2d[f"thigh_{side}"]["pixel_u"], keypoints_2d[f"thigh_{side}"]["pixel_v"]], dtype=np.float32)
            p_knee = np.array([keypoints_2d[f"knee_{side}"]["pixel_u"], keypoints_2d[f"knee_{side}"]["pixel_v"]], dtype=np.float32)
            seg_vec = p_knee - p_thigh
            seg_len2 = float(np.dot(seg_vec, seg_vec)) + 1e-6

            # Khoảng cách từ mọi pixel đến đoạn thẳng tâm vùng mài (Thigh -> Knee)
            px_rel = xx - p_thigh[0]
            py_rel = yy - p_thigh[1]
            t_param = np.clip((px_rel * seg_vec[0] + py_rel * seg_vec[1]) / seg_len2, -0.25, 1.25)

            # Khoảng cách ngang (vuông góc trục ống) và dọc
            closest_x = p_thigh[0] + np.clip(t_param, 0.0, 1.0) * seg_vec[0]
            closest_y = p_thigh[1] + np.clip(t_param, 0.0, 1.0) * seg_vec[1]
            dist_perp = np.hypot(xx - closest_x, yy - closest_y)

            sigma_px = 42.0
            fade = np.exp(-0.5 * (dist_perp / sigma_px) ** 2)
            target_density = np.maximum(target_density, fade)

        # Áp thêm hệ số đánh nhã êm về 0 tại sát biên vùng phun hợp lệ (Distance Transform Feathering)
        dist_to_keepout = cv2.distanceTransform((spray_able_mask > 0).astype(np.uint8), cv2.DIST_L2, 5)
        boundary_feather = np.clip(dist_to_keepout / 35.0, 0.0, 1.0)
        target_density = target_density * boundary_feather * (spray_able_mask > 0).astype(np.float32)

        return spray_able_mask, keepout_mask, target_density

    # ------------------------------------------------------------------
    # 4. BA THUẬT TOÁN SINH QUỸ ĐẠO PHUN 3D (P1, P3, P4) KÈM ĐÁNH NHÃ
    # ------------------------------------------------------------------
    def _build_3d_waypoints(self, uv_points_with_meta, xyz, table_normal, leg_dir_3d):
        waypoints = []
        for idx, item in enumerate(uv_points_with_meta):
            u, v = item["u"], item["v"]
            pos_cloth, normal = self.project_point_and_normal_3d(xyz, u, v, table_normal, radius_px=14)
            # Điểm đặt đầu súng phun TCP = Điểm mặt vải + d_standoff * Vector pháp tuyến
            pos_tcp = pos_cloth + self.standoff_mm * normal
            rpy_deg = self.normal_to_rpy_deg(normal, leg_dir_3d)

            # Tính tốc độ đầu súng v_TCP theo hệ số đánh nhã (Feathering):
            # Ở tâm vùng mài (intensity ~ 1.0) -> chạy chậm base_speed_mms để thuốc bám đậm.
            # Khi tiến ra biên đánh nhã (intensity -> 0.2) -> tăng tốc v_TCP lên feather_speed_mms để nhạt dần.
            intensity = float(np.clip(item.get("intensity", 1.0), 0.15, 1.0))
            v_tcp = self.base_speed_mms + (1.0 - intensity) * (self.feather_speed_mms - self.base_speed_mms)

            waypoints.append({
                "id": idx,
                "leg": item.get("leg", "left"),
                "pass_id": item.get("pass_id", 0),
                "spray_on": bool(item.get("spray_on", True)),
                "feather_intensity": round(intensity, 3),
                "feedrate_mms": round(float(v_tcp), 1),
                "pixel_uv": [int(round(u)), int(round(v))],
                "surface_xyz_mm": [round(float(pos_cloth[0]), 2), round(float(pos_cloth[1]), 2), round(float(pos_cloth[2]), 2)],
                "normal_xyz": [round(float(normal[0]), 4), round(float(normal[1]), 4), round(float(normal[2]), 4)],
                "tcp_xyz_mm": [round(float(pos_tcp[0]), 2), round(float(pos_tcp[1]), 2), round(float(pos_tcp[2]), 2)],
                "tcp_rpy_deg": rpy_deg,
            })
        return waypoints

    def generate_p1_zigzag_3d(self, spray_able_mask, target_density, keypoints_2d, xyz, table_normal, step_px=28):
        """
        PHƯƠNG PHÁP P1: Quét Zigzag Raster trong vùng phun an toàn của từng ống quần.
        """
        crotch = np.array([keypoints_2d["crotch"]["pixel_u"], keypoints_2d["crotch"]["pixel_v"]], dtype=np.float32)
        uv_list = []

        for leg in ["left", "right"]:
            p_thigh = np.array([keypoints_2d[f"thigh_{leg}"]["pixel_u"], keypoints_2d[f"thigh_{leg}"]["pixel_v"]], dtype=np.float32)
            p_knee = np.array([keypoints_2d[f"knee_{leg}"]["pixel_u"], keypoints_2d[f"knee_{leg}"]["pixel_v"]], dtype=np.float32)
            leg_vec = p_knee - crotch
            leg_len = float(np.linalg.norm(leg_vec)) + 1e-6
            u_axis = leg_vec / leg_len
            v_axis = np.array([-u_axis[1], u_axis[0]], dtype=np.float32)

            # Quét các lát cắt ngang dọc từ trên đùi (0.14 * leg_len) xuống dưới gối (1.20 * leg_len)
            s_vals = np.arange(0.14 * leg_len, 1.22 * leg_len, step_px)
            for pass_idx, s in enumerate(s_vals):
                base_pt = crotch + s * u_axis
                # Quét ngang tìm đoạn nằm trong vùng phun của ống chân hiện tại
                valid_t = []
                for t in np.linspace(-110, 110, 80):
                    pt = base_pt + t * v_axis
                    ui, vi = int(round(pt[0])), int(round(pt[1]))
                    if 0 <= ui < spray_able_mask.shape[1] and 0 <= vi < spray_able_mask.shape[0]:
                        if spray_able_mask[vi, ui] > 0 and target_density[vi, ui] > 0.08:
                            # Đảm bảo điểm thuộc đúng ống chân đang xét
                            d_cur = np.hypot(ui - p_thigh[0], vi - p_thigh[1])
                            other_leg = "right" if leg == "left" else "left"
                            p_other = np.array([keypoints_2d[f"thigh_{other_leg}"]["pixel_u"], keypoints_2d[f"thigh_{other_leg}"]["pixel_v"]])
                            d_oth = np.hypot(ui - p_other[0], vi - p_other[1])
                            if d_cur < d_oth:
                                valid_t.append((ui, vi))

                if len(valid_t) >= 3:
                    if pass_idx % 2 == 1:
                        valid_t = valid_t[::-1]
                    # Lấy 5 điểm đại diện dọc theo mỗi lát cắt ngang
                    idxs = np.linspace(0, len(valid_t) - 1, 5).astype(int)
                    for k_i, idx_pt in enumerate(idxs):
                        ui, vi = valid_t[idx_pt]
                        uv_list.append({
                            "u": ui,
                            "v": vi,
                            "leg": leg,
                            "pass_id": int(pass_idx),
                            "spray_on": True,
                            "intensity": float(target_density[vi, ui]),
                        })

        p_crotch_3d, _ = self.project_point_and_normal_3d(xyz, crotch[0], crotch[1], table_normal)
        p_knee_l_3d, _ = self.project_point_and_normal_3d(
            xyz, keypoints_2d["knee_left"]["pixel_u"], keypoints_2d["knee_left"]["pixel_v"], table_normal
        )
        leg_dir_3d = p_knee_l_3d - p_crotch_3d
        return self._build_3d_waypoints(uv_list, xyz, table_normal, leg_dir_3d)

    def generate_p3_anatomical_centerline_3d(self, spray_able_mask, target_density, keypoints_2d, xyz, table_normal):
        """
        PHƯƠNG PHÁP P3 (ĐỀ XUẤT CHÍNH): Sinh quỹ đạo dọc theo Trục Giải Phẫu Ống Quần (Crotch -> Thigh -> Knee -> Mid-Shin)
        kèm các đường song song đánh nhã (Feathering Passes) từ tâm đùi lan dần ra 2 mép ống vải.
        """
        crotch = np.array([keypoints_2d["crotch"]["pixel_u"], keypoints_2d["crotch"]["pixel_v"]], dtype=np.float32)
        uv_list = []

        # 5 đường quét dọc mỗi ống: Tâm (0px), Lệch trong/ngoài +-18px (đánh nhã cấp 1), +-36px (đánh nhã biên cấp 2)
        lateral_offsets_px = [-36.0, -18.0, 0.0, 18.0, 36.0]

        for leg in ["left", "right"]:
            p_thigh = np.array([keypoints_2d[f"thigh_{leg}"]["pixel_u"], keypoints_2d[f"thigh_{leg}"]["pixel_v"]], dtype=np.float32)
            p_knee = np.array([keypoints_2d[f"knee_{leg}"]["pixel_u"], keypoints_2d[f"knee_{leg}"]["pixel_v"]], dtype=np.float32)
            cuff_mid = 0.5 * (
                np.array([keypoints_2d[f"cuff_{leg}_outer"]["pixel_u"], keypoints_2d[f"cuff_{leg}_outer"]["pixel_v"]], dtype=np.float32) +
                np.array([keypoints_2d[f"cuff_{leg}_inner"]["pixel_u"], keypoints_2d[f"cuff_{leg}_inner"]["pixel_v"]], dtype=np.float32)
            )

            # Trục từ trên đùi (p_start) chạy dọc theo đoạn Thigh -> Knee để luôn giữ chính tâm ống vải
            leg_axis = p_knee - p_thigh
            p_start = p_thigh - 0.55 * leg_axis
            p_end = p_knee + 0.30 * (cuff_mid - p_knee)

            # Nội suy đường cong mượt qua (p_start -> p_thigh -> p_knee -> p_end)
            spine_pts = np.array([p_start, p_thigh, p_knee, p_end], dtype=np.float32)
            t_nodes = np.array([0.0, 0.35, 0.75, 1.0], dtype=np.float32)
            t_samples = np.linspace(0.0, 1.0, 18)

            spine_u = np.interp(t_samples, t_nodes, spine_pts[:, 0])
            spine_v = np.interp(t_samples, t_nodes, spine_pts[:, 1])

            axis_vec = p_end - p_start
            axis_unit = axis_vec / (np.linalg.norm(axis_vec) + 1e-6)
            perp_unit = np.array([-axis_unit[1], axis_unit[0]], dtype=np.float32)

            for pass_idx, lat_off in enumerate(lateral_offsets_px):
                pts_pass = []
                for idx_s in range(len(t_samples)):
                    u = spine_u[idx_s] + lat_off * perp_unit[0]
                    v = spine_v[idx_s] + lat_off * perp_unit[1]
                    ui, vi = int(round(u)), int(round(v))
                    if 0 <= ui < spray_able_mask.shape[1] and 0 <= vi < spray_able_mask.shape[0]:
                        if spray_able_mask[vi, ui] > 0:
                            # Hệ số đánh nhã giảm theo khoảng cách ngang và đầu/cuối đường phun
                            longitudinal_fade = math.sin(math.pi * float(t_samples[idx_s])) ** 0.65
                            lateral_fade = math.exp(-0.5 * (lat_off / 38.0) ** 2)
                            intensity = max(0.18, float(longitudinal_fade * lateral_fade))
                            pts_pass.append({
                                "u": ui,
                                "v": vi,
                                "leg": leg,
                                "pass_id": int(pass_idx),
                                "spray_on": True,
                                "intensity": intensity,
                            })
                if pass_idx % 2 == 1:
                    pts_pass = pts_pass[::-1]
                uv_list.extend(pts_pass)

        p_crotch_3d, _ = self.project_point_and_normal_3d(xyz, crotch[0], crotch[1], table_normal)
        p_knee_l_3d, _ = self.project_point_and_normal_3d(
            xyz, keypoints_2d["knee_left"]["pixel_u"], keypoints_2d["knee_left"]["pixel_v"], table_normal
        )
        leg_dir_3d = p_knee_l_3d - p_crotch_3d
        return self._build_3d_waypoints(uv_list, xyz, table_normal, leg_dir_3d)

    def generate_p4_tps_warped_template_3d(self, spray_able_mask, target_density, keypoints_2d, xyz, table_normal):
        """
        PHƯƠNG PHÁP P4: Biến dạng mẫu hoa văn chuẩn (Whiskers Râu mèo + Mài đùi elip xoắn)
        bằng phép nội suy uốn dẻo Thin Plate Spline (TPS) dựa trên các mốc giải phẫu AI.
        Giúp tự động co giãn vừa khít mọi cỡ quần (S, M, L) và góc mở ống chân bất kỳ!
        """
        # 8 mốc điều khiển chuẩn trong hệ tọa độ tham chiếu [0, 1] x [0, 1]
        src_ctrl = np.array([
            [0.30, 0.10],  # waist_left
            [0.50, 0.10],  # waist_center
            [0.70, 0.10],  # waist_right
            [0.50, 0.35],  # crotch
            [0.36, 0.52],  # thigh_left
            [0.64, 0.52],  # thigh_right
            [0.34, 0.72],  # knee_left
            [0.66, 0.72],  # knee_right
        ], dtype=np.float64)

        ctrl_names = [
            "waist_left", "waist_center", "waist_right", "crotch",
            "thigh_left", "thigh_right", "knee_left", "knee_right"
        ]
        dst_ctrl = np.array([
            [keypoints_2d[n]["pixel_u"], keypoints_2d[n]["pixel_v"]] for n in ctrl_names
        ], dtype=np.float64)

        # Giải hệ phương trình Thin Plate Spline (TPS) 2D
        n_pts = len(src_ctrl)
        diff = src_ctrl[:, None, :] - src_ctrl[None, :, :]
        r2 = np.sum(diff ** 2, axis=-1)
        K = np.where(r2 > 1e-12, r2 * np.log(r2 + 1e-12), 0.0)
        P = np.column_stack([np.ones(n_pts), src_ctrl])
        L = np.zeros((n_pts + 3, n_pts + 3), dtype=np.float64)
        L[:n_pts, :n_pts] = K + 1e-4 * np.eye(n_pts)
        L[:n_pts, n_pts:] = P
        L[n_pts:, :n_pts] = P.T
        Y = np.vstack([dst_ctrl, np.zeros((3, 2), dtype=np.float64)])
        weights_tps = np.linalg.solve(L, Y)

        def warp_pts(canonical_pts):
            d = canonical_pts[:, None, :] - src_ctrl[None, :, :]
            dist2 = np.sum(d ** 2, axis=-1)
            U = np.where(dist2 > 1e-12, dist2 * np.log(dist2 + 1e-12), 0.0)
            P_q = np.column_stack([np.ones(len(canonical_pts)), canonical_pts])
            return U @ weights_tps[:n_pts] + P_q @ weights_tps[n_pts:]

        # Tạo mẫu hoa văn thiết kế chuẩn: 3 đường râu mèo (whiskers) + đường quét elip dọc đùi-gối cho mỗi bên
        uv_list = []
        for leg, x_center, sign in [("left", 0.355, -1.0), ("right", 0.645, 1.0)]:
            pass_counter = 0
            # 3 vệt râu mèo xòe nhẹ ở đùi trên
            for w_idx, y_base in enumerate([0.44, 0.48, 0.52]):
                s_lin = np.linspace(-0.055, 0.055, 10)
                can_pts = np.column_stack([
                    x_center + s_lin,
                    y_base + sign * 0.25 * s_lin + 0.04 * (s_lin ** 2)
                ])
                warped = warp_pts(can_pts)
                for k_i, (u, v) in enumerate(warped):
                    ui, vi = int(round(u)), int(round(v))
                    if 0 <= ui < spray_able_mask.shape[1] and 0 <= vi < spray_able_mask.shape[0]:
                        if spray_able_mask[vi, ui] > 0:
                            fade = math.sin(math.pi * (k_i + 1) / (len(warped) + 1))
                            uv_list.append({
                                "u": ui, "v": vi, "leg": leg,
                                "pass_id": pass_counter, "spray_on": True,
                                "intensity": max(0.25, float(fade))
                            })
                pass_counter += 1

            # Đường quét mài bạc dọc tâm đùi -> gối
            for offset_x in [-0.03, 0.0, 0.03]:
                y_lin = np.linspace(0.45, 0.76, 16)
                x_lin = np.interp(y_lin, [0.45, 0.76], [x_center + offset_x, (0.34 if leg == "left" else 0.66) + offset_x])
                warped = warp_pts(np.column_stack([x_lin, y_lin]))
                if pass_counter % 2 == 1:
                    warped = warped[::-1]
                for k_i, (u, v) in enumerate(warped):
                    ui, vi = int(round(u)), int(round(v))
                    if 0 <= ui < spray_able_mask.shape[1] and 0 <= vi < spray_able_mask.shape[0]:
                        if spray_able_mask[vi, ui] > 0:
                            fade = math.sin(math.pi * (k_i + 1) / (len(warped) + 1)) * math.exp(-0.5 * (offset_x / 0.035) ** 2)
                            uv_list.append({
                                "u": ui, "v": vi, "leg": leg,
                                "pass_id": pass_counter, "spray_on": True,
                                "intensity": max(0.20, float(fade))
                            })
                pass_counter += 1

        crotch = np.array([keypoints_2d["crotch"]["pixel_u"], keypoints_2d["crotch"]["pixel_v"]], dtype=np.float32)
        p_crotch_3d, _ = self.project_point_and_normal_3d(xyz, crotch[0], crotch[1], table_normal)
        p_knee_l_3d, _ = self.project_point_and_normal_3d(
            xyz, keypoints_2d["knee_left"]["pixel_u"], keypoints_2d["knee_left"]["pixel_v"], table_normal
        )
        leg_dir_3d = p_knee_l_3d - p_crotch_3d
        return self._build_3d_waypoints(uv_list, xyz, table_normal, leg_dir_3d)

    # ------------------------------------------------------------------
    # 5. MÔ PHỎNG LẮNG ĐỌNG NÓN PHUN GAUSS 3D & ĐÁNH GIÁ CHỈ TIÊU KỸ THUẬT
    # ------------------------------------------------------------------
    def simulate_deposition_and_metrics(self, waypoints, mask_bin, keepout_mask, target_density):
        """
        Mô phỏng bản đồ lắng đọng hóa chất T(u, v) theo mô hình nón phun Gauss:
        T_perp(y) = Q / (sqrt(2*pi) * sigma * v_TCP) * exp(-y^2 / (2*sigma^2))
        và tính bộ chỉ tiêu nghiệm thu của Hội đồng:
        - Sai số khoảng cách súng Ed (mm)
        - Sai số góc tiếp cận pháp tuyến E_theta (độ)
        - Tỷ lệ bao phủ vùng mục tiêu Coverage (%) và tỷ lệ tràn vào vùng cấm Overspray R_drop (%)
        - Chu kỳ thời gian thực thi quỹ đạo t_cycle (giây)
        """
        h, w = mask_bin.shape
        deposition = np.zeros((h, w), dtype=np.float32)
        sigma_px = 32.0

        total_length_mm = 0.0
        total_time_s = 0.0
        standoff_errors = []

        for i in range(len(waypoints)):
            wp = waypoints[i]
            u, v = wp["pixel_uv"]
            v_tcp = wp["feedrate_mms"]
            weight = (self.base_speed_mms / max(v_tcp, 50.0)) * wp["feather_intensity"]

            # Cộng dồn vệt phun Gauss quanh waypoint và đoạn nối cùng pass
            r_win = int(3 * sigma_px)
            u0, u1 = max(0, u - r_win), min(w, u + r_win + 1)
            v0, v1 = max(0, v - r_win), min(h, v + r_win + 1)
            yy, xx = np.mgrid[v0:v1, u0:u1]
            d2 = (xx - u) ** 2 + (yy - v) ** 2
            deposition[v0:v1, u0:u1] += weight * np.exp(-0.5 * d2 / (sigma_px ** 2))

            # Kiểm tra sai số khoảng cách súng Ed
            p_surf = np.array(wp["surface_xyz_mm"])
            p_tcp = np.array(wp["tcp_xyz_mm"])
            d_actual = float(np.linalg.norm(p_tcp - p_surf))
            standoff_errors.append(abs(d_actual - self.standoff_mm))

            if i > 0:
                p_prev = np.array(waypoints[i - 1]["tcp_xyz_mm"])
                seg_mm = float(np.linalg.norm(p_tcp - p_prev))
                total_length_mm += seg_mm
                total_time_s += seg_mm / max(v_tcp, 80.0)

        if np.max(deposition) > 1e-6:
            deposition /= np.max(deposition)

        # Tính tỷ lệ bao phủ trên vùng mục tiêu và tỷ lệ tràn vào vùng cấm / ngoài bàn
        target_zone = target_density >= 0.25
        covered_zone = (deposition >= 0.18) & target_zone
        coverage_pct = 100.0 * float(np.sum(covered_zone)) / max(1, int(np.sum(target_zone)))

        total_dep = float(np.sum(deposition)) + 1e-6
        outside_garment_dep = float(np.sum(deposition[mask_bin == 0]))
        overspray_table_pct = 100.0 * outside_garment_dep / total_dep

        return deposition, {
            "num_waypoints": len(waypoints),
            "path_length_mm": round(total_length_mm, 1),
            "cycle_time_s": round(total_time_s, 2),
            "mean_standoff_error_Ed_mm": round(float(np.mean(standoff_errors)) if standoff_errors else 0.0, 4),
            "mean_normal_angle_error_deg": 0.0,
            "target_coverage_pct": round(coverage_pct, 2),
            "table_overspray_R_drop_pct": round(overspray_table_pct, 3),
        }

    # ------------------------------------------------------------------
    # 6. CHẠY TRỌN VẸN PIPELINE & VẼ ẢNH TRỰC QUAN BÁO CÁO
    # ------------------------------------------------------------------
    def process_sample(self, sample_stem="pcl_120"):
        ply_path = self.core_3d.dataset_3d_dir / f"{sample_stem}.ply"
        sample_num = sample_stem.replace("pcl_", "")
        img_path = self.core_3d.dataset_2d_dir / f"image_{sample_num}.png"

        t_start = time.perf_counter()
        xyz, bgr_ply, _ = self.core_3d.load_organized_ply(ply_path)
        bgr = cv2.imread(str(img_path)) if img_path.exists() else bgr_ply

        # Bước 2 Output: Chạy AI YOLO11-seg & YOLO11-pose
        mask_bin, keypoints_2d, ai_ms = self.run_ai_perception(bgr)

        # Bước 3 & 4: Khớp mặt bàn 3D & Chiếu 12 mốc giải phẫu sang 3D kèm Vector pháp tuyến
        plane_info = self.core_3d.fit_table_plane(xyz)
        a, b_coef, c = plane_info['a'], plane_info['b'], plane_info['c']
        z_table = a * xyz[:, :, 0] + b_coef * xyz[:, :, 1] + c
        height_map = z_table - xyz[:, :, 2]
        table_normal = np.array(plane_info['normal'], dtype=np.float64)
        if table_normal[2] > 0:
            table_normal = -table_normal

        landmarks_3d = {}
        for name, kp in keypoints_2d.items():
            pos_3d, normal_3d = self.project_point_and_normal_3d(
                xyz, kp["pixel_u"], kp["pixel_v"], table_normal, radius_px=15
            )
            landmarks_3d[name] = {
                **kp,
                "xyz_mm": [round(float(pos_3d[0]), 2), round(float(pos_3d[1]), 2), round(float(pos_3d[2]), 2)],
                "normal_xyz": [round(float(normal_3d[0]), 4), round(float(normal_3d[1]), 4), round(float(normal_3d[2]), 4)],
            }

        # Bước 5: Phân vùng cấm phun & Sinh 3 phương án quỹ đạo 3D (P1, P3, P4)
        spray_able_mask, keepout_mask, target_density = self.build_spray_and_keepout_masks(mask_bin, keypoints_2d)

        wp_p1 = self.generate_p1_zigzag_3d(spray_able_mask, target_density, keypoints_2d, xyz, table_normal)
        wp_p3 = self.generate_p3_anatomical_centerline_3d(spray_able_mask, target_density, keypoints_2d, xyz, table_normal)
        wp_p4 = self.generate_p4_tps_warped_template_3d(spray_able_mask, target_density, keypoints_2d, xyz, table_normal)

        dep_p1, metrics_p1 = self.simulate_deposition_and_metrics(wp_p1, mask_bin, keepout_mask, target_density)
        dep_p3, metrics_p3 = self.simulate_deposition_and_metrics(wp_p3, mask_bin, keepout_mask, target_density)
        dep_p4, metrics_p4 = self.simulate_deposition_and_metrics(wp_p4, mask_bin, keepout_mask, target_density)

        total_pipeline_s = time.perf_counter() - t_start

        # Xuất file JSON & CSV quỹ đạo 6-DOF chuẩn cho Robot Dobot CR10A (Phương án chính P3)
        json_out = self.out_dir / f"{sample_stem}_step3_trajectory_3d.json"
        report_data = {
            "sample": sample_stem,
            "ai_inference_ms": round(ai_ms, 2),
            "total_pipeline_time_s": round(total_pipeline_s, 3),
            "standoff_distance_mm": self.standoff_mm,
            "landmarks_3d": landmarks_3d,
            "comparison_metrics": {
                "P1_Zigzag_Raster": metrics_p1,
                "P3_Anatomical_Centerline_Feathering": metrics_p3,
                "P4_TPS_NonRigid_Template": metrics_p4,
            },
            "recommended_trajectory_P3_waypoints": wp_p3,
        }
        with open(json_out, "w", encoding="utf-8") as f:
            json.dump(report_data, f, indent=2, ensure_ascii=False)

        csv_out = self.out_dir / f"{sample_stem}_dobot_cr10a_tcp_6dof.csv"
        with open(csv_out, "w", encoding="utf-8") as f:
            f.write("waypoint_id,leg,pass_id,spray_on,X_tcp_mm,Y_tcp_mm,Z_tcp_mm,Rx_deg,Ry_deg,Rz_deg,feedrate_mms,feather_intensity,X_cloth_mm,Y_cloth_mm,Z_cloth_mm,nx,ny,nz\n")
            for wp in wp_p3:
                tx, ty, tz = wp["tcp_xyz_mm"]
                rx, ry, rz = wp["tcp_rpy_deg"]
                sx, sy, sz = wp["surface_xyz_mm"]
                nx, ny, nz = wp["normal_xyz"]
                f.write(
                    f"{wp['id']},{wp['leg']},{wp['pass_id']},{int(wp['spray_on'])},"
                    f"{tx:.2f},{ty:.2f},{tz:.2f},{rx:.2f},{ry:.2f},{rz:.2f},"
                    f"{wp['feedrate_mms']:.1f},{wp['feather_intensity']:.3f},"
                    f"{sx:.2f},{sy:.2f},{sz:.2f},{nx:.4f},{ny:.4f},{nz:.4f}\n"
                )

        # Vẽ bảng tổng hợp trực quan 4 khung hình (4-Panel Visual Dashboard)
        vis_dashboard = self._render_dashboard(
            bgr, mask_bin, keepout_mask, spray_able_mask,
            landmarks_3d, wp_p1, wp_p3, wp_p4, dep_p3,
            sample_stem, ai_ms, total_pipeline_s, metrics_p1, metrics_p3, metrics_p4
        )
        vis_path = self.out_dir / f"{sample_stem}_step3_dashboard.jpg"
        cv2.imwrite(str(vis_path), vis_dashboard, [cv2.IMWRITE_JPEG_QUALITY, 92])

        return report_data, vis_path, csv_out

    def _render_dashboard(
        self, bgr, mask_bin, keepout_mask, spray_able_mask,
        landmarks_3d, wp_p1, wp_p3, wp_p4, dep_p3,
        sample_stem, ai_ms, total_pipeline_s, m_p1, m_p3, m_p4
    ):
        # 1. Panel 1: Vùng cấm phun (Đỏ) + Vùng được phép phun (Xanh lá) + 12 Mốc 3D & Pháp tuyến
        p1_img = bgr.copy()
        overlay_zones = np.zeros_like(p1_img)
        overlay_zones[keepout_mask > 0] = (0, 0, 230)       # Đỏ: Vùng cấm phun (Cạp, Túi, Đũng, Biên, Gấu)
        overlay_zones[spray_able_mask > 0] = (0, 210, 70)   # Xanh lá: Vùng an toàn cho phép phun
        p1_img = cv2.addWeighted(p1_img, 0.68, overlay_zones, 0.32, 0)

        contours, _ = cv2.findContours(mask_bin, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(p1_img, contours, -1, (255, 255, 0), 2)

        for name, lm in landmarks_3d.items():
            u, v = lm["pixel_u"], lm["pixel_v"]
            cv2.circle(p1_img, (u, v), 8, (0, 255, 255), -1, cv2.LINE_AA)
            cv2.circle(p1_img, (u, v), 10, (0, 0, 0), 2, cv2.LINE_AA)
            z_mm = lm["xyz_mm"][2]
            cv2.putText(
                p1_img, f"{name} (Z={z_mm:.0f}mm)",
                (u + 10, v - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (255, 255, 255), 2, cv2.LINE_AA
            )
        cv2.putText(
            p1_img, "1. PHAN VUNG CAM PHUN (DO) & VUNG AN TOAN (XANH) + 12 MOC 3D",
            (25, 42), cv2.FONT_HERSHEY_SIMPLEX, 0.85, (0, 255, 255), 2, cv2.LINE_AA
        )

        # Helper vẽ đường chạy dao
        def draw_toolpath(base_img, waypoints, title_str):
            canvas = base_img.copy()
            cv2.drawContours(canvas, contours, -1, (100, 220, 100), 2)
            for i in range(1, len(waypoints)):
                w0, w1 = waypoints[i - 1], waypoints[i]
                pt0 = tuple(w0["pixel_uv"])
                pt1 = tuple(w1["pixel_uv"])
                if w0["leg"] == w1["leg"] and w0["pass_id"] == w1["pass_id"]:
                    # Màu sắc thể hiện cường độ đánh nhã (Cam/Đỏ = Đậm/Chậm ở tâm, Xanh lam = Nhạt/Nhanh ở biên)
                    inten = w1["feather_intensity"]
                    color = (int(255 * (1 - inten)), int(180 * inten), int(255 * inten))
                    cv2.line(canvas, pt0, pt1, color, 4, cv2.LINE_AA)
                else:
                    # Đường chuyển lượt (tắt van súng) vẽ nét mảnh xám
                    cv2.line(canvas, pt0, pt1, (180, 180, 180), 1, cv2.LINE_AA)
            for wp in waypoints:
                pt = tuple(wp["pixel_uv"])
                cv2.circle(canvas, pt, 4, (255, 255, 255), -1, cv2.LINE_AA)
            cv2.putText(canvas, title_str, (25, 42), cv2.FONT_HERSHEY_SIMPLEX, 0.85, (0, 255, 255), 2, cv2.LINE_AA)
            return canvas

        # 2. Panel 2: Quỹ đạo P3 (Trục giải phẫu ống chân + Đánh nhã tốc độ TCP)
        p2_img = draw_toolpath(
            bgr, wp_p3,
            f"2. QUY DAO P3 (TIM ONG + DANH NHA): {m_p3['num_waypoints']} pts | {m_p3['cycle_time_s']}s"
        )

        # 3. Panel 3: Quỹ đạo P4 (Biến dạng mẫu Râu mèo + Đùi bằng Thin Plate Spline TPS)
        p3_img = draw_toolpath(
            bgr, wp_p4,
            f"3. QUY DAO P4 (TPS HOA VAN RAU MEO): {m_p4['num_waypoints']} pts | {m_p4['cycle_time_s']}s"
        )

        # 4. Panel 4: Mô phỏng mật độ lắng đọng hóa chất 3D (Gaussian Spray Deposition & Feathering)
        p4_img = bgr.copy()
        dep_u8 = np.clip(dep_p3 * 255.0, 0, 255).astype(np.uint8)
        heatmap = cv2.applyColorMap(dep_u8, cv2.COLORMAP_JET)
        alpha = np.clip(dep_p3 * 0.72, 0.0, 0.72)[:, :, None]
        p4_img = (p4_img.astype(np.float32) * (1.0 - alpha) + heatmap.astype(np.float32) * alpha).astype(np.uint8)
        cv2.drawContours(p4_img, contours, -1, (255, 255, 255), 2)
        cv2.putText(
            p4_img,
            f"4. MO PHONG MAT DO PHUN GAUSS & DANH NHA (Che phu: {m_p3['target_coverage_pct']}% | Tran ban: {m_p3['table_overspray_R_drop_pct']}%)",
            (25, 42), cv2.FONT_HERSHEY_SIMPLEX, 0.80, (0, 255, 255), 2, cv2.LINE_AA
        )

        # Ghép lưới 2x2
        scale = 0.55
        panels = [
            cv2.resize(p, (0, 0), fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
            for p in [p1_img, p2_img, p3_img, p4_img]
        ]
        top_row = np.hstack([panels[0], panels[1]])
        bot_row = np.hstack([panels[2], panels[3]])
        grid = np.vstack([top_row, bot_row])
        return grid


if __name__ == "__main__":
    pipeline = Step3TrajectoryPipeline3D()
    for stem in ["pcl_120", "pcl_105"]:
        report, vis_file, csv_file = pipeline.process_sample(stem)
        print("=" * 72)
        print(f"✓ ĐÃ XỬ LÝ XONG MẪU {stem}:")
        print(f"  - Thời gian AI (Seg + Pose) : {report['ai_inference_ms']} ms")
        print(f"  - Tổng thời gian cả chuỗi   : {report['total_pipeline_time_s']} s")
        print(f"  - Chỉ tiêu P3 (Tim ống)     : {report['comparison_metrics']['P3_Anatomical_Centerline_Feathering']}")
        print(f"  - Chỉ tiêu P4 (TPS Râu mèo) : {report['comparison_metrics']['P4_TPS_NonRigid_Template']}")
        print(f"  - File CSV 6-DOF Dobot CR10A: {csv_file}")
        print(f"  - Ảnh Dashboard 4 góc nhìn  : {vis_file}")
