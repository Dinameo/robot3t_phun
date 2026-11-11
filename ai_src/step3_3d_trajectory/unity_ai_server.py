#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
================================================================================
MÁY CHỦ AI & SINH QUỸ ĐẠO 3D 6-DOF KẾT NỐI UNITY SIMULATOR (DOBOT CR10A)
================================================================================
Đóng vai trò cầu nối trực tiếp (AI Backend Server) giữa Pipeline AI YOLO11 + 3D
Point Cloud và phần mềm mô phỏng Dobot CR10A Workbench trên Unity.

Hỗ trợ đồng thời 3 kênh giao tiếp chuẩn theo `AiServerClient.cs` của Unity:
  1. Kênh UDP Siêu Tốc (Cổng 5005 - Mặc định):
     - Nhận gói tin `"PING"` -> Phản hồi trạng thái trực tuyến `< 1 ms`.
     - Nhận lệnh JSON `{"command":"TRIGGER_DETECT","size":"M","sample":"pcl_120"}`
       -> Chạy AI YOLO11-seg + YOLO11-pose, chiếu 3D Zivid sang hệ quy chiếu
       Robot Dobot CR10A (khớp 100% với Camera Fixed Rig & Global Transform của
       Unity), giải Động học Nghịch 8 nhánh (Analytical IK), xuất file CSV chuẩn
       và trả gói JSON kết quả về cho Unity tự động nạp đám mây điểm + chạy mô phỏng!
  2. Kênh HTTP REST API (Cổng 5000):
     - `GET  http://127.0.0.1:5000/api/status`
     - `POST http://127.0.0.1:5000/api/detect`
  3. Kênh Tự Động Đẩy (Auto-Push Mode qua UDP Cổng 5006):
     - Cho phép đẩy trực tiếp bất kỳ mẫu phôi nào từ Python sang Unity mà không
       cần bấm nút trên giao diện Unity.
================================================================================
"""

import os
import sys
import json
import time
import math
import socket
import argparse
import threading
from http.server import BaseHTTPRequestHandler, HTTPServer
from pathlib import Path
import numpy as np

# Thêm đường dẫn tới Step 1 & Step 3
AI_SRC_DIR = Path("/home/nhan/RB3t_PJ/ai_src")
STEP1_DIR = AI_SRC_DIR / "step1_auto_label"
STEP3_DIR = AI_SRC_DIR / "step3_3d_trajectory"
sys.path.insert(0, str(STEP1_DIR))
sys.path.insert(0, str(STEP3_DIR))

from step3_3d_trajectory_generator import Step3TrajectoryPipeline3D
import cv2

# 12 điểm mốc giải phẫu chuẩn trên bản vẽ Quần Jean 2D Tiêu Chuẩn [0, 1] x [0, 1]
CANONICAL_LANDMARKS_12 = [
    ("waist_left",       "0: Eo trái",         0.28, 0.10),
    ("waist_center",     "1: Giữa eo",         0.50, 0.10),
    ("waist_right",      "2: Eo phải",         0.72, 0.10),
    ("crotch",           "3: Đáy đũng",        0.50, 0.34),
    ("thigh_left",       "4: Đùi trái",        0.35, 0.50),
    ("thigh_right",      "5: Đùi phải",        0.65, 0.50),
    ("knee_left",        "6: Gối trái",        0.33, 0.70),
    ("knee_right",       "7: Gối phải",        0.67, 0.70),
    ("cuff_left_outer",  "8: Gấu trái ngoài",  0.22, 0.91),
    ("cuff_left_inner",  "9: Gấu trái trong",  0.41, 0.91),
    ("cuff_right_inner", "10: Gấu phải trong", 0.59, 0.91),
    ("cuff_right_outer", "11: Gấu phải ngoài", 0.78, 0.91),
]


# ==============================================================================
# 1. BỘ CHUYỂN ĐỔI HỆ TỌA ĐỘ ZIVID -> UNITY WORLD -> DOBOT CR10A GLOBAL
#    VÀ BỘ GIẢI ĐỘNG HỌC NGHỊCH THUẦN GIẢI TÍCH (ĐỒNG BỘ 100% VỚI UNITY C#)
# ==============================================================================
class UnityDobotKinematicsBridge:
    """
    Đồng bộ chính xác hệ quy chiếu với `PLYDirectLoader.cs`, `SampleScene.unity`
    (`Global` Transform) và `DobotCR10AKinematics.cs` trong project Unity:
    
    1. Trong `PLYDirectLoader.cs` (dòng 335-336):
       Mỗi điểm đám mây điểm Zivid (vx, vy, vz) tính bằng mm được đưa vào Unity World (m):
         X_world = cameraOrigin.x + vx * 0.001   (cameraOrigin.x = 0.000 m)
         Y_world = cameraOrigin.y - vz * 0.001   (cameraOrigin.y = 1.685 m)
         Z_world = cameraOrigin.z - vy * 0.001   (cameraOrigin.z = 0.335 m)

    2. Trong `SampleScene.unity` (dòng 1834-1835):
       GameObject `Global` (gốc tọa độ Robot Dobot CR10A) đặt tại:
         localPosition = (0.0, 0.0, 1.1) m
         localRotation = Quaternion(-0.5, 0.5, 0.5, 0.5)  (Euler: X=-90°, Y=90°, Z=0°)
       Ma trận xoay R_global từ Local Robot sang Unity World biến đổi các trục đơn vị:
         - Trục X_robot (local 1,0,0) -> (0, 0, -1) trong Unity World
         - Trục Y_robot (local 0,1,0) -> (-1, 0, 0) trong Unity World
         - Trục Z_robot (local 0,0,1) -> (0, 1, 0)  trong Unity World (hướng thẳng lên trời)
       Do đó phép nghịch đảo `Global.InverseTransformPoint(P_world) * 1000` (mm) là:
         x_mm = (1.100 - Z_world) * 1000 = 765.0 + vy
         y_mm = (0.000 - X_world) * 1000 = -vx
         z_mm = (Y_world - 0.000) * 1000 = 1685.0 - vz
    """

    # Thông số D-H chuẩn của Dobot CR10A trong `DobotCR10AKinematics.cs`
    D1 = 176.5
    A2 = 607.0
    D2 = 193.0
    A3 = 568.0
    D3 = -127.0
    D4 = 125.0
    D5 = 125.0
    D6 = 111.0
    H = -(D2 + D3 + D4)  # -191.0 mm
    L2 = A2
    L3 = A3
    JOINT_WEIGHTS = [1.2, 1.2, 1.0, 0.8, 0.8, 0.5]

    @staticmethod
    def zivid_mm_to_robot_global_mm(vx_mm: float, vy_mm: float, vz_mm: float, z_contact_offset_mm: float = 22.0):
        """
        Chuyển điểm bề mặt quần (vx, vy, vz) từ hệ Camera Zivid (mm) sang hệ
        `targetPosGlobal_mm` (x_mm, y_mm, z_mm) của Robot Dobot CR10A trong Unity.
        - `z_contact_offset_mm`: Độ cao an toàn từ mặt vải/mặt bàn cho đầu công cụ TCP
          (mặc định ~22 mm khớp với `trajectory_pcl_101_p3.csv` trong Unity, đảm bảo
          Z >= 15 mm không bao giờ báo lỗi chạm bàn `lowZCount`).
        """
        x_robot_mm = 765.0 + float(vy_mm)
        y_robot_mm = -float(vx_mm)
        z_surface_mm = 1685.0 - float(vz_mm)
        z_robot_mm = max(18.0, z_surface_mm + float(z_contact_offset_mm))
        return x_robot_mm, y_robot_mm, z_robot_mm

    @staticmethod
    def wrap_pi(angle_rad: float) -> float:
        return math.atan2(math.sin(angle_rad), math.cos(angle_rad))

    @classmethod
    def dh_matrix(cls, theta: float, d: float, a: float, alpha: float) -> np.ndarray:
        ct, st = math.cos(theta), math.sin(theta)
        ca, sa = math.cos(alpha), math.sin(alpha)
        return np.array([
            [ct, -st * ca,  st * sa, a * ct],
            [st,  ct * ca, -ct * sa, a * st],
            [0.0,      sa,       ca,      d],
            [0.0,     0.0,      0.0,    1.0]
        ], dtype=np.float64)

    @classmethod
    def forward_flange_dh(cls, q_rad) -> np.ndarray:
        m1 = cls.dh_matrix(q_rad[0], cls.D1, 0.0, math.pi / 2.0)
        m2 = cls.dh_matrix(q_rad[1] + math.pi / 2.0, cls.D2, cls.A2, 0.0)
        m3 = cls.dh_matrix(q_rad[2], cls.D3, cls.A3, 0.0)
        m4 = cls.dh_matrix(q_rad[3] - math.pi / 2.0, cls.D4, 0.0, -math.pi / 2.0)
        m5 = cls.dh_matrix(q_rad[4], cls.D5, 0.0, math.pi / 2.0)
        m6 = cls.dh_matrix(q_rad[5], cls.D6, 0.0, 0.0)
        return m1 @ m2 @ m3 @ m4 @ m5 @ m6

    @classmethod
    def rpy_to_dh_rotation(cls, roll_deg: float, pitch_deg: float, yaw_deg: float) -> np.ndarray:
        cr, sr = math.cos(math.radians(roll_deg)), math.sin(math.radians(roll_deg))
        cp, sp = math.cos(math.radians(pitch_deg)), math.sin(math.radians(pitch_deg))
        cy, sy = math.cos(math.radians(yaw_deg)), math.sin(math.radians(yaw_deg))
        return np.array([
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [    -sp,                cp * sr,                cp * cr]
        ], dtype=np.float64)

    @classmethod
    def solve_ik(
        cls,
        target_unity_pos_mm,
        roll_deg: float = 180.0,
        pitch_deg: float = 0.0,
        yaw_deg: float = 0.0,
        prev_joints_rad=None,
        dtool_mm: float = 50.0,
        use_adaptive_tilt: bool = True,
    ):
        """
        Giải tích 8 cấu hình nghiệm IK khớp 100% với `DobotCR10AKinematics.SolveTcpPose` trong C#.
        """
        r_dh = cls.rpy_to_dh_rotation(roll_deg, pitch_deg, yaw_deg)
        p_target_dh = np.array([
            -float(target_unity_pos_mm[0]),
             float(target_unity_pos_mm[1]),
             float(target_unity_pos_mm[2]),
        ], dtype=np.float64)

        if use_adaptive_tilt:
            r_xy = math.hypot(p_target_dh[0], p_target_dh[1])
            if r_xy > 680.0:
                for tilt_deg in [0.0, 10.0, 18.0, 25.0, 32.0]:
                    alpha = math.radians(tilt_deg)
                    r_dh_tilt = r_dh.copy()
                    r_dh_tilt[0, 2] = (p_target_dh[0] / r_xy) * math.sin(alpha)
                    r_dh_tilt[1, 2] = (p_target_dh[1] / r_xy) * math.sin(alpha)
                    r_dh_tilt[2, 2] = -math.cos(alpha)
                    sols = cls._solve_internal(p_target_dh, r_dh_tilt, prev_joints_rad, dtool_mm)
                    if sols:
                        return sols[0]

        sols = cls._solve_internal(p_target_dh, r_dh, prev_joints_rad, dtool_mm)
        return sols[0] if sols else None

    @classmethod
    def _solve_internal(cls, p_target_dh, r_dh, prev_joints_rad, dtool_mm):
        solutions = []
        z6 = r_dh[:, 2]
        total_tool_d = cls.D6 + dtool_mm
        p05 = p_target_dh - total_tool_d * z6
        p06 = p_target_dh - dtool_mm * z6

        x5, y5, z5 = p05[0], p05[1], p05[2]
        delta1 = x5 * x5 + y5 * y5 - cls.H * cls.H
        if delta1 < -1e-6:
            return solutions

        sqrt_delta1 = math.sqrt(max(0.0, delta1))
        atan_ab = math.atan2(y5, -x5)

        t06 = np.eye(4, dtype=np.float64)
        t06[:3, :3] = r_dh
        t06[:3, 3] = p06

        for s1 in (1, -1):
            q1 = cls.wrap_pi(math.atan2(cls.H, s1 * sqrt_delta1) - atan_ab)
            cq1, sq1 = math.cos(q1), math.sin(q1)
            a1_inv = np.array([
                [cq1,  sq1, 0.0,     0.0],
                [0.0,  0.0, 1.0, -cls.D1],
                [sq1, -cq1, 0.0,     0.0],
                [0.0,  0.0, 0.0,     1.0]
            ], dtype=np.float64)

            t16 = a1_inv @ t06
            cos_q5 = max(-1.0, min(1.0, float(t16[2, 2])))
            sin_q5_mag = math.sqrt(max(0.0, 1.0 - cos_q5 * cos_q5))

            for s5 in (1, -1):
                sin_q5 = s5 * sin_q5_mag
                q5 = math.atan2(sin_q5, cos_q5)
                if abs(sin_q5) > 1e-4:
                    q6 = math.atan2(t16[2, 1] / sin_q5, -t16[2, 0] / sin_q5)
                    q234 = math.atan2(t16[1, 2] / sin_q5, t16[0, 2] / sin_q5)
                else:
                    q6 = 0.0
                    q234 = math.atan2(t16[1, 0], t16[0, 0])

                p15_x = a1_inv[0, 0] * x5 + a1_inv[0, 1] * y5
                p15_y = z5 - cls.D1

                p14_x = p15_x + cls.D5 * math.sin(q234)
                p14_y = p15_y - cls.D5 * math.cos(q234)

                rho14_sq = p14_x * p14_x + p14_y * p14_y
                cos_q3 = (rho14_sq - cls.L2 * cls.L2 - cls.L3 * cls.L3) / (2.0 * cls.L2 * cls.L3)
                if abs(cos_q3) > 1.0 + 1e-6:
                    continue

                cos_q3 = max(-1.0, min(1.0, cos_q3))
                sin_q3_mag = math.sqrt(max(0.0, 1.0 - cos_q3 * cos_q3))

                for s3 in (1, -1):
                    sin_q3 = s3 * sin_q3_mag
                    q3 = math.atan2(sin_q3, cos_q3)
                    k1 = cls.L2 + cls.L3 * cos_q3
                    k2 = cls.L3 * sin_q3
                    q2 = math.atan2(-p14_x, p14_y) - math.atan2(k2, k1)
                    q4 = q234 - q2 - q3

                    q_sol = [
                        cls.wrap_pi(q1),
                        cls.wrap_pi(q2),
                        cls.wrap_pi(q3),
                        cls.wrap_pi(q4),
                        cls.wrap_pi(q5),
                        cls.wrap_pi(q6),
                    ]

                    t_check = cls.forward_flange_dh(q_sol)
                    check_tcp = t_check[:3, 3] + dtool_mm * t_check[:3, 2]
                    pos_err = float(np.linalg.norm(p_target_dh - check_tcp))

                    cost = 0.0
                    if prev_joints_rad is not None:
                        for idx in range(6):
                            dq = cls.wrap_pi(q_sol[idx] - prev_joints_rad[idx])
                            cost += cls.JOINT_WEIGHTS[idx] * dq * dq
                    else:
                        # Ưu tiên cấu hình Khuỷu Trên và q2 dương khi khởi đầu
                        cost = abs(q_sol[0]) + abs(q_sol[1] - 0.6) + abs(q_sol[2] - 1.5)

                    solutions.append({
                        "joints_rad": q_sol,
                        "joints_deg": [math.degrees(v) for v in q_sol],
                        "pos_err_mm": pos_err,
                        "cost": cost,
                    })

        solutions.sort(key=lambda item: item["cost"])
        return solutions


# ==============================================================================
# 2. LÕI DỊCH VỤ AI SERVER QUẢN LÝ PIPELINE & XUẤT FILE CHO UNITY
# ==============================================================================
class DobotUnityAIServer:
    def __init__(
        self,
        unity_project_dir="/media/nhan/01DC3601B2514510/RB3t_PJ/dobot_cr10a",
        udp_port=5005,
        http_port=5000,
        auto_push_port=5006,
    ):
        self.unity_project_dir = Path(unity_project_dir)
        self.unity_output_dir = self.unity_project_dir / "output_drawing"
        self.ubuntu_output_dir = STEP3_DIR / "outputs"
        self.ubuntu_output_dir.mkdir(parents=True, exist_ok=True)

        self.udp_port = int(udp_port)
        self.http_port = int(http_port)
        self.auto_push_port = int(auto_push_port)

        print("=" * 76)
        print("🚀 ĐANG KHỞI ĐỘNG MÁY CHỦ AI DOBOT CR10A (YOLO11-SEG + YOLO11-POSE + 3D IK)")
        print("=" * 76)
        self.pipeline = Step3TrajectoryPipeline3D()
        self.lock = threading.Lock()
        print("✓ Đã nạp thành công 2 model AI (YOLO11s-seg & YOLO11s-pose)!")

    def _export_landmarks_and_segmentation_3d(self, clean_sample: str):
        """
        Trích xuất 12 Điểm Mốc Giải Phẫu 3D (Landmarks) + Đường Bao Segment Quần 3D (Garment Contour)
        + Đường Bao Vùng Cho Phép Phun 3D (Sprayable Zones) trong hệ tọa độ Robot Global (mm) của Unity.
        Lưu file nhẹ `<sample>_landmarks_seg.json` (~14 KB) để Unity hiển thị trực tiếp ở Tab Camera Trên Xuống.
        """
        ply_path = self.pipeline.core_3d.dataset_3d_dir / f"{clean_sample}.ply"
        sample_num = clean_sample.replace("pcl_", "")
        img_path = self.pipeline.core_3d.dataset_2d_dir / f"image_{sample_num}.png"

        xyz, bgr_ply, _ = self.pipeline.core_3d.load_organized_ply(ply_path)
        bgr = cv2.imread(str(img_path)) if img_path.exists() else bgr_ply
        mask_bin, keypoints_2d, _ = self.pipeline.run_ai_perception(bgr)
        plane_info = self.pipeline.core_3d.fit_table_plane(xyz)
        table_normal = np.array(plane_info["normal"], dtype=np.float64)
        if table_normal[2] > 0:
            table_normal = -table_normal

        spray_able_mask, _, _ = self.pipeline.build_spray_and_keepout_masks(mask_bin, keypoints_2d)

        # 1. 12 Điểm mốc giải phẫu 3D (khớp chính xác 100% với đỉnh 3D trong PLYDirectLoader.cs)
        landmarks_list = []
        for idx, (name, label_vi, can_u, can_v) in enumerate(CANONICAL_LANDMARKS_12):
            kp = keypoints_2d[name]
            pos_3d, normal_3d = self.pipeline.project_point_and_normal_3d(
                xyz, kp["pixel_u"], kp["pixel_v"], table_normal, radius_px=15
            )
            rx_mm = 765.0 + float(pos_3d[1])
            ry_mm = -float(pos_3d[0])
            rz_mm = (1685.0 - float(pos_3d[2])) + 2.0
            landmarks_list.append({
                "id": idx,
                "name": name,
                "label_vi": label_vi,
                "canonical_u": round(float(can_u), 4),
                "canonical_v": round(float(can_v), 4),
                "pixel_u": int(kp["pixel_u"]),
                "pixel_v": int(kp["pixel_v"]),
                "x_mm": round(float(rx_mm), 2),
                "y_mm": round(float(ry_mm), 2),
                "z_mm": round(float(rz_mm), 2),
                "nx": round(float(normal_3d[0]), 4),
                "ny": round(float(normal_3d[1]), 4),
                "nz": round(float(normal_3d[2]), 4),
            })

        # 2. Đường bao ngoài Segment thân quần 3D (Garment Contour)
        def extract_contour_3d(binary_mask, max_pts=160):
            contours, _ = cv2.findContours(binary_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
            if not contours:
                return []
            contours_sorted = sorted(contours, key=cv2.contourArea, reverse=True)
            result_loops = []
            for cnt in contours_sorted[:2]:
                if cv2.contourArea(cnt) < 1500:
                    continue
                pts = cnt[:, 0, :]
                step = max(1, len(pts) // max_pts)
                sampled = pts[::step]
                loop_3d = []
                for u_px, v_px in sampled:
                    p3d, _ = self.pipeline.project_point_and_normal_3d(
                        xyz, int(u_px), int(v_px), table_normal, radius_px=10
                    )
                    gx = 765.0 + float(p3d[1])
                    gy = -float(p3d[0])
                    gz = (1685.0 - float(p3d[2])) + 2.0
                    loop_3d.append({
                        "x_mm": round(float(gx), 2),
                        "y_mm": round(float(gy), 2),
                        "z_mm": round(float(gz), 2),
                    })
                if len(loop_3d) >= 3:
                    result_loops.append(loop_3d)
            return result_loops

        garment_loops = extract_contour_3d(mask_bin, max_pts=140)
        spray_loops = extract_contour_3d(spray_able_mask, max_pts=90)

        # 3. Tính kích thước vật lý từ 12 mốc và phân loại Size Quần (S / M / L) theo ngưỡng K-Means (K=3)
        def dist_xy(i_a, i_b):
            pa = landmarks_list[i_a]
            pb = landmarks_list[i_b]
            return math.hypot(pa["x_mm"] - pb["x_mm"], pa["y_mm"] - pb["y_mm"])

        outseam_mm = 0.5 * (dist_xy(0, 8) + dist_xy(2, 11))
        inseam_mm = 0.5 * (dist_xy(3, 9) + dist_xy(3, 10))
        waist_mm = dist_xy(0, 2)
        cuff_mm = 0.5 * (dist_xy(8, 9) + dist_xy(10, 11))

        # Diện tích đa giác 8 mốc biên ngoài (0->1->2->11->10->3->9->8) đơn vị cm2
        poly_indices = [0, 1, 2, 11, 10, 3, 9, 8]
        px = np.array([landmarks_list[i]["x_mm"] for i in poly_indices], dtype=np.float64)
        py = np.array([landmarks_list[i]["y_mm"] for i in poly_indices], dtype=np.float64)
        area_cm2 = float(0.5 * abs(np.dot(px, np.roll(py, 1)) - np.dot(py, np.roll(px, 1))) / 100.0)

        # Ngưỡng K-Means (K=3) đã xác định trên 300 mẫu:
        # Inseam: T1 = 618.1 mm, T2 = 667.1 mm | Outseam: T1 = 933.7 mm, T2 = 984.3 mm
        if inseam_mm < 618.1 or outseam_mm < 933.7:
            detected_size = "S"
        elif inseam_mm < 667.1:
            detected_size = "M"
        else:
            detected_size = "L"

        seg_payload = {
            "sample": clean_sample,
            "detected_size": detected_size,
            "outseam_mm": round(float(outseam_mm), 1),
            "inseam_mm": round(float(inseam_mm), 1),
            "waist_mm": round(float(waist_mm), 1),
            "cuff_mm": round(float(cuff_mm), 1),
            "area_cm2": round(float(area_cm2), 1),
            "landmarks": landmarks_list,
            "garment_contour_3d": garment_loops[0] if garment_loops else [],
            "spray_zone_left_3d": spray_loops[0] if len(spray_loops) > 0 else [],
            "spray_zone_right_3d": spray_loops[1] if len(spray_loops) > 1 else [],
        }

        json_text = json.dumps(seg_payload, ensure_ascii=False, indent=2)
        ubuntu_seg_path = self.ubuntu_output_dir / f"{clean_sample}_landmarks_seg.json"
        with open(ubuntu_seg_path, "w", encoding="utf-8") as f:
            f.write(json_text)
        if clean_sample == "pcl_25":
            with open(self.ubuntu_output_dir / "full_cloud_world_landmarks_seg.json", "w", encoding="utf-8") as f:
                f.write(json_text)

        try:
            if self.unity_output_dir.exists():
                unity_seg_path = self.unity_output_dir / f"{clean_sample}_landmarks_seg.json"
                with open(unity_seg_path, "w", encoding="utf-8") as f:
                    f.write(json_text)
                if clean_sample == "pcl_25":
                    with open(self.unity_output_dir / "full_cloud_world_landmarks_seg.json", "w", encoding="utf-8") as f:
                        f.write(json_text)
        except OSError:
            pass

        return seg_payload, xyz, mask_bin, spray_able_mask, keypoints_2d, table_normal

    def run_custom_draw_3d_for_unity(
        self,
        sample_name: str = "pcl_101",
        strokes: list = None,
        z_offset_mm: float = 22.0,
    ):
        """
        Nhận các nét vẽ 2D của người dùng trên chiếc quần tiêu chuẩn [0, 1] x [0, 1] từ Unity,
        biến dạng phi cứng bằng Thin Plate Spline (TPS) qua 12 điểm mốc giải phẫu AI của mẫu
        `sample_name`, chiếu trực tiếp lên bề mặt 3D Zivid Point Cloud (X, Y, Z, vector pháp tuyến),
        giải Động học Nghịch 6-DOF Dobot CR10A và xuất file CSV quỹ đạo 3D cho Unity!
        """
        with self.lock:
            t0 = time.perf_counter()
            clean_sample = Path(sample_name).stem.strip()
            if clean_sample == "full_cloud_world":
                clean_sample = "pcl_25"
            elif not clean_sample.startswith("pcl_"):
                clean_sample = f"pcl_{clean_sample}"

            ply_path = self.pipeline.core_3d.dataset_3d_dir / f"{clean_sample}.ply"
            if not ply_path.exists():
                return {
                    "status": "ERROR",
                    "message": f"Không tìm thấy mẫu đám mây điểm {ply_path}",
                    "sample": clean_sample,
                    "error_code": 404,
                }

            # Lấy dữ liệu 3D & 12 mốc giải phẫu của mẫu hiện hành
            _, xyz, mask_bin, spray_able_mask, keypoints_2d, table_normal = (
                self._export_landmarks_and_segmentation_3d(clean_sample)
            )

            # Xây dựng phép ánh xạ phi cứng Thin Plate Spline (TPS) từ 12 mốc Quần 2D Tiêu Chuẩn -> 12 mốc Quần Thực Tế
            src_ctrl = np.array([[c[2], c[3]] for c in CANONICAL_LANDMARKS_12], dtype=np.float64)
            dst_ctrl = np.array(
                [[keypoints_2d[c[0]]["pixel_u"], keypoints_2d[c[0]]["pixel_v"]] for c in CANONICAL_LANDMARKS_12],
                dtype=np.float64,
            )

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

            def warp_canonical_to_pixel(can_uv_array):
                d = can_uv_array[:, None, :] - src_ctrl[None, :, :]
                dist2 = np.sum(d ** 2, axis=-1)
                U = np.where(dist2 > 1e-12, dist2 * np.log(dist2 + 1e-12), 0.0)
                P_q = np.column_stack([np.ones(len(can_uv_array)), can_uv_array])
                return U @ weights_tps[:n_pts] + P_q @ weights_tps[n_pts:]

            # Chuyển đổi toàn bộ các nét vẽ (strokes) của người dùng sang 3D Waypoints
            h_img, w_img = mask_bin.shape
            unity_rows = []
            prev_q_rad = None
            prev_pos_mm = None
            cum_dist_mm = 0.0
            wp_idx = 0

            if not strokes:
                strokes = []

            for s_idx, stroke in enumerate(strokes):
                if not stroke or len(stroke) == 0:
                    continue
                can_pts = np.array([[float(pt["u"]), float(pt["v"])] for pt in stroke], dtype=np.float64)
                warped_px = warp_canonical_to_pixel(can_pts)

                for p_i, (u_f, v_f) in enumerate(warped_px):
                    u_px = int(np.clip(round(u_f), 0, w_img - 1))
                    v_px = int(np.clip(round(v_f), 0, h_img - 1))

                    # Chiếu trực tiếp lên bề mặt 3D Zivid Point Cloud (X, Y, Z, vector pháp tuyến vi mô)
                    pos_3d, normal_3d = self.pipeline.project_point_and_normal_3d(
                        xyz, u_px, v_px, table_normal, radius_px=12
                    )
                    x_mm, y_mm, z_mm = UnityDobotKinematicsBridge.zivid_mm_to_robot_global_mm(
                        pos_3d[0], pos_3d[1], pos_3d[2], z_contact_offset_mm=float(z_offset_mm)
                    )
                    pos_mm = np.array([x_mm, y_mm, z_mm], dtype=np.float64)

                    r_norm = float(np.linalg.norm(pos_mm))
                    if r_norm > 1150.0:
                        scale_r = 1140.0 / r_norm
                        pos_mm[0] *= scale_r
                        pos_mm[1] *= scale_r
                        x_mm, y_mm = float(pos_mm[0]), float(pos_mm[1])

                    if prev_pos_mm is not None:
                        cum_dist_mm += float(np.linalg.norm(pos_mm - prev_pos_mm))
                    prev_pos_mm = pos_mm

                    # Tính góc nghiêng bề mặt vi mô từ pháp tuyến 3D của đám mây điểm
                    pitch_surface = float(np.clip(math.degrees(math.asin(np.clip(normal_3d[1], -0.35, 0.35))), -18.0, 18.0))
                    roll_deg, pitch_deg, yaw_deg = 180.0, round(pitch_surface * 0.35, 2), 0.0

                    ik_sol = UnityDobotKinematicsBridge.solve_ik(
                        pos_mm,
                        roll_deg=roll_deg,
                        pitch_deg=pitch_deg,
                        yaw_deg=yaw_deg,
                        prev_joints_rad=prev_q_rad,
                        dtool_mm=50.0,
                        use_adaptive_tilt=True,
                    )
                    if ik_sol is None:
                        # Fallback về bích chúc thẳng (180, 0, 0) nếu góc nghiêng vi mô vượt tầm
                        roll_deg, pitch_deg, yaw_deg = 180.0, 0.0, 0.0
                        ik_sol = UnityDobotKinematicsBridge.solve_ik(
                            pos_mm,
                            roll_deg=roll_deg,
                            pitch_deg=pitch_deg,
                            yaw_deg=yaw_deg,
                            prev_joints_rad=prev_q_rad,
                            dtool_mm=50.0,
                            use_adaptive_tilt=True,
                        )

                    if ik_sol is not None:
                        j_deg = ik_sol["joints_deg"]
                        j_rad = ik_sol["joints_rad"]
                        err_mm = ik_sol["pos_err_mm"]
                        status_str = "SUCCESS"
                        prev_q_rad = j_rad
                    else:
                        j_deg = [0.0] * 6
                        j_rad = [0.0] * 6
                        err_mm = 999.0
                        status_str = "FAILED_IK"

                    unity_rows.append({
                        "index": wp_idx,
                        "dist_mm": cum_dist_mm,
                        "x_mm": x_mm,
                        "y_mm": y_mm,
                        "z_mm": z_mm,
                        "roll_deg": roll_deg,
                        "pitch_deg": pitch_deg,
                        "yaw_deg": yaw_deg,
                        "j_deg": j_deg,
                        "j_rad": j_rad,
                        "err_pos_mm": err_mm,
                        "spray_enabled": 1 if p_i > 0 else 0,
                        "segment_name": f"CUSTOM3D_STROKE_{s_idx + 1}",
                        "status": status_str,
                        "feedrate_mms": 220.0,
                        "feather_intensity": 1.0,
                    })
                    wp_idx += 1

            csv_header = (
                "index,dist_mm,x_mm,y_mm,z_mm,roll_deg,pitch_deg,yaw_deg,"
                "j1_deg,j2_deg,j3_deg,j4_deg,j5_deg,j6_deg,"
                "j1_rad,j2_rad,j3_rad,j4_rad,j5_rad,j6_rad,"
                "err_pos_mm,spray_enabled,segment_name,status,feedrate_mms,feather_intensity\n"
            )
            csv_lines = [csv_header]
            for r in unity_rows:
                jd = r["j_deg"]
                jr = r["j_rad"]
                line = (
                    f"{r['index']},{r['dist_mm']:.3f},"
                    f"{r['x_mm']:.3f},{r['y_mm']:.3f},{r['z_mm']:.3f},"
                    f"{r['roll_deg']:.4f},{r['pitch_deg']:.4f},{r['yaw_deg']:.4f},"
                    f"{jd[0]:.5f},{jd[1]:.5f},{jd[2]:.5f},{jd[3]:.5f},{jd[4]:.5f},{jd[5]:.5f},"
                    f"{jr[0]:.8f},{jr[1]:.8f},{jr[2]:.8f},{jr[3]:.8f},{jr[4]:.8f},{jr[5]:.8f},"
                    f"{r['err_pos_mm']:.5f},{r['spray_enabled']},{r['segment_name']},{r['status']},"
                    f"{r['feedrate_mms']:.1f},{r['feather_intensity']:.3f}\n"
                )
                csv_lines.append(line)
            csv_content = "".join(csv_lines)

            ubuntu_csv_path = self.ubuntu_output_dir / f"trajectory_{clean_sample}_custom3d.csv"
            with open(ubuntu_csv_path, "w", encoding="utf-8") as f:
                f.write(csv_content)

            trajectory_csv_for_unity = str(ubuntu_csv_path)
            try:
                if self.unity_output_dir.exists():
                    unity_csv_name = f"trajectory_{clean_sample}_custom3d.csv"
                    unity_csv_path = self.unity_output_dir / unity_csv_name
                    with open(unity_csv_path, "w", encoding="utf-8") as f:
                        f.write(csv_content)
                    trajectory_csv_for_unity = f"output_drawing/{unity_csv_name}"
            except OSError:
                trajectory_csv_for_unity = str(ubuntu_csv_path)

            elapsed_s = round(time.perf_counter() - t0, 3)
            response = {
                "status": "SUCCESS",
                "message": f"Đã ánh xạ nét vẽ 2D -> 3D Point Cloud ({len(unity_rows)} điểm) trên {clean_sample}",
                "garment_size": "CUSTOM_3D",
                "sample": clean_sample,
                "pointcloud_ply": str(ply_path),
                "waypoints_count": len(unity_rows),
                "perimeter_mm": round(cum_dist_mm, 1),
                "time_generation_s": elapsed_s,
                "trajectory_csv": trajectory_csv_for_unity,
                "preview_image": "",
                "error_code": 0,
            }
            print(
                f"🎨 [CUSTOM 2D->3D] Đã ánh xạ {len(strokes)} nét vẽ -> {len(unity_rows)} waypoints 3D "
                f"trên [{clean_sample}] ({cum_dist_mm:.1f} mm) trong {elapsed_s:.2f}s!"
            )
            return response

    def run_detection_for_unity(self, sample_name: str = "pcl_101", garment_size: str = "M", mode: str = "P3"):
        """
        Thực thi trọn vẹn chu trình nhận dạng & sinh quỹ đạo 3D cho mẫu `sample_name`,
        lưu file CSV tương thích 100% với `DobotCR10AIKController.LoadTrajectoryFile()`
        và trả về Dictionary JSON chuẩn theo class `AiDetectionResult` trong `AiServerClient.cs`.
        """
        with self.lock:
            t0 = time.perf_counter()
            clean_sample = Path(sample_name).stem.strip()
            if clean_sample == "full_cloud_world":
                clean_sample = "pcl_25"
            elif not clean_sample.startswith("pcl_"):
                clean_sample = f"pcl_{clean_sample}"

            ply_path = self.pipeline.core_3d.dataset_3d_dir / f"{clean_sample}.ply"
            if not ply_path.exists():
                return {
                    "status": "ERROR",
                    "message": f"Không tìm thấy file đám mây điểm {ply_path}",
                    "garment_size": garment_size,
                    "sample": clean_sample,
                    "pointcloud_ply": "",
                    "waypoints_count": 0,
                    "perimeter_mm": 0.0,
                    "time_generation_s": 0.0,
                    "trajectory_csv": "",
                    "preview_image": "",
                    "error_code": 404,
                }

            # Chạy pipeline AI + 3D & xuất đồng thời 12 mốc + đường bao segment 3D cho Unity
            report_data, vis_path, _ = self.pipeline.process_sample(clean_sample)
            seg_payload, _, _, _, _, _ = self._export_landmarks_and_segmentation_3d(clean_sample)
            detected_size = seg_payload.get("detected_size", "S")
            outseam_mm = seg_payload.get("outseam_mm", 0.0)
            inseam_mm = seg_payload.get("inseam_mm", 0.0)
            waist_mm = seg_payload.get("waist_mm", 0.0)
            area_cm2 = seg_payload.get("area_cm2", 0.0)

            # Lấy danh sách waypoints 3D (mặc định P3 - Tim ống + Đánh nhã)
            waypoints_raw = report_data["recommended_trajectory_P3_waypoints"]

            # Chuyển đổi tọa độ Zivid Camera (mm) sang hệ Robot Global của Unity (mm)
            # và giải Động học Nghịch 6 khớp (j1..j6) khớp hoàn toàn với Unity
            unity_rows = []
            prev_q_rad = None
            prev_pos_mm = None
            cum_dist_mm = 0.0

            for idx, wp in enumerate(waypoints_raw):
                sx, sy, sz = wp["surface_xyz_mm"]
                x_mm, y_mm, z_mm = UnityDobotKinematicsBridge.zivid_mm_to_robot_global_mm(
                    sx, sy, sz, z_contact_offset_mm=22.0
                )
                pos_mm = np.array([x_mm, y_mm, z_mm], dtype=np.float64)

                # Kiểm tra giới hạn bán kính an toàn của Dobot CR10A (R <= 1150 mm)
                r_norm = float(np.linalg.norm(pos_mm))
                if r_norm > 1150.0:
                    scale_r = 1140.0 / r_norm
                    pos_mm[0] *= scale_r
                    pos_mm[1] *= scale_r
                    x_mm, y_mm = float(pos_mm[0]), float(pos_mm[1])

                if prev_pos_mm is not None:
                    cum_dist_mm += float(np.linalg.norm(pos_mm - prev_pos_mm))
                prev_pos_mm = pos_mm

                # Hướng mặt bích chuẩn trong Unity: Roll=180°, Pitch=0°, Yaw=0° (Flange-Down)
                roll_deg, pitch_deg, yaw_deg = 180.0, 0.0, 0.0
                ik_sol = UnityDobotKinematicsBridge.solve_ik(
                    pos_mm,
                    roll_deg=roll_deg,
                    pitch_deg=pitch_deg,
                    yaw_deg=yaw_deg,
                    prev_joints_rad=prev_q_rad,
                    dtool_mm=50.0,
                    use_adaptive_tilt=True,
                )

                if ik_sol is not None:
                    j_deg = ik_sol["joints_deg"]
                    j_rad = ik_sol["joints_rad"]
                    err_mm = ik_sol["pos_err_mm"]
                    status_str = "SUCCESS"
                    prev_q_rad = j_rad
                else:
                    j_deg = [0.0] * 6
                    j_rad = [0.0] * 6
                    err_mm = 999.0
                    status_str = "FAILED_IK"

                # Điểm đầu tiên (idx=0) hoặc bước nhảy chuyển tiếp giữa 2 ống/2 pass là di chuyển chuyển tiếp -> TẮT PHUN SƠN (spray_enabled = 0)
                is_spray = 1 if wp.get("spray_on", True) else 0
                if idx == 0:
                    is_spray = 0
                elif idx > 0:
                    prev_raw = waypoints_raw[idx - 1]
                    dist_jump = float(np.linalg.norm(np.array(wp["surface_xyz_mm"]) - np.array(prev_raw["surface_xyz_mm"])))
                    leg_diff = wp.get("leg") != prev_raw.get("leg")
                    pass_diff = wp.get("pass_id") != prev_raw.get("pass_id")
                    if leg_diff or pass_diff or dist_jump > 35.0:
                        is_spray = 0

                unity_rows.append({
                    "index": idx,
                    "dist_mm": cum_dist_mm,
                    "x_mm": x_mm,
                    "y_mm": y_mm,
                    "z_mm": z_mm,
                    "roll_deg": roll_deg,
                    "pitch_deg": pitch_deg,
                    "yaw_deg": yaw_deg,
                    "j_deg": j_deg,
                    "j_rad": j_rad,
                    "err_pos_mm": err_mm,
                    "spray_enabled": is_spray,
                    "segment_name": f"P3_{wp.get('leg', 'left').upper()}_PASS{wp.get('pass_id', 0)}",
                    "status": status_str,
                    "feedrate_mms": wp.get("feedrate_mms", 250.0),
                    "feather_intensity": wp.get("feather_intensity", 1.0),
                })

            # Lưu file CSV định dạng Unity (nhẹ chỉ ~18 KB)
            csv_header = (
                "index,dist_mm,x_mm,y_mm,z_mm,roll_deg,pitch_deg,yaw_deg,"
                "j1_deg,j2_deg,j3_deg,j4_deg,j5_deg,j6_deg,"
                "j1_rad,j2_rad,j3_rad,j4_rad,j5_rad,j6_rad,"
                "err_pos_mm,spray_enabled,segment_name,status,feedrate_mms,feather_intensity\n"
            )
            csv_lines = [csv_header]
            for r in unity_rows:
                jd = r["j_deg"]
                jr = r["j_rad"]
                line = (
                    f"{r['index']},{r['dist_mm']:.3f},"
                    f"{r['x_mm']:.3f},{r['y_mm']:.3f},{r['z_mm']:.3f},"
                    f"{r['roll_deg']:.4f},{r['pitch_deg']:.4f},{r['yaw_deg']:.4f},"
                    f"{jd[0]:.5f},{jd[1]:.5f},{jd[2]:.5f},{jd[3]:.5f},{jd[4]:.5f},{jd[5]:.5f},"
                    f"{jr[0]:.8f},{jr[1]:.8f},{jr[2]:.8f},{jr[3]:.8f},{jr[4]:.8f},{jr[5]:.8f},"
                    f"{r['err_pos_mm']:.5f},{r['spray_enabled']},{r['segment_name']},{r['status']},"
                    f"{r['feedrate_mms']:.1f},{r['feather_intensity']:.3f}\n"
                )
                csv_lines.append(line)
            csv_content = "".join(csv_lines)

            # 1. Lưu trên phân vùng Ubuntu (/home/nhan/RB3t_PJ/ai_src/step3_3d_trajectory/outputs/)
            ubuntu_csv_path = self.ubuntu_output_dir / f"trajectory_{clean_sample}_p3.csv"
            with open(ubuntu_csv_path, "w", encoding="utf-8") as f:
                f.write(csv_content)

            # 2. Đồng bộ bản CSV nhẹ (~18 KB) sang output_drawing/ của Unity nếu còn chỗ trống
            trajectory_csv_for_unity = str(ubuntu_csv_path)
            try:
                if self.unity_output_dir.exists():
                    unity_csv_name = f"trajectory_{clean_sample}_p3.csv"
                    unity_csv_path = self.unity_output_dir / unity_csv_name
                    with open(unity_csv_path, "w", encoding="utf-8") as f:
                        f.write(csv_content)
                    trajectory_csv_for_unity = f"output_drawing/{unity_csv_name}"
            except OSError:
                # Nếu ổ NTFS đầy, truyền trực tiếp đường dẫn tuyệt đối trên Ubuntu (Unity hỗ trợ Path.IsPathRooted)
                trajectory_csv_for_unity = str(ubuntu_csv_path)

            elapsed_s = round(time.perf_counter() - t0, 3)

            response = {
                "status": "SUCCESS",
                "message": f"AI YOLO11 + 3D P3 hoàn tất cho {clean_sample} (Size {detected_size})",
                "garment_size": detected_size,
                "outseam_mm": outseam_mm,
                "inseam_mm": inseam_mm,
                "waist_mm": waist_mm,
                "area_cm2": area_cm2,
                "sample": clean_sample,
                "pointcloud_ply": str(ply_path),
                "waypoints_count": len(unity_rows),
                "perimeter_mm": round(cum_dist_mm, 1),
                "time_generation_s": elapsed_s,
                "trajectory_csv": trajectory_csv_for_unity,
                "preview_image": str(vis_path),
                "error_code": 0,
            }

            print(
                f"🟢 [AI SERVER] Đã xử lý mẫu [{clean_sample}] -> NHẬN DIỆN SIZE: [{detected_size}] "
                f"(Inseam={inseam_mm:.1f}mm, Outseam={outseam_mm:.1f}mm, Eo={waist_mm:.1f}mm) | "
                f"{len(unity_rows)} waypoints | Quãng đường: {cum_dist_mm:.1f} mm | "
                f"Thời gian: {elapsed_s:.2f}s -> CSV: {trajectory_csv_for_unity}"
            )
            return response

    def push_auto_result_to_unity(self, result_dict: dict, target_ip: str = "127.0.0.1"):
        """
        Đẩy gói tin JSON kết quả trực tiếp vào cổng UDP 5006 của Unity (`AutoListenWorker` trong `AiServerClient.cs`).
        """
        try:
            payload = json.dumps(result_dict, ensure_ascii=False).encode("utf-8")
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
                sock.sendto(payload, (target_ip, self.auto_push_port))
            print(f"📡 [AUTO-PUSH UDP:{self.auto_push_port}] Đã đẩy tự động mẫu [{result_dict.get('sample')}] vào Unity!")
        except Exception as e:
            print(f"⚠️ [AUTO-PUSH LỖI] Không thể đẩy gói UDP 5006: {e}")

    # --------------------------------------------------------------------------
    # KÊNH 1: UDP SERVER (CỔNG 5005 - TỐC ĐỘ < 1MS)
    # --------------------------------------------------------------------------
    def start_udp_server(self):
        def udp_worker():
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            sock.bind(("0.0.0.0", self.udp_port))
            print(f"🎧 [UDP SERVER] Đang lắng nghe Unity tại 0.0.0.0:{self.udp_port} ...")

            while True:
                try:
                    data, addr = sock.recvfrom(65535)
                    msg = data.decode("utf-8", errors="ignore").strip()
                    if not msg:
                        continue

                    # 1. Xử lý bản tin kiểm tra kết nối "PING" từ Unity
                    if msg.upper() == "PING":
                        pong_reply = json.dumps({
                            "status": "ONLINE",
                            "service": "Dobot_CR10A_YOLO11_3D_Server",
                            "version": "2.0",
                            "model_ready": True,
                            "protocol": "UDP",
                        })
                        sock.sendto(pong_reply.encode("utf-8"), addr)
                        continue

                    # 2. Xử lý lệnh JSON {"command":"TRIGGER_DETECT" | "CUSTOM_DRAW_3D", ...}
                    req = json.loads(msg)
                    cmd = req.get("command", "TRIGGER_DETECT")
                    sample = req.get("sample", "pcl_101")
                    size = req.get("size", "M")
                    print(f"⚡ [UDP NHẬN LỆNH] Từ {addr}: command={cmd}, sample={sample}, size={size}")

                    if cmd == "CUSTOM_DRAW_3D":
                        strokes = req.get("strokes", [])
                        z_off = float(req.get("z_offset_mm", 22.0))
                        res = self.run_custom_draw_3d_for_unity(
                            sample_name=sample, strokes=strokes, z_offset_mm=z_off
                        )
                    else:
                        res = self.run_detection_for_unity(sample_name=sample, garment_size=size)
                    reply_bytes = json.dumps(res, ensure_ascii=False).encode("utf-8")
                    sock.sendto(reply_bytes, addr)

                except Exception as ex:
                    err_reply = json.dumps({
                        "status": "ERROR",
                        "message": str(ex),
                        "error_code": 500,
                    }).encode("utf-8")
                    try:
                        sock.sendto(err_reply, addr)
                    except Exception:
                        pass
                    print(f"❌ [UDP LỖI] {ex}")

        t = threading.Thread(target=udp_worker, daemon=True, name="UDP_5005_Thread")
        t.start()
        return t

    # --------------------------------------------------------------------------
    # KÊNH 2: HTTP REST SERVER (CỔNG 5000 - /api/status & /api/detect)
    # --------------------------------------------------------------------------
    def start_http_server(self):
        server_ref = self

        class RequestHandler(BaseHTTPRequestHandler):
            def _send_json(self, status_code: int, payload: dict):
                body = json.dumps(payload, ensure_ascii=False).encode("utf-8")
                self.send_response(status_code)
                self.send_header("Content-Type", "application/json; charset=utf-8")
                self.send_header("Content-Length", str(len(body)))
                self.send_header("Access-Control-Allow-Origin", "*")
                self.end_headers()
                self.wfile.write(body)

            def do_GET(self):
                if self.path.startswith("/api/status"):
                    self._send_json(200, {
                        "status": "ONLINE",
                        "service": "Dobot_CR10A_YOLO11_3D_Server",
                        "version": "2.0",
                        "model_ready": True,
                        "protocol": "HTTP",
                    })
                else:
                    self._send_json(404, {"status": "ERROR", "message": "Endpoint not found"})

            def do_POST(self):
                if self.path.startswith("/api/detect") or self.path.startswith("/api/custom_draw_3d"):
                    length = int(self.headers.get("Content-Length", 0))
                    raw_body = self.rfile.read(length).decode("utf-8", errors="ignore") if length > 0 else "{}"
                    try:
                        req = json.loads(raw_body)
                    except Exception:
                        req = {}
                    cmd = req.get("command", "CUSTOM_DRAW_3D" if "custom_draw" in self.path else "TRIGGER_DETECT")
                    sample = req.get("sample", "pcl_101")
                    size = req.get("size", "M")
                    if cmd == "CUSTOM_DRAW_3D":
                        strokes = req.get("strokes", [])
                        z_off = float(req.get("z_offset_mm", 22.0))
                        print(f"🎨 [HTTP POST CUSTOM_DRAW_3D] sample={sample}, strokes={len(strokes)}")
                        res = server_ref.run_custom_draw_3d_for_unity(
                            sample_name=sample, strokes=strokes, z_offset_mm=z_off
                        )
                    else:
                        print(f"⚡ [HTTP POST /api/detect] sample={sample}, size={size}")
                        res = server_ref.run_detection_for_unity(sample_name=sample, garment_size=size)
                    server_ref.push_auto_result_to_unity(res)
                    self._send_json(200, res)
                else:
                    self._send_json(404, {"status": "ERROR", "message": "Endpoint not found"})

            def log_message(self, format, *args):
                # Tắt log mặc định rườm rà của BaseHTTPRequestHandler
                pass

        class ReusableHTTPServer(HTTPServer):
            allow_reuse_address = True

        def http_worker():
            try:
                httpd = ReusableHTTPServer(("0.0.0.0", self.http_port), RequestHandler)
                print(f"🌐 [HTTP SERVER] Đang lắng nghe REST API tại http://0.0.0.0:{self.http_port}/api/status & /api/detect ...")
                httpd.serve_forever()
            except OSError as e:
                print(f"⚠️ [HTTP SERVER] Cổng {self.http_port} đã được mở bởi tiến trình trước ({e}). Tiếp tục dùng kênh UDP {self.udp_port}.")

        t = threading.Thread(target=http_worker, daemon=True, name="HTTP_5000_Thread")
        t.start()
        return t


def main():
    parser = argparse.ArgumentParser(description="AI Server kết nối Unity Dobot CR10A Workbench")
    parser.add_argument("--udp-port", type=int, default=5005, help="Cổng UDP chính (mặc định: 5005)")
    parser.add_argument("--http-port", type=int, default=5000, help="Cổng HTTP REST (mặc định: 5000)")
    parser.add_argument("--auto-port", type=int, default=5006, help="Cổng UDP Auto-Push (mặc định: 5006)")
    parser.add_argument("--warmup-sample", type=str, default="pcl_101", help="Mẫu chạy làm nóng và đẩy thử vào Unity khi khởi động")
    parser.add_argument("--test-only", action="store_true", help="Chỉ chạy kiểm thử 1 mẫu rồi thoát")
    args = parser.parse_args()

    server = DobotUnityAIServer(
        udp_port=args.udp_port,
        http_port=args.http_port,
        auto_push_port=args.auto_port,
    )

    if args.warmup_sample:
        print(f"\n🔥 Đang chạy Warm-up & đồng bộ sẵn quỹ đạo mẫu [{args.warmup_sample}] cho Unity...")
        res = server.run_detection_for_unity(sample_name=args.warmup_sample, garment_size="M")
        server.push_auto_result_to_unity(res)

    if args.test_only:
        print("\n✅ Hoàn tất chế độ --test-only.")
        return

    server.start_udp_server()
    server.start_http_server()

    print("\n" + "=" * 76)
    print("✅ AI SERVER ĐÃ SẴN SÀNG KẾT NỐI VỚI UNITY!")
    print("   - Trong Unity (Tab QUỸ ĐẠO hoặc CAM & ĐÁM MÂY ĐIỂM):")
    print("     + Địa chỉ IP : 127.0.0.1")
    print("     + Cổng UDP   : 5005 (hoặc HTTP cổng 5000)")
    print("     + Bấm nút '⚡ KÍCH HOẠT AI [pcl_...]' hoặc bật 'CHẾ ĐỘ TỰ ĐỘNG (UDP 5006)'")
    print("   - Bạn cũng có thể gõ tên mẫu (ví dụ: pcl_120, pcl_105, pcl_151) ngay tại")
    print("     cửa sổ Terminal này rồi nhấn Enter để đẩy trực tiếp vào Unity!")
    print("=" * 76)

    try:
        while True:
            user_in = input("\n👉 Nhập tên mẫu để đẩy trực tiếp sang Unity (ví dụ: pcl_120) hoặc 'q' để thoát: ").strip()
            if not user_in:
                continue
            if user_in.lower() in ("q", "quit", "exit"):
                print("Đang dừng AI Server...")
                break
            res = server.run_detection_for_unity(sample_name=user_in, garment_size="M")
            server.push_auto_result_to_unity(res)
    except (EOFError, KeyboardInterrupt):
        # Nếu chạy ngầm (daemon / nohup không có stdin), giữ tiến trình sống liên tục
        print("\n🔄 AI Server đang chạy chế độ nền (Background Daemon Mode)...")
        while True:
            time.sleep(3600)


if __name__ == "__main__":
    main()
