#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
=============================================================================
CÔNG TY CỔ PHẦN CHẾ TẠO MÁY 3C / ROBOT 3T - DỰ ÁN DOBOT CR10A
BƯỚC 1: TIỀN XỬ LÝ & TỰ ĐỘNG GÁN NHÃN TỪ ĐÁM MÂY ĐIỂM 3D (AUTO-LABELING PIPELINE)
=============================================================================
Tuân thủ toàn bộ các chỉ đạo kỹ thuật từ file góp ý báo cáo:
1. Không gán nhãn thủ công bằng tay (tránh sai số chủ quan trên vải tối màu).
2. Tự động trích xuất Mask phân vùng chuẩn xác từ độ cao nổi 3D so với mặt bàn (RANSAC).
3. Tinh chỉnh biên cục bộ (3D-Guided GrabCut) để ôm sát từng thớ vải / gấu quần.
4. Trích xuất 12 mốc giải phẫu 3D (Landmarks) & Vector pháp tuyến bề mặt (PCA/SVD).
5. Xuất nhãn chuẩn định dạng YOLO11-seg và YOLO11-pose.
6. Tổ chức cấu trúc thư mục dataset chuẩn train/val (80/20) kèm file cấu hình data.yaml.
7. Xuất ảnh kiểm tra trực quan (Verification Overlays) để kỹ sư kiểm tra thẩm định.
=============================================================================
"""

import os
import sys
import glob
import json
import csv
import time
import shutil
from pathlib import Path

# Đảm bảo in UTF-8 trơn tru
if sys.platform.startswith('win'):
    try:
        sys.stdout.reconfigure(encoding='utf-8')
        sys.stderr.reconfigure(encoding='utf-8')
    except Exception:
        pass

import numpy as np
import cv2
from sklearn.linear_model import RANSACRegressor, LinearRegression


class AutoLabeling3D:
    LANDMARK_NAMES = [
        "waist_left", "waist_center", "waist_right",
        "crotch", "thigh_left", "thigh_right",
        "knee_left", "knee_right",
        "cuff_left_outer", "cuff_left_inner",
        "cuff_right_inner", "cuff_right_outer"
    ]

    SKELETON_EDGES = [
        ("waist_left", "waist_center"), ("waist_center", "waist_right"),
        ("waist_left", "crotch"), ("waist_right", "crotch"),
        ("crotch", "thigh_left"), ("crotch", "thigh_right"),
        ("thigh_left", "knee_left"), ("thigh_right", "knee_right"),
        ("knee_left", "cuff_left_outer"), ("knee_left", "cuff_left_inner"),
        ("knee_right", "cuff_right_outer"), ("knee_right", "cuff_right_inner"),
        ("cuff_left_outer", "cuff_left_inner"), ("cuff_right_inner", "cuff_right_outer"),
        ("waist_left", "thigh_left"), ("waist_right", "thigh_right")
    ]

    DEFAULT_DATASET_DIR = Path("/media/nhan/01DC3601B2514510/RB3t_PJ/ai_src/dataset")

    def __init__(self, dataset_dir=None, root_dir=None, height_threshold_mm=2.0, max_height_mm=120.0):
        self.step_dir = Path(__file__).resolve().parent
        self.ai_src_dir = self.step_dir.parent

        # Định vị thư mục dataset (Ưu tiên dataset gốc trên ổ cứng dữ liệu)
        candidate_dirs = [
            Path(dataset_dir).resolve() if dataset_dir else None,
            self.ai_src_dir / "dataset",
            self.DEFAULT_DATASET_DIR,
            Path("/media/nhan/01DC3601B2514510/RB3t_PJ/dataset"),
            (Path(root_dir).resolve() / "dataset") if root_dir else None
        ]

        found_data_dir = None
        for d in candidate_dirs:
            if d and (d / "_3D").exists() and (d / "_2D").exists():
                found_data_dir = d
                break

        if found_data_dir is None:
            found_data_dir = self.DEFAULT_DATASET_DIR

        self.dataset_dir = found_data_dir
        self.dataset_2d_dir = found_data_dir / "_2D"
        self.dataset_3d_dir = found_data_dir / "_3D"
        
        # Thư mục đích chuẩn YOLO nằm gọn trong step1_auto_label
        self.output_ai_dir = self.step_dir
        self.yolo_dir = self.output_ai_dir / "dataset_yolo"
        self.verification_dir = self.output_ai_dir / "verification_overlays"
        
        self.height_threshold_mm = float(height_threshold_mm)
        self.max_height_mm = float(max_height_mm)

    def load_organized_ply(self, ply_path):
        """Đọc đám mây điểm Zivid có cấu trúc (1200 x 1944)."""
        dt = np.dtype([('x', '<f4'), ('y', '<f4'), ('z', '<f4'), ('r', 'u1'), ('g', 'u1'), ('b', 'u1')])
        with open(ply_path, 'rb') as f:
            while True:
                line = f.readline()
                if not line or line.strip() == b'end_header':
                    break
            data = np.fromfile(f, dtype=dt, count=2332800)

        h, w = 1200, 1944
        xyz = np.stack([data['x'].reshape(h, w), data['y'].reshape(h, w), data['z'].reshape(h, w)], axis=-1)
        bgr = np.stack([data['b'].reshape(h, w), data['g'].reshape(h, w), data['r'].reshape(h, w)], axis=-1)
        rgb = np.stack([data['r'].reshape(h, w), data['g'].reshape(h, w), data['b'].reshape(h, w)], axis=-1)
        return xyz, bgr, rgb

    def fit_table_plane(self, xyz):
        """Ước lượng mặt phẳng bàn làm việc bằng RANSAC trên toàn bộ FOV Zivid."""
        x, y, z = xyz[:, :, 0], xyz[:, :, 1], xyz[:, :, 2]
        valid = (z > 1000.0) & (z < 2200.0)
        pts_xy = np.stack([x[valid], y[valid]], axis=-1)
        pts_z = z[valid]

        if len(pts_xy) < 5000:
            raise ValueError("Không đủ điểm 3D hợp lệ để xác định mặt phẳng bàn.")

        n_samples = min(40000, len(pts_xy))
        idx = np.random.choice(len(pts_xy), size=n_samples, replace=False)

        ransac = RANSACRegressor(LinearRegression(), min_samples=3, residual_threshold=2.5, max_trials=100)
        ransac.fit(pts_xy[idx], pts_z[idx])

        a, b_coef = ransac.estimator_.coef_
        c = ransac.estimator_.intercept_
        norm = np.sqrt(a * a + b_coef * b_coef + 1.0)
        normal = np.array([-a / norm, -b_coef / norm, 1.0 / norm])

        return {'a': float(a), 'b': float(b_coef), 'c': float(c), 'normal': [float(v) for v in normal]}

    def segment_pants_3d(self, xyz, bgr, plane_params):
        """Bóc tách vùng quần tự động từ độ cao 3D kết hợp 3D-Guided GrabCut."""
        h, w = xyz.shape[:2]
        x, y, z = xyz[:, :, 0], xyz[:, :, 1], xyz[:, :, 2]
        a, b_coef, c = plane_params['a'], plane_params['b'], plane_params['c']

        z_table = a * x + b_coef * y + c
        height = z_table - z
        valid_z = (z > 1000.0) & (z < 2200.0)

        # 1. Lọc vật thể nổi cao hơn mặt bàn
        objects_raw = (height >= self.height_threshold_mm) & (height <= self.max_height_mm) & valid_z
        k_morph = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (9, 9))
        objects_clean = cv2.morphologyEx(objects_raw.astype(np.uint8) * 255, cv2.MORPH_CLOSE, k_morph)
        objects_clean = cv2.morphologyEx(objects_clean, cv2.MORPH_OPEN, k_morph)

        # 2. Phân tích thành phần liên thông lớn nhất
        num_labels, labels, stats, _ = cv2.connectedComponentsWithStats(objects_clean)
        valid_candidates = []
        for lbl in range(1, num_labels):
            area = stats[lbl, cv2.CC_STAT_AREA]
            left = stats[lbl, cv2.CC_STAT_LEFT]
            top = stats[lbl, cv2.CC_STAT_TOP]
            bw = stats[lbl, cv2.CC_STAT_WIDTH]
            bh = stats[lbl, cv2.CC_STAT_HEIGHT]

            touches_border = (left <= 3) or (top <= 3) or (left + bw >= w - 3) or (top + bh >= h - 3)
            if not touches_border and area >= 80000:
                valid_candidates.append((lbl, area))

        if not valid_candidates:
            largest_label = 1 + np.argmax(stats[1:, cv2.CC_STAT_AREA])
            pants_seed = (labels == largest_label).astype(np.uint8) * 255
        else:
            best_lbl = max(valid_candidates, key=lambda it: it[1])[0]
            pants_seed = (labels == best_lbl).astype(np.uint8) * 255

        # 3. Tinh chỉnh cạnh bằng 3D-Guided GrabCut
        bx, by, bw, bh = cv2.boundingRect(pants_seed)
        margin = 35
        bx0, by0 = max(0, bx - margin), max(0, by - margin)
        bx1, by1 = min(w, bx + bw + margin), min(h, by + bh + margin)

        crop_bgr = bgr[by0:by1, bx0:bx1]
        crop_h = height[by0:by1, bx0:bx1]
        crop_fg = pants_seed[by0:by1, bx0:bx1]

        gc_mask = np.full(crop_bgr.shape[:2], cv2.GC_PR_BGD, dtype=np.uint8)
        brightness = np.mean(crop_bgr.astype(np.float32), axis=-1)
        is_floor = (crop_h <= 1.2) | ((crop_h <= 2.2) & (brightness > 150.0))
        gc_mask[is_floor] = cv2.GC_BGD

        sure_fg = (crop_fg > 0) & (crop_h >= 4.5)
        gc_mask[sure_fg] = cv2.GC_FGD

        probable_fg = (crop_fg > 0) & (crop_h >= 1.5) & (~is_floor)
        gc_mask[probable_fg] = cv2.GC_PR_FGD

        bgd_m = np.zeros((1, 65), np.float64)
        fgd_m = np.zeros((1, 65), np.float64)
        cv2.grabCut(crop_bgr, gc_mask, None, bgd_m, fgd_m, 4, cv2.GC_INIT_WITH_MASK)

        crop_snapped = np.where((gc_mask == cv2.GC_FGD) | (gc_mask == cv2.GC_PR_FGD), 255, 0).astype(np.uint8)
        contours, _ = cv2.findContours(crop_snapped, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        crop_final = np.zeros_like(crop_snapped)
        cv2.drawContours(crop_final, contours, -1, 255, thickness=cv2.FILLED)

        mask_final = np.zeros(bgr.shape[:2], dtype=np.uint8)
        mask_final[by0:by1, bx0:bx1] = crop_final

        return mask_final, height

    def find_cross_section_center(self, mask, seed_pt, perp_dir, max_radius=200.0):
        """Bắn tia cắt ngang vuông góc trục ống chân để tìm tim giữa thực tế của ống vải (Centerline Ray-Casting)."""
        h_img, w_img = mask.shape
        ts = np.linspace(-max_radius, max_radius, int(2 * max_radius + 1))
        coords = (seed_pt[None, :] + ts[:, None] * perp_dir[None, :]).astype(int)

        valid_bounds = (coords[:, 0] >= 0) & (coords[:, 0] < w_img) & (coords[:, 1] >= 0) & (coords[:, 1] < h_img)
        inside_mask = np.zeros(len(ts), dtype=bool)
        inside_mask[valid_bounds] = mask[coords[valid_bounds, 1], coords[valid_bounds, 0]] > 0

        center_idx = len(ts) // 2
        if inside_mask[center_idx]:
            left = center_idx
            while left > 0 and inside_mask[left - 1]:
                left -= 1
            right = center_idx
            while right < len(ts) - 1 and inside_mask[right + 1]:
                right += 1
            p_outer = coords[left].astype(np.float32)
            p_inner = coords[right].astype(np.float32)
            return (p_outer + p_inner) / 2.0

        fabric_indices = np.where(inside_mask)[0]
        if len(fabric_indices) > 0:
            diffs = np.diff(fabric_indices)
            splits = np.where(diffs > 1)[0] + 1
            segments = np.split(fabric_indices, splits)
            best_seg = min(segments, key=lambda seg: abs(np.mean(seg) - center_idx))
            p_outer = coords[best_seg[0]].astype(np.float32)
            p_inner = coords[best_seg[-1]].astype(np.float32)
            return (p_outer + p_inner) / 2.0

        return seed_pt

    def extract_landmarks_3d(self, xyz, mask, plane_params):
        """
        Trích xuất 12 mốc giải phẫu bất biến góc xoay (Invariant Geometry):
        1. Tìm đáy đũng (Crotch) bằng khuyết tật lồi Convexity Defects sâu nhất.
        2. Xác định trục ống chân và trục cạp lưng.
        3. Chiếu tìm 2 góc cạp (waist_left, waist_right) và tâm cạp (waist_center).
        4. Xác định 4 góc gấu (Cuff corners).
        5. Bắn tia Centerline Ray-Casting tìm tim ống vải cho Đùi (28%) và Gối (55%).
        """
        h, w = mask.shape
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return {}

        c = max(contours, key=cv2.contourArea)
        if len(c) < 10:
            return {}

        hull = cv2.convexHull(c, returnPoints=False)
        defects = cv2.convexityDefects(c, hull) if len(hull) > 3 else None

        landmarks = {}
        pts_all = c.reshape(-1, 2).astype(np.float32)

        # Trường hợp 1: Có khuyết tật lồi (Quần 2 ống tiêu chuẩn xoay bất kỳ góc nào)
        if defects is not None and len(defects) > 0:
            defects_2d = defects.reshape(-1, 4)
            defects_sorted = defects_2d[np.argsort(-defects_2d[:, 3])]
            s, e, f, d_val = defects_sorted[0]
            crotch_pt = np.array(c[f][0], dtype=np.float32)

            tip1 = np.array(c[s][0], dtype=np.float32)
            tip2 = np.array(c[e][0], dtype=np.float32)

            # Vector trục chính giữa 2 ống chân (hướng từ đáy đũng ra hai gấu quần)
            v_mid = (tip1 + tip2) / 2.0 - crotch_pt
            norm_mid = np.linalg.norm(v_mid)
            v_mid_dir = v_mid / norm_mid if norm_mid > 0 else np.array([0.0, 1.0], dtype=np.float32)

            waist_dir = -v_mid_dir
            perp_dir = np.array([-v_mid_dir[1], v_mid_dir[0]], dtype=np.float32)

            # Chuẩn hóa hướng perp_dir: Đảm bảo tip1 luôn nằm ở phía âm (< 0), tip2 ở phía dương (> 0)
            if np.dot(tip1 - crotch_pt, perp_dir) > 0:
                perp_dir = -perp_dir

            # 1. Mốc cạp lưng (Waist): Bắt 2 góc ngoài cùng của cạp và tâm cạp
            waist_projections = np.dot(pts_all - crotch_pt, waist_dir)
            top_w_pts = pts_all[waist_projections >= np.max(waist_projections) * 0.85]

            if len(top_w_pts) >= 2:
                waist_perps = np.dot(top_w_pts - crotch_pt, perp_dir)
                # waist_left cùng phía Ống 1 (âm), waist_right cùng phía Ống 2 (dương)
                waist_left = top_w_pts[np.argmin(waist_perps)]
                waist_right = top_w_pts[np.argmax(waist_perps)]
                waist_center = (waist_left + waist_right) / 2.0
            else:
                waist_center = crotch_pt + waist_dir * 200.0
                waist_left = waist_center - perp_dir * 120.0
                waist_right = waist_center + perp_dir * 120.0

            # 2. Phân vùng biên (Contour Partitioning) tách biệt hoàn toàn Ống 1 và Ống 2
            # Chặn triệt để lỗi gấu ống này bắt nhầm sang biên của ống kia gây chéo dây skeleton
            pts_leg1 = pts_all[(np.dot(pts_all - crotch_pt, v_mid_dir) > 0) & (np.dot(pts_all - crotch_pt, perp_dir) < 0)]
            pts_leg2 = pts_all[(np.dot(pts_all - crotch_pt, v_mid_dir) > 0) & (np.dot(pts_all - crotch_pt, perp_dir) > 0)]

            def extract_cuff_from_leg(pts_leg, tip_fallback):
                if len(pts_leg) < 4:
                    perp_t = np.array([-v_mid_dir[1], v_mid_dir[0]], dtype=np.float32)
                    return tip_fallback - perp_t * 60.0, tip_fallback + perp_t * 60.0
                v_leg = np.mean(pts_leg, axis=0) - crotch_pt
                d_leg = v_leg / (np.linalg.norm(v_leg) + 1e-6)
                projs = np.dot(pts_leg - crotch_pt, d_leg)
                hem = pts_leg[projs >= np.max(projs) - 60.0]
                if len(hem) < 2:
                    hem = pts_leg[projs >= np.max(projs) - 100.0]
                return hem

            # Ống 1 (Bên trái): perp âm hơn là mép ngoài (xa tim), perp gần 0 hơn là mép trong (gần tim)
            hem1 = extract_cuff_from_leg(pts_leg1, tip1)
            perps1 = np.dot(hem1 - crotch_pt, perp_dir)
            cuff_left_outer = hem1[np.argmin(perps1)]
            cuff_left_inner = hem1[np.argmax(perps1)]

            # Ống 2 (Bên phải): perp gần 0 hơn là mép trong (gần tim), perp dương lớn hơn là mép ngoài (xa tim)
            hem2 = extract_cuff_from_leg(pts_leg2, tip2)
            perps2 = np.dot(hem2 - crotch_pt, perp_dir)
            cuff_right_inner = hem2[np.argmin(perps2)]
            cuff_right_outer = hem2[np.argmax(perps2)]

            cuff1_mid = (cuff_left_outer + cuff_left_inner) / 2.0
            cuff2_mid = (cuff_right_outer + cuff_right_inner) / 2.0

            # Trục riêng từng ống
            v_l1 = cuff1_mid - crotch_pt
            n_l1 = np.linalg.norm(v_l1)
            dir_l1 = v_l1 / n_l1 if n_l1 > 0 else v_mid_dir
            perp_l1 = np.array([-dir_l1[1], dir_l1[0]], dtype=np.float32)

            v_l2 = cuff2_mid - crotch_pt
            n_l2 = np.linalg.norm(v_l2)
            dir_l2 = v_l2 / n_l2 if n_l2 > 0 else v_mid_dir
            perp_l2 = np.array([-dir_l2[1], dir_l2[0]], dtype=np.float32)

            # Mặt nạ phân vùng độc lập để tia ray-casting không bao giờ bắn xuyên sang ống đối diện
            h_m, w_m = mask.shape
            y_g, x_g = np.indices((h_m, w_m))
            sep_val = (x_g - crotch_pt[0]) * perp_dir[0] + (y_g - crotch_pt[1]) * perp_dir[1]

            mask_leg1 = mask.copy()
            mask_leg1[sep_val > 0] = 0
            mask_leg2 = mask.copy()
            mask_leg2[sep_val < 0] = 0

            # Điểm neo hình học: Đùi (28%), Gối (55%)
            seed_knee1 = 0.45 * crotch_pt + 0.55 * cuff1_mid
            seed_knee2 = 0.45 * crotch_pt + 0.55 * cuff2_mid
            seed_thigh1 = 0.72 * crotch_pt + 0.28 * cuff1_mid
            seed_thigh2 = 0.72 * crotch_pt + 0.28 * cuff2_mid

            knee_left = self.find_cross_section_center(mask_leg1, seed_knee1, perp_l1)
            knee_right = self.find_cross_section_center(mask_leg2, seed_knee2, perp_l2)
            thigh_left = self.find_cross_section_center(mask_leg1, seed_thigh1, perp_l1)
            thigh_right = self.find_cross_section_center(mask_leg2, seed_thigh2, perp_l2)

            raw_2d = {
                "waist_left": waist_left, "waist_center": waist_center, "waist_right": waist_right,
                "crotch": crotch_pt,
                "thigh_left": thigh_left, "thigh_right": thigh_right,
                "knee_left": knee_left, "knee_right": knee_right,
                "cuff_left_outer": cuff_left_outer, "cuff_left_inner": cuff_left_inner,
                "cuff_right_inner": cuff_right_inner, "cuff_right_outer": cuff_right_outer
            }
        else:
            # Fallback nếu phôi không có defect rõ (dùng PCA trên mask)
            y_indices, x_indices = np.where(mask > 0)
            pts_xy = np.column_stack([x_indices, y_indices]).astype(np.float32)
            mean_pt = np.mean(pts_xy, axis=0)
            _, eig_vec = np.linalg.eigh(np.cov(pts_xy - mean_pt, rowvar=False))
            axis_major = eig_vec[:, 1]
            axis_minor = eig_vec[:, 0]

            raw_2d = {
                "waist_left": mean_pt - axis_major * 150.0 - axis_minor * 80.0,
                "waist_center": mean_pt - axis_major * 150.0,
                "waist_right": mean_pt - axis_major * 150.0 + axis_minor * 80.0,
                "crotch": mean_pt,
                "thigh_left": mean_pt + axis_major * 50.0 - axis_minor * 60.0,
                "thigh_right": mean_pt + axis_major * 50.0 + axis_minor * 60.0,
                "knee_left": mean_pt + axis_major * 120.0 - axis_minor * 60.0,
                "knee_right": mean_pt + axis_major * 120.0 + axis_minor * 60.0,
                "cuff_left_outer": mean_pt + axis_major * 200.0 - axis_minor * 80.0,
                "cuff_left_inner": mean_pt + axis_major * 200.0 - axis_minor * 40.0,
                "cuff_right_inner": mean_pt + axis_major * 200.0 + axis_minor * 40.0,
                "cuff_right_outer": mean_pt + axis_major * 200.0 + axis_minor * 80.0
            }

        # Tra cứu tọa độ 3D và vector pháp tuyến bề mặt vi mô
        landmarks_3d = {}
        for name, pt in raw_2d.items():
            u = int(np.clip(round(pt[0]), 0, w - 1))
            v = int(np.clip(round(pt[1]), 0, h - 1))
            coord_3d = self.get_valid_3d_point(xyz, u, v)
            normal = self.estimate_surface_normal(xyz, u, v)

            a, b_coef, c_val = plane_params['a'], plane_params['b'], plane_params['c']
            z_tbl = a * coord_3d[0] + b_coef * coord_3d[1] + c_val
            h_above = float(z_tbl - coord_3d[2])

            landmarks_3d[name] = {
                "pixel_u": u,
                "pixel_v": v,
                "coord_3d_mm": [float(coord_3d[0]), float(coord_3d[1]), float(coord_3d[2])],
                "height_above_table_mm": round(h_above, 2),
                "normal_vector": [float(normal[0]), float(normal[1]), float(normal[2])]
            }

        return landmarks_3d

    def get_valid_3d_point(self, xyz, u, v, window=9):
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

    def estimate_surface_normal(self, xyz, u, v, window=6):
        h, w = xyz.shape[:2]
        u_int, v_int = int(round(u)), int(round(v))
        u_min, u_max = max(0, u_int - window), min(w, u_int + window + 1)
        v_min, v_max = max(0, v_int - window), min(h, v_int + window + 1)

        sub = xyz[v_min:v_max, u_min:u_max]
        pts = sub.reshape(-1, 3)
        valid = pts[:, 2] > 500.0
        pts_valid = pts[valid]

        if len(pts_valid) >= 5:
            centroid = np.mean(pts_valid, axis=0)
            pts_centered = pts_valid - centroid
            cov = np.dot(pts_centered.T, pts_centered)
            eigenvalues, eigenvectors = np.linalg.eigh(cov)
            normal = eigenvectors[:, 0]
            if normal[2] > 0:
                normal = -normal
            return normal
        return np.array([0.0, 0.0, -1.0])

    def export_yolo_seg(self, mask, out_txt_path, class_id=0):
        """Xuất file nhãn chuẩn YOLO11-seg: class x1 y1 x2 y2 ..."""
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
        with open(out_txt_path, 'w', encoding='utf-8') as f:
            f.write(f"{class_id} {coords_str}\n")

    def export_yolo_pose(self, mask, landmarks_data, out_txt_path, class_id=0):
        """Xuất file nhãn chuẩn YOLO11-pose: class cx cy w h kp1_x kp1_y kp1_v ..."""
        h, w = mask.shape
        y_indices, x_indices = np.where(mask > 0)
        if len(y_indices) == 0:
            return

        bx = float(np.min(x_indices)) / w
        by = float(np.min(y_indices)) / h
        bw = float(np.max(x_indices) - np.min(x_indices)) / w
        bh = float(np.max(y_indices) - np.min(y_indices)) / h
        cx = bx + bw / 2.0
        cy = by + bh / 2.0

        kp_strs = []
        for name in self.LANDMARK_NAMES:
            if name in landmarks_data:
                lm = landmarks_data[name]
                kp_x = float(lm["pixel_u"]) / w
                kp_y = float(lm["pixel_v"]) / h
                v = 2  # Visible and labeled
                kp_strs.append(f"{kp_x:.6f} {kp_y:.6f} {v}")
            else:
                kp_strs.append("0.000000 0.000000 0")

        line = f"{class_id} {cx:.6f} {cy:.6f} {bw:.6f} {bh:.6f} " + " ".join(kp_strs) + "\n"
        with open(out_txt_path, 'w', encoding='utf-8') as f:
            f.write(line)

    def generate_verification_overlay(self, bgr, mask, landmarks_data, sample_name, metrics):
        """Tạo ảnh trực quan thẩm định (Overlay) hiển thị đồng thời Mask viền và 12 mốc giải phẫu."""
        overlay = bgr.copy()
        
        # 1. Phủ màu xanh lá bán trong suốt lên vùng quần
        green_layer = bgr.copy()
        green_layer[mask > 0] = [0, 240, 80]
        cv2.addWeighted(green_layer, 0.35, overlay, 0.65, 0, overlay)

        # 2. Vẽ viền trắng sắc nét
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(overlay, contours, -1, (255, 255, 255), 2)

        # 3. Vẽ khung xương giải phẫu nối giữa các mốc
        for p1_name, p2_name in self.SKELETON_EDGES:
            if p1_name in landmarks_data and p2_name in landmarks_data:
                pt1 = (landmarks_data[p1_name]["pixel_u"], landmarks_data[p1_name]["pixel_v"])
                pt2 = (landmarks_data[p2_name]["pixel_u"], landmarks_data[p2_name]["pixel_v"])
                cv2.line(overlay, pt1, pt2, (0, 255, 255), 2, cv2.LINE_AA)

        # 4. Vẽ các điểm mốc (Vòng tròn đỏ + tâm vàng)
        for name, lm in landmarks_data.items():
            pt = (lm["pixel_u"], lm["pixel_v"])
            cv2.circle(overlay, pt, 7, (0, 0, 255), -1, cv2.LINE_AA)
            cv2.circle(overlay, pt, 3, (0, 255, 255), -1, cv2.LINE_AA)
            cv2.putText(overlay, name, (pt[0] + 10, pt[1] + 4), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1, cv2.LINE_AA)

        # 5. Header hiển thị thông số kiểm tra
        c3d = metrics["centroid_3d_mm"]
        area_cm2 = metrics["approx_area_cm2"]
        h_mean = metrics["bbox_3d_mm"]["height_mean"]
        
        cv2.rectangle(overlay, (20, 20), (780, 100), (20, 24, 32), -1)
        cv2.rectangle(overlay, (20, 20), (780, 100), (60, 75, 100), 1)
        cv2.putText(overlay, f"DOBOT CR10A AUTO-LABELING: {sample_name}", (35, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 230, 255), 2, cv2.LINE_AA)
        cv2.putText(overlay, f"Centroid: [{c3d[0]:.1f}, {c3d[1]:.1f}, {c3d[2]:.1f}] mm | Area: {area_cm2:.1f} cm2 | Height: {h_mean:.1f} mm", (35, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.52, (200, 220, 240), 1, cv2.LINE_AA)

        return overlay

    def compute_3d_metrics(self, xyz, mask, height_map):
        mask_bool = (mask > 0) & (xyz[:, :, 2] > 500.0)
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

        y_indices, x_indices = np.where(mask > 0)
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

    def setup_yolo_directories(self):
        """Khởi tạo cấu trúc thư mục dataset YOLO11 chuẩn (Segmentation & Pose)."""
        for split in ['train', 'val']:
            (self.yolo_dir / "images" / split).mkdir(parents=True, exist_ok=True)
            (self.yolo_dir / "labels" / split).mkdir(parents=True, exist_ok=True)
            (self.yolo_dir / "labels_pose" / split).mkdir(parents=True, exist_ok=True)
        self.verification_dir.mkdir(parents=True, exist_ok=True)

        # Tạo file data.yaml cho Segmentation
        data_yaml_path = self.yolo_dir / "data.yaml"
        with open(data_yaml_path, 'w', encoding='utf-8') as f:
            f.write(f"path: {self.yolo_dir.as_posix()}\n")
            f.write("train: images/train\n")
            f.write("val: images/val\n\n")
            f.write("names:\n")
            f.write("  0: garment\n")

        # Tạo file data_pose.yaml cho Pose Estimation
        data_pose_yaml_path = self.yolo_dir / "data_pose.yaml"
        with open(data_pose_yaml_path, 'w', encoding='utf-8') as f:
            f.write(f"path: {self.yolo_dir.as_posix()}\n")
            f.write("train: images/train\n")
            f.write("val: images/val\n\n")
            f.write("kpt_shape: [12, 3]\n")
            f.write("names:\n")
            f.write("  0: pants\n")
        print(f"✓ Đã tạo cấu trúc thư mục dataset YOLO tại: {self.yolo_dir}")

    def run_batch_processing(self, val_ratio=0.2, max_samples=None):
        """Chạy xử lý toàn bộ các mẫu có cặp dữ liệu 3D và 2D."""
        start_all = time.time()
        self.setup_yolo_directories()

        ply_files = sorted(list(self.dataset_3d_dir.glob("*.ply")))
        if max_samples is not None:
            ply_files = ply_files[:max_samples]
        print(f"\n=============================================================================")
        print(f"BẮT ĐẦU TỰ SINH NHÃN TỪ 3D CHO {len(ply_files)} MẪU POINT CLOUD ZIVID...")
        print(f"=============================================================================")

        report_rows = []
        train_count = 0
        val_count = 0

        # Phân chia ngẫu nhiên cố định (seed) để tái lập kết quả
        np.random.seed(42)
        shuffled_indices = np.random.permutation(len(ply_files))
        num_val = int(round(len(ply_files) * val_ratio))
        val_set_indices = set(shuffled_indices[:num_val])

        for idx, ply_path in enumerate(ply_files):
            stem = ply_path.stem  # e.g. pcl_101
            sample_num = stem.replace("pcl_", "")
            t0 = time.time()
            print(f"[{idx + 1:02d}/{len(ply_files):02d}] Đang xử lý mẫu {stem}...", end="", flush=True)

            try:
                xyz, bgr, rgb = self.load_organized_ply(ply_path)
                plane_params = self.fit_table_plane(xyz)
                mask, height_map = self.segment_pants_3d(xyz, bgr, plane_params)
                metrics = self.compute_3d_metrics(xyz, mask, height_map)
                landmarks = self.extract_landmarks_3d(xyz, mask, plane_params)

                # Xác định tập train hay val
                split = "val" if idx in val_set_indices else "train"
                if split == "val":
                    val_count += 1
                else:
                    train_count += 1

                # 1. Liên kết ảnh gốc vào dataset YOLO bằng Symlink (0 bytes, tránh đầy ổ cứng)
                img_src_path = self.dataset_2d_dir / f"image_{sample_num}.png"
                img_dest_path = self.yolo_dir / "images" / split / f"{stem}.png"
                if img_dest_path.is_symlink() or img_dest_path.exists():
                    img_dest_path.unlink()
                if img_src_path.exists():
                    try:
                        os.symlink(img_src_path, img_dest_path)
                    except OSError:
                        cv2.imwrite(str(img_dest_path), bgr, [cv2.IMWRITE_PNG_COMPRESSION, 9])
                else:
                    cv2.imwrite(str(img_dest_path), bgr, [cv2.IMWRITE_PNG_COMPRESSION, 9])

                # 2. Xuất nhãn YOLO11-seg và YOLO11-pose
                label_dest_path = self.yolo_dir / "labels" / split / f"{stem}.txt"
                self.export_yolo_seg(mask, label_dest_path)
                pose_label_dest_path = self.yolo_dir / "labels_pose" / split / f"{stem}.txt"
                self.export_yolo_pose(mask, landmarks, pose_label_dest_path)

                # 3. Xuất file landmarks 3D JSON
                json_dest_path = self.verification_dir / f"{stem}_landmarks_3d.json"
                json_data = {
                    "sample": stem,
                    "plane_equation": plane_params,
                    "metrics": metrics,
                    "landmarks_count": len(landmarks),
                    "landmarks": landmarks
                }
                with open(json_dest_path, 'w', encoding='utf-8') as f:
                    json.dump(json_data, f, indent=2, ensure_ascii=False, default=lambda x: x.tolist() if hasattr(x, 'tolist') else str(x))

                # 4. Xuất ảnh trực quan thẩm định (Overlay nén JPG chất lượng cao, siêu nhẹ)
                overlay_img = self.generate_verification_overlay(bgr, mask, landmarks, stem, metrics)
                overlay_dest_path = self.verification_dir / f"{stem}_verification.jpg"
                cv2.imwrite(str(overlay_dest_path), overlay_img, [cv2.IMWRITE_JPEG_QUALITY, 80])

                elapsed = time.time() - t0
                c3d = metrics["centroid_3d_mm"]
                print(f" XONG ({elapsed:.2f}s) -> Split: [{split.upper()}], Diện tích: {metrics['approx_area_cm2']:.1f} cm2")

                report_rows.append({
                    "sample": stem,
                    "split": split,
                    "status": "SUCCESS",
                    "area_cm2": round(metrics["approx_area_cm2"], 2),
                    "height_mean_mm": round(metrics["bbox_3d_mm"]["height_mean"], 2),
                    "height_max_mm": round(metrics["bbox_3d_mm"]["height_max"], 2),
                    "centroid_x_mm": round(c3d[0], 2),
                    "centroid_y_mm": round(c3d[1], 2),
                    "centroid_z_mm": round(c3d[2], 2),
                    "process_time_s": round(elapsed, 3)
                })

            except Exception as e:
                print(f" LỖI: {e}")
                report_rows.append({
                    "sample": stem,
                    "split": "NONE",
                    "status": f"ERROR: {str(e)}",
                    "area_cm2": 0, "height_mean_mm": 0, "height_max_mm": 0,
                    "centroid_x_mm": 0, "centroid_y_mm": 0, "centroid_z_mm": 0,
                    "process_time_s": 0
                })

        # Lưu báo cáo tổng kết ra file CSV
        report_csv_path = self.output_ai_dir / "batch_auto_label_report.csv"
        with open(report_csv_path, 'w', newline='', encoding='utf-8') as f:
            writer = csv.DictWriter(f, fieldnames=list(report_rows[0].keys()))
            writer.writeheader()
            writer.writerows(report_rows)

        total_time = time.time() - start_all
        print(f"\n=============================================================================")
        print(f"HOÀN TẤT BƯỚC 1: TỰ ĐỘNG GÁN NHÃN 3D CHO {len(ply_files)} MẪU TRONG {total_time:.1f} GIÂY!")
        print(f"• Tập Train: {train_count} ảnh ({self.yolo_dir / 'images/train'})")
        print(f"• Tập Val:   {val_count} ảnh ({self.yolo_dir / 'images/val'})")
        print(f"• Ảnh thẩm định trực quan: {self.verification_dir}")
        print(f"• Báo cáo thống kê CSV: {report_csv_path}")
        print(f"=============================================================================\n")


if __name__ == "__main__":
    detector = AutoLabeling3D()
    detector.run_batch_processing(val_ratio=0.2)
