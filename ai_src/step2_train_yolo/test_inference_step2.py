#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
BƯỚC 2: KIỂM THỬ SUY LUẬN KẾT HỢP (YOLO11-SEG + YOLO11-POSE) SAU KHI TẢI TRỌNG SỐ TỪ KAGGLE
===========================================================================================
Sau khi bạn huấn luyện xong trên Kaggle và tải file `dobot_cr10a_trained_models.zip`
về giải nén vào thư mục `/home/nhan/RB3t_PJ/ai_src/step2_train_yolo/weights/`,
chạy script này để kiểm tra trực quan Mask + 12 Mốc giải phẫu trên tập ảnh Validation.
"""

import time
from pathlib import Path
import cv2
import numpy as np

STEP2_DIR = Path("/home/nhan/RB3t_PJ/ai_src/step2_train_yolo")
WEIGHTS_DIR = STEP2_DIR / "weights"
VAL_IMG_DIR = Path("/home/nhan/RB3t_PJ/ai_src/step1_auto_label/dataset_yolo/images/val")
OUT_DIR = STEP2_DIR / "inference_results"

LANDMARK_NAMES = [
    "waist_left", "waist_center", "waist_right", "crotch",
    "thigh_left", "thigh_right", "knee_left", "knee_right",
    "cuff_left_outer", "cuff_left_inner", "cuff_right_inner", "cuff_right_outer"
]

SKELETON_EDGES = [
    (0, 1), (1, 2),          # Đường cạp quần
    (1, 3),                  # Trục thân trên (cạp -> đáy đũng)
    (3, 4), (4, 6),          # Trục ống chân trái
    (3, 5), (5, 7),          # Trục ống chân phải
    (0, 8), (3, 9), (8, 9),  # Biên và gấu ống trái
    (2, 11), (3, 10), (10, 11)  # Biên và gấu ống phải
]


def main():
    WEIGHTS_DIR.mkdir(parents=True, exist_ok=True)
    OUT_DIR.mkdir(parents=True, exist_ok=True)

    seg_pt = WEIGHTS_DIR / "best_yolo11_seg.pt"
    pose_pt = WEIGHTS_DIR / "best_yolo11_pose.pt"

    if not seg_pt.exists() or not pose_pt.exists():
        print("!" * 70)
        print("CHƯA TÌM THẤY FILE TRỌNG SỐ!")
        print(f"Hãy tải `dobot_cr10a_trained_models.zip` từ Kaggle về và giải nén 2 file:")
        print(f"  1. {seg_pt}")
        print(f"  2. {pose_pt}")
        print("!" * 70)
        return

    from ultralytics import YOLO

    model_seg = YOLO(str(seg_pt))
    model_pose = YOLO(str(pose_pt))

    val_imgs = sorted(list(VAL_IMG_DIR.glob("*.png")) + list(VAL_IMG_DIR.glob("*.jpg")))[:20]
    print(f"-> Đang chạy suy luận thử nghiệm trên {len(val_imgs)} ảnh Validation...")

    for img_path in val_imgs:
        real_path = img_path.resolve()
        bgr = cv2.imread(str(real_path))
        if bgr is None:
            continue
        vis = bgr.copy()
        h, w = vis.shape[:2]

        t0 = time.perf_counter()
        res_seg = model_seg.predict(bgr, imgsz=640, conf=0.5, verbose=False)[0]
        res_pose = model_pose.predict(bgr, imgsz=640, conf=0.5, verbose=False)[0]
        dt_ms = (time.perf_counter() - t0) * 1000.0

        # 1. Vẽ Mask phân đoạn thân quần
        if res_seg.masks is not None and len(res_seg.masks.data) > 0:
            mask_raw = res_seg.masks.data[0].cpu().numpy()
            mask_resized = cv2.resize(mask_raw, (w, h), interpolation=cv2.INTER_LINEAR)
            mask_bin = (mask_resized > 0.5).astype(np.uint8) * 255
            green = np.zeros_like(vis)
            green[mask_bin > 0] = (0, 220, 80)
            vis = cv2.addWeighted(vis, 0.72, green, 0.28, 0)
            contours, _ = cv2.findContours(mask_bin, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            cv2.drawContours(vis, contours, -1, (0, 255, 120), 2)

        # 2. Vẽ 12 mốc giải phẫu & Khung xương
        if res_pose.keypoints is not None and len(res_pose.keypoints.xy) > 0:
            kpts = res_pose.keypoints.xy[0].cpu().numpy()  # (12, 2)
            for i, j in SKELETON_EDGES:
                pt1 = (int(kpts[i][0]), int(kpts[i][1]))
                pt2 = (int(kpts[j][0]), int(kpts[j][1]))
                cv2.line(vis, pt1, pt2, (255, 220, 0), 2, cv2.LINE_AA)

            for idx, (kx, ky) in enumerate(kpts):
                px, py = int(kx), int(ky)
                cv2.circle(vis, (px, py), 8, (0, 0, 255), -1, cv2.LINE_AA)
                cv2.circle(vis, (px, py), 10, (255, 255, 255), 2, cv2.LINE_AA)
                cv2.putText(
                    vis, f"{idx}:{LANDMARK_NAMES[idx]}",
                    (px + 12, py + 5), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2, cv2.LINE_AA
                )

        cv2.putText(
            vis, f"Sample: {img_path.stem} | Total AI Inference: {dt_ms:.1f} ms",
            (25, 45), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 255), 2, cv2.LINE_AA
        )
        out_file = OUT_DIR / f"{img_path.stem}_ai_pred.jpg"
        cv2.imwrite(str(out_file), vis, [cv2.IMWRITE_JPEG_QUALITY, 90])

    print(f"✓ Đã xuất {len(val_imgs)} ảnh dự đoán AI tại: {OUT_DIR}")


if __name__ == "__main__":
    main()
