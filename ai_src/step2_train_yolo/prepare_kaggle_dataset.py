#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
BƯỚC 2: ĐÓNG GÓI DATASET CHUẨN BỊ UPLOAD LÊN KAGGLE / GOOGLE COLAB
==================================================================
Script này tự động:
1. Đọc bộ dữ liệu đã hiệu chỉnh hoàn tất từ Bước 1 (240 Train + 60 Val = 300 mẫu).
2. Giải tham chiếu Symlink (Dereference symlinks) và nén ảnh PNG (1.3 GB) sang
   JPEG chất lượng cao (Quality=95, giữ nguyên độ phân giải 1944x1200) giúp giảm
   dung lượng từ 1.3 GB xuống ~75 MB để upload lên Kaggle chỉ mất vài giây!
3. Tổ chức thư mục gồm cả 2 bộ nhãn:
   - labels_seg/  (Nhãn phân đoạn thân quần cho YOLO11-seg)
   - labels_pose/ (Nhãn 12 mốc giải phẫu cho YOLO11-pose)
4. Đóng gói thành 1 file duy nhất:
   /home/nhan/RB3t_PJ/ai_src/step2_train_yolo/kaggle_dataset_dobot_cr10a.zip
"""

import os
import shutil
import zipfile
from pathlib import Path
import cv2
from concurrent.futures import ThreadPoolExecutor

STEP1_YOLO_DIR = Path("/home/nhan/RB3t_PJ/ai_src/step1_auto_label/dataset_yolo")
STEP2_DIR = Path("/home/nhan/RB3t_PJ/ai_src/step2_train_yolo")
BUILD_DIR = STEP2_DIR / "kaggle_build_tmp" / "dobot_cr10a_yolo_dataset"
OUTPUT_ZIP = STEP2_DIR / "kaggle_dataset_dobot_cr10a.zip"


def convert_single_image(args):
    src_img_path, dst_jpg_path = args
    img = cv2.imread(str(src_img_path))
    if img is None:
        raise RuntimeError(f"Không thể đọc ảnh: {src_img_path}")
    cv2.imwrite(str(dst_jpg_path), img, [cv2.IMWRITE_JPEG_QUALITY, 95])


def main():
    print("=" * 72)
    print("ĐÓNG GÓI BỘ DỮ LIỆU YOLO11 (SEG + POSE) CHO KAGGLE")
    print("=" * 72)

    if BUILD_DIR.parent.exists():
        shutil.rmtree(BUILD_DIR.parent)

    for split in ["train", "val"]:
        (BUILD_DIR / "images" / split).mkdir(parents=True, exist_ok=True)
        (BUILD_DIR / "labels_seg" / split).mkdir(parents=True, exist_ok=True)
        (BUILD_DIR / "labels_pose" / split).mkdir(parents=True, exist_ok=True)

    tasks = []
    stats = {"train": 0, "val": 0}

    for split in ["train", "val"]:
        img_dir = STEP1_YOLO_DIR / "images" / split
        seg_dir = STEP1_YOLO_DIR / "labels" / split
        pose_dir = STEP1_YOLO_DIR / "labels_pose" / split

        img_files = sorted(list(img_dir.glob("*.png")) + list(img_dir.glob("*.jpg")))
        for img_path in img_files:
            stem = img_path.stem
            real_img_path = img_path.resolve()
            dst_jpg = BUILD_DIR / "images" / split / f"{stem}.jpg"
            tasks.append((real_img_path, dst_jpg))

            # Copy nhãn Seg
            src_seg = seg_dir / f"{stem}.txt"
            if src_seg.exists():
                shutil.copy2(src_seg, BUILD_DIR / "labels_seg" / split / f"{stem}.txt")
            else:
                raise FileNotFoundError(f"Thiếu nhãn seg: {src_seg}")

            # Copy nhãn Pose
            src_pose = pose_dir / f"{stem}.txt"
            if src_pose.exists():
                shutil.copy2(src_pose, BUILD_DIR / "labels_pose" / split / f"{stem}.txt")
            else:
                raise FileNotFoundError(f"Thiếu nhãn pose: {src_pose}")

            stats[split] += 1

    print(f"-> Đang chuyển đổi {len(tasks)} ảnh (Train: {stats['train']}, Val: {stats['val']}) sang JPG Quality=95...")
    with ThreadPoolExecutor(max_workers=8) as executor:
        list(executor.map(convert_single_image, tasks))

    # Tạo file cấu hình mẫu bên trong gói dataset
    data_seg_content = """# Cấu hình YOLO11-seg cho bài toán phân đoạn thân quần Jeans (Dobot CR10A)
train: images/train
val: images/val

names:
  0: garment
"""
    (BUILD_DIR / "data_seg.yaml").write_text(data_seg_content, encoding="utf-8")

    data_pose_content = """# Cấu hình YOLO11-pose cho bài toán định vị 12 mốc giải phẫu quần Jeans (Dobot CR10A)
train: images/train
val: images/val

kpt_shape: [12, 3]
# Đối xứng trái-phải khi lật ảnh ngang (fliplr):
# 0:waist_left <-> 2:waist_right, 1:waist_center, 3:crotch,
# 4:thigh_left <-> 5:thigh_right, 6:knee_left <-> 7:knee_right,
# 8:cuff_left_outer <-> 11:cuff_right_outer, 9:cuff_left_inner <-> 10:cuff_right_inner
flip_idx: [2, 1, 0, 3, 5, 4, 7, 6, 11, 10, 9, 8]

names:
  0: pants
"""
    (BUILD_DIR / "data_pose.yaml").write_text(data_pose_content, encoding="utf-8")

    # Nén thành file .zip
    if OUTPUT_ZIP.exists():
        OUTPUT_ZIP.unlink()

    print(f"-> Đang nén thành gói {OUTPUT_ZIP.name} ...")
    with zipfile.ZipFile(OUTPUT_ZIP, "w", zipfile.ZIP_DEFLATED, compresslevel=6) as zf:
        for root, _, files in os.walk(BUILD_DIR):
            for file in sorted(files):
                full_path = Path(root) / file
                rel_path = full_path.relative_to(BUILD_DIR.parent)
                zf.write(full_path, rel_path)

    # Dọn thư mục tạm
    shutil.rmtree(BUILD_DIR.parent)

    size_mb = OUTPUT_ZIP.stat().st_size / (1024 * 1024)
    print("=" * 72)
    print(f"✓ HOÀN TẤT ĐÓNG GÓI DATASET KAGGLE!")
    print(f"  - Đường dẫn file Zip : {OUTPUT_ZIP}")
    print(f"  - Dung lượng         : {size_mb:.2f} MB (nhẹ hơn ~15 lần so với ảnh PNG gốc 1.3 GB)")
    print(f"  - Số mẫu Train / Val : {stats['train']} mẫu Train / {stats['val']} mẫu Val (Tổng: {stats['train'] + stats['val']} mẫu)")
    print("=" * 72)


if __name__ == "__main__":
    main()
