import os
import sys
import glob
import csv
import argparse
import numpy as np

# Ensure UTF-8 console output on Windows
if sys.stdout.encoding != 'utf-8':
    try:
        sys.stdout.reconfigure(encoding='utf-8')
    except Exception:
        pass

from extractor import PantsLandmarks3D


def calculate_distances(landmarks):
    """Tính các khoảng cách nhân trắc học hình học (mm) giữa các mốc."""
    def p3d(name):
        return np.array(landmarks[name]["coord_3d_mm"])

    # Bề rộng cạp (Waist width)
    waist_width = float(np.linalg.norm(p3d("waist_left") - p3d("waist_right")))

    # Chiều dài ống trái (Đũng đến gấu trái)
    cuff_left_mid = (p3d("cuff_left_outer") + p3d("cuff_left_inner")) / 2.0
    left_leg_length = float(np.linalg.norm(cuff_left_mid - p3d("crotch")))

    # Chiều dài ống phải (Đũng đến gấu phải)
    cuff_right_mid = (p3d("cuff_right_outer") + p3d("cuff_right_inner")) / 2.0
    right_leg_length = float(np.linalg.norm(cuff_right_mid - p3d("crotch")))

    # Khoảng cách giữa 2 gấu (Leg opening spread)
    leg_spread = float(np.linalg.norm(cuff_left_mid - cuff_right_mid))

    return {
        "waist_width_mm": round(waist_width, 1),
        "left_leg_length_mm": round(left_leg_length, 1),
        "right_leg_length_mm": round(right_leg_length, 1),
        "leg_spread_mm": round(leg_spread, 1)
    }


def main():
    parser = argparse.ArgumentParser(description="Trích xuất Mốc hình học 3D (Option 2 - Phương pháp P3)")
    parser.add_argument("--input", type=str, default=r"dataset\_3D", help="Đường dẫn file .ply hoặc thư mục chứa file .ply")
    parser.add_argument("--output", type=str, default=r"output_landmarks", help="Thư mục xuất kết quả")
    args = parser.parse_args()

    extractor = PantsLandmarks3D()
    os.makedirs(args.output, exist_ok=True)

    if os.path.isfile(args.input):
        print(f"Processing single file: {args.input}")
        res = extractor.process_file(args.input, args.output)
        if res:
            payload, overlay_path = res
            dist = calculate_distances(payload["landmarks"])
            print(f"[DONE] Landmarks saved to: {args.output}")
            print(f"  -> Waist width: {dist['waist_width_mm']} mm | Left leg length: {dist['left_leg_length_mm']} mm | Right leg length: {dist['right_leg_length_mm']} mm")
    elif os.path.isdir(args.input):
        ply_files = sorted(glob.glob(os.path.join(args.input, "*.ply")))
        print(f"Found {len(ply_files)} PLY files in {args.input}. Starting batch landmarks extraction...")

        summary_rows = []
        for idx, f in enumerate(ply_files, 1):
            fname = os.path.basename(f)
            print(f"[{idx}/{len(ply_files)}] Extracting landmarks for: {fname}...")
            try:
                res = extractor.process_file(f, args.output)
                if res:
                    payload, _ = res
                    lms = payload["landmarks"]
                    dist = calculate_distances(lms)
                    summary_rows.append({
                        "file": fname,
                        "crotch_x": lms["crotch"]["coord_3d_mm"][0],
                        "crotch_y": lms["crotch"]["coord_3d_mm"][1],
                        "crotch_z": lms["crotch"]["coord_3d_mm"][2],
                        "waist_width_mm": dist["waist_width_mm"],
                        "left_leg_length_mm": dist["left_leg_length_mm"],
                        "right_leg_length_mm": dist["right_leg_length_mm"],
                        "leg_spread_mm": dist["leg_spread_mm"],
                        "thigh_left_z": lms["thigh_left"]["coord_3d_mm"][2],
                        "thigh_right_z": lms["thigh_right"]["coord_3d_mm"][2],
                        "status": "SUCCESS"
                    })
            except Exception as e:
                print(f"  [ERROR] {fname}: {e}")
                summary_rows.append({"file": fname, "status": f"FAILED: {e}"})

        csv_path = os.path.join(args.output, "batch_landmarks_summary.csv")
        if summary_rows:
            with open(csv_path, "w", newline="", encoding="utf-8") as csv_f:
                writer = csv.DictWriter(csv_f, fieldnames=summary_rows[0].keys())
                writer.writeheader()
                writer.writerows(summary_rows)
            print(f"\n[DONE] Landmarks summary CSV saved to: {csv_path}")
    else:
        print(f"Input path not found: {args.input}")


if __name__ == "__main__":
    main()
