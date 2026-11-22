#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Active-learning feedback & automatic YOLO11-pose retraining module.

Workflow:
1. When YOLO11-pose predicts inaccurate 12 landmarks in Unity, the operator corrects them
   on the 2D camera image and saves the corrected sample.
2. Each corrected sample (image + YOLO11-pose label .txt + metadata .json) is stored in:
       ai_src/step2_train_yolo/active_learning_feedback/pending/
3. Once pending corrected samples reach `threshold` (default: 10), `run_pose_retraining()`
   fine-tunes `ai_src/step2_train_yolo/weights/best_yolo11_pose.pt`, backs up the previous
   weights, moves pending samples into `history/` (retained for future retrains), resets
   `pending_count = 0`, and triggers model hot-reload in the AI server.
"""

from __future__ import annotations

import argparse
import base64
import json
import os
import re
import shutil
import threading
import time
from datetime import datetime
from pathlib import Path
from typing import Callable, Dict, List, Optional, Sequence, Tuple

import cv2
import numpy as np

STEP2_DIR = Path(__file__).resolve().parent
AI_SRC_DIR = STEP2_DIR.parent
PROJECT_ROOT = AI_SRC_DIR.parent

DEFAULT_WEIGHTS_PATH = STEP2_DIR / "weights" / "best_yolo11_pose.pt"
DEFAULT_FEEDBACK_DIR = STEP2_DIR / "active_learning_feedback"
DEFAULT_RETRAIN_THRESHOLD = 10

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
    "cuff_right_outer",
]
FLIP_IDX = [2, 1, 0, 3, 5, 4, 7, 6, 11, 10, 9, 8]

_MANIFEST_LOCK = threading.RLock()
_RETRAIN_LOCK = threading.Lock()


def _safe_sample_stem(sample_name: str) -> str:
    raw = Path(str(sample_name or "sample")).stem.strip()
    cleaned = re.sub(r"[^A-Za-z0-9_-]+", "_", raw).strip("_")
    if not cleaned:
        cleaned = f"sample_{int(time.time())}"
    return cleaned[:96]


def _ensure_feedback_dirs(feedback_dir: Optional[Path] = None) -> Dict[str, Path]:
    base = Path(feedback_dir) if feedback_dir else DEFAULT_FEEDBACK_DIR
    dirs = {
        "base": base,
        "pending_images": base / "pending" / "images",
        "pending_labels": base / "pending" / "labels",
        "pending_meta": base / "pending" / "meta",
        "history_images": base / "history" / "images",
        "history_labels": base / "history" / "labels",
        "history_meta": base / "history" / "meta",
        "manifest": base / "feedback_manifest.json",
    }
    for key, path in dirs.items():
        if key != "manifest":
            path.mkdir(parents=True, exist_ok=True)
    return dirs


def _list_paired_stems(images_dir: Path, labels_dir: Path) -> List[str]:
    if not images_dir.is_dir() or not labels_dir.is_dir():
        return []
    label_stems = {p.stem for p in labels_dir.glob("*.txt") if p.is_file()}
    paired = []
    seen = set()
    for ext in ("*.jpg", "*.jpeg", "*.png", "*.bmp"):
        for img_path in sorted(images_dir.glob(ext)):
            if img_path.stem in label_stems and img_path.stem not in seen:
                paired.append(img_path.stem)
                seen.add(img_path.stem)
    paired.sort()
    return paired


def _find_image_by_stem(images_dir: Path, stem: str) -> Optional[Path]:
    for ext in (".jpg", ".jpeg", ".png", ".bmp"):
        p = images_dir / f"{stem}{ext}"
        if p.is_file():
            return p
    return None


def _read_manifest_unlocked(dirs: Dict[str, Path], threshold: int = DEFAULT_RETRAIN_THRESHOLD) -> dict:
    manifest_path = dirs["manifest"]
    data = {}
    if manifest_path.is_file():
        try:
            data = json.loads(manifest_path.read_text(encoding="utf-8"))
        except Exception:
            data = {}

    pending_samples = _list_paired_stems(dirs["pending_images"], dirs["pending_labels"])
    history_samples = _list_paired_stems(dirs["history_images"], dirs["history_labels"])

    resolved_threshold = int(data.get("threshold", threshold) or threshold)
    if resolved_threshold < 1:
        resolved_threshold = DEFAULT_RETRAIN_THRESHOLD

    is_retraining = bool(data.get("is_retraining", False))
    # If lock is not held in this process and manifest claims retraining for > 2 hours, recover stale flag.
    if is_retraining and not _RETRAIN_LOCK.locked():
        started_ts = float(data.get("retrain_started_ts", 0.0) or 0.0)
        if started_ts > 0 and (time.time() - started_ts) > 7200:
            is_retraining = False

    total_retrains = int(data.get("total_retrains", 0) or 0)
    last_retrain_time = str(data.get("last_retrain_time", "") or "")
    last_retrain_status = str(data.get("last_retrain_status", "") or "")
    status_message = str(data.get("status_message", "") or "")
    if not status_message:
        status_message = f"Mẫu sửa mốc chờ train: {len(pending_samples)}/{resolved_threshold}."

    merged = {
        "threshold": resolved_threshold,
        "pending_count": len(pending_samples),
        "history_count": len(history_samples),
        "total_retrains": total_retrains,
        "is_retraining": is_retraining,
        "retrain_started_ts": float(data.get("retrain_started_ts", 0.0) or 0.0),
        "last_retrain_time": last_retrain_time,
        "last_retrain_status": last_retrain_status,
        "status_message": status_message,
        "pending_samples": pending_samples,
        "history_samples": history_samples,
    }
    return merged


def _write_manifest_unlocked(dirs: Dict[str, Path], manifest: dict) -> None:
    tmp_path = dirs["manifest"].with_suffix(".tmp")
    tmp_path.write_text(json.dumps(manifest, indent=2, ensure_ascii=False), encoding="utf-8")
    tmp_path.replace(dirs["manifest"])


def get_feedback_status(
    feedback_dir: Optional[Path] = None,
    threshold: int = DEFAULT_RETRAIN_THRESHOLD,
) -> dict:
    with _MANIFEST_LOCK:
        dirs = _ensure_feedback_dirs(feedback_dir)
        manifest = _read_manifest_unlocked(dirs, threshold=threshold)
        _write_manifest_unlocked(dirs, manifest)
        return {
            "status": "SUCCESS",
            "threshold": manifest["threshold"],
            "pending_count": manifest["pending_count"],
            "history_count": manifest["history_count"],
            "total_retrains": manifest["total_retrains"],
            "is_retraining": manifest["is_retraining"],
            "ready_to_retrain": manifest["pending_count"] >= manifest["threshold"],
            "last_retrain_time": manifest["last_retrain_time"],
            "last_retrain_status": manifest["last_retrain_status"],
            "status_message": manifest["status_message"],
            "pending_samples": manifest["pending_samples"],
        }


def _resolve_source_image(sample_name: str, image_path: Optional[str]) -> Optional[Path]:
    if image_path:
        p = Path(image_path)
        if p.is_file():
            return p

    raw_stem = Path(str(sample_name or "")).stem.strip()
    num_only = raw_stem.replace("pcl_", "")
    candidates = [
        AI_SRC_DIR / "dataset" / "_2D" / f"{raw_stem}.png",
        AI_SRC_DIR / "dataset" / "_2D" / f"{raw_stem}.jpg",
        AI_SRC_DIR / "dataset" / "_2D" / f"image_{num_only}.png",
        AI_SRC_DIR / "dataset" / "_2D" / f"image_{num_only}.jpg",
        PROJECT_ROOT / "dobot_cr10a" / "Assets" / "Resources" / f"{raw_stem}.png",
        PROJECT_ROOT / "dobot_cr10a" / "Assets" / "Resources" / f"{raw_stem}.jpg",
    ]
    for cand in candidates:
        if cand.is_file():
            return cand
    return None


def _format_yolo_pose_label(
    landmarks_uv: Sequence[Sequence[float]],
    bbox_uv: Optional[Sequence[float]] = None,
    class_id: int = 0,
) -> str:
    if len(landmarks_uv) != 12:
        raise ValueError(f"Cần đúng 12 mốc giải phẫu, nhận được {len(landmarks_uv)}.")

    pts = []
    for item in landmarks_uv:
        if isinstance(item, dict):
            u = float(item.get("u", item.get("x", 0.0)))
            v = float(item.get("v", item.get("y", 0.0)))
        else:
            u = float(item[0])
            v = float(item[1])
        pts.append((float(np.clip(u, 0.0, 1.0)), float(np.clip(v, 0.0, 1.0))))

    if bbox_uv is not None and len(bbox_uv) == 4:
        cx, cy, bw, bh = [float(np.clip(val, 0.0, 1.0)) for val in bbox_uv]
        bw = max(bw, 0.05)
        bh = max(bh, 0.05)
    else:
        us = [p[0] for p in pts]
        vs = [p[1] for p in pts]
        min_u, max_u = min(us), max(us)
        min_v, max_v = min(vs), max(vs)
        span_u = max(0.08, max_u - min_u)
        span_v = max(0.08, max_v - min_v)
        # Pad garment bounding box around the 12 keypoints so cuffs/waist edges stay inside box.
        pad_u = max(0.03, span_u * 0.10)
        pad_v = max(0.03, span_v * 0.08)
        x0 = max(0.0, min_u - pad_u)
        y0 = max(0.0, min_v - pad_v)
        x1 = min(1.0, max_u + pad_u)
        y1 = min(1.0, max_v + pad_v)
        bw = max(0.05, x1 - x0)
        bh = max(0.05, y1 - y0)
        cx = x0 + bw * 0.5
        cy = y0 + bh * 0.5

    kp_tokens = [f"{u:.6f} {v:.6f} 2" for (u, v) in pts]
    return f"{class_id} {cx:.6f} {cy:.6f} {bw:.6f} {bh:.6f} " + " ".join(kp_tokens) + "\n"


def save_corrected_landmark_sample(
    sample_name: str,
    landmarks_uv: Sequence[Sequence[float]],
    image_path: Optional[str] = None,
    image_b64: Optional[str] = None,
    bbox_uv: Optional[Sequence[float]] = None,
    threshold: int = DEFAULT_RETRAIN_THRESHOLD,
    feedback_dir: Optional[Path] = None,
) -> dict:
    """
    Save one operator-corrected 12-landmark sample into `pending/` for YOLO11-pose retraining.
    Deduplicates by sample stem so re-editing the same sample updates its label cleanly.
    """
    if not landmarks_uv or len(landmarks_uv) != 12:
        raise ValueError("Cần đúng 12 mốc tọa độ UV để lưu mẫu huấn luyện.")

    label_line = _format_yolo_pose_label(landmarks_uv, bbox_uv=bbox_uv, class_id=0)
    safe_stem = _safe_sample_stem(sample_name)

    with _MANIFEST_LOCK:
        dirs = _ensure_feedback_dirs(feedback_dir)

        # Decode or copy image into pending/images/<safe_stem>.jpg
        dst_img = dirs["pending_images"] / f"{safe_stem}.jpg"
        # Remove any previous file with different extension for the same stem
        for old_ext in (".png", ".jpeg", ".bmp"):
            old_p = dirs["pending_images"] / f"{safe_stem}{old_ext}"
            if old_p.is_file():
                old_p.unlink()

        img_w, img_h = 0, 0
        if image_b64:
            raw_b64 = image_b64.split(",", 1)[-1]
            img_bytes = base64.b64decode(raw_b64)
            arr = np.frombuffer(img_bytes, dtype=np.uint8)
            img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
            if img is None:
                raise ValueError("Không giải mã được dữ liệu ảnh base64.")
            img_h, img_w = img.shape[:2]
            cv2.imwrite(str(dst_img), img, [cv2.IMWRITE_JPEG_QUALITY, 95])
        else:
            src_img = _resolve_source_image(sample_name, image_path)
            if src_img is None:
                raise FileNotFoundError(
                    f"Không tìm thấy ảnh camera cho mẫu '{sample_name}'. "
                    "Hãy truyền image_path hợp lệ hoặc image_b64."
                )
            img = cv2.imread(str(src_img), cv2.IMREAD_COLOR)
            if img is None:
                raise ValueError(f"Không đọc được ảnh từ: {src_img}")
            img_h, img_w = img.shape[:2]
            cv2.imwrite(str(dst_img), img, [cv2.IMWRITE_JPEG_QUALITY, 95])

        dst_lbl = dirs["pending_labels"] / f"{safe_stem}.txt"
        dst_lbl.write_text(label_line, encoding="utf-8")

        meta_obj = {
            "sample": safe_stem,
            "original_sample_name": str(sample_name or safe_stem),
            "saved_at": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
            "image_width": img_w,
            "image_height": img_h,
            "landmarks_uv": [
                {
                    "id": idx,
                    "name": LANDMARK_NAMES[idx],
                    "u": round(float(pt["u"] if isinstance(pt, dict) else pt[0]), 6),
                    "v": round(float(pt["v"] if isinstance(pt, dict) else pt[1]), 6),
                }
                for idx, pt in enumerate(landmarks_uv)
            ],
        }
        dst_meta = dirs["pending_meta"] / f"{safe_stem}.json"
        dst_meta.write_text(json.dumps(meta_obj, indent=2, ensure_ascii=False), encoding="utf-8")

        manifest = _read_manifest_unlocked(dirs, threshold=threshold)
        ready = manifest["pending_count"] >= manifest["threshold"]
        if ready:
            manifest["status_message"] = (
                f"Đã lưu mẫu [{safe_stem}] ({manifest['pending_count']}/{manifest['threshold']}). "
                "Đủ ngưỡng, sẵn sàng train lại YOLO11-pose!"
            )
        else:
            manifest["status_message"] = (
                f"Đã lưu mẫu sửa mốc [{safe_stem}] ({manifest['pending_count']}/{manifest['threshold']})."
            )
        _write_manifest_unlocked(dirs, manifest)

        return {
            "status": "SUCCESS",
            "sample": safe_stem,
            "threshold": manifest["threshold"],
            "pending_count": manifest["pending_count"],
            "history_count": manifest["history_count"],
            "total_retrains": manifest["total_retrains"],
            "is_retraining": manifest["is_retraining"],
            "ready_to_retrain": ready,
            "status_message": manifest["status_message"],
            "saved_image": str(dst_img),
            "saved_label": str(dst_lbl),
        }


def _find_replay_pose_pairs(max_pairs: int = 30) -> List[Tuple[Path, Path]]:
    """
    Optionally gather existing YOLO11-pose labeled pairs from local dataset directories
    (if present on disk) to mix as replay buffer and prevent catastrophic forgetting.
    """
    candidates = [
        AI_SRC_DIR / "step1_auto_label" / "dataset_yolo",
        AI_SRC_DIR / "step2_train_yolo" / "dataset_yolo",
        AI_SRC_DIR / "dataset" / "yolo_pose",
    ]
    pairs: List[Tuple[Path, Path]] = []
    for root in candidates:
        if not root.is_dir():
            continue
        for split in ("train", "val"):
            img_dir = root / "images" / split
            lbl_dir = root / "labels_pose" / split
            if not lbl_dir.is_dir():
                lbl_dir = root / "labels" / split
            if not img_dir.is_dir() or not lbl_dir.is_dir():
                continue
            for stem in _list_paired_stems(img_dir, lbl_dir):
                img_p = _find_image_by_stem(img_dir, stem)
                lbl_p = lbl_dir / f"{stem}.txt"
                if img_p and lbl_p.is_file():
                    pairs.append((img_p, lbl_p))
                    if len(pairs) >= max_pairs:
                        return pairs
    return pairs


def run_pose_retraining(
    weights_path: Optional[Path] = None,
    feedback_dir: Optional[Path] = None,
    min_samples: int = DEFAULT_RETRAIN_THRESHOLD,
    force: bool = False,
    epochs: int = 25,
    imgsz: int = 640,
    batch: int = 4,
    lr0: float = 0.002,
    on_complete: Optional[Callable[[bool, str], None]] = None,
) -> dict:
    """
    Fine-tune `best_yolo11_pose.pt` using accumulated corrected samples in `pending/` (+ `history/`).
    Thread-safe: only one retraining job runs at a time.
    """
    weights = Path(weights_path) if weights_path else DEFAULT_WEIGHTS_PATH
    dirs = _ensure_feedback_dirs(feedback_dir)

    if not _RETRAIN_LOCK.acquire(blocking=False):
        return {
            "status": "BUSY",
            "message": "Đang có tiến trình train lại YOLO11-pose chạy nền.",
            "is_retraining": True,
        }

    try:
        with _MANIFEST_LOCK:
            manifest = _read_manifest_unlocked(dirs, threshold=min_samples)
            pending_stems = list(manifest["pending_samples"])
            history_stems = list(manifest["history_samples"])

        if not force and len(pending_stems) < min_samples:
            msg = f"Chưa đủ {min_samples} mẫu sai để tự động train lại (hiện có {len(pending_stems)}/{min_samples})."
            return {
                "status": "SKIPPED",
                "message": msg,
                "pending_count": len(pending_stems),
                "threshold": min_samples,
                "is_retraining": False,
            }

        if len(pending_stems) == 0 and len(history_stems) == 0:
            msg = "Không có mẫu sửa mốc nào để train lại."
            return {
                "status": "ERROR",
                "message": msg,
                "pending_count": 0,
                "is_retraining": False,
            }

        if not weights.is_file():
            raise FileNotFoundError(f"Không tìm thấy file trọng số gốc: {weights}")

        with _MANIFEST_LOCK:
            manifest = _read_manifest_unlocked(dirs, threshold=min_samples)
            manifest["is_retraining"] = True
            manifest["retrain_started_ts"] = time.time()
            manifest["status_message"] = (
                f"Đang train lại YOLO11-pose ({len(pending_stems)} mẫu mới + {len(history_stems)} mẫu cũ, {epochs} epochs)..."
            )
            _write_manifest_unlocked(dirs, manifest)

        # 1. Build clean temporary YOLO-pose dataset
        work_ds = dirs["base"] / "work_dataset"
        if work_ds.exists():
            shutil.rmtree(work_ds, ignore_errors=True)
        for sub in ("images/train", "labels/train", "images/val", "labels/val"):
            (work_ds / sub).mkdir(parents=True, exist_ok=True)

        pending_pairs: List[Tuple[str, Path, Path]] = []
        for stem in pending_stems:
            img_p = _find_image_by_stem(dirs["pending_images"], stem)
            lbl_p = dirs["pending_labels"] / f"{stem}.txt"
            if img_p and lbl_p.is_file():
                pending_pairs.append((stem, img_p, lbl_p))

        history_pairs: List[Tuple[str, Path, Path]] = []
        pending_set = set(pending_stems)
        for stem in history_stems:
            if stem in pending_set:
                continue  # Prefer newer pending correction if sample was re-corrected
            img_p = _find_image_by_stem(dirs["history_images"], stem)
            lbl_p = dirs["history_labels"] / f"{stem}.txt"
            if img_p and lbl_p.is_file():
                history_pairs.append((stem, img_p, lbl_p))

        replay_pairs = _find_replay_pose_pairs(max_pairs=30)

        def _copy_pair(split: str, out_stem: str, src_img: Path, src_lbl: Path):
            shutil.copy2(src_img, work_ds / "images" / split / f"{out_stem}{src_img.suffix.lower()}")
            shutil.copy2(src_lbl, work_ds / "labels" / split / f"{out_stem}.txt")

        train_count = 0
        val_count = 0

        # Put all pending samples into train (3x oversampling for strong correction signal)
        # and also keep 20% (min 2, up to all if small) in val so YOLO reports pose mAP cleanly.
        for idx, (stem, img_p, lbl_p) in enumerate(pending_pairs):
            for rep in range(3):
                _copy_pair("train", f"pend_{stem}_r{rep}", img_p, lbl_p)
                train_count += 1
            if idx % 5 == 0 or len(pending_pairs) <= 3:
                _copy_pair("val", f"pend_{stem}", img_p, lbl_p)
                val_count += 1

        # Add previously archived corrections (2x oversampling)
        for idx, (stem, img_p, lbl_p) in enumerate(history_pairs):
            for rep in range(2):
                _copy_pair("train", f"hist_{stem}_r{rep}", img_p, lbl_p)
                train_count += 1
            if idx % 5 == 0:
                _copy_pair("val", f"hist_{stem}", img_p, lbl_p)
                val_count += 1

        # Add local replay samples if available
        for idx, (img_p, lbl_p) in enumerate(replay_pairs):
            _copy_pair("train", f"replay_{idx:03d}_{img_p.stem}", img_p, lbl_p)
            train_count += 1
            if idx % 5 == 0:
                _copy_pair("val", f"replay_{idx:03d}_{img_p.stem}", img_p, lbl_p)
                val_count += 1

        # Ensure val is never empty
        if val_count == 0 and (pending_pairs or history_pairs):
            _, img_p, lbl_p = (pending_pairs[0] if pending_pairs else history_pairs[0])
            _copy_pair("val", "fallback_val_0", img_p, lbl_p)
            val_count += 1

        yaml_path = work_ds / "data_pose_retrain.yaml"
        yaml_text = (
            f"path: {work_ds.resolve().as_posix()}\n"
            "train: images/train\n"
            "val: images/val\n"
            "kpt_shape: [12, 3]\n"
            f"flip_idx: {FLIP_IDX}\n"
            "names:\n"
            "  0: pants\n"
        )
        yaml_path.write_text(yaml_text, encoding="utf-8")

        # 2. Backup existing best_yolo11_pose.pt before fine-tuning
        backups_dir = weights.parent / "backups"
        backups_dir.mkdir(parents=True, exist_ok=True)
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        backup_path = backups_dir / f"{weights.stem}_backup_{stamp}{weights.suffix}"
        shutil.copy2(weights, backup_path)

        # 3. Fine-tune YOLO11-pose
        os.environ.setdefault("YOLO_CONFIG_DIR", str(PROJECT_ROOT / "runtime" / "ultralytics"))
        import torch
        from ultralytics import YOLO

        device = 0 if torch.cuda.is_available() else "cpu"
        eff_batch = max(1, min(int(batch), train_count))
        runs_dir = dirs["base"] / "retrain_runs"
        run_name = f"retrain_{stamp}"

        model = YOLO(str(weights))
        model.train(
            data=str(yaml_path),
            epochs=int(epochs),
            imgsz=int(imgsz),
            batch=eff_batch,
            lr0=float(lr0),
            lrf=0.05,
            warmup_epochs=2.0,
            pose=15.0,
            fliplr=0.5,
            mosaic=0.0,
            patience=10,
            workers=0,
            device=device,
            project=str(runs_dir),
            name=run_name,
            exist_ok=True,
            verbose=False,
        )

        # 4. Deploy newly trained weights atomically
        trained_best = runs_dir / run_name / "weights" / "best.pt"
        if not trained_best.is_file():
            trained_best = runs_dir / run_name / "weights" / "last.pt"
        if not trained_best.is_file():
            raise FileNotFoundError(f"Train xong nhưng không thấy file trọng số mới tại {runs_dir / run_name / 'weights'}")

        tmp_weights = weights.with_suffix(".pt.new")
        shutil.copy2(trained_best, tmp_weights)
        tmp_weights.replace(weights)

        # 5. Archive pending samples into history/ so future retrains retain them
        moved_count = 0
        for stem, img_p, lbl_p in pending_pairs:
            shutil.copy2(img_p, dirs["history_images"] / img_p.name)
            shutil.copy2(lbl_p, dirs["history_labels"] / lbl_p.name)
            meta_src = dirs["pending_meta"] / f"{stem}.json"
            if meta_src.is_file():
                shutil.copy2(meta_src, dirs["history_meta"] / meta_src.name)
                meta_src.unlink()
            img_p.unlink(missing_ok=True)
            lbl_p.unlink(missing_ok=True)
            moved_count += 1

        # Clean up work_dataset to save disk space
        shutil.rmtree(work_ds, ignore_errors=True)

        finished_str = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        with _MANIFEST_LOCK:
            manifest = _read_manifest_unlocked(dirs, threshold=min_samples)
            manifest["is_retraining"] = False
            manifest["retrain_started_ts"] = 0.0
            manifest["total_retrains"] = int(manifest.get("total_retrains", 0)) + 1
            manifest["last_retrain_time"] = finished_str
            manifest["last_retrain_status"] = "SUCCESS"
            manifest["status_message"] = (
                f"Đã train lại YOLO11-pose thành công (lần #{manifest['total_retrains']}, "
                f"{moved_count} mẫu mới, device={device}). Đã cập nhật model mới!"
            )
            _write_manifest_unlocked(dirs, manifest)
            final_msg = manifest["status_message"]
            total_retrains = manifest["total_retrains"]

        if on_complete is not None:
            try:
                on_complete(True, final_msg)
            except Exception:
                pass

        return {
            "status": "SUCCESS",
            "message": final_msg,
            "trained_new_samples": moved_count,
            "total_retrains": total_retrains,
            "backup_weights": str(backup_path),
            "weights_path": str(weights),
            "is_retraining": False,
        }

    except Exception as ex:
        err_msg = f"Lỗi khi train lại YOLO11-pose: {ex}"
        with _MANIFEST_LOCK:
            manifest = _read_manifest_unlocked(dirs, threshold=min_samples)
            manifest["is_retraining"] = False
            manifest["retrain_started_ts"] = 0.0
            manifest["last_retrain_status"] = f"ERROR: {ex}"
            manifest["status_message"] = err_msg
            _write_manifest_unlocked(dirs, manifest)
        if on_complete is not None:
            try:
                on_complete(False, err_msg)
            except Exception:
                pass
        return {
            "status": "ERROR",
            "message": err_msg,
            "is_retraining": False,
        }
    finally:
        _RETRAIN_LOCK.release()


def main():
    parser = argparse.ArgumentParser(description="Active-learning feedback & YOLO11-pose retraining")
    parser.add_argument("--status", action="store_true", help="In trạng thái mẫu sai đang chờ train lại")
    parser.add_argument("--retrain", action="store_true", help="Chạy fine-tune YOLO11-pose từ các mẫu đã sửa")
    parser.add_argument("--force", action="store_true", help="Ép train lại ngay cả khi chưa đủ 10 mẫu")
    parser.add_argument("--min-samples", type=int, default=DEFAULT_RETRAIN_THRESHOLD)
    parser.add_argument("--epochs", type=int, default=25)
    parser.add_argument("--imgsz", type=int, default=640)
    parser.add_argument("--batch", type=int, default=4)
    parser.add_argument("--lr0", type=float, default=0.002)
    args = parser.parse_args()

    if args.retrain:
        res = run_pose_retraining(
            min_samples=args.min_samples,
            force=args.force,
            epochs=args.epochs,
            imgsz=args.imgsz,
            batch=args.batch,
            lr0=args.lr0,
        )
        print(json.dumps(res, ensure_ascii=False, indent=2))
        return

    status = get_feedback_status(threshold=args.min_samples)
    print(json.dumps(status, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
