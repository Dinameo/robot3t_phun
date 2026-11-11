#!/usr/bin/env bash
# Khởi chạy giao diện chỉnh sửa mốc giải phẫu 3D Dobot CR10A
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
python3 "$SCRIPT_DIR/landmark_editor_app.py" "$@"
