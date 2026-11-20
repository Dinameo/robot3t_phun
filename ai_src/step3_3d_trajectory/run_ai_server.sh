#!/usr/bin/env bash
# ==============================================================================
# KHỞI ĐỘNG MÁY CHỦ AI DOBOT CR10A KẾT NỐI UNITY (UDP:5005 / HTTP:5000 / AUTO:5006)
# ==============================================================================
pkill -f "unity_ai_server.py" 2>/dev/null || true
sleep 0.4
cd /home/nhan/RB3t_PJ/ai_src/step3_3d_trajectory || exit 1
exec /home/nhan/RB3t_PJ/ai_src/venv_ai/bin/python3 unity_ai_server.py "$@"
