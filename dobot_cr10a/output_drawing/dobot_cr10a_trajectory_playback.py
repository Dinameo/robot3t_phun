#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
DOBOT CR10A REAL ROBOT EXECUTION SCRIPT (TCP/IP dobot_api + MovL + Per-Segment DO)
Ngay xuat: 2026-11-22 19:31:34
So Waypoint: 0 | TCP: (0.0, 0.0, 100.0) mm | V_TCP=220 mm/s
Luu y: Can thu vien chinh hang `dobot_api.py` (Dashboard: 29999, Move: 30003).
       Chay che do --dry-run truoc de kiem tra an toan.
"""

import sys
import time
import argparse

try:
    from dobot_api import DobotApiDashboard, DobotApiMove
    HAS_DOBOT_API = True
except ImportError:
    HAS_DOBOT_API = False

# Danh sach Waypoint: (x_mm, y_mm, z_mm, rx_deg, ry_deg, rz_deg, j1..j6_deg, spray_on)
WAYPOINTS = [
]

def main():
    parser = argparse.ArgumentParser(description='Dobot CR10A Spray Trajectory Runner')
    parser.add_argument('--ip', default='192.168.5.1', help='Robot IP')
    parser.add_argument('--dry-run', action='store_true', help='Mo phong khong gui lenh ra robot that')
    parser.add_argument('--use-joint', action='store_true', help='Dung JointMovJ thay vi MovL')
    args = parser.parse_args()

    dry = args.dry_run or (not HAS_DOBOT_API)
    if not HAS_DOBOT_API and not args.dry_run:
        print('[CANH BAO] Khong tim thay dobot_api.py -> Chuyen sang che do DRY-RUN!')

    dash = None
    move = None
    if not dry:
        dash = DobotApiDashboard(args.ip, 29999)
        move = DobotApiMove(args.ip, 30003)
        dash.ClearError()
        dash.EnableRobot()
        dash.SpeedFactor(30)
        dash.DO(1, 0)  # Dam bao van sung TAT ban dau

    spray_state = False
    try:
        for idx, wp in enumerate(WAYPOINTS):
            x, y, z, rx, ry, rz, j1, j2, j3, j4, j5, j6, want_spray = wp
            if want_spray != spray_state:
                spray_state = want_spray
                if dry:
                    print(f'[DO] Wp #{idx}: Van sung = {int(spray_state)}')
                else:
                    dash.DO(1, 1 if spray_state else 0)

            if dry:
                cmd = 'MovL' if (spray_state and not args.use_joint) else 'JointMovJ'
                print(f'[{cmd}] #{idx}: XYZ=({x:.1f},{y:.1f},{z:.1f}) RPY=({rx:.1f},{ry:.1f},{rz:.1f}) Spray={int(spray_state)}')
            else:
                if spray_state and not args.use_joint:
                    move.MovL(x, y, z, rx, ry, rz)
                else:
                    move.JointMovJ(j1, j2, j3, j4, j5, j6)
        if not dry:
            move.Sync()
    finally:
        if not dry and dash is not None:
            dash.DO(1, 0)  # Luon tat van sung khi ket thuc / dung khan
            move.JointMovJ(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
            dash.close()
            move.close()
        print('[CR10A] Hoan tat chu trinh an toan.')

if __name__ == '__main__':
    main()
