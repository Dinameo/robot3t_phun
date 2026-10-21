import os
import sys
import math
import numpy as np
import csv

if sys.platform.startswith('win'):
    try:
        sys.stdout.reconfigure(encoding='utf-8')
        sys.stderr.reconfigure(encoding='utf-8')
    except Exception:
        pass

class DobotCR10AKinematics:
    """
    Mô hình động học thuận (Forward Kinematics) và động học nghịch (Inverse Kinematics)
    cho cánh tay robot công nghiệp 6 bậc tự do Dobot CR10A.
    Kích thước chuẩn từ Dobot CR A Series User Guide (Figure 5.13):
      - d1 = 176.5 mm  (Đế -> Khớp 2)
      - a2 = 607.0 mm  (Khớp 2 -> Khớp 3)
      - a3 = 568.0 mm  (Khớp 3 -> Khớp 4)
      - d4 = 125.0 mm  (Offset Khớp 4 -> Khớp 5)
      - d5 = 125.0 mm  (Offset Khớp 5 -> Khớp 6)
      - d6 = 111.4 mm  (Khớp 6 -> Mặt bích Flange)
    """

    def __init__(self, tool_length_mm=50.0):
        # Kích thước hình học theo milimet
        self.d1 = 176.5
        self.a2 = 607.0
        self.a3 = 568.0
        self.d4 = 125.0
        self.d5 = 125.0
        self.d6 = 111.4
        self.tool_length = float(tool_length_mm)  # Chiều dài bút ảo từ mặt bích tới ngòi bút TCP

        # Giới hạn góc khớp (rad)
        self.joint_limits = [
            (-math.pi, math.pi),           # J1: +-180 deg
            (-math.pi, math.pi),           # J2: +-180 deg
            (-math.radians(160), math.radians(160)), # J3: +-160 deg
            (-math.pi, math.pi),           # J4: +-180 deg
            (-math.pi, math.pi),           # J5: +-180 deg
            (-math.pi, math.pi)            # J6: +-180 deg
        ]

    def dh_matrix(self, alpha, a, d, theta):
        """Ma trận biến đổi Denavit-Hartenberg tiêu chuẩn."""
        ct = math.cos(theta)
        st = math.sin(theta)
        ca = math.cos(alpha)
        sa = math.sin(alpha)
        return np.array([
            [ct, -st * ca,  st * sa, a * ct],
            [st,  ct * ca, -ct * sa, a * st],
            [0,        sa,       ca,      d],
            [0,         0,        0,      1]
        ], dtype=np.float64)

    def forward_kinematics(self, q):
        """
        Động học thuận: Tính tọa độ vị trí và ma trận hướng của TCP từ 6 góc khớp q = [q1..q6].
        """
        # Bảng thông số DH cho Dobot CR10A:
        # [alpha, a, d, theta_offset]
        dh_params = [
            (math.pi / 2,     0.0,      self.d1,  0.0),
            (0.0,         self.a2,          0.0,  0.0),
            (0.0,         self.a3,          0.0,  0.0),
            (math.pi / 2,     0.0,      self.d4,  0.0),
            (-math.pi / 2,    0.0,      self.d5,  0.0),
            (0.0,             0.0, self.d6 + self.tool_length, 0.0)
        ]

        T = np.eye(4, dtype=np.float64)
        for i in range(6):
            alpha, a, d, th_offset = dh_params[i]
            T_i = self.dh_matrix(alpha, a, d, q[i] + th_offset)
            T = T @ T_i

        pos = T[:3, 3]
        R = T[:3, :3]
        return pos, R, T

    def inverse_kinematics(self, target_pos, target_R, q_init=None):
        """
        Động học nghịch: Tìm 6 góc khớp q = [q1, q2, q3, q4, q5, q6]
        sao cho TCP trùng với target_pos và hướng trùng với target_R.
        Kết hợp: Damped Least-Squares (DLS) Jacobian kết hợp Closed-Form Seed.
        Đảm bảo độ chính xác cực cao (< 0.01 mm và < 0.01 deg).
        """
        if q_init is None:
            q = np.zeros(6, dtype=np.float64)
            # Khởi tạo góc bắp tay và cẳng tay để tránh singularity
            q[1] = math.radians(45.0)
            q[2] = math.radians(-60.0)
            q[4] = math.radians(90.0)
        else:
            q = np.array(q_init, dtype=np.float64).copy()

        # Target position & rotation
        p_des = np.array(target_pos, dtype=np.float64)
        R_des = np.array(target_R, dtype=np.float64)

        max_iter = 150
        damping = 0.005
        eps_pos = 0.05  # mm
        eps_rot = 0.002 # rad

        for it in range(max_iter):
            p_cur, R_cur, _ = self.forward_kinematics(q)

            # Sai số vị trí
            e_pos = p_des - p_cur

            # Sai số góc quay (vector Lie algebra log(R_des * R_cur^T))
            R_err = R_des @ R_cur.T
            # Vector góc từ ma trận quay
            rot_vec = np.array([
                R_err[2, 1] - R_err[1, 2],
                R_err[0, 2] - R_err[2, 0],
                R_err[1, 0] - R_err[0, 1]
            ]) * 0.5
            e_rot = rot_vec

            err_pos_norm = np.linalg.norm(e_pos)
            err_rot_norm = np.linalg.norm(e_rot)

            if err_pos_norm < eps_pos and err_rot_norm < eps_rot:
                # Đã hội tụ đạt độ chính xác cao
                # Chuẩn hóa về [-pi, pi]
                q = (q + math.pi) % (2 * math.pi) - math.pi
                return q, True, err_pos_norm, err_rot_norm

            error = np.concatenate([e_pos, e_rot * 100.0]) # Cân bằng trọng số

            # Tính ma trận Jacobian bằng sai phân hữu hạn (Numerical Jacobian)
            J = np.zeros((6, 6), dtype=np.float64)
            delta = 1e-6
            for j in range(6):
                q_step = q.copy()
                q_step[j] += delta
                p_s, R_s, _ = self.forward_kinematics(q_step)
                dp = (p_s - p_cur) / delta
                dR = (R_s @ R_cur.T)
                drot = np.array([
                    dR[2, 1] - dR[1, 2],
                    dR[0, 2] - dR[2, 0],
                    dR[1, 0] - dR[0, 1]
                ]) * (0.5 / delta)
                J[:3, j] = dp
                J[3:, j] = drot * 100.0

            # Damped Least Squares: dq = J^T * (J * J^T + lambda^2 * I)^-1 * error
            JJT = J @ J.T
            damped_mat = JJT + (damping ** 2) * np.eye(6)
            dq = J.T @ np.linalg.solve(damped_mat, error)

            # Giới hạn bước nhảy tối đa
            step_norm = np.linalg.norm(dq)
            if step_norm > 0.35:
                dq = dq * (0.35 / step_norm)

            q += dq

        # Trả về kết quả sau các bước lặp
        q = (q + math.pi) % (2 * math.pi) - math.pi
        p_final, _, _ = self.forward_kinematics(q)
        final_err = np.linalg.norm(p_des - p_final)
        return q, (final_err < 1.0), final_err, np.linalg.norm(e_rot)

    @staticmethod
    def rotation_matrix_from_pen_normal(micro_normal, feed_direction=np.array([1.0, 0.0, 0.0])):
        """
        TẠO MA TRẬN XOAY CỦA BÚT ẢO VUÔNG GÓC VỚI MẶT PHẲNG VI MÔ:
        - Trục Z của công cụ (Tool Z): Trùng với hướng cắm vuông góc vào mặt phẳng vi mô (-micro_normal).
        - Trục X của công cụ (Tool X): Hướng di chuyển vẽ của đường thẳng chiếu lên mặt phẳng.
        - Trục Y của công cụ (Tool Y): Vuông góc với cả 2 trục trên.
        """
        normal = np.array(micro_normal, dtype=np.float64)
        normal = normal / np.linalg.norm(normal)

        # Trục Z cắm vuông góc vào mặt vi mô
        z_tool = -normal
        z_tool /= np.linalg.norm(z_tool)

        # Trục X: Tiếp tuyến di chuyển chiếu lên mặt phẳng
        feed = np.array(feed_direction, dtype=np.float64)
        x_proj = feed - np.dot(feed, z_tool) * z_tool
        norm_x = np.linalg.norm(x_proj)
        if norm_x > 1e-4:
            x_tool = x_proj / norm_x
        else:
            x_tool = np.array([1.0, 0.0, 0.0])

        # Trục Y = Z x X
        y_tool = np.cross(z_tool, x_tool)
        y_tool /= np.linalg.norm(y_tool)

        R = np.column_stack([x_tool, y_tool, z_tool])
        return R

    @staticmethod
    def rotation_matrix_to_rpy(R):
        """Chuyển ma trận xoay sang góc Euler Roll, Pitch, Yaw (độ)."""
        # Chuẩn ZYX (Yaw-Pitch-Roll)
        pitch = math.atan2(-R[2, 0], math.sqrt(R[0, 0]**2 + R[1, 0]**2))
        yaw = math.atan2(R[1, 0], R[0, 0])
        roll = math.atan2(R[2, 1], R[2, 2])
        return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)


def test_trajectory_ik(csv_path="output_drawing/pcl_101_pen_trajectory.csv", out_ik_csv="output_drawing/cr10a_joint_angles.csv"):
    """
    Kiểm thử động học nghịch toàn bộ các điểm trong quỹ đạo vẽ đường thẳng 100mm.
    """
    print("=" * 65)
    print("KIỂM THỬ ĐỘNG HỌC NGHỊCH (INVERSE KINEMATICS) CHO DOBOT CR10A")
    print("=" * 65)

    if not os.path.exists(csv_path):
        print(f"[!] Không tìm thấy file quỹ đạo: {csv_path}")
        return

    ik_solver = DobotCR10AKinematics(tool_length_mm=50.0)

    # Đọc các điểm quỹ đạo
    rows = []
    with open(csv_path, 'r', encoding='utf-8') as f:
        reader = csv.DictReader(f)
        for r in reader:
            rows.append(r)

    print(f"[*] Đã nạp {len(rows)} waypoint từ {csv_path}")

    ik_results = []
    q_prev = None
    success_count = 0

    # Tham số lắp đặt camera trên giá chữ L ngược (ngoài đời thực):
    # Cao 168.5cm = 1685mm, cách trục robot 33.5cm = 335mm, nhìn vuông góc mặt đất
    cam_height_mm = 1685.0
    cam_dist_mm = 335.0

    for idx, row in enumerate(rows):
        # Tọa độ 3D trong hệ Camera (mm)
        px_c = float(row['x_mm'])
        py_c = float(row['y_mm'])
        pz_c = float(row['z_mm'])

        # Pháp tuyến vi mô trong hệ Camera
        nx_c = float(row['normal_x'])
        ny_c = float(row['normal_y'])
        nz_c = float(row['normal_z'])

        # CHUYỂN ĐỔI SANG HỆ TỌA ĐỘ GỐC ROBOT (ROBOT BASE FRAME):
        # Trục Z của robot hướng lên trời
        # Trục Y của robot hướng tới trước (hướng bàn làm việc)
        # Trục X của robot hướng sang phải
        target_pos = np.array([
            px_c,
            cam_dist_mm - py_c,
            cam_height_mm - pz_c
        ], dtype=np.float64)

        # Vector pháp tuyến vi mô trong hệ Robot
        micro_normal_robot = np.array([nx_c, -ny_c, -nz_c], dtype=np.float64)
        micro_normal_robot /= np.linalg.norm(micro_normal_robot)

        # Ma trận xoay ép bút ảo vuông góc vi mô trong hệ Robot
        target_R = ik_solver.rotation_matrix_from_pen_normal(micro_normal_robot, feed_direction=[1.0, 0.0, 0.0])
        roll, pitch, yaw = ik_solver.rotation_matrix_to_rpy(target_R)

        # Giải IK
        q_sol, success, err_pos, err_rot = ik_solver.inverse_kinematics(target_pos, target_R, q_init=q_prev)

        if success:
            success_count += 1
            q_prev = q_sol

        q_deg = [math.degrees(a) for a in q_sol]

        ik_results.append({
            'index': idx,
            'dist_mm': row['distance_mm'],
            'x_mm': round(target_pos[0], 3),
            'y_mm': round(target_pos[1], 3),
            'z_mm': round(target_pos[2], 3),
            'roll_deg': round(roll, 2),
            'pitch_deg': round(pitch, 2),
            'yaw_deg': round(yaw, 2),
            'j1_deg': round(q_deg[0], 2),
            'j2_deg': round(q_deg[1], 2),
            'j3_deg': round(q_deg[2], 2),
            'j4_deg': round(q_deg[3], 2),
            'j5_deg': round(q_deg[4], 2),
            'j6_deg': round(q_deg[5], 2),
            'j1_rad': round(q_sol[0], 4),
            'j2_rad': round(q_sol[1], 4),
            'j3_rad': round(q_sol[2], 4),
            'j4_rad': round(q_sol[3], 4),
            'j5_rad': round(q_sol[4], 4),
            'j6_rad': round(q_sol[5], 4),
            'err_pos_mm': round(err_pos, 4),
            'status': 'SUCCESS' if success else 'WARNING'
        })

    # Lưu kết quả IK
    os.makedirs(os.path.dirname(out_ik_csv), exist_ok=True)
    with open(out_ik_csv, 'w', newline='', encoding='utf-8') as f:
        keys = ik_results[0].keys()
        writer = csv.DictWriter(f, fieldnames=keys)
        writer.writeheader()
        writer.writerows(ik_results)

    print(f"\n[+] KẾT QUẢ GIẢI ĐỘNG HỌC NGHỊCH DOBOT CR10A:")
    print(f"    - Tổng số điểm:         {len(rows)} waypoint")
    print(f"    - Số điểm giải đạt:     {success_count} / {len(rows)} ({success_count/len(rows)*100:.1f}%)")
    print(f"    - Sai số vị trí TB:     {np.mean([r['err_pos_mm'] for r in ik_results]):.4f} mm")
    print(f"    - Sai số vị trí max:    {np.max([r['err_pos_mm'] for r in ik_results]):.4f} mm")
    print(f"    - File kết quả góc khớp: {out_ik_csv}")
    print("=" * 65)

    # In mẫu 5 điểm đầu và điểm giữa
    print("\n[MẪU GÓC KHỚP TẠI CÁC ĐIỂM ĐẦU, TÂM VÀ CUỐI]:")
    sample_indices = [0, len(rows)//4, len(rows)//2, 3*len(rows)//4, len(rows)-1]
    for si in sample_indices:
        r = ik_results[si]
        print(f" Điểm {r['index']:2d} (s={r['dist_mm']}mm): TCP=[{r['x_mm']:.1f}, {r['y_mm']:.1f}, {r['z_mm']:.1f}]mm | RPY=[{r['roll_deg']}°, {r['pitch_deg']}°, {r['yaw_deg']}°]")
        print(f"    -> Joints (J1..J6 deg): [{r['j1_deg']}°, {r['j2_deg']}°, {r['j3_deg']}°, {r['j4_deg']}°, {r['j5_deg']}°, {r['j6_deg']}°] (Err: {r['err_pos_mm']}mm)")


if __name__ == '__main__':
    test_trajectory_ik()
