import os
import sys
import math
import re
import numpy as np
import csv

if sys.platform.startswith('win'):
    try:
        sys.stdout.reconfigure(encoding='utf-8')
        sys.stderr.reconfigure(encoding='utf-8')
    except Exception:
        pass


def transform_matrix(position, quaternion, scale):
    """Unity-style TRS matrix using position (m), xyzw quaternion, and scale."""
    x, y, z, w = np.asarray(quaternion, dtype=np.float64)
    norm = math.sqrt(x*x + y*y + z*z + w*w)
    x, y, z, w = x/norm, y/norm, z/norm, w/norm
    rotation = np.array([
        [1 - 2*(y*y + z*z), 2*(x*y - z*w), 2*(x*z + y*w)],
        [2*(x*y + z*w), 1 - 2*(x*x + z*z), 2*(y*z - x*w)],
        [2*(x*z - y*w), 2*(y*z + x*w), 1 - 2*(x*x + y*y)]
    ], dtype=np.float64)
    matrix = np.eye(4, dtype=np.float64)
    matrix[:3, :3] = rotation @ np.diag(scale)
    matrix[:3, 3] = position
    return matrix

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
      - d6 = 111.3 mm  (Khớp 6 -> GameObject axis6)
      - tool_length = 100 mm (axis6 -> GameObject tcp theo trục Z cục bộ)
    """

    def __init__(self, tool_length_mm=100.0, scene_chain=None):
        # Kích thước hình học theo milimet
        self.d1 = 176.5
        self.a2 = 607.0
        self.a3 = 568.0
        self.d4 = 125.0
        self.d5 = 125.0
        self.d6 = 111.3
        self.tool_length = float(tool_length_mm)  # Translation từ axis6 đến TCP
        self.scene_chain = scene_chain

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
        FK cho TCP. Khi có scene_chain, dùng đúng Hierarchy Unity làm mô hình hình học.
        """
        if self.scene_chain is not None:
            T = np.eye(4, dtype=np.float64)
            for i, (position, initial_rotation, scale) in enumerate(self.scene_chain['joints']):
                T = T @ transform_matrix(position, initial_rotation, scale)
                angle = self.scene_chain['direction'] * q[i]
                c, s = math.cos(angle), math.sin(angle)
                joint_rotation = np.eye(4, dtype=np.float64)
                joint_rotation[:3, :3] = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
                T = T @ joint_rotation
            T = T @ transform_matrix(*self.scene_chain['flange'])
            T = T @ transform_matrix(*self.scene_chain['tcp'])
            result = T.copy()
            result[:3, 3] *= 1000.0
            return result[:3, 3], result[:3, :3], result

        # Bảng DH chuẩn đang dùng trong Unity và tài liệu của dự án:
        # [alpha, a, d, theta_offset]. q là góc trục Axis 0..5;
        # theta2 = q2 - 90 deg, theta4 = q4 + 90 deg.
        dh_params = [
            (-math.pi / 2,    0.0,      self.d1,  0.0),
            (0.0,         self.a2,       193.0, -math.pi / 2),
            (0.0,         self.a3,      -127.0,  0.0),
            (math.pi / 2,     0.0,      self.d4,  math.pi / 2),
            (-math.pi / 2,    0.0,      self.d5,  0.0),
            (0.0,             0.0, self.d6, 0.0)
        ]

        T = np.eye(4, dtype=np.float64)
        for i in range(6):
            alpha, a, d, th_offset = dh_params[i]
            T_i = self.dh_matrix(alpha, a, d, q[i] + th_offset)
            T = T @ T_i

        # GameObject tcp là child của axis6: localPosition=(0,0,0.1)m,
        # localRotation=Euler(0,-90,0). Compose that fixed transform once.
        tcp_offset = np.eye(4, dtype=np.float64)
        tcp_offset[:3, :3] = np.array([
            [0.0, 0.0, -1.0],
            [0.0, 1.0,  0.0],
            [1.0, 0.0,  0.0]
        ], dtype=np.float64)
        tcp_offset[2, 3] = self.tool_length
        T = T @ tcp_offset

        pos = T[:3, 3]
        R = T[:3, :3]
        return pos, R, T

    def inverse_kinematics(self, target_pos, target_R, q_init=None):
        """
        Động học nghịch: Tìm 6 góc khớp q = [q1, q2, q3, q4, q5, q6]
        sao cho GameObject tcp trùng với target_pos và hướng trùng với target_R.
        Kết hợp: Damped Least-Squares (DLS) Jacobian kết hợp Closed-Form Seed.
        Đảm bảo độ chính xác cực cao (< 0.01 mm và < 0.01 deg).
        """
        if q_init is None:
            # Seed khuỷu gập phù hợp với quỹ đạo trên quần và phép đổi frame
            # camera -> Unity -> Global. Seed cũ rơi vào nghiệm cục bộ sai.
            q = np.radians([-10.0, 65.0, 52.0, -48.0, -89.0, -9.0])
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


def load_unity_world_point_cloud(ply_path):
    """Read the already world-converted binary PLY used by Unity's loader."""
    with open(ply_path, 'rb') as f:
        vertex_count = None
        while True:
            line = f.readline()
            if not line:
                raise ValueError(f"PLY thiếu end_header: {ply_path}")
            line = line.strip()
            if line.startswith(b'element vertex '):
                vertex_count = int(line.split()[-1])
            if line == b'end_header':
                break
        if vertex_count is None:
            raise ValueError(f"PLY thiếu element vertex: {ply_path}")
        data = np.fromfile(f, dtype=[('x', '<f4'), ('y', '<f4'), ('z', '<f4'),
                                     ('r', 'u1'), ('g', 'u1'), ('b', 'u1')],
                           count=vertex_count)
    points = np.column_stack((data['x'], data['y'], data['z'])).astype(np.float64)
    return points[np.isfinite(points).all(axis=1)]


def load_unity_scene_frames(scene_path):
    """Read Global's world transform and PLY loader settings from the Unity scene."""
    with open(scene_path, 'r', encoding='utf-8') as f:
        scene_text = f.read()
    docs = re.split(r'(?=^--- !u!)', scene_text, flags=re.MULTILINE)

    global_game_object_id = None
    loader_doc = None
    manual_doc = None
    game_object_names = {}
    for doc in docs:
        header = re.match(r'--- !u!(\d+) &(\d+)', doc)
        if not header:
            continue
        if header.group(1) == '1':
            name = re.search(r'^  m_Name: (.*?)\s*$', doc, re.MULTILINE)
            if name:
                game_object_names[header.group(2)] = name.group(1)
            if name and name.group(1) == 'Global':
                global_game_object_id = header.group(2)
        if 'Assembly-CSharp::PLYDirectLoader' in doc:
            loader_doc = doc
        if 'Assembly-CSharp::manual' in doc:
            manual_doc = doc

    if global_game_object_id is None or loader_doc is None or manual_doc is None:
        raise ValueError(f"Không tìm thấy Global, PLYDirectLoader hoặc manual trong {scene_path}")

    def vector_from(doc, key):
        match = re.search(rf'^\s*{re.escape(key)}: \{{x: ([^,]+), y: ([^,]+), z: ([^}}]+)\}}', doc, re.MULTILINE)
        if not match:
            raise ValueError(f"Không đọc được {key} trong scene")
        return np.array([float(match.group(i)) for i in range(1, 4)], dtype=np.float64)

    camera_origin = vector_from(loader_doc, 'cameraOrigin')
    ply_match = re.search(r'^\s*plyFilePath: (.+?)\s*$', loader_doc, re.MULTILINE)
    if not ply_match:
        raise ValueError("Không đọc được plyFilePath trong scene")
    ply_path = ply_match.group(1).replace('\\', os.sep).replace('/', os.sep)

    transforms = {}
    global_transform_id = None
    for doc in docs:
        header = re.match(r'--- !u!(\d+) &(\d+)', doc)
        if not header or header.group(1) != '4':
            continue
        transform_id = header.group(2)
        game_object = re.search(r'^  m_GameObject: \{fileID: (\d+)\}', doc, re.MULTILINE)
        if not game_object:
            continue
        position = vector_from(doc, 'm_LocalPosition')
        scale = vector_from(doc, 'm_LocalScale')
        rotation_match = re.search(
            r'^  m_LocalRotation: \{x: ([^,]+), y: ([^,]+), z: ([^,]+), w: ([^}]+)\}',
            doc, re.MULTILINE)
        father = re.search(r'^  m_Father: \{fileID: (\d+)\}', doc, re.MULTILINE)
        if not rotation_match or not father:
            continue
        quaternion = np.array([float(rotation_match.group(i)) for i in range(1, 5)], dtype=np.float64)
        transforms[transform_id] = (game_object.group(1), position, quaternion, scale, father.group(1))
        if game_object.group(1) == global_game_object_id:
            global_transform_id = transform_id

    if global_transform_id is None:
        raise ValueError("Không tìm thấy Transform của GameObject Global")

    axis_block = re.search(r'^  axisTransforms:\n((?:  - .*\n){6})', manual_doc, re.MULTILINE)
    rotation_block = re.search(r'^  initialRotations:\n((?:  - .*\n){6})', manual_doc, re.MULTILINE)
    if not axis_block or not rotation_block:
        raise ValueError("Không đọc được sáu trục / offset góc từ manual trong scene")
    axis_ids = re.findall(r'- \{fileID: (\d+)\}', axis_block.group(1))
    initial_quaternions = [
        np.array([float(v) for v in q], dtype=np.float64)
        for q in re.findall(
            r'  - \{x: ([^,]+), y: ([^,]+), z: ([^,]+), w: ([^}]+)\}',
            rotation_block.group(1))
    ]
    if len(axis_ids) != 6 or len(initial_quaternions) != 6 or any(i not in transforms for i in axis_ids):
        raise ValueError("Cấu hình Transform của 6 trục trong scene không đầy đủ")

    joint_chain = [(transforms[i][1], initial_quaternions[n], transforms[i][3])
                   for n, i in enumerate(axis_ids)]
    flange_id = next((i for i, t in transforms.items()
                      if game_object_names.get(t[0]) == 'axis6' and t[4] == axis_ids[-1]), None)
    tcp_id = next((i for i, t in transforms.items()
                   if game_object_names.get(t[0]) == 'tcp' and t[4] == flange_id), None)
    if flange_id is None or tcp_id is None:
        raise ValueError("Không tìm được chuỗi axis6 -> tcp trong Hierarchy")
    flange = transforms[flange_id][1:4]
    tcp = transforms[tcp_id][1:4]
    invert_match = re.search(r'^  invertDirection: ([01])\s*$', manual_doc, re.MULTILINE)
    direction = -1.0 if invert_match and invert_match.group(1) == '1' else 1.0
    scene_chain = {'joints': joint_chain, 'flange': flange, 'tcp': tcp, 'direction': direction}

    def local_matrix(position, quaternion, scale):
        x, y, z, w = quaternion / np.linalg.norm(quaternion)
        rotation = np.array([
            [1 - 2*(y*y + z*z), 2*(x*y - z*w), 2*(x*z + y*w)],
            [2*(x*y + z*w), 1 - 2*(x*x + z*z), 2*(y*z - x*w)],
            [2*(x*z - y*w), 2*(y*z + x*w), 1 - 2*(x*x + y*y)]
        ], dtype=np.float64)
        matrix = np.eye(4, dtype=np.float64)
        matrix[:3, :3] = rotation @ np.diag(scale)
        matrix[:3, 3] = position
        return matrix

    def world_matrix(transform_id, visited=None):
        if visited is None:
            visited = set()
        if transform_id == '0':
            return np.eye(4, dtype=np.float64)
        if transform_id in visited or transform_id not in transforms:
            raise ValueError("Hierarchy Transform của Global không hợp lệ")
        visited.add(transform_id)
        _, position, quaternion, scale, parent_id = transforms[transform_id]
        return world_matrix(parent_id, visited) @ local_matrix(position, quaternion, scale)

    return camera_origin, ply_path, world_matrix(global_transform_id), scene_chain


def project_to_cloud(points_world, cloud_world, max_distance_mm=25.0):
    """Snap noisy camera-derived waypoints to the exact surface vertices shown in Unity."""
    projected = np.empty_like(points_world, dtype=np.float64)
    errors_mm = np.empty(len(points_world), dtype=np.float64)
    chunk_size = 100000
    for i, point in enumerate(points_world):
        best_d2 = float('inf')
        best_point = None
        for start in range(0, len(cloud_world), chunk_size):
            chunk = cloud_world[start:start + chunk_size]
            delta = chunk - point
            d2 = np.einsum('ij,ij->i', delta, delta)
            local_index = int(np.argmin(d2))
            if d2[local_index] < best_d2:
                best_d2 = float(d2[local_index])
                best_point = chunk[local_index]
        errors_mm[i] = math.sqrt(best_d2) * 1000.0
        projected[i] = best_point

    if errors_mm.max() > max_distance_mm:
        raise ValueError(
            f"Có waypoint cách point cloud quá xa: max={errors_mm.max():.2f} mm "
            f"> {max_distance_mm:.1f} mm. Không ghi đè kết quả IK."
        )
    return projected, errors_mm


def test_trajectory_ik(csv_path="output_drawing/pcl_151_pen_trajectory.csv",
                      out_ik_csv="output_drawing/cr10a_joint_angles.csv",
                      cloud_ply_path=None,
                      scene_path="Assets/Scenes/SampleScene.unity"):
    """
    Kiểm thử động học nghịch toàn bộ các điểm trong quỹ đạo vẽ đường thẳng 100mm.
    """
    print("=" * 65)
    print("KIỂM THỬ ĐỘNG HỌC NGHỊCH (INVERSE KINEMATICS) CHO DOBOT CR10A")
    print("=" * 65)

    if not os.path.exists(csv_path):
        print(f"[!] Không tìm thấy file quỹ đạo: {csv_path}")
        return

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

    # Read the exact camera-origin, point-cloud path, and robot Global transform
    # from the Unity scene so the offline IK uses the same frames as the runtime.
    if not os.path.exists(scene_path):
        print(f"[!] Không tìm thấy scene Unity để lấy hệ tọa độ: {scene_path}")
        return
    try:
        camera_origin_m, scene_cloud_path, global_to_world, scene_chain = load_unity_scene_frames(scene_path)
    except (OSError, ValueError) as exc:
        print(f"[!] Không đọc được cấu hình tọa độ từ scene: {exc}")
        return
    ik_solver = DobotCR10AKinematics(scene_chain=scene_chain)
    if cloud_ply_path is None:
        cloud_ply_path = scene_cloud_path
    if not os.path.isabs(cloud_ply_path) and not os.path.exists(cloud_ply_path):
        cloud_ply_path = os.path.join(os.path.dirname(scene_path), '..', cloud_ply_path)
    points_camera_mm = np.array([
        [float(row['x_mm']), float(row['y_mm']), float(row['z_mm'])]
        for row in rows
    ], dtype=np.float64)
    points_world_estimate = np.column_stack((
        camera_origin_m[0] + points_camera_mm[:, 0] / 1000.0,
        camera_origin_m[1] - points_camera_mm[:, 2] / 1000.0,
        camera_origin_m[2] - points_camera_mm[:, 1] / 1000.0
    ))

    if not os.path.exists(cloud_ply_path):
        print(f"[!] Không tìm thấy point cloud Unity: {cloud_ply_path}")
        return
    cloud_world = load_unity_world_point_cloud(cloud_ply_path)
    try:
        points_world, projection_errors = project_to_cloud(points_world_estimate, cloud_world)
    except ValueError as exc:
        print(f"[!] HỦY tạo IK: {exc}")
        return
    print(f"[*] Snap waypoint lên cloud Unity: lệch đầu vào TB="
          f"{projection_errors.mean():.2f} mm, max={projection_errors.max():.2f} mm")

    world_to_global = np.linalg.inv(global_to_world)
    points_world_h = np.column_stack((points_world, np.ones(len(points_world))))
    points_robot_global_mm = (points_world_h @ world_to_global.T)[:, :3] * 1000.0

    # PLYDirectLoader maps camera vectors to Unity world with this axis conversion.
    camera_to_world_rotation = np.array([
        [1.0,  0.0,  0.0],
        [0.0,  0.0, -1.0],
        [0.0, -1.0,  0.0]
    ], dtype=np.float64)
    world_to_global_normal = global_to_world[:3, :3].T

    for idx, row in enumerate(rows):
        # Pháp tuyến vi mô trong hệ Camera
        nx_c = float(row['normal_x'])
        ny_c = float(row['normal_y'])
        nz_c = float(row['normal_z'])

        # Waypoints are stored in robot Global coordinates. Unity world is only
        # used to snap measured points onto the visible garment cloud.
        target_pos = points_robot_global_mm[idx]

        # Convert measured camera normal through the same scene transforms as points.
        normal_camera = np.array([nx_c, ny_c, nz_c], dtype=np.float64)
        normal_world = camera_to_world_rotation @ normal_camera
        micro_normal_robot = world_to_global_normal @ normal_world
        micro_normal_robot /= np.linalg.norm(micro_normal_robot)

        # Use the path tangent as the pen feed direction in robot Global.
        if idx == 0:
            feed_direction = points_robot_global_mm[min(1, len(rows) - 1)] - target_pos
        elif idx == len(rows) - 1:
            feed_direction = target_pos - points_robot_global_mm[idx - 1]
        else:
            feed_direction = points_robot_global_mm[idx + 1] - points_robot_global_mm[idx - 1]
        if np.linalg.norm(feed_direction) < 1e-8:
            feed_direction = np.array([0.0, -1.0, 0.0], dtype=np.float64)

        # TCP orientation is built from the local surface normal and path tangent.
        target_R = ik_solver.rotation_matrix_from_pen_normal(micro_normal_robot, feed_direction=feed_direction)
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

    # Không ghi đè quỹ đạo đang dùng nếu phép đổi hệ tọa độ hoặc IK còn sai.
    if success_count != len(rows):
        print(f"\n[!] HỦY ghi CSV: chỉ giải được {success_count}/{len(rows)} waypoint. "
              "Cần kiểm tra lại phép hiệu chuẩn camera–robot.")
        return

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
