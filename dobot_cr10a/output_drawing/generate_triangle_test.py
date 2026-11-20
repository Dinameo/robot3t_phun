"""Generate a CR10A Cartesian triangle trajectory using the project's analytic IK."""
import csv
import math
from pathlib import Path

import numpy as np


DH = [
    (0.0, -math.pi / 2, 176.5, 0.0),
    (607.0, 0.0, 193.0, -math.pi / 2),
    (568.0, 0.0, -127.0, 0.0),
    (0.0, math.pi / 2, 125.0, math.pi / 2),
    (0.0, -math.pi / 2, 125.0, 0.0),
    (0.0, 0.0, 111.3, 0.0),
]
S = np.diag([1.0, -1.0, 1.0])


def wrap(x):
    return math.atan2(math.sin(x), math.cos(x))


def dh_step(i, q):
    a, alpha, d, offset = DH[i]
    t = q + offset
    ct, st, ca, sa = math.cos(t), math.sin(t), math.cos(alpha), math.sin(alpha)
    return np.array([
        [ct, -st * ca, st * sa, a * ct],
        [st, ct * ca, -ct * sa, a * st],
        [0.0, sa, ca, d],
        [0.0, 0.0, 0.0, 1.0],
    ])


def through(q, count=6):
    result = np.eye(4)
    for i in range(count):
        result = result @ dh_step(i, q[i])
    return result


def rpy_matrix(roll, pitch, yaw):
    r, p, y = np.radians([roll, pitch, yaw])
    rx = np.array([[1, 0, 0], [0, math.cos(r), -math.sin(r)], [0, math.sin(r), math.cos(r)]])
    ry = np.array([[math.cos(p), 0, math.sin(p)], [0, 1, 0], [-math.sin(p), 0, math.cos(p)]])
    rz = np.array([[math.cos(y), -math.sin(y), 0], [math.sin(y), math.cos(y), 0], [0, 0, 1]])
    return rz @ ry @ rx


def pose_dh(xyz_unity, rpy):
    result = np.eye(4)
    result[:3, :3] = S @ rpy_matrix(*rpy) @ S
    result[:3, 3] = [xyz_unity[0], -xyz_unity[1], xyz_unity[2]]
    return result


def inverse_rigid(t):
    result = np.eye(4)
    result[:3, :3] = t[:3, :3].T
    result[:3, 3] = -result[:3, :3] @ t[:3, 3]
    return result


def inverse_point(t, p):
    return t[:3, :3].T @ (p - t[:3, 3])


def arm_solutions(point, a3_length, enforce_plane=True):
    result = []
    h, a2 = 193.0 - 127.0 + 125.0, 607.0
    rho = math.hypot(point[0], point[1])
    if rho < abs(h):
        return result
    phi = math.atan2(point[1], point[0])
    offset = math.asin(max(-1.0, min(1.0, h / rho)))
    for q1 in (wrap(phi - offset), wrap(phi - math.pi + offset)):
        p14 = inverse_point(dh_step(0, q1), point)
        x, y = p14[:2]
        radius = math.hypot(x, y)
        if radius < 1e-9:
            continue
        cos_q3 = (radius**2 - a2**2 - a3_length**2) / (2 * a2 * a3_length)
        if abs(cos_q3) > 1 + 1e-6:
            continue
        q3_magnitude = math.acos(max(-1.0, min(1.0, cos_q3)))
        heading = math.atan2(y, x)
        for q3 in (q3_magnitude, -q3_magnitude):
            theta2 = wrap(heading - math.atan2(a3_length * math.sin(q3), a2 + a3_length * math.cos(q3)))
            q2 = wrap(theta2 + math.pi / 2)
            p24 = inverse_point(dh_step(1, q2), p14)
            if enforce_plane and abs(p24[2] - (-127.0 + 125.0)) > 2.0:
                continue
            result.append(np.array([q1, q2, wrap(math.atan2(p24[1], p24[0]))]))
    return result


def set_wrist(q, target_r, wrist_sign):
    r36 = through(q, 3)[:3, :3].T @ target_r
    beta = math.acos(max(-1.0, min(1.0, r36[2, 2])))
    b = wrist_sign * beta
    sb = math.sin(b)
    if abs(sb) < 1e-8:
        a, c = math.atan2(r36[1, 0], r36[0, 0]), 0.0
    else:
        a = math.atan2(r36[1, 2] / sb, r36[0, 2] / sb)
        c = math.atan2(r36[2, 1] / sb, -r36[2, 0] / sb)
    q[3], q[4], q[5] = wrap(a - math.pi / 2), wrap(-b), wrap(c)


def solve_flange(target):
    target_r = target[:3, :3]
    p05 = target[:3, 3] - 111.3 * target_r[:, 2]
    seeds = []
    for q4_seed in (-math.pi, -3 * math.pi / 4, -math.pi / 2, -math.pi / 4, 0,
                    math.pi / 4, math.pi / 2, 3 * math.pi / 4):
        seeds.extend(arm_solutions(p05, 568.0 + 125.0 * math.cos(q4_seed), False))
    solutions = []
    for seed in seeds:
        for wrist_sign in (1.0, -1.0):
            q = np.zeros(6)
            q[:3] = seed
            set_wrist(q, target_r, wrist_sign)
            for _ in range(20):
                t04 = through(q, 4)
                p04 = p05 - 125.0 * t04[:3, 2]
                arms = arm_solutions(p04, 568.0, True)
                if not arms:
                    break
                selected = min(arms, key=lambda a: sum(wrap(a[j] - q[j])**2 for j in range(3)))
                converged = sum(wrap(selected[j] - q[j])**2 for j in range(3)) < 1e-14
                q[:3] = selected
                set_wrist(q, target_r, wrist_sign)
                if converged:
                    break
            check = through(q)
            pos_error = float(np.linalg.norm(target[:3, 3] - check[:3, 3]))
            rot_delta = target_r @ check[:3, :3].T
            angle = math.acos(max(-1.0, min(1.0, (np.trace(rot_delta) - 1) / 2)))
            if pos_error < 0.1 and angle < math.radians(0.1):
                if not any(np.linalg.norm([wrap(q[j] - old[j]) for j in range(6)]) < 1e-5 for old in solutions):
                    solutions.append(q.copy())
    return solutions


def solve_tcp(xyz, rpy):
    tcp_local = np.eye(4)
    # SampleScene's tcp child has a -90 degree local Y rotation and a 100 mm Z offset.
    tcp_local[:3, :3] = S @ rpy_matrix(0.0, -90.0, 0.0) @ S
    tcp_local[2, 3] = 100.0
    target_flange = pose_dh(xyz, rpy) @ inverse_rigid(tcp_local)
    return solve_flange(target_flange)


def main():
    # Flat, 120 mm triangle in the robot's horizontal XY plane, with TCP Z down.
    vertices = np.array([[700.0, 200.0, 300.0], [820.0, 200.0, 300.0],
                         [760.0, 303.923, 300.0], [700.0, 200.0, 300.0]])
    rpy = (139.5858, 90.0, -40.4142)
    targets = []
    points_per_side = 20
    for side in range(3):
        for step in range(points_per_side):
            t = step / points_per_side
            targets.append(vertices[side] * (1 - t) + vertices[side + 1] * t)
    targets.append(vertices[-1])

    rows = []
    previous = None
    distance = 0.0
    previous_xyz = None
    for index, xyz in enumerate(targets):
        candidates = solve_tcp(xyz, rpy)
        if not candidates:
            raise RuntimeError(f"IK failed at triangle point {index}: {xyz.tolist()}")
        q = min(candidates, key=lambda candidate: 0.0 if previous is None else
                sum(wrap(candidate[j] - previous[j])**2 for j in range(6)))
        if previous_xyz is not None:
            distance += float(np.linalg.norm(xyz - previous_xyz))
        degrees = np.degrees(q)
        tcp_local = np.eye(4)
        tcp_local[:3, :3] = S @ rpy_matrix(0.0, -90.0, 0.0) @ S
        tcp_local[2, 3] = 100.0
        target_flange = pose_dh(xyz, rpy) @ inverse_rigid(tcp_local)
        position_error = float(np.linalg.norm(through(q)[:3, 3] - target_flange[:3, 3]))
        row = [index, distance, *xyz, *rpy, *degrees, *q, position_error, "SUCCESS"]
        rows.append(row)
        previous, previous_xyz = q, xyz

    output = Path(__file__).with_name("triangle_test_cr10a.csv")
    header = ["index", "dist_mm", "x_mm", "y_mm", "z_mm", "roll_deg", "pitch_deg", "yaw_deg"]
    header += [f"j{i}_deg" for i in range(1, 7)] + [f"j{i}_rad" for i in range(1, 7)]
    header += ["err_pos_mm", "status"]
    with output.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(header)
        writer.writerows([[f"{value:.8f}" if isinstance(value, (float, np.floating)) else value
                           for value in row] for row in rows])
    print(f"Wrote {len(rows)} points to {output}")
    print(f"Perimeter: {distance:.1f} mm; all IK points converged; max flange position error: "
          f"{max(float(row[-2]) for row in rows):.4f} mm.")


if __name__ == "__main__":
    main()
