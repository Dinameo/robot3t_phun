"""Generate a slow, closed CR10A path around the isolated garment outline."""

import csv
import math
from pathlib import Path

import cv2
import numpy as np

from solve_ik_cr10a import (
    DobotCR10AKinematics,
    load_unity_scene_frames,
    transform_matrix,
)


ROOT = Path(__file__).resolve().parent
PROJECT = ROOT / "dobot_cr10a"
MESH_PATH = PROJECT / "Assets" / "PointCloud" / "garment_3d.obj"
SCENE_PATH = PROJECT / "Assets" / "Scenes" / "SampleScene.unity"
SEED_CSV = ROOT / "output_drawing" / "cr10a_joint_angles.csv"
OUTPUTS = [
    ROOT / "output_drawing" / "pants_outline_cr10a.csv",
    PROJECT / "output_drawing" / "pants_outline_cr10a.csv",
]
PREVIEW_PATH = ROOT / "output_drawing" / "pants_outline_trajectory_preview.png"

RASTER_RESOLUTION_M = 0.002
PATH_SPACING_M = 0.005
EDGE_INSET_KERNEL_PX = 3
SURFACE_SEARCH_RADIUS_M = 0.008


def read_obj_vertices(path):
    vertices = []
    with path.open("r", encoding="utf-8", errors="ignore") as stream:
        for line in stream:
            if line.startswith("v "):
                fields = line.split()
                vertices.append((float(fields[1]), float(fields[2]), float(fields[3])))
    points = np.asarray(vertices, dtype=np.float64)
    if len(points) < 100:
        raise ValueError(f"Not enough garment mesh vertices in {path}")
    return points


def extract_outline(vertices):
    x_min, x_max = vertices[:, 0].min(), vertices[:, 0].max()
    z_min, z_max = vertices[:, 2].min(), vertices[:, 2].max()
    resolution = RASTER_RESOLUTION_M
    width = int(np.ceil((x_max - x_min) / resolution)) + 20
    height = int(np.ceil((z_max - z_min) / resolution)) + 20

    px = np.rint((vertices[:, 0] - x_min) / resolution + 10).astype(np.int32)
    py = np.rint((vertices[:, 2] - z_min) / resolution + 10).astype(np.int32)
    mask = np.zeros((height, width), dtype=np.uint8)
    mask[py, px] = 255

    close_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, close_kernel)
    if EDGE_INSET_KERNEL_PX > 1:
        inset_kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE, (EDGE_INSET_KERNEL_PX, EDGE_INSET_KERNEL_PX)
        )
        mask = cv2.erode(mask, inset_kernel)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    if not contours:
        raise ValueError("Could not extract the garment's outer contour")
    contour = max(contours, key=cv2.contourArea)[:, 0, :].astype(np.float64)
    contour_xz = np.column_stack(
        (x_min + (contour[:, 0] - 10) * resolution,
         z_min + (contour[:, 1] - 10) * resolution)
    )

    segment_lengths = np.linalg.norm(
        np.diff(np.vstack((contour_xz, contour_xz[0])), axis=0), axis=1
    )
    cumulative = np.concatenate(([0.0], np.cumsum(segment_lengths)))
    perimeter = cumulative[-1]
    count = max(3, int(round(perimeter / PATH_SPACING_M)))
    sample_distances = np.linspace(0.0, perimeter, count, endpoint=False)

    samples = []
    surface_xz = vertices[:, (0, 2)]
    radius_sq = SURFACE_SEARCH_RADIUS_M ** 2
    for distance in sample_distances:
        segment = min(np.searchsorted(cumulative, distance, side="right") - 1,
                      len(segment_lengths) - 1)
        fraction = (distance - cumulative[segment]) / max(segment_lengths[segment], 1e-12)
        next_index = (segment + 1) % len(contour_xz)
        xz = contour_xz[segment] * (1.0 - fraction) + contour_xz[next_index] * fraction

        delta = surface_xz - xz
        distance_sq = np.einsum("ij,ij->i", delta, delta)
        nearest = int(np.argmin(distance_sq))
        local = np.flatnonzero(distance_sq <= radius_sq)
        if len(local) == 0:
            local = np.asarray([nearest])
        # Use the highest scanned cloth surface close to the inset edge.
        point = vertices[local[np.argmax(vertices[local, 1])]].copy()
        samples.append((point, float(distance)))

    return (
        np.asarray([item[0] for item in samples]),
        np.asarray([item[1] for item in samples]),
        mask,
        contour_xz,
        perimeter,
    )


def initial_joint_seed():
    if SEED_CSV.exists():
        with SEED_CSV.open("r", newline="", encoding="utf-8-sig") as stream:
            row = next(csv.DictReader(stream), None)
        if row and all(f"j{joint}_deg" in row for joint in range(1, 7)):
            return np.radians([float(row[f"j{joint}_deg"]) for joint in range(1, 7)])
    return np.radians([-30.0, 30.0, 109.0, 20.0, -31.0, 101.0])


def nearest_equivalent_angles(q, reference):
    return reference + np.arctan2(np.sin(q - reference), np.cos(q - reference))


def main():
    vertices = read_obj_vertices(MESH_PATH)
    surface_points, path_distance_m, mask, contour_xz, perimeter_m = extract_outline(vertices)
    _, _, global_to_world, scene_chain = load_unity_scene_frames(str(SCENE_PATH))
    world_to_global = np.linalg.inv(global_to_world)
    tcp_local_rotation = transform_matrix(*scene_chain["tcp"])[:3, :3]
    solver = DobotCR10AKinematics(scene_chain=scene_chain)

    # Fixed ZYX flange orientation: roll=180°, pitch=0°, yaw=0°.
    # Thus flange +Z is exactly robot-frame -Z at every waypoint.
    flange_rotation = np.diag([1.0, -1.0, -1.0])
    target_tcp_rotation = flange_rotation @ tcp_local_rotation
    q_previous = initial_joint_seed()
    rows = []
    max_position_error = 0.0
    max_orientation_error = 0.0

    for index, (world_point, path_distance) in enumerate(zip(surface_points, path_distance_m)):
        target_global_mm = (world_to_global @ np.append(world_point, 1.0))[:3] * 1000.0
        q, converged, _, _ = solver.inverse_kinematics(
            target_global_mm, target_tcp_rotation, q_previous
        )
        q = nearest_equivalent_angles(q, q_previous)
        actual_position, actual_rotation, _ = solver.forward_kinematics(q)
        position_error = float(np.linalg.norm(actual_position - target_global_mm))
        rotation_delta = actual_rotation.T @ target_tcp_rotation
        orientation_error = math.acos(np.clip((np.trace(rotation_delta) - 1.0) / 2.0, -1.0, 1.0))
        if not converged or position_error > 0.5 or orientation_error > math.radians(0.5):
            raise RuntimeError(
                f"IK failed at contour point {index}: position={position_error:.3f} mm, "
                f"orientation={math.degrees(orientation_error):.3f} deg"
            )

        # Confirm the solved flange's Z axis is directed toward the floor.
        flange_actual = actual_rotation @ tcp_local_rotation.T
        if float(np.dot(flange_actual[:, 2], np.array([0.0, 0.0, -1.0]))) < math.cos(math.radians(0.5)):
            raise RuntimeError(f"Flange Z is not pointing down at contour point {index}")

        q_previous = q
        roll, pitch, yaw = solver.rotation_matrix_to_rpy(target_tcp_rotation)
        row = {
            "index": index,
            "dist_mm": f"{path_distance * 1000.0:.3f}",
            "x_mm": f"{target_global_mm[0]:.3f}",
            "y_mm": f"{target_global_mm[1]:.3f}",
            "z_mm": f"{target_global_mm[2]:.3f}",
            "roll_deg": f"{roll:.4f}",
            "pitch_deg": f"{pitch:.4f}",
            "yaw_deg": f"{yaw:.4f}",
        }
        for joint in range(6):
            row[f"j{joint + 1}_deg"] = f"{math.degrees(q[joint]):.5f}"
        for joint in range(6):
            row[f"j{joint + 1}_rad"] = f"{q[joint]:.8f}"
        row["err_pos_mm"] = f"{position_error:.5f}"
        row["status"] = "SUCCESS"
        rows.append(row)
        max_position_error = max(max_position_error, position_error)
        max_orientation_error = max(max_orientation_error, orientation_error)

    fields = list(rows[0].keys())
    for output in OUTPUTS:
        output.parent.mkdir(parents=True, exist_ok=True)
        with output.open("w", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=fields)
            writer.writeheader()
            writer.writerows(rows)

    # A compact top-down silhouette preview with the generated closed path.
    canvas = np.zeros((*mask.shape, 3), dtype=np.uint8)
    canvas[mask > 0] = (70, 95, 125)
    pixel_x = np.rint((surface_points[:, 0] - vertices[:, 0].min()) / RASTER_RESOLUTION_M + 10).astype(int)
    pixel_y = np.rint((surface_points[:, 2] - vertices[:, 2].min()) / RASTER_RESOLUTION_M + 10).astype(int)
    path_pixels = np.column_stack((pixel_x, pixel_y)).reshape(-1, 1, 2)
    cv2.polylines(canvas, [path_pixels], True, (0, 0, 255), 2)
    for point in path_pixels[::max(1, len(path_pixels) // 80), 0]:
        cv2.circle(canvas, tuple(point), 2, (0, 255, 0), -1)
    PREVIEW_PATH.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(PREVIEW_PATH), canvas)

    joint_array = np.radians([[float(row[f"j{joint}_deg"]) for joint in range(1, 7)] for row in rows])
    step_delta = np.arctan2(np.sin(np.diff(joint_array, axis=0)), np.cos(np.diff(joint_array, axis=0)))
    seam_delta = np.arctan2(np.sin(joint_array[0] - joint_array[-1]),
                            np.cos(joint_array[0] - joint_array[-1]))
    print(f"Generated {len(rows)} closed-loop waypoints, perimeter {perimeter_m * 1000.0:.1f} mm")
    print(f"Maximum FK error: {max_position_error:.4f} mm; orientation error: {math.degrees(max_orientation_error):.4f} deg")
    print(f"Maximum adjacent joint step: {np.max(np.abs(np.degrees(step_delta))):.2f} deg")
    print(f"Loop seam joint step: {np.max(np.abs(np.degrees(seam_delta))):.2f} deg")
    print(f"Preview: {PREVIEW_PATH}")
    for output in OUTPUTS:
        print(f"Trajectory: {output}")


if __name__ == "__main__":
    main()
