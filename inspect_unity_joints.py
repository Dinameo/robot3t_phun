import numpy as np
from scipy.spatial.transform import Rotation as R

initial_rots = [
    [0.0, 0.0, 0.0, 1.0],
    [0.7071068, 0.0, 0.0, 0.7071068],
    [0.0, 0.0, 0.7071068, 0.7071068],
    [0.0, 0.0, 0.0, 1.0],
    [-0.5, 0.5, -0.5, 0.5],
    [0.7071068, 0.0, 0.0, 0.7071068]
]

local_positions = [
    [0.0, 0.0, 0.0],
    [0.0, 0.0, 0.1765],
    [0.0, 0.607, 0.193],
    [0.568, 0.0, -0.127],
    [0.0, 0.0, 0.125],
    [0.0, 0.0, 0.125],
    [0.0, 0.0, 0.111]
]

def get_flange(joints_deg, invert=True, global_pos=[0,0,0], global_euler=[-90, 90, 0]):
    dir_mult = -1.0 if invert else 1.0
    T_curr = np.eye(4)
    if global_euler is not None:
        T_curr[:3, :3] = R.from_euler('xyz', global_euler, degrees=True).as_matrix()
    T_curr[:3, 3] = global_pos
    
    for i in range(6):
        angle = dir_mult * joints_deg[i]
        q_init = R.from_quat(initial_rots[i])
        q_rot = R.from_euler('z', angle, degrees=True)
        q_total = q_init * q_rot
        
        T_local = np.eye(4)
        T_local[:3, :3] = q_total.as_matrix()
        T_local[:3, 3] = local_positions[i]
        T_curr = T_curr @ T_local
        
    T_flange = np.eye(4)
    T_flange[:3, 3] = local_positions[6]
    T_curr = T_curr @ T_flange
    return T_curr[:3, 3], T_curr[:3, :3]

# Test at zero pose
p0, R0 = get_flange([0,0,0,0,0,0], global_pos=[0,0,0])
print("Pose at q=0 (global at 0,0,0):", p0)

# Check effect of each joint (+30 degrees)
for j in range(6):
    q = [0]*6
    q[j] = 30
    pj, _ = get_flange(q, global_pos=[0,0,0])
    delta = np.round(pj - p0, 4)
    print(f"Delta for J{j+1}=+30 deg: {delta}")
