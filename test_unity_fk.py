import numpy as np
from scipy.spatial.transform import Rotation as R
import csv

# 1. Load joint angles from cr10a_joint_angles.csv
with open('dobot_cr10a/Assets/PointCloud/cr10a_joint_angles.csv', 'r') as f:
    reader = csv.DictReader(f)
    rows = list(reader)

row0 = rows[0]
j_deg = [float(row0[f'j{i}_deg']) for i in range(1, 7)]
print("Target Joint angles (deg):", j_deg)

# 2. Local transforms from SampleScene.unity
# axis0: pos=(0, 0, 0), rot=(0, 0, 0, 1)
# axis1: pos=(0, 0, 0.1765), rot=(0.7071068, 0, 0, 0.7071068)
# axis2: pos=(0, 0.607, 0.193), rot=(0, 0, 0.7071068, 0.7071068)
# axis3: pos=(0.568, 0, -0.127), rot=(0, 0, 0, 1)
# axis4: pos=(0, 0, 0.125), rot=(-0.5, 0.5, -0.5, 0.5)
# axis5: pos=(0, 0, 0.125), rot=(0.7071068, 0, 0, 0.7071068)
# axis6: pos=(0, 0, 0.111), rot=(0, 0, 0, 1)

# In manual.cs:
# initialRotations[i] is the rotation when slider=0
# when slider changes:
# axisTransforms[i].localRotation = initialRotations[i] * Quaternion.Euler(0, 0, angleDeg)
# where angleDeg = -1 * slider_deg (since invertDirection = true)

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
    [0.0, 0.0, 0.111] # axis6 (flange)
]

def unity_fk(joint_angles_deg, invert_dir=True):
    dir_mult = -1.0 if invert_dir else 1.0
    
    # Global transform in Unity
    T_curr = np.eye(4)
    # Global pos=[-0, -0, 1.5], rot=[-0.5, 0.5, 0.5, 0.5]
    T_global = np.eye(4)
    T_global[:3, :3] = R.from_quat([-0.5, 0.5, 0.5, 0.5]).as_matrix()
    T_global[:3, 3] = [0.0, 0.0, 1.5]
    
    T_curr = T_global.copy()
    
    for i in range(6):
        angle = dir_mult * joint_angles_deg[i]
        # rot = initialRotations[i] * Euler(0, 0, angle)
        q_init = R.from_quat(initial_rots[i])
        q_rot = R.from_euler('z', angle, degrees=True)
        q_total = q_init * q_rot
        
        T_local = np.eye(4)
        T_local[:3, :3] = q_total.as_matrix()
        T_local[:3, 3] = local_positions[i]
        
        T_curr = T_curr @ T_local
        
    # Flange (axis6)
    T_flange = np.eye(4)
    T_flange[:3, 3] = local_positions[6]
    T_curr = T_curr @ T_flange
    
    return T_curr[:3, 3]

flange_pos = unity_fk(j_deg)
print("Flange pos in Unity with current Global (at z=1.5):", flange_pos)
print("Target pos in Unity:", [float(row0['x_mm'])*0.001, float(row0['z_mm'])*0.001, float(row0['y_mm'])*0.001])
