"""Validated eye-to-hand transform. Translation and all input points use mm."""
from pathlib import Path
import json
import numpy as np

def load_calibration(path):
    data=json.loads(Path(path).read_text(encoding='utf-8-sig'))
    if data.get('calibrated') is not True or data.get('units')!='mm':
        raise ValueError('Calibration must be marked calibrated=true with units=mm after independent measurement.')
    m=np.asarray(data['camera_to_robot'],dtype=float)
    if m.shape!=(4,4) or not np.isfinite(m).all(): raise ValueError('Expected a finite 4 x 4 transform.')
    if not np.allclose(m[3],[0,0,0,1],atol=1e-6): raise ValueError('Invalid homogeneous last row.')
    r=m[:3,:3]
    if not np.allclose(r.T@r,np.eye(3),atol=1e-4) or not np.isclose(np.linalg.det(r),1,atol=1e-4):
        raise ValueError('Rotation must be orthonormal with determinant +1; scaling/reflection is not calibration.')
    return m

def camera_to_robot(points,m):
    return np.asarray(points)@m[:3,:3].T+m[:3,3]

def robot_to_legacy_view_camera(points):
    """Inverse of the old display rig, so calibrated geometry matches the Unity robot."""
    p=np.asarray(points)
    return np.stack([-p[...,1],p[...,0]-765.,1685.-p[...,2]],axis=-1)
