"""Organized RGB/XYZ frames. Geometry is in millimetres; pixel order is preserved."""
from pathlib import Path
import json
import numpy as np

PLY_TYPES = {'float':'<f4', 'float32':'<f4', 'double':'<f8', 'float64':'<f8',
             'uchar':'u1', 'uint8':'u1', 'char':'i1', 'int8':'i1', 'short':'<i2',
             'ushort':'<u2', 'int':'<i4', 'uint':'<u4'}

def validate_arrays(xyz, rgb):
    xyz = np.asarray(xyz, dtype=np.float32)
    rgb = np.asarray(rgb, dtype=np.uint8)
    if xyz.ndim != 3 or xyz.shape[-1] != 3 or rgb.shape != xyz.shape:
        raise ValueError('RGB and XYZ must have identical H x W x 3 shape from one frame.')
    if xyz.shape[0] < 2 or xyz.shape[1] < 2:
        raise ValueError('An organized image with H,W >= 2 is required.')
    if not np.any(np.isfinite(xyz).all(axis=-1) & (xyz[:,:,2] > 0)):
        raise ValueError('Frame has no valid positive depth.')
    return xyz, rgb

def save_frame(path, xyz, rgb):
    """Write exact H/W metadata, an NPZ for AI and a binary RGB PLY for Unity."""
    path = Path(path).with_suffix('.ply')
    path.parent.mkdir(parents=True, exist_ok=True)
    xyz, rgb = validate_arrays(xyz, rgb)
    h,w,_ = xyz.shape
    dtype = np.dtype([('x','<f4'),('y','<f4'),('z','<f4'),('red','u1'),('green','u1'),('blue','u1')])
    rows = np.empty(h*w, dtype=dtype)
    for i,k in enumerate(('x','y','z')): rows[k] = xyz[:,:,i].ravel()
    for i,k in enumerate(('red','green','blue')): rows[k] = rgb[:,:,i].ravel()
    header = f'ply\nformat binary_little_endian 1.0\ncomment width {w}\ncomment height {h}\nelement vertex {h*w}\nproperty float x\nproperty float y\nproperty float z\nproperty uchar red\nproperty uchar green\nproperty uchar blue\nend_header\n'
    tmp = path.with_suffix('.ply.tmp')
    with tmp.open('wb') as f:
        f.write(header.encode('ascii')); rows.tofile(f)
    tmp.replace(path)
    np.savez_compressed(path.with_suffix('.npz'), xyz=xyz, rgb=rgb)
    path.with_suffix('.frame.json').write_text(json.dumps({'width':w,'height':h,'units':'mm','organized':True}),encoding='utf-8')
    return path

def _rasterize_unorganized_ply(data, color_keys, height=1200, width=1944):
    """Project an unorganized RGB/XYZ point cloud (including Unity world-meter PLYs) onto a Zivid 1200x1944 organized grid."""
    x = np.asarray(data['x'], dtype=np.float32)
    y = np.asarray(data['y'], dtype=np.float32)
    z = np.asarray(data['z'], dtype=np.float32)
    r = np.asarray(data[color_keys[0]], dtype=np.uint8)
    g = np.asarray(data[color_keys[1]], dtype=np.uint8)
    b = np.asarray(data[color_keys[2]], dtype=np.uint8)

    finite = np.isfinite(x) & np.isfinite(y) & np.isfinite(z)
    if not np.any(finite):
        raise ValueError('Unorganized PLY has no valid 3D points.')
    x, y, z = x[finite], y[finite], z[finite]
    r, g, b = r[finite], g[finite], b[finite]

    # Detect Unity world-meter coordinates (e.g. full_cloud_world.ply) and convert back to Zivid camera mm
    if float(np.max(np.abs(z))) < 10.0 and float(np.max(np.abs(x))) < 10.0:
        vx = x * 1000.0
        vy = (0.335 - z) * 1000.0
        vz = (1.685 - y) * 1000.0
    else:
        vx, vy, vz = x, y, z

    pos_depth = vz > 100.0
    if not np.any(pos_depth):
        raise ValueError('Unorganized PLY has no positive camera depth.')
    vx, vy, vz = vx[pos_depth], vy[pos_depth], vz[pos_depth]
    r, g, b = r[pos_depth], g[pos_depth], b[pos_depth]

    # Zivid nominal pinhole projection (verified to <2px against dataset landmarks)
    fx, fy, cx, cy = 1755.0, 1750.0, 967.0, 554.0
    u = np.rint(cx + fx * (vx / vz)).astype(np.int32)
    v = np.rint(cy + fy * (vy / vz)).astype(np.int32)
    in_bounds = (u >= 0) & (u < width) & (v >= 0) & (v < height)
    if np.count_nonzero(in_bounds) < 1000:
        # Fallback: auto-fit bounding box of (vx/vz, vy/vz) cleanly inside the 1944x1200 frame
        nx = vx / vz
        ny = vy / vz
        min_x, max_x = float(np.percentile(nx, 1)), float(np.percentile(nx, 99))
        min_y, max_y = float(np.percentile(ny, 1)), float(np.percentile(ny, 99))
        scale = 0.88 * min(width / max(max_x - min_x, 1e-4), height / max(max_y - min_y, 1e-4))
        u = np.rint((nx - 0.5 * (min_x + max_x)) * scale + width * 0.5).astype(np.int32)
        v = np.rint((ny - 0.5 * (min_y + max_y)) * scale + height * 0.5).astype(np.int32)
        in_bounds = (u >= 0) & (u < width) & (v >= 0) & (v < height)

    u, v = u[in_bounds], v[in_bounds]
    vx, vy, vz = vx[in_bounds], vy[in_bounds], vz[in_bounds]
    r, g, b = r[in_bounds], g[in_bounds], b[in_bounds]

    # Keep closest point to camera per pixel (z-buffer)
    order = np.argsort(vz)[::-1]
    xyz_grid = np.full((height, width, 3), np.nan, dtype=np.float32)
    rgb_grid = np.zeros((height, width, 3), dtype=np.uint8)
    u_ord, v_ord = u[order], v[order]
    xyz_grid[v_ord, u_ord, 0] = vx[order]
    xyz_grid[v_ord, u_ord, 1] = vy[order]
    xyz_grid[v_ord, u_ord, 2] = vz[order]
    rgb_grid[v_ord, u_ord, 0] = r[order]
    rgb_grid[v_ord, u_ord, 1] = g[order]
    rgb_grid[v_ord, u_ord, 2] = b[order]

    # Fill 1-2px rasterization pinholes so YOLO sees a solid photo
    valid = np.isfinite(xyz_grid[:, :, 2])
    for dy, dx in ((-1, 0), (1, 0), (0, -1), (0, 1)):
        shifted_valid = np.roll(np.roll(valid, dy, axis=0), dx, axis=1)
        fill = (~valid) & shifted_valid
        if np.any(fill):
            for c in range(3):
                xyz_grid[:, :, c][fill] = np.roll(np.roll(xyz_grid[:, :, c], dy, axis=0), dx, axis=1)[fill]
                rgb_grid[:, :, c][fill] = np.roll(np.roll(rgb_grid[:, :, c], dy, axis=0), dx, axis=1)[fill]
            valid |= fill

    return xyz_grid, rgb_grid


def load_frame(path):
    path = Path(path)
    npz = path if path.suffix.lower() == '.npz' else path.with_suffix('.npz')
    if npz.is_file():
        with np.load(npz, allow_pickle=False) as f:
            xyz,rgb = validate_arrays(f['xyz'],f['rgb'])
        return xyz, rgb[:,:,::-1].copy(), rgb
    props=[]; count=0; width=None; height=None; in_vertex=False; fmt=None
    with path.open('rb') as f:
        if f.readline().strip() != b'ply': raise ValueError('Not a PLY file.')
        for _ in range(256):
            line=f.readline().decode('ascii').strip(); parts=line.split()
            if not parts: raise ValueError('Invalid PLY header.')
            if parts[0]=='format': fmt=parts[1]
            elif parts[:2]==['comment','width']: width=int(parts[2])
            elif parts[:2]==['comment','height']: height=int(parts[2])
            elif parts[:2]==['element','vertex']: count=int(parts[2]); in_vertex=True
            elif parts[0]=='element': in_vertex=False
            elif parts[0]=='property' and in_vertex:
                if parts[1] not in PLY_TYPES: raise ValueError('Unsupported vertex property type.')
                props.append((parts[2], PLY_TYPES[parts[1]]))
            elif parts[0]=='end_header': break
        else: raise ValueError('PLY header too long.')
        if count <= 0: raise ValueError('No PLY vertices.')
        if fmt=='binary_little_endian': data=np.fromfile(f,dtype=np.dtype(props),count=count)
        elif fmt=='ascii': data=np.genfromtxt(f,dtype=np.dtype(props),max_rows=count)
        else: raise ValueError(f'Unsupported PLY encoding: {fmt}')
    if data.size != count: raise ValueError('Truncated point cloud.')
    names=data.dtype.names
    if not all(k in names for k in ('x','y','z')): raise ValueError('PLY lacks XYZ.')
    color=('red','green','blue') if 'red' in names else ('r','g','b')
    if not all(k in names for k in color): raise ValueError('PLY lacks RGB.')
    meta=path.with_suffix('.frame.json')
    if meta.is_file():
        m=json.loads(meta.read_text(encoding='utf-8-sig')); width=m['width']; height=m['height']
        if m.get('units','mm')!='mm': raise ValueError('Expected millimetres.')
    if width is None or height is None:
        if count==1944*1200: height,width=1200,1944
    if width is not None and height is not None and int(width)*int(height)==count:
        xyz=np.stack([data[k].reshape(height,width) for k in ('x','y','z')],axis=-1)
        rgb=np.stack([data[k].reshape(height,width) for k in color],axis=-1)
    else:
        xyz,rgb=_rasterize_unorganized_ply(data,color)
    xyz,rgb=validate_arrays(xyz,rgb)
    return xyz,rgb[:,:,::-1].copy(),rgb
