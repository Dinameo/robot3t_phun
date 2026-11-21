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
    meta=path.with_suffix('.frame.json')
    if meta.is_file():
        m=json.loads(meta.read_text(encoding='utf-8-sig')); width=m['width']; height=m['height']
        if m.get('units','mm')!='mm': raise ValueError('Expected millimetres.')
    if width is None or height is None:
        if count==1944*1200: height,width=1200,1944
        else: raise ValueError('Unorganized PLY: capture an organized RGB/XYZ frame or provide .frame.json.')
    if int(width)*int(height)!=count: raise ValueError('PLY size differs from frame metadata.')
    names=data.dtype.names
    if not all(k in names for k in ('x','y','z')): raise ValueError('PLY lacks XYZ.')
    color=('red','green','blue') if 'red' in names else ('r','g','b')
    if not all(k in names for k in color): raise ValueError('PLY lacks RGB.')
    xyz=np.stack([data[k].reshape(height,width) for k in ('x','y','z')],axis=-1)
    rgb=np.stack([data[k].reshape(height,width) for k in color],axis=-1)
    xyz,rgb=validate_arrays(xyz,rgb)
    return xyz,rgb[:,:,::-1].copy(),rgb
