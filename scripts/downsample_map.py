#!/usr/bin/env python3
"""Voxel-downsample a binary PLY cloud (x,y,z,intensity float32) for a lighter NDT
localization target on CPU-constrained hardware (e.g. Jetson).

The localizer's per-scan local-map crop iterates the FULL map (O(N)); shrinking N
cuts that cost directly with negligible accuracy loss (NDT bins points into voxel
distributions anyway). Keeps one representative point per leaf voxel.

Usage (in the container, paths under /ws):
  python3 /ws/scripts/downsample_map.py /ws/gt_map/gt_map.ply /ws/gt_map/gt_map_ds.ply 0.2
"""
import sys
import numpy as np


def read_ply_xyzi(path):
    with open(path, 'rb') as f:
        assert f.readline().strip() == b'ply', f'not a PLY: {path}'
        n, props, fmt = None, [], None
        while True:
            s = f.readline().strip()
            if s.startswith(b'format'):
                fmt = s.split()[1]
            elif s.startswith(b'element vertex'):
                n = int(s.split()[2])
            elif s.startswith(b'property'):
                props.append(s.split()[-1].decode())
            elif s == b'end_header':
                break
        assert fmt == b'binary_little_endian', f'unsupported PLY format {fmt}'
        data = np.frombuffer(f.read(n * 4 * len(props)), dtype='<f4').reshape(n, len(props))
    return data, props


def voxel_downsample(data, leaf):
    keys = np.floor(data[:, :3] / leaf).astype(np.int64)
    _, idx = np.unique(keys, axis=0, return_index=True)
    return data[np.sort(idx)]


def write_ply_xyzi(path, data, props):
    header = b'ply\nformat binary_little_endian 1.0\nelement vertex %d\n' % data.shape[0]
    for p in props:
        header += b'property float %s\n' % p.encode()
    header += b'end_header\n'
    with open(path, 'wb') as f:
        f.write(header)
        f.write(data.astype('<f4').tobytes())


def main():
    inp, outp = sys.argv[1], sys.argv[2]
    leaf = float(sys.argv[3]) if len(sys.argv) > 3 else 0.2
    data, props = read_ply_xyzi(inp)
    out = voxel_downsample(data, leaf)
    write_ply_xyzi(outp, out, props)
    print(f'{inp}: {data.shape[0]} pts -> {outp}: {out.shape[0]} pts '
          f'(leaf {leaf} m, {100 * out.shape[0] / data.shape[0]:.1f}%)')


if __name__ == '__main__':
    main()
