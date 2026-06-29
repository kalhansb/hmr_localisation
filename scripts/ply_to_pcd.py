#!/usr/bin/env python3
"""Convert a binary-little-endian PLY point cloud to a binary PCD (XYZ only).

icp_localization_ros2 loads its reference map with pcl::io::loadPCDFile into a
pcl::PointCloud<PointXYZ>, so it needs a .pcd; the GLIM map is shipped as .ply.
Extra per-point fields (e.g. intensity) are dropped — only x/y/z are written.

Usage:  python3 ply_to_pcd.py in.ply out.pcd
Pure numpy, no PCL/Open3D needed.
"""
import sys
import numpy as np


def read_ply_xyz(path):
    with open(path, "rb") as f:
        if f.readline().strip() != b"ply":
            raise ValueError("not a PLY file")
        fmt = f.readline().strip()
        if fmt != b"format binary_little_endian 1.0":
            raise ValueError(f"unsupported PLY format: {fmt!r} (expected binary_little_endian 1.0)")
        n = None
        props = []  # (name, numpy dtype) in file order
        ply_to_np = {
            b"float": "<f4", b"float32": "<f4", b"double": "<f8", b"float64": "<f8",
            b"uchar": "u1", b"uint8": "u1", b"char": "i1", b"int8": "i1",
            b"ushort": "<u2", b"uint16": "<u2", b"short": "<i2", b"int16": "<i2",
            b"uint": "<u4", b"uint32": "<u4", b"int": "<i4", b"int32": "<i4",
        }
        while True:
            line = f.readline().strip()
            if line == b"end_header":
                break
            tok = line.split()
            if tok[:1] == [b"element"] and tok[1] == b"vertex":
                n = int(tok[2])
            elif tok[:1] == [b"property"]:
                if tok[1] == b"list":
                    raise ValueError("list properties not supported")
                props.append((tok[2].decode(), ply_to_np[tok[1]]))
        if n is None:
            raise ValueError("no 'element vertex' in header")
        names = [p[0] for p in props]
        for axis in ("x", "y", "z"):
            if axis not in names:
                raise ValueError(f"PLY has no '{axis}' property; found {names}")
        dt = np.dtype([(nm, d) for nm, d in props])
        data = np.fromfile(f, dtype=dt, count=n)
    xyz = np.empty((n, 3), dtype="<f4")
    xyz[:, 0] = data["x"]
    xyz[:, 1] = data["y"]
    xyz[:, 2] = data["z"]
    # PCL chokes on NaN/Inf in the map; drop any non-finite points.
    finite = np.isfinite(xyz).all(axis=1)
    return np.ascontiguousarray(xyz[finite])


def write_pcd_binary_xyz(path, xyz):
    n = xyz.shape[0]
    header = (
        "# .PCD v0.7 - Point Cloud Data file format\n"
        "VERSION 0.7\n"
        "FIELDS x y z\n"
        "SIZE 4 4 4\n"
        "TYPE F F F\n"
        "COUNT 1 1 1\n"
        f"WIDTH {n}\n"
        "HEIGHT 1\n"
        "VIEWPOINT 0 0 0 1 0 0 0\n"
        f"POINTS {n}\n"
        "DATA binary\n"
    )
    with open(path, "wb") as f:
        f.write(header.encode("ascii"))
        f.write(xyz.tobytes())


def main():
    if len(sys.argv) != 3:
        print(__doc__)
        sys.exit(1)
    src, dst = sys.argv[1], sys.argv[2]
    xyz = read_ply_xyz(src)
    write_pcd_binary_xyz(dst, xyz)
    print(f"wrote {xyz.shape[0]} points -> {dst}")


if __name__ == "__main__":
    main()
