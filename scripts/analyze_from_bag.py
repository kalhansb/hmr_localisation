#!/usr/bin/env python3
"""Measure localization accuracy from a recorded results bag (/ouster/points +
/pcl_pose): for each scan, transform into the map using the localized pose and
report nearest-neighbour distance to the GT map (registration accuracy)."""
import sys
import numpy as np
from pathlib import Path
from rosbags.highlevel import AnyReader
from scipy.spatial import cKDTree

PLY = sys.argv[1] if len(sys.argv) > 1 else "gt_map/gt_map.ply"
BAG = sys.argv[2] if len(sys.argv) > 2 else "output/acc_bag"


def load_ply(p):
    raw = open(p, "rb").read()
    e = raw.index(b"end_header\n") + len(b"end_header\n")
    return np.frombuffer(raw[e:], dtype=np.float32).reshape(-1, 4)[:, :3].astype(np.float64)


def quat_to_R(x, y, z, w):
    n = (x*x+y*y+z*z+w*w) ** 0.5
    x, y, z, w = x/n, y/n, z/n, w/n
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def cloud_xyz(msg):
    off = {f.name: f.offset for f in msg.fields}
    raw = np.frombuffer(msg.data, dtype=np.uint8).reshape(-1, msg.point_step)
    def col(o): return raw[:, o:o+4].copy().view(np.float32).ravel()
    xyz = np.column_stack([col(off["x"]), col(off["y"]), col(off["z"])]).astype(np.float64)
    xyz = xyz[np.isfinite(xyz).all(1)]
    r = np.linalg.norm(xyz, axis=1)
    return xyz[(r > 1.0) & (r < 80.0)]


print("loading map + KD-tree...")
tree = cKDTree(load_ply(PLY))

poses, scans = [], []
with AnyReader([Path(BAG)]) as rd:
    for con, ts, raw in rd.messages():
        m = rd.deserialize(raw, con.msgtype)
        t = m.header.stamp.sec + m.header.stamp.nanosec*1e-9
        if con.topic == "/pcl_pose":
            p, q = m.pose.pose.position, m.pose.pose.orientation
            poses.append((t, p.x, p.y, p.z, q.x, q.y, q.z, q.w))
        elif con.topic == "/ouster/points":
            scans.append((t, cloud_xyz(m)))
pt = np.array([p[0] for p in poses])
print(f"poses={len(poses)} scans={len(scans)}\n")

print(f"{'scan':>4} {'pts':>6} {'med_cm':>7} {'rms_cm':>7} {'p95_cm':>7} {'<10cm':>6}")
all_d = []
for t, xyz in scans:
    if len(xyz) < 500:
        continue
    j = int(np.argmin(np.abs(pt - t)))
    if abs(poses[j][0] - t) > 0.2:
        continue
    _, x, y, z, qx, qy, qz, qw = poses[j]
    # downsample scan for speed
    k = np.floor(xyz/0.3).astype(np.int64)
    _, idx = np.unique(k, axis=0, return_index=True)
    s = xyz[idx]
    pm = (quat_to_R(qx, qy, qz, qw) @ s.T).T + np.array([x, y, z])
    d, _ = tree.query(pm, k=1, workers=-1)
    all_d.append(d)
    print(f"{len(all_d):>4} {len(s):>6} {np.median(d)*100:>7.1f} "
          f"{np.sqrt((d**2).mean())*100:>7.1f} {np.percentile(d,95)*100:>7.1f} "
          f"{(d<0.1).mean()*100:>5.0f}%")

d = np.concatenate(all_d)
print(f"\n=== OVERALL  ({len(all_d)} scans, {len(d)} points vs GT map) ===")
print(f"  median NN dist : {np.median(d)*100:.1f} cm")
print(f"  RMS    NN dist : {np.sqrt((d**2).mean())*100:.1f} cm")
print(f"  95th pct       : {np.percentile(d,95)*100:.1f} cm")
print(f"  within  5 cm   : {(d<0.05).mean()*100:.1f} %")
print(f"  within 10 cm   : {(d<0.10).mean()*100:.1f} %")
print(f"  within 20 cm   : {(d<0.20).mean()*100:.1f} %")
print("\n(scan->GT-map registration error; includes real surface thickness/foliage,")
print(" so it upper-bounds the pose error.)")
