#!/usr/bin/env python3
"""Measure localization accuracy = nearest-neighbour distance from each
localized scan to the GT map. Reports per-scan and overall stats in cm."""
import sys
import numpy as np
from scipy.spatial import cKDTree

PLY = sys.argv[1] if len(sys.argv) > 1 else "gt_map/gt_map.ply"
NPZ = sys.argv[2] if len(sys.argv) > 2 else "output/scan_pose_pairs.npz"


def load_ply(p):
    raw = open(p, "rb").read()
    e = raw.index(b"end_header\n") + len(b"end_header\n")
    return np.frombuffer(raw[e:], dtype=np.float32).reshape(-1, 4)[:, :3].astype(np.float64)


m = load_ply(PLY)
print(f"map: {len(m)} pts -> building KD-tree...")
tree = cKDTree(m)
data = np.load(NPZ)
n = int(data["n"])
print(f"samples: {n}\n")

all_d = []
print(f"{'scan':>4} {'pts':>6} {'median_cm':>10} {'rms_cm':>8} {'p95_cm':>8} {'<10cm%':>7}")
for i in range(n):
    scan = data[f"scan{i}"]; T = data[f"T{i}"]
    pm = (T[:3, :3] @ scan.T).T + T[:3, 3]      # scan -> map frame
    d, _ = tree.query(pm, k=1, workers=-1)       # NN distance to map (m)
    all_d.append(d)
    print(f"{i:>4} {len(scan):>6} {np.median(d)*100:>10.1f} "
          f"{np.sqrt((d**2).mean())*100:>8.1f} {np.percentile(d,95)*100:>8.1f} "
          f"{(d<0.10).mean()*100:>6.1f}%")

d = np.concatenate(all_d)
print(f"\n=== OVERALL (all {len(d)} scan points vs GT map) ===")
print(f"  median NN distance : {np.median(d)*100:.1f} cm")
print(f"  mean   NN distance : {d.mean()*100:.1f} cm")
print(f"  RMS    NN distance : {np.sqrt((d**2).mean())*100:.1f} cm")
print(f"  90th percentile    : {np.percentile(d,90)*100:.1f} cm")
print(f"  95th percentile    : {np.percentile(d,95)*100:.1f} cm")
print(f"  within 5 cm        : {(d<0.05).mean()*100:.1f} %")
print(f"  within 10 cm       : {(d<0.10).mean()*100:.1f} %")
print(f"  within 20 cm       : {(d<0.20).mean()*100:.1f} %")
print("\nNote: this is scan-to-GT-map registration accuracy (the relevant")
print("metric for map-based localization). It includes real map thickness")
print("(walls, foliage) so it is an upper bound on pose error.")
