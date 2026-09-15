#!/usr/bin/env python3
"""Convert an hmr_localisation poses.csv into the lidar-frame x,y,z CSV that
glim_traj_compare.py consumes as its "test" trajectory.

WHY THIS EXISTS
docs/glim_comparison.md D2 found the ATE comparison was asymmetric: GLIM was
scored with a 6-DOF Umeyama fit while hmr_localisation went through
traj_eval.py --align, which fits yaw+translation only (4 DOF). More alignment
freedom absorbs more error, so the two columns were not comparable. The fix is
to push BOTH systems through glim_traj_compare.py. That script already converts
the reference from base_link to the lidar frame; this does the identical
conversion for the test trajectory so the two sides meet in the same frame.

    hmr_to_lidar_poses.py IN.poses.csv curtmini|bunker OUT.lidar_poses.csv
"""
import csv
import sys

import numpy as np

sys.path.insert(0, __file__.rsplit("/", 1)[0])
from glim_traj_compare import T_BASE_LIDAR, quat_to_R  # noqa: E402

src, bag, dst = sys.argv[1], sys.argv[2], sys.argv[3]
t_bl, q_bl = T_BASE_LIDAR[bag]
t_bl = np.array(t_bl)

rows = []
for row in csv.DictReader(open(src)):
    t = float(row["stamp_sec"])
    if t <= 0.0:                      # seed pose published before sim time arrives
        continue
    p = np.array([float(row["position_x"]), float(row["position_y"]),
                  float(row["position_z"])])
    q = np.array([float(row["orientation_x"]), float(row["orientation_y"]),
                  float(row["orientation_z"]), float(row["orientation_w"])])
    rows.append((t, *(quat_to_R(q) @ t_bl + p)))

with open(dst, "w", newline="") as fh:
    w = csv.writer(fh)
    w.writerow(["stamp_sec", "x", "y", "z"])
    w.writerows(rows)
print(f"{dst}: {len(rows)} poses")
