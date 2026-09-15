#!/usr/bin/env python3
"""Convert a GLIM lidar-pose CSV into the bare format traj_eval.py accepts,
expressed in base_link so it can be scored by exactly the same tool as the
hmr_localisation runs.

    glim_to_baselink.py IN.lidar_poses.csv OUT.csv curtmini|bunker

WHY: comparing the two systems' ATE numbers as originally produced was unfair to
hmr_localisation on two counts, both favouring GLIM.
  1. traj_eval.py --align fits yaw + translation (4 DOF); glim_traj_compare.py
     fit a full 3D rigid transform (6 DOF). More freedom absorbs more error.
  2. traj_eval.py interpolates the reference onto every test pose (~1139
     matches); glim_traj_compare.py took nearest-stamp within 20 ms, capped by
     the reference's own density (~739 matches).
Routing GLIM's poses through traj_eval.py removes both at once.

GLIM publishes T_world_lidar; the reference is base_link in map. So push GLIM
out to base_link via T_world_base = T_world_lidar * T_lidar_base, rather than
pulling the reference in -- same rigid relation, but this way the file handed to
traj_eval.py is in the reference's own frame and the tool needs no changes.
"""
import csv
import sys

import numpy as np

sys.path.insert(0, "/tmp/claude-1000/-home-jetsondevkit-jetbot-slam/"
                   "5ca092d7-b48b-4dbe-bc68-f725e36dd46d/scratchpad")
from glim_traj_compare import T_BASE_LIDAR, quat_to_R


def R_to_quat(R):
    t = np.trace(R)
    if t > 0:
        s = np.sqrt(t + 1.0) * 2
        w, x, y, z = 0.25 * s, (R[2,1]-R[1,2])/s, (R[0,2]-R[2,0])/s, (R[1,0]-R[0,1])/s
    elif R[0,0] > R[1,1] and R[0,0] > R[2,2]:
        s = np.sqrt(1.0 + R[0,0] - R[1,1] - R[2,2]) * 2
        w, x, y, z = (R[2,1]-R[1,2])/s, 0.25*s, (R[0,1]+R[1,0])/s, (R[0,2]+R[2,0])/s
    elif R[1,1] > R[2,2]:
        s = np.sqrt(1.0 + R[1,1] - R[0,0] - R[2,2]) * 2
        w, x, y, z = (R[0,2]-R[2,0])/s, (R[0,1]+R[1,0])/s, 0.25*s, (R[1,2]+R[2,1])/s
    else:
        s = np.sqrt(1.0 + R[2,2] - R[0,0] - R[1,1]) * 2
        w, x, y, z = (R[1,0]-R[0,1])/s, (R[0,2]+R[2,0])/s, (R[1,2]+R[2,1])/s, 0.25*s
    q = np.array([x, y, z, w])
    return q / np.linalg.norm(q)


src, dst, bag = sys.argv[1], sys.argv[2], sys.argv[3]
t_bl, q_bl = T_BASE_LIDAR[bag]
T_base_lidar = np.eye(4)
T_base_lidar[:3, :3] = quat_to_R(np.array(q_bl))
T_base_lidar[:3, 3] = np.array(t_bl)
T_lidar_base = np.linalg.inv(T_base_lidar)

with open(dst, "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["t", "x", "y", "z", "qx", "qy", "qz", "qw"])
    for row in csv.DictReader(open(src)):
        T = np.eye(4)
        T[:3, :3] = quat_to_R(np.array([float(row["qx"]), float(row["qy"]),
                                        float(row["qz"]), float(row["qw"])]))
        T[:3, 3] = [float(row["x"]), float(row["y"]), float(row["z"])]
        M = T @ T_lidar_base
        q = R_to_quat(M[:3, :3])
        w.writerow([row["stamp_sec"], M[0, 3], M[1, 3], M[2, 3],
                    q[0], q[1], q[2], q[3]])
print(dst)
