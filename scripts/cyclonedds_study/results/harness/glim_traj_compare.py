#!/usr/bin/env python3
"""Compare a GLIM trajectory against an hmr_localisation reference trajectory.

    glim_traj_compare.py REF.poses.csv GLIM.lidar_poses.csv curtmini|bunker

THIS IS NOT THE SAME QUANTITY AS THE LOCALIZER'S ATE COLUMN.

hmr_localisation localises against a prior map, so its error is absolute: metres
away from where the map says the robot is. GLIM is SLAM -- it has no prior map
and its world frame starts wherever it started -- so the only meaningful
comparison is after a rigid (Umeyama, no scale) alignment onto the reference.
What that measures is DRIFT: how much the shape of GLIM's trajectory diverges
from the localizer's over the window. A perfect-shape trajectory in a completely
different frame scores zero here, which is the intended behaviour.

The reference records base_link in the map frame while GLIM publishes the LiDAR
pose, so the reference is first pushed out to the LiDAR frame through the known
static T_base_lidar -- otherwise the lever arm (0.40 m on curtmini) would show
up as spurious error whenever the robot turns.
"""
import csv
import sys

import numpy as np

# base_link -> lidar, resolved from each bag's /tf_static with tf_extrinsics.py
T_BASE_LIDAR = {
    "curtmini": ([0.110500, 0.000000, 0.404000],
                 [0.0, 0.0, 1.0, 0.0]),
    "bunker":   ([0.112963, 0.002776, 0.365981],
                 [0.000281764, 0.000283917, 0.706542473, 0.707670526]),
}


def quat_to_R(q):
    x, y, z, w = q / np.linalg.norm(q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w),     2 * (x * z + y * w)],
        [2 * (x * y + z * w),     1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w),     2 * (y * z + x * w),     1 - 2 * (x * x + y * y)],
    ])


def load_ref(path, bag):
    t_bl, q_bl = T_BASE_LIDAR[bag]
    R_bl, t_bl = quat_to_R(np.array(q_bl)), np.array(t_bl)
    ts, ps = [], []
    for row in csv.DictReader(open(path)):
        t = float(row["stamp_sec"])
        if t <= 0.0:          # seed pose published before sim time arrives
            continue
        p = np.array([float(row["position_x"]), float(row["position_y"]),
                      float(row["position_z"])])
        q = np.array([float(row["orientation_x"]), float(row["orientation_y"]),
                      float(row["orientation_z"]), float(row["orientation_w"])])
        ts.append(t)
        ps.append(quat_to_R(q) @ t_bl + p)      # base_link pose -> lidar position
    return np.array(ts), np.array(ps)


def load_glim(path):
    ts, ps = [], []
    for row in csv.DictReader(open(path)):
        ts.append(float(row["stamp_sec"]))
        ps.append([float(row["x"]), float(row["y"]), float(row["z"])])
    return np.array(ts), np.array(ps)


def umeyama_rigid(src, dst):
    """Rigid R,t minimising ||R@src + t - dst|| (no scale)."""
    mu_s, mu_d = src.mean(0), dst.mean(0)
    S = (dst - mu_d).T @ (src - mu_s) / len(src)
    U, _, Vt = np.linalg.svd(S)
    D = np.eye(3)
    if np.linalg.det(U) * np.linalg.det(Vt) < 0:
        D[2, 2] = -1
    R = U @ D @ Vt
    return R, mu_d - R @ mu_s


def main():
    ref_path, glim_path, bag = sys.argv[1], sys.argv[2], sys.argv[3]
    rt, rp = load_ref(ref_path, bag)
    gt, gp = load_glim(glim_path)
    if len(gt) < 10 or len(rt) < 10:
        print(f"{glim_path}: too few poses (ref={len(rt)} glim={len(gt)})")
        return

    # nearest-stamp association, 20 ms tolerance (scans are 100 ms apart)
    idx = np.searchsorted(rt, gt).clip(1, len(rt) - 1)
    left = np.abs(gt - rt[idx - 1])
    right = np.abs(gt - rt[idx])
    take = np.where(left < right, idx - 1, idx)
    ok = np.minimum(left, right) < 0.020
    if ok.sum() < 10:
        print(f"{glim_path}: only {ok.sum()} matched poses -- no overlap")
        return
    src, dst = gp[ok], rp[take[ok]]

    R, t = umeyama_rigid(src, dst)
    err = np.linalg.norm((R @ src.T).T + t - dst, axis=1)
    length = np.linalg.norm(np.diff(dst, axis=0), axis=1).sum()

    name = glim_path.rsplit("/", 1)[-1].replace(".lidar_poses.csv", "")
    print("%-24s %7d %9.3f %9.3f %9.3f %9.1f %8.2f%%" % (
        name, ok.sum(), np.median(err), np.quantile(err, 0.95),
        np.sqrt((err ** 2).mean()), length,
        100.0 * np.median(err) / length if length > 0 else float("nan")))


if __name__ == "__main__":
    print("%-24s %7s %9s %9s %9s %9s %8s" % (
        "run", "matched", "ate_med", "ate_p95", "ate_rmse", "path_m", "drift%"))
    main()
