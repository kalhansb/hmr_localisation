#!/usr/bin/env python3
"""Absolute trajectory error of a localizer run against a reference trajectory.

Both trajectories must already be in the same frame (the localizer publishes `map`
poses seeded from the same gt_map, so runs of the same bag are directly comparable and
no alignment is needed). Pass --align for the general case -- two tracks in frames that
differ by a rigid transform -- which solves the Umeyama fit first.

Inputs are pose CSVs in either format this repo produces:
  benchmark_pose_recorder  message_index,stamp_sec,frame_id,position_x,...,orientation_w,covariance
  a bare recorder          t,x,y,z,qx,qy,qz,qw

The reference is resampled onto each test stamp by linear interpolation (shortest-angle
for yaw), so a test run that dropped scans is still scored on the scans it did produce.
A test sample whose bracketing reference samples are further apart than --max-gap is
left unmatched rather than interpolated across a hole.

Usage:
  traj_eval.py REF.csv TEST.csv [--align] [--max-gap 0.5] [--diverge-threshold 0.5]
               [--t0 SEC] [--t1 SEC] [--out per_sample.csv]

Reported: translation error percentiles, yaw error, coverage, and the first time the
error crosses --diverge-threshold -- the number that separates "tracks with some noise"
from "lost lock and never recovered".
"""
import argparse
import csv
import math
import sys

import numpy as np

BENCH_COLS = ("stamp_sec", "position_x", "position_y", "position_z",
              "orientation_x", "orientation_y", "orientation_z", "orientation_w")
BARE_COLS = ("t", "x", "y", "z", "qx", "qy", "qz", "qw")


def load(path):
    """-> (t[N], xyz[N,3], yaw[N]) sorted by time, duplicate stamps dropped."""
    with open(path, newline="", encoding="utf-8") as fh:
        rows = list(csv.DictReader(fh))
    if not rows:
        sys.exit(f"{path}: no rows")
    have = set(rows[0].keys())
    if set(BENCH_COLS) <= have:
        cols = BENCH_COLS
    elif set(BARE_COLS) <= have:
        cols = BARE_COLS
    else:
        sys.exit(f"{path}: unrecognised columns {sorted(have)}")

    t, xyz, quat = [], [], []
    for r in rows:
        try:
            vals = [float(r[c]) for c in cols]
        except (TypeError, ValueError):
            continue  # a partially written final row
        t.append(vals[0])
        xyz.append(vals[1:4])
        quat.append(vals[4:8])
    if not t:
        sys.exit(f"{path}: no parseable rows")

    t = np.asarray(t)
    xyz = np.asarray(xyz)
    q = np.asarray(quat)  # x, y, z, w
    # yaw from the quaternion, the only angle a ground vehicle's error is read in
    yaw = np.arctan2(2.0 * (q[:, 3] * q[:, 2] + q[:, 0] * q[:, 1]),
                     1.0 - 2.0 * (q[:, 1] ** 2 + q[:, 2] ** 2))

    order = np.argsort(t, kind="stable")
    t, xyz, yaw = t[order], xyz[order], yaw[order]
    keep = np.concatenate(([True], np.diff(t) > 0))
    return t[keep], xyz[keep], yaw[keep]


def wrap(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def resample(ref_t, ref_xyz, ref_yaw, query_t, max_gap):
    """Linear-interpolate the reference onto query_t. -> (xyz, yaw, valid mask)."""
    # Validate on the time range, not the index: a query landing exactly on the first or
    # last reference stamp is in range and must interpolate to itself, not drop out.
    valid = (query_t >= ref_t[0]) & (query_t <= ref_t[-1])
    idx = np.clip(np.searchsorted(ref_t, query_t), 1, len(ref_t) - 1)
    lo, hi = idx - 1, idx
    span = ref_t[hi] - ref_t[lo]
    valid &= span <= max_gap

    w = np.where(span > 0, (query_t - ref_t[lo]) / np.where(span > 0, span, 1.0), 0.0)
    xyz = ref_xyz[lo] + (ref_xyz[hi] - ref_xyz[lo]) * w[:, None]
    yaw = ref_yaw[lo] + wrap(ref_yaw[hi] - ref_yaw[lo]) * w
    return xyz, wrap(yaw), valid


def umeyama_2d(src, dst):
    """Rigid yaw+translation fit (no scale) minimising ||dst - (R src + t)||."""
    sc, dc = src.mean(0), dst.mean(0)
    s, d = src - sc, dst - dc
    num = float((s[:, 0] * d[:, 1] - s[:, 1] * d[:, 0]).sum())
    den = float((s[:, 0] * d[:, 0] + s[:, 1] * d[:, 1]).sum())
    theta = math.atan2(num, den)
    c, sn = math.cos(theta), math.sin(theta)
    R = np.array([[c, -sn], [sn, c]])
    return R, dc - R @ sc, theta


def stats(v):
    return dict(n=len(v), med=float(np.median(v)), mean=float(v.mean()),
                p95=float(np.percentile(v, 95)), max=float(v.max()),
                rmse=float(math.sqrt(float((v ** 2).mean()))))


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("reference")
    ap.add_argument("test")
    ap.add_argument("--align", action="store_true",
                    help="solve a yaw+translation fit first (tracks in different frames)")
    ap.add_argument("--max-gap", type=float, default=0.5,
                    help="do not interpolate the reference across a hole wider than this (s)")
    ap.add_argument("--diverge-threshold", type=float, default=0.5,
                    help="report the first sample whose error exceeds this (m)")
    ap.add_argument("--t0", type=float, help="ignore samples before this bag-relative time")
    ap.add_argument("--t1", type=float, help="ignore samples after this bag-relative time")
    ap.add_argument("--out", help="write per-sample errors to this CSV")
    a = ap.parse_args()

    ref_t, ref_xyz, ref_yaw = load(a.reference)
    tst_t, tst_xyz, tst_yaw = load(a.test)

    t0 = min(ref_t[0], tst_t[0])          # bag-relative, so the times read like the run
    rel = tst_t - t0
    win = np.ones(len(tst_t), bool)
    if a.t0 is not None:
        win &= rel >= a.t0
    if a.t1 is not None:
        win &= rel <= a.t1

    ri_xyz, ri_yaw, valid = resample(ref_t, ref_xyz, ref_yaw, tst_t, a.max_gap)
    ok = valid & win
    if not ok.any():
        sys.exit("no test sample overlaps the reference (check the time windows)")

    est, gt = tst_xyz[ok], ri_xyz[ok]
    est_yaw, gt_yaw = tst_yaw[ok], ri_yaw[ok]
    theta = 0.0
    if a.align:
        R, tvec, theta = umeyama_2d(est[:, :2], gt[:, :2])
        est = est.copy()
        est[:, :2] = est[:, :2] @ R.T + tvec
        est_yaw = wrap(est_yaw + theta)

    err = np.linalg.norm(est - gt, axis=1)
    err2d = np.linalg.norm(est[:, :2] - gt[:, :2], axis=1)
    yerr = np.abs(wrap(est_yaw - gt_yaw))
    rt = rel[ok]

    s3, s2, sy = stats(err), stats(err2d), stats(np.degrees(yerr))
    print(f"reference : {a.reference}  ({len(ref_t)} poses, "
          f"{ref_t[-1] - ref_t[0]:.1f} s)")
    print(f"test      : {a.test}  ({len(tst_t)} poses, {tst_t[-1] - tst_t[0]:.1f} s)")
    if a.align:
        print(f"alignment : yaw {math.degrees(theta):+.3f} deg + translation")
    print(f"matched   : {ok.sum()} / {win.sum()} test poses in window "
          f"({100.0 * ok.sum() / max(win.sum(), 1):.1f}%), "
          f"{len(tst_t) - valid.sum()} unmatched overall")
    print()
    print(f"{'':<12}{'median':>10}{'mean':>10}{'p95':>10}{'max':>10}{'rmse':>10}")
    for name, s, unit in (("ATE 3D (m)", s3, 1.0), ("ATE 2D (m)", s2, 1.0),
                          ("yaw (deg)", sy, 1.0)):
        print(f"{name:<12}{s['med']*unit:>10.4f}{s['mean']*unit:>10.4f}"
              f"{s['p95']*unit:>10.4f}{s['max']*unit:>10.4f}{s['rmse']*unit:>10.4f}")
    print()

    bad = np.flatnonzero(err > a.diverge_threshold)
    if bad.size:
        print(f"first error > {a.diverge_threshold} m at t={rt[bad[0]]:.1f} s "
              f"({err[bad[0]]:.2f} m); {bad.size} of {ok.sum()} samples exceed it; "
              f"final error {err[-1]:.2f} m at t={rt[-1]:.1f} s")
    else:
        print(f"no sample exceeds {a.diverge_threshold} m "
              f"(final error {err[-1]:.3f} m at t={rt[-1]:.1f} s)")

    if a.out:
        with open(a.out, "w", newline="", encoding="utf-8") as fh:
            w = csv.writer(fh)
            w.writerow(["t_rel", "stamp_sec", "err_3d", "err_2d", "err_yaw_deg",
                        "est_x", "est_y", "est_z", "ref_x", "ref_y", "ref_z"])
            for i in range(len(rt)):
                w.writerow([f"{rt[i]:.4f}", f"{tst_t[ok][i]:.9f}",
                            f"{err[i]:.6f}", f"{err2d[i]:.6f}",
                            f"{math.degrees(yerr[i]):.6f}",
                            f"{est[i,0]:.6f}", f"{est[i,1]:.6f}", f"{est[i,2]:.6f}",
                            f"{gt[i,0]:.6f}", f"{gt[i,1]:.6f}", f"{gt[i,2]:.6f}"])
        print(f"per-sample -> {a.out}")


if __name__ == "__main__":
    main()
