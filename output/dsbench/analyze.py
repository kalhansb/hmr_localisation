#!/usr/bin/env python3
"""Aggregate the map-resolution sweep: APE vs full-map reference, CPU/mem, fitness.
Run in-container: python3 /ws/output/dsbench/analyze.py"""
import csv, os, glob
import numpy as np

BASE = "/ws/output/dsbench"
RUNS = os.path.join(BASE, "runs")
ORDER = ["full", "us0001", "us005", "us010", "us020", "us030", "us040", "us050",
         "np020", "np030"]
LEAF = {"full": 0.0, "us0001": 0.001, "us005": 0.05, "us010": 0.1, "us020": 0.2,
        "us030": 0.3, "us040": 0.4, "us050": 0.5, "np020": 0.2, "np030": 0.3}


def load_traj(path):
    t, xyz = [], []
    with open(path) as f:
        for r in csv.reader(f):
            if r[0] == "stamp":
                continue
            ts = float(r[0])
            if ts < 1e6:           # drop the t=0 identity seed
                continue
            t.append(ts); xyz.append([float(r[1]), float(r[2]), float(r[3])])
    return np.array(t), np.array(xyz)


def ape(ref_t, ref_xyz, q_t, q_xyz, tol=0.06):
    if len(ref_t) == 0 or len(q_t) == 0:
        return None
    order = np.argsort(ref_t); ref_t = ref_t[order]; ref_xyz = ref_xyz[order]
    idx = np.searchsorted(ref_t, q_t)
    idx = np.clip(idx, 1, len(ref_t) - 1)
    left = ref_t[idx - 1]; right = ref_t[idx]
    pick = np.where(np.abs(q_t - left) <= np.abs(q_t - right), idx - 1, idx)
    dt = np.abs(q_t - ref_t[pick])
    ok = dt <= tol
    if ok.sum() == 0:
        return None
    d = np.linalg.norm(q_xyz[ok] - ref_xyz[pick[ok]], axis=1)
    return dict(n=int(ok.sum()), mean=float(d.mean()), median=float(np.median(d)),
               rmse=float(np.sqrt((d**2).mean())), p95=float(np.percentile(d, 95)),
               maxv=float(d.max()))


def cpu_mem(path, warmup=15.0, tail=2.0):
    rows = []
    with open(path) as f:
        for r in csv.reader(f):
            if r[0] == "t_wall":
                continue
            rows.append([float(r[0]), int(r[1]), int(r[2]), int(r[3]), int(r[4]), int(r[5])])
    if len(rows) < 4:
        return None
    a = np.array(rows, float)
    t = a[:, 0]; rss = a[:, 1]; hwm = a[:, 2]; thr = a[:, 3]; ticks = a[:, 4]; clk = a[0, 5]
    t0 = t[0]
    m = (t >= t0 + warmup) & (t <= t[-1] - tail)
    if m.sum() < 3:
        m = np.ones(len(t), bool)
    dt = np.diff(t); dtick = np.diff(ticks)
    cpu = np.where(dt > 0, dtick / clk / dt * 100.0, 0.0)
    mid = m[1:]
    cpu_steady = cpu[mid] if mid.sum() else cpu
    total_cpu_s = float((ticks[-1] - ticks[0]) / clk)   # total CPU-seconds consumed
    return dict(cpu_mean=float(cpu_steady.mean()), cpu_peak=float(cpu_steady.max()),
                rss_mean_mb=float(rss[m].mean() / 1024), rss_peak_mb=float(rss[m].max() / 1024),
                hwm_mb=float(hwm.max() / 1024), threads=int(np.median(thr)),
                total_cpu_s=total_cpu_s)


def fit_stats(path):
    if not os.path.exists(path):
        return None
    v = []
    for ln in open(path):
        try:
            v.append(float(ln))
        except ValueError:
            pass
    if not v:
        return None
    v = np.array(v)
    return dict(n=len(v), mean=float(v.mean()), median=float(np.median(v)),
               p95=float(np.percentile(v, 95)))


pts = {}
ri = os.path.join(BASE, "run_index.csv")
if os.path.exists(ri):
    for r in csv.DictReader(open(ri)):
        pts[r["tag"]] = int(r["pts"])

ref_t, ref_xyz = load_traj(os.path.join(RUNS, "full", "pose.csv"))
print(f"reference (full) trajectory: {len(ref_t)} poses\n")

hdr = ["tag", "leaf_m", "map_pts", "%full", "poses", "scans",
       "APE_mean_m", "APE_med_m", "APE_rmse_m", "APE_p95_m", "APE_max_m",
       "fit_mean", "fit_med", "CPUs/scan", "CPU%_mean", "RSS_mean_MB", "RSS_peak_MB", "HWM_MB", "thr"]
rows_out = []
for tag in ORDER:
    d = os.path.join(RUNS, tag)
    if not os.path.isdir(d) or not os.path.exists(os.path.join(d, "pose.csv")):
        continue
    qt, qx = load_traj(os.path.join(d, "pose.csv"))
    a = ape(ref_t, ref_xyz, qt, qx) if tag != "full" else dict(n=len(qt), mean=0, median=0, rmse=0, p95=0, maxv=0)
    cm = cpu_mem(os.path.join(d, "proc.csv"))
    fs = fit_stats(os.path.join(d, "fitness.txt"))
    p = pts.get(tag, 0)
    row = [tag, LEAF[tag], p, round(100 * p / pts.get("full", p or 1), 1), len(qt),
           fs["n"] if fs else 0,
           round(a["mean"], 4) if a else None, round(a["median"], 4) if a else None,
           round(a["rmse"], 4) if a else None, round(a["p95"], 4) if a else None,
           round(a["maxv"], 4) if a else None,
           round(fs["mean"], 4) if fs else None, round(fs["median"], 4) if fs else None,
           round(cm["total_cpu_s"] / (fs["n"] if fs and fs["n"] else 1), 3) if cm else None,
           round(cm["cpu_mean"], 1) if cm else None,
           round(cm["rss_mean_mb"], 1) if cm else None, round(cm["rss_peak_mb"], 1) if cm else None,
           round(cm["hwm_mb"], 1) if cm else None, cm["threads"] if cm else None]
    rows_out.append(row)

w = [max(len(str(h)), *(len(str(r[i])) for r in rows_out)) for i, h in enumerate(hdr)]
print(" ".join(str(h).rjust(w[i]) for i, h in enumerate(hdr)))
for r in rows_out:
    print(" ".join(str(c).rjust(w[i]) for i, c in enumerate(r)))

with open(os.path.join(BASE, "summary.csv"), "w", newline="") as f:
    wr = csv.writer(f); wr.writerow(hdr); wr.writerows(rows_out)
print(f"\nwrote {os.path.join(BASE, 'summary.csv')}")
