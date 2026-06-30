#!/usr/bin/env python3
"""Full-bag trajectory: full-map vs 0.5m-map, with the 0.5m path colored by deviation."""
import csv
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

RUNS = "/ws/output/dsbench/runs"
PLY = "/ws/gt_map/gt_map.ply"
OUT = "/ws/output/dsbench/traj_full_vs_05_fullbag.png"
REF, QRY = "full_long", "us050_long"


def load_ply_xyz(path, stride=20):
    raw = open(path, "rb").read()
    e = raw.index(b"end_header\n") + len(b"end_header\n")
    return np.frombuffer(raw[e:], dtype=np.float32).reshape(-1, 4)[::stride, :3]


def load_traj(tag):
    t, xyz = [], []
    for r in csv.reader(open(f"{RUNS}/{tag}/pose.csv")):
        if r[0] == "stamp":
            continue
        ts = float(r[0])
        if ts < 1e6:
            continue
        t.append(ts); xyz.append([float(r[1]), float(r[2]), float(r[3])])
    return np.array(t), np.array(xyz)


m = load_ply_xyz(PLY)
ft, fx = load_traj(REF)
qt, qx = load_traj(QRY)

order = np.argsort(ft); fts = ft[order]; fxs = fx[order]
idx = np.clip(np.searchsorted(fts, qt), 1, len(fts) - 1)
pick = np.where(np.abs(qt - fts[idx - 1]) <= np.abs(qt - fts[idx]), idx - 1, idx)
ok = np.abs(qt - fts[pick]) <= 0.06
dev_mm = np.linalg.norm(qx[ok] - fxs[pick[ok]], axis=1) * 1000.0
qok = qx[ok]
print(f"ref {len(ft)} poses, qry {len(qt)} poses, matched {ok.sum()}; "
      f"dev mean {dev_mm.mean():.1f} mm, p95 {np.percentile(dev_mm,95):.1f}, max {dev_mm.max():.1f}")

fig, ax = plt.subplots(1, 2, figsize=(21, 9))

# left: both trajectories over the map
ax[0].scatter(m[:, 0], m[:, 1], c="0.8", s=0.3, linewidths=0)
ax[0].plot(fx[:, 0], fx[:, 1], "-", color="#1f77b4", lw=1.8, label=f"full map ({len(ft)} poses)")
ax[0].plot(qx[:, 0], qx[:, 1], "--", color="#d62728", lw=1.2, label=f"0.5m map ({len(qt)} poses)")
ax[0].scatter(*fx[0, :2], c="lime", s=130, edgecolors="k", zorder=5, label="start")
ax[0].scatter(*fx[-1, :2], c="magenta", s=130, marker="s", edgecolors="k", zorder=5, label="end")
ax[0].set_title("Full bag top-down (XY): full-map vs 0.5m-map trajectory")
ax[0].set_xlabel("x [m]"); ax[0].set_ylabel("y [m]"); ax[0].axis("equal")
ax[0].legend(loc="upper right")

# right: 0.5m path colored by deviation from full
ax[1].scatter(m[:, 0], m[:, 1], c="0.85", s=0.3, linewidths=0)
sc = ax[1].scatter(qok[:, 0], qok[:, 1], c=dev_mm, s=10, cmap="inferno", vmin=0,
                   vmax=np.percentile(dev_mm, 99), linewidths=0)
ax[1].set_title(f"0.5m-map path colored by deviation from full  "
                f"(mean {dev_mm.mean():.0f} / p95 {np.percentile(dev_mm,95):.0f} / max {dev_mm.max():.0f} mm)")
ax[1].set_xlabel("x [m]"); ax[1].set_ylabel("y [m]"); ax[1].axis("equal")
plt.colorbar(sc, ax=ax[1], label="deviation [mm]", shrink=0.75)

plt.tight_layout()
plt.savefig(OUT, dpi=100)
print("saved", OUT)
