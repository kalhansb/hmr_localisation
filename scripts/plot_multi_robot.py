#!/usr/bin/env python3
"""Static multi-robot summary (run on the HOST).

Left:  top-down GT map (grey) with robot1 + robot2 trajectories overlaid.
Right: robot1->robot2 relative range vs time (from output/relative_pose.csv).

Usage:
  python3 scripts/plot_multi_robot.py [gt_map.ply] [path_robot1.csv]
          [path_robot2.csv] [relative_pose.csv] [out.png]
"""
import csv
import sys

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

PLY = sys.argv[1] if len(sys.argv) > 1 else "gt_map/gt_map.ply"
C1 = sys.argv[2] if len(sys.argv) > 2 else "output/path_robot1.csv"
C2 = sys.argv[3] if len(sys.argv) > 3 else "output/path_robot2.csv"
REL = sys.argv[4] if len(sys.argv) > 4 else "output/relative_pose.csv"
OUT = sys.argv[5] if len(sys.argv) > 5 else "output/multi_robot_eval.png"


def load_ply(path):
    raw = open(path, "rb").read()
    end = raw.index(b"end_header\n") + len(b"end_header\n")
    return np.frombuffer(raw[end:], dtype=np.float32).reshape(-1, 4)[:, :3]


def load_xy(path):
    rows = list(csv.reader(open(path)))[1:]
    return np.array([[float(r[1]), float(r[2])] for r in rows if float(r[0]) > 1.0])


t1, t2 = load_xy(C1), load_xy(C2)
m = load_ply(PLY)

allxy = np.vstack([t1, t2])
margin = 25.0
x0, x1 = allxy[:, 0].min() - margin, allxy[:, 0].max() + margin
y0, y1 = allxy[:, 1].min() - margin, allxy[:, 1].max() + margin
sel = (m[:, 0] > x0) & (m[:, 0] < x1) & (m[:, 1] > y0) & (m[:, 1] < y1)
mz = m[sel]

fig, (ax, ax2) = plt.subplots(1, 2, figsize=(20, 9))

ax.scatter(mz[:, 0], mz[:, 1], c=mz[:, 2], s=1.0, cmap="Greys", alpha=0.45, linewidths=0)
ax.plot(t1[:, 0], t1[:, 1], "-", color="lime", lw=1.8, label="robot1", zorder=4)
ax.plot(t2[:, 0], t2[:, 1], "-", color="orange", lw=1.8, label="robot2 (+offset)", zorder=4)
ax.scatter(*t1[0], c="lime", s=180, marker="o", edgecolors="black", zorder=6)
ax.scatter(*t2[0], c="orange", s=180, marker="o", edgecolors="black", zorder=6)
ax.set_title("Multi-robot trajectories on shared GT map")
ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]"); ax.axis("equal"); ax.legend(loc="best")

try:
    rows = list(csv.DictReader(open(REL)))
    tt = np.array([float(r["wall_t"]) for r in rows])
    rng = np.array([float(r["range_m"]) for r in rows])
    ax2.plot(tt, rng, "-", color="crimson", lw=1.6)
    ax2.fill_between(tt, 0, rng, color="crimson", alpha=0.12)
    ax2.set_title(f"robot1 -> robot2 range  (mean {rng.mean():.2f} m, "
                  f"min {rng.min():.2f}, max {rng.max():.2f})")
    ax2.set_xlabel("time [s]"); ax2.set_ylabel("range [m]"); ax2.grid(alpha=0.3)
except FileNotFoundError:
    ax2.text(0.5, 0.5, f"no {REL}", ha="center", va="center")

plt.tight_layout(); plt.savefig(OUT, dpi=100)
print("saved", OUT)
