#!/usr/bin/env python3
"""Zoomed overlay: map points near the trajectory + the path colored by time."""
import sys, csv
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

PLY, CSV, OUT = sys.argv[1], sys.argv[2], sys.argv[3]
margin = float(sys.argv[4]) if len(sys.argv) > 4 else 25.0


def load_ply(path):
    raw = open(path, "rb").read()
    end = raw.index(b"end_header\n") + len(b"end_header\n")
    return np.frombuffer(raw[end:], dtype=np.float32).reshape(-1, 4)[:, :3]


rows = list(csv.reader(open(CSV)))[1:]
t = np.array([[float(r[1]), float(r[2]), float(r[3])] for r in rows])
m = load_ply(PLY)

x0, x1 = t[:, 0].min() - margin, t[:, 0].max() + margin
y0, y1 = t[:, 1].min() - margin, t[:, 1].max() + margin
sel = (m[:, 0] > x0) & (m[:, 0] < x1) & (m[:, 1] > y0) & (m[:, 1] < y1)
mz = m[sel]
print(f"traj span: x[{t[:,0].min():.1f},{t[:,0].max():.1f}] "
      f"y[{t[:,1].min():.1f},{t[:,1].max():.1f}]  local map pts: {len(mz)}")

fig, ax = plt.subplots(figsize=(13, 12))
ax.scatter(mz[:, 0], mz[:, 1], c=mz[:, 2], s=1.5, cmap="Greys",
           alpha=0.55, linewidths=0)
ax.scatter(t[:, 0], t[:, 1], c=np.arange(len(t)), cmap="autumn", s=18, zorder=4)
ax.plot(t[:, 0], t[:, 1], "-", color="orange", lw=1.0, alpha=0.6, zorder=3)
ax.scatter(t[0, 0], t[0, 1], c="lime", s=200, marker="o", zorder=5,
           edgecolors="black", label="start")
ax.scatter(t[-1, 0], t[-1, 1], c="red", s=220, marker="X", zorder=6,
           edgecolors="black", label="LOST LOCK here")
ax.set_title("Zoomed: localized path (yellow→red = time) on GT map; X = divergence")
ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]"); ax.axis("equal")
ax.legend(loc="best")
plt.tight_layout(); plt.savefig(OUT, dpi=100)
print("saved", OUT)
