#!/usr/bin/env python3
"""Overlay the localized trajectory on the GT map (top-down + side view)."""
import sys
import csv
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

PLY = sys.argv[1]
CSV = sys.argv[2]
OUT = sys.argv[3]


def load_ply_xyz(path, stride=1):
    with open(path, "rb") as f:
        raw = f.read()
    end = raw.index(b"end_header\n") + len(b"end_header\n")
    pts = np.frombuffer(raw[end:], dtype=np.float32).reshape(-1, 4)  # x y z intensity
    return pts[::stride, :3]


def load_traj(path):
    rows = list(csv.reader(open(path)))[1:]
    a = np.array([[float(r[1]), float(r[2]), float(r[3])] for r in rows])
    return a


m = load_ply_xyz(PLY, stride=15)
t = load_traj(CSV)
print(f"map pts (plotted): {len(m)}   traj pts: {len(t)}")

fig, ax = plt.subplots(1, 2, figsize=(20, 9))

# Top-down XY, map colored by height
sc = ax[0].scatter(m[:, 0], m[:, 1], c=m[:, 2], s=0.4, cmap="viridis",
                   alpha=0.5, linewidths=0)
ax[0].plot(t[:, 0], t[:, 1], "-", color="red", lw=2.0, label="localized path")
ax[0].scatter(t[0, 0], t[0, 1], c="lime", s=120, marker="o", zorder=5,
              edgecolors="black", label="start")
ax[0].scatter(t[-1, 0], t[-1, 1], c="magenta", s=120, marker="s", zorder=5,
              edgecolors="black", label="end")
ax[0].set_title("Top-down (XY): GT map + localized trajectory")
ax[0].set_xlabel("x [m]"); ax[0].set_ylabel("y [m]"); ax[0].axis("equal")
ax[0].legend(loc="upper right")
plt.colorbar(sc, ax=ax[0], label="map height z [m]", shrink=0.7)

# Side view XZ
ax[1].scatter(m[:, 0], m[:, 2], c=m[:, 2], s=0.4, cmap="viridis",
              alpha=0.4, linewidths=0)
ax[1].plot(t[:, 0], t[:, 2], "-", color="red", lw=2.0)
ax[1].set_title("Side (XZ): trajectory should ride at sensor height")
ax[1].set_xlabel("x [m]"); ax[1].set_ylabel("z [m]"); ax[1].axis("equal")

plt.tight_layout()
plt.savefig(OUT, dpi=95)
print("saved", OUT)
