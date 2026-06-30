#!/usr/bin/env python3
"""Full-route trajectory (reject-off, rate 1.0) over the map, colored by NDT fitness.
Confirms the localizer rides through the ~340s ramp instead of diverging."""
import csv, os
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

RUNS = "/ws/output/dsbench/runs"
PLY = "/ws/gt_map/gt_map.ply"
TAG = "us050_norej"
OUT = "/ws/output/dsbench/traj_norej_fullroute.png"


def load_ply_xyz(path, stride=20):
    raw = open(path, "rb").read()
    e = raw.index(b"end_header\n") + len(b"end_header\n")
    return np.frombuffer(raw[e:], dtype=np.float32).reshape(-1, 4)[::stride, :3]


t, xyz = [], []
for r in csv.reader(open(f"{RUNS}/{TAG}/pose.csv")):
    if r[0] == "stamp":
        continue
    ts = float(r[0])
    if ts < 1e6:
        continue
    t.append(ts); xyz.append([float(r[1]), float(r[2]), float(r[3])])
t = np.array(t); xyz = np.array(xyz)
o = np.argsort(t); t = t[o]; xyz = xyz[o]
t0 = t[0]; rel = t - t0

fit = np.array([float(x) for x in open(f"{RUNS}/{TAG}/fitness.txt") if x.strip()])
# align fitness to poses (both are per-accepted-scan with reject-off); trim to min len
n = min(len(fit), len(xyz))
fit = fit[:n]; xyzf = xyz[:n]; relf = rel[:n]

m = load_ply_xyz(PLY)
print(f"poses={len(xyz)} span={rel[-1]:.1f}s path={np.linalg.norm(np.diff(xyz,axis=0),axis=1).sum():.0f}m "
      f"fit med={np.median(fit):.2f} p95={np.percentile(fit,95):.1f} max={fit.max():.1f}")

fig = plt.figure(figsize=(21, 9))
ax0 = fig.add_subplot(1, 2, 1)
ax0.scatter(m[:, 0], m[:, 1], c="0.85", s=0.3, linewidths=0)
sc = ax0.scatter(xyzf[:, 0], xyzf[:, 1], c=np.clip(fit, 0, 15), s=7, cmap="turbo", linewidths=0)
ax0.scatter(*xyz[0, :2], c="lime", s=160, edgecolors="k", zorder=5, label="start")
ax0.scatter(*xyz[-1, :2], c="magenta", s=160, marker="s", edgecolors="k", zorder=5, label="end")
ax0.set_title(f"Full route, rate 1.0, reject-off — 0.5m map ({len(xyz)} poses, {rel[-1]:.0f}s)\n"
              f"colored by NDT fitness (clipped 0-15)")
ax0.set_xlabel("x [m]"); ax0.set_ylabel("y [m]"); ax0.axis("equal"); ax0.legend(loc="upper right")
plt.colorbar(sc, ax=ax0, label="fitness", shrink=0.75)

# right: z(t) and fitness(t) to locate the ramp + hard spots
ax1 = fig.add_subplot(2, 2, 2)
ax1.plot(rel, xyz[:, 2], "-", color="#1f77b4", lw=1.2)
ax1.axvspan(325, 345, color="orange", alpha=0.2, label="~340s ramp")
ax1.set_ylabel("z [m]"); ax1.set_title("height vs time (the ramp climb)"); ax1.legend(loc="upper left")
ax1.grid(alpha=0.3)

ax2 = fig.add_subplot(2, 2, 4)
ax2.plot(relf, fit, "-", color="#d62728", lw=0.8)
ax2.axhline(5.0, color="k", ls="--", lw=1, label="old gate 5.0")
ax2.axvspan(325, 345, color="orange", alpha=0.2)
ax2.set_xlabel("bag time [s]"); ax2.set_ylabel("fitness"); ax2.set_ylim(0, 50)
ax2.set_title("NDT fitness vs time (spikes at ramp, pose still tracks)"); ax2.legend(loc="upper left")
ax2.grid(alpha=0.3)

plt.tight_layout()
plt.savefig(OUT, dpi=100)
print("saved", OUT)
