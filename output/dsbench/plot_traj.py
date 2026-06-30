#!/usr/bin/env python3
"""Top-down trajectory overlay: full-map vs 0.5m-downsampled-map NDT localization."""
import csv
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

RUNS = "/ws/output/dsbench/runs"
PLY = "/ws/gt_map/gt_map.ply"
OUT = "/ws/output/dsbench/traj_full_vs_05.png"


def load_ply_xyz(path, stride=25):
    raw = open(path, "rb").read()
    e = raw.index(b"end_header\n") + len(b"end_header\n")
    pts = np.frombuffer(raw[e:], dtype=np.float32).reshape(-1, 4)
    return pts[::stride, :3]


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
ft, fx = load_traj("full")
qt, qx = load_traj("us050")

# timestamp-match us050 -> full to find the max-deviation location
order = np.argsort(ft); fts = ft[order]; fxs = fx[order]
idx = np.clip(np.searchsorted(fts, qt), 1, len(fts) - 1)
pick = np.where(np.abs(qt - fts[idx - 1]) <= np.abs(qt - fts[idx]), idx - 1, idx)
dt = np.abs(qt - fts[pick]); ok = dt <= 0.06
dev = np.linalg.norm(qx[ok] - fxs[pick[ok]], axis=1)
qx_ok = qx[ok]
kmax = int(np.argmax(dev)); cmax = qx_ok[kmax]
r = 1.2          # marker rectangle on the overview
rr = 0.5         # half-width of the zoom window (1m box) so a ~70mm gap is visible
print(f"matched {ok.sum()} poses; max dev {dev.max()*1000:.1f} mm at "
      f"({cmax[0]:.1f},{cmax[1]:.1f}); mean {dev.mean()*1000:.1f} mm")

fig, ax = plt.subplots(1, 2, figsize=(20, 9))

# ---- left: full top-down ----
ax[0].scatter(m[:, 0], m[:, 1], c="0.78", s=0.3, linewidths=0)
ax[0].plot(fx[:, 0], fx[:, 1], "-", color="#1f77b4", lw=2.2, label="full map (3.06M pts)")
ax[0].plot(qx[:, 0], qx[:, 1], "--", color="#d62728", lw=1.6, label="0.5m map (196k pts)")
ax[0].scatter(*fx[0, :2], c="lime", s=130, edgecolors="k", zorder=5, label="start")
ax[0].scatter(*fx[-1, :2], c="magenta", s=130, marker="s", edgecolors="k", zorder=5, label="end")
ax[0].add_patch(plt.Rectangle((cmax[0] - rr, cmax[1] - rr), 2 * rr, 2 * rr,
                              fill=False, ec="k", lw=1.5, zorder=6))
ax[0].set_title("Top-down (XY): full-map vs 0.5m-map trajectory  (overlap at building scale)")
ax[0].set_xlabel("x [m]"); ax[0].set_ylabel("y [m]"); ax[0].axis("equal")
ax[0].legend(loc="upper right")

# ---- right: zoom on max-deviation ----
ax[1].scatter(m[:, 0], m[:, 1], c="0.82", s=2, linewidths=0)
ax[1].plot(fx[:, 0], fx[:, 1], "-", color="#1f77b4", lw=3, label="full map")
ax[1].plot(qx[:, 0], qx[:, 1], "--", color="#d62728", lw=2.2, label="0.5m map")
ax[1].scatter(fx[:, 0], fx[:, 1], c="#1f77b4", s=18, zorder=4)
ax[1].scatter(qx[:, 0], qx[:, 1], c="#d62728", s=12, zorder=5)
ax[1].set_aspect("equal", adjustable="box")
ax[1].set_xlim(cmax[0] - rr, cmax[0] + rr); ax[1].set_ylim(cmax[1] - rr, cmax[1] + rr)
ax[1].set_title(f"Zoom @ max deviation = {dev.max()*1000:.0f} mm  (mean {dev.mean()*1000:.0f} mm)")
ax[1].set_xlabel("x [m]"); ax[1].set_ylabel("y [m]")
ax[1].legend(loc="upper right")

plt.tight_layout()
plt.savefig(OUT, dpi=100)
print("saved", OUT)
