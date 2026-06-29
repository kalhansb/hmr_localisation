#!/usr/bin/env python3
"""Top-down overlay of GLIM's global map (binary XYZ PCD) + trajectory (CSV).

Unlike scripts/plot_zoom.py (which reads the 4-float gt_map PLY), this reads the
3-float binary PCD written by scripts/glim/save_glim_map.py and the pose CSV from
record_glim_pose.py. The dense GLIM map (~16 M pts) is randomly subsampled for a
fast, light figure.

Usage:  python3 scripts/glim/plot_glim.py output/glim_map.pcd output/path_glim.csv output/eval_glim.png [max_map_pts]
"""
import sys
import csv
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

PCD, CSVF, OUT = sys.argv[1], sys.argv[2], sys.argv[3]
MAX = int(sys.argv[4]) if len(sys.argv) > 4 else 800000


def load_pcd_xyz(path):
    """Read a binary PCD with FIELDS x y z (3x float32)."""
    with open(path, "rb") as f:
        raw = f.read()
    hdr_end = raw.index(b"DATA binary\n") + len(b"DATA binary\n")
    header = raw[:hdr_end].decode("ascii", "replace")
    n = None
    for line in header.splitlines():
        if line.startswith("POINTS"):
            n = int(line.split()[1])
    pts = np.frombuffer(raw[hdr_end:], dtype="<f4")
    pts = pts[: (n if n else len(pts) // 3) * 3].reshape(-1, 3)
    finite = np.isfinite(pts).all(axis=1)
    return pts[finite]


def main():
    m = load_pcd_xyz(PCD)
    if m.shape[0] > MAX:
        idx = np.random.default_rng(0).choice(m.shape[0], MAX, replace=False)
        m = m[idx]
    rows = list(csv.reader(open(CSVF)))[1:]
    t = np.array([[float(r[1]), float(r[2]), float(r[3])] for r in rows])

    fig, ax = plt.subplots(figsize=(11, 11))
    ax.scatter(m[:, 0], m[:, 1], s=0.2, c="0.7", linewidths=0, label=f"GLIM map ({m.shape[0]} pts shown)")
    sc = ax.scatter(t[:, 0], t[:, 1], s=4, c=np.arange(len(t)), cmap="viridis", label="GLIM trajectory")
    ax.scatter([t[0, 0]], [t[0, 1]], s=120, marker="o", edgecolor="k", facecolor="lime", zorder=5, label="start")
    ax.scatter([t[-1, 0]], [t[-1, 1]], s=120, marker="*", edgecolor="k", facecolor="red", zorder=5, label="end")
    ax.set_aspect("equal")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_title(f"GLIM SLAM — {len(t)} poses, top-down (x-y)")
    ax.legend(loc="best", markerscale=3)
    fig.colorbar(sc, ax=ax, label="pose index (time)")
    fig.tight_layout()
    fig.savefig(OUT, dpi=120)
    print(f"wrote {OUT}  (map {m.shape[0]} pts, {len(t)} poses)")


if __name__ == "__main__":
    main()
