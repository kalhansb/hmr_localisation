#!/usr/bin/env python3
"""Compare the spatial extent of scovox map vs GLIM map vs GLIM trajectory, to tell
whether scovox's map 'size' (extent) is geometrically correct (should match GLIM &
the path the robot drove) or genuinely wrong."""
import numpy as np
import os


def npy_ext(p):
    a = np.load(p)[:, :3].astype(np.float64)
    return a.min(0), a.max(0), len(a)


def pcd_ext(p, cap=3_000_000):
    with open(p, "rb") as f:
        n = 0
        while True:
            ln = f.readline().decode("ascii", "replace")
            if ln.startswith("POINTS"): n = int(ln.split()[1])
            if ln.startswith("DATA"): fmt = ln.split()[1].strip(); break
        if fmt == "binary":
            buf = f.read(n * 12)
            a = np.frombuffer(buf, np.float32, n * 3).reshape(-1, 3).astype(np.float64)
        else:
            a = np.loadtxt(p, skiprows=12)[:, :3]
    return a.min(0), a.max(0), len(a)


def traj_ext(p):
    rows = []
    for ln in open(p):
        q = ln.strip().split(",")
        if len(q) >= 4:
            try: rows.append([float(q[1]), float(q[2]), float(q[3])])
            except ValueError: pass
    a = np.asarray(rows)
    return a.min(0), a.max(0), len(a)


def show(name, mn, mx, n, extra=""):
    e = mx - mn
    print(f"{name:<22} n={n:>12,}  X[{mn[0]:7.1f},{mx[0]:6.1f}] Y[{mn[1]:7.1f},{mx[1]:6.1f}] "
          f"Z[{mn[2]:6.1f},{mx[2]:6.1f}]  extent {e[0]:5.1f} x {e[1]:5.1f} x {e[2]:5.1f} m  {extra}")


O = "/ws/output"
print("Frames: scovox/path in odom ; GLIM map in map (odom~=map within ~0.3 m here)\n")
mn, mx, n = traj_ext(f"{O}/path_glim.csv"); show("GLIM trajectory", mn, mx, n, "(where the robot drove)")
mn, mx, n = pcd_ext(f"{O}/glim_map.pcd"); show("GLIM map (rate1.0)", mn, mx, n)
mn, mx, n = npy_ext(f"{O}/scovox_map.npy"); show("scovox map (rate1.0)", mn, mx, n)
# trajectory + typical sensor range tells the expected map extent
tmn, tmx, _ = traj_ext(f"{O}/path_glim.csv")
print(f"\nExpected map extent ~= trajectory span + 2*max_range(15m):")
print(f"  X ~ {(tmx-tmn)[0]+30:.0f} m, Y ~ {(tmx-tmn)[1]+30:.0f} m  (vs scovox X/Y above)")
