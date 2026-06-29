#!/usr/bin/env python3
"""Measure scovox occupancy map size: points/cells, bbox extent, occupied volume,
disk footprint. Compares rate-1.0 vs rate-0.5 captures and prints GLIM map size."""
import numpy as np
import os

RES = 0.2


def stat(path):
    if not os.path.exists(path):
        return None
    a = np.load(path)[:, :3].astype(np.float64)
    mn, mx = a.min(0), a.max(0)
    cells = np.unique(np.floor(a / RES + 0.5).astype(np.int64), axis=0)
    return dict(n=len(a), cells=len(cells), mn=mn, mx=mx, ext=mx - mn,
                fsz=os.path.getsize(path) / 1e6, vol=len(cells) * RES ** 3)


def show(name, s):
    if s is None:
        print(f"{name}: MISSING"); return
    print(f"=== {name} ===")
    print(f"  points (occupied voxels) = {s['n']:,}   unique cells = {s['cells']:,}")
    print(f"  bbox X[{s['mn'][0]:.1f},{s['mx'][0]:.1f}] "
          f"Y[{s['mn'][1]:.1f},{s['mx'][1]:.1f}] Z[{s['mn'][2]:.1f},{s['mx'][2]:.1f}] m")
    print(f"  extent = {s['ext'][0]:.1f} x {s['ext'][1]:.1f} x {s['ext'][2]:.1f} m   "
          f"footprint = {s['ext'][0] * s['ext'][1]:,.0f} m^2")
    print(f"  occupied volume = {s['vol']:,.0f} m^3 (0.2 m voxels)   .npy on disk = {s['fsz']:.1f} MB")


def pcd_n(path):
    with open(path, "rb") as f:
        for _ in range(20):
            ln = f.readline().decode("ascii", "replace")
            if ln.startswith("POINTS"):
                return int(ln.split()[1])
    return None


show("scovox rate 1.0", stat("/ws/output/scovox_map.npy"))
show("scovox rate 0.5", stat("/ws/output/scovox_map_rate05.npy"))
gp = "/ws/output/glim_map.pcd"
if os.path.exists(gp):
    print("=== GLIM map (reference) ===")
    print(f"  points = {pcd_n(gp):,}   .pcd on disk = {os.path.getsize(gp) / 1e6:.1f} MB")
