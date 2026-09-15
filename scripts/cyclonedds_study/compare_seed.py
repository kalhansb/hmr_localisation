#!/usr/bin/env python3
"""Seed-perturbation analysis.

Compares each perturbed-seed run against the nominal-seed run of the same config,
pose-to-pose at matched timestamps, in the map frame, with NO rigid alignment.
Alignment would absorb exactly the offset under test.

Converged  -> the map determines the pose; the shared frame is seed-independent.
Not        -> the pose is whatever you seeded; two robots would not share a frame.
"""
import os
import csv, math, os, sys
import numpy as np

O = os.environ.get('HMR_OUT', os.path.expanduser('~/jetbot-slam/hmr_localisation/output/jetson_test')) + '/'

def load(name):
    p = O + name + '.poses.csv'
    if not os.path.exists(p): return None
    rows = list(csv.DictReader(open(p)))[1:]          # row 0 is the seed itself
    if len(rows) < 10: return None
    t = np.array([float(r['stamp_sec']) for r in rows])
    xyz = np.array([[float(r['position_x']), float(r['position_y']), float(r['position_z'])] for r in rows])
    q = np.array([[float(r['orientation_x']), float(r['orientation_y']),
                   float(r['orientation_z']), float(r['orientation_w'])] for r in rows])
    yaw = np.arctan2(2*(q[:,3]*q[:,2] + q[:,0]*q[:,1]),
                     1 - 2*(q[:,1]**2 + q[:,2]**2))
    return t, xyz, yaw

def compare(ref, test):
    rt, rx, ry_ = ref; tt, tx, ty_ = test
    j = np.searchsorted(rt, tt)
    j = np.clip(j, 1, len(rt)-1)
    j = np.where(np.abs(rt[j-1]-tt) < np.abs(rt[j]-tt), j-1, j)
    ok = np.abs(rt[j]-tt) < 0.05
    d = np.linalg.norm(tx[ok] - rx[j[ok]], axis=1)
    dy = np.degrees(np.arctan2(np.sin(ty_[ok]-ry_[j[ok]]), np.cos(ty_[ok]-ry_[j[ok]])))
    return tt[ok]-tt[0], d, np.abs(dy)

CASES = sys.argv[1] if len(sys.argv) > 1 else \
        os.path.join(os.path.dirname(os.path.abspath(__file__)), 'results', 'seed_cases.txt')
REF = {'bunker': 'cyc_hmr_bunk_s4_r1', 'curtmini': 'cyc_hmr_curt_s4_r1'}

# noise floor: two nominal-seed runs of the same config against each other
for bag, a, b in [('bunker','cyc_hmr_bunk_s4_r1','cyc_hmr_bunk_s4_r2'),
                  ('curtmini','cyc_hmr_curt_s4_r1','cyc_hmr_curt_s4_r2')]:
    ra, rb = load(a), load(b)
    if ra and rb:
        t, d, dy = compare(ra, rb)
        print(f"noise floor {bag:9s}: two nominal runs differ by "
              f"{np.median(d)*100:.1f} cm median, {np.percentile(d,95)*100:.1f} cm p95")
print()

print("%-16s %8s | %s" % ("case", "seed err", "  position error vs nominal-seed run (m), by time"))
print("%-16s %8s | %7s %7s %7s %7s %7s   %s" % ("", "", "0-5s", "5-15s", "15-40s", "40-80s", "80s+", "yaw_end"))
print("-"*96)
for ln in open(CASES):
    f = ln.split()
    if len(f) < 6: continue
    n, bag, x, y = f[0], f[1], float(f[2]), float(f[3])
    ref = load(REF[bag]); test = load('seed_' + n)
    if ref is None or test is None:
        print("%-16s %8s | (no data yet)" % (n, "")); continue
    seed_err = math.hypot(x - ref[1][0][0], y - ref[1][0][1])
    t, d, dy = compare(ref, test)
    bins = [(0,5),(5,15),(15,40),(40,80),(80,1e9)]
    cells = []
    for lo,hi in bins:
        m = (t>=lo)&(t<hi)
        cells.append(f"{np.median(d[m]):7.3f}" if m.sum() else "      -")
    m = t >= 80
    ye = f"{np.median(dy[m]):5.2f}d" if m.sum() else "    -"
    print("%-16s %8.3f | %s   %s   first pose %+.3f m @ t=%.1fs"
          % (n, seed_err, " ".join(cells), ye, d[0], t[0]))
