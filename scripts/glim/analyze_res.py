#!/usr/bin/env python3
"""Find the scovox resolution that best MATCHES the downsampled deskewed GLIM points.

Reference = /ws/output/glim_points_accum.npy (accumulated /glim_ros/aligned_points,
map frame) -- literally 'the downsampled deskewed GLIM points'.

For each scovox map (odom frame) at resolution R:
  - per-config translation ICP align to the reference (odom vs map offset)
  - MATCH metrics vs the GLIM points:
      coverage/recall = frac of GLIM ref points within 1.5*R of a scovox voxel
      precision       = frac of scovox voxels within 1.5*R of a GLIM ref point
                        (low precision => scovox has surfaces GLIM points miss = noise OR
                         genuine fill; high => faithful)
      surface-hole frac = of XY columns the GLIM ref occupies, frac NOT covered by scovox
  - SIZE: occupied voxels, MB
The 'match' sweet spot: finest R that keeps coverage high AND surface-hole low.

Usage: python3 analyze_res.py [res_sweep_dir] [glim_points_accum.npy]
"""
import os, sys, glob
import numpy as np
from scipy.spatial import cKDTree

DIR = sys.argv[1] if len(sys.argv) > 1 else "/ws/output/res_sweep"
REF = sys.argv[2] if len(sys.argv) > 2 else "/ws/output/glim_map_res.pcd"
TRAJ = sys.argv[3] if len(sys.argv) > 3 else "/ws/output/path_glim_res.csv"
R_REGION = 15.0          # scovox max_range -> compare only GLIM points scovox could see
ORDER = ["res020", "res010", "res005"]
RES = {"res020": 0.20, "res010": 0.10, "res005": 0.05}


def load_ref(path, traj_path):
    """Reference = GLIM deskewed map, restricted to within R_REGION of the trajectory
    (the region scovox's 15 m range can actually observe)."""
    if path.endswith(".pcd"):
        with open(path, "rb") as f:
            n = 0
            while True:
                l = f.readline().decode("ascii", "replace")
                if l.startswith("POINTS"): n = int(l.split()[1])
                if l.startswith("DATA"): fmt = l.split()[1].strip(); break
            a = (np.frombuffer(f.read(n * 12), np.float32, n * 3).reshape(-1, 3).astype(np.float64)
                 if fmt == "binary" else np.loadtxt(path, skiprows=12)[:, :3])
    else:
        a = np.load(path)[:, :3].astype(np.float64)
    rows = []
    for ln in open(traj_path):
        q = ln.strip().split(",")
        if len(q) >= 4:
            try: rows.append([float(q[1]), float(q[2]), float(q[3])])
            except ValueError: pass
    traj = np.asarray(rows)
    d, _ = cKDTree(traj).query(a, k=1, workers=-1)
    return a[d <= R_REGION]


def colset(p, res):
    return set(map(tuple, np.unique(np.floor(p[:, :2] / res + 0.5).astype(np.int64), axis=0)))


def align(sc, ref_tree, ref):
    rng = np.random.default_rng(0)
    sub = sc[rng.choice(len(sc), size=min(80000, len(sc)), replace=False)]
    shift = np.zeros(3)
    for _ in range(6):
        d, idx = ref_tree.query(sub + shift, k=1, workers=-1)
        keep = d < np.percentile(d, 70)
        step = np.median(ref[idx[keep]] - (sub[keep] + shift), axis=0)
        shift += step
        if np.linalg.norm(step) < 1e-3:
            break
    return shift


def main():
    if not os.path.exists(REF):
        print(f"MISSING reference {REF}"); return
    ref = load_ref(REF, TRAJ)
    print(f"GLIM deskewed-points reference (within {R_REGION} m of path): {len(ref):,} pts  "
          f"bbox {np.round(ref.min(0),1)}..{np.round(ref.max(0),1)}")
    ref_tree = cKDTree(ref)

    rows = []
    for p in sorted(glob.glob(os.path.join(DIR, "*.npy")),
                    key=lambda q: ORDER.index(os.path.basename(q)[:-4]) if os.path.basename(q)[:-4] in ORDER else 9):
        cid = os.path.basename(p)[:-4]; R = RES.get(cid, 0.2)
        sc = np.load(p)[:, :3].astype(np.float64)
        sc = sc + align(sc, ref_tree, ref)
        tol = 1.5 * R
        # coverage: GLIM points that have a scovox voxel nearby
        d_r2s, _ = cKDTree(sc).query(ref, k=1, workers=-1)
        cov = float((d_r2s <= tol).mean())
        # precision: scovox voxels near a GLIM point
        d_s2r, _ = ref_tree.query(sc, k=1, workers=-1)
        prec = float((d_s2r <= tol).mean())
        # surface holes: XY columns (at 0.2 m, common grid) GLIM occupies but scovox doesn't
        gcol = colset(ref, 0.2); scol = colset(sc, 0.2)
        hole = 1.0 - len(gcol & scol) / len(gcol)
        mb = os.path.getsize(p) / 1e6
        rows.append((cid, R, len(sc), mb, cov, prec, hole))
        print(f"\n=== {cid}  (resolution {R} m) ===")
        print(f"  occupied voxels = {len(sc):,}   .npy = {mb:.1f} MB")
        print(f"  coverage(GLIM pts w/ scovox within {tol:.2f} m) = {cov*100:.1f}%")
        print(f"  precision(scovox near GLIM pts)                = {prec*100:.1f}%")
        print(f"  surface holes (GLIM XY cols uncovered @0.2m)   = {hole*100:.1f}%")

    print("\n================ RESOLUTION SWEEP SUMMARY ================")
    h = f"{'config':<9}{'res_m':>6}{'voxels':>12}{'MB':>7}{'cover%':>8}{'prec%':>7}{'holes%':>8}"
    print(h); print("-" * len(h))
    for cid, R, n, mb, cov, prec, hole in rows:
        print(f"{cid:<9}{R:>6.2f}{n:>12,}{mb:>7.1f}{cov*100:>8.1f}{prec*100:>7.1f}{hole*100:>8.1f}")
    print("\nMatch sweet spot = finest res with coverage high AND holes low.")


main()
