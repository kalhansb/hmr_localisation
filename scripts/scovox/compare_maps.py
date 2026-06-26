#!/usr/bin/env python3
# Compare the captured scovox occupancy map (/ws/output/scovox_map.npy) against
# the GLIM ground-truth map (/ws/gt_map/gt_map.ply). Both are in the `map` frame.
#
# Reports the standard point-cloud / reconstruction-quality metric suite:
#   - Accuracy/Precision (scovox -> GT)      : are scovox points real (on GT)?
#   - Completeness/Recall (GT_region -> scovox): did scovox cover the surface?
#   - Chamfer (mean both dirs), Hausdorff (max + p95)
#   - F-score @ tau (precision/recall harmonic mean)
#   - Voxel occupancy: TP/FP/FN -> precision, recall, IoU, F1 (density-robust)
# Plus a top-down overlay PNG colouring matched vs unmatched scovox points.
import numpy as np
from scipy.spatial import cKDTree

GT = "/ws/gt_map/gt_map.ply"
SC = "/ws/output/scovox_map.npy"
PNG = "/ws/output/map_vs_gt.png"
TAUS = [0.2, 0.35, 0.5, 1.0]   # match thresholds [m]
VOX = 0.3                       # voxel size for occupancy metrics [m]
REGION_PAD = 1.0               # pad scovox bbox to define the "observed" GT region


def load_ply_xyz(path):
    with open(path, "rb") as f:
        header = b""
        while b"end_header" not in header:
            line = f.readline()
            if not line:
                raise RuntimeError("no end_header")
            header += line
        n, ncols = 0, 0
        for ln in header.decode("ascii", "replace").splitlines():
            if ln.startswith("element vertex"):
                n = int(ln.split()[-1])
            elif ln.startswith("property"):
                ncols += 1
        data = np.fromfile(f, dtype=np.float32, count=n * ncols).reshape(n, ncols)
        return data[:, :3].astype(np.float64)


def dstats(name, d):
    return (f"  {name}: mean={d.mean():.3f}  median={np.median(d):.3f}  rms={np.sqrt((d**2).mean()):.3f}  "
            f"p90={np.percentile(d,90):.3f}  p95={np.percentile(d,95):.3f}  max={d.max():.3f}  [m]")


def main():
    gt = load_ply_xyz(GT)
    sc = np.load(SC)
    smn, smx = sc.min(0), sc.max(0)

    print("=== clouds (map frame) ===")
    print(f"  GT    : {len(gt):>8d} pts  bbox={np.round(gt.min(0),1)}..{np.round(gt.max(0),1)}")
    print(f"  scovox: {len(sc):>8d} pts  bbox={np.round(smn,1)}..{np.round(smx,1)}")
    off = sc.mean(0) - gt.mean(0)
    print(f"  centroid offset (scovox-GT)={np.round(off,2)}  |.|={np.linalg.norm(off):.2f} m")

    # GT restricted to the region scovox actually observed (for fair recall/FN)
    lo, hi = smn - REGION_PAD, smx + REGION_PAD
    gin = gt[np.all((gt >= lo) & (gt <= hi), axis=1)]

    # nearest-neighbour distances both directions
    d_s2g, _ = cKDTree(gt).query(sc, k=1, workers=-1)      # accuracy / precision
    d_g2s, _ = cKDTree(sc).query(gin, k=1, workers=-1)     # completeness / recall

    print("\n=== ACCURACY  scovox -> GT  (are scovox points real?) ===")
    print(dstats("dist", d_s2g))
    print("\n=== COMPLETENESS  GT_region -> scovox  (did scovox cover the surface?) ===")
    print(f"  GT_region={len(gin)} pts (scovox bbox +{REGION_PAD} m)")
    print(dstats("dist", d_g2s))

    print("\n=== CHAMFER / HAUSDORFF ===")
    print(f"  Chamfer-L2 (mean of both means) = {0.5*(d_s2g.mean()+d_g2s.mean()):.3f} m")
    print(f"  Hausdorff (max)                 = {max(d_s2g.max(), d_g2s.max()):.3f} m")
    print(f"  Hausdorff robust (max p95)      = {max(np.percentile(d_s2g,95), np.percentile(d_g2s,95)):.3f} m")

    print("\n=== PRECISION / RECALL / F-SCORE @ tau (point-based) ===")
    for t in TAUS:
        p = float(np.mean(d_s2g < t))      # precision: scovox pts near GT
        r = float(np.mean(d_g2s < t))      # recall: GT_region pts near scovox
        f = 2 * p * r / (p + r) if (p + r) else 0.0
        print(f"  tau={t:4.2f} m :  precision={p*100:5.1f}%   recall={r*100:5.1f}%   F-score={f*100:5.1f}%")

    print(f"\n=== VOXEL OCCUPANCY @ {VOX} m (within scovox bbox) ===")
    vox = lambda a: set(map(tuple, np.floor(a / VOX).astype(np.int64)))
    A, B = vox(sc), vox(gin)
    TP = len(A & B); FP = len(A - B); FN = len(B - A)
    P = TP / max(TP + FP, 1); R = TP / max(TP + FN, 1)
    IoU = TP / max(TP + FP + FN, 1); F1 = 2 * P * R / max(P + R, 1e-9)
    print(f"  scovox voxels={len(A)}  GT voxels={len(B)}")
    print(f"  TP={TP}  FP={FP}  FN={FN}")
    print(f"  precision={P*100:.1f}%   recall={R*100:.1f}%   IoU={IoU:.3f}   F1={F1*100:.1f}%")

    # headline match rate + unmatched export
    THRESH = 0.35
    matched = d_s2g < THRESH
    nun = int((~matched).sum())
    print(f"\n>>> MATCH RATE: {matched.mean()*100:.1f}% of scovox points lie within {THRESH} m of GT "
          f"({nun}/{len(sc)} unmatched)")
    if nun:
        np.save("/ws/output/scovox_unmatched.npy", sc[~matched])

    # --- top-down overlay ---
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    rng = np.random.default_rng(0)
    gsub = gin[rng.choice(len(gin), min(len(gin), 250000), replace=False)] if len(gin) else gin
    fig, ax = plt.subplots(figsize=(12, 12))
    if len(gsub):
        ax.scatter(gsub[:, 0], gsub[:, 1], s=0.5, c="0.8", label="GT surface", linewidths=0)
    ax.scatter(sc[matched, 0], sc[matched, 1], s=1.0, c="green",
               label=f"scovox matches GT (<{THRESH} m)", linewidths=0)
    if nun:
        ax.scatter(sc[~matched, 0], sc[~matched, 1], s=2.0, c="red",
                   label=f"scovox unmatched ({nun})", linewidths=0)
    ax.set_aspect("equal"); ax.legend(markerscale=8, loc="upper right")
    ax.set_title(f"scovox vs GT [XY] — match {matched.mean()*100:.1f}%  IoU {IoU:.2f}  "
                 f"F@0.35 {2*np.mean(d_s2g<0.35)*np.mean(d_g2s<0.35)/max(np.mean(d_s2g<0.35)+np.mean(d_g2s<0.35),1e-9)*100:.0f}%")
    ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]"); ax.grid(True, alpha=0.3)
    fig.savefig(PNG, dpi=110, bbox_inches="tight")
    print(f"\nwrote {PNG}")


main()
