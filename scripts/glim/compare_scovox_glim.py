#!/usr/bin/env python3
# Compare the SCovox occupancy map (output/scovox_map.npy) against GLIM's OWN
# geometric SLAM map (output/glim_map.pcd). Both must come from the SAME run so
# they share GLIM's "map" frame and are co-registered (no alignment step).
#
# This script was hardened after an adversarial methodology review. Key points:
#  * ACCURACY (scovox->GLIM) is the headline -- the only metric that is fair under
#    the asymmetries below. It is computed against the FULL GLIM cloud (not a
#    bbox-clipped subset) so edge voxels are not penalised.
#  * SCovox emits 0.2 m OCCUPIED voxel CENTERS, so even a perfect voxel sits up to
#    res*sqrt(3)/2 = 0.173 m (median ~res/2 = 0.10 m) from GLIM's true surface.
#    This QUANTIZATION FLOOR is annotated next to every accuracy number; tau below
#    it measures quantization, not error.
#  * COMPLETENESS/recall is OBSERVABILITY-LIMITED, not an error rate. The "observed
#    region" is GLIM points within max_range=15 m (3-D) of the GLIM trajectory
#    (output/path_glim.csv) -- exactly SCovox's integration cap -- which removes
#    far/never-seeable geometry. Recall is also binned by range to show it degrade.
#  * Occupancy IoU voxelizes BOTH clouds onto the SAME 0.2 m lattice with ROUND
#    (not floor, which collapsed scovox's on-boundary centers) for an apples-to-
#    apples cell-vs-cell comparison, restricted to the observed region.
#  * A CO-REGISTRATION GUARD aborts if the maps are not actually same-run aligned.
#
# Usage (container has numpy+scipy+matplotlib):
#   python3 scripts/glim/compare_scovox_glim.py [scovox.npy] [glim.pcd] [traj.csv] [out.png]
import sys, os, csv
import numpy as np
from scipy.spatial import cKDTree

SC   = sys.argv[1] if len(sys.argv) > 1 else "/ws/output/scovox_map.npy"
GLIM = sys.argv[2] if len(sys.argv) > 2 else "/ws/output/glim_map.pcd"
TRAJ = sys.argv[3] if len(sys.argv) > 3 else "/ws/output/path_glim.csv"
PNG  = sys.argv[4] if len(sys.argv) > 4 else "/ws/output/scovox_vs_glim.png"

RES = 0.20                      # SCovox voxel size [m]
FLOOR_MED = RES / 2             # 0.100 m  -- expected median accuracy for a perfect map
FLOOR_MAX = RES * np.sqrt(3)/2  # 0.173 m  -- worst voxel-center-to-corner offset
MAX_RANGE = 15.0                # SCovox integration cap -> defines observed region [m]
TAUS = [FLOOR_MAX, RES, 0.35, 0.5, 1.0]   # <=RES are quantization baselines, not pass/fail
COREG_TOL = float(os.environ.get("COREG_TOL", "0.5"))   # abort if un-co-registered [m]


def load_pcd_xyz(path):
    """Binary PCD, FIELDS x y z (3x float32). Fails loudly on any mismatch."""
    with open(path, "rb") as f:
        raw = f.read()
    key = b"DATA binary\n"
    if key not in raw:
        raise RuntimeError(f"{path}: not an uncompressed binary PCD (no 'DATA binary')")
    hdr_end = raw.index(key) + len(key)
    fields = sizes = None; n = None
    for line in raw[:hdr_end].decode("ascii", "replace").splitlines():
        t = line.split()
        if not t: continue
        if t[0] == "FIELDS": fields = t[1:]
        elif t[0] == "SIZE": sizes = list(map(int, t[1:]))
        elif t[0] == "POINTS": n = int(t[1])
    if fields != ["x", "y", "z"] or sizes != [4, 4, 4]:
        raise RuntimeError(f"{path}: expected FIELDS x y z / SIZE 4 4 4, got {fields}/{sizes}")
    payload = raw[hdr_end:]
    if len(payload) != n * 12:
        raise RuntimeError(f"{path}: payload {len(payload)} != POINTS*12 ({n*12}) -- truncated/padded")
    pts = np.frombuffer(payload, dtype="<f4").reshape(n, 3).astype(np.float64)
    return pts[np.isfinite(pts).all(axis=1)]


def load_traj(path):
    xs = []
    with open(path) as f:
        for row in csv.DictReader(f):
            xs.append([float(row["x"]), float(row["y"]), float(row["z"])])
    return np.asarray(xs, dtype=np.float64)


def dstats(name, d):
    return (f"  {name}: median={np.median(d):.3f}  mean={d.mean():.3f}  rms={np.sqrt((d**2).mean()):.3f}  "
            f"p90={np.percentile(d,90):.3f}  p95={np.percentile(d,95):.3f}  max={d.max():.3f}  [m]")


def vox_round(a, res):
    """Voxel keys by ROUNDING to lattice (scovox centers sit ON 0.2 m boundaries;
    np.floor would split them across neighbouring cells)."""
    return set(map(tuple, np.round(a / res).astype(np.int64)))


def main():
    sc = np.load(SC)
    sc = sc.reshape(-1, sc.shape[-1])[:, :3].astype(np.float64)
    sc = sc[np.isfinite(sc).all(axis=1)]
    gl = load_pcd_xyz(GLIM)
    traj = load_traj(TRAJ)
    smn, smx = sc.min(0), sc.max(0)

    print("=== inputs (expect same run, GLIM 'map' frame) ===")
    print(f"  GLIM  : {len(gl):>9d} pts  bbox={np.round(gl.min(0),1)}..{np.round(gl.max(0),1)}")
    print(f"  scovox: {len(sc):>9d} pts  bbox={np.round(smn,1)}..{np.round(smx,1)}")
    print(f"  traj  : {len(traj):>9d} poses  ({TRAJ})")

    # ---- accuracy: scovox -> FULL GLIM (the fair headline metric) ----
    d_s2g, _ = cKDTree(gl).query(sc, k=1, workers=-1)

    # ---- CO-REGISTRATION GUARD ----
    med = float(np.median(d_s2g))
    # centroid offset over the co-observed overlap (scovox vs GLIM in scovox bbox)
    gin_bbox = gl[np.all((gl >= smn - 1.0) & (gl <= smx + 1.0), axis=1)]
    off = sc.mean(0) - gin_bbox.mean(0) if len(gin_bbox) else np.full(3, np.nan)
    print("\n=== CO-REGISTRATION CHECK ===")
    print(f"  median(accuracy) = {med:.3f} m   region centroid offset |.| = {np.linalg.norm(off):.3f} m")
    if med > COREG_TOL:
        print(f"  !!! NOT CO-REGISTERED: median accuracy {med:.2f} m > {COREG_TOL} m. The maps are not")
        print(f"      from the same GLIM 'map'-frame run (stale file / frame mismatch). ABORTING -- the")
        print(f"      numbers below would measure misalignment, not SCovox accuracy.")
        sys.exit(2)
    print(f"  OK: maps are co-registered (median accuracy << route scale).")

    print("\n=== ACCURACY  scovox -> GLIM  [HEADLINE: are scovox voxels on real geometry?] ===")
    print(dstats("dist", d_s2g))
    print(f"  quantization floor: a perfect 0.2 m voxel map has median~{FLOOR_MED:.3f} m, "
          f"max corner offset {FLOOR_MAX:.3f} m -> read sub-{RES:.1f} m distances as quantization, not error.")
    print("  precision (fraction of scovox voxels within tau of GLIM):")
    for t in TAUS:
        tag = "  [quantization baseline]" if t <= RES + 1e-9 else ""
        print(f"    tau={t:4.3f} m : {100*np.mean(d_s2g<t):5.1f}%{tag}")

    # ---- observed region = GLIM within MAX_RANGE of the trajectory (3-D) ----
    d_traj, _ = cKDTree(traj).query(gl, k=1, workers=-1)
    region = d_traj <= MAX_RANGE
    gin = gl[region]; gin_d = d_traj[region]
    print(f"\n=== OBSERVED REGION = GLIM within {MAX_RANGE:.0f} m of trajectory ===")
    print(f"  {len(gin)} / {len(gl)} GLIM pts ({100*len(gin)/len(gl):.1f}%) are within sensor range of a pose.")
    if len(gin) == 0:
        print("  empty observed region -- cannot score completeness."); return

    # ---- completeness: GLIM_region -> scovox (OBSERVABILITY-LIMITED coverage) ----
    d_g2s, _ = cKDTree(sc).query(gin, k=1, workers=-1)
    print("\n=== COMPLETENESS  GLIM_region -> scovox  [coverage, NOT error] ===")
    print(dstats("dist", d_g2s))
    print("  recall (fraction of observed GLIM surface within tau of a scovox voxel):")
    for t in [RES, 0.35, 0.5, 1.0]:
        print(f"    tau={t:4.2f} m : {100*np.mean(d_g2s<t):5.1f}%")
    print("  recall@0.35 m binned by range-to-trajectory (coverage should fall with range):")
    for lo, hi in [(0, 5), (5, 10), (10, 15)]:
        m = (gin_d >= lo) & (gin_d < hi)
        r = 100*np.mean(d_g2s[m] < 0.35) if m.any() else float("nan")
        print(f"    {lo:2d}-{hi:2d} m : {r:5.1f}%   ({m.sum()} pts)")

    # ---- occupancy IoU: voxelize BOTH to the SAME 0.2 m lattice (apples-to-apples) ----
    A = vox_round(sc, RES); B = vox_round(gin, RES)
    TP = len(A & B); FP = len(A - B); FN = len(B - A)
    P = TP/max(TP+FP,1); R = TP/max(TP+FN,1); IoU = TP/max(TP+FP+FN,1)
    print(f"\n=== OCCUPANCY IoU @ {RES} m lattice (both voxelized; region-restricted) ===")
    print(f"  scovox cells={len(A)}  GLIM_region cells={len(B)}   TP={TP} FP={FP} FN={FN}")
    print(f"  cell-precision={100*P:.1f}%  cell-recall={100*R:.1f}%  IoU={IoU:.3f}  F1={100*2*P*R/max(P+R,1e-9):.1f}%")

    # ---- symmetric (report cautiously) ----
    print("\n=== SYMMETRIC (caveated) ===")
    print(f"  one-sided means: accuracy={d_s2g.mean():.3f} m  completeness={d_g2s.mean():.3f} m")
    print(f"  Hausdorff robust (max p95 of both)= {max(np.percentile(d_s2g,95), np.percentile(d_g2s,95)):.3f} m")
    print(f"  Hausdorff raw max (outlier-driven)= {max(d_s2g.max(), d_g2s.max()):.3f} m")

    # ---- headline + overlay ----
    THRESH = 0.35
    matched = d_s2g < THRESH
    nun = int((~matched).sum())
    print(f"\n>>> SCovox accuracy: median {med*100:.0f} mm; {100*matched.mean():.1f}% of scovox voxels within "
          f"{THRESH} m of GLIM geometry ({nun}/{len(sc)} unmatched). Observed-region coverage "
          f"(recall@0.35)={100*np.mean(d_g2s<0.35):.1f}%, occupancy IoU={IoU:.2f}.")
    if nun:
        np.save(os.path.splitext(PNG)[0] + "_unmatched.npy", sc[~matched])

    import matplotlib; matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    rng = np.random.default_rng(0)
    gsub = gin[rng.choice(len(gin), min(len(gin), 300000), replace=False)]
    fig, ax = plt.subplots(figsize=(12, 12))
    ax.scatter(gsub[:,0], gsub[:,1], s=0.4, c="0.8", label="GLIM surface (observed region)", linewidths=0)
    ax.scatter(sc[matched,0], sc[matched,1], s=1.0, c="green", label=f"scovox match (<{THRESH} m)", linewidths=0)
    if nun:
        ax.scatter(sc[~matched,0], sc[~matched,1], s=2.0, c="red", label=f"scovox unmatched ({nun})", linewidths=0)
    ax.plot(traj[:,0], traj[:,1], "-", c="royalblue", lw=1.0, label="GLIM trajectory")
    ax.set_aspect("equal"); ax.legend(markerscale=8, loc="upper right")
    ax.set_title(f"SCovox vs GLIM [XY] — accuracy median {med*100:.0f} mm  match {100*matched.mean():.1f}%  IoU {IoU:.2f}")
    ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]"); ax.grid(True, alpha=0.3)
    fig.savefig(PNG, dpi=110, bbox_inches="tight")
    print(f"\nwrote {PNG}")


main()
