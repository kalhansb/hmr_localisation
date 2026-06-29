#!/usr/bin/env python3
"""Comprehensive SCovox occupancy sweep analysis (run in container: numpy+scipy+matplotlib).

For each /ws/output/sweep/<id>.npy (SCovox occupancy, odom frame) compares against
this run's GLIM SLAM map (/ws/output/glim_map_sweep.pcd) and reports:

  PRIMARY (intrinsic, alignment-free) -- "is it still a 3-D blob?"
    z-cells per occupied XY column: median / mean / p90 / frac>=5, median z-extent[m]
    n occupied cells, XY footprint columns
  SECONDARY (vs GLIM, single shared translation alignment to kill odom<->map offset)
    accuracy = median NN scovox->GLIM [m]; precision@tau; recall@tau (coverage kept?);
    IoU @0.2 lattice (region-restricted)

The over-fill metric is the headline: GLIM represents the scene as thin surfaces
(~2 z-cells/col); a good SCovox config should approach that WITHOUT losing recall.

Usage: python3 analyze_sweep.py [sweep_dir] [glim.pcd] [traj.csv] [out_png]
"""
import os, sys, glob
import numpy as np
from scipy.spatial import cKDTree
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

SWEEP = sys.argv[1] if len(sys.argv) > 1 else "/ws/output/sweep"
GLIM  = sys.argv[2] if len(sys.argv) > 2 else "/ws/output/glim_map_sweep.pcd"
TRAJ  = sys.argv[3] if len(sys.argv) > 3 else "/ws/output/path_glim_sweep.csv"
OUTPNG= sys.argv[4] if len(sys.argv) > 4 else "/ws/output/sweep/sweep_analysis.png"

RES = 0.20
R_REGION = 10.0          # observed region = GLIM within this of trajectory (all configs have max_range>=10)
TAU = 0.35               # match tolerance for precision/recall [m]


def load_pcd_xyz(path):
    with open(path, "rb") as f:
        hdr, n = [], 0
        while True:
            line = f.readline().decode("ascii", "replace")
            hdr.append(line)
            if line.startswith("POINTS"):
                n = int(line.split()[1])
            if line.startswith("DATA"):
                fmt = line.split()[1].strip()
                break
        if fmt == "binary":
            buf = f.read(n * 12)
            return np.frombuffer(buf, dtype=np.float32, count=n * 3).reshape(-1, 3).astype(np.float64)
        else:
            return np.loadtxt(path, skiprows=len(hdr))[:, :3]


def load_traj(path):
    rows = []
    with open(path) as f:
        for ln in f:
            p = ln.strip().split(",")
            if len(p) >= 4:
                try: rows.append([float(p[1]), float(p[2]), float(p[3])])
                except ValueError: pass
    return np.asarray(rows)


def vox(p):                      # integer voxel coords on the 0.2 lattice
    return np.floor(p / RES + 0.5).astype(np.int64)


def colstats(cells):
    """z-cells-per-XY-column stats from integer voxel coords (n,3)."""
    order = np.lexsort((cells[:, 2], cells[:, 1], cells[:, 0]))
    c = cells[order]
    xy = c[:, :2]
    # group by xy
    same = np.all(xy[1:] == xy[:-1], axis=1)
    bnd = np.where(~same)[0] + 1
    groups = np.split(c[:, 2], bnd)
    zc = np.array([len(g) for g in groups])                 # z-cells per column
    zext = np.array([(g.max() - g.min() + 1) for g in groups]) * RES   # z extent [m]
    return dict(ncols=len(zc),
                z_med=float(np.median(zc)), z_mean=float(zc.mean()),
                z_p90=float(np.percentile(zc, 90)), z_ge5=float((zc >= 5).mean()),
                zext_med=float(np.median(zext)), zc=zc)


def main():
    print(f"loading GLIM {GLIM} ...", flush=True)
    gl = load_pcd_xyz(GLIM)
    traj = load_traj(TRAJ)
    print(f"  GLIM pts={len(gl):,}  traj poses={len(traj):,}")

    # observed region: GLIM within R_REGION of trajectory
    dtr, _ = cKDTree(traj).query(gl, k=1, workers=-1)
    gin = gl[dtr <= R_REGION]
    print(f"  GLIM observed-region pts={len(gin):,} (<= {R_REGION} m of traj)")
    gl_tree = cKDTree(gl)
    gin_tree = cKDTree(gin)
    g_cells = np.unique(vox(gin), axis=0)
    gstat = colstats(g_cells)
    gset = set(map(tuple, g_cells))

    npys = sorted(glob.glob(os.path.join(SWEEP, "*.npy")))
    # shared translation alignment from the densest baseline-like config (c0 if present)
    align_src = next((p for p in npys if "c0" in os.path.basename(p)), npys[0] if npys else None)
    shift = np.zeros(3)
    if align_src is not None:
        sc0 = np.load(align_src)
        sub = sc0[np.random.default_rng(0).choice(len(sc0), size=min(80000, len(sc0)), replace=False)]
        for _ in range(4):                       # translation-only ICP, trimmed
            d, idx = gl_tree.query(sub + shift, k=1, workers=-1)
            keep = d < np.percentile(d, 70)
            shift += np.median(gl[idx[keep]] - (sub[keep] + shift), axis=0)
        print(f"  shared translation alignment (from {os.path.basename(align_src)}): "
              f"{np.round(shift,3)}  |.|={np.linalg.norm(shift):.3f} m")

    rows = []
    for p in npys:
        cid = os.path.basename(p)[:-4]
        sc = np.load(p) + shift
        cells = np.unique(vox(sc), axis=0)
        st = colstats(cells)
        # accuracy / precision: scovox cells near trajectory only (fair vs GLIM region)
        dnt, _ = cKDTree(traj).query(sc, k=1, workers=-1)
        sc_in = sc[dnt <= R_REGION]
        if len(sc_in) == 0: sc_in = sc
        d_s2g, _ = gl_tree.query(sc_in, k=1, workers=-1)
        acc = float(np.median(d_s2g)); prec = float((d_s2g <= TAU).mean())
        # recall: GLIM-region -> scovox
        d_g2s, _ = cKDTree(sc).query(gin, k=1, workers=-1)
        rec = float((d_g2s <= TAU).mean())
        # IoU @ lattice (region: scovox cells within region vs GLIM region cells)
        sc_cells_in = np.unique(vox(sc_in), axis=0)
        sset = set(map(tuple, sc_cells_in))
        tp = len(sset & gset); fp = len(sset - gset); fn = len(gset - sset)
        iou = tp / (tp + fp + fn) if (tp + fp + fn) else 0.0
        f1 = 2 * prec * rec / (prec + rec) if (prec + rec) else 0.0
        rows.append((cid, len(sc), len(cells), st, acc, prec, rec, iou, f1))
        print(f"\n=== {cid} ===")
        print(f"  cells={len(cells):,}  footprint_cols={st['ncols']:,}")
        print(f"  z/col: median={st['z_med']:.0f} mean={st['z_mean']:.1f} p90={st['z_p90']:.0f} "
              f"frac>=5={st['z_ge5']*100:.0f}%  z-extent_med={st['zext_med']:.1f} m")
        print(f"  vs GLIM: accuracy(med)={acc:.3f} m  precision@{TAU}={prec*100:.1f}%  "
              f"recall@{TAU}={rec*100:.1f}%  IoU={iou:.3f}  F1={f1:.3f}")

    # GLIM reference line
    print(f"\n=== GLIM (reference, observed region) ===")
    print(f"  cells={len(g_cells):,}  footprint_cols={gstat['ncols']:,}")
    print(f"  z/col: median={gstat['z_med']:.0f} mean={gstat['z_mean']:.1f} p90={gstat['z_p90']:.0f} "
          f"frac>=5={gstat['z_ge5']*100:.0f}%  z-extent_med={gstat['zext_med']:.1f} m")

    # summary table
    print("\n================ SUMMARY (sorted by F1) ================")
    hdr = f"{'config':<13}{'cells':>10}{'cols':>9}{'z_med':>6}{'z_mean':>7}{'%>=5':>6}{'acc_m':>7}{'prec%':>7}{'rec%':>7}{'IoU':>7}{'F1':>7}"
    print(hdr); print("-" * len(hdr))
    for r in sorted(rows, key=lambda r: -r[8]):
        cid, npt, nc, st, acc, prec, rec, iou, f1 = r
        print(f"{cid:<13}{nc:>10,}{st['ncols']:>9,}{st['z_med']:>6.0f}{st['z_mean']:>7.1f}"
              f"{st['z_ge5']*100:>5.0f}%{acc:>7.2f}{prec*100:>7.1f}{rec*100:>7.1f}{iou:>7.3f}{f1:>7.3f}")
    print(f"{'GLIM(ref)':<13}{len(g_cells):>10,}{gstat['ncols']:>9,}{gstat['z_med']:>6.0f}"
          f"{gstat['z_mean']:>7.1f}{gstat['z_ge5']*100:>5.0f}%{'-':>7}{'-':>7}{'-':>7}{'-':>7}{'-':>7}")

    # ---- figure ----
    cids = [r[0] for r in rows]
    fig, ax = plt.subplots(2, 2, figsize=(15, 10))
    # A: z/col histograms
    for r in rows:
        zc = r[3]['zc']; h, e = np.histogram(np.clip(zc, 0, 40), bins=40, range=(0, 40), density=True)
        ax[0, 0].plot((e[:-1] + e[1:]) / 2, h, label=r[0])
    zcg = gstat['zc']; h, e = np.histogram(np.clip(zcg, 0, 40), bins=40, range=(0, 40), density=True)
    ax[0, 0].plot((e[:-1] + e[1:]) / 2, h, 'k--', lw=2, label='GLIM')
    ax[0, 0].set_title("z-cells per XY column (lower/left = thinner)"); ax[0, 0].set_xlabel("z-cells/col"); ax[0, 0].legend(fontsize=8)
    # B: median z/col bars
    ax[0, 1].bar(cids, [r[3]['z_med'] for r in rows])
    ax[0, 1].axhline(gstat['z_med'], color='k', ls='--', label=f"GLIM={gstat['z_med']:.0f}")
    ax[0, 1].set_title("median z-cells/col"); ax[0, 1].tick_params(axis='x', rotation=45); ax[0, 1].legend()
    # C: precision/recall/IoU bars
    x = np.arange(len(cids)); w = 0.27
    ax[1, 0].bar(x - w, [r[5] for r in rows], w, label='precision@.35')
    ax[1, 0].bar(x,      [r[6] for r in rows], w, label='recall@.35')
    ax[1, 0].bar(x + w,  [r[7] for r in rows], w, label='IoU')
    ax[1, 0].set_xticks(x); ax[1, 0].set_xticklabels(cids, rotation=45); ax[1, 0].set_title("vs GLIM (higher=better)"); ax[1, 0].legend()
    # D: side view x-z for baseline vs best-F1
    best = max(rows, key=lambda r: r[8])[0]
    for cid, col, a in [("c0_baseline", 'r', 0.4), (best, 'g', 0.4)]:
        fp = os.path.join(SWEEP, cid + ".npy")
        if os.path.exists(fp):
            s = (np.load(fp) + shift)
            s = s[np.random.default_rng(1).choice(len(s), size=min(60000, len(s)), replace=False)]
            ax[1, 1].scatter(s[:, 0], s[:, 2], s=1, c=col, alpha=a, label=cid)
    ax[1, 1].set_title("side view (x-z): blob vs thinned"); ax[1, 1].set_xlabel("x [m]"); ax[1, 1].set_ylabel("z [m]"); ax[1, 1].legend()
    plt.tight_layout(); plt.savefig(OUTPNG, dpi=110)
    print(f"\nsaved figure -> {OUTPNG}")


main()
