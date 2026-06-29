#!/usr/bin/env python3
"""SCovox INPUT-FEED comparison analysis (run in container: numpy+scipy+matplotlib).

Compares the 3 feeds (raw / glim_points / glim_aligned) against this run's GLIM
map. Unlike analyze_sweep.py, each config is aligned to GLIM INDEPENDENTLY
(translation-only trimmed ICP) because the feeds live in different frames:
raw & glim_points in `odom`, glim_aligned already in `map`.

  PRIMARY (intrinsic, alignment-FREE) -- "is it still a 3-D vertical smear?"
    z-cells per occupied XY column (median/mean/p90/frac>=5) and median z-EXTENT[m].
    z-EXTENT is the decisive smear metric: it is invariant to downsampling, so it
    cleanly separates the deskew effect from the ~11x sparser GLIM feeds.
  SECONDARY (vs GLIM, per-config alignment)
    accuracy=median NN scovox->GLIM[m]; precision@tau; recall@tau; IoU @0.2 lattice.

Usage: python3 analyze_feed_cmp.py [dir] [glim.pcd] [traj.csv] [out_png]
"""
import os, sys, glob
import numpy as np
from scipy.spatial import cKDTree
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

DIR   = sys.argv[1] if len(sys.argv) > 1 else "/ws/output/feed_cmp"
GLIM  = sys.argv[2] if len(sys.argv) > 2 else "/ws/output/glim_map_feed.pcd"
TRAJ  = sys.argv[3] if len(sys.argv) > 3 else "/ws/output/path_glim_feed.csv"
OUTPNG= sys.argv[4] if len(sys.argv) > 4 else "/ws/output/feed_cmp/feed_cmp_analysis.png"

RES = 0.20
R_REGION = 10.0
TAU = 0.35
ORDER = ["raw", "glim_points", "glim_aligned"]    # display order


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


def vox(p):
    return np.floor(p / RES + 0.5).astype(np.int64)


def colstats(cells):
    order = np.lexsort((cells[:, 2], cells[:, 1], cells[:, 0]))
    c = cells[order]
    xy = c[:, :2]
    same = np.all(xy[1:] == xy[:-1], axis=1)
    bnd = np.where(~same)[0] + 1
    groups = np.split(c[:, 2], bnd)
    zc = np.array([len(g) for g in groups])
    zext = np.array([(g.max() - g.min() + 1) for g in groups]) * RES
    return dict(ncols=len(zc),
                z_med=float(np.median(zc)), z_mean=float(zc.mean()),
                z_p90=float(np.percentile(zc, 90)), z_ge5=float((zc >= 5).mean()),
                zext_med=float(np.median(zext)), zext_mean=float(zext.mean()), zc=zc)


def align_to_glim(sc, gl_tree, gl):
    """Per-config translation-only trimmed ICP (4 iters)."""
    rng = np.random.default_rng(0)
    sub = sc[rng.choice(len(sc), size=min(80000, len(sc)), replace=False)]
    shift = np.zeros(3)
    for _ in range(6):
        d, idx = gl_tree.query(sub + shift, k=1, workers=-1)
        keep = d < np.percentile(d, 70)
        step = np.median(gl[idx[keep]] - (sub[keep] + shift), axis=0)
        shift += step
        if np.linalg.norm(step) < 1e-3:
            break
    return shift


def main():
    print(f"loading GLIM {GLIM} ...", flush=True)
    gl = load_pcd_xyz(GLIM)
    traj = load_traj(TRAJ)
    print(f"  GLIM pts={len(gl):,}  traj poses={len(traj):,}")
    dtr, _ = cKDTree(traj).query(gl, k=1, workers=-1)
    gin = gl[dtr <= R_REGION]
    print(f"  GLIM observed-region pts={len(gin):,} (<= {R_REGION} m of traj)")
    gl_tree = cKDTree(gl)
    g_cells = np.unique(vox(gin), axis=0)
    gstat = colstats(g_cells)
    gset = set(map(tuple, g_cells))

    npys = sorted(glob.glob(os.path.join(DIR, "*.npy")),
                  key=lambda p: ORDER.index(os.path.basename(p)[:-4])
                  if os.path.basename(p)[:-4] in ORDER else 99)
    rows = []
    for p in npys:
        cid = os.path.basename(p)[:-4]
        raw = np.load(p)
        shift = align_to_glim(raw, gl_tree, gl)     # per-config alignment
        sc = raw + shift
        cells = np.unique(vox(sc), axis=0)
        st = colstats(cells)
        dnt, _ = cKDTree(traj).query(sc, k=1, workers=-1)
        sc_in = sc[dnt <= R_REGION]
        if len(sc_in) == 0: sc_in = sc
        d_s2g, _ = gl_tree.query(sc_in, k=1, workers=-1)
        acc = float(np.median(d_s2g)); prec = float((d_s2g <= TAU).mean())
        d_g2s, _ = cKDTree(sc).query(gin, k=1, workers=-1)
        rec = float((d_g2s <= TAU).mean())
        sc_cells_in = np.unique(vox(sc_in), axis=0)
        sset = set(map(tuple, sc_cells_in))
        tp = len(sset & gset); fp = len(sset - gset); fn = len(gset - sset)
        iou = tp / (tp + fp + fn) if (tp + fp + fn) else 0.0
        f1 = 2 * prec * rec / (prec + rec) if (prec + rec) else 0.0
        rows.append(dict(cid=cid, npts=len(raw), cells=len(cells), st=st,
                         acc=acc, prec=prec, rec=rec, iou=iou, f1=f1,
                         shift=shift, sc=sc))
        print(f"\n=== {cid} ===  (npts={len(raw):,}, align shift={np.round(shift,2)} |.|={np.linalg.norm(shift):.2f}m)")
        print(f"  cells={len(cells):,}  footprint_cols={st['ncols']:,}")
        print(f"  z/col: median={st['z_med']:.0f} mean={st['z_mean']:.1f} p90={st['z_p90']:.0f} "
              f"frac>=5={st['z_ge5']*100:.0f}%  z-extent_med={st['zext_med']:.2f} m (mean={st['zext_mean']:.2f})")
        print(f"  vs GLIM: accuracy(med)={acc:.3f} m  precision@{TAU}={prec*100:.1f}%  "
              f"recall@{TAU}={rec*100:.1f}%  IoU={iou:.3f}  F1={f1:.3f}")

    print(f"\n=== GLIM (reference, observed region) ===")
    print(f"  cells={len(g_cells):,}  footprint_cols={gstat['ncols']:,}")
    print(f"  z/col: median={gstat['z_med']:.0f} mean={gstat['z_mean']:.1f} p90={gstat['z_p90']:.0f} "
          f"frac>=5={gstat['z_ge5']*100:.0f}%  z-extent_med={gstat['zext_med']:.2f} m")

    print("\n================ SUMMARY ================")
    hdr = f"{'feed':<13}{'cells':>10}{'cols':>9}{'z_med':>6}{'z_mean':>7}{'zext_m':>8}{'%>=5':>6}{'acc_m':>7}{'prec%':>7}{'rec%':>7}{'IoU':>7}{'F1':>7}"
    print(hdr); print("-" * len(hdr))
    for r in rows:
        st = r['st']
        print(f"{r['cid']:<13}{r['cells']:>10,}{st['ncols']:>9,}{st['z_med']:>6.0f}{st['z_mean']:>7.1f}"
              f"{st['zext_med']:>8.2f}{st['z_ge5']*100:>5.0f}%{r['acc']:>7.2f}{r['prec']*100:>7.1f}"
              f"{r['rec']*100:>7.1f}{r['iou']:>7.3f}{r['f1']:>7.3f}")
    print(f"{'GLIM(ref)':<13}{len(g_cells):>10,}{gstat['ncols']:>9,}{gstat['z_med']:>6.0f}"
          f"{gstat['z_mean']:>7.1f}{gstat['zext_med']:>8.2f}{gstat['z_ge5']*100:>5.0f}%"
          f"{'-':>7}{'-':>7}{'-':>7}{'-':>7}{'-':>7}")

    # ---- figure ----
    cids = [r['cid'] for r in rows]
    fig, ax = plt.subplots(2, 2, figsize=(15, 10))
    for r in rows:
        zc = r['st']['zc']; h, e = np.histogram(np.clip(zc, 0, 40), bins=40, range=(0, 40), density=True)
        ax[0, 0].plot((e[:-1] + e[1:]) / 2, h, label=r['cid'])
    zcg = gstat['zc']; h, e = np.histogram(np.clip(zcg, 0, 40), bins=40, range=(0, 40), density=True)
    ax[0, 0].plot((e[:-1] + e[1:]) / 2, h, 'k--', lw=2, label='GLIM')
    ax[0, 0].set_title("z-cells per XY column (left=thinner=better)"); ax[0, 0].set_xlabel("z-cells/col"); ax[0, 0].legend(fontsize=9)
    # median z-extent bars (the headline metric)
    ax[0, 1].bar(cids, [r['st']['zext_med'] for r in rows], color=['r', 'orange', 'g'])
    ax[0, 1].axhline(gstat['zext_med'], color='k', ls='--', label=f"GLIM={gstat['zext_med']:.2f} m")
    ax[0, 1].set_title("median z-EXTENT per column [m] (lower=thinner)"); ax[0, 1].tick_params(axis='x', rotation=20); ax[0, 1].legend()
    x = np.arange(len(cids)); w = 0.27
    ax[1, 0].bar(x - w, [r['prec'] for r in rows], w, label='precision@.35')
    ax[1, 0].bar(x,      [r['rec'] for r in rows], w, label='recall@.35')
    ax[1, 0].bar(x + w,  [r['iou'] for r in rows], w, label='IoU')
    ax[1, 0].set_xticks(x); ax[1, 0].set_xticklabels(cids, rotation=20); ax[1, 0].set_title("vs GLIM (higher=better)"); ax[1, 0].legend()
    # side view x-z, all feeds + GLIM
    rng = np.random.default_rng(1)
    cols = {'raw': 'r', 'glim_points': 'orange', 'glim_aligned': 'g'}
    for r in rows:
        s = r['sc']; s = s[rng.choice(len(s), size=min(40000, len(s)), replace=False)]
        ax[1, 1].scatter(s[:, 0], s[:, 2], s=1, c=cols.get(r['cid'], 'b'), alpha=0.3, label=r['cid'])
    g = gin[rng.choice(len(gin), size=min(40000, len(gin)), replace=False)]
    ax[1, 1].scatter(g[:, 0], g[:, 2], s=1, c='k', alpha=0.4, label='GLIM')
    ax[1, 1].set_title("side view (x-z): vertical smear?"); ax[1, 1].set_xlabel("x [m]"); ax[1, 1].set_ylabel("z [m]"); ax[1, 1].legend(markerscale=6)
    plt.tight_layout(); plt.savefig(OUTPNG, dpi=110)
    print(f"\nsaved figure -> {OUTPNG}")


main()
