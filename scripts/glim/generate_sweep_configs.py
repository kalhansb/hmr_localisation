#!/usr/bin/env python3
"""Generate the SCovox LiDAR occupancy parameter-sweep configs.

Reads the base config/scovox_lidar_glim.yaml (integration_frame=odom) and emits
one variant per config under config/sweep/. Each variant changes ONLY the
occupancy-integration knobs identified as the levers for the vertical over-fill
artifact (see analysis): the single-hit carve latch, far-return down-weighting,
range cap, evidence ratio, and the emission gate.

Run on the HOST (just text munging):  python3 generate_sweep_configs.py
"""
import os
import re

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
BASE = os.path.join(ROOT, "config", "scovox_lidar_glim.yaml")
OUT  = os.path.join(ROOT, "config", "sweep")

# id -> overrides. carve_skip_occ_threshold is NOT in the base file (node default
# 0.7); we always emit it explicitly so the sweep is self-documenting.
#
# Mechanism being probed (LiDAR path, Beta(1,1) prior):
#   ONE hit -> a_occ=1+w_occ, a_free=1 -> p_occ=(1+w_occ)/(2+w_occ).
#   With w_occ=8 that is 0.90, which exceeds carve_skip_occ_threshold(0.7), so the
#   voxel becomes a permanent "wall": all later carve rays STOP at it and never
#   add a_free. => every once-hit voxel latches occupied forever (the blob).
CONFIGS = {
    # Reproduce the blob (current production values, explicit).
    "c0_baseline": dict(carve_skip_occ_threshold=0.7,  range_decay_length=-1.0, max_range=15.0, w_occ=8.0, w_free=4.0, occupancy_vis_threshold=0.6),
    # Lever 1: break the single-hit latch so full-ray carving can erase spurious
    # hits (0.90<0.95 -> carvable; walls hit >=3x stay >0.95 -> protected).
    "c1_carve":    dict(carve_skip_occ_threshold=0.95, range_decay_length=-1.0, max_range=15.0, w_occ=8.0, w_free=4.0, occupancy_vis_threshold=0.6),
    # Lever 2: down-weight far returns (rw=exp(-r/L)) so far single hits deposit
    # little occupied evidence and do not latch.
    "c2_decay":    dict(carve_skip_occ_threshold=0.7,  range_decay_length=8.0,  max_range=15.0, w_occ=8.0, w_free=4.0, occupancy_vis_threshold=0.6),
    # Lever 3: geometric cut. Vertical spread of one scan = range*tan(halfFoV);
    # OS-128 ~+-22.5deg -> at 15m ~+-6.2m, at 10m ~+-4.1m. Smaller cap, thinner pillars.
    "c3_range":    dict(carve_skip_occ_threshold=0.7,  range_decay_length=-1.0, max_range=10.0, w_occ=8.0, w_free=4.0, occupancy_vis_threshold=0.6),
    # Combo: all levers + 1:1 evidence (w_occ=6,w_free=8 -> carving wins faster)
    # + higher emission gate (only effective once p_occ of spurious voxels drops).
    "c4_combo":    dict(carve_skip_occ_threshold=0.95, range_decay_length=10.0, max_range=12.0, w_occ=6.0, w_free=8.0, occupancy_vis_threshold=0.7),
    # Control: emission gate ALONE. Expected weak -- single hits sit at p_occ=0.90,
    # above 0.80, so raising the gate cannot prune them without also pruning real
    # lightly-evidenced surface.
    "c5_vis":      dict(carve_skip_occ_threshold=0.7,  range_decay_length=-1.0, max_range=15.0, w_occ=8.0, w_free=4.0, occupancy_vis_threshold=0.8),
}

# Params present in the base file -> patch the line in place.
INPLACE = {"range_decay_length", "max_range", "w_occ", "w_free", "occupancy_vis_threshold"}


def patch(text, key, val):
    pat = re.compile(rf"^(\s*){re.escape(key)}:\s*\S.*$", re.M)
    repl = rf"\g<1>{key}: {val}        # [sweep]"
    new, n = pat.subn(repl, text)
    if n != 1:
        raise RuntimeError(f"expected exactly 1 '{key}:' line, found {n}")
    return new


def main():
    os.makedirs(OUT, exist_ok=True)
    base = open(BASE).read()
    for cid, ov in CONFIGS.items():
        t = base
        for k, v in ov.items():
            if k in INPLACE:
                t = patch(t, k, v)
        # carve_skip_occ_threshold is absent in the base: inject after w_free.
        csk = ov["carve_skip_occ_threshold"]
        t = re.sub(r"^(\s*w_free:.*$)",
                   rf"\g<1>\n    carve_skip_occ_threshold: {csk}        # [sweep] carve wall-guard",
                   t, count=1, flags=re.M)
        # Header banner so the file is self-identifying.
        banner = (f"# === SWEEP CONFIG {cid} ===\n"
                  f"# overrides: {ov}\n")
        t = banner + t
        path = os.path.join(OUT, f"{cid}.yaml")
        open(path, "w").write(t)
        print(f"wrote {path}")
    print(f"\n{len(CONFIGS)} configs in {OUT}")


if __name__ == "__main__":
    main()
