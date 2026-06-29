#!/usr/bin/env python3
"""Generate scovox RESOLUTION-sweep configs (to find the voxel size that matches the
downsampled deskewed GLIM /glim_ros/points feed without holes / over-coarsening).

All configs are identical to the production config/scovox_lidar_glim.yaml
(input=/glim_ros/points, base_frame=imu, integration_frame=odom, same occupancy)
EXCEPT the voxel `resolution`. Run on HOST: python3 generate_res_configs.py
"""
import os
import re

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
BASE = os.path.join(ROOT, "config", "scovox_lidar_glim.yaml")
OUT = os.path.join(ROOT, "config", "res_sweep")

RES = {"res020": 0.20, "res010": 0.10, "res005": 0.05}


def patch(text, key, val):
    pat = re.compile(rf"^(\s*){re.escape(key)}:\s*\S.*$", re.M)
    new, n = pat.subn(rf"\g<1>{key}: {val}        # [res_sweep]", text)
    if n != 1:
        raise RuntimeError(f"expected 1 '{key}:' line, found {n}")
    return new


def main():
    os.makedirs(OUT, exist_ok=True)
    base = open(BASE).read()
    for cid, r in RES.items():
        t = patch(base, "resolution", r)
        t = f"# === RES-SWEEP {cid}: resolution={r} m ===\n" + t
        open(os.path.join(OUT, f"{cid}.yaml"), "w").write(t)
        print(f"wrote {cid}.yaml  resolution={r}")
    print(f"{len(RES)} configs in {OUT}")


if __name__ == "__main__":
    main()
