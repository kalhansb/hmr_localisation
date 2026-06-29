#!/usr/bin/env python3
"""Generate the 3 SCovox INPUT-FEED comparison configs.

Holds the occupancy integration knobs FIXED at the baseline (w_occ=8, w_free=4,
carve_band=-1, max_range=15, min_range=1, carve_skip default 0.7, vis 0.6) and
varies ONLY the input cloud + its frames, to isolate the effect of GLIM's
per-point deskew (and loop-closure-optimized pose) vs raw scans:

  raw           /ouster/points                       odom / os_lidar   (today's baseline)
  glim_points   /glim_ros/points                     odom / imu        (deskew only)
  glim_aligned  /glim_ros/aligned_points_corrected   map  / imu        (deskew + optimized pose)

All three integrate the SAME GLIM run's TF tree (map->odom->imu->os_lidar), so
they are directly comparable. Range gate is frame-agnostic in scovox_node
(r2 = (Hp-O).squaredNorm(), both in the integration frame), so max_range=15 caps
true sensor range identically for the map-frame feed.

Run on HOST (text munging only):  python3 generate_feed_configs.py
"""
import os
import re

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
BASE = os.path.join(ROOT, "config", "scovox_lidar_glim.yaml")
OUT  = os.path.join(ROOT, "config", "feed_cmp")

# id -> (input_topic, integration_frame, base_frame, map_frame)
CONFIGS = {
    "raw":          ("/ouster/points",                     "odom", "os_lidar", "odom"),
    "glim_points":  ("/glim_ros/points",                   "odom", "imu",      "odom"),
    "glim_aligned": ("/glim_ros/aligned_points_corrected", "map",  "imu",      "map"),
}


def patch(text, key, val, quote=False):
    v = f'"{val}"' if quote else f"{val}"
    pat = re.compile(rf"^(\s*){re.escape(key)}:\s*\S.*$", re.M)
    repl = rf"\g<1>{key}: {v}        # [feed_cmp]"
    new, n = pat.subn(repl, text)
    if n != 1:
        raise RuntimeError(f"expected exactly 1 '{key}:' line for {key}, found {n}")
    return new


def main():
    os.makedirs(OUT, exist_ok=True)
    base = open(BASE).read()
    for cid, (topic, intf, basef, mapf) in CONFIGS.items():
        t = base
        t = patch(t, "input_pointcloud_topic", topic, quote=True)
        t = patch(t, "integration_frame", intf, quote=True)
        t = patch(t, "base_frame", basef, quote=True)
        t = patch(t, "map_frame", mapf, quote=True)
        banner = (f"# === FEED-CMP CONFIG {cid} ===\n"
                  f"# input={topic}  integration_frame={intf}  base_frame={basef}\n"
                  f"# occupancy knobs held at baseline (deskew/pose isolation test)\n")
        t = banner + t
        path = os.path.join(OUT, f"{cid}.yaml")
        open(path, "w").write(t)
        print(f"wrote {path}  (input={topic}, int={intf}, base={basef})")
    print(f"\n{len(CONFIGS)} configs in {OUT}")


if __name__ == "__main__":
    main()
