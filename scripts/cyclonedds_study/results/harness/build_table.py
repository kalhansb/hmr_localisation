#!/usr/bin/env python3
"""Aggregate the 30-run Cyclone matrix into the requested table.

Per row (n=3): cores/p95/RSS from cpu_window (busiest contiguous 120 s),
memory growth from mem_growth, rate from the harness summary line, and ATE from
glim_traj_compare -- both systems through the SAME 6-DOF scorer, per D2.
"""
import re
import subprocess
import sys

import numpy as np

sys.path.insert(0, "/tmp/claude-1000/-home-jetsondevkit/0e7ee909-3f2f-4b65-85f4-bec3127c6fd2/scratchpad/harness")
import cpu_window
import mem_growth

SG = "/tmp/claude-1000/-home-jetsondevkit/0e7ee909-3f2f-4b65-85f4-bec3127c6fd2/scratchpad"
G = "/home/jetsondevkit/glim-output"
O = "/home/jetsondevkit/jetbot-slam/hmr_localisation/output/jetson_test"
H = f"{SG}/harness"
REF = {"bunker": f"{O}/bunk_ch1.poses.csv", "curtmini": f"{O}/curt_ch1.poses.csv"}

ROWS = [
    ("hmr_loc", "stride 4",  "hmr",  "s4"),
    ("hmr_loc", "stride 8",  "hmr",  "s8"),
    ("glim",    "odom cpu",  "glim", "o_combo"),
    ("glim",    "odom gpu",  "glim", "o_combogpu"),
    ("glim",    "full+loop", "glim", "o_combofull"),
]
BAGS = [("bunker", "bunk"), ("curtmini", "curt")]

rates = {}
for line in open(f"{SG}/table_results.txt"):
    m = re.match(r"^(cyc_\w+_r\d)\s+(\d+)\s+([\d.]+)\s", line)
    if m:
        rates[m.group(1)] = float(m.group(3))


def ate(bag, sys_kind, name):
    """Both systems through glim_traj_compare (6-DOF Umeyama). Returns (med_cm, p95_cm)."""
    if sys_kind == "glim":
        test = f"{G}/{name}.lidar_poses.csv"
    else:
        test = f"{SG}/{name}.lidar_poses.csv"
        subprocess.run([f"{H}/hmr_to_lidar_poses.py", f"{O}/{name}.poses.csv", bag, test],
                       check=True, capture_output=True)
    out = subprocess.run([f"{H}/glim_traj_compare.py", REF[bag], test, bag],
                         check=True, capture_output=True, text=True).stdout.strip().split("\n")[-1]
    f = out.split()
    if len(f) < 4 or not re.match(r"^[\d.]+$", f[2]):
        return None, None
    return float(f[2]) * 100.0, float(f[3]) * 100.0


print(f"{'system':8} {'mode':10} {'bag':9} {'rate Hz':>7} {'cores':>6} {'range':>11} "
      f"{'p95':>5} {'ATE med':>8} {'ATE p95':>8} {'RSS MB':>7}  memory")
for syst, mode, kind, tag in ROWS:
    for bag, p in BAGS:
        cs, ps, rs, ms, ts, ams, aps, sg = [], [], [], [], [], [], [], []
        for i in (1, 2, 3):
            name = f"cyc_{kind}_{p}_{tag}_r{i}"
            cpu = f"{G}/{name}.cpu.csv" if kind == "glim" else f"{O}/{name}.cpu.csv"
            w = cpu_window.window(cpu)
            g = mem_growth.report(cpu)
            cs.append(w["cores"]); ps.append(w["p95"]); rs.append(w["rss"])
            ms.append(g["tail"]); ts.append(g["still"])
            a, b = ate(bag, kind, name)
            if a is not None:
                ams.append(a); aps.append(b)
        rate = np.mean([rates[f"cyc_{kind}_{p}_{tag}_r{i}"] for i in (1, 2, 3)])
        grow = np.mean(ms)
        still = sum(ts) >= 2
        mem = f"{grow:+.1f} MB/min, {'STILL GROWING' if still else 'flat'}"
        print(f"{syst:8} {mode:10} {bag:9} {rate:7.2f} {np.mean(cs):6.2f} "
              f"{min(cs):.2f}-{max(cs):.2f}".ljust(54)
              + f"{np.mean(ps):5.2f} {np.mean(ams):8.1f} {np.mean(aps):8.1f} "
                f"{max(rs):7.0f}  {mem}")
