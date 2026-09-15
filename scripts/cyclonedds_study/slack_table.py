#!/usr/bin/env python3
"""Regenerate the Slack-pasteable results table (docs/cyclonedds_transport_study.md 3).

Adds the C1 work-normalisation column: hmr_localisation does not process the same
number of scans as GLIM, so raw `cores` is not like-for-like. norm = cores scaled to
the full-rate reference for that bag (GLIM's rate, which tracks the bag at ~10 Hz).
"""
import os, sys, subprocess, re

HERE = os.path.dirname(os.path.abspath(__file__))
out = subprocess.run([sys.executable, os.path.join(HERE, "build_table.py")],
                     capture_output=True, text=True).stdout

rows = []
for ln in out.splitlines():
    m = re.match(r"^(hmr_loc|glim)\s+(\S+ \S+|\S+)\s+(bunker|curtmini)\s+"
                 r"([\d.]+)\s+([\d.]+)\s+([\d.]+-[\d.]+)\s+([\d.]+)\s+"
                 r"([\d.]+)\s+([\d.]+)\s+(\d+)\s+([-+][\d.]+) MB/min, (\S+)", ln)
    if m:
        g = m.groups()
        rows.append(dict(sys=g[0], mode=g[1], bag=g[2], rate=float(g[3]),
                         cores=float(g[4]), rng=g[5], p95=float(g[6]),
                         am=float(g[7]), ap=float(g[8]), rss=int(g[9]),
                         mbm=float(g[10]), grow=g[11]))

# full-rate reference per bag = the GLIM rate on that bag
ref = {b: max(r["rate"] for r in rows if r["bag"] == b and r["sys"] == "glim")
       for b in {r["bag"] for r in rows}}

H = ("system   mode       bag       rate  cores    range    norm   p95  "
     "ATEmed ATEp95   RSS  MB/min grow")
print(H)
print("-" * len(H))
for r in rows:
    norm = r["cores"] * ref[r["bag"]] / r["rate"]
    print("%-8s %-10s %-9s %5.2f  %4.2f  %9s  %4.2f  %4.2f  %5.1f  %5.1f %5d %+7.1f %s"
          % (r["sys"], r["mode"], r["bag"], r["rate"], r["cores"], r["rng"],
             norm, r["p95"], r["am"], r["ap"], r["rss"], r["mbm"],
             "YES" if r["grow"] == "STILL" else "no"))
