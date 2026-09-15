#!/usr/bin/env python3
"""Peak RSS, growth rate, and whether memory was STILL growing when sampling stopped.

    mem_growth.py LABEL:run.cpu.csv [LABEL:run.cpu.csv ...]

Peak RSS is taken over the same busiest-contiguous-120 s window cpu_window.py
scores, so the two columns describe the same slice of the run.

WHY A SLOPE AND A VERDICT, NOT JUST A PEAK
A peak over 120 s cannot distinguish "allocated 280 MB and held it" from
"passed through 280 MB on the way up". docs/glim_comparison.md turns on exactly
that difference: odometry-only is flat, while the full stack accumulates a
global map and keeps climbing for as long as the mission runs. A 120 s peak
understates the full stack by an unknown amount and the number alone does not
say so.

  rate   -- least-squares slope over the scored window, MB/min.
  tail   -- slope over the final 40 s of SAMPLING (for GLIM that includes the
            20 s post-playback drain, so a still-rising tail there also catches
            a run that is chewing through backlog rather than mapping).
  still  -- yes if the tail is rising faster than 1.0 MB/min AND the last sample
            sits within 2% of the run's maximum. Both conditions matter: the
            first excludes noise, the second excludes a run that peaked early
            and is now flat or falling back.
"""
import csv
import os
import sys

import numpy as np

CLK = os.sysconf("SC_CLK_TCK")
WIN = 120.0
TAIL = 40.0
RATE_EPS = 1.0     # MB/min
NEAR_MAX = 0.98


def slope_mb_per_min(t, rss_mb):
    if len(t) < 3 or t[-1] - t[0] < 5.0:
        return 0.0
    return float(np.polyfit(t - t[0], rss_mb, 1)[0] * 60.0)


def report(path):
    t, c, r = [], [], []
    for row in csv.DictReader(open(path)):
        t.append(float(row["time_sec"]))
        c.append(float(row["cpu_ticks"]))
        r.append(float(row["rss_kb"]) / 1024.0)
    t, c, r = np.array(t), np.array(c), np.array(r)
    if len(t) < 5:
        return None

    # same busiest-120 s window cpu_window.py picks, by CPU ticks consumed
    best = None
    for i in range(len(t)):
        j = np.searchsorted(t, t[i] + WIN)
        if j >= len(t):
            break
        used = (c[j] - c[i]) / CLK / (t[j] - t[i])
        if best is None or used > best[0]:
            best = (used, i, j)
    i, j = (best[1], best[2]) if best else (0, len(t) - 1)

    peak = float(r[i:j + 1].max())
    rate = slope_mb_per_min(t[i:j + 1], r[i:j + 1])
    tail_m = t >= (t[-1] - TAIL)
    tail = slope_mb_per_min(t[tail_m], r[tail_m])
    still = tail > RATE_EPS and r[-1] >= NEAR_MAX * r.max()
    return dict(peak=peak, rate=rate, tail=tail, still=still,
                final=float(r[-1]), max=float(r.max()))


print("%-26s %8s %10s %10s %7s" % ("run", "rss_MB", "rate_MB/m", "tail_MB/m", "still"))
for a in sys.argv[1:]:
    label, path = a.split(":", 1)
    d = report(path)
    if d is None:
        print("%-26s %8s" % (label, "n/a"))
    else:
        print("%-26s %8.0f %10.1f %10.1f %7s" % (
            label, d["peak"], d["rate"], d["tail"], "yes" if d["still"] else "no"))
