#!/usr/bin/env python3
"""CPU over the busiest contiguous 120 s -- replaces cpu_busy.py, which was biased.

WHY cpu_busy.py WAS WRONG
It averaged samples above 20% of the run's 90th percentile. On these runs that
threshold lands at ~0.14 cores. hmr_localisation's idle tail is 0.00 cores and
was cleanly excluded; GLIM's idle tail is 0.15 cores -- the node keeps
publishing/servicing DDS after playback -- so it sat JUST above the threshold and
was counted as work. That dragged GLIM's average down by up to 9% and, worse,
made GLIM's run-to-run spread look real: whether the threshold landed above or
below 0.15 flipped inclusion of ~8 samples, which is exactly the observed
anti-correlation between busy_cores and busy_s.

THIS METRIC
Both systems played exactly 120 s of bag at rate 1.0, so score the busiest
contiguous 120 s of each run: the window that consumed the most CPU ticks. No
threshold, no assumption about what idle looks like, and it answers the actual
question -- what did it cost to process 120 s of data.
"""
import csv
import os
import sys

import numpy as np

CLK = os.sysconf("SC_CLK_TCK")
WIN = 120.0


def window(path):
    t, c, r = [], [], []
    for row in csv.DictReader(open(path)):
        t.append(float(row["time_sec"]))
        c.append(float(row["cpu_ticks"]))
        r.append(float(row["rss_kb"]))
    t, c, r = np.array(t), np.array(c), np.array(r)
    if len(t) < 5 or t[-1] - t[0] < WIN * 0.5:
        return None
    best = None
    for i in range(len(t)):
        j = np.searchsorted(t, t[i] + WIN)
        if j >= len(t):
            break
        used = (c[j] - c[i]) / CLK / (t[j] - t[i])
        if best is None or used > best[0]:
            best = (used, i, j)
    if best is None:                     # run shorter than the window
        used = (c[-1] - c[0]) / CLK / (t[-1] - t[0])
        best = (used, 0, len(t) - 1)
    used, i, j = best
    dt = np.diff(t[i:j + 1])
    inst = np.diff(c[i:j + 1]) / CLK / np.maximum(dt, 1e-9)
    return dict(cores=used, p95=float(np.quantile(inst, 0.95)),
                span=float(t[j] - t[i]), rss=float(r[i:j + 1].max() / 1024.0))


def main(argv):
    print("%-24s %7s %7s %8s %8s" % ("run", "cores", "p95", "span_s", "rss_MB"))
    for a in argv:
        label, path = a.split(":", 1)
        d = window(path)
        print("%-24s %7s %7s %8s %8s" % (label, "n/a", "", "", "") if d is None else
              "%-24s %7.2f %7.2f %8.0f %8.0f" % (label, d["cores"], d["p95"],
                                                 d["span"], d["rss"]))


if __name__ == "__main__":
    main(sys.argv[1:])
