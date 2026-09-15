#!/usr/bin/env python3
"""Throughput report for a GLIM run, in the same shape as throughput_summary.py.

    glim_summary.py RUN.lidar_poses.csv PLAY_START_WALL PLAY_END_WALL

rate_Hz and gap are defined exactly as in hmr_localisation's
throughput_summary.py -- scans over the SIM-time span, and sim-time gaps between
consecutive accepted scans -- so the columns mean the same thing in both tables.

The extra columns exist because GLIM's odometry queue is unbounded (see the
header of glim_bag_test.sh):
  in_win   poses that arrived while the player was running: the honest
           real-time throughput, and what rate_Hz/gap are computed from
  backlog  poses that arrived after the player stopped, i.e. work GLIM was
           still carrying when the data ended
  lag_end  (wall elapsed - sim elapsed) at the last in-window pose: how far
           behind real time GLIM had fallen
"""
import csv
import statistics
import sys


def quantile(values, q):
    values = sorted(values)
    return values[min(len(values) - 1, int(q * len(values)))]


def main():
    path, t_start, t_end = sys.argv[1], float(sys.argv[2]), float(sys.argv[3])
    rows = list(csv.DictReader(open(path)))
    name = path.rsplit("/", 1)[-1].replace(".lidar_poses.csv", "")

    hdr = "%-26s %6s %8s %8s %8s %8s %8s %8s" % (
        "run", "scans", "rate_Hz", "gap_med", "gap_p95", "in_win", "backlog", "lag_end")
    print(hdr)
    if len(rows) < 2:
        print("%-26s (no poses recorded)" % name)
        return

    win = [r for r in rows if float(r["recv_wall_sec"]) <= t_end]
    backlog = len(rows) - len(win)
    if len(win) < 2:
        print("%-26s (no poses inside the playback window; backlog=%d)" % (name, backlog))
        return

    stamps = [float(r["stamp_sec"]) for r in win]
    span = stamps[-1] - stamps[0]
    gaps = [float(r["accepted_gap_sec"]) for r in win[1:]]
    lag = (float(win[-1]["recv_wall_sec"]) - float(win[0]["recv_wall_sec"])) - span

    print("%-26s %6d %8.2f %8.3f %8.3f %8d %8d %8.2f" % (
        name, len(win), len(win) / span if span > 0 else 0.0,
        statistics.median(gaps), quantile(gaps, 0.95),
        len(win), backlog, lag))


if __name__ == "__main__":
    main()
