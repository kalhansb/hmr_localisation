#!/usr/bin/env python3
"""Throughput report from one or more record_alignment_status.py CSVs.

    throughput_summary.py RUN.status.csv [...]

The sensor publishes at 10 Hz, so "rate" is what fraction of scans the localizer kept
up with; "gap" is the sensor time between consecutive accepted scans (0.1 s = every
scan, 0.2 s = every other). Alignment time excludes preprocessing and the fitness score.
"""
import csv
import statistics
import sys


def quantile(values, q):
    values = sorted(values)
    return values[min(len(values) - 1, int(q * len(values)))]


def floats(rows, key):
    out = []
    for row in rows:
        try:
            out.append(float(row[key]))
        except (KeyError, ValueError):
            pass
    return out


def main():
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    print("%-28s %6s %8s %10s %10s %8s %8s %8s" % (
        "run", "scans", "rate_Hz", "align_med", "align_p95", "gap_med", "gap_p95", "fit_med"))
    for path in sys.argv[1:]:
        rows = list(csv.DictReader(open(path)))
        # Drop leading rows that are not from this run: the recorder can receive the
        # previous run's latched (transient-local) status -- possibly from another bag
        # entirely, so earlier or later -- before this run's first scan, which would
        # stretch the span and zero the rate. Consecutive real rows are ~0.1 s apart.
        while len(rows) > 1 and \
                abs(float(rows[0]["stamp_sec"]) - float(rows[1]["stamp_sec"])) > 60.0:
            rows.pop(0)
        if len(rows) < 2:
            print("%-28s (no scans recorded)" % path)
            continue
        # ...and a zero stamp published before sim time arrives.
        stamps = [s for s in floats(rows, "stamp_sec") if s > 0.0]
        span = (stamps[-1] - stamps[0]) if len(stamps) >= 2 else 0.0
        align = floats(rows, "alignment_time_sec")
        gap = floats(rows, "accepted_gap_sec")
        fit = floats(rows, "fitness_score")
        name = path.rsplit("/", 1)[-1].replace(".status.csv", "")
        print("%-28s %6d %8.2f %10.4f %10.4f %8.3f %8.3f %8.4f" % (
            name, len(rows), len(rows) / span if span > 0 else 0.0,
            statistics.median(align), quantile(align, 0.95),
            statistics.median(gap) if gap else float("nan"),
            quantile(gap, 0.95) if gap else float("nan"),
            statistics.median(fit)))


if __name__ == '__main__':
    main()
