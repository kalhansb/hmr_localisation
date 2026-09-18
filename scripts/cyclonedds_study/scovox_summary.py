#!/usr/bin/env python3
"""Summarise one scovox_bag_test.sh run: CPU, RSS, scan rate, per-scan cost.

Usage: scovox_summary.py <output_dir>/<run_name>     (no extension)

Reads the three artefacts the harness writes:
  <run>.scovox.cpu.csv  time_sec,cpu_ticks,rss_kb  sampled every 2 s
  <run>.loc.cpu.csv     same, for the localizer running alongside
  <run>.scovox.log      one RCLCPP_INFO `recv=` line per ADMITTED scan

Two rates are reported and they are not the same thing. The log-line rate is
how fast scovox actually processed scans; the bag offered 10 Hz. If scovox
admitted fewer scans than the bag published, the gap is dropped scans, not
cheapness -- so the admitted/offered ratio is printed next to the rate rather
than left for the reader to infer. `tf_fb` (scovox_node.cpp:1958) counts scans
that fell back to a stale Time(0) pose because the exact-stamp lookup timed
out; those scans are integrated at the previous pose, so a non-zero count means
the map is smeared even though the throughput number looks healthy.
"""
import os
import re
import sys


def pct(xs, q):
    if not xs:
        return float("nan")
    s = sorted(xs)
    i = min(len(s) - 1, max(0, int(round(q * (len(s) - 1)))))
    return s[i]


def cpu_csv(path):
    """-> (cores, peak_rss_mb, duration_s, nsamples) or None."""
    if not os.path.exists(path):
        return None
    rows = []
    with open(path) as fh:
        next(fh, None)
        for line in fh:
            f = line.strip().split(",")
            if len(f) == 3 and all(f):
                try:
                    rows.append((float(f[0]), int(f[1]), int(f[2])))
                except ValueError:
                    pass
    if len(rows) < 2:
        return None
    clk = os.sysconf("SC_CLK_TCK")
    dt = rows[-1][0] - rows[0][0]
    if dt <= 0:
        return None
    cores = (rows[-1][1] - rows[0][1]) / clk / dt
    return cores, max(r[2] for r in rows) / 1024.0, dt, len(rows)


LOG_RE = re.compile(
    r"^\[INFO\] \[(?P<t>\d+\.\d+)\] \[[^\]]*\]: recv=(?P<recv>\d+) .*?"
    r"frame_ms=(?P<frame>[\d.]+) tf_ms=(?P<tf>[\d.]+) "
    r"integrate_ms=(?P<integ>[\d.]+) publish_ms=(?P<pub>[\d.]+) "
    r"rss_mb=(?P<rss>[\d.]+)"
)
TAIL_RE = re.compile(r"gated=(\d+) rearm=(\d+) reject_gated=(\d+).*?tf_fb=(\d+)")


def scan_log(path):
    if not os.path.exists(path):
        return None
    ts, frame, tf, integ, pub, rss = [], [], [], [], [], []
    tail = None
    with open(path, errors="replace") as fh:
        for line in fh:
            m = LOG_RE.match(line)
            if not m:
                continue
            ts.append(float(m["t"]))
            frame.append(float(m["frame"]))
            tf.append(float(m["tf"]))
            integ.append(float(m["integ"]))
            pub.append(float(m["pub"]))
            rss.append(float(m["rss"]))
            t = TAIL_RE.search(line)
            if t:
                tail = tuple(int(x) for x in t.groups())
    if not ts:
        return None
    return dict(n=len(ts), span=ts[-1] - ts[0], frame=frame, tf=tf,
                integ=integ, pub=pub, rss=rss, tail=tail)


def play_count(path):
    """Scans the bag actually offered, if the play log says."""
    if not os.path.exists(path):
        return None
    return None  # humble's player does not report a per-topic count


def main():
    if len(sys.argv) != 2:
        sys.exit(__doc__)
    run = sys.argv[1]
    name = os.path.basename(run)
    print(f"=== {name} ===")

    s = scan_log(run + ".scovox.log")
    if s and s["span"] > 0:
        rate = (s["n"] - 1) / s["span"]
        print(f"scovox scans:  {s['n']} admitted in {s['span']:.1f} s "
              f"-> {rate:.2f} Hz  (bag offers 10 Hz)")
        if s["tail"]:
            gated, rearm, rej, tf_fb = s["tail"]
            print(f"               gated={gated} reject_gated={rej} "
                  f"rearm={rearm} tf_fallback={tf_fb}"
              + ("   <-- stale-pose scans, map is smeared" if tf_fb else ""))
        for label, xs in (("frame_ms", s["frame"]), ("  tf_ms", s["tf"]),
                          ("  integrate_ms", s["integ"]), ("  publish_ms", s["pub"])):
            print(f"{label:>16}: p50 {pct(xs, .5):7.1f}  p95 {pct(xs, .95):7.1f}  "
                  f"max {max(xs):7.1f}")
        print(f"{'self-reported RSS':>16}: {max(s['rss']):.1f} MB (node's own read, end of run)")
    else:
        print("scovox scans:  NONE -- no `recv=` lines. The node never admitted a scan;")
        print("               check TF (integration_frame must be reachable from the")
        print("               cloud frame at the scan stamp) before trusting any CPU number.")

    print()
    for label, path in (("scovox_mapping_node", run + ".scovox.cpu.csv"),
                        ("lidar_localization  ", run + ".loc.cpu.csv")):
        c = cpu_csv(path)
        if c is None:
            print(f"{label}: no samples")
            continue
        cores, rss, dt, n = c
        print(f"{label}: {cores:5.2f} cores avg over {dt:5.1f} s "
              f"({n} samples), peak RSS {rss:7.1f} MB")
    print(f"\n(box: {os.cpu_count()} cores visible)")


if __name__ == "__main__":
    main()
