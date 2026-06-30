#!/usr/bin/env python3
"""Uniform /proc sampler: VmRSS, VmHWM, utime+stime ticks, num_threads for a PID.
Usage: sample_proc.py <pid> <interval_s> <out.csv>"""
import sys, time, os

pid = int(sys.argv[1]); interval = float(sys.argv[2]); out = sys.argv[3]
CLK = os.sysconf("SC_CLK_TCK")


def read():
    with open(f"/proc/{pid}/status") as f:
        st = f.read()
    rss = hwm = thr = 0
    for ln in st.splitlines():
        if ln.startswith("VmRSS:"):
            rss = int(ln.split()[1])          # kB
        elif ln.startswith("VmHWM:"):
            hwm = int(ln.split()[1])          # kB
        elif ln.startswith("Threads:"):
            thr = int(ln.split()[1])
    with open(f"/proc/{pid}/stat") as f:
        p = f.read().rsplit(")", 1)[1].split()
    utime, stime = int(p[11]), int(p[12])     # fields 14,15 -> after ') ' index 11,12
    return rss, hwm, thr, utime + stime


with open(out, "w") as f:
    f.write("t_wall,rss_kb,hwm_kb,threads,cpu_ticks,clk_tck\n")
    while True:
        try:
            rss, hwm, thr, ticks = read()
        except (FileNotFoundError, ProcessLookupError, IndexError):
            break
        f.write(f"{time.time():.3f},{rss},{hwm},{thr},{ticks},{CLK}\n")
        f.flush()
        time.sleep(interval)
