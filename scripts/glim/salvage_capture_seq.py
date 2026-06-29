#!/usr/bin/env python3
"""Sequentially salvage occupancy maps from N still-alive scovox nodes.

Unlike salvage_capture_multi (which subscribes to all topics at once and lets
all nodes republish simultaneously -> contention starves the big maps), this
subscribes to ONE node at a time. A node only publishes when it has >=1
subscriber, so exactly one big map transfers at a time -> no contention. The
/clock is advanced MONOTONICALLY across the whole sequence (going backward would
re-freeze the sim-time republish timers), so multiple captures work in one run.

Usage: python3 salvage_capture_seq.py <base_sim_stamp> <out_dir> <id1> <id2> ...
"""
import os, sys
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from rosgraph_msgs.msg import Clock
from builtin_interfaces.msg import Time

BASE = float(sys.argv[1])
OUTDIR = sys.argv[2]
IDS = sys.argv[3:]
PER_TIMEOUT = float(os.environ.get("SALVAGE_TIMEOUT", "90"))


class Seq(Node):
    def __init__(self):
        super().__init__("scovox_salvage_seq")
        os.makedirs(OUTDIR, exist_ok=True)
        self.qos = QoSProfile(depth=1)
        self.qos.history = HistoryPolicy.KEEP_LAST
        # RELIABLE subscriber (SALVAGE_RELIABLE=1) reliably reassembles large
        # multi-MB clouds from scovox's reliable KeepLast(1) publisher; BEST_EFFORT
        # (default) is lighter but can drop fragments of big maps.
        self.qos.reliability = (ReliabilityPolicy.RELIABLE
                                if os.environ.get("SALVAGE_RELIABLE") == "1"
                                else ReliabilityPolicy.BEST_EFFORT)
        self.qos.durability = DurabilityPolicy.VOLATILE
        self.clk = self.create_publisher(Clock, "/clock", 10)
        self.t = 0.0
        self.create_timer(0.05, self.tick)   # monotonic clock advance for the whole run
        self.cur = None
        self.sub = None
        self.got = False

    def tick(self):
        self.t += 0.2
        now = BASE + self.t
        m = Clock(); m.clock = Time(sec=int(now), nanosec=int((now - int(now)) * 1e9))
        self.clk.publish(m)

    def cb(self, msg):
        if self.got:
            return
        s = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        arr = np.stack([s["x"], s["y"], s["z"]], axis=-1).astype(np.float64)
        if arr.shape[0] == 0:
            return
        out = os.path.join(OUTDIR, f"{self.cur}.npy")
        np.save(out, arr)
        mn, mx = arr.min(0), arr.max(0)
        print(f"SALVAGED {self.cur}: {arr.shape[0]} pts frame={msg.header.frame_id} "
              f"bbox={np.round(mn,1)}..{np.round(mx,1)} -> {out}", flush=True)
        self.got = True


def main():
    rclpy.init()
    n = Seq()
    for cid in IDS:
        n.cur = cid; n.got = False
        n.sub = n.create_subscription(PointCloud2, f"/{cid}/pointcloud", n.cb, n.qos)
        waited = 0.0
        while rclpy.ok() and not n.got and waited < PER_TIMEOUT:
            rclpy.spin_once(n, timeout_sec=0.1)
            waited += 0.1
        if not n.got:
            print(f"FAILED {cid}: no map within {PER_TIMEOUT:.0f}s", flush=True)
        n.destroy_subscription(n.sub); n.sub = None
    rclpy.shutdown()


main()
