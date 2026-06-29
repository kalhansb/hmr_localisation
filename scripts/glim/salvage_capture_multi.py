#!/usr/bin/env python3
"""Salvage the occupancy maps from N still-alive scovox nodes at once.

Each scovox node's republish timer is frozen because the bag (its /clock source)
stopped. This single node publishes an advancing /clock (from BASE) which
unfreezes ALL of them simultaneously (they share sim time), then saves the first
non-empty full-map republish from each node's /<id>/pointcloud topic.

Usage: python3 salvage_capture_multi.py <base_sim_stamp> <out_dir> <id1> <id2> ...
  -> writes <out_dir>/<id>.npy  (Nx3 float64) for each id.
"""
import os
import sys
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
TIMEOUT = 120.0


class SalvageMulti(Node):
    def __init__(self):
        super().__init__("scovox_salvage_multi")   # system time, not sim
        os.makedirs(OUTDIR, exist_ok=True)
        qos = QoSProfile(depth=1)
        qos.history = HistoryPolicy.KEEP_LAST
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        qos.durability = DurabilityPolicy.VOLATILE
        self.saved = {}
        self.subs = []
        for cid in IDS:
            topic = f"/{cid}/pointcloud"
            # default-arg capture so each lambda binds its own cid
            self.subs.append(self.create_subscription(
                PointCloud2, topic,
                lambda msg, c=cid: self.cb(msg, c), qos))
        self.clk = self.create_publisher(Clock, "/clock", 10)
        self.t = 0.0
        self.create_timer(0.05, self.tick)

    def tick(self):
        self.t += 0.2                       # ~4x wall so the 2 s timers fire fast
        now = BASE + self.t
        m = Clock()
        m.clock = Time(sec=int(now), nanosec=int((now - int(now)) * 1e9))
        self.clk.publish(m)

    def cb(self, msg, cid):
        if cid in self.saved:
            return
        s = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        arr = np.stack([s["x"], s["y"], s["z"]], axis=-1).astype(np.float64)
        if arr.shape[0] == 0:
            return                          # skip empty republishes
        out = os.path.join(OUTDIR, f"{cid}.npy")
        np.save(out, arr)
        mn, mx = arr.min(0), arr.max(0)
        self.saved[cid] = arr.shape[0]
        print(f"SALVAGED {cid}: {arr.shape[0]} pts frame={msg.header.frame_id} "
              f"bbox={np.round(mn,1)}..{np.round(mx,1)} -> {out}", flush=True)


def main():
    rclpy.init()
    n = SalvageMulti()
    waited = 0.0
    while rclpy.ok() and len(n.saved) < len(IDS) and waited < TIMEOUT:
        rclpy.spin_once(n, timeout_sec=0.1)
        waited += 0.1
    missing = [c for c in IDS if c not in n.saved]
    if missing:
        print(f"SALVAGE INCOMPLETE: no map within {TIMEOUT:.0f}s for {missing}", flush=True)
    else:
        print(f"ALL {len(IDS)} maps salvaged.", flush=True)
    rclpy.shutdown()


main()
