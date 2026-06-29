#!/usr/bin/env python3
# Salvage-capture the SCovox occupancy map from a still-alive scovox_node whose
# republish timer has FROZEN because the bag (its /clock source) stopped. This
# node publishes an advancing /clock (from BASE stamp) to unfreeze scovox's
# sim-time republish timer, then saves the first full map it republishes.
#
# Usage: python3 salvage_capture.py <out.npy> <base_sim_stamp>
import sys
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from rosgraph_msgs.msg import Clock
from builtin_interfaces.msg import Time

OUT = sys.argv[1]
BASE = float(sys.argv[2])


class Salvage(Node):
    def __init__(self):
        super().__init__("scovox_salvage")          # NOTE: this node uses system time, not sim
        qos = QoSProfile(depth=1)
        qos.history = HistoryPolicy.KEEP_LAST
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        qos.durability = DurabilityPolicy.VOLATILE
        self.create_subscription(PointCloud2, "/scovox_node/pointcloud", self.cb, qos)
        self.clk = self.create_publisher(Clock, "/clock", 10)
        self.t = 0.0
        self.create_timer(0.05, self.tick)
        self.saved = False

    def tick(self):
        self.t += 0.2                                # advance sim ~4x wall so the 2 s timer fires fast
        now = BASE + self.t
        m = Clock(); m.clock = Time(sec=int(now), nanosec=int((now - int(now)) * 1e9))
        self.clk.publish(m)

    def cb(self, msg):
        if self.saved:
            return
        s = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        arr = np.stack([s["x"], s["y"], s["z"]], axis=-1).astype(np.float64)
        np.save(OUT, arr)
        mn, mx = arr.min(0), arr.max(0)
        print(f"SALVAGED {arr.shape[0]} pts  frame={msg.header.frame_id} -> {OUT}", flush=True)
        print(f"  bbox_min={np.round(mn,2)} bbox_max={np.round(mx,2)} size={np.round(mx-mn,2)}", flush=True)
        self.saved = True


def main():
    rclpy.init()
    n = Salvage()
    waited = 0.0
    while rclpy.ok() and not n.saved and waited < 30.0:
        rclpy.spin_once(n, timeout_sec=0.1)
        waited += 0.1
    if not n.saved:
        print("SALVAGE FAILED: no scovox message within 30 s", flush=True)
    rclpy.shutdown()


main()
