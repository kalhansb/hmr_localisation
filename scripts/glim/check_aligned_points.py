#!/usr/bin/env python3
"""Capture one /glim_ros/aligned_points message to see if it's usable as SCovox
input: frame_id (must be a fixed frame, ideally odom/map), per-frame size (not the
whole map), and bbox. Also report whether a per-frame origin is resolvable."""
import sys
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2

TOPIC = sys.argv[1] if len(sys.argv) > 1 else "/glim_ros/aligned_points"


class Cap(Node):
    def __init__(self):
        super().__init__("check_aligned_points")
        for rel in (ReliabilityPolicy.BEST_EFFORT, ReliabilityPolicy.RELIABLE):
            q = QoSProfile(depth=5)
            q.history = HistoryPolicy.KEEP_LAST
            q.reliability = rel
            q.durability = DurabilityPolicy.VOLATILE
            self.create_subscription(PointCloud2, TOPIC, self.cb, q)
        self.n = 0

    def cb(self, msg):
        self.n += 1
        s = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        a = np.stack([s["x"], s["y"], s["z"]], -1).astype(float)
        mn, mx = a.min(0), a.max(0)
        print(f"#{self.n} topic={TOPIC}")
        print(f"  frame_id={msg.header.frame_id!r}  stamp={msg.header.stamp.sec}.{msg.header.stamp.nanosec:09d}")
        print(f"  width={msg.width} height={msg.height} npts={a.shape[0]:,} fields={[f.name for f in msg.fields]}")
        print(f"  bbox_min={np.round(mn,1)} bbox_max={np.round(mx,1)} size={np.round(mx-mn,1)}")
        print(f"  -> {'PER-FRAME scan (good)' if a.shape[0] < 400000 else 'looks like a big/global cloud'}")


def main():
    rclpy.init()
    n = Cap()
    w = 0.0
    while rclpy.ok() and n.n < 3 and w < 40:
        rclpy.spin_once(n, timeout_sec=0.1); w += 0.1
    if n.n == 0:
        print(f"no message on {TOPIC} within 40s")
    rclpy.shutdown()


main()
