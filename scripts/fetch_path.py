#!/usr/bin/env python3
"""Grab the latched /path (transient_local) and dump poses to CSV, then exit."""
import csv
import sys
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from nav_msgs.msg import Path

OUT = sys.argv[1] if len(sys.argv) > 1 else "/ws/output/path.csv"
TOPIC = sys.argv[2] if len(sys.argv) > 2 else "/path"   # e.g. /robot1/path for multi-robot


class Fetch(Node):
    def __init__(self):
        super().__init__("path_fetch")
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         history=HistoryPolicy.KEEP_LAST)
        self.got = False
        self.create_subscription(Path, TOPIC, self.cb, qos)

    def cb(self, m):
        with open(OUT, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["stamp", "x", "y", "z", "qx", "qy", "qz", "qw"])
            for ps in m.poses:
                t = ps.header.stamp.sec + ps.header.stamp.nanosec * 1e-9
                p, q = ps.pose.position, ps.pose.orientation
                w.writerow([f"{t:.6f}", p.x, p.y, p.z, q.x, q.y, q.z, q.w])
        self.get_logger().info(f"wrote {len(m.poses)} poses (frame={m.header.frame_id}) to {OUT}")
        self.got = True


def main():
    rclpy.init()
    n = Fetch()
    import time
    t0 = time.time()
    while rclpy.ok() and not n.got and time.time() - t0 < 10:
        rclpy.spin_once(n, timeout_sec=0.2)
    if not n.got:
        n.get_logger().error("no /path received")
    n.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
