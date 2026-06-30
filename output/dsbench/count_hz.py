#!/usr/bin/env python3
"""Count /ouster/points arrivals (best_effort) over a fixed window."""
import sys, time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import PointCloud2

DUR = float(sys.argv[1]) if len(sys.argv) > 1 else 25.0


class C(Node):
    def __init__(self):
        super().__init__("ouster_counter")
        q = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                       durability=DurabilityPolicy.VOLATILE, history=HistoryPolicy.KEEP_LAST)
        self.n = 0
        self.t0 = None
        self.create_subscription(PointCloud2, "/ouster/points", self.cb, q)

    def cb(self, m):
        if self.t0 is None:
            self.t0 = time.time()
        self.n += 1


rclpy.init()
c = C()
t_end = time.time() + DUR
while rclpy.ok() and time.time() < t_end:
    rclpy.spin_once(c, timeout_sec=0.1)
span = (time.time() - c.t0) if c.t0 else DUR
print(f"received {c.n} /ouster/points in {span:.1f}s -> {c.n/span:.2f} Hz")
c.destroy_node(); rclpy.shutdown()
