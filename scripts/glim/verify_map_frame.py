#!/usr/bin/env python3
"""Verify the map-frame fix: for each /scovox_node/pointcloud message, attempt the
exact transform RViz does with Fixed Frame=map -> map <- <cloud frame> at the
cloud's stamp. Reports OK/FAIL and the gap between the cloud stamp and the latest
available map->odom TF (negative = stamp in the past = resolvable; positive =
future = the old bug)."""
import sys
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import PointCloud2
import tf2_ros


class V(Node):
    def __init__(self):
        super().__init__("verify_map_frame")
        self.buf = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buf, self)
        q = QoSProfile(depth=5)
        q.history = HistoryPolicy.KEEP_LAST
        q.reliability = ReliabilityPolicy.RELIABLE
        q.durability = DurabilityPolicy.VOLATILE
        self.create_subscription(PointCloud2, "/scovox_node/pointcloud", self.cb, q)
        self.ok = 0
        self.fail = 0

    def cb(self, msg):
        frame = msg.header.frame_id
        st = rclpy.time.Time.from_msg(msg.header.stamp)
        # latest available map->odom time (for context)
        try:
            latest = self.buf.lookup_transform("map", frame, rclpy.time.Time())
            latest_t = rclpy.time.Time.from_msg(latest.header.stamp)
            gap = (st.nanoseconds - latest_t.nanoseconds) / 1e9
        except Exception:
            gap = float("nan")
        try:
            self.buf.lookup_transform("map", frame, st, timeout=Duration(seconds=0.2))
            self.ok += 1
            print(f"OK   map<-{frame} @ cloud stamp  (stamp-latestTF gap={gap:+.2f}s)", flush=True)
        except Exception as e:
            self.fail += 1
            print(f"FAIL map<-{frame} @ cloud stamp  (gap={gap:+.2f}s): {str(e)[:90]}", flush=True)


def main():
    rclpy.init()
    n = V()
    w = 0.0
    while rclpy.ok() and (n.ok + n.fail) < 6 and w < 40:
        rclpy.spin_once(n, timeout_sec=0.1); w += 0.1
    print(f"\nRESULT: OK={n.ok} FAIL={n.fail} -> "
          f"{'MAP FRAME WORKS' if n.fail == 0 and n.ok > 0 else 'STILL BROKEN' if n.fail else 'NO DATA'}",
          flush=True)
    rclpy.shutdown()


main()
