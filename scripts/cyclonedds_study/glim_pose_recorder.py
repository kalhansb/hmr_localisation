#!/usr/bin/env python3
"""Record GLIM's ~/pose (PoseStamped) to CSV.

hmr_localisation's benchmark_pose_recorder wants PoseWithCovarianceStamped;
GLIM publishes plain PoseStamped on ~/pose, so this is a direct equivalent.

Records BOTH the header stamp (sim time, for the rate/gap columns that match
throughput_summary.py) and the wall-clock arrival time. The wall clock matters
here in a way it did not for the localizer: GLIM buffers scans internally, so it
can report a healthy sim-time rate while running steadily further behind the
player. lag = wall_elapsed - sim_elapsed makes that visible instead of hiding it.
"""
import csv
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy


class Recorder(Node):
    def __init__(self):
        super().__init__("glim_pose_recorder")
        self.declare_parameter("topic", "/glim_rosnode/pose")
        self.declare_parameter("output_path", "/tmp/glim_poses.csv")
        topic = self.get_parameter("topic").value
        path = self.get_parameter("output_path").value

        self._f = open(path, "w", newline="")
        self._w = csv.writer(self._f)
        self._w.writerow(["stamp_sec", "recv_wall_sec", "accepted_gap_sec",
                          "x", "y", "z", "qx", "qy", "qz", "qw"])
        self._prev = None
        self._t0_wall = None
        self._t0_sim = None
        self._n = 0

        # GLIM's pose publishers are plain create_publisher(..., 10): reliable,
        # volatile, keep_last(10).
        qos = QoSProfile(depth=50, history=HistoryPolicy.KEEP_LAST,
                         reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.VOLATILE)
        self.create_subscription(PoseStamped, topic, self._cb, qos)
        self.get_logger().info(f"recording {topic} -> {path}")

    def _cb(self, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        wall = time.time()
        if self._t0_wall is None:
            self._t0_wall, self._t0_sim = wall, stamp
        gap = (stamp - self._prev) if self._prev is not None else 0.0
        self._prev = stamp
        self._n += 1
        p, o = msg.pose.position, msg.pose.orientation
        self._w.writerow([f"{stamp:.9f}", f"{wall:.6f}", f"{gap:.6f}",
                          p.x, p.y, p.z, o.x, o.y, o.z, o.w])
        if self._n % 100 == 0:
            self._f.flush()

    def destroy_node(self):
        self._f.flush()
        self._f.close()
        super().destroy_node()


def main():
    rclpy.init(args=sys.argv)
    node = Recorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
