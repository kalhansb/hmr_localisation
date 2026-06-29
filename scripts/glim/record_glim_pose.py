#!/usr/bin/env python3
"""Record GLIM's estimated pose (geometry_msgs/PoseStamped) to CSV.

GLIM (librviz_viewer.so) publishes /glim_ros/pose in the map frame (loop-closure
corrected). We dump the same CSV columns the plotters expect
(stamp,x,y,z,qx,qy,qz,qw) so output works directly with scripts/plot_zoom.py.

GLIM gates this publisher on get_subscription_count()>0, so simply subscribing
here makes GLIM start emitting it.

Usage (inside the GLIM container, bag playing on sim clock):
  python3 scripts/glim/record_glim_pose.py /ws/output/path_glim.csv [topic]
"""
import csv
import sys
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped

OUT = sys.argv[1] if len(sys.argv) > 1 else "/ws/output/path_glim.csv"
TOPIC = sys.argv[2] if len(sys.argv) > 2 else "/glim_ros/pose"


class Recorder(Node):
    def __init__(self):
        super().__init__("glim_pose_recorder")
        self.set_parameters([rclpy.parameter.Parameter("use_sim_time", value=True)])
        self.f = open(OUT, "w", newline="")
        self.w = csv.writer(self.f)
        self.w.writerow(["stamp", "x", "y", "z", "qx", "qy", "qz", "qw"])
        self.n = 0
        qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(PoseStamped, TOPIC, self.cb, qos)

    def cb(self, m):
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        p, q = m.pose.position, m.pose.orientation
        self.w.writerow([f"{t:.6f}", p.x, p.y, p.z, q.x, q.y, q.z, q.w])
        self.n += 1
        if self.n % 25 == 0:
            self.f.flush()
            self.get_logger().info(
                f"poses={self.n} last=({p.x:.2f},{p.y:.2f},{p.z:.2f})")


def main():
    rclpy.init()
    node = Recorder()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.f.flush()
        node.f.close()
        node.get_logger().info(f"wrote {node.n} poses to {OUT}")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
