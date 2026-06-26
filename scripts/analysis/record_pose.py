#!/usr/bin/env python3
"""Subscribe to the localizer output and log the trajectory to CSV.
Run inside the container with use_sim_time so stamps match the bag clock."""
import csv
import sys
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from diagnostic_msgs.msg import DiagnosticArray

OUT = sys.argv[1] if len(sys.argv) > 1 else "/ws/output/pose_log.csv"


class Recorder(Node):
    def __init__(self):
        super().__init__("pose_recorder")
        self.set_parameters([rclpy.parameter.Parameter("use_sim_time", value=True)])
        self.f = open(OUT, "w", newline="")
        self.w = csv.writer(self.f)
        self.w.writerow(["stamp", "x", "y", "z", "qx", "qy", "qz", "qw"])
        self.n = 0
        self.last_score = None
        qos = QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST)
        # /pcl_pose may be PoseStamped (typical) -- subscribe and handle.
        self.create_subscription(PoseStamped, "/pcl_pose", self.cb_pose, qos)
        self.create_subscription(DiagnosticArray, "/alignment_status",
                                 self.cb_diag, qos)

    def cb_pose(self, m):
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        p, q = m.pose.position, m.pose.orientation
        self.w.writerow([f"{t:.6f}", p.x, p.y, p.z, q.x, q.y, q.z, q.w])
        self.n += 1
        if self.n % 25 == 0:
            self.f.flush()
            score = f" score={self.last_score}" if self.last_score else ""
            self.get_logger().info(
                f"poses={self.n} last=({p.x:.2f},{p.y:.2f},{p.z:.2f}){score}")

    def cb_diag(self, m):
        for st in m.status:
            for kv in st.values:
                if "score" in kv.key.lower() or "fitness" in kv.key.lower():
                    self.last_score = f"{kv.key}={kv.value}"


def main():
    rclpy.init()
    node = Recorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.f.flush()
        node.f.close()
        node.get_logger().info(f"wrote {node.n} poses to {OUT}")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
