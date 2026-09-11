#!/usr/bin/env python3
"""Record the localizer's /alignment_status diagnostics to CSV, one row per scan.

    record_alignment_status.py OUT.csv

Columns are the DiagnosticArray key/values the node publishes per accepted scan;
throughput_summary.py turns the file into a rate / alignment-time / gap report.
"""
import csv
import sys

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)

KEYS = ["fitness_score", "alignment_time_sec", "has_converged", "filtered_point_count",
        "correction_translation_m", "correction_yaw_deg", "seed_translation_since_accept_m",
        "seed_yaw_since_accept_deg", "accepted_gap_sec", "imu_prediction_active",
        "registration_seed_source", "registration_method", "consecutive_rejected_updates"]


class Recorder(Node):
    def __init__(self, path):
        super().__init__('alignment_status_recorder')
        self.fh = open(path, 'w', newline='')
        self.writer = csv.writer(self.fh)
        self.writer.writerow(["stamp_sec", "level", "message"] + KEYS)
        qos = QoSProfile(depth=1, history=QoSHistoryPolicy.KEEP_LAST,
                         reliability=QoSReliabilityPolicy.RELIABLE,
                         durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(DiagnosticArray, '/alignment_status', self.callback, qos)
        self.count = 0

    def callback(self, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        for status in msg.status:
            values = {kv.key: kv.value for kv in status.values}
            level = status.level if isinstance(status.level, int) else ord(status.level)
            self.writer.writerow(["%.9f" % stamp, level, status.message]
                                 + [values.get(k, "") for k in KEYS])
        self.count += 1
        if self.count % 100 == 0:
            self.fh.flush()


def main():
    # The run scripts pass --ros-args (use_sim_time); strip those before the arg check.
    args = rclpy.utilities.remove_ros_args(sys.argv)
    if len(args) != 2:
        sys.exit(__doc__)
    rclpy.init()
    node = Recorder(args[1])
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.fh.flush()
        node.fh.close()


if __name__ == '__main__':
    main()
