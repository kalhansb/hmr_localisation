#!/usr/bin/env python3
"""Record a TF edge (default map -> base_link) at a fixed rate to CSV, for the
EKF-smoothing A/B test (scripts/test_ekf_smoothing.sh).

Samples the *composed* transform at a uniform rate, so a layer that only updates at
the NDT scan rate shows up as a held "staircase" between updates, while a smooth
high-rate odom layer (the EKF) shows up as continuous motion.

Columns: t_sample, t_tf, x, y, z, qx, qy, qz, qw
  t_sample = node clock (sim time) when sampled   t_tf = the transform's own stamp

Usage (in container, with the bag's /clock):
  python3 scripts/record_tf_chain.py <out.csv> [parent] [child] [rate_hz] --ros-args -p use_sim_time:=true
"""
import csv
import os
import sys

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import (Buffer, TransformListener, LookupException,
                     ConnectivityException, ExtrapolationException)

OUT = sys.argv[1] if len(sys.argv) > 1 and not sys.argv[1].startswith("--") else "/ws/output/tf_chain.csv"
PARENT = sys.argv[2] if len(sys.argv) > 2 and not sys.argv[2].startswith("--") else "map"
CHILD = sys.argv[3] if len(sys.argv) > 3 and not sys.argv[3].startswith("--") else "base_link"
RATE = float(sys.argv[4]) if len(sys.argv) > 4 and not sys.argv[4].startswith("--") else 50.0


class Rec(Node):
    def __init__(self):
        super().__init__(f"tf_rec_{PARENT}_{CHILD}".replace("-", "_"))
        self.buf = Buffer()
        self.listener = TransformListener(self.buf, self)
        os.makedirs(os.path.dirname(OUT) or ".", exist_ok=True)
        self.f = open(OUT, "w", newline="")
        self.w = csv.writer(self.f)
        self.w.writerow(["t_sample", "t_tf", "x", "y", "z", "qx", "qy", "qz", "qw"])
        self.n = 0
        self.create_timer(1.0 / RATE, self.tick)

    def tick(self):
        try:
            tf = self.buf.lookup_transform(PARENT, CHILD, Time())
        except (LookupException, ConnectivityException, ExtrapolationException):
            return
        ts = self.get_clock().now().nanoseconds * 1e-9
        ttf = tf.header.stamp.sec + tf.header.stamp.nanosec * 1e-9
        tr = tf.transform.translation
        q = tf.transform.rotation
        self.w.writerow([f"{ts:.6f}", f"{ttf:.6f}", tr.x, tr.y, tr.z, q.x, q.y, q.z, q.w])
        self.n += 1

    def destroy_node(self):
        try:
            self.f.flush()
            self.f.close()
        except Exception:
            pass
        super().destroy_node()


def main():
    rclpy.init()
    n = Rec()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.get_logger().info(f"recorded {n.n} samples -> {OUT}")
        n.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
