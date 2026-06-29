#!/usr/bin/env python3
"""Save GLIM's global map (/glim_ros/map PointCloud2) to a binary XYZ PCD.

GLIM has no headless map-save service -- the "Save Map" action lives in the
Iridescence GUI / OfflineViewer. But librviz_viewer.so publishes the optimized
global map on /glim_ros/map as a LATCHED (TRANSIENT_LOCAL + RELIABLE) PointCloud2,
recomputed every ~10 s while it has a subscriber. So we subscribe (which makes
GLIM compute+publish it), keep the latest message, and write it to PCD on
shutdown (Ctrl-C / SIGINT). Output matches scripts/ply_to_pcd.py's PCD layout so
it drops into scripts/plot_zoom.py and scripts/scovox/compare_maps.py.

Usage (inside the GLIM container):
  python3 scripts/glim/save_glim_map.py /ws/output/glim_map.pcd [topic]
"""
import sys
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

OUT = sys.argv[1] if len(sys.argv) > 1 else "/ws/output/glim_map.pcd"
TOPIC = sys.argv[2] if len(sys.argv) > 2 else "/glim_ros/map"


def write_pcd_binary_xyz(path, xyz):
    n = xyz.shape[0]
    header = (
        "# .PCD v0.7 - Point Cloud Data file format\n"
        "VERSION 0.7\n"
        "FIELDS x y z\n"
        "SIZE 4 4 4\n"
        "TYPE F F F\n"
        "COUNT 1 1 1\n"
        f"WIDTH {n}\n"
        "HEIGHT 1\n"
        "VIEWPOINT 0 0 0 1 0 0 0\n"
        f"POINTS {n}\n"
        "DATA binary\n"
    )
    with open(path, "wb") as f:
        f.write(header.encode("ascii"))
        f.write(np.ascontiguousarray(xyz, dtype="<f4").tobytes())


class MapSaver(Node):
    def __init__(self):
        super().__init__("glim_map_saver")
        self.set_parameters([rclpy.parameter.Parameter("use_sim_time", value=True)])
        self.latest = None
        # Match GLIM's latched global map publisher: transient-local + reliable.
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(PointCloud2, TOPIC, self.cb, qos)
        self.get_logger().info(f"subscribed {TOPIC}; will save latest to {OUT} on Ctrl-C")

    def cb(self, m):
        self.latest = m
        self.get_logger().info(f"got global map: {m.width * m.height} points")

    def save(self):
        if self.latest is None:
            self.get_logger().warn(f"no {TOPIC} received -- nothing to save")
            return
        pts = point_cloud2.read_points_numpy(
            self.latest, field_names=("x", "y", "z"), skip_nans=True)
        xyz = np.asarray(pts, dtype="<f4").reshape(-1, 3)
        finite = np.isfinite(xyz).all(axis=1)
        xyz = np.ascontiguousarray(xyz[finite])
        write_pcd_binary_xyz(OUT, xyz)
        self.get_logger().info(f"wrote {xyz.shape[0]} points -> {OUT}")


def main():
    rclpy.init()
    node = MapSaver()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.save()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
