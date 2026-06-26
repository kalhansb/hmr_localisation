#!/usr/bin/env python3
"""Lightweight 'play' of a recorded localization result for RViz.
Publishes a voxel-downsampled GT map (once, latched) and animates the recorded
trajectory: /pcl_pose, growing /path, and the map->os_lidar TF. No NDT, no bag.

Usage (in container):
  ros2 run ... no -- just: python3 scripts/replay_result.py [path_csv] [voxel_m] [rate_hz]
"""
import sys, csv
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, TransformStamped
from nav_msgs.msg import Path
from std_msgs.msg import Header
from tf2_ros import TransformBroadcaster

PLY = "/ws/gt_map/gt_map.ply"
CSV = sys.argv[1] if len(sys.argv) > 1 else "/ws/output/path_final.csv"
VOX = float(sys.argv[2]) if len(sys.argv) > 2 else 0.5
RATE = float(sys.argv[3]) if len(sys.argv) > 3 else 20.0


def load_ply(p):
    raw = open(p, "rb").read()
    e = raw.index(b"end_header\n") + len(b"end_header\n")
    return np.frombuffer(raw[e:], dtype=np.float32).reshape(-1, 4)[:, :3]


def voxel_ds(pts, v):
    keys = np.floor(pts / v).astype(np.int64)
    _, idx = np.unique(keys, axis=0, return_index=True)
    return pts[idx]


class Replay(Node):
    def __init__(self):
        super().__init__("replay_result")
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             history=HistoryPolicy.KEEP_LAST)
        self.map_pub = self.create_publisher(PointCloud2, "/initial_map", latched)
        self.path_pub = self.create_publisher(Path, "/path", latched)
        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, "/pcl_pose", latched)
        self.tfb = TransformBroadcaster(self)

        m = voxel_ds(load_ply(PLY), VOX)
        self.get_logger().info(f"map: {len(m)} pts after voxel {VOX} m (lightweight)")
        h = Header(); h.frame_id = "map"
        self.map_msg = point_cloud2.create_cloud_xyz32(h, m)

        rows = list(csv.reader(open(CSV)))[1:]
        self.poses = [[float(c) for c in r[1:8]] for r in rows]  # x y z qx qy qz qw
        self.get_logger().info(f"trajectory: {len(self.poses)} poses, animating at {RATE} Hz")
        self.i = 0
        self.path = Path(); self.path.header.frame_id = "map"

        self.create_timer(1.0, self.pub_map)   # keep map latched & fresh
        self.create_timer(1.0 / RATE, self.tick)
        self.pub_map()

    def pub_map(self):
        self.map_msg.header.stamp = self.get_clock().now().to_msg()
        self.map_pub.publish(self.map_msg)

    def tick(self):
        if self.i >= len(self.poses):           # loop the animation
            self.i = 0
            self.path = Path(); self.path.header.frame_id = "map"
        x, y, z, qx, qy, qz, qw = self.poses[self.i]
        now = self.get_clock().now().to_msg()

        pc = PoseWithCovarianceStamped(); pc.header.stamp = now; pc.header.frame_id = "map"
        pc.pose.pose.position.x, pc.pose.pose.position.y, pc.pose.pose.position.z = x, y, z
        pc.pose.pose.orientation.x, pc.pose.pose.orientation.y = qx, qy
        pc.pose.pose.orientation.z, pc.pose.pose.orientation.w = qz, qw
        self.pose_pub.publish(pc)

        p = PoseStamped(); p.header = pc.header; p.pose = pc.pose.pose
        self.path.poses.append(p); self.path.header.stamp = now
        self.path_pub.publish(self.path)

        t = TransformStamped(); t.header.stamp = now
        t.header.frame_id = "map"; t.child_frame_id = "os_lidar"
        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = x, y, z
        t.transform.rotation.x, t.transform.rotation.y = qx, qy
        t.transform.rotation.z, t.transform.rotation.w = qz, qw
        self.tfb.sendTransform(t)
        self.i += 1


def main():
    rclpy.init()
    rclpy.spin(Replay())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
