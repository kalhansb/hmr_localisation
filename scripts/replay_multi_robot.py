#!/usr/bin/env python3
"""Lightweight RViz replay of a recorded MULTI-robot result.

Publishes a voxel-downsampled GT map (once, latched) and animates two recorded
trajectories together in the shared `map` frame:
  /robot1/path, /robot1/pcl_pose, TF map->robot1/os_lidar   (green)
  /robot2/path, /robot2/pcl_pose, TF map->robot2/os_lidar   (orange)
  /multi_robot/markers : link line + live range label between the two

No NDT, no bag -- just the two CSVs from scripts/run_multi_robot.sh. Drives
config/multi_robot.rviz.

Usage (in container):
  python3 scripts/replay_multi_robot.py [csv1] [csv2] [voxel_m] [rate_hz]
"""
import csv
import sys

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from builtin_interfaces.msg import Duration as DurationMsg
from geometry_msgs.msg import (Point, PoseStamped, PoseWithCovarianceStamped,
                               TransformStamped)
from nav_msgs.msg import Path
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray

PLY = "/ws/gt_map/gt_map.ply"
CSV1 = sys.argv[1] if len(sys.argv) > 1 else "/ws/output/path_robot1.csv"
CSV2 = sys.argv[2] if len(sys.argv) > 2 else "/ws/output/path_robot2.csv"
VOX = float(sys.argv[3]) if len(sys.argv) > 3 else 0.5
RATE = float(sys.argv[4]) if len(sys.argv) > 4 else 20.0

ROBOTS = [
    ("robot1", CSV1, (0.0, 1.0, 0.0)),
    ("robot2", CSV2, (1.0, 0.66, 0.0)),
]


def load_ply(p):
    raw = open(p, "rb").read()
    e = raw.index(b"end_header\n") + len(b"end_header\n")
    return np.frombuffer(raw[e:], dtype=np.float32).reshape(-1, 4)[:, :3]


def voxel_ds(pts, v):
    keys = np.floor(pts / v).astype(np.int64)
    _, idx = np.unique(keys, axis=0, return_index=True)
    return pts[idx]


def load_poses(path):
    rows = list(csv.reader(open(path)))[1:]
    return [[float(c) for c in r[1:8]] for r in rows]  # x y z qx qy qz qw


class Replay(Node):
    def __init__(self):
        super().__init__("replay_multi_robot")
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             history=HistoryPolicy.KEEP_LAST)
        self.tfb = TransformBroadcaster(self)
        self.map_pub = self.create_publisher(PointCloud2, "/initial_map", latched)
        self.marker_pub = self.create_publisher(MarkerArray, "/multi_robot/markers", 10)

        m = voxel_ds(load_ply(PLY), VOX)
        h = Header(); h.frame_id = "map"
        self.map_msg = point_cloud2.create_cloud_xyz32(h, m)
        self.get_logger().info(f"map: {len(m)} pts after voxel {VOX} m")

        self.robots = []
        for name, path, rgb in ROBOTS:
            poses = load_poses(path)
            self.robots.append({
                "name": name, "rgb": rgb, "poses": poses,
                "path_pub": self.create_publisher(Path, f"/{name}/path", latched),
                "pose_pub": self.create_publisher(PoseWithCovarianceStamped, f"/{name}/pcl_pose", latched),
                "path": Path(),
            })
            self.robots[-1]["path"].header.frame_id = "map"
            self.get_logger().info(f"{name}: {len(poses)} poses")

        self.n = min(len(r["poses"]) for r in self.robots)
        self.i = 0
        self.create_timer(1.0, self.pub_map)
        self.create_timer(1.0 / RATE, self.tick)
        self.pub_map()

    def pub_map(self):
        self.map_msg.header.stamp = self.get_clock().now().to_msg()
        self.map_pub.publish(self.map_msg)

    def tick(self):
        if self.i >= self.n:                       # loop
            self.i = 0
            for r in self.robots:
                r["path"] = Path(); r["path"].header.frame_id = "map"
        now = self.get_clock().now().to_msg()
        cur = {}
        for r in self.robots:
            x, y, z, qx, qy, qz, qw = r["poses"][self.i]
            cur[r["name"]] = (x, y, z)

            pc = PoseWithCovarianceStamped()
            pc.header.stamp, pc.header.frame_id = now, "map"
            pc.pose.pose.position.x, pc.pose.pose.position.y, pc.pose.pose.position.z = x, y, z
            pc.pose.pose.orientation.x, pc.pose.pose.orientation.y = qx, qy
            pc.pose.pose.orientation.z, pc.pose.pose.orientation.w = qz, qw
            r["pose_pub"].publish(pc)

            ps = PoseStamped(); ps.header = pc.header; ps.pose = pc.pose.pose
            r["path"].poses.append(ps); r["path"].header.stamp = now
            r["path_pub"].publish(r["path"])

            t = TransformStamped()
            t.header.stamp, t.header.frame_id = now, "map"
            t.child_frame_id = f"{r['name']}/os_lidar"
            t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = x, y, z
            t.transform.rotation.x, t.transform.rotation.y = qx, qy
            t.transform.rotation.z, t.transform.rotation.w = qz, qw
            self.tfb.sendTransform(t)

        self.publish_markers(now, cur)
        self.i += 1

    def publish_markers(self, now, cur):
        a, b = np.array(cur["robot1"]), np.array(cur["robot2"])
        rng = float(np.linalg.norm(b - a))
        ma = MarkerArray()

        line = Marker()
        line.header.frame_id, line.header.stamp = "map", now
        line.ns, line.id, line.type, line.action = "links", 0, Marker.LINE_LIST, Marker.ADD
        line.scale.x = 0.25
        line.color.r = line.color.g = line.color.b = 1.0; line.color.a = 0.85
        line.pose.orientation.w = 1.0
        line.points = [Point(x=a[0], y=a[1], z=a[2]), Point(x=b[0], y=b[1], z=b[2])]
        line.lifetime = DurationMsg(sec=1)
        ma.markers.append(line)

        mid = (a + b) / 2.0
        label = Marker()
        label.header.frame_id, label.header.stamp = "map", now
        label.ns, label.id, label.type, label.action = "labels", 0, Marker.TEXT_VIEW_FACING, Marker.ADD
        label.pose.position.x, label.pose.position.y, label.pose.position.z = float(mid[0]), float(mid[1]), float(mid[2]) + 1.8
        label.pose.orientation.w = 1.0
        label.scale.z = 1.4
        label.color.r = label.color.g = label.color.b = label.color.a = 1.0
        label.text = f"{rng:.2f} m"
        label.lifetime = DurationMsg(sec=1)
        ma.markers.append(label)
        self.marker_pub.publish(ma)


def main():
    rclpy.init()
    rclpy.spin(Replay())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
