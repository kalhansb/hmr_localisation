#!/usr/bin/env python3
"""Probe GLIM's published clouds so we can wire SCovox to them correctly.

Reports, for each of /glim_ros/points and /glim_ros/aligned_points_corrected
(falling back to /glim_ros/aligned_points):
  - header.frame_id   (the frame SCovox must connect to integration_frame via TF)
  - per-frame point count (confirm it's a per-scan cloud, not the global map)
  - bbox
Also dumps the TF edges seen on /tf and /tf_static (parent -> child) so we can
confirm odom->imu and map->odom->imu->os_lidar exist.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from tf2_msgs.msg import TFMessage
import numpy as np

CLOUD_TOPICS = [
    "/glim_ros/points",
    "/glim_ros/aligned_points_corrected",
    "/glim_ros/aligned_points",
]


def both_qos():
    out = []
    for rel in (ReliabilityPolicy.BEST_EFFORT, ReliabilityPolicy.RELIABLE):
        q = QoSProfile(depth=5)
        q.history = HistoryPolicy.KEEP_LAST
        q.reliability = rel
        q.durability = DurabilityPolicy.VOLATILE
        out.append(q)
    return out


class Probe(Node):
    def __init__(self):
        super().__init__("probe_glim_topics")
        self.cloud = {}          # topic -> (frame_id, npts, bbox)
        for t in CLOUD_TOPICS:
            for q in both_qos():
                self.create_subscription(PointCloud2, t,
                                         lambda m, tt=t: self.cloud_cb(tt, m), q)
        self.tf_edges = set()    # (parent, child, static?)
        tl = QoSProfile(depth=100); tl.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(TFMessage, "/tf", lambda m: self.tf_cb(m, False),
                                 QoSProfile(depth=100))
        self.create_subscription(TFMessage, "/tf_static",
                                 lambda m: self.tf_cb(m, True), tl)

    def cloud_cb(self, topic, msg):
        if topic in self.cloud:
            return
        s = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        a = np.stack([s["x"], s["y"], s["z"]], -1).astype(float)
        if a.shape[0] == 0:
            return
        mn, mx = a.min(0), a.max(0)
        self.cloud[topic] = (msg.header.frame_id, a.shape[0],
                             (np.round(mn, 1), np.round(mx, 1)),
                             [f.name for f in msg.fields])

    def tf_cb(self, msg, static):
        for tr in msg.transforms:
            self.tf_edges.add((tr.header.frame_id, tr.child_frame_id, static))


def main():
    rclpy.init()
    n = Probe()
    w = 0.0
    while rclpy.ok() and w < 45 and len(n.cloud) < 2:
        rclpy.spin_once(n, timeout_sec=0.1); w += 0.1
    # let TF accumulate a touch more
    for _ in range(20):
        rclpy.spin_once(n, timeout_sec=0.1)
    print("==== GLIM cloud topics ====")
    for t in CLOUD_TOPICS:
        if t in n.cloud:
            fid, npts, (mn, mx), fields = n.cloud[t]
            print(f"  {t}")
            print(f"     frame_id={fid!r}  npts={npts:,}  fields={fields}")
            print(f"     bbox={mn} .. {mx}  size={np.round(mx-mn,1)}")
        else:
            print(f"  {t}: NO MESSAGE")
    print("\n==== TF edges (parent -> child) ====")
    for p, c, st in sorted(n.tf_edges):
        print(f"  {p} -> {c}   {'[static]' if st else '[dynamic]'}")
    rclpy.shutdown()


main()
