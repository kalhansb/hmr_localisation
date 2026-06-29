#!/usr/bin/env python3
"""Publish the saved maps as latched PointCloud2 topics for RViz (static viz, no rerun).

Topics (all latched / transient_local so RViz gets them on connect):
  /viz/scovox       production scovox occupancy map (output/scovox_map.npy, odom frame)
  /viz/glim         GLIM reference map (output/glim_map.pcd, map frame, subsampled)
  /viz/scovox_raw   OLD raw-input blob (output/feed_cmp/raw.npy, odom frame) -- the "before"

Also broadcasts a static map->odom = identity TF (scovox/odom and GLIM/map agree to
~0.3 m on this bag), so RViz fixed_frame=map shows everything overlaid.

Run in container with ROS sourced. Ctrl-C / kill to stop.
"""
import os
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster

OUT = "/ws/output"
GLIM_MAX = 2_000_000      # subsample GLIM cloud to keep RViz responsive


def load_npy(path):
    if not os.path.exists(path):
        print(f"  (missing {path})")
        return None
    a = np.load(path).astype(np.float32)
    return a[:, :3]


def load_pcd(path, cap=None):
    if not os.path.exists(path):
        print(f"  (missing {path})")
        return None
    with open(path, "rb") as f:
        hdr, n = [], 0
        while True:
            line = f.readline().decode("ascii", "replace")
            hdr.append(line)
            if line.startswith("POINTS"):
                n = int(line.split()[1])
            if line.startswith("DATA"):
                fmt = line.split()[1].strip(); break
        if fmt == "binary":
            buf = f.read(n * 12)
            a = np.frombuffer(buf, dtype=np.float32, count=n * 3).reshape(-1, 3)
        else:
            a = np.loadtxt(path, skiprows=len(hdr))[:, :3].astype(np.float32)
    if cap and len(a) > cap:
        idx = np.random.default_rng(0).choice(len(a), size=cap, replace=False)
        a = a[idx]
    return a


def make_cloud(xyz, frame):
    msg = PointCloud2()
    msg.header = Header(frame_id=frame)
    msg.height = 1
    msg.width = xyz.shape[0]
    msg.fields = [PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1)
                  for i, n in enumerate(("x", "y", "z"))]
    msg.is_bigendian = False
    msg.point_step = 12
    msg.row_step = 12 * xyz.shape[0]
    msg.is_dense = True
    msg.data = np.ascontiguousarray(xyz, dtype=np.float32).tobytes()
    return msg


class Pub(Node):
    def __init__(self):
        super().__init__("viz_publish_maps")
        latched = QoSProfile(depth=1)
        latched.history = HistoryPolicy.KEEP_LAST
        latched.durability = DurabilityPolicy.TRANSIENT_LOCAL
        latched.reliability = ReliabilityPolicy.RELIABLE

        self.clouds = []
        defs = [
            ("/viz/scovox",     load_npy(f"{OUT}/scovox_map.npy"),                 "odom"),
            ("/viz/glim",       load_pcd(f"{OUT}/glim_map.pcd", GLIM_MAX),         "map"),
            ("/viz/scovox_raw", load_npy(f"{OUT}/feed_cmp/raw.npy"),              "odom"),
        ]
        for topic, xyz, frame in defs:
            if xyz is None or len(xyz) == 0:
                continue
            pub = self.create_publisher(PointCloud2, topic, latched)
            msg = make_cloud(xyz, frame)
            self.clouds.append((pub, msg))
            print(f"  {topic}: {len(xyz):,} pts (frame={frame})")

        # static map->odom identity
        self.stf = StaticTransformBroadcaster(self)
        t = TransformStamped()
        t.header.frame_id = "map"; t.child_frame_id = "odom"
        t.transform.rotation.w = 1.0
        self.stf.sendTransform(t)

        self.timer = self.create_timer(3.0, self.republish)
        self.republish()

    def republish(self):
        now = self.get_clock().now().to_msg()
        for pub, msg in self.clouds:
            msg.header.stamp = now
            pub.publish(msg)


def main():
    rclpy.init()
    print("publishing maps for RViz ...")
    n = Pub()
    print("ready. (Ctrl-C to stop)")
    rclpy.spin(n)
    rclpy.shutdown()


main()
