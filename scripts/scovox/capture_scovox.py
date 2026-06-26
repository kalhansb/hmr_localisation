#!/usr/bin/env python3
# Capture one full /scovox_node/pointcloud message (the persistent occupancy map,
# in the `map` frame) to /ws/output/scovox_map.npy as an Nx3 float64 array.
# scovox republishes the whole map at ~0.5 Hz, so a fresh full snapshot arrives
# within ~2 s. Run inside the container with ROS sourced.
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2

OUT = "/ws/output/scovox_map.npy"


class Cap(Node):
    def __init__(self):
        super().__init__("scovox_capture")
        qos = QoSProfile(depth=1)
        qos.history = HistoryPolicy.KEEP_LAST
        qos.reliability = ReliabilityPolicy.BEST_EFFORT  # accepts reliable or best-effort pub
        qos.durability = DurabilityPolicy.VOLATILE
        self.create_subscription(PointCloud2, "/scovox_node/pointcloud", self.cb, qos)
        self.done = False

    def cb(self, msg):
        s = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        arr = np.stack([s["x"], s["y"], s["z"]], axis=-1).astype(np.float64)
        np.save(OUT, arr)
        mn, mx = arr.min(0), arr.max(0)
        print(f"captured {arr.shape[0]} pts  frame={msg.header.frame_id}")
        print(f"bbox_min={np.round(mn,2)}  bbox_max={np.round(mx,2)}  size={np.round(mx-mn,2)}")
        self.done = True


def main():
    rclpy.init()
    n = Cap()
    waited = 0.0
    while rclpy.ok() and not n.done and waited < 20.0:
        rclpy.spin_once(n, timeout_sec=0.5)
        waited += 0.5
    if not n.done:
        print("NO MESSAGE received on /scovox_node/pointcloud within 20 s")
    rclpy.shutdown()


main()
