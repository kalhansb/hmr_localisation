#!/usr/bin/env python3
# Live capture of the SCovox occupancy map. Subscribes to /scovox_node/pointcloud
# (scovox republishes the whole map at ~0.5 Hz) and ATOMICALLY writes the latest
# full map to OUT on EVERY republish. So OUT always holds the most recent complete
# map regardless of how this process dies -- no dependency on a clean shutdown
# (an earlier version relied on a SIGINT handler that rclpy.spin() never delivered,
# and the scovox node deregisters its publisher on SIGTERM, so post-run capture is
# impossible). The last write before the bag's /clock freezes is the final map
# (at most ~2 s of sim time stale vs the last scan -- negligible for a ~1000 s run).
#
# Usage: python3 capture_scovox_live.py /ws/output/scovox_map_odom.npy
import os
import sys

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2

OUT = sys.argv[1] if len(sys.argv) > 1 else "/ws/output/scovox_map.npy"
TMP = OUT + ".tmp.npy"          # ends in .npy so np.save writes it verbatim (no extra suffix)


class LiveCap(Node):
    def __init__(self):
        super().__init__("scovox_live_capture")
        qos = QoSProfile(depth=1)
        qos.history = HistoryPolicy.KEEP_LAST
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        qos.durability = DurabilityPolicy.VOLATILE
        self.create_subscription(PointCloud2, "/scovox_node/pointcloud", self.cb, qos)
        self.n = 0
        self.rx = 0

    def cb(self, msg):
        self.rx += 1
        s = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        arr = np.stack([s["x"], s["y"], s["z"]], axis=-1).astype(np.float64)
        print(f"rx#{self.rx}: width={msg.width} pts={arr.shape[0]} frame={msg.header.frame_id}", flush=True)
        if arr.shape[0] == 0:
            return                       # skip empty republishes; keep last non-empty on disk
        np.save(TMP, arr)
        os.replace(TMP, OUT)             # atomic: OUT always holds the latest complete non-empty map
        self.n += 1


def main():
    rclpy.init()
    n = LiveCap()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        print(f"capture_scovox_live: stopping after {n.n} writes (OUT={OUT})", flush=True)
        rclpy.try_shutdown()


main()
