#!/usr/bin/env python3
"""Accumulate GLIM's deskewed points (/glim_ros/aligned_points, map frame) over the
run into a voxel-deduped reference cloud -- this IS 'the downsampled deskewed GLIM
points' the scovox map should match. Saves on SIGINT to /ws/output/glim_points_accum.npy
(unique REF_RES voxel centers, map frame)."""
import signal
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2

OUT = "/ws/output/glim_points_accum.npy"
REF_RES = 0.025          # dedup grid (fine, so it can be re-voxelized to any test res)


class Acc(Node):
    def __init__(self):
        super().__init__("accumulate_aligned")
        q = QoSProfile(depth=10)
        q.history = HistoryPolicy.KEEP_LAST
        q.reliability = ReliabilityPolicy.RELIABLE
        q.durability = DurabilityPolicy.VOLATILE
        self.create_subscription(PointCloud2, "/glim_ros/aligned_points", self.cb, q)
        self.vox = set()
        self.nmsg = 0

    def cb(self, msg):
        s = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        a = np.stack([s["x"], s["y"], s["z"]], -1).astype(np.float64)
        if a.shape[0] == 0:
            return
        c = np.floor(a / REF_RES + 0.5).astype(np.int64)
        self.vox.update(map(tuple, c))
        self.nmsg += 1

    def save(self):
        if not self.vox:
            print("accumulate_aligned: nothing to save"); return
        arr = (np.array(sorted(self.vox), dtype=np.float64)) * REF_RES
        np.save(OUT, arr)
        print(f"accumulate_aligned: saved {arr.shape[0]:,} ref voxels "
              f"({self.nmsg} msgs) -> {OUT}", flush=True)


def main():
    # rclpy installs its own SIGINT handler that raises KeyboardInterrupt inside
    # spin (a custom signal.signal handler races/loses against it -> never saved).
    # So catch SIGTERM -> KeyboardInterrupt ourselves and always save in finally.
    signal.signal(signal.SIGTERM, lambda *_: (_ for _ in ()).throw(KeyboardInterrupt()))
    rclpy.init()
    n = Acc()
    try:
        while rclpy.ok():
            rclpy.spin_once(n, timeout_sec=0.2)
    except KeyboardInterrupt:
        pass
    finally:
        n.save()
        rclpy.try_shutdown()


main()
