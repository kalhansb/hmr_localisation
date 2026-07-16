#!/usr/bin/env python3
"""Accumulate live LiDAR scans transformed into `map` via TF, overlay them on the
gt_map, and report a nearest-neighbour fitness (mean scan->map distance).

If localization is correct the aligned scan lands ON the gt_map surface -> crisp
overlay + small fitness. If NDT is lost/drifting -> smeared overlay + large fitness.
"""
import signal
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation as R
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
import tf2_ros


def load_pcd_bin(path):
    with open(path, "rb") as f:
        data = f.read()
    idx = data.find(b"DATA binary")
    nl = data.find(b"\n", idx)
    arr = np.frombuffer(data[nl + 1:], dtype=np.float32)
    n = arr.size // 4
    return arr[: n * 4].reshape(n, 4)[:, :3].astype(np.float64)


class Overlay(Node):
    def __init__(self):
        super().__init__("overlay_check")
        gp = self.declare_parameter
        self.cloud_topic = gp("cloud_topic", "/hesai/points").value
        self.sensor_frame = gp("sensor_frame", "hesai_lidar").value
        self.map_frame = gp("map_frame", "map").value
        self.out_png = gp("out_png", "/ws/bags/_introspect/overlay.png").value
        self.gt_pcd = gp("gt_pcd", "/ws/gt_map/gt_map_us050.pcd").value
        self.warmup = int(gp("warmup_scans", 15).value)
        self.maxscans = int(gp("max_scans", 60).value)
        self.stride = int(gp("stride", 2).value)
        self.ppscan = int(gp("pts_per_scan", 3000).value)
        self.set_parameters(
            [rclpy.parameter.Parameter("use_sim_time", rclpy.Parameter.Type.BOOL, True)]
        )
        self.gt = load_pcd_bin(self.gt_pcd)
        self.tree = cKDTree(self.gt)
        self.get_logger().info(f"gt_map loaded {self.gt.shape}")
        self.buf = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buf, self)
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST)
        self.sub = self.create_subscription(PointCloud2, self.cloud_topic, self.cb, qos)
        self.acc = []
        self.seen = 0
        self.kept = 0
        self.skipped = 0
        self.saved = False

    def cb(self, msg):
        self.seen += 1
        if self.seen < self.warmup or self.seen % self.stride:
            return
        try:
            # Latest available transform, NON-blocking: a synchronous timeout would
            # deadlock the single-threaded executor (tf can't update mid-callback).
            # map->odom is 30 Hz so "latest" lags the scan by <0.1 s (sub-dm at walk pace).
            t = self.buf.lookup_transform(
                self.map_frame, self.sensor_frame, Time(),
                Duration(seconds=0.0))
        except Exception:
            self.skipped += 1
            return
        pc = point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        xyz = np.stack([pc["x"], pc["y"], pc["z"]], axis=-1).astype(np.float64)
        if xyz.shape[0] > self.ppscan:
            xyz = xyz[np.random.choice(xyz.shape[0], self.ppscan, replace=False)]
        q = t.transform.rotation
        tr = t.transform.translation
        Rm = R.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
        self.acc.append(xyz @ Rm.T + np.array([tr.x, tr.y, tr.z]))
        self.kept += 1
        if self.kept % 10 == 0:
            self.get_logger().info(f"kept={self.kept} skipped={self.skipped}")
        if self.kept >= self.maxscans:
            self.save()
            rclpy.shutdown()

    def save(self):
        if self.saved or not self.acc:
            self.get_logger().warn(f"no data to save (kept={self.kept} skipped={self.skipped})")
            return
        self.saved = True
        A = np.concatenate(self.acc, axis=0)
        samp = A if A.shape[0] <= 30000 else A[np.random.choice(A.shape[0], 30000, replace=False)]
        d, _ = self.tree.query(samp)
        fit_mean, fit_med, fit_p90 = float(d.mean()), float(np.median(d)), float(np.percentile(d, 90))
        gt = self.gt
        gts = gt if gt.shape[0] <= 60000 else gt[np.random.choice(gt.shape[0], 60000, replace=False)]
        plt.figure(figsize=(11, 11))
        plt.scatter(gts[:, 0], gts[:, 1], s=0.4, c="0.6", label="gt_map")
        plt.scatter(A[:, 0], A[:, 1], s=0.4, c="red", label="aligned scans")
        # mark accumulated trajectory extent
        plt.gca().set_aspect("equal")
        plt.legend(markerscale=12, loc="upper right")
        plt.title(f"{self.sensor_frame}: aligned scans vs gt_map  |  "
                  f"fitness mean={fit_mean:.2f}m med={fit_med:.2f}m p90={fit_p90:.2f}m  "
                  f"(scans={self.kept})")
        plt.tight_layout()
        plt.savefig(self.out_png, dpi=90)
        print(f"OVERLAY_SAVED {self.out_png}", flush=True)
        print(f"FITNESS mean={fit_mean:.3f} median={fit_med:.3f} p90={fit_p90:.3f} "
              f"scans_kept={self.kept} scans_skipped={self.skipped} pts={A.shape[0]}", flush=True)


def main():
    rclpy.init()
    node = Overlay()

    def _sig(*_):
        node.save()
        rclpy.try_shutdown()
    signal.signal(signal.SIGTERM, _sig)
    signal.signal(signal.SIGINT, _sig)
    try:
        rclpy.spin(node)
    except Exception:
        pass
    finally:
        node.save()


if __name__ == "__main__":
    main()
