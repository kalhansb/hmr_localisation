#!/usr/bin/env python3
"""Capture a few (scan, localized-pose) pairs so we can measure registration
accuracy against the GT map offline. Pairs /ouster/points with the nearest
/pcl_pose by timestamp; saves raw scan points (lidar frame) + the 4x4
map<-lidar transform to an npz. No tf2 needed."""
import sys
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy, \
    DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from geometry_msgs.msg import PoseWithCovarianceStamped

OUT = sys.argv[1] if len(sys.argv) > 1 else "/ws/output/scan_pose_pairs.npz"
N_WANT = int(sys.argv[2]) if len(sys.argv) > 2 else 10


def quat_to_R(x, y, z, w):
    n = (x*x + y*y + z*z + w*w) ** 0.5
    x, y, z, w = x/n, y/n, z/n, w/n
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w),   2*(x*z+y*w)],
        [2*(x*y+z*w),   1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w),   2*(y*z+x*w),   1-2*(x*x+y*y)]])


class Cap(Node):
    def __init__(self):
        super().__init__("cap_scan")
        self.set_parameters([Parameter("use_sim_time", value=True)])
        self.poses = []   # (t, x,y,z,qx,qy,qz,qw)
        self.samples = []
        self.last_t = -1e9
        pose_qos = QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE,
                              durability=DurabilityPolicy.TRANSIENT_LOCAL,
                              history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(PoseWithCovarianceStamped, "/pcl_pose",
                                 self.cb_pose, pose_qos)
        self.create_subscription(PointCloud2, "/ouster/points",
                                 self.cb_scan, qos_profile_sensor_data)

    def cb_pose(self, m):
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        p, q = m.pose.pose.position, m.pose.pose.orientation
        self.poses.append((t, p.x, p.y, p.z, q.x, q.y, q.z, q.w))

    def cb_scan(self, m):
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        if t - self.last_t < 3.0 or not self.poses:
            return
        # nearest pose by time (require <0.2s)
        arr = np.array([p[0] for p in self.poses])
        j = int(np.argmin(np.abs(arr - t)))
        if abs(self.poses[j][0] - t) > 0.2:
            return
        pts = point_cloud2.read_points(m, field_names=("x", "y", "z"),
                                       skip_nans=True)
        xyz = np.column_stack([pts["x"], pts["y"], pts["z"]]).astype(np.float64)
        rng = np.linalg.norm(xyz, axis=1)
        xyz = xyz[(rng > 1.0) & (rng < 80.0)]
        if len(xyz) < 1000:
            return
        # voxel downsample scan to ~6k pts for fast NN
        k = np.floor(xyz / 0.3).astype(np.int64)
        _, idx = np.unique(k, axis=0, return_index=True)
        xyz = xyz[idx]
        _, x, y, z, qx, qy, qz, qw = self.poses[j]
        T = np.eye(4); T[:3, :3] = quat_to_R(qx, qy, qz, qw); T[:3, 3] = [x, y, z]
        self.samples.append((xyz, T))
        self.last_t = t
        self.get_logger().info(f"captured sample {len(self.samples)} "
                               f"({len(xyz)} pts) at pose ({x:.1f},{y:.1f},{z:.1f})")
        if len(self.samples) >= N_WANT:
            np.savez(OUT, **{f"scan{i}": s[0] for i, s in enumerate(self.samples)},
                     **{f"T{i}": s[1] for i, s in enumerate(self.samples)},
                     n=len(self.samples))
            self.get_logger().info(f"saved {len(self.samples)} pairs to {OUT}")
            rclpy.shutdown()


def main():
    rclpy.init()
    rclpy.spin(Cap())


if __name__ == "__main__":
    main()
