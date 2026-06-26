#!/usr/bin/env python3
"""Multi-robot relative-pose monitor.

All robots localize into the shared `map` frame, so the pose of robot B relative
to robot A is simply  T_AB = inv(T_A) * T_B  where T_x is robot x's map pose.

Subscribes to each robot's latched /<ns>/pcl_pose (map frame), and for every
non-reference robot publishes its pose relative to the reference (first) robot:

  /multi_robot/<other>_in_<ref>  geometry_msgs/PoseStamped   (frame <ref>/os_lidar)
  /multi_robot/markers           visualization_msgs/MarkerArray  (map frame:
                                 a sphere per robot, a link line, a range label)

and appends a row per tick to output/relative_pose.csv.

Usage: relative_pose.py [ref_robot] [other_robot ...]   (first arg = reference)
       defaults to: robot1 robot2
"""
import csv
import math
import os
import sys

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from builtin_interfaces.msg import Duration as DurationMsg
from geometry_msgs.msg import Point, PoseStamped, PoseWithCovarianceStamped
from visualization_msgs.msg import Marker, MarkerArray

NS = sys.argv[1:] or ['robot1', 'robot2']
REF = NS[0]
OUT_CSV = os.environ.get('REL_CSV', '/ws/output/relative_pose.csv')
TRAJ_DIR = os.environ.get('TRAJ_DIR', '/ws/output')   # per-robot path_<ns>.csv


def quat_to_mat(x, y, z, w):
    n = math.sqrt(x * x + y * y + z * z + w * w) or 1.0
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def mat_to_quat(R):
    t = np.trace(R)
    if t > 0:
        s = math.sqrt(t + 1.0) * 2
        w, x, y, z = 0.25 * s, (R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        w, x, y, z = (R[2, 1] - R[1, 2]) / s, 0.25 * s, (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        w, x, y, z = (R[0, 2] - R[2, 0]) / s, (R[0, 1] + R[1, 0]) / s, 0.25 * s, (R[1, 2] + R[2, 1]) / s
    else:
        s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        w, x, y, z = (R[1, 0] - R[0, 1]) / s, (R[0, 2] + R[2, 0]) / s, (R[1, 2] + R[2, 1]) / s, 0.25 * s
    return x, y, z, w


def pose_to_T(p):
    T = np.eye(4)
    T[:3, :3] = quat_to_mat(p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w)
    T[:3, 3] = [p.position.x, p.position.y, p.position.z]
    return T


# stable, visually distinct colours per robot index
COLORS = [(0.10, 0.90, 0.20), (0.95, 0.75, 0.10), (0.20, 0.55, 0.95),
          (0.90, 0.25, 0.55), (0.60, 0.40, 0.90)]


class RelMon(Node):
    def __init__(self):
        super().__init__('relative_pose')
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         history=HistoryPolicy.KEEP_LAST)
        self.color = {ns: COLORS[i % len(COLORS)] for i, ns in enumerate(NS)}
        self.latest = {}     # ns -> 4x4 map pose
        for ns in NS:
            self.create_subscription(
                PoseWithCovarianceStamped, f'/{ns}/pcl_pose',
                lambda m, n=ns: self._on_pose(n, m), qos)
        self.pose_pubs = {
            ns: self.create_publisher(PoseStamped, f'/multi_robot/{ns}_in_{REF}', 10)
            for ns in NS[1:]}
        self.marker_pub = self.create_publisher(MarkerArray, '/multi_robot/markers', 10)

        os.makedirs(os.path.dirname(OUT_CSV), exist_ok=True)
        self.csv_f = open(OUT_CSV, 'w', newline='')
        self.csv = csv.writer(self.csv_f)
        self.csv.writerow(['wall_t', 'ref', 'other', 'dx', 'dy', 'dz', 'range_m', 'dyaw_deg'])

        # per-robot map-frame trajectory CSVs, driven by pcl_pose (the localizer's
        # /path topic is low-rate and unreliable here). Same columns as the
        # single-robot path.csv so the replay/plot scripts read them directly.
        os.makedirs(TRAJ_DIR, exist_ok=True)
        self.traj_f, self.traj_w = {}, {}
        for ns in NS:
            f = open(os.path.join(TRAJ_DIR, f'path_{ns}.csv'), 'w', newline='')
            w = csv.writer(f)
            w.writerow(['stamp', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw'])
            self.traj_f[ns], self.traj_w[ns] = f, w
        self._traj_writes = 0

        self.t0 = None
        self.create_timer(0.1, self.tick)
        self.create_timer(2.0, self.report)
        self.get_logger().info(f'reference={REF}; tracking {NS}; csv={OUT_CSV}')

    def _on_pose(self, ns, msg):
        self.latest[ns] = pose_to_T(msg.pose.pose)
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.traj_w[ns].writerow([f'{t:.6f}', p.x, p.y, p.z, q.x, q.y, q.z, q.w])
        self._traj_writes += 1
        if self._traj_writes % 25 == 0:
            for f in self.traj_f.values():
                f.flush()

    # ---- marker helpers (all in the shared map frame) ----
    def _stamp(self):
        return self.get_clock().now().to_msg()

    def _sphere(self, mid, xyz, rgb):
        m = Marker()
        m.header.frame_id, m.header.stamp = 'map', self._stamp()
        m.ns, m.id, m.type, m.action = 'robots', mid, Marker.SPHERE, Marker.ADD
        m.pose.position.x, m.pose.position.y, m.pose.position.z = (float(xyz[0]), float(xyz[1]), float(xyz[2]))
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = 1.6
        m.color.r, m.color.g, m.color.b, m.color.a = (rgb[0], rgb[1], rgb[2], 1.0)
        m.lifetime = DurationMsg(sec=1)
        return m

    def _line(self, mid, a, b):
        m = Marker()
        m.header.frame_id, m.header.stamp = 'map', self._stamp()
        m.ns, m.id, m.type, m.action = 'links', mid, Marker.LINE_LIST, Marker.ADD
        m.scale.x = 0.25
        m.color.r = m.color.g = m.color.b = 1.0
        m.color.a = 0.85
        m.pose.orientation.w = 1.0
        m.points = [Point(x=float(a[0]), y=float(a[1]), z=float(a[2])),
                    Point(x=float(b[0]), y=float(b[1]), z=float(b[2]))]
        m.lifetime = DurationMsg(sec=1)
        return m

    def _label(self, mid, xyz, text):
        m = Marker()
        m.header.frame_id, m.header.stamp = 'map', self._stamp()
        m.ns, m.id, m.type, m.action = 'labels', mid, Marker.TEXT_VIEW_FACING, Marker.ADD
        m.pose.position.x, m.pose.position.y, m.pose.position.z = (float(xyz[0]), float(xyz[1]), float(xyz[2]) + 1.8)
        m.pose.orientation.w = 1.0
        m.scale.z = 1.3
        m.color.r = m.color.g = m.color.b = m.color.a = 1.0
        m.text = text
        m.lifetime = DurationMsg(sec=1)
        return m

    def tick(self):
        if REF not in self.latest:
            return
        now = self.get_clock().now()
        if self.t0 is None:
            self.t0 = now
        wall_t = (now - self.t0).nanoseconds * 1e-9

        Tref = self.latest[REF]
        ref_xyz = Tref[:3, 3]
        Tref_inv = np.linalg.inv(Tref)

        markers = MarkerArray()
        markers.markers.append(self._sphere(0, ref_xyz, self.color[REF]))
        markers.markers.append(self._label(0, ref_xyz, REF))

        for i, ns in enumerate(NS[1:], start=1):
            if ns not in self.latest:
                continue
            Toth = self.latest[ns]
            oth_xyz = Toth[:3, 3]
            Trel = Tref_inv @ Toth
            d = Trel[:3, 3]
            rng = float(np.linalg.norm(d))
            dyaw = math.degrees(math.atan2(Trel[1, 0], Trel[0, 0]))

            ps = PoseStamped()
            ps.header.stamp = now.to_msg()
            ps.header.frame_id = f'{REF}/os_lidar'
            ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = (float(d[0]), float(d[1]), float(d[2]))
            qx, qy, qz, qw = mat_to_quat(Trel[:3, :3])
            ps.pose.orientation.x, ps.pose.orientation.y = float(qx), float(qy)
            ps.pose.orientation.z, ps.pose.orientation.w = float(qz), float(qw)
            self.pose_pubs[ns].publish(ps)

            self.csv.writerow([f'{wall_t:.3f}', REF, ns,
                               f'{d[0]:.4f}', f'{d[1]:.4f}', f'{d[2]:.4f}',
                               f'{rng:.4f}', f'{dyaw:.2f}'])

            markers.markers.append(self._sphere(i, oth_xyz, self.color[ns]))
            markers.markers.append(self._label(i, oth_xyz, ns))
            markers.markers.append(self._line(i, ref_xyz, oth_xyz))
            markers.markers.append(self._label(100 + i, (ref_xyz + oth_xyz) / 2.0, f'{rng:.2f} m'))

        self.marker_pub.publish(markers)
        self.csv_f.flush()

    def report(self):
        if REF not in self.latest:
            self.get_logger().info(f'waiting for /{REF}/pcl_pose ...')
            return
        Tref_inv = np.linalg.inv(self.latest[REF])
        parts = []
        for ns in NS[1:]:
            if ns in self.latest:
                d = (Tref_inv @ self.latest[ns])[:3, 3]
                parts.append(f'{ns}: {np.linalg.norm(d):.2f} m '
                             f'(dx={d[0]:.2f} dy={d[1]:.2f} dz={d[2]:.2f})')
            else:
                parts.append(f'{ns}: (waiting)')
        self.get_logger().info(f'range from {REF} -> ' + ' | '.join(parts))


def main():
    rclpy.init()
    node = RelMon()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.csv_f.close()
    for f in node.traj_f.values():
        f.flush()
        f.close()
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
