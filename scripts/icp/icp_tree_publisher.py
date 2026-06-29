#!/usr/bin/env python3
"""Expose the ICP localizer's pose as a proper REP-105 map -> odom -> base_link
TF tree (the ICP equivalent of NDT "Mode B" / run_localization_tree_noekf.sh).

WHY a separate node: icp_localization_ros2 can only broadcast map -> odom itself
when is_use_odometry=true, which ALSO requires a real nav_msgs/Odometry stream on
odometry_data_topic (TfPublisher::odometryCallback). With only lidar + IMU there
is no such topic, and the IMU-only odom path in the package is an empty TODO
(TfPublisher.cpp:181). The `is_provide_odom_frame` YAML param is dead (never read;
the internal flag is aliased to is_use_odometry at ICPlocalization.cpp:192). So in
Mode A the node hard-publishes the single edge map -> range_sensor and nothing else.

Instead of patching/rebuilding the package, we consume its absolute pose topic
`range_sensor_pose` (PoseStamped in the map frame = map->os_lidar, since the map is
identity-seeded in the lidar frame) and rebuild the canonical tree as a SEPARATE
branch under map:

    map  --(this node, dynamic @ scan rate)-->  odom
    odom --(static identity)-->                 base_link
    base_link --(static extrinsic)-->           os_lidar, imu

Like the NDT noekf baseline, odom->base_link is a static identity (no wheel
odometry to smooth with), so map->odom carries the full base pose:
    map->odom = map->os_lidar o (base_link->os_lidar)^-1   ( = map->base_link ).
The base_link->os_lidar / base_link->imu extrinsics are the SAME numbers as NDT
Mode B (run_localization_tree_noekf.sh), so ICP's base_link coincides with NDT's
and the two localizers' map->base_link trajectories are directly comparable.

The ICP node's own map->range_sensor (+ range_sensor->inertial_sensor) keep being
broadcast on a DISJOINT set of frame names, so there is no double-parent conflict:
every child frame (odom, base_link, os_lidar, imu, range_sensor, inertial_sensor)
has exactly one publisher. The live /ouster/points (frame os_lidar) is placed in
the map via this tree.

Usage (inside the Humble container, with the bag playing on the sim clock):
  python3 scripts/icp/icp_tree_publisher.py --ros-args -p use_sim_time:=true
Tunables (all have NDT-matching defaults):
  -p pose_topic:=range_sensor_pose
  -p map_frame:=map -p odom_frame:=odom -p base_frame:=base_link
  -p lidar_frame:=os_lidar -p imu_frame:=imu
  -p base_to_lidar:=[0.1105,0.0,0.404,3.14159265]   # x y z yaw(rad)
  -p base_to_imu:=[0.062,0.0,0.015,1.5707963]       # x y z yaw(rad)
"""
import math
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster


def quat_to_R(x, y, z, w):
    n = (x * x + y * y + z * z + w * w) ** 0.5
    if n == 0.0:
        return np.eye(3)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w),     2 * (x * z + y * w)],
        [2 * (x * y + z * w),     1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w),     2 * (y * z + x * w),     1 - 2 * (x * x + y * y)]])


def R_to_quat(R):
    # Shepperd's method (numerically stable).
    t = np.trace(R)
    if t > 0.0:
        s = math.sqrt(t + 1.0) * 2.0
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    return x, y, z, w


def xform(parent, child, t, q, stamp):
    m = TransformStamped()
    m.header.stamp = stamp
    m.header.frame_id = parent
    m.child_frame_id = child
    m.transform.translation.x = float(t[0])
    m.transform.translation.y = float(t[1])
    m.transform.translation.z = float(t[2])
    m.transform.rotation.x = float(q[0])
    m.transform.rotation.y = float(q[1])
    m.transform.rotation.z = float(q[2])
    m.transform.rotation.w = float(q[3])
    return m


class IcpTreePublisher(Node):
    def __init__(self):
        super().__init__("icp_tree_publisher")
        self.set_parameters([Parameter("use_sim_time", value=True)])
        gp = self.declare_parameter
        self.pose_topic = gp("pose_topic", "range_sensor_pose").value
        self.map_frame = gp("map_frame", "map").value
        self.odom_frame = gp("odom_frame", "odom").value
        self.base_frame = gp("base_frame", "base_link").value
        self.lidar_frame = gp("lidar_frame", "os_lidar").value
        self.imu_frame = gp("imu_frame", "imu").value
        b2l = gp("base_to_lidar", [0.1105, 0.0, 0.404, 3.14159265]).value
        b2i = gp("base_to_imu", [0.062, 0.0, 0.015, 1.5707963]).value

        # base_link -> os_lidar extrinsic (the NDT Mode B lidar_tf): keep R and t
        # so we can analytically remove it to turn the lidar pose into the base pose.
        self.R_bl = quat_to_R(0.0, 0.0, math.sin(b2l[3] / 2.0), math.cos(b2l[3] / 2.0))
        self.t_bl = np.array([b2l[0], b2l[1], b2l[2]])

        self.tfb = TransformBroadcaster(self)
        self.static = StaticTransformBroadcaster(self)
        self._publish_static(b2l, b2i)

        # range_sensor_pose is published KeepLast(1) reliable/volatile; match
        # reliable with a deeper local queue so we don't drop poses.
        qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(PoseStamped, self.pose_topic, self.cb, qos)
        self.n = 0
        self.get_logger().info(
            f"REP-105 tree: {self.map_frame}->{self.odom_frame}(dynamic from "
            f"{self.pose_topic})->{self.base_frame}->{{{self.lidar_frame},{self.imu_frame}}}")

    def _publish_static(self, b2l, b2i):
        stamp = self.get_clock().now().to_msg()

        def yaw_q(yaw):
            return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))

        tfs = [
            # odom -> base_link : static IDENTITY (no wheel odometry to smooth).
            xform(self.odom_frame, self.base_frame, (0.0, 0.0, 0.0),
                  (0.0, 0.0, 0.0, 1.0), stamp),
            # base_link -> os_lidar : NDT Mode B lidar extrinsic.
            xform(self.base_frame, self.lidar_frame, (b2l[0], b2l[1], b2l[2]),
                  yaw_q(b2l[3]), stamp),
            # base_link -> imu : NDT Mode B imu extrinsic (cosmetic / parity).
            xform(self.base_frame, self.imu_frame, (b2i[0], b2i[1], b2i[2]),
                  yaw_q(b2i[3]), stamp),
        ]
        self.static.sendTransform(tfs)

    def cb(self, m):
        p, q = m.pose.position, m.pose.orientation
        R_ml = quat_to_R(q.x, q.y, q.z, q.w)          # map <- os_lidar (ICP pose)
        t_ml = np.array([p.x, p.y, p.z])
        # map->odom = map->lidar o (base->lidar)^-1, with odom->base = identity,
        # so map->odom == map->base_link.
        R_mo = R_ml @ self.R_bl.T
        t_mo = t_ml - R_mo @ self.t_bl
        qx, qy, qz, qw = R_to_quat(R_mo)
        self.tfb.sendTransform(
            xform(self.map_frame, self.odom_frame, t_mo, (qx, qy, qz, qw),
                  m.header.stamp))
        self.n += 1
        if self.n % 25 == 0:
            self.get_logger().info(
                f"map->odom poses={self.n} base=({t_mo[0]:.2f},{t_mo[1]:.2f},{t_mo[2]:.2f})")


def main():
    rclpy.init()
    node = IcpTreePublisher()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
