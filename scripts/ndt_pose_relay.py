#!/usr/bin/env python3
"""Restamp the NDT global pose into the odom frame for the robot_localization EKF.

lidar_localization_ros2 (Mode B) publishes its absolute pose on /pcl_pose with
header.frame_id = "map". The odom EKF (config/ekf_odom.yaml) runs with
world_frame=odom and would try to transform that map-frame pose into odom -- but
map->odom is produced *from* the EKF's odom->base output, so the lookup is
circular and deadlocks at startup.

odom is, by construction, a continuous frame coincident with map up to the small
residual the localizer keeps in map->odom. So we simply rewrite the pose's
frame_id from "map" to "odom" and republish on /pcl_pose_odom. The EKF fuses it
directly (no transform), broadcasts odom->base_link, and the localizer publishes
  map->odom = map->base_raw o (odom->base)^-1
leaving the map->base product unchanged while odom->base becomes smooth and
high-rate. See config/ekf_odom.yaml for the full rationale.

Run standalone (defaults shown):
  python3 ndt_pose_relay.py --ros-args \
      -p input_topic:=/pcl_pose -p output_topic:=/pcl_pose_odom -p output_frame:=odom
Normally brought up by launch/ekf_odom.launch.py.
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped


class NdtPoseRelay(Node):
    def __init__(self):
        super().__init__('ndt_pose_relay')
        in_topic = self.declare_parameter('input_topic', '/pcl_pose').value
        out_topic = self.declare_parameter('output_topic', '/pcl_pose_odom').value
        self.out_frame = self.declare_parameter('output_frame', 'odom').value

        # Default QoS (reliable, volatile, depth 10): compatible with the
        # localizer's reliable+transient_local /pcl_pose publisher, and matches
        # robot_localization's default subscription on the output topic.
        self.pub = self.create_publisher(PoseWithCovarianceStamped, out_topic, 10)
        self.sub = self.create_subscription(
            PoseWithCovarianceStamped, in_topic, self._relay, 10)
        self.get_logger().info(
            f"relaying {in_topic} (map) -> {out_topic} (frame_id={self.out_frame})")

    def _relay(self, msg):
        # Preserve stamp, pose and covariance; only rewrite the frame.
        msg.header.frame_id = self.out_frame
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = NdtPoseRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
