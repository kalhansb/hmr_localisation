"""Bring up one namespaced lidar_localization_ros2 instance per robot.

Reads a roster YAML (see config/robots.yaml). Every robot:
  * runs as a LifecycleNode in its own namespace (/<name>/...), so all of the
    node's RELATIVE topics (pcl_pose, path, cloud, map, ...) are auto-separated;
  * localizes against the shared gt_map.ply (map_path lives in the param file);
  * broadcasts a distinct  map -> <name>/os_lidar  TF edge.

Because every topic is relative in the component, namespacing is all that is
needed to run N independent localizers on one machine against one map.

Run by path (no colcon package needed):
  ros2 launch /ws/launch/multi_robot_localization.launch.py \
      robots_config:=/tmp/robots_active.yaml
"""
import yaml

import launch
import launch.actions
import launch.events
import launch_ros.actions
import launch_ros.events
import launch_ros.event_handlers

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode

import lifecycle_msgs.msg


def _build_robots(context, *_args, **_kwargs):
    robots_config = LaunchConfiguration('robots_config').perform(context)
    param_file = LaunchConfiguration('localization_param_dir').perform(context)

    with open(robots_config) as f:
        roster = yaml.safe_load(f)['robots']

    entities = []
    for rb in roster:
        name = rb['name']
        points_topic = rb['points_topic']
        ip = [float(v) for v in rb['initial_pose']]

        node = LifecycleNode(
            name='lidar_localization',
            namespace=name,
            package='lidar_localization_ros2',
            executable='lidar_localization_node',
            parameters=[
                # base parameters (NDT tuning, map_path, crop, ...) shared by all
                param_file,
                # per-robot overrides. Robots are separated by NAMESPACE, so every
                # relative topic (pcl_pose, path, cloud, ...) is auto-scoped to
                # /<name>/...  The base frame stays 'os_lidar' to MATCH the bag's
                # cloud frame_id -- otherwise the component TF-transforms the scan
                # into base_frame and drops it when no such transform exists. (The
                # resulting map->os_lidar TF is shared on /tf and is only cosmetic;
                # relative pose is taken from the namespaced /<name>/pcl_pose, and
                # RViz uses the replay which restamps to distinct robot frames.)
                {
                    'use_sim_time': False,        # timing is driven by msg stamps
                    'global_frame_id': 'map',
                    'odom_frame_id': f'{name}/odom',
                    'base_frame_id': 'os_lidar',
                    'enable_map_odom_tf': False,  # publish map -> base directly
                    'use_odom': False,
                    'use_imu': False,
                    'use_imu_preintegration': False,
                    'set_initial_pose': True,
                    'initial_pose_x': ip[0],
                    'initial_pose_y': ip[1],
                    'initial_pose_z': ip[2],
                    'initial_pose_qx': ip[3],
                    'initial_pose_qy': ip[4],
                    'initial_pose_qz': ip[5],
                    'initial_pose_qw': ip[6],
                },
            ],
            # the component subscribes to relative "cloud"; point it at this
            # robot's frame-corrected stream
            remappings=[('cloud', points_topic)],
            output='screen',
        )

        # lifecycle: configure now, then activate once it reaches 'inactive'
        configure = launch.actions.EmitEvent(
            event=launch_ros.events.lifecycle.ChangeState(
                lifecycle_node_matcher=launch.events.matches_action(node),
                transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
            ))

        activate_on_inactive = launch.actions.RegisterEventHandler(
            launch_ros.event_handlers.OnStateTransition(
                target_lifecycle_node=node,
                start_state='configuring',
                goal_state='inactive',
                entities=[
                    launch.actions.LogInfo(msg=f'[{name}] inactive -> activating'),
                    launch.actions.EmitEvent(
                        event=launch_ros.events.lifecycle.ChangeState(
                            lifecycle_node_matcher=launch.events.matches_action(node),
                            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
                        )),
                ],
            ))

        entities.extend([activate_on_inactive, node, configure])

    return entities


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'robots_config',
            default_value='/ws/config/robots.yaml',
            description='Roster YAML: one entry per robot (name, points_topic, initial_pose).'),
        DeclareLaunchArgument(
            'localization_param_dir',
            default_value='/ws/config/gt_ouster_ndt.yaml',
            description='Shared lidar_localization_ros2 parameter YAML.'),
        OpaqueFunction(function=_build_robots),
    ])
