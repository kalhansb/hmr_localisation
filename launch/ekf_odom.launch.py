"""Continuous `odom -> base_link` estimator for the Mode B (map->odom->base_link)
TF tree -- the robot_localization replacement for the identity
static_transform_publisher in run_localization_tree.sh / run_loc_scovox_tree.sh.

Brings up two processes:
  * ndt_pose_relay.py    -- restamp /pcl_pose (map) -> /pcl_pose_odom (odom),
                            so the odom EKF can fuse the NDT pose without the
                            circular map->odom TF lookup (see the script header).
  * robot_localization ekf_node (name: ekf_filter_node) -- fuse the relayed NDT
                            pose + /imu/data angular velocity and broadcast
                            odom -> base_link.

Invoked by path (no colcon package needed), matching multi_robot_localization:
  ros2 launch /ws/launch/ekf_odom.launch.py use_sim_time:=true
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    ekf_params = LaunchConfiguration('ekf_params')
    pose_topic = LaunchConfiguration('pose_topic')
    relayed_pose_topic = LaunchConfiguration('relayed_pose_topic')
    relay_script = LaunchConfiguration('relay_script')

    # Restamp the NDT global pose (map) into the odom frame for the EKF. The relay
    # copies the message stamp verbatim, so it needs no clock of its own.
    relay = ExecuteProcess(
        cmd=[
            'python3', relay_script,
            '--ros-args',
            '-p', ['input_topic:=', pose_topic],
            '-p', ['output_topic:=', relayed_pose_topic],
            '-p', 'output_frame:=odom',
        ],
        output='screen')

    # EKF: world_frame=odom -> broadcasts odom -> base_link. pose0/imu0 topics are
    # set in ekf_params (config/ekf_odom.yaml); pose0 must equal relayed_pose_topic.
    ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            ekf_params,
            {'use_sim_time': ParameterValue(use_sim_time, value_type=bool)},
        ])

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('ekf_params', default_value='/ws/config/ekf_odom.yaml'),
        DeclareLaunchArgument('pose_topic', default_value='/pcl_pose'),
        DeclareLaunchArgument('relayed_pose_topic', default_value='/pcl_pose_odom'),
        DeclareLaunchArgument('relay_script', default_value='/ws/scripts/ndt_pose_relay.py'),
        relay,
        ekf,
    ])
