# =======================================================================
# Two-robot NDT localization for the coop multi-robot map-merge experiment.
# =======================================================================
# Brings up, for EACH robot, an NDT localizer against the SHARED gt_map,
# namespaced so two instances coexist in ONE ROS graph:
#
#   map --(NDT vs gt_map, lidar_localization_ros2, Mode B)--> <robot>/odom
#   <robot>/odom --(identity static TF)---------------------> <robot base>
#   <robot base> --(the bag's /tf_static)------------------> <sensor>
#
# Both robots localize against the same gt_map, so they share one map frame;
# frame names are already disjoint, so only the NDT node and /pcl_pose are
# namespaced. odom->base is an identity static (noekf).
# (notes: coop-loc-shared-map-noekf)
#
# YOU still supply, in the same graph:
#   * each bag's sensor stream + /tf_static (play the bags — see the runbook).
#     curt's /ouster/points is best_effort; override it to reliable on playback.
#   * the scovox mappers + merger (scovox_mapping dscovox_multi_robot.launch.py).
#
# Run (inside the hmr_localisation `ros` container):
#   source /opt/ros/jazzy/setup.bash; source /ws/install/setup.bash
#   export FASTRTPS_DEFAULT_PROFILES_FILE=/ws/config/fastdds_shm.xml
#   ros2 launch /ws/launch/coop_multi_robot_localization.launch.py use_sim_time:=true
#
# Wait until BOTH nodes log "Activating end" before playing the bags.
# =======================================================================
# Moved comments: docs/hmr_localisation_code_notes.md
import launch
import launch_ros
import lifecycle_msgs.msg
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node
from launch_ros.parameter_descriptions import ParameterValue


# Per-robot wiring discovered from the two coop bags.
_ROBOTS = [
    {"name": "bunker", "cloud": "/hesai/points",  "imu": "/imu/data",
     "base": "base_link",      "odom": "odom"},
    {"name": "curt",   "cloud": "/ouster/points", "imu": "/curt/imu/data",
     "base": "base_link_curt", "odom": "odom_curt"},
]


def _robot_stack(r, use_sim_time, ndt_param):
    ndt = LifecycleNode(
        name="lidar_localization",
        namespace="/" + r["name"],
        package="lidar_localization_ros2",
        executable="lidar_localization_node",
        parameters=[
            ndt_param,
            {
                "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                "global_frame_id": "map",
                "odom_frame_id": r["odom"],
                "base_frame_id": r["base"],
                "use_imu_preintegration": True,
                "imu_preintegration_use_base_frame_transform": True,
            },
        ],
        # The component subscribes to RELATIVE "cloud"/"imu"/"twist" (-> /<robot>/cloud
        # under this namespace), so the remap keys must be relative; the targets are
        # the absolute bag-global topics. /twist is unused.
        remappings=[("cloud", r["cloud"]), ("imu", r["imu"]),
                    ("twist", "/" + r["name"] + "/twist")],
        output="screen",
    )

    # Lifecycle auto-transition: unconfigured -> (configure) -> inactive -> (activate).
    to_configure = EmitEvent(event=launch_ros.events.lifecycle.ChangeState(
        lifecycle_node_matcher=launch.events.matches_action(ndt),
        transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE))
    on_unconfigured = RegisterEventHandler(launch_ros.event_handlers.OnStateTransition(
        target_lifecycle_node=ndt, goal_state="unconfigured",
        entities=[EmitEvent(event=launch_ros.events.lifecycle.ChangeState(
            lifecycle_node_matcher=launch.events.matches_action(ndt),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE))]))
    on_inactive = RegisterEventHandler(launch_ros.event_handlers.OnStateTransition(
        target_lifecycle_node=ndt, start_state="configuring", goal_state="inactive",
        entities=[EmitEvent(event=launch_ros.events.lifecycle.ChangeState(
            lifecycle_node_matcher=launch.events.matches_action(ndt),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE))]))

    # Identity <robot>/odom -> <robot base> ('noekf'): map -> odom carries the pose.
    odom_to_base = Node(
        package="tf2_ros", executable="static_transform_publisher",
        name=f"odom_to_base_{r['name']}",
        arguments=["--x", "0", "--y", "0", "--z", "0",
                   "--qx", "0", "--qy", "0", "--qz", "0", "--qw", "1",
                   "--frame-id", r["odom"], "--child-frame-id", r["base"]])

    return [ndt, on_unconfigured, on_inactive, odom_to_base, to_configure]


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    ndt_param = LaunchConfiguration("ndt_param")

    actions = [
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("ndt_param",
                              default_value="/ws/config/gt_ouster_ndt_tree_fused.yaml",
                              description="Shared NDT param YAML (frames overridden per robot)."),
    ]
    for r in _ROBOTS:
        actions += _robot_stack(r, use_sim_time, ndt_param)
    return LaunchDescription(actions)
