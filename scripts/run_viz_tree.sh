#!/usr/bin/env bash
# Live RViz: localization with the PROPER REP-105 TF tree (map->odom->base_link)
# and the robot_localization EKF odom layer -- WITHOUT scovox mapping.
# (= run_loc_scovox_tree.sh minus the scovox node; = run_viz.sh but Mode B + EKF.)
#
#   map ──(NDT vs gt_map)──> odom ──(robot_localization EKF)──> base_link ──┬──> os_lidar
#                                                                           └──> imu
#
# On the HOST first (once per login):
#   xhost +local:
#   docker compose up -d
#   docker compose exec -e DISPLAY=:1 ros bash /ws/scripts/run_viz_tree.sh [duration_s] [rate]
#
# RViz (config/localization.rviz, fixed frame = map) shows: green = GT map
# (by height), red = live Ouster scan placed in the map frame via the full TF
# chain, green line = /path, yellow = current /pcl_pose.
set -e
source /opt/ros/jazzy/setup.bash
source /ws/install/setup.bash
cd /ws
: "${DISPLAY:=:1}"
export DISPLAY LIBGL_ALWAYS_SOFTWARE=1   # software GL (no GPU passthrough)

DUR="${1:-}"
RATE="${2:-1.0}"
CONFIG="${3:-/ws/config/gt_ouster_ndt_tree.yaml}"   # 3rd arg: localizer param YAML
PREINT="${4:-false}"                                 # 4th arg: IMU preintegration on/off
DUR_ARG=""
[ -n "$DUR" ] && DUR_ARG="--playback-duration $DUR"
echo "config=$CONFIG  preint=$PREINT"

PIDS=()
cleanup() {
  kill "${PIDS[@]}" 2>/dev/null || true
  pkill -f ndt_pose_relay 2>/dev/null || true
  pkill -f ekf_node 2>/dev/null || true
}
trap cleanup EXIT INT TERM

# 1) odom -> base_link : robot_localization EKF (NDT pose + IMU). See ekf_odom.yaml.
ros2 launch /ws/launch/ekf_odom.launch.py use_sim_time:=true > /tmp/ekf_odom.log 2>&1 &
PIDS+=($!)

# 2) localizer, Mode B (map->odom), base frame base_link. IMU preintegration OFF
#    (matches run_loc_scovox_tree.sh); the launch's static publishers emit
#    base_link->os_lidar and base_link->imu to complete the tree.
ros2 launch lidar_localization_ros2 lidar_localization.launch.py \
  localization_param_dir:=$CONFIG \
  cloud_topic:=/ouster/points imu_topic:=/imu/data use_sim_time:=true \
  global_frame_id:=map odom_frame_id:=odom base_frame_id:=base_link \
  use_imu_preintegration:=$PREINT imu_preintegration_use_base_frame_transform:=$PREINT \
  publish_lidar_tf:=true lidar_frame_id:=os_lidar \
  lidar_tf_x:=0.1105 lidar_tf_y:=0.0 lidar_tf_z:=0.404 lidar_tf_yaw:=3.14159265 \
  publish_imu_tf:=true imu_frame_id:=imu \
  imu_tf_x:=0.062 imu_tf_y:=0.0 imu_tf_z:=0.015 imu_tf_yaw:=1.5707963 \
  > /tmp/loc.log 2>&1 &
PIDS+=($!)
echo "localizer pid=${PIDS[-1]} ; waiting for map load + activation..."
until grep -aq "Activating end" /tmp/loc.log; do sleep 1; done
echo "localizer active; map published."

# 3) rviz
ros2 run rviz2 rviz2 -d /ws/config/localization.rviz > /tmp/rviz.log 2>&1 &
PIDS+=($!)
sleep 5

# 4) play the bag (/ouster/points forced RELIABLE for RViz). Ctrl-C to stop early.
echo "playing bag ${DUR:+(first ${DUR}s) }at rate ${RATE}... watch RViz."
ros2 bag play bags/2026_06_19_18_19_06__kalhan-map-test-2_ \
  --topics /ouster/points /imu/data --clock --rate "$RATE" \
  --qos-profile-overrides-path /ws/config/ouster_reliable_qos.yaml $DUR_ARG

echo "bag finished. RViz still live; Ctrl-C to stop."
wait
