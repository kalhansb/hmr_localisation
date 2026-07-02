#!/usr/bin/env bash
# LIVE (real-robot) localization tree — run_localization_tree.sh minus the bag:
#   map ──(NDT vs gt_map)──> odom ──(EKF: NDT pose + IMU gyro)──> base_link ──> {os_lidar, imu}
#
# Expects the Ouster driver + IMU publishing /ouster/points + /imu/data on the
# live DDS graph. NOTE: the localizer subscribes /ouster/points RELIABLE —
# configure the driver to publish RELIABLE (a best-effort publisher will not
# match; the bag runs needed the same override).
#
# Usage (HOST):
#   docker compose up -d
#   docker compose exec -e ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET ros \
#     bash /ws/scripts/run_localization_live.sh
# (compose.yaml pins discovery to loopback; SUBNET reaches sensors/robots on
# the network.) Stop with Ctrl-C, or `docker compose stop` from the host.
set -e
source /opt/ros/jazzy/setup.bash
source /ws/install/setup.bash
cd /ws
# SHM only engages for same-host peers; remote sensors still use UDP.
export FASTRTPS_DEFAULT_PROFILES_FILE=/ws/config/fastdds_shm.xml

PIDS=()
cleanup() {
  kill "${PIDS[@]}" 2>/dev/null || true
  pkill -f ndt_pose_relay 2>/dev/null || true
  pkill -f ekf_node 2>/dev/null || true
}
trap cleanup EXIT INT TERM

# 1) odom -> base_link : robot_localization EKF (config/ekf_odom.yaml).
ros2 launch /ws/launch/ekf_odom.launch.py use_sim_time:=false > /tmp/ekf_odom.log 2>&1 &
PIDS+=($!)

# 2) map -> odom : NDT localizer (config/gt_ouster_ndt_tree_realtime.yaml) +
#    static base_link -> {os_lidar, imu} extrinsics. The extrinsics below are
#    the map-test robot's mounting — re-measure them for a different platform.
ros2 launch lidar_localization_ros2 lidar_localization.launch.py \
  localization_param_dir:=/ws/config/gt_ouster_ndt_tree_realtime.yaml \
  cloud_topic:=/ouster/points imu_topic:=/imu/data use_sim_time:=false \
  global_frame_id:=map odom_frame_id:=odom base_frame_id:=base_link \
  use_imu_preintegration:=true imu_preintegration_use_base_frame_transform:=true \
  publish_lidar_tf:=true lidar_frame_id:=os_lidar \
  lidar_tf_x:=0.1105 lidar_tf_y:=0.0 lidar_tf_z:=0.404 lidar_tf_yaw:=3.14159265 \
  publish_imu_tf:=true imu_frame_id:=imu \
  imu_tf_x:=0.062 imu_tf_y:=0.0 imu_tf_z:=0.015 imu_tf_yaw:=1.5707963 \
  > /tmp/loc_tree.log 2>&1 &
PIDS+=($!)
echo "localizer pid=${PIDS[-1]} ; waiting for map load + activation..."
until grep -aq "Activating end" /tmp/loc_tree.log; do sleep 1; done

echo "active. TF tree: map -> odom -> base_link -> {os_lidar, imu}"
echo "--- map -> base_link (sampled; needs live scans to resolve) ---"
timeout 10 ros2 run tf2_ros tf2_echo map base_link 2>&1 | head -12 || true
echo "running. logs: /tmp/loc_tree.log /tmp/ekf_odom.log ; Ctrl-C to stop."
wait
