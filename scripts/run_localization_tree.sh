#!/usr/bin/env bash
# Real-time localization with the full REP-105 TF tree:
#   map ──(NDT vs gt_map, this localizer)──> odom
#   odom ──(robot_localization EKF: NDT pose + /imu/data gyro)──> base_link
#   base_link ──(static extrinsics from the bag /tf_static)──> os_lidar, imu
#
# Runs at live playback rate 1.0 (config/gt_ouster_ndt_tree_realtime.yaml: 16 NDT
# threads, reject gate off). The EKF supplies a smooth, continuous, high-rate
# odom->base_link (what Nav2's local costmap + controller need) while map->odom carries
# the discrete NDT corrections -- see the README and scripts/test_ekf_smoothing.sh.
#
# Usage (HOST):
#   docker compose up -d
#   docker compose exec ros bash /ws/scripts/run_localization_tree.sh [playback_duration_s]
# Output -> /ws/output/path.csv  (the map->base_link trajectory).
set -e
source /opt/ros/jazzy/setup.bash
source /ws/install/setup.bash
cd /ws
mkdir -p output
# Route the multi-MB Ouster clouds over shared memory (else they throttle to ~0.1 Hz
# over UDP loopback and the localizer sees ~8 s gaps between scans).
export FASTRTPS_DEFAULT_PROFILES_FILE=/ws/config/fastdds_shm.xml

DUR="${1:-}"
DUR_ARG=""
[ -n "$DUR" ] && DUR_ARG="--playback-duration $DUR"

PIDS=()
cleanup() {
  kill "${PIDS[@]}" 2>/dev/null || true
  pkill -f ndt_pose_relay 2>/dev/null || true
  pkill -f ekf_node 2>/dev/null || true
}
trap cleanup EXIT INT TERM

# 1) odom -> base_link : robot_localization EKF (NDT pose restamped to odom + IMU gyro).
#    Broadcasts a smooth 50 Hz odom->base_link and publishes nav_msgs/Odometry.
ros2 launch /ws/launch/ekf_odom.launch.py use_sim_time:=true > /tmp/ekf_odom.log 2>&1 &
PIDS+=($!)

# 2) localizer, Mode B (map->odom) real-time, base frame base_link, IMU preintegration.
#    The launch's static publishers emit base_link->os_lidar and base_link->imu.
ros2 launch lidar_localization_ros2 lidar_localization.launch.py \
  localization_param_dir:=/ws/config/gt_ouster_ndt_tree_realtime.yaml \
  cloud_topic:=/ouster/points imu_topic:=/imu/data use_sim_time:=true \
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
echo "active. playing bag at rate 1.0 ${DUR:+(first ${DUR}s)}..."

# 3) best-effort TF-tree check ~12 s into playback (proves map -> base_link resolves)
( sleep 12
  echo "--- map -> base_link (sampled during playback) ---" > /tmp/tf_chain.log
  timeout 5 ros2 run tf2_ros tf2_echo map base_link \
    --ros-args -p use_sim_time:=true >> /tmp/tf_chain.log 2>&1 || true ) &
PIDS+=($!)

# 4) play the bag. Force /ouster/points RELIABLE so the localizer's reliable cloud
#    subscriber receives the scans (the bag recorded it BEST_EFFORT). The TF tree
#    comes from our publishers, not the bag's /tf.
ros2 bag play bags/2026_06_19_18_19_06__kalhan-map-test-2_ \
  --topics /ouster/points /imu/data --clock --rate 1.0 $DUR_ARG \
  --qos-profile-overrides-path /ws/config/ouster_reliable_qos.yaml

# 5) dump the latched /path trajectory (map frame) to CSV
python3 /ws/scripts/fetch_path.py /ws/output/path.csv

echo "good_scans=$(grep -ac 'fitness score:' /tmp/loc_tree.log)"
echo "--- TF tree (map -> base_link) ---"; cat /tmp/tf_chain.log 2>/dev/null || true
echo "done. trajectory -> output/path.csv"
