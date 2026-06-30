#!/usr/bin/env bash
# A/B test: does robot_localization + IMU smooth the trajectory?
#
# Runs the Mode B real-time localizer (map -> odom -> base_link, rate 1.0) and
# records the FULL map -> base_link TF at 50 Hz. The odom -> base_link layer is
# either:
#   ekf    -> robot_localization EKF (NDT pose + /imu/data gyro)  [launch/ekf_odom.launch.py]
#   noekf  -> static identity publisher (no smoothing) -- the baseline
# Everything else (Mode B NDT, base_link extrinsics, bag, rate) is identical, so the
# only difference between the two recordings is the EKF/IMU smoothing layer.
#
# Usage (in the hmr_loc container):
#   bash /ws/scripts/test_ekf_smoothing.sh <ekf|noekf> [duration_s]
# Output: /ws/output/tf_<mode>.csv  (then analyze with scripts/analysis/analyze_smoothing.py)
set -e
MODE="${1:?usage: test_ekf_smoothing.sh <ekf|noekf> [duration_s]}"
DUR="${2:-90}"
source /opt/ros/jazzy/setup.bash
source /ws/install/setup.bash
cd /ws
mkdir -p output
# Route the multi-MB Ouster clouds over shared memory (else they throttle to ~0.1 Hz
# over UDP loopback and the localizer sees ~8 s gaps between scans). All child
# processes inherit this, so the bag player and localizer share the SHM segment.
export FASTRTPS_DEFAULT_PROFILES_FILE=/ws/config/fastdds_shm.xml

PIDS=()
cleanup() {
  kill "${PIDS[@]}" 2>/dev/null || true
  pkill -f ndt_pose_relay 2>/dev/null || true
  pkill -f ekf_node 2>/dev/null || true
  pkill -f tf_chain_recorder 2>/dev/null || true
  pkill -f "static_transform_publisher.*base_link" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

# 1) odom -> base_link layer
if [ "$MODE" = "ekf" ]; then
  ros2 launch /ws/launch/ekf_odom.launch.py use_sim_time:=true > /tmp/odom_$MODE.log 2>&1 &
  PIDS+=($!)
elif [ "$MODE" = "noekf" ]; then
  ros2 run tf2_ros static_transform_publisher \
    --x 0 --y 0 --z 0 --qx 0 --qy 0 --qz 0 --qw 1 \
    --frame-id odom --child-frame-id base_link > /tmp/odom_$MODE.log 2>&1 &
  PIDS+=($!)
else
  echo "MODE must be 'ekf' or 'noekf'"; exit 1
fi

# 2) localizer, Mode B real-time (16 threads, reject off), base_link static extrinsics
ros2 launch lidar_localization_ros2 lidar_localization.launch.py \
  localization_param_dir:=/ws/config/gt_ouster_ndt_tree_realtime.yaml \
  cloud_topic:=/ouster/points imu_topic:=/imu/data use_sim_time:=true \
  global_frame_id:=map odom_frame_id:=odom base_frame_id:=base_link \
  use_imu_preintegration:=true imu_preintegration_use_base_frame_transform:=true \
  publish_lidar_tf:=true lidar_frame_id:=os_lidar \
  lidar_tf_x:=0.1105 lidar_tf_y:=0.0 lidar_tf_z:=0.404 lidar_tf_yaw:=3.14159265 \
  publish_imu_tf:=true imu_frame_id:=imu \
  imu_tf_x:=0.062 imu_tf_y:=0.0 imu_tf_z:=0.015 imu_tf_yaw:=1.5707963 \
  > /tmp/loc_$MODE.log 2>&1 &
PIDS+=($!)
echo "[$MODE] waiting for map load + activation..."
until grep -aq "Activating end" /tmp/loc_$MODE.log; do sleep 1; done
echo "[$MODE] active. recording map->base_link @ 50 Hz; playing ${DUR}s @ rate 1.0..."

# 3) TF recorders (50 Hz, sim clock): the GLOBAL pose (map->base_link) AND the
#    LOCAL odometry edge (odom->base_link) where the EKF smoothing actually lives.
python3 /ws/scripts/record_tf_chain.py /ws/output/tf_${MODE}_map.csv map base_link 50.0 \
  --ros-args -p use_sim_time:=true > /tmp/rec_${MODE}_map.log 2>&1 &
PIDS+=($!)
python3 /ws/scripts/record_tf_chain.py /ws/output/tf_${MODE}_odom.csv odom base_link 50.0 \
  --ros-args -p use_sim_time:=true > /tmp/rec_${MODE}_odom.log 2>&1 &
PIDS+=($!)
sleep 2

# 4) play the bag (only the two topics the localizer needs; our static pubs own the
#    tree). Force /ouster/points RELIABLE so the localizer's reliable cloud
#    subscriber actually receives the scans (the bag recorded it BEST_EFFORT).
ros2 bag play bags/2026_06_19_18_19_06__kalhan-map-test-2_ \
  --topics /ouster/points /imu/data --clock --rate 1.0 --playback-duration "$DUR" \
  --qos-profile-overrides-path /ws/config/ouster_reliable_qos.yaml

sleep 1
echo "[$MODE] done. good_scans=$(grep -ac 'fitness score:' /tmp/loc_$MODE.log) rejects=$(grep -ac 'fitness score is over' /tmp/loc_$MODE.log) map_samples=$(($(wc -l < /ws/output/tf_${MODE}_map.csv) - 1)) odom_samples=$(($(wc -l < /ws/output/tf_${MODE}_odom.csv) - 1))"
