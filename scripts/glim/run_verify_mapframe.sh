#!/usr/bin/env bash
# Bring up GLIM + scovox (production /glim_ros/points wiring), play a short slice,
# and run verify_map_frame.py to confirm the scovox map is transformable into the
# map frame at its own stamp (the map-frame fix). Stops via script-internal kill.
set -e
source /opt/ros/jazzy/setup.bash
source /root/ros2_ws/install/setup.bash
source /scovox/install_glim/setup.bash
cd /ws
BAG="bags/2026_06_19_18_19_06__kalhan-map-test-2_"

PIDS=(); cleanup() { kill "${PIDS[@]}" 2>/dev/null || true; }
trap cleanup EXIT INT TERM

ros2 run glim_ros glim_rosnode --ros-args \
  -p config_path:=/ws/glim_config -p use_sim_time:=true > /tmp/glim_v.log 2>&1 &
PIDS+=($!); GLIM=$!
for i in $(seq 1 40); do
  kill -0 $GLIM 2>/dev/null || { echo "GLIM died:"; tail -15 /tmp/glim_v.log; exit 1; }
  grep -qaiE "global_mapping|odometry_estimation" /tmp/glim_v.log && break; sleep 1
done
sleep 2; echo "GLIM up."

ros2 launch scovox_mapping lidar_mapping.launch.py \
  params_file:=/ws/config/scovox_lidar_glim.yaml \
  pointcloud_topic:=/glim_ros/points use_sim_time:=true > /tmp/scovox_v.log 2>&1 &
PIDS+=($!)
sleep 3

ros2 bag play "$BAG" --topics /ouster/points /imu/data --clock \
  --qos-profile-overrides-path config/ouster_reliable_qos.yaml \
  --read-ahead-queue-size 2000 --rate 1.0 --playback-duration 35 > /tmp/bag_v.log 2>&1 &
sleep 10   # let scovox start republishing its map

echo "==== verifying map-frame transform ===="
python3 -u /ws/scripts/glim/verify_map_frame.py
echo "verify done."
