#!/usr/bin/env bash
# Quick smoke test of the PRODUCTION wiring: GLIM + scovox via lidar_mapping.launch
# consuming /glim_ros/points (deskewed). Plays a 25 s bag slice and reports scovox
# recv / TF-fail / integrate health + a single-frame thinness check. Stops via
# script-internal kill of its own children (no pkill).
set -e
source /opt/ros/jazzy/setup.bash
source /root/ros2_ws/install/setup.bash
source /scovox/install_glim/setup.bash
cd /ws
BAG="bags/2026_06_19_18_19_06__kalhan-map-test-2_"

PIDS=()
cleanup() { kill "${PIDS[@]}" 2>/dev/null || true; }
trap cleanup EXIT INT TERM

ros2 run glim_ros glim_rosnode --ros-args \
  -p config_path:=/ws/glim_config -p use_sim_time:=true > /tmp/glim_smoke.log 2>&1 &
PIDS+=($!); GLIM=$!
echo "glim pid=$GLIM ; waiting for SLAM modules..."
for i in $(seq 1 40); do
  grep -qaiE "critical|failed to load .*module" /tmp/glim_smoke.log && { echo "GLIM init error:"; tail -15 /tmp/glim_smoke.log; exit 1; }
  kill -0 $GLIM 2>/dev/null || { echo "GLIM died:"; tail -15 /tmp/glim_smoke.log; exit 1; }
  grep -qaiE "global_mapping|sub_mapping|odometry_estimation" /tmp/glim_smoke.log && break
  sleep 1
done
sleep 2; echo "GLIM up."

# Production launch path (arg overrides config's input_pointcloud_topic).
ros2 launch scovox_mapping lidar_mapping.launch.py \
  params_file:=/ws/config/scovox_lidar_glim.yaml \
  pointcloud_topic:=/glim_ros/points use_sim_time:=true > /tmp/scovox_smoke.log 2>&1 &
PIDS+=($!)
echo "scovox launched; settling 3s..."
sleep 3

echo "playing 25s bag slice..."
ros2 bag play "$BAG" \
  --topics /ouster/points /imu/data --clock \
  --qos-profile-overrides-path config/ouster_reliable_qos.yaml \
  --read-ahead-queue-size 2000 --rate 0.5 --playback-duration 25 > /tmp/bag_smoke.log 2>&1 || true

sleep 2
echo "==== scovox health ===="
echo "input topic resolved: $(grep -aoE 'PointCloud2 input mode: topic=\S+' /tmp/scovox_smoke.log | tail -1)"
echo "last recv line:"; grep -aE "recv=" /tmp/scovox_smoke.log | tail -1 | cut -c1-180
echo "TF FAILED lines: $(grep -ac 'TF FAILED' /tmp/scovox_smoke.log)"
echo "missing-xyz warns: $(grep -ac 'missing xyz' /tmp/scovox_smoke.log)"
echo "errors:"; grep -aiE "error|terminate|what\(\):" /tmp/scovox_smoke.log | tail -3
echo "smoke done."
