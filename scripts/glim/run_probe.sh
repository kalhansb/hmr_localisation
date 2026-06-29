#!/usr/bin/env bash
# Bring up GLIM, play a short slice of the bag, probe its published cloud topics
# (frame_id, per-frame size) and the TF tree, then stop GLIM (script-internal
# kill of its own child PID -- no pkill).
set -e
source /opt/ros/jazzy/setup.bash
source /root/ros2_ws/install/setup.bash
cd /ws
BAG="bags/2026_06_19_18_19_06__kalhan-map-test-2_"

GLIMPID=""
cleanup() { [ -n "$GLIMPID" ] && kill "$GLIMPID" 2>/dev/null || true; }
trap cleanup EXIT INT TERM

ros2 run glim_ros glim_rosnode --ros-args \
  -p config_path:=/ws/glim_config -p use_sim_time:=true > /tmp/glim_probe.log 2>&1 &
GLIMPID=$!
echo "glim pid=$GLIMPID ; waiting for SLAM modules..."
for i in $(seq 1 40); do
  grep -qaiE "critical|failed to load .*module" /tmp/glim_probe.log && { echo "GLIM init error:"; tail -20 /tmp/glim_probe.log; exit 1; }
  kill -0 $GLIMPID 2>/dev/null || { echo "GLIM died:"; tail -20 /tmp/glim_probe.log; exit 1; }
  grep -qaiE "global_mapping|sub_mapping|odometry_estimation" /tmp/glim_probe.log && break
  sleep 1
done
sleep 2; echo "GLIM up; playing 30s slice..."

# play a short slice (rate 0.5) in background so glim builds a few frames
ros2 bag play "$BAG" \
  --topics /ouster/points /imu/data --clock \
  --qos-profile-overrides-path config/ouster_reliable_qos.yaml \
  --read-ahead-queue-size 2000 --rate 0.5 --playback-duration 30 > /tmp/bag_probe.log 2>&1 &
BAGPID=$!
sleep 8   # let a few aligned frames accumulate

python3 -u /ws/scripts/glim/probe_glim_topics.py

wait $BAGPID 2>/dev/null || true
echo "probe done."
