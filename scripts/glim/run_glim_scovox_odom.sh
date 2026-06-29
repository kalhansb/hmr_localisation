#!/usr/bin/env bash
# EXPERIMENT A: GLIM LiDAR-IMU SLAM + SCovox integrating in GLIM's `odom` frame
# (smooth, jump-free, but NO loop closure -> drifts globally). Empirical test of
# whether odom-frame integration yields a clean map vs the smeared `map`-frame run.
#
# Outputs (suffixed _odom so they don't clobber the map-frame results):
#   /ws/output/scovox_map_odom.npy  (SCovox occupancy, ODOM frame)
#   /ws/output/glim_map_odom.pcd    (GLIM global map, MAP frame)
#   /ws/output/path_glim_odom.csv   (GLIM online-corrected pose, MAP frame)
#
# Requires config/scovox_lidar_glim.yaml with integration_frame: "odom".
# Usage: docker compose exec glim bash /ws/scripts/glim/run_glim_scovox_odom.sh [duration_s] [rate]
set -e
source /opt/ros/jazzy/setup.bash
source /root/ros2_ws/install/setup.bash
cd /ws

DUR="${1:-}"
RATE="${2:-0.5}"          # 0.5 -> GLIM GPU keeps up easily -> minimal odometry tracking error
BAG="bags/2026_06_19_18_19_06__kalhan-map-test-2_"
mkdir -p /ws/output

if [ ! -f /scovox/install_glim/setup.bash ]; then
  echo "building SCovox (first run) into /scovox/install_glim ..."
  ( cd /scovox
    rosdep install --from-paths src --ignore-src -y --rosdistro jazzy || true
    colcon build --build-base build_glim --install-base install_glim \
      --packages-select scovox_msgs scovox_core scovox_mapping \
      --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
  ) || { echo "SCovox build failed"; exit 1; }
fi
source /scovox/install_glim/setup.bash

PIDS=()
cleanup() { kill "${PIDS[@]}" 2>/dev/null || true; }
trap cleanup EXIT INT TERM

# 1) GLIM SLAM (headless).
ros2 run glim_ros glim_rosnode --ros-args \
  -p config_path:=/ws/glim_config -p use_sim_time:=true > /tmp/glim.log 2>&1 &
PIDS+=($!); GLIM=$!
echo "glim pid=$GLIM ; loading SLAM modules..."
for i in $(seq 1 40); do
  grep -qaiE "critical|failed to load .*module" /tmp/glim.log && { echo "GLIM init error:"; tail -25 /tmp/glim.log; exit 1; }
  kill -0 $GLIM 2>/dev/null || { echo "GLIM died:"; tail -25 /tmp/glim.log; exit 1; }
  grep -qaiE "global_mapping|sub_mapping|odometry_estimation" /tmp/glim.log && break
  sleep 1
done
sleep 2; echo "GLIM up."

# 2) SCovox occupancy mapping (integration_frame=odom from the yaml).
ros2 launch scovox_mapping lidar_mapping.launch.py \
  params_file:=/ws/config/scovox_lidar_glim.yaml \
  pointcloud_topic:=/glim_ros/points use_sim_time:=true > /tmp/scovox.log 2>&1 &
PIDS+=($!)
echo "scovox pid=${PIDS[-1]} (integration_frame=odom)"

# 3) recorders (GLIM trajectory + global map). The SCovox map is grabbed at the END
#    via salvage (advancing /clock) -- scovox's republish timer only fires at startup
#    when the map is empty, then stalls on the bag's /clock epoch jump, so a normal
#    subscriber sees only empty republishes during playback.
python3 /ws/scripts/glim/record_glim_pose.py /ws/output/path_glim_odom.csv > /tmp/glim_pose.log 2>&1 &
PIDS+=($!); REC=$!
python3 /ws/scripts/glim/save_glim_map.py /ws/output/glim_map_odom.pcd > /tmp/glim_map.log 2>&1 &
PIDS+=($!); MAP=$!
sleep 2

# 4) play the bag (sim clock; /ouster/points forced RELIABLE for scovox).
echo "playing bag ${DUR:+(first ${DUR}s) }at rate ${RATE}..."
PLAY=(ros2 bag play "$BAG"
  --topics /ouster/points /imu/data --clock
  --qos-profile-overrides-path config/ouster_reliable_qos.yaml
  --read-ahead-queue-size 2000
  --rate "$RATE")
if [ -n "$DUR" ]; then timeout "${DUR}s" "${PLAY[@]}" || true; else "${PLAY[@]}"; fi

# 5) flush GLIM, then SALVAGE the SCovox map WHILE scovox is still fully alive:
#    advance /clock past the bag end so scovox's republish timer fires once and
#    publishes the complete accumulated occupancy map, which salvage_capture saves.
echo "bag done; flushing (8s)..."
sleep 8
LAST=$(tail -1 /ws/output/path_glim_odom.csv 2>/dev/null | cut -d, -f1)
# base must exceed the TRUE bag-end /clock (~1781893646); GLIM poses can stop early,
# so overshoot by 300 s (harmless -- a big forward jump just fires the timer at once).
BASE=$(python3 -c "print(float('${LAST:-1781893646}') + 300.0)")
echo "salvaging SCovox map (advancing /clock from ${BASE}; scovox still alive)..."
python3 /ws/scripts/glim/salvage_capture.py /ws/output/scovox_map_odom.npy "$BASE" 2>&1 | tee /tmp/scovox_capture.log
if [ -f /ws/output/scovox_map_odom.npy ]; then
  echo "SCovox odom map captured OK."
else
  echo "AUTO-SALVAGE FAILED -- keeping scovox alive 600 s for manual salvage (container must stay up)."
  sleep 600
fi
kill -INT $MAP 2>/dev/null || true     # save_glim_map writes latest /glim_ros/map on SIGINT
sleep 4
kill $REC 2>/dev/null || true
sleep 1

echo "poses=$(($(wc -l < /ws/output/path_glim_odom.csv 2>/dev/null || echo 1) - 1))"
echo "outputs: scovox_map_odom.npy  glim_map_odom.pcd  path_glim_odom.csv"
