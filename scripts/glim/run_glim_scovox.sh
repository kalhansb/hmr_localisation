#!/usr/bin/env bash
# GLIM LiDAR-IMU SLAM (koide3/glim_ros2, CUDA) + SCovox occupancy mapping on the
# recorded Ouster+IMU bag, all in ONE ROS 2 Jazzy container.
#
#   /ouster/points + /imu/data ──(GLIM SLAM)──> TF: map ─> odom ─> imu ─> os_lidar
#                                               + /glim_ros/points (DESKEWED cloud)
#   SCovox integrates GLIM's DESKEWED /glim_ros/points (frame imu) in the `odom`
#   frame (base_frame=imu), building the occupancy map online. scovox does NOT
#   deskew -- GLIM does; feeding raw /ouster/points instead gives a 7 m vertical
#   smear (see output/feed_cmp + config/scovox_lidar_glim.yaml header).
#
# Unlike the NDT/ICP scripts, GLIM does NOT localize against the prior gt_map --
# it builds its OWN map+trajectory from scratch, and SCovox maps on top of that
# pose. So no gt_map/PCD is needed here.
#
# Frames/extrinsics come entirely from GLIM (glim_config/), so we replay ONLY
# /ouster/points + /imu/data (NOT the bag's /tf,/tf_static) to avoid a
# duplicate-parent TF conflict with GLIM's own tree.
#
# Usage (on the HOST first, once):
#   xhost +local:            # only needed if you later re-enable the GLIM GUI
#   docker compose up -d glim
#   docker compose exec glim bash /ws/scripts/glim/run_glim_scovox.sh [duration_s] [rate]
#
# duration_s : optional bag playback length in seconds (default: full ~500 s bag)
# rate       : optional playback rate (default 1.0; lower it if scovox logs TF
#              extrapolation warnings, i.e. GLIM lagging behind playback)
# Outputs: /ws/output/path_glim.csv (trajectory), /ws/output/glim_map.pcd (GLIM
# global map). TF check -> /tmp/tf_chain_glim.log ; logs -> /tmp/glim.log, /tmp/scovox.log
set -e
source /opt/ros/jazzy/setup.bash
source /root/ros2_ws/install/setup.bash          # prebuilt GLIM (glim, glim_ros)
cd /ws

DUR="${1:-}"
RATE="${2:-1.0}"
BAG="bags/2026_06_19_18_19_06__kalhan-map-test-2_"
mkdir -p /ws/output

# 0) Build SCovox once into a dedicated install_glim/ (keeps the Jazzy-desktop
#    build in /scovox/install untouched). Mirrors install/ vs install_humble/.
if [ ! -f /scovox/install_glim/setup.bash ]; then
  echo "building SCovox (first run) into /scovox/install_glim ..."
  ( cd /scovox
    rosdep install --from-paths src --ignore-src -y --rosdistro jazzy || true
    colcon build --build-base build_glim --install-base install_glim \
      --packages-select scovox_msgs scovox_core scovox_mapping \
      --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
  ) || { echo "SCovox build failed -- see output above"; exit 1; }
fi
source /scovox/install_glim/setup.bash

PIDS=()
cleanup() { kill "${PIDS[@]}" 2>/dev/null || true; }
trap cleanup EXIT INT TERM

# 1) GLIM SLAM, headless (libstandard_viewer.so disabled in glim_config/config_ros.json).
#    Loads our tuned config dir (topics /ouster/points + /imu/data, T_lidar_imu,
#    GPU sub-configs). librviz_viewer.so broadcasts the TF + /glim_ros topics.
ros2 run glim_ros glim_rosnode --ros-args \
  -p config_path:=/ws/glim_config \
  -p use_sim_time:=true > /tmp/glim.log 2>&1 &
PIDS+=($!); GLIM=$!
echo "glim pid=$GLIM ; loading SLAM modules..."
for i in $(seq 1 40); do
  if grep -qaiE "critical|failed to load .*module" /tmp/glim.log; then
    echo "GLIM failed to initialize:"; tail -25 /tmp/glim.log; exit 1
  fi
  kill -0 $GLIM 2>/dev/null || { echo "GLIM died during init:"; tail -25 /tmp/glim.log; exit 1; }
  # once the global-mapping module is loaded GLIM is subscribed and ready
  grep -qaiE "global_mapping|sub_mapping|odometry_estimation" /tmp/glim.log && break
  sleep 1
done
sleep 2
kill -0 $GLIM 2>/dev/null || { echo "GLIM exited:"; tail -25 /tmp/glim.log; exit 1; }
echo "GLIM up."

# 2) SCovox occupancy mapping consuming GLIM's odom->os_lidar TF.
ros2 launch scovox_mapping lidar_mapping.launch.py \
  params_file:=/ws/config/scovox_lidar_glim.yaml \
  pointcloud_topic:=/glim_ros/points use_sim_time:=true > /tmp/scovox.log 2>&1 &
PIDS+=($!)
echo "scovox pid=${PIDS[-1]}"

# 3) recorders: GLIM trajectory (CSV) + GLIM global map (PCD, saved on SIGINT).
python3 /ws/scripts/glim/record_glim_pose.py /ws/output/path_glim.csv > /tmp/glim_pose.log 2>&1 &
PIDS+=($!); REC=$!
python3 /ws/scripts/glim/save_glim_map.py /ws/output/glim_map.pcd > /tmp/glim_map.log 2>&1 &
PIDS+=($!); MAP=$!
sleep 2

# 4) TF sanity check a few seconds into playback (sim clock). map->os_lidar
#    resolving end-to-end means GLIM's tree is connected for SCovox.
( sleep 15
  : > /tmp/tf_chain_glim.log
  for pair in "map odom" "odom os_lidar" "map os_lidar"; do
    echo "--- $pair ---" >> /tmp/tf_chain_glim.log
    timeout 4 ros2 run tf2_ros tf2_echo $pair --ros-args -p use_sim_time:=true \
      >> /tmp/tf_chain_glim.log 2>&1 || true
  done ) &
PIDS+=($!)

# 5) play the bag (sim clock; /ouster/points forced RELIABLE for scovox -- GLIM's
#    best-effort sub still accepts a reliable publisher). Only the two sensor
#    topics, so GLIM owns the whole TF tree.
echo "playing bag ${DUR:+(first ${DUR}s) }at rate ${RATE}..."
PLAY=(ros2 bag play "$BAG"
  --topics /ouster/points /imu/data --clock
  --qos-profile-overrides-path config/ouster_reliable_qos.yaml
  --read-ahead-queue-size 2000
  --rate "$RATE")
if [ -n "$DUR" ]; then timeout "${DUR}s" "${PLAY[@]}" || true; else "${PLAY[@]}"; fi

# 6) let GLIM finish global optimization + flush the final global map, then save.
echo "bag done; letting GLIM flush final global map (12s)..."
sleep 12
# capture the final SCovox occupancy map while scovox is still alive (in GLIM mode
# it is in the `odom` integration frame). NOTE: scovox's full-map republish is a
# SIM-TIME timer; once the bag's /clock stops it FREEZES, so a plain subscriber
# gets nothing. salvage_capture_seq publishes an advancing monotonic /clock to
# unfreeze it, and SALVAGE_RELIABLE=1 reliably reassembles large maps.
echo "capturing final SCovox occupancy map -> output/scovox_map.npy ..."
LAST=$(tail -1 /ws/output/path_glim.csv 2>/dev/null | cut -d, -f1)
BASE=$(python3 -c "print(float('${LAST:-1781893646}') + 300.0)")
SALVAGE_RELIABLE=1 SALVAGE_TIMEOUT=120 \
  python3 /ws/scripts/glim/salvage_capture_seq.py "$BASE" /ws/output scovox_node > /tmp/scovox_capture.log 2>&1 || true
mv /ws/output/scovox_node.npy /ws/output/scovox_map.npy 2>/dev/null || true
tail -2 /tmp/scovox_capture.log 2>/dev/null || true
kill -INT $MAP 2>/dev/null || true   # save_glim_map writes the latest /glim_ros/map on SIGINT
sleep 4
kill $REC 2>/dev/null || true
sleep 1

echo "poses=$(($(wc -l < /ws/output/path_glim.csv 2>/dev/null || echo 1) - 1))"
echo "--- TF tree (map->odom, odom->os_lidar, map->os_lidar) ---"
cat /tmp/tf_chain_glim.log 2>/dev/null || true
echo "GLIM log: /tmp/glim.log   SCovox log: /tmp/scovox.log   map save: /tmp/glim_map.log"
echo "outputs: path_glim.csv (GLIM traj), glim_map.pcd (GLIM map), scovox_map.npy (SCovox occupancy)"
echo "plot GLIM map+traj: python3 scripts/glim/plot_glim.py output/glim_map.pcd output/path_glim.csv output/eval_glim.png"
