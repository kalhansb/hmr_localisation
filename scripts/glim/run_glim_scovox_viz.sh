#!/usr/bin/env bash
# LIVE RViz: GLIM LiDAR-IMU SLAM + SCovox occupancy mapping on the bag.
#
# Same pipeline as run_glim_scovox.sh, plus an RViz window
# (config/glim_scovox.rviz) showing, as it builds:
#   - SCovox occupancy (/scovox_node/pointcloud, RGB boxes) in GLIM's map frame
#   - GLIM aligned scan (/glim_ros/aligned_points) + toggleable global map (/glim_ros/map)
#   - GLIM odometry arrows (/glim_ros/odom) + the TF tree (map->odom->imu->os_lidar)
# Fixed Frame = map (GLIM globally-optimized frame; switch to odom in RViz for the
# smooth/jump-free odometry frame).
#
# RViz needs the host X server. On the HOST first (once per login):
#   xhost +local:
#   docker compose up -d glim
#   docker compose exec glim bash /ws/scripts/glim/run_glim_scovox_viz.sh [duration_s] [rate]
#
# duration_s : optional bag length in seconds (default: full ~500 s bag)
# rate       : optional playback rate (default 0.5; GLIM GPU keeps up at 0.5)
# Uses hardware (NVIDIA) GL by default. If RViz fails to open a GL context, rerun
# with LIBGL_ALWAYS_SOFTWARE=1 set (software rendering).
set -e
source /opt/ros/jazzy/setup.bash
source /root/ros2_ws/install/setup.bash          # prebuilt GLIM
cd /ws
: "${DISPLAY:=:1}"
export DISPLAY
# GL backend. Default: render RViz on the NVIDIA GPU (hardware GL 4.6) -- without
# this the GLVND loader picks the Intel 'iris' MESA driver, which can't get a DRM
# device in the container ("Failed to query drm device" / "failed to load driver:
# iris"). If NVIDIA graphics caps aren't exposed to the container, run with
# LIBGL_ALWAYS_SOFTWARE=1 (llvmpipe software rendering -- reliable, slower):
#   docker compose exec -e LIBGL_ALWAYS_SOFTWARE=1 glim bash .../run_glim_scovox_viz.sh "" 0.5
if [ "${LIBGL_ALWAYS_SOFTWARE:-0}" = "1" ]; then
  echo "(software GL / llvmpipe)"
  unset __GLX_VENDOR_LIBRARY_NAME __NV_PRIME_RENDER_OFFLOAD
else
  export __GLX_VENDOR_LIBRARY_NAME=nvidia
  export __NV_PRIME_RENDER_OFFLOAD=1
fi

DUR="${1:-}"
RATE="${2:-0.5}"
BAG="bags/2026_06_19_18_19_06__kalhan-map-test-2_"
mkdir -p /ws/output

# Build SCovox once into install_glim/ (first run only).
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

# 1) GLIM SLAM (headless core; RViz is the viewer).
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

# 2) SCovox occupancy mapping (integrates in GLIM's map frame).
ros2 launch scovox_mapping lidar_mapping.launch.py \
  params_file:=/ws/config/scovox_lidar_glim.yaml \
  pointcloud_topic:=/glim_ros/points use_sim_time:=true > /tmp/scovox.log 2>&1 &
PIDS+=($!)
echo "scovox pid=${PIDS[-1]}"

# 3) recorders (so the viz run also writes output/path_glim.csv + glim_map.pcd).
python3 /ws/scripts/glim/record_glim_pose.py /ws/output/path_glim.csv > /tmp/glim_pose.log 2>&1 &
PIDS+=($!)
python3 /ws/scripts/glim/save_glim_map.py /ws/output/glim_map.pcd > /tmp/glim_map.log 2>&1 &
PIDS+=($!); MAP=$!

# 4) RViz.
echo "launching RViz (DISPLAY=$DISPLAY) ..."
ros2 run rviz2 rviz2 -d /ws/config/glim_scovox.rviz --ros-args -p use_sim_time:=true \
  > /tmp/rviz_glim.log 2>&1 &
PIDS+=($!)
sleep 4
echo "RViz pid=${PIDS[-1]} (if no window appeared, check /tmp/rviz_glim.log + run 'xhost +local:' on host)"

# 5) play the bag (sim clock; /ouster/points forced RELIABLE for scovox + GLIM).
echo "playing bag ${DUR:+(first ${DUR}s) }at rate ${RATE}... watch RViz."
PLAY=(ros2 bag play "$BAG"
  --topics /ouster/points /imu/data --clock
  --qos-profile-overrides-path config/ouster_reliable_qos.yaml
  --read-ahead-queue-size 2000
  --rate "$RATE")
if [ -n "$DUR" ]; then timeout "${DUR}s" "${PLAY[@]}" || true; else "${PLAY[@]}"; fi

echo "bag finished; flushing GLIM global map + capturing SCovox map..."
sleep 8
python3 /ws/scripts/scovox/capture_scovox.py > /tmp/scovox_capture.log 2>&1 || true
kill -INT $MAP 2>/dev/null || true
echo "map still live in RViz. Ctrl-C to stop."
# keep scovox alive so the map stays in RViz (PIDS[1] = scovox launch)
wait "${PIDS[1]}" 2>/dev/null || true
