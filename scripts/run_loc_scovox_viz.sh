#!/usr/bin/env bash
# Live RViz visualization of localization + SCovox occupancy mapping.
#
# Pipeline:
#   lidar_localization_ros2  -> NDT pose of os_lidar vs gt_map.ply, broadcasts
#                               TF map->os_lidar, publishes /initial_map /path /pcl_pose
#   scovox_mapping_node      -> integrates /ouster/points in the `map` frame using
#                               that TF, publishes /scovox_node/pointcloud (occupancy)
#   rviz2                    -> GT map (faint, by height) + SCovox occupancy (boxes,
#                               RGB) + trajectory (green) + current pose (yellow)
#
# /ouster/points is republished RELIABLE (config/ouster_reliable_qos.yaml) so it
# reaches scovox's reliable subscriber as well as the localizer.
#
# On the HOST first (once per login): xhost +local:
# Then:
#   docker compose up -d
#   docker compose exec -e DISPLAY=:1 ros bash /ws/scripts/run_loc_scovox_viz.sh [duration_s] [rate]
#
# duration_s : optional bag playback length in seconds (default: full bag)
# rate       : optional playback rate (default: 1.0)
set -e
source /opt/ros/jazzy/setup.bash
source /ws/install/setup.bash
source /scovox/install_jazzy/setup.bash
cd /ws
: "${DISPLAY:=:1}"
export DISPLAY LIBGL_ALWAYS_SOFTWARE=1   # software GL (no GPU passthrough)

DUR="${1:-}"
RATE="${2:-1.0}"
DUR_ARG=""
[ -n "$DUR" ] && DUR_ARG="--playback-duration $DUR"

cleanup() { kill $LOC $SCV $RVIZ 2>/dev/null || true; }
trap cleanup EXIT INT TERM

# 1) localizer (localize os_lidar directly; identity seed; NDT-only)
ros2 launch lidar_localization_ros2 lidar_localization.launch.py \
  localization_param_dir:=/ws/config/gt_ouster_ndt.yaml \
  cloud_topic:=/ouster/points imu_topic:=/imu/data use_sim_time:=true \
  global_frame_id:=map base_frame_id:=os_lidar lidar_frame_id:=os_lidar \
  publish_lidar_tf:=false use_imu_preintegration:=false > /tmp/loc.log 2>&1 &
LOC=$!
echo "localizer pid=$LOC ; waiting for map load + activation..."
until grep -aq "Activating end" /tmp/loc.log; do sleep 1; done
echo "localizer active; map published."

# 2) scovox occupancy mapping (integrates in the map frame via localizer TF)
ros2 launch scovox_mapping lidar_mapping.launch.py \
  params_file:=/ws/config/scovox_lidar_gt.yaml \
  pointcloud_topic:=/ouster/points use_sim_time:=true > /tmp/scovox.log 2>&1 &
SCV=$!
echo "scovox pid=$SCV"
sleep 3

# 3) rviz
ros2 run rviz2 rviz2 -d /ws/config/loc_scovox.rviz > /tmp/rviz.log 2>&1 &
RVIZ=$!
sleep 5

# 4) play the bag (Ctrl-C to stop early). /ouster/points forced RELIABLE.
echo "playing bag ${DUR:+(first ${DUR}s) }at rate ${RATE}... watch RViz."
ros2 bag play bags/2026_06_19_18_19_06__kalhan-map-test-2_ \
  --topics /ouster/points /imu/data --clock --rate "$RATE" \
  --qos-profile-overrides-path /ws/config/ouster_reliable_qos.yaml $DUR_ARG

echo "bag finished. scovox map still live in RViz; Ctrl-C to stop."
wait $SCV
