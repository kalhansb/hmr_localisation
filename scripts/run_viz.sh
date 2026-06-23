#!/usr/bin/env bash
# Live RViz visualization of localization against gt_map.ply.
#
# On the HOST first (once per login), allow the container to use your X server:
#   xhost +local:
# Then:
#   docker compose up -d
#   docker compose exec -e DISPLAY=:1 ros bash /ws/scripts/run_viz.sh
#
# RViz shows: green = GT map (colored by height), red = live Ouster scan placed
# in the map frame by localization, green line = trajectory (/path), yellow =
# current pose (/pcl_pose). Fixed frame = map.
set -e
source /opt/ros/jazzy/setup.bash
source /ws/install/setup.bash
cd /ws
: "${DISPLAY:=:1}"
export DISPLAY LIBGL_ALWAYS_SOFTWARE=1   # software GL (no GPU passthrough in this container)

# 1) localizer
ros2 launch lidar_localization_ros2 lidar_localization.launch.py \
  localization_param_dir:=/ws/config/gt_ouster_ndt.yaml \
  cloud_topic:=/ouster/points imu_topic:=/imu/data use_sim_time:=true \
  global_frame_id:=map base_frame_id:=os_lidar lidar_frame_id:=os_lidar \
  publish_lidar_tf:=false use_imu_preintegration:=false > /tmp/loc.log 2>&1 &
until grep -aq "Activating end" /tmp/loc.log; do sleep 1; done
echo "localizer active; map published."

# 2) rviz
ros2 run rviz2 rviz2 -d /ws/config/localization.rviz > /tmp/rviz.log 2>&1 &
sleep 5

# 3) play the bag (Ctrl-C to stop early)
echo "playing bag... watch RViz."
ros2 bag play bags/2026_06_19_18_19_06__kalhan-map-test-2_ \
  --topics /ouster/points /imu/data --clock --rate 1.0
