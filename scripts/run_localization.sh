#!/usr/bin/env bash
# Run the real-time NDT localizer (config/gt_ouster_ndt_realtime.yaml) against the
# GLIM map at live playback rate 1.0 (Ouster ~7.6 Hz). Localizes the os_lidar frame
# directly (identity seed; the map is built in the lidar frame) and dumps the path.
#
# Usage (from the workspace root, on the HOST):
#   docker compose up -d
#   docker compose exec ros bash /ws/scripts/run_localization.sh [playback_duration_s]
#
# Leave playback_duration empty for the full bag. Output -> /ws/output/path.csv.
# The config loads /ws/gt_map/gt_map_us050.pcd -- generate it once from gt_map.ply
# with scripts/downsample_map_pcl.cpp (see README), or point map_path at gt_map.ply.
set -e
source /opt/ros/jazzy/setup.bash
source /ws/install/setup.bash
cd /ws

DUR="${1:-}"
DUR_ARG=""
[ -n "$DUR" ] && DUR_ARG="--playback-duration $DUR"

# 1) real-time localizer (16 NDT threads, reject gate off -- see the config header)
ros2 launch lidar_localization_ros2 lidar_localization.launch.py \
  localization_param_dir:=/ws/config/gt_ouster_ndt_realtime.yaml \
  cloud_topic:=/ouster/points imu_topic:=/imu/data use_sim_time:=true \
  global_frame_id:=map base_frame_id:=os_lidar lidar_frame_id:=os_lidar \
  publish_lidar_tf:=false use_imu_preintegration:=false > /tmp/loc.log 2>&1 &
LOC=$!
echo "localizer pid=$LOC ; waiting for map load + activation..."
until grep -aq "Activating end" /tmp/loc.log; do sleep 1; done
echo "active. playing bag at rate 1.0 ${DUR:+(first ${DUR}s)}..."

# 2) play the bag (sim clock). Only the two topics the localizer needs.
ros2 bag play bags/2026_06_19_18_19_06__kalhan-map-test-2_ \
  --topics /ouster/points /imu/data --clock --rate 1.0 $DUR_ARG

# 3) dump the latched /path trajectory (map frame) to CSV
python3 /ws/scripts/fetch_path.py /ws/output/path.csv

echo "good_scans=$(grep -ac 'fitness score:' /tmp/loc.log)"
kill $LOC 2>/dev/null || true
echo "done. trajectory -> output/path.csv"
