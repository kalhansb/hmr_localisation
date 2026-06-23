#!/usr/bin/env bash
# Reproduce the single-robot localization test against gt_map.ply.
# Usage (from the workspace root, on the HOST):
#   docker compose up -d
#   docker compose exec ros bash /ws/scripts/run_localization.sh [playback_duration_s]
#
# Leave playback_duration empty for the full bag. Outputs land in /ws/output/.
set -e
source /opt/ros/jazzy/setup.bash
source /ws/install/setup.bash
cd /ws

DUR="${1:-}"
DUR_ARG=""
[ -n "$DUR" ] && DUR_ARG="--playback-duration $DUR"

# 1) localizer: localize the os_lidar frame directly (bag has no base_link->lidar
#    extrinsic) against the GLIM map; identity seed (map built in lidar frame).
ros2 launch lidar_localization_ros2 lidar_localization.launch.py \
  localization_param_dir:=/ws/config/gt_ouster_ndt.yaml \
  cloud_topic:=/ouster/points imu_topic:=/imu/data use_sim_time:=true \
  global_frame_id:=map base_frame_id:=os_lidar lidar_frame_id:=os_lidar \
  publish_lidar_tf:=false use_imu_preintegration:=false > /tmp/loc.log 2>&1 &
LOC=$!
echo "localizer pid=$LOC ; waiting for map load + activation..."
until grep -aq "Activating end" /tmp/loc.log; do sleep 1; done
echo "active. playing bag ${DUR:+(first ${DUR}s)}..."

# 2) play the bag (sim clock). Only the two topics the localizer needs.
ros2 bag play bags/2026_06_19_18_19_06__kalhan-map-test-2_ \
  --topics /ouster/points /imu/data --clock --rate 1.0 $DUR_ARG

# 3) dump the latched /path trajectory (map frame) to CSV
python3 /ws/scripts/fetch_path.py /ws/output/path.csv

echo "good_scans=$(grep -ac 'fitness score:' /tmp/loc.log)  rejects=$(grep -ac 'fitness score is over' /tmp/loc.log)"
kill $LOC 2>/dev/null || true
echo "done. plot on host with: python3 scripts/plot_zoom.py gt_map/gt_map.ply output/path.csv output/eval.png"
