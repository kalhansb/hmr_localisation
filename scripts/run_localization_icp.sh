#!/usr/bin/env bash
# ICP localization (baiyeweiguang/icp_localization_ros2, libpointmatcher) against
# the GLIM map (downsampled gt_map_ds03.pcd) using the Ouster bag. Runs in the
# ROS 2 HUMBLE container.
# Usage (from the workspace root, on the HOST):
#   docker compose up -d ros_humble
#   docker compose exec ros_humble bash /ws/scripts/run_localization_icp.sh [wall_seconds] [rate]
#
# wall_seconds: cap playback to this many WALL-clock seconds (covers wall*rate of
#   bag time); empty = full bag. rate: playback speed (default 1.0). ICP runs at
#   ~5 Hz here; the cloud sub is KeepLast(1) so it drops scans rather than lag,
#   but a lower rate (e.g. 0.5) processes more scans for a denser trajectory.
# Outputs land in /ws/output/.
set -e
source /opt/ros/humble/setup.bash
source /ws/install_humble/setup.bash
cd /ws

# Humble's `ros2 bag play` has no --playback-duration; cap with `timeout` instead.
DUR="${1:-}"
RATE="${2:-1.0}"
SRC_BAG="bags/2026_06_19_18_19_06__kalhan-map-test-2_"
# The bag was recorded on Jazzy (rosbag2 v9 metadata) which Humble's rosbag2
# can't parse. Build a non-destructive sibling that symlinks the mcap and carries
# a Humble-format metadata.yaml (reindex reads the mcap summary -> instant).
BAG="${SRC_BAG}__humble"
if [ ! -f "/ws/$BAG/metadata.yaml" ]; then
  echo "creating Humble-compatible bag view: $BAG"
  mkdir -p "/ws/$BAG"
  name="${SRC_BAG##*/}"
  for f in /ws/"$SRC_BAG"/*.mcap; do ln -sf "../$name/$(basename "$f")" "/ws/$BAG/$(basename "$f")"; done
  ros2 bag reindex -s mcap "/ws/$BAG"
fi

# 0) ICP loads a PCD map (PCL loadPCDFile); the GLIM map ships as PLY. Generate the
#    downsampled map (referenced by gt_ouster_icp.yaml) once.
[ -f /ws/gt_map/gt_map_ds03.pcd ] || python3 /ws/scripts/ply_to_pcd.py /ws/gt_map/gt_map_ds03.ply /ws/gt_map/gt_map_ds03.pcd

# 1) ICP localizer: map -> range_sensor (os_lidar) directly, identity seed,
#    IMU used to extrapolate the pose between scans.
ros2 run icp_localization_ros2 icp_localization --ros-args \
  --params-file /ws/config/gt_ouster_icp.yaml \
  -p use_sim_time:=true > /tmp/icp.log 2>&1 &
LOC=$!
echo "icp pid=$LOC ; loading ${BAG##*/} map + initializing (map normals take a moment)..."
until grep -aq "succesfully initialized icp" /tmp/icp.log; do
  sleep 1
  kill -0 $LOC 2>/dev/null || { echo "ICP node died during init:"; tail -20 /tmp/icp.log; exit 1; }
done
echo "initialized. recording pose + playing bag ${DUR:+(first ${DUR}s)}..."

# 2) record the range_sensor_pose trajectory (ICP has no /path)
python3 /ws/scripts/icp/record_range_sensor_pose.py /ws/output/path_icp.csv &
REC=$!
sleep 1

# 3) play the bag (sim clock). Republish /ouster/points as RELIABLE so the ICP
#    accumulator (reliable subscriber) matches the bag's best-effort publisher.
PLAY=(ros2 bag play "$BAG"
  --topics /ouster/points /imu/data --clock
  --qos-profile-overrides-path config/icp_qos_overrides.yaml
  --rate "$RATE")
if [ -n "$DUR" ]; then
  timeout "${DUR}s" "${PLAY[@]}" || true   # timeout exits 124 when it caps playback
else
  "${PLAY[@]}"
fi

sleep 2
kill $REC 2>/dev/null || true
if kill -0 $LOC 2>/dev/null; then kill $LOC 2>/dev/null; else
  echo "WARNING: ICP node exited before playback finished:"; tail -3 /tmp/icp.log
fi
echo "poses=$(($(wc -l < /ws/output/path_icp.csv) - 1))  (see /tmp/icp.log for ICP errors/score)"
echo "done. plot on host: python3 scripts/plot_zoom.py gt_map/gt_map_ds03.ply output/path_icp.csv output/eval_icp.png"
