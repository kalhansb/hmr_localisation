#!/usr/bin/env bash
# ICP localization (baiyeweiguang/icp_localization_ros2, libpointmatcher) against
# the GLIM map, exposed as a PROPER REP-105 TF tree map -> odom -> base_link, i.e.
# the ICP equivalent of NDT "Mode B" (run_localization_tree_noekf.sh). Runs in the
# ROS 2 HUMBLE container.
#
#   map ──(ICP scan-match -> range_sensor_pose, re-broadcast by icp_tree_publisher)──> odom
#   odom ──(STATIC identity, no wheel odometry)──> base_link
#   base_link ──(static extrinsics, same as NDT Mode B)──> os_lidar, imu
#
# The ICP node itself can only emit map->odom with is_use_odometry=true, which also
# needs a real nav_msgs/Odometry topic we don't have (the IMU-only odom path is an
# empty TODO in the package, and is_provide_odom_frame is a dead param). So instead
# of patching/rebuilding, icp_tree_publisher.py consumes range_sensor_pose and
# rebuilds the canonical tree as a disjoint branch under map. See that script's
# header and the README "ICP" section for the full rationale.
#
# Usage (from the workspace root, on the HOST):
#   docker compose up -d ros_humble
#   docker compose exec ros_humble bash /ws/scripts/run_localization_icp_tree.sh [wall_seconds] [rate]
#
# wall_seconds: cap playback to this many WALL-clock seconds (empty = full bag).
# rate: playback speed (default 1.0; 0.5 processes more scans for a denser path).
# Outputs land in /ws/output/ ; the TF-tree check is written to /tmp/tf_chain_icp.log.
set -e
source /opt/ros/humble/setup.bash
source /ws/install_humble/setup.bash
cd /ws

DUR="${1:-}"
RATE="${2:-1.0}"
SRC_BAG="bags/2026_06_19_18_19_06__kalhan-map-test-2_"
# Jazzy-recorded bag (rosbag2 v9 metadata) -> build a non-destructive Humble view.
BAG="${SRC_BAG}__humble"
if [ ! -f "/ws/$BAG/metadata.yaml" ]; then
  echo "creating Humble-compatible bag view: $BAG"
  mkdir -p "/ws/$BAG"
  name="${SRC_BAG##*/}"
  for f in /ws/"$SRC_BAG"/*.mcap; do ln -sf "../$name/$(basename "$f")" "/ws/$BAG/$(basename "$f")"; done
  ros2 bag reindex -s mcap "/ws/$BAG"
fi

# ICP loads a PCD map; the GLIM map ships as PLY. Generate the downsampled PCD once.
[ -f /ws/gt_map/gt_map_ds03.pcd ] || python3 /ws/scripts/ply_to_pcd.py /ws/gt_map/gt_map_ds03.ply /ws/gt_map/gt_map_ds03.pcd

PIDS=()
cleanup() { kill "${PIDS[@]}" 2>/dev/null || true; }
trap cleanup EXIT INT TERM

# 1) ICP localizer (Mode A: map->range_sensor + range_sensor_pose). is_use_odometry
#    STAYS false -- turning it on would make the node ALSO publish map->odom and
#    collide with our tree publisher's odom frame.
ros2 run icp_localization_ros2 icp_localization --ros-args \
  --params-file /ws/config/gt_ouster_icp.yaml \
  -p use_sim_time:=true > /tmp/icp_tree.log 2>&1 &
PIDS+=($!); LOC=$!
echo "icp pid=$LOC ; loading map + initializing (map normals take a moment)..."
until grep -aq "succesfully initialized icp" /tmp/icp_tree.log; do
  sleep 1
  kill -0 $LOC 2>/dev/null || { echo "ICP node died during init:"; tail -20 /tmp/icp_tree.log; exit 1; }
done
echo "initialized."

# 2) REP-105 tree publisher: map->odom (dynamic, from range_sensor_pose) plus the
#    static odom->base_link (identity) and base_link->{os_lidar,imu} extrinsics.
python3 /ws/scripts/icp/icp_tree_publisher.py --ros-args -p use_sim_time:=true \
  > /tmp/icp_tree_pub.log 2>&1 &
PIDS+=($!)

# 3) record the lidar trajectory (range_sensor_pose = map->os_lidar) for plotting.
python3 /ws/scripts/icp/record_range_sensor_pose.py /ws/output/path_icp_tree.csv &
PIDS+=($!); REC=$!
sleep 1
echo "recording pose + playing bag ${DUR:+(first ${DUR}s)}..."

# 4) TF-tree sanity check a few seconds into playback (sim clock). Two key facts:
#    (a) map->base_link resolves  => the REP-105 tree is connected end to end.
#    (b) range_sensor->os_lidar ROTATION is identity => map->os_lidar (via
#        odom/base_link) and map->range_sensor (ICP's own edge) carry the same
#        orientation, i.e. the base_link detour cancels exactly. A non-identity
#        rotation means the base_to_lidar extrinsic (icp_tree_publisher.py) is
#        wrong. The translation will show a SMALL (~0.05-0.2 m) motion-correlated
#        residual: that is NOT a bug, it is ICP's own TF-stamp latency -- the
#        package stamps map->range_sensor with nh_->now() (TfPublisher.cpp:76),
#        ~one processing delay AFTER the scan time, whereas our map->odom is
#        stamped at the true scan time (range_sensor_pose.header.stamp). Composing
#        the two branches at a common time leaves a gap = latency x speed. Our
#        branch (used by /ouster/points in os_lidar) is the time-accurate one.
( sleep 12
  : > /tmp/tf_chain_icp.log
  for pair in "map base_link" "map os_lidar" "range_sensor os_lidar"; do
    echo "--- $pair ---" >> /tmp/tf_chain_icp.log
    timeout 4 ros2 run tf2_ros tf2_echo $pair --ros-args -p use_sim_time:=true \
      >> /tmp/tf_chain_icp.log 2>&1 || true
  done ) &
PIDS+=($!)

# 5) play the bag (sim clock; republish topics RELIABLE to match the ICP subs).
PLAY=(ros2 bag play "$BAG"
  --topics /ouster/points /imu/data --clock
  --qos-profile-overrides-path config/icp_qos_overrides.yaml
  --rate "$RATE")
if [ -n "$DUR" ]; then
  timeout "${DUR}s" "${PLAY[@]}" || true
else
  "${PLAY[@]}"
fi

sleep 2
kill $REC 2>/dev/null || true
if ! kill -0 $LOC 2>/dev/null; then
  echo "WARNING: ICP node exited before playback finished:"; tail -3 /tmp/icp_tree.log
fi
echo "poses=$(($(wc -l < /ws/output/path_icp_tree.csv) - 1))  (ICP errors/score: /tmp/icp_tree.log)"
echo "--- TF tree (map->base_link, map->os_lidar, map->range_sensor) ---"
cat /tmp/tf_chain_icp.log 2>/dev/null || true
echo "done. plot on host: python3 scripts/plot_zoom.py gt_map/gt_map_ds03.ply output/path_icp_tree.csv output/eval_icp_tree.png"
