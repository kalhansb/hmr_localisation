#!/usr/bin/env bash
# SCovox INPUT-FEED comparison, single bag pass.
#
# One GLIM SLAM instance + 3 SCovox nodes, each fed a DIFFERENT input cloud but
# with IDENTICAL occupancy knobs, to isolate GLIM's per-point deskew (and
# loop-closure-optimized pose) from raw scans:
#   raw          <- /ouster/points                     (odom / os_lidar)  baseline
#   glim_points  <- /glim_ros/points                   (odom / imu)       deskew only
#   glim_aligned <- /glim_ros/aligned_points_corrected (map  / imu)       deskew + optimized pose
#
# Each config's yaml (config/feed_cmp/<id>.yaml) carries its own
# input_pointcloud_topic + frames, so (unlike the sweep runner) we do NOT
# override input_pointcloud_topic on the command line.
#
# Outputs:
#   /ws/output/feed_cmp/<id>.npy     SCovox occupancy per feed
#   /ws/output/glim_map_feed.pcd     GLIM global map (this run's reference)
#   /ws/output/path_glim_feed.csv    GLIM trajectory (this run)
#
# Usage: docker compose exec glim bash /ws/scripts/glim/run_feed_cmp.sh [rate]
set -e
source /opt/ros/jazzy/setup.bash
source /root/ros2_ws/install/setup.bash
cd /ws

RATE="${1:-0.5}"
BAG="bags/2026_06_19_18_19_06__kalhan-map-test-2_"
CFG_DIR="/ws/config/feed_cmp"
OUT="/ws/output/feed_cmp"
mkdir -p "$OUT"

if [ ! -f /scovox/install_glim/setup.bash ]; then echo "scovox not built"; exit 1; fi
source /scovox/install_glim/setup.bash

IDS=()
for f in "$CFG_DIR"/*.yaml; do IDS+=("$(basename "$f" .yaml)"); done
echo "feed configs: ${IDS[*]}"

PIDS=()
cleanup() { kill "${PIDS[@]}" 2>/dev/null || true; }
trap cleanup EXIT INT TERM

# 1) GLIM SLAM once.
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

# 2) One SCovox node per feed config (yaml sets its own input topic + frames).
for cid in "${IDS[@]}"; do
  ros2 run scovox_mapping scovox_mapping_node --ros-args \
    -r __node:="$cid" \
    --params-file "$CFG_DIR/$cid.yaml" \
    -p use_sim_time:=true > "/tmp/feed_$cid.log" 2>&1 &
  PIDS+=($!)
  echo "scovox '$cid' pid=${PIDS[-1]}"
  sleep 0.4
done
sleep 3

# 3) GLIM recorders (this run's reference map + trajectory).
python3 /ws/scripts/glim/record_glim_pose.py /ws/output/path_glim_feed.csv > /tmp/glim_pose.log 2>&1 &
PIDS+=($!); REC=$!
python3 /ws/scripts/glim/save_glim_map.py /ws/output/glim_map_feed.pcd > /tmp/glim_map.log 2>&1 &
PIDS+=($!); MAP=$!
sleep 2

# 4) Play the bag once.
echo "playing bag at rate ${RATE} ..."
ros2 bag play "$BAG" \
  --topics /ouster/points /imu/data --clock \
  --qos-profile-overrides-path config/ouster_reliable_qos.yaml \
  --read-ahead-queue-size 2000 \
  --rate "$RATE"

# 5) Flush, then salvage maps SEQUENTIALLY while every node is still HEALTHY.
echo "bag done; flushing (8s)..."
sleep 8
LAST=$(tail -1 /ws/output/path_glim_feed.csv 2>/dev/null | cut -d, -f1)
BASE=$(python3 -c "print(float('${LAST:-1781893646}') + 300.0)")
echo "salvaging ${#IDS[@]} SCovox maps sequentially (advancing /clock from ${BASE}) ..."
SALVAGE_TIMEOUT=180 python3 /ws/scripts/glim/salvage_capture_seq.py "$BASE" "$OUT" "${IDS[@]}" 2>&1 | tee /tmp/feed_capture.log

NPYS=$(ls "$OUT"/*.npy 2>/dev/null | wc -l)
echo "captured ${NPYS}/${#IDS[@]} maps."

kill -INT $MAP 2>/dev/null || true
sleep 4
kill $REC 2>/dev/null || true
sleep 1

echo "=== per-node recv counts (fairness check) ==="
for cid in "${IDS[@]}"; do
  n=$(grep -aoE "recv=[0-9]+" "/tmp/feed_$cid.log" 2>/dev/null | tail -1)
  tf=$(grep -aoE "TF FAILED" "/tmp/feed_$cid.log" 2>/dev/null | wc -l)
  echo "  $cid: ${n:-none}  (TF_FAILED lines=$tf)"
done
echo "poses=$(($(wc -l < /ws/output/path_glim_feed.csv 2>/dev/null || echo 1) - 1))"
echo "DONE. outputs in $OUT"
