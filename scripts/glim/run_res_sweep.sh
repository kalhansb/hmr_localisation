#!/usr/bin/env bash
# SCovox RESOLUTION sweep, single bag pass. 1 GLIM + 3 scovox nodes (0.20/0.10/0.05 m),
# all fed GLIM's deskewed /glim_ros/points with identical occupancy. Also accumulates
# GLIM's deskewed points (/glim_ros/aligned_points) as the match reference.
#
# Outputs:
#   /ws/output/res_sweep/<id>.npy        scovox occupancy per resolution (odom frame)
#   /ws/output/glim_points_accum.npy     accumulated deskewed GLIM points (map frame)
#   /ws/output/glim_map_res.pcd          GLIM global map ; path_glim_res.csv  trajectory
#
# Usage: docker compose exec glim bash /ws/scripts/glim/run_res_sweep.sh [rate]
set -e
source /opt/ros/jazzy/setup.bash
source /root/ros2_ws/install/setup.bash
source /scovox/install_glim/setup.bash
cd /ws

RATE="${1:-0.5}"
BAG="bags/2026_06_19_18_19_06__kalhan-map-test-2_"
CFG_DIR="/ws/config/res_sweep"
OUT="/ws/output/res_sweep"; mkdir -p "$OUT"

IDS=(); for f in "$CFG_DIR"/*.yaml; do IDS+=("$(basename "$f" .yaml)"); done
echo "res configs: ${IDS[*]}  (rate=$RATE)"

PIDS=(); cleanup() { kill "${PIDS[@]}" 2>/dev/null || true; }
trap cleanup EXIT INT TERM

# 1) GLIM
ros2 run glim_ros glim_rosnode --ros-args \
  -p config_path:=/ws/glim_config -p use_sim_time:=true > /tmp/glim.log 2>&1 &
PIDS+=($!); GLIM=$!
echo "glim pid=$GLIM ; loading..."
for i in $(seq 1 40); do
  grep -qaiE "critical|failed to load .*module" /tmp/glim.log && { echo "GLIM init error:"; tail -20 /tmp/glim.log; exit 1; }
  kill -0 $GLIM 2>/dev/null || { echo "GLIM died:"; tail -20 /tmp/glim.log; exit 1; }
  grep -qaiE "global_mapping|odometry_estimation" /tmp/glim.log && break; sleep 1
done
sleep 2; echo "GLIM up."

# 2) scovox nodes (one per resolution; yaml carries input topic + frames)
for cid in "${IDS[@]}"; do
  ros2 run scovox_mapping scovox_mapping_node --ros-args \
    -r __node:="$cid" --params-file "$CFG_DIR/$cid.yaml" -p use_sim_time:=true \
    > "/tmp/res_$cid.log" 2>&1 &
  PIDS+=($!); echo "scovox '$cid' pid=${PIDS[-1]}"; sleep 0.4
done
sleep 3

# 3) reference accumulator + GLIM recorders
python3 /ws/scripts/glim/accumulate_aligned.py > /tmp/accum.log 2>&1 &
PIDS+=($!); ACC=$!
python3 /ws/scripts/glim/record_glim_pose.py /ws/output/path_glim_res.csv > /tmp/glim_pose.log 2>&1 &
PIDS+=($!); REC=$!
python3 /ws/scripts/glim/save_glim_map.py /ws/output/glim_map_res.pcd > /tmp/glim_map.log 2>&1 &
PIDS+=($!); MAP=$!
sleep 2

# 4) play bag once
echo "playing bag at rate ${RATE} ..."
ros2 bag play "$BAG" --topics /ouster/points /imu/data --clock \
  --qos-profile-overrides-path config/ouster_reliable_qos.yaml \
  --read-ahead-queue-size 2000 --rate "$RATE"

echo "bag done; flushing (10s)..."
sleep 10
# save reference accumulator + GLIM map BEFORE the salvage clock-jump
kill -INT $ACC 2>/dev/null || true; sleep 3
kill -INT $MAP 2>/dev/null || true; sleep 4

# 5) salvage scovox maps sequentially (RELIABLE for the big fine-res maps)
LAST=$(tail -1 /ws/output/path_glim_res.csv 2>/dev/null | cut -d, -f1)
BASE=$(python3 -c "print(float('${LAST:-1781893646}') + 400.0)")
echo "salvaging ${#IDS[@]} scovox maps (advancing /clock from ${BASE}) ..."
SALVAGE_RELIABLE=1 SALVAGE_TIMEOUT=240 \
  python3 /ws/scripts/glim/salvage_capture_seq.py "$BASE" "$OUT" "${IDS[@]}" 2>&1 | tee /tmp/res_capture.log

kill $REC 2>/dev/null || true; sleep 1
echo "=== per-node recv (fairness) ==="
for cid in "${IDS[@]}"; do echo "  $cid: $(grep -aoE 'recv=[0-9]+' /tmp/res_$cid.log | tail -1)  TF_FAIL=$(grep -ac 'TF FAILED' /tmp/res_$cid.log)"; done
echo "poses=$(($(wc -l < /ws/output/path_glim_res.csv 2>/dev/null || echo 1) - 1))"
echo "ref accum: $(tail -1 /tmp/accum.log 2>/dev/null)"
echo "DONE -> $OUT"
