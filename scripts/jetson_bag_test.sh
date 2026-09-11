#!/usr/bin/env bash
# Throughput test of the localizer against a recorded bag -- the check to run on a
# Jetson (or any new host) before trusting the live setup.  Localizer only, no EKF:
# odom -> base_link is a static identity so the numbers are the node's alone.
#
# Usage (HOST):
#   docker compose up -d
#   docker compose exec ros bash /ws/scripts/jetson_bag_test.sh [curtmini|bunker] [run_name] [duration_s]
#
# Two trimmed bags (lidar + IMU + /tf_static only, zstd mcap; see docs/jetson_runs.md,
# which also carries the bag manifest, the provenance of the seeds below, and how to
# score a run against a reference trajectory with scripts/analysis/traj_eval.py):
#   curtmini  /ws/bags/curtmini_jetson  Ouster  /ouster/points  /curt/imu/data  base_link_curt
#   bunker    /ws/bags/bunker_jetson    Hesai   /hesai/points   /imu/data       base_link
# Output -> /ws/output/jetson_test/<run_name>.{status.csv,poses.csv,log} and a
# rate / alignment-time / gap table on stdout.  gap_med 0.100 s means the node keeps up
# with the 10 Hz sensor; 0.200 s means it processes every other scan.
set -e
source /opt/ros/jazzy/setup.bash
source /ws/install/setup.bash
export FASTRTPS_DEFAULT_PROFILES_FILE=/ws/config/fastdds_shm.xml
export RCUTILS_COLORIZED_OUTPUT=0

PROFILE="${1:-curtmini}"
NAME="${2:-${PROFILE}_$(date +%Y%m%d_%H%M%S)}"
DUR="${3:-}"
case "$PROFILE" in
  curtmini)
    BAG=/ws/bags/curtmini_jetson; CLOUD=/ouster/points; IMU=/curt/imu/data; BASE=base_link_curt
    SEED=(8.33 -3.78 -1.39 0.853050270749362 0.5218287416139898) ;;
  bunker)
    BAG=/ws/bags/bunker_jetson; CLOUD=/hesai/points; IMU=/imu/data; BASE=base_link
    SEED=(7.82 -4.11 -1.17 0.8550024006656964 0.5186240399903342)
    # The Hesai cloud is unorganised (230400x1) with channels interleaved point by point,
    # so scan_channel_stride needs the beam count to find a point's channel.
    SET="scan_channel_count=128 ${SET:-}" ;;
  *) echo "unknown bag profile '$PROFILE' (curtmini|bunker)"; exit 1 ;;
esac
OUT=/ws/output/jetson_test
mkdir -p "$OUT"
[ -f "$BAG/metadata.yaml" ] || { echo "no bag at $BAG"; exit 1; }

# What the container can actually use.  A cpuset or CFS quota shows up here first. The
# node does NOT clamp its OpenMP pool to what it sees (ndt_num_threads is taken as-is),
# so set that to match these before tuning anything else.
echo "cores visible: $(nproc); cpuset: $(cat /sys/fs/cgroup/cpuset.cpus.effective 2>/dev/null || echo n/a); cpu quota: $(cat /sys/fs/cgroup/cpu.max 2>/dev/null || echo n/a)"

# The shipped config carries the real robot's start pose and base frame; each bag was
# recorded from its own spot, so derive a copy with that bag's seed (x y z qz qw, yaw-only).
CFG=/tmp/jetson_bag_test.yaml
sed -e "s/^\(\s*initial_pose_x:\).*/\1 ${SEED[0]}/" \
    -e "s/^\(\s*initial_pose_y:\).*/\1 ${SEED[1]}/" \
    -e "s/^\(\s*initial_pose_z:\).*/\1 ${SEED[2]}/" \
    -e 's/^\(\s*initial_pose_qx:\).*/\1 0.0/' \
    -e 's/^\(\s*initial_pose_qy:\).*/\1 0.0/' \
    -e "s/^\(\s*initial_pose_qz:\).*/\1 ${SEED[3]}/" \
    -e "s/^\(\s*initial_pose_qw:\).*/\1 ${SEED[4]}/" \
    -e "s/^\(\s*base_frame_id:\).*/\1 $BASE/" \
    /ws/config/gt_ouster_ndt_tree_realtime.yaml > "$CFG"

# Optional A/B overrides, applied to the derived copy only (the shipped config is not
# touched):  MAP=/ws/gt_map/gt_map_us100.pcd   swaps the map;
#            SET="voxel_leaf_size=0.3 ndt_max_iterations=30"   sets any scalar parameter.
[ -n "${MAP:-}" ] && SET="map_path=\"$MAP\" ${SET:-}"
for kv in ${SET:-}; do
  k="${kv%%=*}"; v="${kv#*=}"
  grep -q "^\s*$k:" "$CFG" || { echo "override '$k' is not a parameter in the config"; exit 1; }
  sed -i "s|^\(\s*$k:\)[^#]*\(#.*\)\?$|\1 $v  \2|" "$CFG"
  echo "override: $k = $v"
done

PIDS=()
# TERM everything we started and wait for it to actually exit (the launch process only
# returns once the node has finished its shutdown), so nothing from this run is still
# alive -- holding Fast-DDS SHM port locks and re-publishing its latched status -- when
# the next run starts.  KILL is the last resort after 20 s.
cleanup() {
  kill -TERM "${PIDS[@]}" 2>/dev/null || true
  for _ in $(seq 1 40); do
    alive=0
    for p in "${PIDS[@]}"; do kill -0 "$p" 2>/dev/null && alive=1; done
    [ "$alive" -eq 0 ] && return 0
    sleep 0.5
  done
  kill -KILL "${PIDS[@]}" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

# The binary itself, not `ros2 run`: the python wrapper does not forward TERM, so its
# child would outlive every run.
"$(ros2 pkg prefix tf2_ros)/lib/tf2_ros/static_transform_publisher" --x 0 --y 0 --z 0 \
  --frame-id odom --child-frame-id "$BASE" > /dev/null 2>&1 &
PIDS+=($!)
ros2 launch lidar_localization_ros2 lidar_localization.launch.py \
  localization_param_dir:="$CFG" cloud_topic:="$CLOUD" imu_topic:="$IMU" \
  use_sim_time:=true global_frame_id:=map odom_frame_id:=odom base_frame_id:="$BASE" \
  use_imu_preintegration:=true imu_preintegration_use_base_frame_transform:=true \
  publish_lidar_tf:=false publish_imu_tf:=false > "$OUT/$NAME.log" 2>&1 &
PIDS+=($!)
LAUNCH_PID=$!
echo "waiting for map load + activation..."
for _ in $(seq 1 300); do grep -aq "Activating end" "$OUT/$NAME.log" && break; sleep 1; done
grep -aq "Activating end" "$OUT/$NAME.log" || { echo "localizer failed to activate:"; tail -20 "$OUT/$NAME.log"; exit 1; }
grep -a "registration_method\|ndt_num_threads\|Map Size" "$OUT/$NAME.log" | sed 's/.*\]: //'

python3 /ws/scripts/analysis/record_alignment_status.py "$OUT/$NAME.status.csv" \
  --ros-args -p use_sim_time:=true > /dev/null 2>&1 &
PIDS+=($!)
python3 /ws/src/lidar_localization_ros2/scripts/benchmark_pose_recorder --ros-args \
  -p topic:=/pcl_pose -p output_path:="$OUT/$NAME.poses.csv" -p qos_reliability:=reliable \
  -p qos_durability:=transient_local -p use_sim_time:=true > /dev/null 2>&1 &
PIDS+=($!)
sleep 2

# Sample the node's CPU time and resident size every 2 s while the bag plays: the
# throughput table says whether it keeps up, this says what that costs.
NODE_PID=$(pgrep -P "$LAUNCH_PID" -f lidar_localization_node | head -1)
CPU_CSV="$OUT/$NAME.cpu.csv"
echo "time_sec,cpu_ticks,rss_kb" > "$CPU_CSV"
(
  while [ -n "$NODE_PID" ] && [ -r "/proc/$NODE_PID/stat" ]; do
    printf '%s,%s,%s\n' "$(date +%s.%N)" \
      "$(awk '{print $14+$15}' "/proc/$NODE_PID/stat")" \
      "$(awk '/^VmRSS:/{print $2}' "/proc/$NODE_PID/status")" >> "$CPU_CSV"
    sleep 2
  done
) &
SAMPLER_PID=$!

echo "playing $BAG at rate 1.0${DUR:+ (first ${DUR}s)}..."
ros2 bag play "$BAG" --topics "$CLOUD" "$IMU" /tf_static --clock --rate 1.0 ${DUR:+--playback-duration $DUR} > /dev/null 2>&1

# Let the node drain any queued backlog before reading the numbers.
last=0; idle=0
while [ $idle -lt 10 ]; do
  sleep 2
  cur=$(wc -l < "$OUT/$NAME.status.csv" 2>/dev/null || echo 0)
  if [ "$cur" -gt "$last" ]; then last=$cur; idle=0; else idle=$((idle+2)); fi
done
kill "$SAMPLER_PID" 2>/dev/null || true
echo
python3 /ws/scripts/analysis/throughput_summary.py "$OUT/$NAME.status.csv"
awk -F, -v hz="$(getconf CLK_TCK)" -v ncpu="$(nproc)" 'NR==2{t0=$1; c0=$2} NR>1{if($3>rss)rss=$3; t1=$1; c1=$2}
  END{if(t1>t0) printf "node CPU: %.2f cores avg over %.0f s (of %d visible); peak RSS %.0f MB\n",
      (c1-c0)/hz/(t1-t0), t1-t0, ncpu, rss/1024}' "$CPU_CSV"
echo "files -> $OUT/$NAME.*"
