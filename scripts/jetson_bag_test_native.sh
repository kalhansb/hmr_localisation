#!/usr/bin/env bash
# Native (no-Docker) port of scripts/jetson_bag_test.sh for this Jetson.
# Tracks upstream at 7918b5f; keep the two in sync when jetson_bag_test.sh changes.
#
# Usage:
#   scripts/jetson_bag_test_native.sh [curtmini|bunker] [run_name] [duration_s]
#   MAP=gt_map/gt_map_us100.pcd  scripts/... bunker bunk_us100
#   SET="scan_channel_stride=4"  scripts/... curtmini curt_ch4
#
# DIFFERENCES FROM THE CONTAINER VERSION, all forced by the host environment:
#  1. ROS 2 Humble, not Jazzy. The Jazzy-targeted fork builds clean against Humble.
#  2. rosbag2-storage-mcap is not installed system-wide; it was unpacked into
#     ~/ros-extra with `dpkg -x` (no root) and is reached via AMENT_PREFIX_PATH.
#  3. small_gicp is built into the Docker image upstream; here it is a user-prefix
#     build at the same pinned tag (v1.0.1) and needs an explicit LD_LIBRARY_PATH,
#     or the node aborts with "small_gicp backend requested but support is not
#     available".
#  4. The bags carry rosbag2 metadata version 9 (Jazzy). Humble's rosbag2 0.15
#     cannot parse it -- Jazzy writes offered_qos_profiles as a YAML sequence where
#     Humble expects a string ("bad conversion"). So: play the .mcap FILE directly
#     with `-s mcap` (bypasses metadata.yaml) AND supply --qos-profile-overrides-path
#     (bypasses the profiles stored in the mcap's own channel metadata).
#     `ros2 bag info` works without either because it never parses QoS.
#  5. Humble's rosbag2 has no --playback-duration, so `timeout` bounds the run.
#  6. Orphaned Fast-DDS SHM segments make a new node HANG before rclcpp logging
#     starts, so it never activates and never says why. Reaped below.
set -e
WS=/home/jetsondevkit/jetbot-slam/hmr_localisation
# No `set -u`: /opt/ros/humble/setup.bash reads AMENT_TRACE_SETUP_FILES unguarded.
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"
export AMENT_PREFIX_PATH=/home/jetsondevkit/ros-extra/prefix/opt/ros/humble:$AMENT_PREFIX_PATH
# The aarch64-linux-gnu subdir carries Cyclone's libddsc.so.0 / iceoryx libs, which
# were also unpacked into ~/ros-extra with dpkg -x. Harmless under Fast-DDS.
export LD_LIBRARY_PATH=/home/jetsondevkit/ros-extra/prefix/opt/ros/humble/lib:/home/jetsondevkit/ros-extra/prefix/opt/ros/humble/lib/aarch64-linux-gnu:/home/jetsondevkit/small_gicp-install/lib:$LD_LIBRARY_PATH
# SHM_PROFILE= swaps the Fast-DDS profile (e.g. to A/B the SHM segment_size, which
# is mapped end to end into every subscriber and so shows up directly in the node's
# RSS). Defaults to the shipped profile.
export FASTRTPS_DEFAULT_PROFILES_FILE="${SHM_PROFILE:-$WS/config/fastdds_shm.xml}"
export RCUTILS_COLORIZED_OUTPUT=0

PROFILE="${1:-curtmini}"
NAME="${2:-${PROFILE}_$(date +%Y%m%d_%H%M%S)}"
DUR="${3:-}"
case "$PROFILE" in
  curtmini)
    BAG="$WS/bags/curtmini_jetson/curtmini_jetson_0.mcap"
    CLOUD=/ouster/points; IMU=/curt/imu/data; BASE=base_link_curt
    SEED=(8.33 -3.78 -1.39 0.853050270749362 0.5218287416139898) ;;
  bunker)
    BAG="$WS/bags/bunker_jetson/bunker_jetson_0.mcap"
    CLOUD=/hesai/points; IMU=/imu/data; BASE=base_link
    SEED=(7.82 -4.11 -1.17 0.8550024006656964 0.5186240399903342)
    # Hesai cloud is unorganised (230400x1) with channels interleaved point by
    # point, so scan_channel_stride needs the beam count to find a point's channel.
    SET="scan_channel_count=128 ${SET:-}" ;;
  *) echo "unknown bag profile '$PROFILE' (curtmini|bunker)"; exit 1 ;;
esac
OUT="$WS/output/jetson_test"
mkdir -p "$OUT"
[ -f "$BAG" ] || { echo "no bag at $BAG"; exit 1; }

# Reap leftovers from a previous run FIRST: a surviving node holds the SHM port
# locks and re-publishes its latched status into the next run's recorder. The
# bracket stops pgrep matching this script's own command line.
for _p in "lidar_localization_nod[e]" "static_transform_publishe[r]" \
          "ros2 bag pla[y]" "record_alignment_statu[s]" "benchmark_pose_recorde[r]"; do
  pgrep -f "$_p" | xargs -r kill -9 2>/dev/null
done
sleep 3
command -v fastdds > /dev/null && \
  echo "shm: $(timeout 30 fastdds shm clean 2>/dev/null | tr '\n' ' ' | sed 's/  */ /g')"

echo "cores visible: $(nproc); affinity: $(taskset -pc $$ 2>/dev/null | sed 's/.*: //')"
echo "rmw: ${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp (default)}"
# The Fast-DDS SHM profile is meaningless under Cyclone, which reads CYCLONEDDS_URI
# instead -- record which transport config is actually in force.
[ "${RMW_IMPLEMENTATION:-}" = "rmw_cyclonedds_cpp" ] \
  && echo "cyclone uri: ${CYCLONEDDS_URI:-<none, Cyclone defaults>}" \
  || echo "fastdds profile: $FASTRTPS_DEFAULT_PROFILES_FILE"
echo "bag: $BAG"

CFG=/tmp/jetson_bag_test_native.yaml
sed -e "s/^\(\s*initial_pose_x:\).*/\1 ${SEED[0]}/" \
    -e "s/^\(\s*initial_pose_y:\).*/\1 ${SEED[1]}/" \
    -e "s/^\(\s*initial_pose_z:\).*/\1 ${SEED[2]}/" \
    -e 's/^\(\s*initial_pose_qx:\).*/\1 0.0/' \
    -e 's/^\(\s*initial_pose_qy:\).*/\1 0.0/' \
    -e "s/^\(\s*initial_pose_qz:\).*/\1 ${SEED[3]}/" \
    -e "s/^\(\s*initial_pose_qw:\).*/\1 ${SEED[4]}/" \
    -e "s/^\(\s*base_frame_id:\).*/\1 $BASE/" \
    "$WS/config/gt_ouster_ndt_tree_realtime.yaml" > "$CFG"

# MAP= is repo-relative or absolute here (no /ws mount). This is ALWAYS applied,
# defaulting to the shipped map: the config's map_path is the container path
# "/ws/gt_map/gt_map_us050.pcd", which does not exist natively, and the node then
# fails activation with only "Failed to load pcd file" to show for it.
MAP="${MAP:-gt_map/gt_map_us050.pcd}"
case "$MAP" in /*) MAP_ABS="$MAP" ;; *) MAP_ABS="$WS/$MAP" ;; esac
[ -f "$MAP_ABS" ] || { echo "no map at $MAP_ABS"; exit 1; }
SET="map_path=\"$MAP_ABS\" ${SET:-}"
for kv in ${SET:-}; do
  k="${kv%%=*}"; v="${kv#*=}"
  grep -q "^\s*$k:" "$CFG" || { echo "override '$k' is not a parameter in the config"; exit 1; }
  sed -i "s|^\(\s*$k:\)[^#]*\(#.*\)\?$|\1 $v  \2|" "$CFG"
  echo "override: $k = $v"
done

PIDS=()
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

# PIN_CORES=0 pins ONLY the localizer to those cores, leaving the bag player,
# recorders and tf publisher on the rest. Wrapping the whole script in
# `taskset -c 0` instead is misleading: affinity is inherited across fork, so the
# player ends up sharing the one core with the node and the measured rate is
# depressed by contention that would not exist on a real robot (where scans come
# from a driver, not a co-resident bag player).
NODE_TASKSET=""
if [ -n "${PIN_CORES:-}" ]; then
  NODE_TASKSET="taskset -c $PIN_CORES"
  echo "pinning localizer to core(s) $PIN_CORES (player/recorders unpinned)"
fi

# The binary itself, not `ros2 run`: the python wrapper does not forward TERM.
"$(ros2 pkg prefix tf2_ros)/lib/tf2_ros/static_transform_publisher" --x 0 --y 0 --z 0 \
  --frame-id odom --child-frame-id "$BASE" > /dev/null 2>&1 &
PIDS+=($!)
$NODE_TASKSET ros2 launch lidar_localization_ros2 lidar_localization.launch.py \
  localization_param_dir:="$CFG" cloud_topic:="$CLOUD" imu_topic:="$IMU" \
  use_sim_time:=true global_frame_id:=map odom_frame_id:=odom base_frame_id:="$BASE" \
  use_imu_preintegration:=true imu_preintegration_use_base_frame_transform:=true \
  publish_lidar_tf:=false publish_imu_tf:=false > "$OUT/$NAME.log" 2>&1 &
PIDS+=($!)
LAUNCH_PID=$!
echo "waiting for map load + activation..."
for _ in $(seq 1 300); do grep -aq "Activating end" "$OUT/$NAME.log" && break; sleep 1; done
grep -aq "Activating end" "$OUT/$NAME.log" || { echo "localizer failed to activate:"; tail -25 "$OUT/$NAME.log"; exit 1; }
grep -a "registration_method\|ndt_num_threads\|scan_channel_stride\|Map Size" "$OUT/$NAME.log" | sed 's/.*\]: //'

# record_alignment_status.py strips --ros-args itself as of upstream 7918b5f.
python3 "$WS/scripts/analysis/record_alignment_status.py" "$OUT/$NAME.status.csv" \
  --ros-args -p use_sim_time:=true > "$OUT/$NAME.statusrec.log" 2>&1 &
PIDS+=($!)
python3 "$WS/src/lidar_localization_ros2/scripts/benchmark_pose_recorder" --ros-args \
  -p topic:=/pcl_pose -p output_path:="$OUT/$NAME.poses.csv" -p qos_reliability:=reliable \
  -p qos_durability:=transient_local -p use_sim_time:=true > "$OUT/$NAME.poserec.log" 2>&1 &
PIDS+=($!)
sleep 2

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

# /tf_static depth MUST exceed the number of /tf_static messages in the bag (4 for
# curtmini, 1 for bunker): it is transient_local, so a late subscriber only gets the
# last `depth` retained samples. At depth 1 the localizer receives one of the four
# transforms and rejects every scan with "Could not transform os_lidar to
# base_link_curt", which reads as a missing extrinsic rather than a QoS problem.
QOS=/tmp/jetson_bag_test_native_qos.yaml
cat > "$QOS" <<EOF
$CLOUD:
  history: keep_last
  depth: 5
  reliability: best_effort
  durability: volatile
$IMU:
  history: keep_last
  depth: 100
  reliability: best_effort
  durability: volatile
/tf_static:
  history: keep_last
  depth: 100
  reliability: reliable
  durability: transient_local
EOF

echo "playing at rate 1.0${DUR:+ (first ${DUR}s)}..."
if [ -n "$DUR" ]; then
  timeout "$DUR" ros2 bag play "$BAG" -s mcap --topics "$CLOUD" "$IMU" /tf_static \
    --clock --rate 1.0 --qos-profile-overrides-path "$QOS" > "$OUT/$NAME.play.log" 2>&1 || true
else
  ros2 bag play "$BAG" -s mcap --topics "$CLOUD" "$IMU" /tf_static \
    --clock --rate 1.0 --qos-profile-overrides-path "$QOS" > "$OUT/$NAME.play.log" 2>&1
fi

last=0; idle=0
while [ $idle -lt 10 ]; do
  sleep 2
  cur=$(wc -l < "$OUT/$NAME.status.csv" 2>/dev/null || echo 0)
  if [ "$cur" -gt "$last" ]; then last=$cur; idle=0; else idle=$((idle+2)); fi
done
kill "$SAMPLER_PID" 2>/dev/null || true
echo
python3 "$WS/scripts/analysis/throughput_summary.py" "$OUT/$NAME.status.csv"
awk -F, -v hz="$(getconf CLK_TCK)" -v ncpu="$(nproc)" 'NR==2{t0=$1; c0=$2} NR>1{if($3>rss)rss=$3; t1=$1; c1=$2}
  END{if(t1>t0) printf "node CPU: %.2f cores avg over %.0f s (of %d visible); peak RSS %.0f MB\n",
      (c1-c0)/hz/(t1-t0), t1-t0, ncpu, rss/1024}' "$CPU_CSV"
echo "files -> $OUT/$NAME.*"
