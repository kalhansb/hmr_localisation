#!/usr/bin/env bash
# GLIM counterpart to scripts/jetson_bag_test_native.sh.
#
# Deliberately mirrors that script so the two sets of numbers are comparable:
# same bags, same player at rate 1.0, same Fast-DDS SHM profile, same
# /proc/<pid>/stat CPU+RSS sampling, same 120 s window.
#
# Usage:
#   glim_bag_test.sh [curtmini|bunker] [odom_cpu|odom_gpu|full_cpu|full_gpu] \
#                    [run_name] [duration_s]
#
# ONE IMPORTANT DIFFERENCE FROM THE LOCALIZER HARNESS
# GlimROS::points_callback preprocesses synchronously and then calls
# odometry_estimation->insert_frame(), which pushes onto an UNBOUNDED async
# queue. hmr_localisation drops scans it cannot keep up with at the DDS layer
# (best_effort, small depth) and they are gone; GLIM can instead accept
# everything and fall behind, draining after the player stops. Counting every
# pose GLIM ever emits would therefore flatter it -- a run that finished 40 s
# late would score the same as one that kept up.
#
# So the summary counts only poses that ARRIVED WHILE THE PLAYER WAS RUNNING
# (wall clock), and separately reports the backlog drained afterwards plus the
# end-of-window lag. The recorder logs both the data stamp and the wall arrival
# time to make that possible.
set -e
WS="${WS:-$HOME/jetbot-slam/hmr_localisation}"
PREFIX="${GLIM_PREFIX:-$HOME/glim-install}"
SG="${SG:-$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)}"

# No `set -u`: ROS setup.bash reads AMENT_TRACE_SETUP_FILES unguarded.
source /opt/ros/humble/setup.bash
source $HOME/glim_ws/install/setup.bash
export AMENT_PREFIX_PATH=$HOME/ros-extra/prefix/opt/ros/humble:$AMENT_PREFIX_PATH
export LD_LIBRARY_PATH=$PREFIX/lib:$HOME/ros-extra/prefix/opt/ros/humble/lib:$HOME/ros-extra/prefix/opt/ros/humble/lib/aarch64-linux-gnu:$LD_LIBRARY_PATH
# Same transport as every hmr_localisation measurement, or the comparison is
# not about the mapper.
export FASTRTPS_DEFAULT_PROFILES_FILE="${SHM_PROFILE:-$WS/config/fastdds_shm.xml}"
export RCUTILS_COLORIZED_OUTPUT=0
[ "${RMW_IMPLEMENTATION:-}" = "rmw_cyclonedds_cpp" ] \
  && echo "cyclone uri: ${CYCLONEDDS_URI:-<none, Cyclone defaults>}" \
  || echo "fastdds profile: $FASTRTPS_DEFAULT_PROFILES_FILE"

PROFILE="${1:-curtmini}"
VARIANT="${2:-odom_cpu}"
NAME="${3:-${PROFILE}_${VARIANT}}"
DUR="${4:-120}"
case "$PROFILE" in
  curtmini) BAG="$WS/bags/curtmini_jetson/curtmini_jetson_0.mcap"
            CLOUD=/ouster/points; IMU=/curt/imu/data ;;
  bunker)   BAG="$WS/bags/bunker_jetson/bunker_jetson_0.mcap"
            CLOUD=/hesai/points;  IMU=/imu/data ;;
  *) echo "unknown bag profile '$PROFILE' (curtmini|bunker)"; exit 1 ;;
esac
CFG=$HOME/glim-config/${PROFILE}_${VARIANT}
[ -d "$CFG" ] || { echo "no config at $CFG"; exit 1; }
[ -f "$BAG" ] || { echo "no bag at $BAG"; exit 1; }
OUT=$HOME/glim-output
mkdir -p "$OUT"

for _p in "glim_rosnod[e]" "rosbag2_playe[r]" "glim_pose_recorde[r]"; do
  pgrep -f "$_p" | xargs -r kill -9 2>/dev/null
done
sleep 2
# Orphaned SHM segments make a fresh node hang before logging starts.
fastdds shm clean > /dev/null 2>&1 || true

echo "bag: $BAG"
echo "config: $CFG"
echo "variant: $VARIANT"
python3 - "$CFG" <<'PY'
import json, sys
c = sys.argv[1]
g = json.load(open(f"{c}/config_ros.json"))["glim_ros"]
t = json.load(open(f"{c}/config.json"))["global"]
s = json.load(open(f"{c}/config_sensors.json"))["sensors"]
p = json.load(open(f"{c}/config_preprocess.json"))["preprocess"]
o = json.load(open(f"{c}/{t['config_odometry']}"))["odometry_estimation"]
print(f"odometry: {t['config_odometry']} so={o['so_name']} threads={o.get('num_threads')}")
print(f"mapping: local={g['enable_local_mapping']} global={g['enable_global_mapping']}")
print(f"preprocess: downsample_res={p['downsample_resolution']} "
      f"random_target={p['random_downsample_target']} threads={p['num_threads']}")
print(f"T_lidar_imu: {s['T_lidar_imu']}")
PY

NODE_TASKSET=""
if [ -n "${PIN_CORES:-}" ]; then
  NODE_TASKSET="taskset -c $PIN_CORES"
  echo "pinning glim to core(s) $PIN_CORES (player/recorder unpinned)"
fi

PIDS=()
# Exec the binary directly rather than via `ros2 run`: that wrapper is a python
# process which stays alive as the parent, and `pgrep -f glim_rosnode | head -1`
# picks IT rather than the real node -- the first smoke run duly reported
# "0.00 cores / 23 MB RSS", which is the wrapper sitting idle. taskset execs in
# place, so $! is the node's own PID either way.
GLIM_BIN=$HOME/glim_ws/install/glim_ros/lib/glim_ros/glim_rosnode
[ -x "$GLIM_BIN" ] || { echo "no glim_rosnode at $GLIM_BIN"; exit 1; }
$NODE_TASKSET "$GLIM_BIN" --ros-args \
  -p config_path:="$CFG" > "$OUT/$NAME.node.log" 2>&1 &
NODE_PID=$!
PIDS+=($NODE_PID)

# Wait for the node to come up and load its modules before starting the player.
for i in $(seq 1 40); do
  grep -q "config_path:" "$OUT/$NAME.node.log" 2>/dev/null && break
  sleep 1
done
sleep 5

python3 "$SG/glim_pose_recorder.py" --ros-args \
  -p topic:=/glim_ros/lidar_pose -p output_path:="$OUT/$NAME.lidar_poses.csv" \
  > "$OUT/$NAME.rec_lidar.log" 2>&1 &
PIDS+=($!)
python3 "$SG/glim_pose_recorder.py" --ros-args -r __node:=glim_pose_recorder_imu \
  -p topic:=/glim_ros/pose -p output_path:="$OUT/$NAME.imu_poses.csv" \
  > "$OUT/$NAME.rec_imu.log" 2>&1 &
PIDS+=($!)
sleep 4

[ -r "/proc/$NODE_PID/stat" ] || { echo "glim_rosnode died"; tail -20 "$OUT/$NAME.node.log"; exit 1; }
echo "sampling pid $NODE_PID ($(tr -d '\0' < /proc/$NODE_PID/comm))"
CPU_CSV="$OUT/$NAME.cpu.csv"
echo "time_sec,cpu_ticks,rss_kb" > "$CPU_CSV"
(
  while [ -r "/proc/$NODE_PID/stat" ]; do
    printf '%s,%s,%s\n' "$(date +%s.%N)" \
      "$(awk '{print $14+$15}' "/proc/$NODE_PID/stat")" \
      "$(awk '/^VmRSS:/{print $2}' "/proc/$NODE_PID/status")" >> "$CPU_CSV"
    sleep 2
  done
) &
SAMPLER_PID=$!

QOS=/tmp/glim_bag_test_qos.yaml
cat > "$QOS" <<EOF
$CLOUD:
  history: keep_last
  depth: 5
  reliability: best_effort
  durability: volatile
$IMU:
  history: keep_last
  depth: 2000
  reliability: reliable
  durability: volatile
/tf_static:
  history: keep_last
  depth: 100
  reliability: reliable
  durability: transient_local
EOF

echo "playing at rate 1.0 (first ${DUR}s)..."
PLAY_START=$(date +%s.%N)
timeout "$DUR" ros2 bag play "$BAG" -s mcap --topics "$CLOUD" "$IMU" /tf_static \
  --clock --rate 1.0 --qos-profile-overrides-path "$QOS" \
  > "$OUT/$NAME.play.log" 2>&1 || true
PLAY_END=$(date +%s.%N)
echo "player finished; letting the queue drain for 20 s to size the backlog..."
sleep 20

kill "$SAMPLER_PID" 2>/dev/null || true
for p in "${PIDS[@]}"; do kill -INT "$p" 2>/dev/null || true; done
sleep 3
for _p in "glim_rosnod[e]" "glim_pose_recorde[r]"; do
  pgrep -f "$_p" | xargs -r kill -9 2>/dev/null
done

echo
python3 "$SG/glim_summary.py" "$OUT/$NAME.lidar_poses.csv" "$PLAY_START" "$PLAY_END"
awk -F, -v hz="$(getconf CLK_TCK)" -v ncpu="$(nproc)" -v t_end="$PLAY_END" '
  NR==2{t0=$1; c0=$2}
  NR>1{if($3>rss)rss=$3; t1=$1; c1=$2; if($1<=t_end){t1w=$1; c1w=$2}}
  END{
    if(t1>t0) printf "node CPU: %.2f cores avg over %.0f s (of %d visible); peak RSS %.0f MB\n",
      (c1-c0)/hz/(t1-t0), t1-t0, ncpu, rss/1024;
    if(t1w>t0) printf "node CPU (playback window only): %.2f cores over %.0f s\n",
      (c1w-c0)/hz/(t1w-t0), t1w-t0;
  }' "$CPU_CSV"
grep -ciE "warn|error" "$OUT/$NAME.node.log" | xargs -I{} echo "node log warn/error lines: {}"
echo "files -> $OUT/$NAME.*"
