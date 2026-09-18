#!/usr/bin/env bash
# Profile the SCovox mapping NODE (CPU / RSS / scan rate) on a recorded bag.
#
# Usage:
#   scripts/cyclonedds_study/scovox_bag_test.sh [bunker|curtmini] [run_name] [duration_s]
#   PIN_CORES=4 scripts/cyclonedds_study/scovox_bag_test.sh bunker sc_pin1
#   SCOVOX_SET="resolution:=0.20 carve_band:=2.0" scripts/... bunker sc_band2
#
# WHAT RUNS, AND WHY
#
# The bag carries /hesai/points + /imu/data + /tf_static and NOTHING ELSE --
# there is no /tf, so no odometry. SCovox needs a pose per scan or it silently
# drops the scan (scovox_node.cpp:1960/1970 both `return` on TF failure), so an
# external localizer has to run alongside. That is hmr_localisation's
# lidar_localization_ros2 with registration_method SMALL_VGICP, wired exactly as
# scripts/jetson_bag_test_native.sh wires it -- same seed pose, same map, same
# scan_channel_count=128, same QoS overrides. Its own measured cost on this box
# is 0.80 cores / 161 MB / 9.77 Hz, so it is sampled here too and reported
# separately: the two processes share the box and the SCovox number is only
# honest next to what else was running.
#
# THE FRAME THAT MATTERS
#
# The localizer publishes map->odom. The odom->base_link edge is a STATIC
# IDENTITY from static_transform_publisher (same as the native harness). So ALL
# platform motion lives in map->odom, and integrating in `odom` -- which is what
# lidar_mapping.yaml ships -- would pile every scan on top of itself at the
# origin: a degenerate one-room map, and an unrepresentative CPU number because
# the voxel grid never grows. integration_frame is therefore overridden to
# `map` here. This is the single most important override in this script.
#
# Everything else about the environment (Humble not Jazzy, ~/ros-extra for the
# mcap storage plugin, small_gicp on LD_LIBRARY_PATH, play the .mcap FILE with
# -s mcap plus --qos-profile-overrides-path, /tf_static depth 100) is inherited
# from scripts/jetson_bag_test_native.sh; see that file for the reasoning.
set -e
WS=/home/jetsondevkit/jetbot-slam/hmr_localisation
SCOVOX_WS=/home/jetsondevkit/scovox_new_experiments/scovox
# No `set -u`: /opt/ros/humble/setup.bash reads AMENT_TRACE_SETUP_FILES unguarded.
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"
source "$SCOVOX_WS/install/setup.bash"
export AMENT_PREFIX_PATH=/home/jetsondevkit/ros-extra/prefix/opt/ros/humble:$AMENT_PREFIX_PATH
export LD_LIBRARY_PATH=/home/jetsondevkit/ros-extra/prefix/opt/ros/humble/lib:/home/jetsondevkit/ros-extra/prefix/opt/ros/humble/lib/aarch64-linux-gnu:/home/jetsondevkit/small_gicp-install/lib:$LD_LIBRARY_PATH
export FASTRTPS_DEFAULT_PROFILES_FILE="${SHM_PROFILE:-$WS/config/fastdds_shm.xml}"
export RCUTILS_COLORIZED_OUTPUT=0

PROFILE="${1:-bunker}"
NAME="${2:-scovox_${PROFILE}_$(date +%Y%m%d_%H%M%S)}"
DUR="${3:-}"
case "$PROFILE" in
  bunker)
    BAG="$WS/bags/bunker_jetson/bunker_jetson_0.mcap"
    CLOUD=/hesai/points; IMU=/imu/data; BASE=base_link
    SEED=(7.82 -4.11 -1.17 0.8550024006656964 0.5186240399903342)
    SET="scan_channel_count=128 ${SET:-}" ;;
  curtmini)
    BAG="$WS/bags/curtmini_jetson/curtmini_jetson_0.mcap"
    CLOUD=/ouster/points; IMU=/curt/imu/data; BASE=base_link_curt
    SEED=(8.33 -3.78 -1.39 0.853050270749362 0.5218287416139898) ;;
  *) echo "unknown bag profile '$PROFILE' (bunker|curtmini)"; exit 1 ;;
esac
OUT="$WS/output/scovox_test"
mkdir -p "$OUT"
[ -f "$BAG" ] || { echo "no bag at $BAG"; exit 1; }

SCOVOX_CFG="$SCOVOX_WS/install/scovox_mapping/share/scovox_mapping/config/lidar_mapping.yaml"
[ -f "$SCOVOX_CFG" ] || { echo "no scovox config at $SCOVOX_CFG"; exit 1; }

# Reap leftovers FIRST: a surviving node holds the SHM port locks and
# re-publishes latched state into the next run. Brackets stop pgrep matching
# this script's own command line.
for _p in "scovox_mapping_nod[e]" "lidar_localization_nod[e]" \
          "static_transform_publishe[r]" "ros2 bag pla[y]"; do
  pgrep -f "$_p" | xargs -r kill -9 2>/dev/null
done
sleep 3
command -v fastdds > /dev/null && \
  echo "shm: $(timeout 30 fastdds shm clean 2>/dev/null | tr '\n' ' ' | sed 's/  */ /g')"

echo "cores visible: $(nproc); affinity: $(taskset -pc $$ 2>/dev/null | sed 's/.*: //')"
echo "bag: $BAG"

# ---- localizer config: same construction as jetson_bag_test_native.sh --------
CFG=/tmp/scovox_bag_test_loc.yaml
sed -e "s/^\(\s*initial_pose_x:\).*/\1 ${SEED[0]}/" \
    -e "s/^\(\s*initial_pose_y:\).*/\1 ${SEED[1]}/" \
    -e "s/^\(\s*initial_pose_z:\).*/\1 ${SEED[2]}/" \
    -e 's/^\(\s*initial_pose_qx:\).*/\1 0.0/' \
    -e 's/^\(\s*initial_pose_qy:\).*/\1 0.0/' \
    -e "s/^\(\s*initial_pose_qz:\).*/\1 ${SEED[3]}/" \
    -e "s/^\(\s*initial_pose_qw:\).*/\1 ${SEED[4]}/" \
    -e "s/^\(\s*base_frame_id:\).*/\1 $BASE/" \
    "$WS/config/gt_ouster_ndt_tree_realtime.yaml" > "$CFG"
MAP="${MAP:-gt_map/gt_map_us050.pcd}"
case "$MAP" in /*) MAP_ABS="$MAP" ;; *) MAP_ABS="$WS/$MAP" ;; esac
[ -f "$MAP_ABS" ] || { echo "no map at $MAP_ABS"; exit 1; }
SET="map_path=\"$MAP_ABS\" ${SET:-}"
for kv in ${SET:-}; do
  k="${kv%%=*}"; v="${kv#*=}"
  grep -q "^\s*$k:" "$CFG" || { echo "override '$k' is not a parameter in the config"; exit 1; }
  sed -i "s|^\(\s*$k:\)[^#]*\(#.*\)\?$|\1 $v  \2|" "$CFG"
  echo "loc override: $k = $v"
done
echo "registration: $(grep -E '^\s*registration_method:' "$CFG" | sed 's/.*: *//')"

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

# PIN_CORES pins the two NODES only, never the bag player: affinity is inherited
# across fork, so wrapping the whole script would put the player on the same core
# and depress the measured rate with contention a real robot would not have.
# PIN_CORES pins the LOCALIZER; SCOVOX_PIN pins the node under test. They are
# separate on purpose: the localizer wants ~2.1 cores, so pinning both to the
# same core starves it and the poses -- and therefore scovox's scan rate --
# degrade for a reason that has nothing to do with scovox.
NODE_TASKSET=""
if [ -n "${PIN_CORES:-}" ]; then
  NODE_TASKSET="taskset -c $PIN_CORES"
  echo "pinning localizer to core(s) $PIN_CORES"
fi
SCOVOX_TASKSET=""
if [ -n "${SCOVOX_PIN:-}" ]; then
  SCOVOX_TASKSET="taskset -c $SCOVOX_PIN"
  echo "pinning scovox to core(s) $SCOVOX_PIN (player unpinned)"
fi

# ---- odometry source: hmr_localisation, SMALL_VGICP -------------------------
"$(ros2 pkg prefix tf2_ros)/lib/tf2_ros/static_transform_publisher" --x 0 --y 0 --z 0 \
  --frame-id odom --child-frame-id "$BASE" > /dev/null 2>&1 &
PIDS+=($!)
$NODE_TASKSET ros2 launch lidar_localization_ros2 lidar_localization.launch.py \
  localization_param_dir:="$CFG" cloud_topic:="$CLOUD" imu_topic:="$IMU" \
  use_sim_time:=true global_frame_id:=map odom_frame_id:=odom base_frame_id:="$BASE" \
  use_imu_preintegration:=true imu_preintegration_use_base_frame_transform:=true \
  publish_lidar_tf:=false publish_imu_tf:=false > "$OUT/$NAME.loc.log" 2>&1 &
PIDS+=($!)
LAUNCH_PID=$!
echo "waiting for map load + localizer activation..."
for _ in $(seq 1 300); do grep -aq "Activating end" "$OUT/$NAME.loc.log" && break; sleep 1; done
grep -aq "Activating end" "$OUT/$NAME.loc.log" || { echo "localizer failed to activate:"; tail -25 "$OUT/$NAME.loc.log"; exit 1; }
grep -a "registration_method\|scan_channel_count\|Map Size" "$OUT/$NAME.loc.log" | sed 's/.*\]: //'
LOC_PID=$(pgrep -P "$LAUNCH_PID" -f lidar_localization_node | head -1)

# ---- the node under test ----------------------------------------------------
# integration_frame:=map -- see the header. base_frame stays base_link: that is
# the observer pose scovox needs for its second lookup, and map->base_link
# resolves through the localizer's map->odom plus the static identity.
# The binary is exec'd directly, not via `ros2 run`: the python wrapper does not
# forward TERM and pgrep then matches the wrapper instead of the node.
SCOVOX_BIN="$SCOVOX_WS/install/scovox_mapping/lib/scovox_mapping/scovox_mapping_node"
[ -x "$SCOVOX_BIN" ] || { echo "no scovox binary at $SCOVOX_BIN"; exit 1; }
$SCOVOX_TASKSET "$SCOVOX_BIN" --ros-args --params-file "$SCOVOX_CFG" \
  -p use_sim_time:=true \
  -p input_pointcloud_topic:="$CLOUD" \
  -p integration_frame:=map \
  -p base_frame:="$BASE" \
  ${SCOVOX_SET:+$(for kv in $SCOVOX_SET; do printf -- '-p %s ' "$kv"; done)} \
  > "$OUT/$NAME.scovox.log" 2>&1 &
SCOVOX_PID=$!
PIDS+=($SCOVOX_PID)
sleep 3
kill -0 "$SCOVOX_PID" 2>/dev/null || { echo "scovox node died at startup:"; tail -25 "$OUT/$NAME.scovox.log"; exit 1; }
echo "scovox pid $SCOVOX_PID, localizer pid ${LOC_PID:-<none>}"

# ---- samplers: one per process, 2 s, /proc utime+stime and VmRSS ------------
sample_proc() {  # $1 = pid, $2 = csv path
  echo "time_sec,cpu_ticks,rss_kb" > "$2"
  while [ -r "/proc/$1/stat" ]; do
    printf '%s,%s,%s\n' "$(date +%s.%N)" \
      "$(awk '{print $14+$15}' "/proc/$1/stat" 2>/dev/null)" \
      "$(awk '/^VmRSS:/{print $2}' "/proc/$1/status" 2>/dev/null)" >> "$2"
    sleep 2
  done
}
sample_proc "$SCOVOX_PID" "$OUT/$NAME.scovox.cpu.csv" & SAMP_S=$!
SAMP_L=""
[ -n "$LOC_PID" ] && { sample_proc "$LOC_PID" "$OUT/$NAME.loc.cpu.csv" & SAMP_L=$!; }

# /tf_static depth MUST exceed the bag's /tf_static message count: it is
# transient_local, so a late subscriber only gets the last `depth` samples, and
# a short depth reads as a missing extrinsic rather than a QoS problem.
QOS=/tmp/scovox_bag_test_qos.yaml
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
PLAY=(ros2 bag play "$BAG" -s mcap --topics "$CLOUD" "$IMU" /tf_static
      --clock --rate 1.0 --qos-profile-overrides-path "$QOS")
if [ -n "$DUR" ]; then
  timeout "$DUR" "${PLAY[@]}" > "$OUT/$NAME.play.log" 2>&1 || true
else
  "${PLAY[@]}" > "$OUT/$NAME.play.log" 2>&1
fi

# Drain: wait until scovox stops logging scans.
last=0; idle=0
while [ $idle -lt 10 ]; do
  sleep 2
  cur=$(grep -ac 'recv=' "$OUT/$NAME.scovox.log" 2>/dev/null || echo 0)
  if [ "$cur" -gt "$last" ]; then last=$cur; idle=0; else idle=$((idle+2)); fi
done
kill "$SAMP_S" ${SAMP_L:-} 2>/dev/null || true
echo
python3 "$(dirname "$0")/scovox_summary.py" "$OUT/$NAME"
echo "files -> $OUT/$NAME.*"
