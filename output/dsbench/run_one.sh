#!/usr/bin/env bash
# Run NDT localization on one map; capture CPU/mem (/proc), trajectory (/path), fitness (log).
# Usage: run_one.sh <TAG> <MAP_PCD> <RATE> <BAG_WINDOW_S>
source /opt/ros/jazzy/setup.bash
source /ws/install/setup.bash
# Force multi-MB Ouster clouds over Shared Memory (UDP loopback w/ 208KB buffers
# throttles them to ~0.4Hz). Applies to both the localizer and the bag player.
export FASTRTPS_DEFAULT_PROFILES_FILE=/ws/output/dsbench/fastdds_shm.xml
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
cd /ws

TAG="$1"; MAP="$2"; RATE="${3:-0.5}"; BAGWIN="${4:-120}"
WALL=$(python3 -c "print(int($BAGWIN/$RATE))")          # wall seconds to play
D=/ws/output/dsbench/runs/$TAG
CFG=/ws/output/dsbench/cfg/$TAG.yaml
mkdir -p "$D"

# per-map config = base NDT config with map_path swapped
# BASECFG env var lets a caller swap in a tuned base (e.g. a fast/real-time config)
BASECFG="${BASECFG:-/ws/config/gt_ouster_ndt.yaml}"
sed "s#^\( *map_path: \).*#\1\"$MAP\"#" "$BASECFG" > "$CFG"
# optional 5th arg: raise the NDT fitness reject threshold (real-time robustness)
# (score_threshold is a ROS 'double' param -> must carry a decimal point)
SCORE="${5:-}"
[ -n "$SCORE" ] && { case "$SCORE" in *.*) ;; *) SCORE="$SCORE.0";; esac; \
  sed -i "s#^\( *score_threshold: \).*#\1$SCORE#" "$CFG"; }
MAPPTS=$(grep -a -m1 '^POINTS' "$MAP" | awk '{print $2}')
echo "[$TAG] map=$MAP pts=$MAPPTS rate=$RATE bagwin=${BAGWIN}s wall=${WALL}s score=${SCORE:-default}"

# 1) launch localizer
ros2 launch lidar_localization_ros2 lidar_localization.launch.py \
  localization_param_dir:="$CFG" \
  cloud_topic:=/ouster/points imu_topic:=/imu/data use_sim_time:=true \
  global_frame_id:=map base_frame_id:=os_lidar lidar_frame_id:=os_lidar \
  publish_lidar_tf:=false use_imu_preintegration:=false > "$D/loc.log" 2>&1 &
LOC=$!

# wait for activation (map load can take a while for the full map)
for i in $(seq 1 120); do
  grep -aq "Activating end" "$D/loc.log" && break
  kill -0 $LOC 2>/dev/null || { echo "[$TAG] localizer died during load"; break; }
  sleep 1
done
sleep 3
PID=$(pgrep -f 'lidar_localization_node' | head -1)
echo "[$TAG] node pid=$PID active after $(grep -ac Activating "$D/loc.log") activation lines"

# 2) start /proc sampler
python3 /ws/output/dsbench/sample_proc.py "$PID" 0.5 "$D/proc.csv" &
SAMP=$!

# 3) play the ouster-only filtered bag (full 54GB bag throttles the player), bounded by wall clock
timeout ${WALL}s ros2 bag play /ws/bags/maptest2_ouster \
  --clock --rate "$RATE" >> "$D/loc.log" 2>&1

# 4) dump latched /path trajectory
python3 /ws/scripts/fetch_path.py "$D/pose.csv" /path >> "$D/loc.log" 2>&1

# 5) teardown
kill $SAMP 2>/dev/null
kill $LOC 2>/dev/null
sleep 2
pkill -f 'lidar_localization_node' 2>/dev/null
pkill -f 'ros2 bag play' 2>/dev/null

# 6) fitness summary from log (lines are prefixed by [lidar_localization_node-1])
grep -aoE 'fitness score: [-0-9.eE+]+' "$D/loc.log" | awk '{print $3}' > "$D/fitness.txt"
NACC=$(wc -l < "$D/fitness.txt")
NREJ=$(grep -ac 'fitness score is over' "$D/loc.log")
NPOSE=$(($(wc -l < "$D/pose.csv") - 1))
echo "[$TAG] DONE  accepted=$NACC rejected=$NREJ poses=$NPOSE  (pts=$MAPPTS)"
echo "$TAG,$MAP,$MAPPTS,$NACC,$NREJ,$NPOSE" >> /ws/output/dsbench/run_index.csv
