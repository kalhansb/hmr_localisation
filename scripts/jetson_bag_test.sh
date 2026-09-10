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
    SEED=(7.82 -4.11 -1.17 0.8550024006656964 0.5186240399903342) ;;
  *) echo "unknown bag profile '$PROFILE' (curtmini|bunker)"; exit 1 ;;
esac
OUT=/ws/output/jetson_test
mkdir -p "$OUT"
[ -f "$BAG/metadata.yaml" ] || { echo "no bag at $BAG"; exit 1; }

# What the container can actually use.  A cpuset or CFS quota shows up here first; the
# node clamps its OpenMP pool to the cores it sees, so check these before tuning.
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

PIDS=()
cleanup() {
  kill -TERM "${PIDS[@]}" 2>/dev/null || true
  sleep 2
  kill -KILL "${PIDS[@]}" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

ros2 run tf2_ros static_transform_publisher --x 0 --y 0 --z 0 \
  --frame-id odom --child-frame-id "$BASE" > /dev/null 2>&1 &
PIDS+=($!)
ros2 launch lidar_localization_ros2 lidar_localization.launch.py \
  localization_param_dir:="$CFG" cloud_topic:="$CLOUD" imu_topic:="$IMU" \
  use_sim_time:=true global_frame_id:=map odom_frame_id:=odom base_frame_id:="$BASE" \
  use_imu_preintegration:=true imu_preintegration_use_base_frame_transform:=true \
  publish_lidar_tf:=false publish_imu_tf:=false > "$OUT/$NAME.log" 2>&1 &
PIDS+=($!)
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

echo "playing $BAG at rate 1.0${DUR:+ (first ${DUR}s)}..."
ros2 bag play "$BAG" --topics "$CLOUD" "$IMU" /tf_static --clock --rate 1.0 ${DUR:+--playback-duration $DUR} > /dev/null 2>&1

# Let the node drain any queued backlog before reading the numbers.
last=0; idle=0
while [ $idle -lt 10 ]; do
  sleep 2
  cur=$(wc -l < "$OUT/$NAME.status.csv" 2>/dev/null || echo 0)
  if [ "$cur" -gt "$last" ]; then last=$cur; idle=0; else idle=$((idle+2)); fi
done
echo
python3 /ws/scripts/analysis/throughput_summary.py "$OUT/$NAME.status.csv"
echo "files -> $OUT/$NAME.*"
