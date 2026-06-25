#!/usr/bin/env bash
# Multi-robot localization demo against the shared gt_map.ply.
#
# We only have ONE robot's bag, so robot2 is the SAME platform replayed
# `--start-offset OFFSET` seconds ahead on the route. Both robots localize into
# the shared `map` frame; scripts/relative_pose.py reports robot2's pose relative
# to robot1 (a genuine, non-zero, time-varying baseline). Swap in a real second
# bag later by pointing robot2's bag play / topics at it -- nothing else changes.
#
# Per robot:  bag play --> /<r>/points_raw --> frame_relay (restamp frame_id)
#             --> /<r>/points --> namespaced localizer --> map -> <r>/os_lidar
#
# Usage (on the HOST):
#   docker compose up -d
#   docker compose exec ros bash /ws/scripts/run_multi_robot.sh [offset_s] [duration_s]
# Defaults: offset_s=60, duration=full bag (robot1). Outputs land in /ws/output/.
set -e
source /opt/ros/jazzy/setup.bash
source /ws/install/setup.bash
cd /ws

OFFSET="${1:-60}"          # seconds robot2 leads robot1 on the route
DUR="${2:-}"               # optional playback duration (s) for both robots
BAG=bags/2026_06_19_18_19_06__kalhan-map-test-2_
SEED_CSV=output/path_final.csv
ROSTER=/tmp/robots_active.yaml
ROBOTS=(robot1 robot2)

DUR_ARG=""
[ -n "$DUR" ] && DUR_ARG="--playback-duration $DUR"

PIDS=()
cleanup() {
  echo "--- cleanup ---"
  kill "${PIDS[@]}" 2>/dev/null || true
  pkill -f lidar_localization_node 2>/dev/null || true
  pkill -f relative_pose.py 2>/dev/null || true
  pkill -f rosbag2_player 2>/dev/null || true
  pkill -f 'ros2 bag play' 2>/dev/null || true
}
trap cleanup EXIT

# 1) resolve robot2's seed from the validated trajectory at +OFFSET
python3 scripts/seed_robots.py config/robots.yaml "$SEED_CSV" "$OFFSET" "$ROSTER"

# 2) launch both namespaced localizers (wall clock; timing is msg-stamp driven)
ros2 launch launch/multi_robot_localization.launch.py \
  robots_config:="$ROSTER" > /tmp/multi.log 2>&1 &
PIDS+=($!)

# wait for every localizer to finish activating (each prints "Activating end"
# once its map is loaded). We read the launch log rather than `ros2 lifecycle
# get` / `ros2 node list`, whose graph queries are unreliable under FastDDS here.
echo "waiting for ${#ROBOTS[@]} localizers to activate (map load ~ a few s)..."
for _ in $(seq 1 180); do
  n_active="$(grep -ac 'lidar_localization]: Activating end' /tmp/multi.log 2>/dev/null || true)"
  [ "${n_active:-0}" -ge "${#ROBOTS[@]}" ] && break
  sleep 1
done
echo "  localizers active: ${n_active:-0}/${#ROBOTS[@]}"

# 4) relative-pose monitor: writes output/relative_pose.csv, the per-robot
#    output/path_<ns>.csv trajectories (from pcl_pose), and RViz markers
REL_CSV=/ws/output/relative_pose.csv TRAJ_DIR=/ws/output \
  python3 scripts/relative_pose.py "${ROBOTS[@]}" > /tmp/relpose.log 2>&1 &
REL=$!; PIDS+=($REL)
sleep 1

# 5) play the bag twice (wall clock, points only) straight into each robot's
# input topic. robot2 starts OFFSET s ahead. `timeout` is a safety net so a
# stuck player can never wedge the run.
PLAY_TO=$(( ${DUR:-330} + 60 ))
echo "playing bag: robot1 @ t0, robot2 @ +${OFFSET}s ${DUR:+(first ${DUR}s each)}"
timeout "$PLAY_TO" ros2 bag play "$BAG" --rate 1.0 \
  --topics /ouster/points --remap /ouster/points:=/robot1/points \
  $DUR_ARG > /tmp/play_robot1.log 2>&1 &
P1=$!; PIDS+=($P1)
timeout "$PLAY_TO" ros2 bag play "$BAG" --rate 1.0 --start-offset "$OFFSET" \
  --topics /ouster/points --remap /ouster/points:=/robot2/points \
  $DUR_ARG > /tmp/play_robot2.log 2>&1 &
P2=$!; PIDS+=($P2)
wait $P1 $P2 || true
echo "playback finished; settling..."
sleep 3

# 6) stop the monitor cleanly (SIGINT) so its CSVs flush + close
kill -INT "$REL" 2>/dev/null || true
for _ in $(seq 1 12); do kill -0 "$REL" 2>/dev/null || break; sleep 0.5; done

# 7) summary
echo "==================== SUMMARY ===================="
for ns in "${ROBOTS[@]}"; do
  n=$(($(wc -l < "/ws/output/path_${ns}.csv" 2>/dev/null || echo 1) - 1))
  echo "  /$ns tracked poses: ${n}"
done
python3 - <<'PY'
import csv, os
p = "/ws/output/relative_pose.csv"
if os.path.exists(p):
    rows = list(csv.DictReader(open(p)))
    if rows:
        rng = [float(r["range_m"]) for r in rows]
        print(f"  relative range robot1->robot2: "
              f"min={min(rng):.2f} m  max={max(rng):.2f} m  "
              f"mean={sum(rng)/len(rng):.2f} m  ({len(rng)} samples)")
        last = rows[-1]
        print(f"  final: range={last['range_m']} m  "
              f"dx={last['dx']} dy={last['dy']} dz={last['dz']} dyaw={last['dyaw_deg']} deg")
PY
echo "  trajectories: output/path_robot1.csv, output/path_robot2.csv"
echo "  relative pose log: output/relative_pose.csv"
echo "  plot on host: python3 scripts/plot_multi_robot.py"
echo "================================================="
