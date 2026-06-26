#!/usr/bin/env bash
# End-to-end scovox-vs-GT experiment, unattended:
#   1) launch run_loc_scovox_tree.sh (localizer tree + scovox + rviz + bag)
#   2) wait for the bag to finish
#   3) clock_capture.py — un-freeze scovox's sim-time timer and grab the final map
#   4) compare_maps.py  — score the captured map against the GT map
#   5) kill the run
#
# The captured map lands at /ws/output/scovox_map.npy (overwrites). Back up any
# prior run first. Pass a tag as $1 to also copy the result PNG/npy with a suffix.
set +e
TAG="${1:-}"
source /opt/ros/jazzy/setup.bash
source /ws/install/setup.bash
export DISPLAY="${DISPLAY:-:1}"

RUNLOG=/tmp/run_exp.log
: > "$RUNLOG"
echo "[exp] launching run_loc_scovox_tree.sh (full bag, rate 1.0)..."
bash /ws/scripts/scovox/run_loc_scovox_tree.sh > "$RUNLOG" 2>&1 &
RUN_PID=$!

# Wait for bag completion (bag is ~500 s; allow generous headroom).
echo "[exp] waiting for bag to finish..."
for i in $(seq 1 800); do
  grep -qa "bag finished" "$RUNLOG" && { echo "[exp] bag finished at ~${i}s"; break; }
  kill -0 "$RUN_PID" 2>/dev/null || { echo "[exp] run process exited early"; break; }
  sleep 1
done
sleep 3

echo "[exp] === final loc stats ==="
grep -aoE "Accepted|accepted|reject" /tmp/loc.log | sort | uniq -c 2>/dev/null
grep -aE "score|fitness" /tmp/loc.log | tail -3 2>/dev/null
echo "[exp] === scovox gate stats (last frame line) ==="
grep -aE "gated=|rearm=" /tmp/scovox.log | tail -3
grep -aE "Runtime TF jump" /tmp/scovox.log | tail -8
echo "[exp] rearm count total:"; grep -ac "Runtime TF jump" /tmp/scovox.log

echo "[exp] === capturing final map ==="
python3 /ws/scripts/scovox/clock_capture.py

if [ -n "$TAG" ] && [ -f /ws/output/scovox_map.npy ]; then
  cp /ws/output/scovox_map.npy "/ws/output/scovox_map_${TAG}.npy"
fi

echo "[exp] === comparing to GT ==="
python3 /ws/scripts/scovox/compare_maps.py

if [ -n "$TAG" ]; then
  [ -f /ws/output/map_vs_gt.png ] && cp /ws/output/map_vs_gt.png "/ws/output/map_vs_gt_${TAG}.png"
  [ -f /ws/output/scovox_unmatched.npy ] && cp /ws/output/scovox_unmatched.npy "/ws/output/scovox_unmatched_${TAG}.npy"
fi

echo "[exp] === cleanup ==="
kill "$RUN_PID" 2>/dev/null
pkill -f run_loc_scovox_tree 2>/dev/null
pkill -f lidar_localization_node 2>/dev/null
pkill -f scovox_mapping_node 2>/dev/null
pkill -f rviz2 2>/dev/null
pkill -f static_transform_publisher 2>/dev/null
pkill -f ekf_odom 2>/dev/null          # the `ros2 launch ekf_odom` parent
pkill -f ekf_node 2>/dev/null
pkill -f ndt_pose_relay 2>/dev/null
echo "[exp] DONE"
