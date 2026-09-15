#!/usr/bin/env bash
# Seed-perturbation test: does hmr_localisation converge to the SAME map-frame
# trajectory from a deliberately wrong initial pose, or does it track the seed?
# If it converges, the map determines the pose and the shared frame is
# seed-independent. If it does not, "same frame" is an artifact of a good guess.
set -e
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="${WS:-$HOME/jetbot-slam/hmr_localisation}"
SG="${SG:-$HERE/results}"; mkdir -p "$SG"
CASES="${CASES:-$SG/seed_cases.txt}"
RESULTS="$SG/seed_results.txt"; : > "$RESULTS"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://$HERE/cyc_user.xml"
cd "$WS"
while read -r n bag x y qz qw; do
  [ -n "$n" ] || continue
  echo "=========== seed_$n  (x=$x y=$y qz=$qz) ===========" | tee -a "$RESULTS"
  if PIN_CORES=0 \
     SET="scan_channel_stride=4 ndt_num_threads=1 initial_pose_x=$x initial_pose_y=$y initial_pose_qz=$qz initial_pose_qw=$qw" \
     timeout 900 ./scripts/jetson_bag_test_native.sh "$bag" "seed_$n" 120 \
       > "$SG/seed_$n.log" 2>&1; then
    grep -aE "^override: initial|^seed_$n |node CPU" "$SG/seed_$n.log" | tee -a "$RESULTS"
  else
    echo "  FAILED"; tail -8 "$SG/seed_$n.log" | sed 's/^/    /' | tee -a "$RESULTS"
  fi
  sleep 5
done < "$CASES"
echo "DONE" | tee -a "$RESULTS"
