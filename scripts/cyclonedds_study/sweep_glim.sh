#!/usr/bin/env bash
# GLIM matrix: both bags x {odometry-only, full stack} x {CPU, GPU backend}.
#
# odom_* is the like-for-like comparison against hmr_localisation: per-scan
# registration against a local voxel map, no mapping, no loop closure.
# full_*  is GLIM as you would actually deploy it (odometry + sub mapping +
#         global mapping), which the localizer has no analogue for.
WS="${WS:-$HOME/jetbot-slam/hmr_localisation}"
cd "$WS" || exit 1
SG="${SG:-$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)}"
RESULTS="$SG/glim_results.txt"
: > "$RESULTS"

for bag in curtmini bunker; do
  p=curt; [ "$bag" = bunker ] && p=bunk
  for v in odom_cpu odom_gpu full_cpu full_gpu; do
    name="glim_${p}_${v}"
    echo "=========== $name ===========" | tee -a "$RESULTS"
    rm -rf /tmp/dump 2>/dev/null
    if timeout 600 "$SG/glim_bag_test.sh" "$bag" "$v" "$name" 120 \
         > "$SG/${name}.log" 2>&1; then
      grep -aE "^odometry:|^mapping:|^run |^${name} |node CPU|warn/error" \
        "$SG/${name}.log" | tee -a "$RESULTS"
    else
      echo "  FAILED"; tail -6 "$SG/${name}.log" | sed 's/^/    /' | tee -a "$RESULTS"
    fi
    rm -rf /tmp/dump 2>/dev/null
    sleep 5
  done
done
echo "===== GLIM SWEEP COMPLETE =====" | tee -a "$RESULTS"
