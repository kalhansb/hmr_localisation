#!/usr/bin/env bash
# Sequentially benchmark NDT on each map resolution (one localizer at a time).
RATE="${1:-0.5}"; BAGWIN="${2:-120}"
echo "tag,map,pts,accepted,rejected,poses" > /ws/output/dsbench/run_index.csv

# tag  map_pcd
MAPS="
full   /ws/gt_map/gt_map.pcd
us0001 /ws/gt_map/gt_map_us0001.pcd
us005  /ws/gt_map/gt_map_us005.pcd
us010  /ws/gt_map/gt_map_us010.pcd
us020  /ws/gt_map/gt_map_us020.pcd
us030  /ws/gt_map/gt_map_us030.pcd
"
echo "$MAPS" | while read -r tag map; do
  [ -z "$tag" ] && continue
  bash /ws/output/dsbench/run_one.sh "$tag" "$map" "$RATE" "$BAGWIN"
  sleep 5      # let DDS discovery tear down before the next run
done
chown -R 1001:1001 /ws/output/dsbench/runs /ws/output/dsbench/cfg /ws/output/dsbench/run_index.csv 2>/dev/null
echo "ALL_DSBENCH_DONE"
