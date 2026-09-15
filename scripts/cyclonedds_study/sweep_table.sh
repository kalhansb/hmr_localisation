#!/usr/bin/env bash
# GLIM vs hmr_localisation, regenerated under the user's CycloneDDS config
# instead of the Fast-DDS SHM profile. Everything else matches docs/glim_comparison.md:
# pinned to core 0, 120 s at rate 1.0, both systems TUNED, n=3 per row.
#
#   hmr_loc  -> ndt_num_threads=1, scan_channel_stride 4 and 8
#   GLIM     -> o_combo (odom CPU), o_combogpu (odom GPU), o_combofull (odom+local+global mapping)
#
# GLIM rows run FIRST as a canary: if the Cyclone config breaks GLIM (reliable IMU
# at depth 2000, WhcHigh=500kB, 65500B datagrams) it shows up in ~4 min, not after
# the hmr rows have burned 30.
WS="${WS:-$HOME/jetbot-slam/hmr_localisation}"
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SG="${SG:-$HERE/results}"
mkdir -p "$SG"
RESULTS="$SG/table_results.txt"
: > "$RESULTS"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://$HERE/cyc_user.xml"
{
  echo "rmw: $RMW_IMPLEMENTATION"
  echo "uri: $CYCLONEDDS_URI"
  echo "config md5: $(md5sum $HERE/cyc_user.xml | cut -d' ' -f1)"
  echo "rmem_max: $(sysctl -n net.core.rmem_max)"
} | tee -a "$RESULTS"

cd "$WS" || exit 1

for bag in bunker curtmini; do
  p=bunk; [ "$bag" = curtmini ] && p=curt
  # ---- GLIM rows ----
  for v in o_combo o_combogpu o_combofull; do
    for i in 1 2 3; do
      n="cyc_glim_${p}_${v}_r${i}"
      echo "=========== $n ===========" | tee -a "$RESULTS"
      rm -rf /tmp/dump 2>/dev/null
      if PIN_CORES=0 timeout 600 "$HERE/glim_bag_test.sh" "$bag" "$v" "$n" 120 \
           > "$SG/$n.log" 2>&1; then
        grep -aE "^cyclone uri:|^run |^${n} |node CPU|warn/error" "$SG/$n.log" | tee -a "$RESULTS"
      else
        echo "  FAILED"; tail -6 "$SG/$n.log" | sed 's/^/    /' | tee -a "$RESULTS"
      fi
      rm -rf /tmp/dump 2>/dev/null
      sleep 5
    done
  done
  # ---- hmr_localisation rows ----
  for s in 4 8; do
    for i in 1 2 3; do
      n="cyc_hmr_${p}_s${s}_r${i}"
      echo "=========== $n ===========" | tee -a "$RESULTS"
      if PIN_CORES=0 SET="scan_channel_stride=$s ndt_num_threads=1" \
         timeout 900 ./scripts/jetson_bag_test_native.sh "$bag" "$n" 120 \
           > "$SG/$n.log" 2>&1; then
        grep -aE "^cyclone uri:|^${n} |node CPU" "$SG/$n.log" | tee -a "$RESULTS"
      else
        echo "  FAILED"; tail -6 "$SG/$n.log" | sed 's/^/    /' | tee -a "$RESULTS"
      fi
      sleep 5
    done
  done
done
echo "===== TABLE SWEEP COMPLETE =====" | tee -a "$RESULTS"
