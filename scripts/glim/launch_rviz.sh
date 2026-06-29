#!/usr/bin/env bash
# Detach the map publisher + RViz inside the container so they survive this exec.
# Visualises the saved maps (no rerun). Logs -> /tmp/viz_pub.log, /tmp/rviz.log
set -e
source /opt/ros/jazzy/setup.bash
source /scovox/install_glim/setup.bash 2>/dev/null || true
export DISPLAY="${DISPLAY:-:1}"
SOFT="${1:-}"                       # pass "soft" to force software GL
[ "$SOFT" = "soft" ] && export LIBGL_ALWAYS_SOFTWARE=1 && echo "(software GL)"

setsid python3 -u /ws/scripts/glim/viz_publish_maps.py > /tmp/viz_pub.log 2>&1 < /dev/null &
echo "publisher pid=$!"
sleep 3
echo "---- publisher log ----"; cat /tmp/viz_pub.log

setsid rviz2 -d /ws/config/feed_cmp/scovox_glim.rviz > /tmp/rviz.log 2>&1 < /dev/null &
echo "rviz pid=$!"
sleep 6
echo "---- rviz log (first 25 lines) ----"; head -25 /tmp/rviz.log 2>/dev/null
echo "launch_rviz done."
