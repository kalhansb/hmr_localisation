#!/usr/bin/env bash
# Native-Humble localization + SCovox occupancy mapping, with the bag replayed
# from the Jazzy Docker container.
#
# Why this split:
#   The bag is mcap recorded on Jazzy (rosbag2 metadata v9) -> host Humble's
#   rosbag2 can't read it without the mcap plugin + a reindex. Instead we let the
#   Jazzy container (which reads it natively) replay it, and run everything else
#   natively on Humble (real GPU for RViz, reuse of the SCovox Humble build).
#
#   Cross-distro DDS (Jazzy 2.14 <-> Humble 2.6 Fast DDS) only works over UDP:
#   shared-memory segment formats differ, so we force UDPv4-only on BOTH ends via
#   config/fastdds_udp_only.xml. /ouster/points is also republished RELIABLE
#   (config/ouster_reliable_qos.yaml) for scovox's reliable subscriber.
#
# Layout:
#   container (Jazzy):  ros2 bag play  -> /ouster/points /imu/data /clock
#   host (Humble):      lidar_localization_ros2 (NDT vs gt_map.ply)
#                       scovox_mapping_node     (occupancy map in `map` frame)
#                       rviz2                    (combined view)
#
# Prereqs (once per login):  xhost +local:
# Usage (run on the HOST):
#   scripts/run_loc_scovox_local.sh [duration_s] [rate]
set -e
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PROFILE="$REPO/config/fastdds_udp_only.xml"
export FASTRTPS_DEFAULT_PROFILES_FILE="$PROFILE"
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
unset ROS_DOMAIN_ID            # domain 0, matches the container
: "${DISPLAY:=:1}"; export DISPLAY   # host GPU — no LIBGL_ALWAYS_SOFTWARE needed

DUR="${1:-}"; RATE="${2:-1.0}"
DUR_ARG=""; [ -n "$DUR" ] && DUR_ARG="--playback-duration $DUR"

source /opt/ros/humble/setup.bash
source "$REPO/install_humble/setup.bash"
source "$REPO/../scovox/install/setup.bash"

cleanup() { kill $LOC $SCV $RVIZ 2>/dev/null || true; }
trap cleanup EXIT INT TERM

# 0) bag player in the Jazzy container (UDP-only transport, reliable cloud)
( cd "$REPO" && DISPLAY="$DISPLAY" docker compose up -d >/dev/null )
docker compose -f "$REPO/compose.yaml" exec ros bash -lc "pkill -f 'bag [p]lay' 2>/dev/null; true"
docker compose -f "$REPO/compose.yaml" exec -d ros bash -lc "
  source /opt/ros/jazzy/setup.bash; cd /ws
  export FASTRTPS_DEFAULT_PROFILES_FILE=/ws/config/fastdds_udp_only.xml
  ros2 bag play bags/2026_06_19_18_19_06__kalhan-map-test-2_ \
    --topics /ouster/points /imu/data --clock --rate $RATE \
    --qos-profile-overrides-path /ws/config/ouster_reliable_qos.yaml $DUR_ARG \
    > /tmp/xbag.log 2>&1"
echo "container bag player started (rate=$RATE ${DUR:+dur=${DUR}s})"

# 1) localizer (host, Humble) — localize os_lidar directly vs gt_map.ply
ros2 launch lidar_localization_ros2 lidar_localization.launch.py \
  localization_param_dir:="$REPO/config/gt_ouster_ndt_local.yaml" \
  cloud_topic:=/ouster/points imu_topic:=/imu/data use_sim_time:=true \
  global_frame_id:=map base_frame_id:=os_lidar lidar_frame_id:=os_lidar \
  publish_lidar_tf:=false use_imu_preintegration:=false > /tmp/loc.log 2>&1 &
LOC=$!
echo "localizer pid=$LOC ; waiting for map load + activation..."
until grep -aq "Activating end" /tmp/loc.log; do sleep 1; done
echo "localizer active."

# 2) scovox occupancy mapping (host, Humble)
ros2 launch scovox_mapping lidar_mapping.launch.py \
  params_file:="$REPO/config/scovox_lidar_gt.yaml" \
  pointcloud_topic:=/ouster/points use_sim_time:=true > /tmp/scovox.log 2>&1 &
SCV=$!
sleep 3

# 3) rviz (host GPU)
ros2 run rviz2 rviz2 -d "$REPO/config/loc_scovox.rviz" > /tmp/rviz.log 2>&1 &
RVIZ=$!
echo "rviz launched. Watch the map build. Ctrl-C to stop everything."
wait $SCV
