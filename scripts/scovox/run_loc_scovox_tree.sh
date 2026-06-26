#!/usr/bin/env bash
# Live RViz: localization (PROPER REP-105 TF tree) + SCovox occupancy mapping.
#
# This is run_loc_scovox_viz.sh but with the map->odom->base_link tree of
# run_localization_tree.sh instead of the flat map->os_lidar. The full tree:
#
#   map ──(NDT map-match, this localizer, enable_map_odom_tf)──> odom
#   odom ──(identity static, replay/eval)──────────────────────> base_link
#   base_link ──(static extrinsics)──> os_lidar, imu
#
# Extrinsics (from the bag's base_link-rooted /tf_static URDF):
#   base_link -> os_lidar : (0.1105, 0, 0.404)  yaw 180  (q = 0,0,1,0)
#   base_link -> imu      : (0.062,  0, 0.015)  yaw  90  (q = 0,0,0.70711,0.70711)
#
# SCovox integrates /ouster/points in the `map` frame; map->os_lidar now resolves
# through the 3-hop chain above (it does a TF lookup, so hop count is irrelevant).
# LiDAR occupancy is capped at 15 m (config/scovox_lidar_tree.yaml, max_range).
#
# /ouster/points is republished RELIABLE (config/ouster_reliable_qos.yaml) so it
# reaches scovox's reliable subscriber as well as the localizer.
#
# Runs entirely inside the Jazzy container. On the HOST first (once per login):
#   xhost +local:
#   docker compose up -d
#   docker compose exec -e DISPLAY=:1 ros bash /ws/scripts/scovox/run_loc_scovox_tree.sh [duration_s] [rate]
#
# duration_s : optional bag playback length in seconds (default: full bag)
# rate       : optional playback rate (default: 1.0)
set -e
source /opt/ros/jazzy/setup.bash
source /ws/install/setup.bash
# scovox jazzy overlay: prefer install_jazzy, fall back to install (built in-container)
if [ -f /scovox/install_jazzy/setup.bash ]; then
  source /scovox/install_jazzy/setup.bash
else
  source /scovox/install/setup.bash
fi
cd /ws
: "${DISPLAY:=:1}"
export DISPLAY LIBGL_ALWAYS_SOFTWARE=1   # software GL (no GPU passthrough)

DUR="${1:-}"
RATE="${2:-1.0}"
DUR_ARG=""
[ -n "$DUR" ] && DUR_ARG="--playback-duration $DUR"

PIDS=()
cleanup() { kill "${PIDS[@]}" 2>/dev/null || true; }
trap cleanup EXIT INT TERM

# 1) odom -> base_link : identity passthrough. For replay/eval the NDT map-match
#    carries the motion (map->odom). Swap this for KISS-ICP / a LIO publishing a
#    real odom->base_link when you need a smooth, jump-free odometry layer.
ros2 run tf2_ros static_transform_publisher \
  --frame-id odom --child-frame-id base_link \
  --ros-args -p use_sim_time:=true > /tmp/odom_tf.log 2>&1 &
PIDS+=($!)

# 2) localizer in Mode B (map->odom), base frame = base_link.
#    IMU preintegration is OFF: empirically it diverges the NDT track ~108-130 s
#    in (see gt_ouster_ndt_local.yaml note); NDT + constant-velocity prediction
#    (predict_pose_from_previous_delta, set in the yaml) tracks ~3x longer. The
#    launch's own static publishers still emit base_link->os_lidar (lidar_tf_*)
#    and base_link->imu (imu_tf_*) to complete the REP-105 tree.
ros2 launch lidar_localization_ros2 lidar_localization.launch.py \
  localization_param_dir:=/ws/config/gt_ouster_ndt_tree.yaml \
  cloud_topic:=/ouster/points imu_topic:=/imu/data use_sim_time:=true \
  global_frame_id:=map odom_frame_id:=odom base_frame_id:=base_link \
  use_imu_preintegration:=false imu_preintegration_use_base_frame_transform:=false \
  publish_lidar_tf:=true lidar_frame_id:=os_lidar \
  lidar_tf_x:=0.1105 lidar_tf_y:=0.0 lidar_tf_z:=0.404 lidar_tf_yaw:=3.14159265 \
  publish_imu_tf:=true imu_frame_id:=imu \
  imu_tf_x:=0.062 imu_tf_y:=0.0 imu_tf_z:=0.015 imu_tf_yaw:=1.5707963 \
  > /tmp/loc.log 2>&1 &
PIDS+=($!)
echo "localizer pid=${PIDS[-1]} ; waiting for map load + activation..."
until grep -aq "Activating end" /tmp/loc.log; do sleep 1; done
echo "localizer active; map published."

# 3) scovox occupancy mapping (integrates in the map frame via the TF chain;
#    LiDAR capped at 15 m by config/scovox_lidar_tree.yaml).
ros2 launch scovox_mapping lidar_mapping.launch.py \
  params_file:=/ws/config/scovox_lidar_tree.yaml \
  pointcloud_topic:=/ouster/points use_sim_time:=true > /tmp/scovox.log 2>&1 &
PIDS+=($!)
echo "scovox pid=${PIDS[-1]}"
sleep 3

# 4) rviz
ros2 run rviz2 rviz2 -d /ws/config/loc_scovox.rviz > /tmp/rviz.log 2>&1 &
PIDS+=($!)
sleep 5

# 5) play the bag (Ctrl-C to stop early). /ouster/points forced RELIABLE.
echo "playing bag ${DUR:+(first ${DUR}s) }at rate ${RATE}... watch RViz."
ros2 bag play bags/2026_06_19_18_19_06__kalhan-map-test-2_ \
  --topics /ouster/points /imu/data --clock --rate "$RATE" \
  --qos-profile-overrides-path /ws/config/ouster_reliable_qos.yaml $DUR_ARG

echo "bag finished. scovox map still live in RViz; Ctrl-C to stop."
SCV_PID="${PIDS[2]}"   # 0=odom_tf 1=localizer 2=scovox 3=rviz; wait on scovox to keep the map alive
wait "$SCV_PID" 2>/dev/null || true
