#!/usr/bin/env bash
# Single-robot localization against gt_map.ply with the PROPER REP-105 TF tree,
# but WITHOUT robot_localization -- odom->base_link is a STATIC identity publisher
# instead of the EKF. This is the A/B baseline for run_localization_tree.sh:
# everything else (Mode B map->odom NDT, base_link, IMU preintegration, static
# extrinsics, /path eval output) is identical, so the only difference is the
# missing EKF smoothing layer.
#
#   map ──(NDT map-match, this localizer)──> odom
#   odom ──(STATIC identity, no EKF)──> base_link
#   base_link ──(static extrinsic)──> os_lidar, imu
#
# With odom->base_link = identity, the localizer (Mode B) publishes
#   map->odom = map->base o (odom->base)^-1 = map->base,
# so map->odom carries the full pose and odom is a pass-through. The /pcl_pose
# trajectory (and output/path.csv) is therefore identical to the EKF run minus
# the high-rate odom smoothing -- exactly the layer robot_localization adds.
#
# Usage (from the workspace root, on the HOST). Allow X first (once per login):
#   xhost +local:
#   docker compose up -d
#   docker compose exec -e DISPLAY=:1 ros bash /ws/scripts/run_localization_tree_noekf.sh [playback_duration_s]
#
# RViz (config/localization.rviz, fixed frame = map) shows: green = GT map,
# red = live Ouster scan placed in the map frame via the full TF chain, green
# line = /path, yellow = current /pcl_pose. Leave playback_duration empty for the
# full bag. Outputs land in /ws/output/.
set -e
source /opt/ros/jazzy/setup.bash
source /ws/install/setup.bash
cd /ws
: "${DISPLAY:=:1}"
export DISPLAY LIBGL_ALWAYS_SOFTWARE=1   # software GL (no GPU passthrough)

DUR="${1:-}"
DUR_ARG=""
[ -n "$DUR" ] && DUR_ARG="--playback-duration $DUR"

PIDS=()
cleanup() {
  kill "${PIDS[@]}" 2>/dev/null || true
  # backstop: kill the static odom->base_link publisher so no stale identity
  # transform lingers into a back-to-back run (host net, ROS_DOMAIN_ID=0).
  pkill -f "static_transform_publisher.*base_link" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

# 1) odom -> base_link : STATIC identity publisher (NO robot_localization).
#    This is the layer the EKF replaced; here it is a plain pass-through so the
#    REP-105 tree is structurally complete but odom does no smoothing.
ros2 run tf2_ros static_transform_publisher \
  --x 0 --y 0 --z 0 --qx 0 --qy 0 --qz 0 --qw 1 \
  --frame-id odom --child-frame-id base_link \
  > /tmp/static_odom_base.log 2>&1 &
PIDS+=($!)

# 2) localizer in Mode B (map->odom), base frame = base_link, IMU preintegration
#    on with the base-frame transform. The launch's own static publishers emit
#    base_link->os_lidar (lidar_tf_*) and base_link->imu (imu_tf_*).
ros2 launch lidar_localization_ros2 lidar_localization.launch.py \
  localization_param_dir:=/ws/config/gt_ouster_ndt_tree.yaml \
  cloud_topic:=/ouster/points imu_topic:=/imu/data use_sim_time:=true \
  global_frame_id:=map odom_frame_id:=odom base_frame_id:=base_link \
  use_imu_preintegration:=true imu_preintegration_use_base_frame_transform:=true \
  publish_lidar_tf:=true lidar_frame_id:=os_lidar \
  lidar_tf_x:=0.1105 lidar_tf_y:=0.0 lidar_tf_z:=0.404 lidar_tf_yaw:=3.14159265 \
  publish_imu_tf:=true imu_frame_id:=imu \
  imu_tf_x:=0.062 imu_tf_y:=0.0 imu_tf_z:=0.015 imu_tf_yaw:=1.5707963 \
  > /tmp/loc_tree.log 2>&1 &
PIDS+=($!)
echo "localizer pid=${PIDS[-1]} ; waiting for map load + activation..."
until grep -aq "Activating end" /tmp/loc_tree.log; do sleep 1; done
echo "active (no EKF; odom->base_link = static identity)."

# 3) rviz (fixed frame = map; watch the red scan register onto the green GT map)
ros2 run rviz2 rviz2 -d /ws/config/localization.rviz > /tmp/rviz.log 2>&1 &
PIDS+=($!)
sleep 5

# 4) best-effort TF-tree check a few seconds into playback (sim clock from --clock)
( sleep 12
  echo "--- map -> base_link (sampled during playback) ---" > /tmp/tf_chain.log
  timeout 5 ros2 run tf2_ros tf2_echo map base_link \
    --ros-args -p use_sim_time:=true >> /tmp/tf_chain.log 2>&1 || true ) &
PIDS+=($!)

# 5) play the bag (sim clock). Only the two topics the localizer needs; the TF
#    tree comes from our static publishers, not the bag's /tf.
echo "playing bag ${DUR:+(first ${DUR}s) }... watch RViz."
ros2 bag play bags/2026_06_19_18_19_06__kalhan-map-test-2_ \
  --topics /ouster/points /imu/data --clock --rate 1.0 $DUR_ARG

# 6) dump the latched /path trajectory (map frame) to CSV
python3 /ws/scripts/fetch_path.py /ws/output/path_noekf.csv

echo "good_scans=$(grep -ac 'fitness score:' /tmp/loc_tree.log)  rejects=$(grep -ac 'fitness score is over' /tmp/loc_tree.log)"
echo "imu_preint_warns=$(grep -ac 'IMU preintegration' /tmp/loc_tree.log)  tf_fail=$(grep -ac 'Could not get transform' /tmp/loc_tree.log)"
echo "--- TF tree (map -> base_link) ---"; cat /tmp/tf_chain.log 2>/dev/null || true
echo "done. plot on host with: python3 scripts/plot_zoom.py gt_map/gt_map.ply output/path_noekf.csv output/eval_noekf.png"
echo "bag finished. RViz still live; Ctrl-C (or 'docker compose stop') to exit."
wait
