#!/usr/bin/env bash
# Single-robot localization against gt_map.ply with a PROPER REP-105 TF tree and
# IMU integrated, replacing the flat map->os_lidar of run_localization.sh.
#
#   map ──(NDT map-match, this localizer)──> odom
#   odom ──(identity static, replay/eval)──> base_link
#   base_link ──(static extrinsic from bag /tf_static)──> os_lidar, imu
#
# IMU (/imu/data, frame "imu") is fed to the localizer's preintegration with the
# base-frame transform, now valid because we publish base_link->imu. The bag's
# own /tf_static carries these extrinsics; we republish the two we need as static
# transforms so there is no bag-replay QoS race.
#
# Extrinsics (from the bag's /tf_static, base_link-rooted URDF):
#   base_link -> os_lidar : (0.1105, 0, 0.404)  yaw 180  (q = 0,0,1,0)
#   base_link -> imu      : (0.062,  0, 0.015)  yaw  90  (q = 0,0,0.70711,0.70711)
#
# Usage (from the workspace root, on the HOST):
#   docker compose up -d
#   docker compose exec ros bash /ws/scripts/run_localization_tree.sh [playback_duration_s]
#
# Leave playback_duration empty for the full bag. Outputs land in /ws/output/.
set -e
source /opt/ros/jazzy/setup.bash
source /ws/install/setup.bash
cd /ws

DUR="${1:-}"
DUR_ARG=""
[ -n "$DUR" ] && DUR_ARG="--playback-duration $DUR"

PIDS=()
cleanup() { kill "${PIDS[@]}" 2>/dev/null || true; }
trap cleanup EXIT

# 1) odom -> base_link : identity passthrough. For replay/eval the NDT map-match
#    carries the motion (map->odom). Swap this for KISS-ICP / a LIO publishing a
#    real odom->base_link when you need a smooth, jump-free odometry layer.
ros2 run tf2_ros static_transform_publisher \
  --frame-id odom --child-frame-id base_link \
  --ros-args -p use_sim_time:=true > /tmp/odom_tf.log 2>&1 &
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
echo "active. playing bag ${DUR:+(first ${DUR}s)}..."

# 3) best-effort TF-tree check a few seconds into playback (sim clock from --clock)
( sleep 12
  echo "--- map -> base_link (sampled during playback) ---" > /tmp/tf_chain.log
  timeout 5 ros2 run tf2_ros tf2_echo map base_link \
    --ros-args -p use_sim_time:=true >> /tmp/tf_chain.log 2>&1 || true ) &
PIDS+=($!)

# 4) play the bag (sim clock). Only the two topics the localizer needs; the TF
#    tree comes from our static publishers, not the bag's /tf.
ros2 bag play bags/2026_06_19_18_19_06__kalhan-map-test-2_ \
  --topics /ouster/points /imu/data --clock --rate 1.0 $DUR_ARG

# 5) dump the latched /path trajectory (map frame) to CSV
python3 /ws/scripts/fetch_path.py /ws/output/path.csv

echo "good_scans=$(grep -ac 'fitness score:' /tmp/loc_tree.log)  rejects=$(grep -ac 'fitness score is over' /tmp/loc_tree.log)"
echo "imu_preint_warns=$(grep -ac 'IMU preintegration' /tmp/loc_tree.log)  tf_fail=$(grep -ac 'Could not get transform' /tmp/loc_tree.log)"
echo "--- TF tree (map -> base_link) ---"; cat /tmp/tf_chain.log 2>/dev/null || true
echo "done. plot on host with: python3 scripts/plot_zoom.py gt_map/gt_map.ply output/path.csv output/eval.png"
