# hmr_localisation — NDT localization with a map → odom → base_link tree

Real-time (live rate 1.0, Ouster ~7.6 Hz) map-based NDT LiDAR localization against a
GLIM ground-truth map (`gt_map/gt_map.ply`), publishing a full REP-105 TF tree in a
ROS 2 Jazzy Docker container:

```
map ──(NDT vs gt_map)──> odom ──(robot_localization EKF: NDT pose + IMU gyro)──> base_link ──┬──> os_lidar
                                                                                             └──> imu
```

- [`config/gt_ouster_ndt_tree_realtime.yaml`](config/gt_ouster_ndt_tree_realtime.yaml)
  publishes `map → odom` (16 NDT threads, reject gate off, IMU preintegration), base
  frame `base_link`, seeded at `(base_link → os_lidar)⁻¹`.
- **`odom → base_link` is a `robot_localization` EKF**
  ([`config/ekf_odom.yaml`](config/ekf_odom.yaml), launched by
  [`launch/ekf_odom.launch.py`](launch/ekf_odom.launch.py)): it fuses the NDT pose
  (`/pcl_pose`, restamped to `odom` by [`scripts/ndt_pose_relay.py`](scripts/ndt_pose_relay.py))
  with `/imu/data` gyro and broadcasts a smooth 50 Hz `odom → base_link` plus a
  `nav_msgs/Odometry`. The localizer sets `map → odom = map→base · (odom→base)⁻¹`, so
  `map → base` stays the raw NDT pose while `odom` is the smoothing layer.
- `base_link → os_lidar` (0.1105, 0, 0.404; yaw 180°) and `base_link → imu`
  (0.062, 0, 0.015; yaw 90°) are static, from the bag's own `/tf_static`.

## Run
The rosbag lives **beside** the repo (not in git, ~53 GB):
`../bags/2026_06_19_18_19_06__kalhan-map-test-2_/` (mounted at `/ws/bags`).
```bash
# build once: fetch the pinned NDT sources + patch, build image + workspace, make the 0.5 m map
vcs import src < hmr_localisation.repos
git -C src/lidar_localization_ros2 apply ../../patches/lidar_localization_ros2-keepalive-count.patch
docker compose build && docker compose up -d
docker compose exec ros bash -lc 'source /opt/ros/jazzy/setup.bash && cd /ws &&
  rosdep install --from-paths src --ignore-src -r -y &&
  colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release'
docker compose exec ros bash -lc 'cd /ws && g++ -O2 -std=c++17 scripts/downsample_map_pcl.cpp -o downsample_map_pcl \
  $(pkg-config --cflags --libs pcl_common pcl_io pcl_filters) -lpcl_kdtree -lpcl_search -lpcl_octree &&
  ./downsample_map_pcl gt_map/gt_map.ply gt_map/gt_map_us050.pcd 0.5 uniform'

# run the map -> odom -> base_link tree at rate 1.0
docker compose exec ros bash /ws/scripts/run_localization_tree.sh        # full bag
docker compose exec ros bash /ws/scripts/run_localization_tree.sh 180    # first 180 s
docker compose stop
```
The run script sets the **SHM transport** ([`config/fastdds_shm.xml`](config/fastdds_shm.xml)
— else the multi-MB clouds throttle to ~0.1 Hz over UDP loopback) and **reliable QoS**
([`config/ouster_reliable_qos.yaml`](config/ouster_reliable_qos.yaml) — the localizer
subscribes RELIABLE but the bag recorded `/ouster/points` BEST_EFFORT). Trajectory →
`output/path.csv`.

## Does robot_localization + IMU smooth the trajectory?
A/B over 60 s at 50 Hz (reproduce: `scripts/test_ekf_smoothing.sh ekf|noekf`, then
`scripts/analysis/analyze_smoothing.py`):

| edge | held_frac | rms_jerk | |
|---|---|---|---|
| `map→base_link` — EKF vs no-EKF | 0.77 / 0.77 | 3908 / 3848 | **identical** — EKF cancels out of the global pose |
| `odom→base_link` — EKF | **0.11** | **1096** | smooth, continuous 50 Hz (3.5× smoother) |
| `odom→base_link` — no-EKF | 1.00 | 0 | static identity — no motion |

The EKF does **not** change the global `map → base_link` pose (pinned to raw NDT by
design). It smooths the **`odom → base_link`** edge — interpolating between the ~9 Hz
NDT poses into a continuous high-rate estimate.

**For Nav2:** use the EKF, **not** a static-identity `odom → base_link`. A static
identity makes the tree *exist* but pins `base_link` to the `odom` origin (the rolling
local costmap never follows the robot) and provides no `nav_msgs/Odometry`/velocity;
all motion and all NDT jumps land in `map → odom`. The EKF gives Nav2 the smooth
continuous `odom → base_link` (and odometry topic) it needs while `map → odom` absorbs
the discrete corrections — the correct REP-105 split.

---
A flat `map → os_lidar` variant (`scripts/run_localization.sh`) is also included; the
earlier ICP/GLIM exploration lives on the `experiments` branch / tags `v1.0`, `v2.0`.
