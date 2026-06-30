# hmr_localisation — real-time NDT LiDAR localization

Real-time (live playback rate 1.0, Ouster ~7.6 Hz) map-based 3D LiDAR localization
against a GLIM ground-truth point-cloud map (`gt_map/gt_map.ply`), using
[rsasaki0109/lidar_localization_ros2](https://github.com/rsasaki0109/lidar_localization_ros2)
(NDT) in a **ROS 2 Jazzy Docker container**, driven from a recorded Ouster + IMU rosbag.

> This branch is trimmed to **just the real-time recipe**. The fuller stack (Mode A/B
> tree configs, the EKF/IMU odom layer, RViz, the Jetson config, and the ICP/GLIM
> alternative localizers) is preserved on the **`experiments`** branch and tag **`v1.0`**.

## The recipe
[`config/gt_ouster_ndt_realtime.yaml`](config/gt_ouster_ndt_realtime.yaml) is the
validated rate-1.0 config — two changes vs a stock NDT setup let it track the entire
500 s / ~337 m route at live rate without diverging:
1. **`ndt_num_threads: 16`** — keep up with the 7.6 Hz sensor (a quality-neutral speedup;
   NDT_OMP parallelizes the per-point accumulation with no effect on the result).
2. **`reject_above_score_threshold: false`** — never freeze the pose on a high-residual
   scan. The route has an elevated stretch where NDT *fitness* spikes even though the
   *pose* is correct; with the reject gate on, those scans were dropped and the pose
   froze → divergence. With it off the localizer rides straight through.

## Files
| | |
|---|---|
| `docker/Dockerfile` + `compose.yaml` | the Jazzy container (single `ros` service) |
| `hmr_localisation.repos` | pinned NDT sources (`lidar_localization_ros2` + `ndt_omp_ros2`) |
| `patches/lidar_localization_ros2-keepalive-count.patch` | registration cloud keep-alive `4096 → 4` (memory fix) |
| `config/gt_ouster_ndt_realtime.yaml` | **the real-time config** |
| `gt_map/gt_map.ply` | the ground-truth map (~48 MB, 3.06 M pts) |
| `scripts/downsample_map_pcl.cpp` | makes the lighter 0.5 m `.pcd` the config loads |
| `scripts/run_localization.sh` | launch the localizer + play the bag |
| `scripts/fetch_path.py` | dump the trajectory to CSV |

## Prerequisites
- Docker + Docker Compose v2, and (optionally) [`vcstool`](https://github.com/dirk-thomas/vcstool) (`pip install vcstool`)
- The **rosbag** (not in git, ~53 GB), placed beside the repo at
  `../bags/2026_06_19_18_19_06__kalhan-map-test-2_/` (mounted into the container at
  `/ws/bags`). Topics: `/ouster/points` (`os_lidar`) + `/imu/data`.

## Setup
```bash
# 1) fetch the pinned NDT sources into src/ and apply the keep-alive memory patch
vcs import src < hmr_localisation.repos
git -C src/lidar_localization_ros2 apply ../../patches/lidar_localization_ros2-keepalive-count.patch

# 2) build the image + the ROS 2 workspace
docker compose build && docker compose up -d
docker compose exec ros bash -lc '
  source /opt/ros/jazzy/setup.bash && cd /ws &&
  rosdep install --from-paths src --ignore-src -r -y &&
  colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release'

# 3) make the 0.5 m map the config loads (once). Lightest footprint, ~3 cm relative APE.
docker compose exec ros bash -lc '
  cd /ws && g++ -O2 -std=c++17 scripts/downsample_map_pcl.cpp -o downsample_map_pcl \
    $(pkg-config --cflags --libs pcl_common pcl_io pcl_filters) -lpcl_kdtree -lpcl_search -lpcl_octree &&
  ./downsample_map_pcl gt_map/gt_map.ply gt_map/gt_map_us050.pcd 0.5 uniform'
# (Or skip step 3 and set map_path: "/ws/gt_map/gt_map.ply" in the config — NDT loads .ply directly, just heavier.)
```

## Run
```bash
docker compose exec ros bash /ws/scripts/run_localization.sh        # full bag, rate 1.0
docker compose exec ros bash /ws/scripts/run_localization.sh 180    # first 180 s
docker compose stop
```
Live output: `map → os_lidar` TF and `/pcl_pose`; the trajectory is dumped to
`output/path.csv` at the end.

**On a Jetson:** same flow — NDT_OMP is CPU-only and the `osrf/ros:jazzy-*` base is
multi-arch (pulls `arm64` when you build on the board). Lower `ndt_num_threads` in the
config to fit the CPU budget (e.g. Orin Nano → 2, Orin NX → 3, AGX Orin → 4–6).
