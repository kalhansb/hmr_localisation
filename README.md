# hmr_localisation — NDT LiDAR localization (ROS 2 Jazzy, Docker)

Real-time map-based NDT localization against a GLIM ground-truth map, publishing a
full REP-105 TF tree. Runs live (real robot) or against a recorded bag.

```
map ──(NDT vs gt_map)──> odom ──(EKF: NDT pose + IMU gyro)──> base_link ──┬──> os_lidar
                                                                          └──> imu
```

`map → odom` is this NDT localizer; `odom → base_link` is a `robot_localization`
EKF (smooth 50 Hz odometry for Nav2); the `base_link → {os_lidar, imu}` extrinsics
are static.

## Requirements

- **Docker** + **Docker Compose** (v2). CPU only — base image `osrf/ros:jazzy-desktop`,
  no GPU/CUDA needed.
- **vcstool** on the host to fetch the pinned sources: `sudo apt install python3-vcstool`.
- **GT map** at `gt_map/gt_map.ply` (GLIM map; not in git).
- **X11** only if you want RViz (`compose.yaml` forwards `$DISPLAY`).
- For the **bag replay** path only: the rosbag beside the repo at
  `../../../bags/2026_06_19_18_19_06__kalhan-map-test-2_/` (~53 GB, not in git;
  mounted at `/ws/bags`).

## Setup (once)

```bash
vcs import src < hmr_localisation.repos      # fetch pinned NDT sources (our forks)
docker compose build && docker compose up -d

# build the workspace
docker compose exec ros bash -lc 'cd /ws &&
  rosdep install --from-paths src --ignore-src -r -y &&
  colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release'

# make the 0.5 m localization map from the GLIM map (once)
docker compose exec ros bash -lc 'cd /ws &&
  g++ -O2 -std=c++17 scripts/downsample_map_pcl.cpp -o downsample_map_pcl \
    $(pkg-config --cflags --libs pcl_common pcl_io pcl_filters) \
    -lpcl_kdtree -lpcl_search -lpcl_octree &&
  ./downsample_map_pcl gt_map/gt_map.ply gt_map/gt_map_us050.pcd 0.5 uniform'
```

## Run

**Real robot (live sensors):**
```bash
docker compose up -d
docker compose exec ros bash /ws/scripts/run_localization_live.sh
```
`compose.yaml` keeps DDS discovery on loopback; if the sensors or teammate robots are
on the network, add `-e ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET` to the `exec`.

**Recorded bag:**
```bash
docker compose exec ros bash /ws/scripts/run_localization_tree.sh       # full bag
docker compose exec ros bash /ws/scripts/run_localization_tree.sh 180   # first 180 s
```

Stop the container when done — **never `pkill`** the ROS processes:
```bash
docker compose stop
```

## Notes

- The localizer subscribes `/ouster/points` **best-effort** (`SensorDataQoS`), so the
  Ouster driver's default best-effort output matches directly — no QoS
  reconfiguration on the robot, and the bag plays with its recorded best-effort QoS.
- Multi-MB clouds are routed over **shared memory**
  ([`config/fastdds_shm.xml`](config/fastdds_shm.xml)); over UDP loopback they throttle
  to ~0.1 Hz. The run scripts set this automatically.
- The static `base_link → {os_lidar, imu}` extrinsics baked into the scripts are the
  map-test robot's — **re-measure for another platform**.
- Config: [`config/gt_ouster_ndt_tree_realtime.yaml`](config/gt_ouster_ndt_tree_realtime.yaml)
  (tree, `base_link`); [`config/gt_ouster_ndt_realtime.yaml`](config/gt_ouster_ndt_realtime.yaml)
  (flat `map → os_lidar` variant, `scripts/run_localization.sh`).
- **Multi-robot mapping** builds on this tree (all robots localize against the same
  `gt_map`, sharing one global `map` frame). Runbook:
  [scovox `docs/distributed_mapping.md`](https://github.com/kalhansb/scovox/blob/main/docs/distributed_mapping.md).
