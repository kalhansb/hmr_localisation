# hmr_localisation — LiDAR map localization (ROS 2 Jazzy, Docker)

Real-time map-based LiDAR localization (small_gicp VGICP, NDT bootstrap) against a GLIM
ground-truth map, publishing a full REP-105 TF tree. Runs live (real robot) or against
a recorded bag.

```
map ──(VGICP vs gt_map)──> odom ──(EKF: scan pose + IMU gyro)──> base_link ──┬──> os_lidar
                                                                             └──> imu
```

`map → odom` is this scan-to-map localizer; `odom → base_link` is a `robot_localization`
EKF (smooth 50 Hz odometry for Nav2); the `base_link → {os_lidar, imu}` extrinsics
are static.

## Requirements

- **Docker** + **Docker Compose** (v2). CPU only — base image `osrf/ros:jazzy-desktop`,
  no GPU/CUDA needed. The image builds [small_gicp](https://github.com/koide3/small_gicp)
  from source at a pinned tag (`docker/Dockerfile`); rebuild the image after pulling.
- **vcstool** on the host to fetch the pinned sources: `sudo apt install python3-vcstool`.
- **GT map** at `gt_map/gt_map.ply` (GLIM map, 48 MB) — committed, along with the
  `gt_map_us050`–`us200` downsamples.
- **X11** only if you want RViz (`compose.yaml` forwards `$DISPLAY`).
- For the **bag replay** path only: rosbags beside the repo under `../../../bags/`
  (not in git; mounted at `/ws/bags`). The run scripts default to
  `2026_06_19_18_19_06__kalhan-map-test-2_` and take a `BAG=...` override; each
  recording needs its own `initial_pose`. The two 2026-07-31 robot bags, their trimmed
  120 s cuts and their verified start poses are catalogued in
  [`docs/jetson_runs.md`](docs/jetson_runs.md).

## Setup (once)

```bash
vcs import src < hmr_localisation.repos      # fetch pinned localizer sources (our forks)
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
- **Registration backend.** The tree config runs `registration_method: "SMALL_VGICP"`
  (small_gicp voxelized GICP, `vgicp_voxel_resolution` 1.0 m on the 0.5 m map). The node
  bootstraps the first 5 scans with NDT_OMP on the full map, then tracks with VGICP on
  the cached cropped target. Switch back with `registration_method: "NDT_OMP"` — the
  `ndt_*`, crop, EKF and map settings are shared. The other `gt_ouster_ndt_*` configs
  (flat, per-rig coop) are still on `NDT_OMP`; they take the same one-line switch.
  If the node exits with "small_gicp backend requested but support is not available",
  the workspace was built against an image without small_gicp — `docker compose build`
  and rebuild.
- **CPU budget.** The sensor runs at 10 Hz, so the node has 100 ms per scan. On an
  8-core x86 host the tree config uses ~2.6 cores and aligns in 26 ms median / 66 ms
  p95, keeping ~9 of every 10 scans; the EKF bridges the dropped ones (the pose is
  never frozen). What that cost is made of, which knobs reduce it and what each one
  costs in accuracy -- `scan_channel_stride` first, then the rest of the ladder -- is
  measured in [`docs/cpu_optimisation.md`](docs/cpu_optimisation.md).
- **Jetson AGX Orin (same containers).** Full runbook — the two trimmed bags, the
  verified per-bag start poses, the throughput test and how to score a run against a
  reference trajectory — is [`docs/jetson_runs.md`](docs/jetson_runs.md).
  The image is CPU-only and portable:
  `osrf/ros:jazzy-desktop` is multi-arch and small_gicp is built without
  `-march=native`, so `docker compose build` works unchanged on arm64 (~30 min).
  On the **host**, before `docker compose up`: `sudo nvpmodel -m 0 && sudo jetson_clocks`.
  `compose.yaml` sets no CPU limit, so the container sees all 12 cores;
  `ndt_num_threads: 8` is what leaves GLIM, the drivers and the planner (in their own
  containers) their cores -- do not raise it. `ipc: host` is required for the
  shared-memory cloud transport between containers, but killed ROS processes leave
  orphaned Fast-DDS segments in the host's `/dev/shm` (half of RAM on a Jetson): always
  `docker compose stop`, and reclaim with `fastdds shm clean` inside the container if
  `df /dev/shm` fills up. If the board still cannot hold 10 Hz, follow the ladder in
  [`docs/cpu_optimisation.md`](docs/cpu_optimisation.md).
- **Multi-robot mapping** builds on this tree (all robots localize against the same
  `gt_map`, sharing one global `map` frame). Runbook:
  [scovox `docs/distributed_mapping.md`](https://github.com/kalhansb/scovox/blob/main/docs/distributed_mapping.md).
