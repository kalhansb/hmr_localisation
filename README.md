# mr_localisation

Map-based 3D LiDAR localization against a GLIM ground-truth point-cloud map
(`gt_map/gt_map.ply`), using
[rsasaki0109/lidar_localization_ros2](https://github.com/rsasaki0109/lidar_localization_ros2)
(NDT). Everything runs in a **ROS 2 Jazzy Docker container**, driven from a
recorded **Ouster + IMU rosbag**.

**Status:** single-robot localization validated — scan-to-GT-map registration

> The third-party packages in `src/` are **unmodified** — used as-is at the
> commits pinned in [`mr_localisation.repos`](mr_localisation.repos). All custom
> work is in `config/`, `scripts/`, `docker/`, `compose.yaml`.

---

## Repo layout
```
compose.yaml              # Jazzy container (mounts ./ at /ws, X11, host net)
docker/Dockerfile         # osrf/ros:jazzy-desktop + colcon + mcap plugin
mr_localisation.repos     # pinned third-party sources (reconstructs src/)
config/
  gt_ouster_ndt.yaml      # localizer parameters (tuned)
  localization.rviz       # RViz layout
scripts/                  # run / replay / evaluate helpers
gt_map/gt_map.ply         # ground-truth map (committed, ~48 MB)
output/                   # result plots + trajectory CSVs
bags/                     # <-- you place the rosbag here (NOT in git)
src/                      # <-- third-party packages (fetched, NOT in git)
```

## Prerequisites
- Docker + Docker Compose v2
- `git` (and optionally [`vcstool`](https://github.com/dirk-thomas/vcstool): `pip install vcstool`)
- An X server for RViz (a desktop session; note your `DISPLAY`, e.g. `:1`)
- The **rosbag** (see step 1) — not in the repo (it is ~53 GB)
- GPU is optional; RViz falls back to software GL

---

## Setup

### 1. Place the bag
This project is wired for the Ouster/IMU bag recorded on ROS 2 Jazzy (mcap).
Put the bag directory here (exact name matters — the scripts reference it):
```
bags/2026_06_19_18_19_06__kalhan-map-test-2_/
    ├── metadata.yaml
    └── *.mcap
```
Relevant topics: `/ouster/points` (`sensor_msgs/PointCloud2`, frame `os_lidar`,
best-effort QoS) and `/imu/data` (`sensor_msgs/Imu`, frame `imu`).
Using a different bag? Edit the bag path / `cloud_topic` / `imu_topic` in the
scripts and `config/`.

### 2. Fetch the third-party sources into `src/`
Option A — vcstool (uses the pinned commits):
```bash
vcs import src < mr_localisation.repos
```
Option B — plain git:
```bash
git clone https://github.com/rsasaki0109/lidar_localization_ros2.git src/lidar_localization_ros2
git -C src/lidar_localization_ros2 checkout 0fe85a563b6d83641d09550d14cc4981ad0f5a97
git clone -b humble https://github.com/rsasaki0109/ndt_omp_ros2.git src/ndt_omp_ros2
git -C src/ndt_omp_ros2 checkout ef8a34985876359ecac7b7ad0004b6f409f6fbbc
```

### 3. Build the container image
```bash
docker compose build          # pulls osrf/ros:jazzy-desktop (~4 GB), first time only
docker compose up -d          # start container 'mr_loc'
```

### 4. Build the ROS 2 workspace (inside the container)
```bash
docker compose exec ros bash -lc '
  source /opt/ros/jazzy/setup.bash && cd /ws &&
  rosdep install --from-paths src --ignore-src -r -y &&
  colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release'
```

---

## Run

### Localization (headless)
Localizes against `gt_map.ply` and dumps the trajectory to `output/path.csv`.
```bash
docker compose exec ros bash /ws/scripts/run_localization.sh        # full bag
docker compose exec ros bash /ws/scripts/run_localization.sh 180    # first 180 s
```

### Visualize in RViz (lightweight replay)
Replays a recorded result over a downsampled map — no NDT, no 53 GB bag, smooth
on software GL.
```bash
xhost +local:                                   # on the host, once per login
docker compose exec -d ros bash -lc \
  'source /opt/ros/jazzy/setup.bash; source /ws/install/setup.bash; python3 /ws/scripts/replay_result.py'
docker compose exec -d -e DISPLAY=:1 ros bash -lc \
  'source /opt/ros/jazzy/setup.bash; source /ws/install/setup.bash; ros2 run rviz2 rviz2 -d /ws/config/localization.rviz'
```
RViz: green = GT map (by height), green line = trajectory, yellow = current pose.
For live scan-on-map (heavier) use `scripts/run_viz.sh` instead, which launches
the localizer + RViz + bag play together.

### Measure accuracy
Records a short results bag, then reports scan-to-GT-map nearest-neighbour error:
```bash
# (with the localizer running and the bag playing — see scripts/run_localization.sh)
docker compose exec ros bash -lc \
  'source /opt/ros/jazzy/setup.bash; cd /ws; ros2 bag record -o output/acc_bag /ouster/points /pcl_pose'
python3 scripts/analyze_from_bag.py gt_map/gt_map.ply output/acc_bag   # needs: pip install rosbags scipy numpy
```

### Stop
```bash
docker compose stop      # keep container   |   docker compose down  # remove it
```

---

## Results
| | |
|---|---|
| Scan→map registration (median / RMS) | **4.8 cm / 6.8 cm** |
| within 10 cm / 20 cm | 90% / 99% |
| Trajectory overlay | `output/eval_final_zoom.png` |

GPS (`/fix`) is **not** a usable reference here (~20 m std-dev, not RTK). For an
absolute APE-in-metres number, export the GLIM trajectory and compare with the
package's `evo`-based tooling.

## Configuration & frames (`config/gt_ouster_ndt.yaml`)
- NDT_OMP, `ndt_resolution: 2.0`, 50 iters, local-map crop r=80 m (robust at
  turns). Drop `ndt_resolution` to 1.0 for max accuracy in structured areas.
- The bag has **no `base_link → os_lidar` extrinsic** (its `/tf_static` is
  camera-only), so we localize the **`os_lidar`** frame directly; identity is the
  correct initial seed because the GLIM map is built in the lidar frame.

## Gotchas 
- **PLY loads directly** (`pcl::io::loadPLYFile`) — no PLY→PCD conversion.
- **IMU:** enabling preintegration *hurt* (the bag lacks the lidar↔IMU extrinsic,
  so accel gravity/lever-arm are wrong in `os_lidar`). To use it, supply the real
  extrinsic and set `imu_preintegration_use_base_frame_transform: true`.
- **Known limitation:** NDT-only loses lock once at a mid-route maneuver (not map
  sparsity). IMU/odom fusion with the correct extrinsic is the fix.

## Roadmap: multi-robot
Run one `lidar_localization_ros2` instance per robot in its own namespace
(`/robotN/...`) against this same `gt_map.ply`, all publishing into the shared
`map` frame — relative poses then fall out directly.
