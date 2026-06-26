# hmr_localisation

Map-based 3D LiDAR localization against a GLIM ground-truth point-cloud map
(`gt_map/gt_map.ply`), using
[rsasaki0109/lidar_localization_ros2](https://github.com/rsasaki0109/lidar_localization_ros2)
(NDT). Everything runs in a **ROS 2 Jazzy Docker container**, driven from a
recorded **Ouster + IMU rosbag**.

**Status:** single-robot localization validated — scan-to-GT-map registration

> The third-party packages in `src/` are **unmodified** — used as-is at the
> commits pinned in [`hmr_localisation.repos`](hmr_localisation.repos). All custom
> work is in `config/`, `scripts/`, `docker/`, `compose.yaml`.

📖 **Detailed findings & methodology:** [docs/localization_findings.md](docs/localization_findings.md)

---

## Project layout
The rosbag lives **next to** the repo (it is huge and shared), not inside it:
```
HMR_Explo/                    # parent dir (not a git repo)
├── bags/                     # <-- you place the rosbag here (large, NOT in git)
└── hmr_localisation/         # this repo
    compose.yaml              # Jazzy container (mounts ./ at /ws + ../bags at /ws/bags)
    docker/Dockerfile         # osrf/ros:jazzy-desktop + colcon + mcap plugin
    hmr_localisation.repos    # pinned third-party sources (reconstructs src/)
    config/
      gt_ouster_ndt.yaml      # Mode A localizer params (flat map -> os_lidar)
      gt_ouster_ndt_tree.yaml # Mode B params (map -> odom -> base_link, IMU)
      localization.rviz       # RViz layout
    scripts/                  # run / replay / evaluate helpers
      multi_robot/ analysis/ scovox/   # grouped secondary scripts
    gt_map/gt_map.ply         # ground-truth map (committed, ~48 MB)
    output/                   # result plots + trajectory CSVs
    src/                      # <-- third-party packages (fetched, NOT in git)
```
The container bind-mounts `../bags` to `/ws/bags`, so the scripts still reference
the bag as `bags/<name>` inside the container.

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
Put the bag directory in `../bags` (one level up, beside the repo — exact name
matters, the scripts reference it). It is mounted into the container at `/ws/bags`:
```
HMR_Explo/bags/2026_06_19_18_19_06__kalhan-map-test-2_/
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
vcs import src < hmr_localisation.repos
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
docker compose up -d          # start container 'hmr_loc'
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

### Localization with a proper TF tree (`map → odom → base_link`, IMU integrated)
`run_localization.sh` (above) publishes a single flat `map → os_lidar`. For a
REP-105 tree with IMU integrated, use:
```bash
docker compose exec ros bash /ws/scripts/run_localization_tree.sh        # full bag
docker compose exec ros bash /ws/scripts/run_localization_tree.sh 180    # first 180 s
```
Resulting tree (verified with `tf2_tools view_frames`):
```
map ──(NDT vs gt_map)──> odom ──(robot_localization EKF)──> base_link ──┬──> os_lidar
                                                                        └──> imu
```
- `config/gt_ouster_ndt_tree.yaml` — Mode B (`enable_map_odom_tf: true`), base
  frame `base_link`, initial pose re-seeded to `(base_link → os_lidar)⁻¹`, IMU
  preintegration on with `imu_preintegration_use_base_frame_transform: true`.
- `base_link → os_lidar` (0.1105, 0, 0.404; yaw 180°) and `base_link → imu`
  (0.062, 0, 0.015; yaw 90°) are published as static transforms using the
  extrinsics from the bag's own `/tf_static`.

**`odom → base_link` is a `robot_localization` EKF** (`config/ekf_odom.yaml`,
launched by `launch/ekf_odom.launch.py`). It replaces the earlier identity
`static_transform_publisher`. The robot has no wheel encoders, so — rather than a
second scan matcher (e.g. FAST-LIO, which would register the LiDAR twice) — the
EKF fuses the **single NDT pose** (`/pcl_pose`, restamped to the `odom` frame by
`scripts/ndt_pose_relay.py` → `/pcl_pose_odom`) with **`/imu/data`** angular
velocity (hdl_localization / Autoware `ekf_localizer` style): the IMU smooths
**orientation** at high rate between the (10–30 Hz) NDT poses, NDT bounds the
drift, and you get a genuine continuous `odom → base_link` from one scan matcher.
(Gyro-only, so the IMU does *not* dead-reckon translation — between NDT poses
position is carried by the filter's constant-velocity model; see the tuning note
in **Next steps** to add linear-acceleration fusion.) The localizer (Mode B) then publishes
`map → odom = map→base · (odom→base)⁻¹`, so the `map → base` *product* is
unchanged (SCovox, which looks up `map → os_lidar`, is unaffected) while `odom`
becomes a real smoothing layer. Feeding the pose restamped to `odom` (not `map`)
avoids a circular `map → odom` TF lookup — see `scripts/ndt_pose_relay.py`.

> Requires `ros-jazzy-robot-localization` (added to `docker/Dockerfile`). After
> pulling these changes, rebuild the image: `docker compose build`.

### Visualize in RViz (lightweight replay)
Replays a recorded result over a downsampled map — no NDT, no 53 GB bag, smooth
on software GL.
```bash
xhost +local:                                   # on the host, once per login
docker compose exec -d ros bash -lc \
  'source /opt/ros/jazzy/setup.bash; source /ws/install/setup.bash; python3 /ws/scripts/analysis/replay_result.py'
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
python3 scripts/analysis/analyze_from_bag.py gt_map/gt_map.ply output/acc_bag   # needs: pip install rosbags scipy numpy
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

## Configuration & frames
Two localizer configs:
- `config/gt_ouster_ndt.yaml` — **Mode A**, flat `map → os_lidar` (simplest;
  `run_localization.sh`).
- `config/gt_ouster_ndt_tree.yaml` — **Mode B**, full `map → odom → base_link`
  tree with IMU (`run_localization_tree.sh`).

- NDT_OMP, `ndt_resolution: 2.0`, 50 iters, local-map crop r=80 m (robust at
  turns). Drop `ndt_resolution` to 1.0 for max accuracy in structured areas.
- The bag's `/tf_static` carries the **full robot URDF** rooted at `base_link`
  (`base_link → os_lidar`, `base_link → imu`, wheels, camera, …) — it is *not*
  camera-only. **Mode A** localizes `os_lidar` directly (identity seed, since the
  GLIM map is built in the lidar frame); **Mode B** localizes `base_link`,
  re-seeding the initial pose to `(base_link → os_lidar)⁻¹`.

## Gotchas 
- **PLY loads directly** (`pcl::io::loadPLYFile`) — no PLY→PCD conversion.
- **IMU:** the lidar↔IMU extrinsic *is* in the bag (`base_link → imu`, yaw 90°).
  The earlier "preintegration hurt" result was because the flat Mode A run never
  published `/tf_static`, so the localizer had no extrinsic to rotate the IMU into
  the base frame — **not** a missing extrinsic. Mode B publishes the transform and
  sets `imu_preintegration_use_base_frame_transform: true`, so preintegration is
  valid there (verified: 0 IMU base-frame transform failures).
- **Known limitation (validated full-bag, Mode B):** recovers *twice* through the
  ~130–300 s mid-route maneuver where Mode A diverged, but still loses lock in the
  back third at a feature-sparse / open region near the map edge (last good pose
  ~`(7.5, 32.2)`; 432 good / 165 rejected scans; overlay `output/eval_tree.png`).
  Confounded by a real-time deficit — the localizer ran at ~1.2 Hz, dropping ~88 %
  of the 10 Hz scans. See **Next steps** below.

## Next steps (localization robustness)
1. **Make it real-time first.** At ~1.2 Hz the localizer drops ~88 % of scans, so
   IMU must bridge ~0.8 s gaps and the late-route result is confounded. Lower the
   per-scan NDT cost (`ndt_max_iterations` 50 → ~15–20, raise `voxel_leaf_size`,
   keep the local-map crop; or use GPU NDT) to reach ≥10 Hz, then re-run. This
   separates "method fails in the sparse zone" from "couldn't keep up."
2. **EKF/IMU-fused `odom → base_link`.** ✅ *Done* — `config/ekf_odom.yaml` +
   `launch/ekf_odom.launch.py` run a `robot_localization` EKF that fuses the
   single NDT pose with `/imu/data` (hdl_localization / Autoware `ekf_localizer`
   style), replacing the identity passthrough. See "Localization with a proper TF
   tree" above. The gyro smooths orientation at high rate and the
   constant-velocity model carries translation between scans — one scan matcher,
   *not* a second matcher like FAST-LIO. Next tuning step: fuse IMU linear
   acceleration (`ax/ay/az` in `ekf_odom.yaml`, needs a valid IMU orientation for
   gravity removal) for genuine translational dead-reckoning through long NDT
   dropouts; first fix the real-time deficit (step 1) so gaps stay short.

## Roadmap: multi-robot
Run one `lidar_localization_ros2` instance per robot in its own namespace
(`/robotN/...`) against this same `gt_map.ply`, all publishing into the shared
`map` frame — relative poses then fall out directly.
