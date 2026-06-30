# hmr_localisation

Map-based 3D LiDAR localization against a GLIM ground-truth point-cloud map
(`gt_map/gt_map.ply`), using
[rsasaki0109/lidar_localization_ros2](https://github.com/rsasaki0109/lidar_localization_ros2)
(NDT). Everything runs in a **ROS 2 Jazzy Docker container**, driven from a
recorded **Ouster + IMU rosbag**.

**Status:** single-robot localization validated — scan-to-GT-map registration, and
the full route tracked at **real-time playback rate 1.0** with
[`config/gt_ouster_ndt_realtime.yaml`](config/gt_ouster_ndt_realtime.yaml).

> The third-party packages in `src/` are used at the commits pinned in
> [`hmr_localisation.repos`](hmr_localisation.repos). The NDT package carries one
> tracked patch in [`patches/`](patches/): the registration cloud keep-alive memory
> fix (`4096 → 4`; see [Configuration & frames](#configuration--frames)). Apply it
> after fetching the sources (Setup step 2). All other custom work is in `config/`,
> `scripts/`, `docker/`, `compose.yaml`.

> Earlier exploration — two **alternative localizers** (libpointmatcher **ICP** on
> Humble, and **GLIM** LiDAR-IMU SLAM on CUDA), multi-robot wiring, and
> map-resolution sweeps — lives on the **`experiments`** branch (and the matching
> tag). `main` is kept lean: the single, Docker-deployable NDT stack.

📖 **Detailed findings & methodology:** [docs/localization_findings.md](docs/localization_findings.md)

---

## Project layout
The rosbag lives **next to** the repo (it is huge and shared), not inside it:
```
HMR_Explo/                    # parent dir (not a git repo)
├── bags/                     # <-- you place the rosbag here (large, NOT in git)
└── hmr_localisation/         # this repo
    compose.yaml              # Jazzy container (mounts ./ at /ws + ../bags at /ws/bags)
    docker/Dockerfile         # osrf/ros:jazzy-desktop + colcon + mcap + robot_localization
    hmr_localisation.repos    # pinned third-party sources (reconstructs src/)
    config/
      gt_ouster_ndt.yaml          # Mode A localizer params (flat map -> os_lidar)
      gt_ouster_ndt_tree.yaml     # Mode B params (map -> odom -> base_link, IMU)
      gt_ouster_ndt_realtime.yaml # real-time rate-1.0 recipe (16 threads, reject off)
      gt_ouster_ndt_tree_jetson.yaml  # CPU-budget Mode B for a Jetson
      ekf_odom.yaml               # robot_localization EKF (odom -> base_link)
      localization.rviz           # RViz layout
    launch/ekf_odom.launch.py # NDT-pose + IMU EKF for the Mode B tree
    patches/                  # tracked NDT keep-alive memory fix
    scripts/                  # run / replay / evaluate helpers
    gt_map/gt_map.ply         # ground-truth map (committed, ~48 MB, 3.06 M pts)
    gt_map/gt_map_ds.ply      # 0.2 m downsample (for the Jetson config)
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
After fetching (either option), apply the NDT keep-alive memory patch (`4096 → 4`;
see [Configuration & frames](#configuration--frames)) — without it a fresh checkout
keeps the high-RAM upstream default:
```bash
git -C src/lidar_localization_ros2 apply ../../patches/lidar_localization_ros2-keepalive-count.patch
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

### Localization (headless, Mode A)
Localizes against `gt_map.ply` and dumps the trajectory to `output/path.csv`.
```bash
docker compose exec ros bash /ws/scripts/run_localization.sh        # full bag
docker compose exec ros bash /ws/scripts/run_localization.sh 180    # first 180 s
```

### Real-time (rate 1.0, the validated recipe)
[`config/gt_ouster_ndt_realtime.yaml`](config/gt_ouster_ndt_realtime.yaml) is the
Mode A base with exactly **two** changes, and it tracks the **entire 500 s / ~337 m
route at live playback rate 1.0** (Ouster ~7.6 Hz) without diverging:
1. **`ndt_num_threads: 4 → 16`** — NDT_OMP parallelizes the per-point accumulation
   with zero effect on the result (pure speedup); lifts throughput from ~3.2 Hz
   (could not keep up, dropped scans, drifted) to ~7.6 Hz.
2. **`reject_above_score_threshold: true → false`** — the route has an elevated
   stretch (~325–420 s, z climbs to +3.5 m) where the live scan has many points
   matching no map surface, so NDT *fitness* spikes to 30–45 even though the *pose*
   is correct. With the reject gate **on**, those high-residual scans were dropped →
   pose frozen while the robot moved → divergence at ~340 s. With it **off** the
   localizer keeps applying NDT's (correct) estimate and rides straight through.

It loads the 0.5 m `gt_map_us050.pcd` by default (lightest footprint, ~3 cm relative
APE). Generate that map from the committed `gt_map.ply` once:
```bash
docker compose exec ros bash -lc '
  cd /ws && g++ -O2 -std=c++17 scripts/downsample_map_pcl.cpp -o downsample_map_pcl \
    $(pkg-config --cflags --libs pcl_common pcl_io pcl_filters) -lpcl_kdtree -lpcl_search -lpcl_octree &&
  ./downsample_map_pcl gt_map/gt_map.ply gt_map/gt_map_us050.pcd 0.5 uniform'
```
Then point any of the run scripts at it (`localization_param_dir:=/ws/config/gt_ouster_ndt_realtime.yaml`),
e.g.:
```bash
docker compose exec ros bash /ws/scripts/run_viz_tree.sh "" 1.0 /ws/config/gt_ouster_ndt_realtime.yaml
```
(NDT also loads `.ply` directly, so you can leave `map_path` as `gt_map.ply` and skip
the downsample — it just runs heavier.)

### Localization with a proper TF tree (`map → odom → base_link`, IMU integrated)
`run_localization.sh` publishes a single flat `map → os_lidar`. For a REP-105 tree
with IMU integrated, use:
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
launched by `launch/ekf_odom.launch.py`). The robot has no wheel encoders, so —
rather than a second scan matcher — the EKF fuses the **single NDT pose**
(`/pcl_pose`, restamped to the `odom` frame by `scripts/ndt_pose_relay.py` →
`/pcl_pose_odom`) with **`/imu/data`** angular velocity (hdl_localization /
Autoware `ekf_localizer` style): the IMU smooths **orientation** at high rate
between the (10–30 Hz) NDT poses, NDT bounds the drift, and you get a genuine
continuous `odom → base_link` from one scan matcher. (Gyro-only, so the IMU does
*not* dead-reckon translation — between NDT poses position is carried by the
filter's constant-velocity model.) The localizer (Mode B) publishes
`map → odom = map→base · (odom→base)⁻¹`, so the `map → base` *product* is unchanged
while `odom` becomes a real smoothing layer. Feeding the pose restamped to `odom`
(not `map`) avoids a circular `map → odom` TF lookup — see
`scripts/ndt_pose_relay.py`.

> Requires `ros-jazzy-robot-localization` (already in `docker/Dockerfile`).

**A/B baseline without the EKF.** `scripts/run_localization_tree_noekf.sh` is the
same Mode B tree but with `odom → base_link` as a static identity instead of the
EKF, so the only difference is the missing smoothing layer (it also brings up RViz).

### Visualize in RViz
Live scan-on-map (heavier — localizer + RViz + bag play together):
```bash
xhost +local:                                   # on the host, once per login
docker compose exec -e DISPLAY=:1 ros bash /ws/scripts/run_viz.sh        # Mode A
docker compose exec -e DISPLAY=:1 ros bash /ws/scripts/run_viz_tree.sh   # Mode B + EKF
```
RViz (`config/localization.rviz`, fixed frame `map`): green = GT map (by height),
red = live Ouster scan placed in the map frame by localization, green line =
trajectory (`/path`), yellow = current pose (`/pcl_pose`).

For a **lightweight replay** of a recorded result (no NDT, no 53 GB bag — smooth on
software GL), `scripts/analysis/replay_result.py` publishes a voxel-downsampled map
and animates a `output/path*.csv` trajectory:
```bash
docker compose exec -d ros bash -lc \
  'source /opt/ros/jazzy/setup.bash; source /ws/install/setup.bash; python3 /ws/scripts/analysis/replay_result.py output/path.csv'
docker compose exec -d -e DISPLAY=:1 ros bash -lc \
  'source /opt/ros/jazzy/setup.bash; ros2 run rviz2 rviz2 -d /ws/config/localization.rviz'
```

### Measure accuracy
Records a short results bag, then reports scan-to-GT-map nearest-neighbour error:
```bash
# (with the localizer running and the bag playing — see scripts/run_localization.sh)
docker compose exec ros bash -lc \
  'source /opt/ros/jazzy/setup.bash; cd /ws; mkdir -p output; ros2 bag record -o output/acc_bag /ouster/points /pcl_pose'
python3 scripts/analysis/analyze_from_bag.py gt_map/gt_map.ply output/acc_bag   # needs: pip install rosbags scipy numpy
```
Overlay a trajectory CSV on the map: `python3 scripts/plot_zoom.py gt_map/gt_map.ply output/path.csv output/eval.png`.

### Stop
```bash
docker compose stop      # keep container   |   docker compose down  # remove it
```

### Run on a Jetson (Docker)
Same repo, same pinned NDT sources ([`hmr_localisation.repos`](hmr_localisation.repos):
`lidar_localization_ros2` + `ndt_omp_ros2`). NDT_OMP is **CPU-only**, so no CUDA is
needed — the `osrf/ros:jazzy-*` base in [`docker/Dockerfile`](docker/Dockerfile) is
multi-arch and pulls the `arm64` layer when you `docker compose build` **on the
Jetson** (JetPack 6 / L4T r36, Ubuntu 24.04). Setup/build/run are otherwise
identical to above (the `ros` service in [`compose.yaml`](compose.yaml)) — including
the NDT keep-alive memory patch
([`patches/lidar_localization_ros2-keepalive-count.patch`](patches/), `4096 → 4`,
applied in Setup step 2), which matters most on the memory-constrained Jetson: the
upstream default retains ~every registration cloud and grows RAM unboundedly.

Two Jetson-specific things:
- **Config:** use [`config/gt_ouster_ndt_tree_jetson.yaml`](config/gt_ouster_ndt_tree_jetson.yaml)
  (point `localization_param_dir` in [`scripts/run_localization_tree.sh`](scripts/run_localization_tree.sh)
  at it). It is the Mode B tree config tuned to leave CPU headroom for a co-resident
  Nav2 + planner stack: lighter 0.2 m map, crop radius 60 m, `enable_debug` off — and
  **set `ndt_num_threads` for your board** (Orin Nano → 2, Orin NX → 3, AGX Orin → 4–6).
- **Map:** it loads the 0.2 m-downsampled `gt_map/gt_map_ds.ply` (~1/3 the points);
  regenerate with `python3 scripts/downsample_map.py gt_map/gt_map.ply gt_map/gt_map_ds.ply 0.2`.

---

## Results
| | |
|---|---|
| Scan→map registration (median / RMS) | **4.8 cm / 6.8 cm** |
| within 10 cm / 20 cm | 90% / 99% |
| Real-time rate 1.0, full route | tracked end-to-end (0.5 m vs full map agree to 37 mm mean / 58 mm p95) |

GPS (`/fix`) is **not** a usable reference here (~20 m std-dev, not RTK). For an
absolute APE-in-metres number, export the GLIM trajectory and compare with the
package's `evo`-based tooling.

## Configuration & frames
Three NDT configs:
- `config/gt_ouster_ndt.yaml` — **Mode A**, flat `map → os_lidar` (simplest;
  `run_localization.sh`).
- `config/gt_ouster_ndt_tree.yaml` — **Mode B**, full `map → odom → base_link`
  tree with IMU (`run_localization_tree.sh`).
- `config/gt_ouster_ndt_realtime.yaml` — Mode A tuned for **rate-1.0 real-time**
  (16 threads, reject gate off). See [Real-time](#real-time-rate-10-the-validated-recipe).

- NDT_OMP, `ndt_resolution: 2.0`, 50 iters, local-map crop r=80 m (robust at
  turns). Drop `ndt_resolution` to 1.0 for max accuracy in structured areas.
- **Registration cloud keep-alive = 4**
  (`src/lidar_localization_ros2/src/lidar_localization_component.cpp`,
  `kRegistration{Source,Target}CloudKeepAliveCount`). NDT_OMP/GICP hold pointers
  into their input source/target clouds, so the node retains the last *N* clouds
  to avoid a use-after-free (including the shutdown-leak path). The pinned upstream
  value was **4096** (retain ~everything → large, ever-growing RAM); cutting it to
  **4** dropped memory markedly with **no accuracy cost** — the deques are
  write-only lifetime buffers, never read back into registration (alignment always
  uses only the latest cloud). **2** is the theoretical floor (current + previous);
  **4** keeps a safety margin.
- The bag's `/tf_static` carries the **full robot URDF** rooted at `base_link`
  (`base_link → os_lidar`, `base_link → imu`, wheels, camera, …) — it is *not*
  camera-only. **Mode A** localizes `os_lidar` directly (identity seed, since the
  GLIM map is built in the lidar frame); **Mode B** localizes `base_link`,
  re-seeding the initial pose to `(base_link → os_lidar)⁻¹`.

## Gotchas
- **PLY loads directly** (`pcl::io::loadPLYFile`) — no PLY→PCD conversion needed
  (the realtime config's `.pcd` is only for the lighter 0.5 m default map).
- **IMU:** the lidar↔IMU extrinsic *is* in the bag (`base_link → imu`, yaw 90°).
  The earlier "preintegration hurt" result was because the flat Mode A run never
  published `/tf_static`, so the localizer had no extrinsic to rotate the IMU into
  the base frame — **not** a missing extrinsic. Mode B publishes the transform and
  sets `imu_preintegration_use_base_frame_transform: true`, so preintegration is
  valid there (verified: 0 IMU base-frame transform failures).

## Roadmap: multi-robot
Run one `lidar_localization_ros2` instance per robot in its own namespace
(`/robotN/...`) against this same `gt_map.ply`, all publishing into the shared
`map` frame — relative poses then fall out directly. (Wiring for this, plus the ICP
and GLIM alternative localizers, is on the `experiments` branch.)
