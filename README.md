# hmr_localisation

Map-based 3D LiDAR localization against a GLIM ground-truth point-cloud map
(`gt_map/gt_map.ply`), using
[rsasaki0109/lidar_localization_ros2](https://github.com/rsasaki0109/lidar_localization_ros2)
(NDT). Everything runs in a **ROS 2 Jazzy Docker container**, driven from a
recorded **Ouster + IMU rosbag**.

**Status:** single-robot localization validated — scan-to-GT-map registration

> The third-party packages in `src/` are used at the commits pinned in
> [`hmr_localisation.repos`](hmr_localisation.repos). The NDT and ICP packages
> each carry small **tracked patches** in [`patches/`](patches/): NDT gets the
> registration cloud keep-alive memory fix (`4096 → 4`; see
> [Configuration & frames](#configuration--frames)), and `icp_localization_ros2`
> two IMU-buffer patches (see [Alternative localizer: ICP](#alternative-localizer-icp-libpointmatcher-humble)).
> Apply them after fetching the sources (NDT in Setup step 2; ICP in its build steps).
> All other custom work is in `config/`, `scripts/`, `docker/`, `compose.yaml`.

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
After fetching (either option), apply the NDT keep-alive memory patch (`4096 → 4`;
see [Configuration & frames](#configuration--frames)) — without it a fresh checkout
keeps the high-RAM upstream default:
```bash
git -C src/lidar_localization_ros2 apply patches/lidar_localization_ros2-keepalive-count.patch
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

## Alternative localizer: ICP (libpointmatcher, Humble)

[baiyeweiguang/icp_localization_ros2](https://github.com/baiyeweiguang/icp_localization_ros2)
(a fork of ETHZ's libpointmatcher-based ICP localizer) is wired up as an
alternative to NDT. It localizes the `os_lidar` frame against the same GLIM map
and publishes `map → range_sensor` (a flat Mode-A-style edge); a helper node adds
an optional REP-105 `map → odom → base_link` tree — see
[ICP with a proper TF tree](#icp-with-a-proper-tf-tree-map--odom--base_link-like-ndt-mode-b).

It runs in a **separate ROS 2 Humble container** (`ros_humble` /
`hmr_loc_humble`), because the package targets Humble and its `libpointmatcher`
CMake builds cleanly there, whereas Jazzy's does not propagate the
`yaml-cpp::yaml-cpp` target. It builds into `install_humble/` so it never touches
the Jazzy NDT workspace in `install/`.

```bash
# 1) fetch sources + apply the two tracked patches (see "patches" note below)
vcs import src < hmr_localisation.repos        # adds src/icp_localization_ros2
git -C src/icp_localization_ros2 apply patches/icp_localization_ros2-imu-queue-depth.patch
git -C src/icp_localization_ros2 apply patches/icp_localization_ros2-imu-buffer-clamp.patch

# 2) build the Humble image + workspace
docker compose build ros_humble
docker compose up -d ros_humble
docker compose exec ros_humble bash -lc '
  source /opt/ros/humble/setup.bash && cd /ws &&
  rosdep install --from-paths src/icp_localization_ros2 --ignore-src -r -y \
    --skip-keys "pointmatcher pointmatcher_ros" &&
  colcon build --packages-select icp_localization_ros2 \
    --build-base build_humble --install-base install_humble \
    --cmake-args -DCMAKE_BUILD_TYPE=Release'

# 3) run (wall_seconds + rate are optional; empty wall_seconds = full bag)
docker compose exec ros_humble bash /ws/scripts/run_localization_icp.sh          # full bag, rate 1.0
docker compose exec ros_humble bash /ws/scripts/run_localization_icp.sh 120 0.5  # 120 s wall @ 0.5x
# overlay the result on the map (matplotlib ships in the Humble image):
docker compose exec ros_humble bash -lc \
  'cd /ws && python3 scripts/plot_zoom.py gt_map/gt_map_ds03.ply output/path_icp.csv output/eval_icp.png'
```
Result trajectory → `output/path_icp.csv`, overlay → `output/eval_icp.png`.

**What the run script handles for you**
- **PCD map.** ICP loads PCD (`pcl::io::loadPCDFile`), not PLY, and matches each
  scan against the *whole* map (no NDT-style local crop). It uses the
  **downsampled** `gt_map_ds03.pcd` (494k pts; auto-generated from
  `gt_map_ds03.ply` via [`scripts/ply_to_pcd.py`](scripts/ply_to_pcd.py)) so ICP
  stays fast enough to keep up with the IMU stream. The full 3.06M-pt map is too
  slow (ICP fell ~5 s behind and crashed on a stale IMU lookup).
- **Bag compatibility.** The bag is rosbag2 v9 (Jazzy); Humble's rosbag2 can't
  parse its `metadata.yaml`. The script builds a non-destructive sibling
  (`…__humble/`, symlinked mcap + `ros2 bag reindex`) — the original bag is
  untouched.
- **QoS.** [`config/icp_qos_overrides.yaml`](config/icp_qos_overrides.yaml)
  republishes **both** played topics RELIABLE: `/ouster/points` so the reliable
  ICP subscriber connects, and `/imu/data` because after reindexing, bag play
  can't parse the per-topic offered QoS in the metadata (every played topic needs
  an override entry or playback aborts).

**Config** ([`config/gt_ouster_icp.yaml`](config/gt_ouster_icp.yaml),
[`config/icp_gt.yaml`](config/icp_gt.yaml),
[`config/input_filters_ouster.yaml`](config/input_filters_ouster.yaml))
- Identity initial pose (GLIM map is in the lidar frame). `is_use_odometry:false`
  → IMU extrapolates the pose between scans.
- `calibration.imu_to_range_sensor` is **T_imu←lidar** (the node applies
  `imuToLidar⁻¹·dT_imu·imuToLidar`): `(0, −0.0485, 0.389)`, yaw `π/2`, derived
  from the bag `/tf_static`. **Note:** calibration RPY is in **radians** (only
  `initial_pose` RPY is degrees).
- `icp_gt.yaml`: point-to-plane, KDTree knn 1, 20 ICP iterations (trimmed for
  speed).

**The patches** (in [`patches/`](patches/), both applied after `vcs import`).
1. **`…-imu-queue-depth.patch`** — the IMU subscription was `KeepLast(1)`. With a
   500 Hz IMU and the single-threaded executor stalled deserializing each ~131k-pt
   cloud, that coalesces away almost all IMU samples, so the interpolation buffer
   lags the scans and the node aborts (`ImuInterpolationBuffer: Missing
   measurement`). The patch raises the IMU queue to `KeepLast(2000)` so the
   executor drains the backlog in order after each cloud.
2. **`…-imu-buffer-clamp.patch`** — `ImuTracker::fillIntegrationBuffer` clamped
   `start` only *up* to `earliest` and `end` only *down* to `latest`, so a scan
   that momentarily ran ahead of the newest buffered IMU (`start > latest_time()`,
   e.g. a brief IMU gap in the bag) left `start` above `latest` and
   `getRawReadings` aborted with `Missing measurement`. The patch clamps **both**
   endpoints into `[earliest, latest]`; a degenerate window just means no IMU
   extrapolation for that one scan instead of a crash. (Found when the full-bag run
   reached ~307 s before this fired; with the patch it runs to the end.)

Even with both patches the executor is single-threaded, so **play the full bag at
`rate ≤ 0.3`** — at rate ≥ 0.5 cloud processing (~0.2 s) saturates the executor
(clouds arrive every ~0.131 s of bag time), the IMU starves, and the 3 s
(`kBuffSize=1500`) buffer desyncs. At rate 0.3 it processes every cloud with no
IMU warnings.

### ICP with a proper TF tree (`map → odom → base_link`, like NDT Mode B)

```bash
docker compose exec ros_humble bash /ws/scripts/run_localization_icp_tree.sh        # full bag, rate 1.0
docker compose exec ros_humble bash /ws/scripts/run_localization_icp_tree.sh 120 0.5 # 120 s wall @ 0.5x
docker compose exec ros_humble bash -lc \
  'cd /ws && python3 scripts/plot_zoom.py gt_map/gt_map_ds03.ply output/path_icp_tree.csv output/eval_icp_tree.png'
```

Resulting tree (verified live with `tf2_echo`):

```
map
├─ range_sensor → inertial_sensor   (the ICP node's own Mode-A edges, untouched)
└─ odom → base_link → {os_lidar, imu}   (REP-105 tree, our publisher)
```

**Why a helper node, not native odom.** The package *does* support odometry — its
upstream `node_params.yaml` defaults to `is_use_odometry:true` +
`odometry_data_topic:/Odometry` and natively publishes `map → odom → odom_source →
range_sensor` (it was built for a FAST-LIO `/Odometry` source). The catch is **our
bag has no odometry source**: there is no `nav_msgs/Odometry` topic, and its `/tf`
carries only wheel-spin (`base_link → {wheels}`) plus URDF statics — *not*
`odom → base_link`. The node only emits `map → odom` when `is_use_odometry:true`,
which requires that `nav_msgs/Odometry` stream (`TfPublisher::odometryCallback` —
[`src/transform/TfPublisher.cpp`](src/icp_localization_ros2/src/transform/TfPublisher.cpp) L131-143;
the IMU-only odom path there is an empty `TODO` at L181). The native tree is also
`map → odom → odom_source → range_sensor` — there is **no `base_link` frame** in the
package, and the documented `is_provide_odom_frame` param is **dead** (never
declared/read; the flag is hard-set from `is_use_odometry` at
[`src/ICPlocalization.cpp`](src/icp_localization_ros2/src/ICPlocalization.cpp) L192).
So with only lidar + IMU, getting a `map → odom → base_link` tree means either
adding an odometry node (e.g. KISS-ICP / FAST-LIO publishing `/Odometry`) or — what
we do here — keeping `is_use_odometry:false` and letting
[`scripts/icp/icp_tree_publisher.py`](scripts/icp/icp_tree_publisher.py)
subscribes to the node's absolute `range_sensor_pose` (= `map → os_lidar`) and
rebuilds the canonical tree as a **disjoint frame branch** under `map`:
`map → odom` is dynamic (re-broadcast at scan rate), `odom → base_link` is a
**static identity** (no wheel odometry to smooth, exactly like the NDT
[noekf](#localization-with-a-proper-tf-tree-map--odom--base_link-imu-integrated)
baseline), and `base_link → {os_lidar, imu}` uses the **same extrinsics as NDT
Mode B**, so the two localizers' `map → base_link` are directly comparable. Since
the new frames (`odom`, `base_link`, `os_lidar`, `imu`) are disjoint from the ICP
node's own (`range_sensor`, `inertial_sensor`), no frame has two parents — both
branches coexist under `map`. `map → odom = map→os_lidar · (base→lidar)⁻¹`, so
with `odom → base_link` identity the full pose lives in `map → odom`.

Keep `is_use_odometry:false`: setting it true would make the ICP node *also*
publish `map → odom` and collide with our publisher's `odom` frame.

The run script prints a TF check 12 s into playback (`/tmp/tf_chain_icp.log`):
`map → base_link` resolving proves the tree is connected; `range_sensor → os_lidar`
having an **identity rotation** proves the extrinsic cancels. Its small
(~0.05–0.2 m) motion-correlated *translation* residual is **not** an error — it is
ICP's own TF-stamp latency (the node stamps `map → range_sensor` with `nh_->now()`,
~one processing delay after the scan, while our `map → odom` uses the true scan
stamp; our branch is the time-accurate one).

**Status (full bag, with the REP-105 tree):** runs the **entire 500 s bag** at
rate 0.3 with both patches — **3802 poses / ~348 m**, every cloud processed, zero
crashes and zero IMU warnings (`output/path_icp_tree.csv`, overlay
`output/eval_icp_tree.png`). It tracks the structured majority of the route
cleanly and stays within the map bounds throughout; only ~5 of 3801 inter-pose
steps exceed 1 m (≤1.6 m isolated snaps), with visibly noisier tracking in the
feature-sparse **back-third upper region (~y 30–37)** — the *same* region where
NDT Mode B also loses lock. No catastrophic divergence. (The red **X** in the
overlay is just the trajectory endpoint — `plot_zoom.py` always labels the last
pose "LOST LOCK here"; it is not an automated divergence flag.) Accuracy-vs-NDT
was descoped by the user.

## Alternative localizer: GLIM SLAM (+ SCovox), CUDA/Jazzy

[GLIM](https://github.com/koide3/glim) (koide3) is a LiDAR-IMU **SLAM** system —
unlike NDT/ICP it does **not** localize against the prebuilt `gt_map`; it builds
its **own** factor-graph map and trajectory online from `/ouster/points` +
`/imu/data`, and **SCovox maps on top of GLIM's live pose**. So this pipeline is
"GLIM SLAM → TF → SCovox occupancy mapping", with no prior map involved.

It runs in its **own ROS 2 Jazzy container** (`glim` / `hmr_loc_glim`), built
**FROM the official prebuilt image** `koide3/glim_ros2:jazzy_cuda13.1` (no
from-source GTSAM/gtsam_points/Iridescence build). That CUDA-13.1 image ships
`sm_120` cubins, so GLIM's GPU VGICP runs natively on the host **RTX 5070 Ti**
(Blackwell; driver 595.x / CUDA 13.2; docker default-runtime is `nvidia`). SCovox
is colcon-built **into the same container** at first run (`/scovox/install_glim`,
separate from the Jazzy-desktop `/scovox/install`), avoiding cross-distro DDS.

```bash
# 1) build the GLIM image (pulls koide3/glim_ros2:jazzy_cuda13.1 ~14 GB) + start it
docker compose build glim
docker compose up -d glim

# 2) run GLIM SLAM + SCovox on the bag (first run also colcon-builds SCovox).
#    args: [duration_s] [rate]   (empty duration = full bag; rate 0.5 = validated)
docker compose exec glim bash /ws/scripts/glim/run_glim_scovox.sh            # full bag @ 0.5
docker compose exec glim bash /ws/scripts/glim/run_glim_scovox.sh 60 0.5     # 60 s wall @ 0.5x

# 3) overlay GLIM's own map + trajectory (numpy + matplotlib ship in the image):
docker compose exec glim bash -lc \
  'cd /ws && python3 scripts/glim/plot_glim.py output/glim_map.pcd output/path_glim.csv output/eval_glim.png'
```
Outputs: GLIM trajectory → `output/path_glim.csv`, GLIM global map →
`output/glim_map.pcd`, SCovox occupancy map → `output/scovox_map.npy` (captured at
the end of the run; also published live on `/scovox_node/pointcloud`), overlay →
`output/eval_glim.png`. A validated full run: **5475 poses / 368 m / 16.8 M-pt
GLIM map**, no crash.

**Live RViz.** To *watch* GLIM SLAM + SCovox build live (SCovox occupancy boxes,
GLIM aligned scan + toggleable global map, GLIM odometry arrows, TF tree):
```bash
xhost +local:                 # host: let the container reach the X server (once per login)
docker compose up -d glim
docker compose exec -e DISPLAY=:1 glim bash /ws/scripts/glim/run_glim_scovox_viz.sh [dur] [rate]
```
RViz config: [`config/glim_scovox.rviz`](config/glim_scovox.rviz) (Fixed Frame
`map`; toggle "GLIM Global Map" / "Live Scan" on as desired). The script renders
on the **NVIDIA GPU** (`__GLX_VENDOR_LIBRARY_NAME=nvidia`, hardware GL 4.6) —
without it the GLVND loader picks the Intel `iris` MESA driver, which can't get a
DRM device in the container and RViz gets no GL. If hardware GL still fails, set
`LIBGL_ALWAYS_SOFTWARE=1` for software rendering.

**Frame wiring.** GLIM (`librviz_viewer.so`) broadcasts the whole TF tree
`map → odom → imu → os_lidar` (it auto-detects `os_lidar` from the cloud and
`imu` from the IMU header; `imu → os_lidar` uses the configured `T_lidar_imu`).
We replay **only** `/ouster/points` + `/imu/data` (not the bag's `/tf`,`/tf_static`)
so GLIM owns the tree with no duplicate-parent conflict. SCovox
([`config/scovox_lidar_glim.yaml`](config/scovox_lidar_glim.yaml)) integrates in
GLIM's **`odom`** frame (smooth, jump-free) via `odom → os_lidar`; set
`integration_frame: "map"` instead for the globally loop-closed (but
seam-prone) frame.

**Config** ([`glim_config/`](glim_config/), passed via `-p config_path:=`)
- `config_sensors.json` → `T_lidar_imu = [0.0485, 0, -0.389, 0, 0, -0.70711, 0.70711]`
  = `inv(T_base_lidar)·T_base_imu` from the bag's `/tf_static` (transforms imu→lidar).
- `config_ros.json` → topics `/ouster/points` + `/imu/data`; the GUI
  `libstandard_viewer.so` is **disabled** (headless) while `librviz_viewer.so`
  (TF + `/glim_ros/*` topics) stays on.
- `config.json` → GPU sub-configs (CUDA image). **CPU fallback:** build with
  `--build-arg BASE_IMAGE=koide3/glim_ros2:jazzy` and switch the three `*_gpu`
  sub-configs to `*_cpu` (else GLIM aborts loading `libodometry_estimation_gpu.so`).

**What the run script handles**
- **SCovox build** into `/scovox/install_glim` on first run (`-DBUILD_TESTING=OFF`,
  since the lean GLIM `ros-base` image lacks the gtest test deps).
- **Reliable QoS (critical).** GLIM's points subscriber is set **RELIABLE +
  depth 100** in `config_ros.json`. With the default best-effort `sensor_data`
  QoS, GLIM dropped ~95% of clouds while its GPU initialized → a fatal *"large
  time gap between consecutive LiDAR frames"* (`IndexedSlidingWindow: index out
  of range`). The bag is replayed RELIABLE (`config/ouster_reliable_qos.yaml`),
  so the reliable sub drops nothing.
- **Headless map save.** GLIM has no headless save service (it lives in the
  Iridescence GUI), so [`scripts/glim/save_glim_map.py`](scripts/glim/save_glim_map.py)
  subscribes to the latched `/glim_ros/map` and writes a PCD on shutdown;
  [`record_glim_pose.py`](scripts/glim/record_glim_pose.py) logs `/glim_ros/pose`
  to CSV (same columns the plotters expect).

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
- **Registration cloud keep-alive = 4**
  (`src/lidar_localization_ros2/src/lidar_localization_component.cpp`,
  `kRegistration{Source,Target}CloudKeepAliveCount`). NDT_OMP/GICP hold pointers
  into their input source/target clouds, so the node retains the last *N* clouds
  to avoid a use-after-free (including the shutdown-leak path). The pinned upstream
  value was **4096** (retain ~everything → large, ever-growing RAM); cutting it to
  **4** dropped memory markedly with **no accuracy cost** — the deques are
  write-only lifetime buffers, never read back into registration (alignment always
  uses only the latest cloud). **2** is the theoretical floor (current + previous);
  **4** keeps a safety margin (e.g. async/retry overlap, the shutdown leak).
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
