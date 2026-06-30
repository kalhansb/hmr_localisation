# Localization — findings & methodology

Deep notes behind the single-robot localization result in the top-level
[README](../README.md). Covers how the system was brought up, the dead-ends, the
tuning, and how accuracy was measured.

---

## 1. Environment

| | |
|---|---|
| Host ROS | Humble |
| Bag | ROS 2 **Jazzy**, mcap, ~53 GB, 500 s |
| Map | `gt_map/gt_map.ply` — GLIM output, 3,059,991 pts, binary LE float32 (x,y,z,intensity) |
| Localizer | `lidar_localization_ros2` (NDT_OMP) + `ndt_omp_ros2` |

Host is Humble but the bag is Jazzy/mcap, so everything runs in a **Jazzy
container** (`compose.yaml` + `docker/Dockerfile`). Only two standard message
types are needed from the bag, so cross-distro concerns don't apply.

### Bag topics / frames
- `/ouster/points` — `sensor_msgs/PointCloud2`, frame **`os_lidar`** (OS-128, 1024 wide), **best-effort** QoS
- `/imu/data` — `sensor_msgs/Imu`, frame `imu`, ~500 Hz
- `/fix` — GPS, frame `gps` (≈20 m std-dev — not RTK)
- `/tf_static` — **3 latched messages = the FULL robot static tree**, not just the
  camera. It contains `base_link → {os_sensor→os_lidar, camera_link→camera_color_frame
  →camera_color_optical_frame, imu_link, gps, …}`. ⚠️ Reading only ONE message
  (`ros2 topic echo --once`, or a non-`transient_local` listener) shows only the
  RealSense-internal subset — that is the mistake behind the old "camera-only" claim.
  Verified values (`tf2_echo`, 2026-06-29): `base_link→os_lidar` = t(0.111,0,0.404),
  yaw 180°; `base_link→camera_color_optical_frame` = t(0.271,0.049,0.279); and the
  derived LiDAR↔camera extrinsic `os_lidar→camera_color_optical_frame` =
  t(-0.160,-0.049,-0.125). So the camera/LiDAR extrinsic IS available from the bag.
- `/tf` — dynamic `base_link → wheels` joints (odometry-style)

---

## 2. Setup decisions & gotchas

- **DDS discovery.** `ROS_LOCALHOST_ONLY=1` silently breaks FastDDS participant
  discovery in the container — *no topics are ever discovered* (even
  talker/listener fail). Fix: `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` (set in
  `compose.yaml`). This cost real debugging time; it presents as "playback runs
  but nothing subscribes."
- **PLY loads directly.** The node detects the extension and uses
  `pcl::io::loadPLYFile` — no PLY→PCD conversion. The map's binary-LE-float32
  layout is exactly what it expects.
- **Frames.** ⚠️ **CORRECTION (2026-06-29):** the bag's `/tf_static` DOES contain
  `base_link → os_lidar` plus the full camera chain (see §1) — the earlier "camera-only"
  reading was an artifact of inspecting a single `tf_static` message. The text below is
  retained for history but is **superseded**: prefer localizing down to **`base_link`**
  (rely on the bag's `base_link→os_lidar`) so the entire sensor tree — including the
  RealSense camera — resolves in `map`. Do NOT publish `map→os_lidar` directly, and do
  NOT add a duplicate `base_link→os_lidar` static_tf (it would give `os_lidar` two
  parents and break the tree). This is what makes RGB-D + LiDAR fusion possible without
  any measured extrinsic.
  - *(superseded)* Because the bag was thought to lack a `base_link → os_lidar`
    transform, earlier runs localized the **`os_lidar` frame directly**
    (`base_frame_id:=os_lidar lidar_frame_id:=os_lidar publish_lidar_tf:=false`); the
    node's default `initial_pose_qw: 0.0` is an invalid quaternion — set to `1.0`.
- **Map units.** Local metric coordinates (~230 m span), not UTM, despite GPS
  being present.

---

## 3. Methodology & tuning journey

1. **Smoke test (90 s, NDT res 1.0).** Clean tracking, NDT fitness **~0.005 m²**
   (excellent). Confirms frames + seed are right. → `output/eval_90s.png`
2. **Full run, first attempt.** Lost lock partway. At 2× playback it failed
   early (dropped scans starved prediction); even at 1× it lost lock at a
   **mid-route maneuver** (~150 s) with NDT-only constant-velocity prediction.
3. **Hardened NDT** (`ndt_resolution 2.0`, `ndt_max_iterations 50`,
   `enable_local_map_crop` r=80 m, `score_threshold 5.0`). Tracked through the
   turn; failure pushed out to ~300 s. → `output/eval_robust180.png`,
   `output/eval_final_zoom.png`
4. **IMU preintegration — made it worse.** Diverged earlier (~130 s). The IMU is
   z-up/gravity-aligned (mean accel ≈ (−0.9, 0.1, 9.75)), and yaw-rate is
   frame-invariant about the shared z, *but* accel double-integration needs the
   gravity direction **in the working frame** plus the lever-arm — both wrong
   without the real `os_lidar → imu` extrinsic. Reverted. To use IMU: supply the
   extrinsic and set `imu_preintegration_use_base_frame_transform: true`.

### Diagnosis of the mid-route failure
It is **not** map sparsity: the failure location has ~800k map points within
25 m — comparable to well-tracked spots. It's a motion/geometry issue (fast
maneuver + locally weak constraints) that NDT-only prediction can't coast
through. The robot stays within a ~50 × 30 m area while the map spans ~230 m
(long Ouster range maps distant buildings), so the tracked portion is most of
the actual travel.

---

## 4. Accuracy

"Accuracy" needs a reference. What we have:

| Method | Verdict |
|---|---|
| NDT fitness score | Built-in proxy (mean sq. point-to-map dist); ~0.005 m² in good areas |
| **Scan → GT-map NN distance** | **Used** — the map *is* the ground truth |
| APE/RPE vs reference trajectory | Needs a reference path we don't have |
| GPS `/fix` | Unusable (~20 m std-dev, not RTK) |
| GLIM trajectory | Ideal reference, but no trajectory file in the workspace (only the map) |

**Measured (scan→GT-map registration, deployed res-2.0 config):**

| Metric | Value |
|---|---|
| Median NN distance | **4.8 cm** |
| RMS | 6.8 cm |
| 95th percentile | 12.5 cm |
| within 10 cm | 89.8 % |
| within 20 cm | 99.2 % |

This *upper-bounds* the pose error (it includes real surface thickness/foliage).
Reproduce with `scripts/analyze_from_bag.py` over a results bag of
`/ouster/points` + `/pcl_pose`. For an absolute APE-in-metres number, export the
GLIM trajectory (TUM) and use the package's `evo`-based tooling.

---

## 5. Visualization

- **Lightweight replay** (`scripts/replay_result.py`): publishes a
  voxel-downsampled map (3 M → ~196 k pts) and animates a recorded trajectory —
  no NDT, no 53 GB bag. Smooth even on software GL. → `output/rviz_replay.png`
- **Live** (`scripts/run_viz.sh`): localizer + RViz + bag together. Heavier;
  on software GL the 3 M-point render starves the localizer and it loses lock
  earlier. Use GPU passthrough (`--gpus all` + nvidia-container-toolkit) for
  smooth live viewing. → `output/rviz_live.png`

---

## 6. Known limitation & next step

NDT-only loses lock once at a mid-route maneuver. The fix is IMU/odom fusion with
the correct `os_lidar → imu` extrinsic (from the robot URDF or the GLIM config).
For the multi-robot goal, run one localizer per robot in its own namespace
against this same map — all in the shared `map` frame.
