# hmr_localisation — code comment notes

Long comments from files in the `hmr_localisation` repository, moved out of the code on 2026-09-23 so the sources carry short comments only. Where a comment was moved, the code keeps a short gist ending in `(notes: <id>)`; the section headed `<id>` below holds the original comment, word for word. Files with many moved comments have their own notes doc next to this one.

One section per source file, in file order. Each entry names the function (or section) the comment sat in, the line of code it was attached to and its original line number. Line numbers, dates, generation numbers and cross-references inside the moved text are as they were when written; they record history and are not maintained.

## Contents

- [config/ekf_odom.yaml](#configekf_odomyaml) — 2
- [config/ekf_odom_bunker.yaml](#configekf_odom_bunkeryaml) — 1
- [config/ekf_odom_go1.yaml](#configekf_odom_go1yaml) — 1
- [config/gt_ouster_ndt_realtime.yaml](#configgt_ouster_ndt_realtimeyaml) — 1
- [config/gt_ouster_ndt_tree_bunker.yaml](#configgt_ouster_ndt_tree_bunkeryaml) — 2
- [config/gt_ouster_ndt_tree_curt.yaml](#configgt_ouster_ndt_tree_curtyaml) — 2
- [config/gt_ouster_ndt_tree_fused.yaml](#configgt_ouster_ndt_tree_fusedyaml) — 1
- [config/gt_ouster_ndt_tree_go1.yaml](#configgt_ouster_ndt_tree_go1yaml) — 2
- [launch/coop_multi_robot_localization.launch.py](#launchcoop_multi_robot_localizationlaunchpy) — 1

## config/ekf_odom.yaml

### ekf-odom-design

**EKF odom layer design** — in `robot_localization EKF — continuous `odom -> base_link` for the Mode B tree`, attached to `ekf_filter_node:` (line 12)

```text
Design (hdl_localization / Autoware ekf_localizer style, one scan matcher):
  * pose0 = the NDT global pose (/pcl_pose) restamped into the odom frame by
    scripts/ndt_pose_relay.py -> /pcl_pose_odom. This is the absolute,
    drift-bounded measurement.
  * imu0 = /imu/data angular velocity, for high-rate ORIENTATION smoothing
    between the (10-30 Hz) NDT poses. (Gyro-only -> it does NOT dead-reckon
    translation; between NDT poses position rides the constant-velocity model.
    See the imu0 notes below to add acceleration fusion.)
  The EKF runs with world_frame=odom, so it BROADCASTS odom -> base_link.
  The localizer then publishes map -> odom = map->base_raw o (odom->base)^-1,
  so the map->base PRODUCT still equals the raw NDT pose (SCovox, which looks
  up map->os_lidar, is unaffected) while odom->base becomes smooth, high-rate
  and continuous.

Why feed the NDT pose restamped to `odom` (not map): with world_frame=odom the
EKF would otherwise transform the map-frame pose via map->odom -- which is
produced FROM this EKF's output (circular / startup deadlock). map and odom are
coincident up to the residual the localizer keeps in map->odom, so restamping
is the standard, stable resolution. See scripts/ndt_pose_relay.py.
```

### ekf-odom-imu-fusion

**Gyro-only IMU fusion** — in `IMU (/imu/data, frame "imu")`, attached to `imu0: /imu/data` (line 73)

```text
Fuse angular velocity only (vroll, vpitch, vyaw) -> high-rate orientation
smoothing between NDT poses. Gyro is rotated into base_link via the static
base_link->imu TF, so it is always valid regardless of whether /imu/data
carries a fused orientation quaternion.

NOT fused by default:
  * absolute orientation (roll/pitch/yaw): owned by the NDT pose. This bag
    publishes /imu/mag separately, which usually means /imu/data has no
    fused orientation -- so do not trust it here.
  * linear acceleration (ax/ay/az): noisy and needs gravity removal (which
    in turn needs a valid IMU orientation). To enable it, set ax/ay/az true
    below AND keep imu0_remove_gravitational_acceleration: true, but only if
    your /imu/data actually reports a valid orientation.
```

## config/ekf_odom_bunker.yaml

### ekf-odom-bunker-frames

**Bunker bag frames and IMU** — in `robot_localization EKF — continuous `odom -> base_link` for the Mode B tree`, attached to `ekf_filter_node:` (line 4)

```text
Fork of ekf_odom_go1.yaml for the 2026_07_06 "bunker_kalhan_coop" bag (AgileX
Bunker tracked robot + Hesai LiDAR, SAME coop environment / gt_map as the go1
and curt bags). Keeps the stock `base_link` frame — the bag's /tf_static (a
large CAD-assembly tree) has base_link as its ROOT with the sensors below it:

  base_link --(Rz 90)--> ... --> hesai_lidar   (LiDAR cloud frame, LEVEL)
  base_link --(Rz 90)--> ... --> imu           (/imu/data frame, LEVEL)

base_link is the TF ROOT (no odom->base_link in the bag), so broadcasting
map->odom (NDT) + odom->base_link (this EKF) does NOT collide with replay.

IMU: /imu/data (frame "imu", near-level body IMU, |a|~9.8 gravity +Z). This bag
has NO /hesai/imu, so /imu/data serves both the EKF and the mapper deskew. Fuse
angular velocity only; the gyro is rotated into base_link via the static
base_link->..->imu TF, valid whether or not /imu/data carries a fused quaternion.
```

## config/ekf_odom_go1.yaml

### ekf-go1-frames-and-imu

**Go1 EKF frame and IMU choices** — in `robot_localization EKF — continuous `odom -> base_link` for the Mode B tree`, attached to `ekf_filter_node:` (line 4)

```text
Fork of ekf_odom.yaml for the 2026_06_29 "go1_kalhan_coop_2" bag (Unitree Go1
+ Hesai LiDAR, SAME coop environment / gt_map as the curt bag). Unlike the curt
bag this stack keeps the stock `base_link` frame, because the Go1 URDF already
publishes the full static chain on /tf_static:

  base_link --(Rz 90)--> trunk --> hesai --> hesai_lidar   (LiDAR cloud frame)
                               \--> imu                     (/imu/data frame)

base_link is the bag's TF ROOT (no odom->base_link in the bag), so broadcasting
map->odom (NDT) + odom->base_link (this EKF) does NOT collide with replay.

IMU: /imu/data (frame "imu", the near-level body IMU). Fuse angular velocity
only; the gyro is rotated into base_link via the static base_link->..->imu TF,
so it is valid whether or not /imu/data carries a fused orientation quaternion.
(The LiDAR's own /hesai/imu is reserved for mapper deskew — see the mapper cmd.)
```

## config/gt_ouster_ndt_realtime.yaml

### ndt-realtime-recipe

**Real-time NDT threads and reject gate** — in `Top level`, attached to `/**:` (line 1)

```text
Real-time NDT localization config — runs the full route at live playback rate 1.0
(Ouster ~7.6 Hz) without diverging. Two parameters (vs a stock NDT setup) carry
the recipe, and both were validated empirically:

  1) ndt_num_threads: 4 -> 16
     NDT_OMP parallelizes the per-point gradient/Hessian accumulation across
     threads with ZERO effect on the result — pure speedup on the 24-core host.
     Lifts throughput from ~3.2 Hz (could not keep up, skipped scans, drifted)
     to ~7.6 Hz so every live scan is processed. Free, no accuracy cost.

  2) reject_above_score_threshold: true -> false
     The bag has an elevated stretch (~325-420 s, z climbs to +3.5 m) where the
     live scan has many points that match no map surface, so NDT *fitness* spikes
     to 30-45 even though the *pose* is correct. With the reject gate ON, those
     high-residual scans were dropped -> pose frozen while the robot moved ->
     permanent divergence at ~340 s. With the gate OFF the localizer keeps
     applying NDT's (correct) estimate and rides straight through; fitness then
     recovers to ~0 afterwards, proving the pose stayed locked.

Validation: 0.5 m map and the full 3 M-pt map (both with this config) complete
the entire 500 s route and agree to 37 mm mean / 58 mm p95 over the whole route,
including 38 mm through the hard elevated zone — i.e. both track correctly.
```

## config/gt_ouster_ndt_tree_bunker.yaml

### ndt-bunker-bag-wiring

**Bunker bag sensor and frame wiring** — in `Top level`, attached to `/**:` (line 1)

```text
NDT config for the 2026_07_06 "bunker_kalhan_coop" bag (AgileX Bunker tracked
robot + Hesai LiDAR) over the SAME gt_map as the curt/go1/map-test-2 bags.
Fork of gt_ouster_ndt_tree_go1.yaml. What differs from the go1 bag:

  1. LiDAR is again a Hesai on /hesai/points (frame hesai_lidar), 230400 pts,
     per-point time field `timestamp` FLOAT64 (offset 18) — same decode as go1.
  2. On bunker the Hesai is mounted LEVEL, same as go1 physically — both scan a
     horizontal plane. The difference is only the FRAME axis convention: go1's
     hesai_lidar frame is rolled (its +Z points sideways), whereas bunker's is
     upright — the composed base_link->hesai_lidar from /tf_static is a pure
     Rz +90 with base_link +Z mapping to hesai_lidar +Z (see below). base_frame_id stays the
     stock `base_link`, which is the bag's TF ROOT (no odom->base_link in the
     bag), so Mode B (map->odom NDT + odom->base_link EKF) doesn't collide.
  3. There is NO /hesai/imu on this bag. The body IMU /imu/data (frame "imu",
     near-level, |a|~9.8 gravity +Z) drives NDT preintegration and the EKF; the
     mapper deskews with /imu/data too (hesai_lidar<-imu extrinsic from TF).

Sensor wiring — composed from the bag's /tf_static (a large CAD-assembly tree,
root base_link); play /tf_static on replay so tf2 can resolve these:
  base_link -> hesai_lidar : xyz(0.113,0.003,0.366)  q(0,0,0.7065,0.7077) Rz +90 (LEVEL)
  base_link -> imu         : xyz(0.096,0.040,0.116)  q(0,0,0.7077,0.7065) Rz +90 (LEVEL)
  hesai_lidar -> imu       : xyz(0.037,0.017,-0.250) q(0,0,0.0016,1.0)     ~identity rot
```

### ndt-bunker-initial-pose

**Bunker initial pose derivation** — in `initial pose: base_link pose in map (LEVEL — base_link is gravity-aligned)`, attached to `set_initial_pose: true` (line 62)

```text
Pinned geometrically by structural scan-to-gt_map overlap (99.8% inliers,
scan transformed hesai_lidar->base_link via base_link->hesai_lidar above).
x~2.50 y~-0.25 yaw~194 deg (robot heads the opposite way from go1's ~106).
z: map ground ~-0.82 + base height ~0.36 -> -0.46.
```

## config/gt_ouster_ndt_tree_curt.yaml

### ndt-curt-bag-wiring

**Curt bag frames and sensor wiring** — in `Top level`, attached to `/**:` (line 1)

```text
NDT config for the 2026_07_06 "curt_kalhan_coop" bag over the SAME gt_map.
Fork of gt_ouster_ndt_tree_fused.yaml. Two things differ, both because this
bag's TF tree is namespaced with a `_curt` suffix and the robot starts facing
the opposite side of the same environment:

  1. base_frame_id: base_link_curt   (vs base_link)
  2. initial_pose: same start spot as map-test-2 but heading rotated ~165 deg
     (old base heading was map-yaw 180 deg; new is ~-15 deg / map +x).

Sensor wiring (all from the bag's /tf_static, so play /tf_static on replay):
  base_link_curt -> os_sensor -> os_lidar : (0.1105,0,0.404) quat(0,0,1,0)  yaw pi
                                            (identical rig to map-test-2)
  base_link_curt -> imu_curt              : (0.062,0,0.015)  quat(0,0,-0.70711,0.70711)
  /curt/imu/data frame_id == imu_curt  ->  NO imu_link->imu alias needed
    (unlike the map-test-2 bring-up, whose /imu/data frame_id was "imu").
```

### ndt-curt-initial-pose

**Curt initial pose derivation** — in `initial pose: base_link_curt pose in map`, attached to `set_initial_pose: true` (line 58)

```text
Pinned geometrically by structural scan-to-gt_map overlap (99.7% inliers):
start x~3.0 y~0.0, heading yaw ~13 deg. That is ~168 deg from the map-test-2
start (which was yaw 180 deg) -- i.e. facing the opposite side, as expected.
z is loosely constrained (2D overlap); local ground ~-0.84 m, NDT refines it.
```

## config/gt_ouster_ndt_tree_fused.yaml

### ndt-fused-sensor-statics

**Fused run sensor TF ownership** — in `Top level`, attached to `/**:` (line 12)

```text
publish_lidar_tf/publish_imu_tf:=false → NDT does map->odom ONLY and does NOT
broadcast the sensor legs; standalone static_transform_publishers own them
(values read from the bag's /tf_static — we do NOT play /tf_static: its
base_link->camera_link is an uncalibrated identity, and playing it would
double-parent os_lidar against the live tree):
  base_link -> os_lidar           : x 0.1105  y 0 z 0.404   quat(0,0,1,0)  (yaw pi)
  base_link -> imu                : x 0.062   y 0 z 0.015   quat(0,0,0.7071068,0.7071068)
                                    (/imu/data frame_id is "imu", NOT the bag's "imu_link")
  base_link -> camera_color_frame : x 0.270676 y 0.049297 z 0.279109
                                    quat(-0.013514, 0.000986, 0.000780, 0.999908)
                                    (CALIBRATED; collapsed from the bag's
                                     os_lidar->camera_color_optical_frame leg)
```

## config/gt_ouster_ndt_tree_go1.yaml

### ndt-go1-bag-wiring

**Go1 bag sensor and frame wiring** — in `Top level`, attached to `/**:` (line 1)

```text
NDT config for the 2026_06_29 "go1_kalhan_coop_2" bag (Unitree Go1 + Hesai
LiDAR) over the SAME gt_map as the curt/map-test-2 bags. Fork of
gt_ouster_ndt_tree_curt.yaml. What differs from the curt bag:

  1. LiDAR is a Hesai on /hesai/points (frame hesai_lidar), NOT Ouster. The
     per-point time field is `timestamp` FLOAT64 (absolute) — the scovox mapper
     already decodes that; the NDT node only needs xyz so it is agnostic.
  2. base_frame_id stays the stock `base_link` — the Go1 URDF publishes the full
     static chain on /tf_static, and base_link is the bag's TF ROOT (no
     odom->base_link in the bag), so Mode B doesn't collide with replay:
       base_link --(Rz 90)--> trunk --> hesai --(Rz 114.6)--> hesai_lidar
                                   \--> imu   (/imu/data, near-level body IMU)
  3. The Hesai is mounted LEVEL — the physical scan plane is horizontal
     (verified: raw-cloud PCA disk-normal, pushed through base_link<-hesai_lidar,
     lands 9.4 deg off gravity = level within standing-pose noise). What looks
     like a tilt is only the hesai_lidar FRAME's rolled axis convention: its +Z
     points sideways (along base +Y) and +Y is the near-up axis. NDT is agnostic
     to that because base_frame_id=base_link is level and the cloud->base TF
     (from /tf_static) carries the frame rotation. Initial pose below is LEVEL.

Sensor wiring — all from the bag's /tf_static, so play /tf_static on replay:
  base_link -> trunk           : xyz(0,0,0)          q(0,0,0.7068,0.7074)  Rz +90
  trunk     -> hesai           : xyz(0.175,-0.020,0.189) q(0,0.7068,0,0.7074) Ry +90
  hesai     -> hesai_lidar     : xyz(0,0,0)          q(0,0,0.8415,0.5403)  Rz +114.6
  trunk     -> imu             : xyz(0.115,-0.008,0.269) q(0,0,0.7068,0.7074) Rz +90
```

### ndt-go1-initial-pose

**Go1 initial pose derivation** — in `initial pose: base_link pose in map (LEVEL — base_link is gravity-aligned)`, attached to `set_initial_pose: true` (line 66)

```text
Pinned geometrically by structural scan-to-gt_map overlap (99.8% inliers,
scan transformed hesai_lidar->base_link via the static chain above).
x~2.75 y~-0.50 yaw~106 deg (robot faces a different side than the curt bag,
whose yaw was ~13 deg). z: map ground ~-0.84 + base height ~0.74 -> -0.10.
```

## launch/coop_multi_robot_localization.launch.py

### coop-loc-shared-map-noekf

**Shared map frame and identity odom** — in `Two-robot NDT localization for the coop multi-robot map-merge experiment.`, attached to `import launch` (line 11)

```text
Because BOTH robots localize against the SAME gt_map they share ONE global
`map` frame — no robot-to-robot pose estimation. The two bags' frame names are
already disjoint (bunker: odom/base_link/hesai_lidar/imu; curt: odom_curt/
base_link_curt/os_lidar/imu_curt), so only the NDT node name + /pcl_pose need
namespacing (done here via the /<robot> namespace).

odom -> base is an IDENTITY static (the 'noekf' baseline): NDT then publishes
  map -> odom = map -> base_raw,
stamped per scan, so scovox resolves an exact-stamp map -> <sensor> for every
scan. This is the configuration verified end-to-end on the coop bags (aligned-
scan-vs-gt_map fitness ~0.2-0.4 m from the config's default near-origin seed —
NO custom initial pose needed). To add the smoothing robot_localization EKF
later, replace each identity static with the ekf_odom stack (see ekf_odom.yaml).
```
