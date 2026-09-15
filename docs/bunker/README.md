# Running hmr_localisation on the Bunker

VGICP matches the live Hesai scan against a prior map and publishes `map -> odom`,
stamped per scan. It does **not** build a map and does **not** drift-correct — if
the prior map is wrong or the robot leaves it, the pose is wrong.

Config to use: **`config/gt_ouster_ndt_tree_bunker_jetson.yaml`** (already tuned —
don't edit it to go faster).

---

## Quickstart

Already set up, robot wired, done this before:

```bash
docker compose up -d
docker compose exec -e ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET ros bash -lc '
  source /opt/ros/jazzy/setup.bash && source /ws/install/setup.bash &&
  ros2 launch lidar_localization_ros2 lidar_localization.launch.py \
    localization_param_dir:=/ws/config/gt_ouster_ndt_tree_bunker_jetson.yaml \
    cloud_topic:=/hesai/points imu_topic:=/imu/data \
    base_frame_id:=base_link use_sim_time:=false \
    publish_lidar_tf:=false publish_imu_tf:=false'
```

Then set the pose in RViz ([Step 3](#step-3)). First time through, start at Step 0.

---

## Step 0 — One-time setup

Follow [`README.md` → Setup](../../README.md). It covers `vcs import`,
`docker compose build`, `colcon build`, and generating `gt_map/gt_map_us050.pcd`
from `gt_map.ply`. **Nothing below works until that map file exists.**

```bash
ls -la gt_map/gt_map_us050.pcd     # ~3 MB. Missing => go do Step 0.
```

<a name="step-1"></a>
## Step 1 — Prove the stack works, on a bag

Do this before touching the robot. It removes sensors, networking and start-pose
guesswork from the picture, so if it fails you know it's the build.

```bash
./scripts/jetson_bag_test_native.sh bunker smoketest 120
```

Expect ~9.8 Hz and `state: OK`. If this fails, stop here — a live robot will not
go better.

Note this runs **natively** (ROS 2 Humble, the host build), while the Quickstart
runs **in the container** (Jazzy). They are two separate builds of the same
source, so a passing smoke test proves the config and the map, not the container
build. If Step 1 passes and Step 5 fails, suspect the container, not the tuning.

<a name="step-2"></a>
## Step 2 — Wire the robot

| Need | Check |
|---|---|
| Cloud | `ros2 topic hz /hesai/points` → ~10 Hz |
| IMU | `ros2 topic hz /imu/data` → gravity on +Z when level |
| Extrinsics | `ros2 run tf2_ros tf2_echo base_link hesai_lidar` resolves |
| Map | `gt_map_us050.pcd` covers where the robot will drive |

`map_path` in the config is `/ws/gt_map/gt_map_us050.pcd` — correct inside the
container. Running **natively**, override it with a real path or the node fails
activation with only `Failed to load pcd file`.

<a name="step-3"></a>
## Step 3 — Set the start pose

The one thing you must get right, and it's a property of the *run*, not the robot.

### A. In the config — the normal way

The config already does this, and the node comes up localized with no further
action. For a robot that always starts in the same place, set it once and forget it:

```yaml
set_initial_pose: true
initial_pose_x: 7.82
initial_pose_y: -4.11
initial_pose_z: -1.17
initial_pose_qx: 0.0
initial_pose_qy: 0.0
initial_pose_qz: 0.8550024006656964  # yaw 117.5 deg
initial_pose_qw: 0.5186240399903342
```

Shipped values are where the `bunker_jetson` bag starts. Verified poses for the
other recordings are catalogued in [`docs/jetson_runs.md`](../jetson_runs.md).

> **You cannot pass this as a launch argument.** `lidar_localization.launch.py`
> forwards a fixed parameter set (frames, sim time, IMU, deskew) on top of the
> YAML — `initial_pose_x:=...` on the command line is accepted and silently
> ignored. Edit the YAML, or derive a per-run copy of it (that is what
> `scripts/jetson_bag_test_native.sh` does, substituting the seed with `sed`).

### B. At runtime, exactly — scripted

Publish to `initialpose` (the node subscribes; `header.frame_id` **must** be
`map` or it is rejected). Use this when the start pose changes per run and you
want it exact rather than clicked:

```bash
ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
'{header: {frame_id: "map"},
  pose: {pose: {position: {x: 7.82, y: -4.11, z: -1.17},
                orientation: {z: 0.8550024006656964, w: 0.5186240399903342}}}}'
```

Works after startup and re-seeds a running node — no restart.

### C. Interactively — RViz

When you don't know the pose and need to eyeball it against the map:
`rviz2`, set **Fixed Frame** to `map`, add a PointCloud2 on `/hesai/points`, then
**2D Pose Estimate** — click the robot's spot and drag along its heading.

If the Fixed Frame isn't `map`, the node logs `initial pose ignored: frame_id ...
does not match global_frame_id` and keeps its old pose, which looks exactly like
the click doing nothing.

### What actually happens when you specify one

The seed is a **starting guess for an optimiser, not a declaration of where you
are**. In order:

1. The pose is stored as the current pose.
2. On the next scan it becomes the registration's initial guess.
3. For the first **5 scans** an NDT initializer aligns the live scan to the map
   starting from that guess, and each converged result *replaces* the seed
   (`NDT init scan 1/5 fitness=...` in the log).
4. It then logs `NDT init complete, switching to SMALL_VGICP` and hands over.

So the node moves off your number to whatever actually matches the map geometry.
That is why a seed 1 m out is corrected before the first pose is ever published.

The flip side: it can only *refine*, never relocate. There is no global search in
this path — if the seed is far enough out that the scan doesn't overlap the right
part of the map, NDT happily converges on whatever nearby geometry fits and
reports good fitness for the wrong place.

### Can I just say the robot is at 0, 0?

Two different questions hide in that one:

**"The robot is parked at map coordinates (0, 0)."** Fine, if true. `(0,0)` is a
real location in `gt_map_us050.pcd` — 181 map points within 2 m of it. Seed it
and it works like any other pose.

**"Treat wherever the robot is now as the origin."** No — and this is the thing
to understand about map-based localisation. The `map` frame is defined by the PCD
file. You cannot re-origin it by asserting a pose; the seed claims *"I am here in
the existing map"*, and NDT immediately checks that claim against real geometry.

Concretely: the bunker starts **8.83 m** from map origin. Seed it `(0, 0)` and
you are 8.83 m out — far outside the ~1 m that reliably converges. It will lock
onto whatever near the origin resembles the scan and report a confident, wrong
pose.

This also matters for [two robots](#two-robots): they share a frame *only*
because they share the map's origin. Re-originating one would silently put them
in different frames while both still called it `map`.

### How close is close enough?

Measured on the bunker bag: **1 m of position error or 20° of heading** converges
exactly, during init, before the first pose is published. Beyond that it can lock
onto similar-looking geometry and stay there confidently. Get within a metre;
don't agonise. (Measured in one region of one map — indicative, not a guarantee
everywhere.)

<a name="step-4"></a>
## Step 4 — Give it an `odom -> base_link`

The config sets `enable_map_odom_tf: true`, so this node publishes **only**
`map -> odom`. Something else must own `odom -> base_link` or TF never connects
and `tf2_echo map base_link` just hangs with no error.

**Normal operation — EKF** (smooths between scans):
```bash
ros2 launch /ws/launch/ekf_odom.launch.py use_sim_time:=false
```
Uses `config/ekf_odom_bunker.yaml`.

**Debug — identity static** (`map -> odom` then equals `map -> base_link`):
```bash
ros2 run tf2_ros static_transform_publisher \
  --x 0 --y 0 --z 0 --roll 0 --pitch 0 --yaw 0 \
  --frame-id odom --child-frame-id base_link
```

## Step 5 — Run and verify

Launch with the Quickstart command, wait for `Activating end`, then:

```bash
ros2 run tf2_ros tf2_echo map base_link   # tracks smoothly, no jumps
ros2 topic hz /pcl_pose                   # ~9.8 Hz
ros2 topic echo /alignment_status         # fitness flat or falling
```

Rising fitness means the match is degrading — wrong start pose, or the robot has
driven outside the prior map.

> `/pcl_pose` is per-scan only because `enable_timer_publishing: false`. If you
> ever set it `true`, the pose republishes at `pose_publish_frequency` (30 Hz)
> with restamped copies and this rate check stops meaning anything.

---

## What's already tuned

Measured on the `bunker_jetson` bag, 120 s — **natively, pinned to one core**
(`PIN_CORES=0`), not in the container the Quickstart uses. Unpinned or sharing
cores, the CPU figure will differ:

| CPU | Rate | Memory | Accuracy |
|---|---|---|---|
| 0.80 cores | 9.77 of 10 Hz | 161 MB, flat | 2.3 cm median, 8.3 cm p95 |

| Param | Value | Why |
|---|---|---|
| `scan_channel_stride` | 4 | The last safe value — **8 is the knee**, 32 loses lock. Going to 8 saves 0.1 core and roughly doubles error (2.3 → 4.4 cm). |
| `scan_channel_count` | 128 | **Mandatory on Hesai** — the cloud is unorganised, so stride can't tell which beam a point came from without it. |
| `ndt_num_threads` | 1 | Matches the single pinned core. Not clamped to visible cores — set it to what you actually give the node. |
| `fitness_score_max_points` | 1000 | Exact scoring was 40–48% of node CPU for a number nothing gates. |
| `scan_max_range` | 50 | **Halved from the old NDT config's 100 m.** Beyond ~50 m returns are too sparse in the map to match. Raise it for spaces larger than the test site. |
| `local_map_radius` | 50 | Matches `scan_max_range`; a wider crop only rebuilds more kd-tree per refresh. |

## Two robots

Both robots load the same `gt_map_us050.pcd`, so they share one `map` frame with
no robot-to-robot estimation. Use `launch/coop_multi_robot_localization.launch.py`
— it namespaces both stacks into one graph.

Bunker and Curt already have disjoint frame names (`odom`/`base_link` vs
`odom_curt`/`base_link_curt`), which is what keeps them off each other's `/tf`.
Keep it that way if you add a third robot. Note that under a namespace the pose
topics move too — `/bunker/initialpose`, not `/initialpose`, so point RViz's tool
at the right one.

## Troubleshooting

| Symptom | Cause |
|---|---|
| `Failed to load pcd file` | `map_path` is the container path; running natively needs a real one ([Step 2](#step-2)) |
| `tf2_echo map base_link` hangs, no error | Nothing publishing `odom -> base_link` ([Step 4](#step-4)) |
| RViz pose click does nothing | RViz Fixed Frame isn't `map`; node logs `initial pose ignored` ([Step 3](#step-3)) |
| Confident pose, wrong place | Seed too far off; locked onto similar geometry |
| Rising fitness | Robot outside the prior map's coverage |
| Pose rate below scan rate | CPU-bound. Cloud subscription is best-effort depth 1 (a code default, not in the yaml), so overrun drops scans silently instead of lagging |
| `rmw handle is invalid` at startup | CycloneDDS needs `sudo sysctl -w net.core.rmem_max=10485760` — **not persistent across reboot** |

## The other bunker config

`config/gt_ouster_ndt_tree_bunker.yaml` is the older **NDT_OMP** coop-bag config.
Worth reading for its sensor wiring and extrinsics provenance, but it was never
the measured one and is desktop-tuned (`ndt_num_threads: 16`, `local_map_radius:
80`). Don't run it on the robot.

## See also

- [`docs/jetson_runs.md`](../jetson_runs.md) — verified start poses per recording
- [`docs/cyclonedds_transport_study.md`](../cyclonedds_transport_study.md) — transport, CPU, memory
- [`docs/glim_comparison.md`](../glim_comparison.md) — hmr_localisation vs GLIM
