# Single-robot LiDAR localization test (against gt_map.ply)

Goal: localize the Ouster bag against the GLIM ground-truth map using
[`lidar_localization_ros2`](https://github.com/rsasaki0109/lidar_localization_ros2),
as the first step toward multi-robot localization in a shared `map` frame.

## Result: PASS (localization works)
- Map `gt_map/gt_map.ply` (3,059,991 pts, binary LE float32 x/y/z/intensity)
  loads directly — no PLY→PCD conversion needed.
- 90 s smoke test: clean tracking, **NDT fitness ~0.005** (excellent).
- Full run (hardened config): tracks ~60% of the route incl. a sharp turn;
  loses lock once at a mid-route maneuver (see Caveat).
- Trajectory is in the `map` frame and follows the mapped paths; height stable.
  See `output/eval_final_zoom.png`.

## Key facts discovered
- Host is ROS 2 **Humble**; bag is **Jazzy/mcap** → everything runs in a Jazzy
  container (`compose.yaml` + `docker/Dockerfile`).
- **DDS gotcha:** `ROS_LOCALHOST_ONLY=1` breaks FastDDS participant discovery in
  this container (no topics ever discovered). Fixed by using
  `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` instead (see `compose.yaml`).
- Bag frames: cloud `os_lidar` (OS-128, 1024 wide, best_effort QoS), IMU `imu`
  (~500 Hz), GPS `gps`. `/tf_static` is **camera-only** — there is **no
  `base_link → os_lidar` extrinsic**. So we localize the `os_lidar` frame
  directly as the base; identity is the correct initial seed because the GLIM
  map is built in the lidar frame.

## How to run
```bash
docker compose up -d
# full run (~500 s) or pass a duration in seconds for a slice:
docker compose exec ros bash /ws/scripts/run_localization.sh 180
# then, on the host, plot:
python3 scripts/plot_zoom.py gt_map/gt_map.ply output/path.csv output/eval.png
```

## Config (`config/gt_ouster_ndt.yaml`)
NDT_OMP, `ndt_resolution: 2.0`, `ndt_max_iterations: 50`, local-map crop r=80 m,
`score_threshold: 5.0`. Lowering `ndt_resolution` to 1.0 gives higher accuracy
(fitness ~0.005) but a narrower convergence basin (less robust at turns).

## Caveat + next step
- The robot makes a maneuver mid-route where constant-velocity NDT prediction
  fails (the map is NOT sparse there — ~800k pts within 25 m — so it is a motion
  / geometry issue, not coverage).
- **IMU preintegration made it worse** (diverged earlier): the bag has no
  lidar↔IMU extrinsic, so accel gravity-compensation + lever-arm are wrong in the
  `os_lidar` frame. To use IMU for full-route robustness, supply the real
  `os_lidar → imu` extrinsic (from the robot URDF or the GLIM config) and set
  `imu_preintegration_use_base_frame_transform: true`.

## Multi-robot plan (the actual goal)
Each robot runs its own `lidar_localization_ros2` instance in its own namespace
(`/robotN/...`) against this same `gt_map.ply`, all publishing into the shared
`map` frame. Because every robot localizes into the same global frame, relative
poses fall out directly. Per-robot namespaced launch + initial-pose seeding is
the next deliverable.
