# Jetson runs

How to reproduce the localizer benchmark on a Jetson AGX Orin (or any new host) from a
clean checkout: which bags, which start poses, how to run the throughput test, and how
to score the result against a reference trajectory.

The two trimmed bags are the whole point of this doc. The source recordings are 140 GB
and full of camera topics the localizer never subscribes to; the trimmed pair is 3 GB
and carries exactly what the node needs, so a board can be qualified without moving the
originals onto it.

---

## 1. Prerequisites

```bash
vcs import src < hmr_localisation.repos     # pinned fork, see §6
docker compose build                        # ~30 min on arm64; osrf/ros:jazzy-desktop is multi-arch
docker compose exec ros bash -lc 'cd /ws &&
  rosdep install --from-paths src --ignore-src -r -y &&
  colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release'
```

On the **host**, before `docker compose up`:

```bash
sudo nvpmodel -m 0 && sudo jetson_clocks     # max power mode + clocks locked
```

Without both of these the board runs at a lower power profile and the numbers below are
not comparable. `compose.yaml` sets no CPU limit, so the container sees all 12 cores.

`ipc: host` is required for the shared-memory cloud transport, but killed ROS processes
leave orphaned Fast-DDS segments in the host's `/dev/shm` (half of RAM on a Jetson).
Always `docker compose stop` — **never `pkill`** — and reclaim with `fastdds shm clean`
inside the container if `df /dev/shm` fills up.

---

## 2. The bags

### Source recordings (not in git)

Both were recorded 2026-07-31 in the same environment as `gt_map`, on the two robots,
~27 min apart. They live beside the repo at `../../../bags/`, mounted at `/ws/bags`.

| | CURTMINI | bunker |
|---|---|---|
| directory | `2026_07_31_11_24_12__kalhan_2_CURTMINI` | `2026_07_31_10_57_06__kalhan_2_` |
| size / files | 85.6 GB, 16 mcap | 54.4 GB, 11 mcap |
| duration | 416.2 s | 427.4 s |
| start (epoch) | 1785497053.524738 | 1785495426.848236 |
| lidar | `/ouster/points`, 3983 msgs, Ouster | `/hesai/points`, 4274 msgs, Hesai |
| imu | `/curt/imu/data`, 165853 msgs | `/imu/data`, 170874 msgs |
| `/tf_static` | 4 msgs | 1 msg |
| base frame | `base_link_curt` | `base_link` |
| also carries | RealSense + MapIR, `/plan`, scovox maps | OAK stereo, `/plan`, scovox maps |

The bunker recording started 1626.676502 s before the CURTMINI one. That offset is what
lets the two be replayed on one timeline.

> **The bunker bag's `odom -> base_link` edge has two interleaved publishers.** Read
> naively it gives 25,642 samples and 177 km of "path". Split by z: z==0 is 50 Hz wheel
> odometry (21,367 samples, 73.3 m); z!=0 is 10 Hz lidar odometry (4,275 samples, one
> per scan, 95.1 m). Use the latter as the odometry reference. The throughput test below
> sidesteps this entirely by publishing a static identity `odom -> base_link`.

### Trimmed Jetson bags (produced by `scripts/make_jetson_bag.py`)

```bash
python3 /ws/src/hmr_localisation/scripts/make_jetson_bag.py \
  /ws/bags/2026_07_31_11_24_12__kalhan_2_CURTMINI /ws/bags/curtmini_jetson \
  /ouster/points /curt/imu/data --keep-fields x,y,z,intensity,t --duration 120

python3 /ws/src/hmr_localisation/scripts/make_jetson_bag.py \
  /ws/bags/2026_07_31_10_57_06__kalhan_2_ /ws/bags/bunker_jetson \
  /hesai/points /imu/data --keep-fields x,y,z,intensity,timestamp --duration 120
```

Each keeps only the lidar, the IMU and `/tf_static`, repacks every `PointCloud2` down to
the listed fields, and writes mcap with `zstd_small` chunk compression. Verified output:

| | `curtmini_jetson` | `bunker_jetson` |
|---|---|---|
| size | 1.9 GiB | 1.1 GiB |
| duration | 119.991 s | 119.999 s |
| clouds | 1195 (9.96 Hz) | 1200 (10.00 Hz) |
| imu | 53246 (444 Hz) | 47987 (400 Hz) |
| `/tf_static` | 4 | 1 |
| cloud frame | `os_lidar`, 1024×128 | `hesai_lidar`, 230400×1 |
| fields | `x,y,z,intensity,t`, `point_step` 20 | `x,y,z,intensity,timestamp`, `point_step` 24 |

Fields are tightly packed at offsets 0/4/8/12/16 with no padding, so `point_step` is the
minimum for each sensor. `ros2 bag info` reports these counts; note that rosbag2's own
`compression_format` reads empty because compression is at the mcap chunk level, and
that the `files[0].message_count` in `metadata.yaml` is written as exactly 2× the true
count (a rosbag2 writer quirk — the top-level and per-topic counts are correct, and
nothing in the run path reads the per-file one).

The 120 s window starts at each bag's first message, so **the cuts do not contain the
CURTMINI end-of-run divergence** described in §5.

---

## 3. Start poses

Every recording starts from its own spot. The seed in
`config/gt_ouster_ndt_tree_realtime.yaml` is the *map-test-2* robot's, so replaying
either 2026-07-31 bag with the shipped config starts the localizer in the wrong place and
it never recovers. `scripts/jetson_bag_test.sh` carries the correct seeds and derives a
per-bag copy of the config at runtime.

| bag | x | y | z | yaw | qz | qw | base frame |
|---|---|---|---|---|---|---|---|
| CURTMINI | 8.33 | −3.78 | −1.39 | 117.09° | 0.853050270749362 | 0.5218287416139898 | `base_link_curt` |
| bunker | 7.82 | −4.11 | −1.17 | 117.52° | 0.855002400665696 | 0.5186240399903342 | `base_link` |

These are **not hand-placed**. They were recovered with the fork's own `BBS_2D` global
localization plus NDT refinement against the full 3,059,991-point `gt_map.ply`:

- CURTMINI: PCL fitness 0.0236 m² (RMS nearest-neighbour 0.15 m); 7 of 24 independent
  BBS basins converged to within 1 cm of each other.
- bunker: fitness 0.0371 m² (RMS 0.19 m); 6 of 24 basins converged identically, best BBS
  hit ratio 0.963.

The two robots start ~0.6 m apart at the same heading — independent cross-validation
across two sensors and two recordings.

---

## 4. Running the throughput test

```bash
docker compose up -d
docker compose exec ros bash /ws/scripts/jetson_bag_test.sh curtmini
docker compose exec ros bash /ws/scripts/jetson_bag_test.sh bunker
```

Optional second and third arguments are a run name and a playback duration in seconds.
The script runs the localizer alone — no EKF, `odom -> base_link` is a static identity —
so the numbers are the node's own. It prints the cores it can actually see
(`nproc`, `cpuset.cpus.effective`, `cpu.max`) before anything else; a cgroup limit shows
up there first, and the node clamps its OpenMP pool to what it sees.

Output lands in `/ws/output/jetson_test/<run_name>.{status.csv,poses.csv,log}`, with a
rate / alignment-time / gap table on stdout from `scripts/analysis/throughput_summary.py`.

**Reading it:** `gap_med` 0.100 s means the node keeps up with the 10 Hz sensor;
0.200 s means it is processing every other scan. For reference, an 8-core x86 host
headless reaches 6.6 Hz on CURTMINI with alignment median 48 ms / p95 117 ms and fitness
median 0.0644 m².

> **Do not judge throughput with RViz attached.** A live viewer on the same host roughly
> halves the scan rate (6.6 Hz → 3.2–4.4 Hz) and can push the node into losing lock
> entirely, because the contention widens `accepted_gap_sec` past 1 s, the seed then
> extrapolates over 2 s, registration stops converging, and
> `reject_above_score_threshold: false` accepts the bad pose. Fitness and corrections look
> healthy right up to the break. Record headless and replay for viewing.

If the board cannot hold 10 Hz, in order of cost:

1. `local_map_refresh_distance: 5`
2. `fitness_score_max_points: 4000`
3. `voxel_leaf_size: 0.3`

`ndt_num_threads: 8` is what leaves GLIM, the drivers and the planner (in their own
containers) their cores on a 12-core Orin — do not raise it.

`fitness_score_max_points` defaults to `0`, which evaluates every point and reproduces
PCL's value exactly. A positive value estimates the mean from a uniform stride sample:
much cheaper, but the sample is systematic rather than random (VoxelGrid emits points
ordered by voxel index) and has been observed to bias the score high. Prefer the default
unless CPU-bound, and never compare a sampled score against an exact one.

---

## 5. Scoring accuracy

Throughput alone does not tell you the board localized *correctly*. To score a run,
compare its pose trace against a reference trajectory with
`scripts/analysis/traj_eval.py`:

```bash
python3 /ws/scripts/analysis/traj_eval.py REF.poses.csv /ws/output/jetson_test/RUN.poses.csv
```

It reads either CSV format this repo produces (`benchmark_pose_recorder`'s columns or a
bare `t,x,y,z,qx,qy,qz,qw`), resamples the reference onto each test stamp by linear
interpolation, and reports translation and yaw error percentiles, coverage, and **the
first time the error crosses a threshold** (`--diverge-threshold`, default 0.5 m) — the
number that separates "tracks with some noise" from "lost lock and never recovered".
Useful flags: `--t0/--t1` to window the run, `--align` to solve a yaw+translation fit
first (only needed for tracks in frames that differ by a rigid transform; two runs of the
same bag seeded from the same `gt_map` are directly comparable), `--out` for a per-sample
CSV.

### Building a reference trajectory

The references are not in git — they are ~4000-row CSVs regenerated on demand. Run the
same bag through the localizer against the **full-resolution** `gt_map.ply` with the
registration wound up, at a playback rate slow enough that nothing is dropped:

- `map_path` → `gt_map/gt_map.ply` (not the 0.5 m `.pcd`)
- NDT resolution 0.6, 120 iterations, `transform_epsilon` 0.001
- `scan_max_range` 50, `voxel_leaf_size` 0.2, `local_map_radius` 60
- playback `--rate 0.206`

That is ~34 min per bag and yields ~100% scan coverage. Reference uncertainty measured
this way is **1.7–1.8 cm median, 5.1 cm p95**, established by running two independent
backends (`NDT_OMP` and `SMALL_GICP`) over the same map and differencing them.

### What to expect

The realtime config on the 0.5 m map tracks that reference to **~4 cm median, ~10 cm p95**.

> **The full CURTMINI recording diverges in its last ~19 s, in both backends.** At
> t≈397 s of 416 s, during fast yaw motion where the coarse map already fits poorly
> (fitness 0.13–0.17 vs 0.065 nominal), registration loses lock and fitness jumps to
> 5–13 m². Final error 19.7 m (`NDT_OMP`) / 11.4 m (`SMALL_VGICP`). The full-resolution
> reference config handles the same moment fine, so the cause is the 0.5 m map plus
> coarse registration, not the backend. The bunker recording never diverges.
>
> The 120 s Jetson cuts stop long before this, so a clean `jetson_bag_test.sh` result is
> **not** evidence that the config is safe through fast rotations on the downsampled map.

---

## 6. What the results depend on

Every measured number in this doc was produced with the fork pinned in
`hmr_localisation.repos` at `4e8499a`. That pin is load-bearing: it contains the
multi-threaded fitness score. Without it, `getFitnessScore()` runs PCL's single-threaded
path — 114 ms of a 192 ms callback, more than the alignment itself — and no amount of
tuning will hold 10 Hz. `small_gicp` is not vcs-imported; it is built into the image at
tag `v1.0.1` (`docker/Dockerfile`).

The localizer subscribes to clouds **best-effort** (`SensorDataQoS`), matching both the
driver's default output and the bags' recorded QoS, so no QoS reconfiguration is needed
on the robot. Multi-MB clouds are routed over shared memory (`config/fastdds_shm.xml`);
over UDP loopback they throttle to ~0.1 Hz. The run scripts set this automatically.
