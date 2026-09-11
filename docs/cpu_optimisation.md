# Making the localizer cheaper

What the node costs per scan, which knobs reduce it, what each one does to accuracy, and
which apparently obvious levers do nothing. Everything measured here was produced with
`scripts/jetson_bag_test.sh` (`docs/jetson_runs.md` §4) on the 8-core x86 host, both
120 s bags, `ndt_num_threads: 8`, `SMALL_VGICP`, fork pinned at `2ed0255`. The Orin has
not been measured yet; the ratios should carry because every stage is per-point, the
absolute numbers will not.

Accuracy in this doc is **relative**: each variant is scored with `traj_eval.py` against
its own bag's baseline run, because no absolute reference trajectory (`jetson_runs.md`
§5) existed at the time. The floor of that comparison is the run-to-run noise of an
identical config, **2 mm median / 2 cm p95 2D ATE** on both bags, so anything above
that is a real shift. None of the runs below diverged (no sample above 0.5 m).

---

## 1. What it costs today

The runner samples the node's CPU time and resident size from `/proc` every 2 s and
prints one line after the throughput table:

```
node CPU: 2.62 cores avg over 132 s (of 8 visible); peak RSS 468 MB
```

Shipped config (`scan_channel_stride: 2`), 120 s bags, 10 Hz sensor:

| bag | scans kept | rate | align med / p95 | gap p95 | node CPU | peak RSS |
|---|---|---|---|---|---|---|
| CURTMINI (Ouster, 1024×128) | 1063 / 1195 | 9.1 Hz | 25.9 / 66.3 ms | 0.200 s | 2.62 cores | 468 MB |
| bunker (Hesai, 230400×1) | 1162 / 1200 | 10.0 Hz | 22.2 / 36.7 ms | 0.100 s | 2.44 cores | 483 MB |

`gap p95` 0.100 s means the node keeps up with every scan; 0.200 s means the slow scans
still cost it every other one. Alignment time excludes preprocessing and the fitness
score. Memory is dominated by the map, the cropped target with its covariances and the
NDT bootstrap grid; no variant below moves it by more than ~30 MB.

---

## 2. The ladder

Measured on both bags, each as an A/B against the shipped config with the same
`traj_eval.py` scoring. "Free" means the track moved less than 5 mm median against the
baseline run, i.e. inside the run-to-run noise once the different set of accepted scans
is allowed for. Everything in §8 is config-only.

| rung | change | CPU (CURTMINI / bunker) | memory | accuracy cost | status |
|---|---|---|---|---|---|
| free | `fitness_score_max_points: 1000` | −41% / −48% | — | none (§8) | measured, not yet shipped |
| free | `segment_size` 256 → 64 MB in `config/fastdds_shm.xml` | — | −212 MB RSS | none (§8) | measured, not yet shipped |
| free | `local_map_radius: 50` | 0 / −6% | −10 MB | none (§8) | measured, not yet shipped |
| shipped | `scan_channel_stride: 2` | −29% / −32% | −10 MB | 2–3 cm median vs full resolution | §3 |
| 1 | `scan_channel_stride: 4` | a further −19% / −25% | −15 MB | 4.1 / 3.2 cm median vs full resolution | §3 |
| 2 | `voxel_leaf_size: 0.3` | ~20 ms alignment | — | measurably worse yaw on turns | measured earlier (README) |

The three free rows together take the node from 3.0 to 1.8 cores and 467 to 250 MB on
CURTMINI, and from 2.5 to 1.3 cores and 481 to 263 MB on bunker, with 4–5 mm median
shift. Not on the ladder, and why, in §4, §5 and §8.

---

## 3. Channel stride (measured 2026-09-11)

`scan_channel_stride: N` keeps every N-th LiDAR channel (beam) and drops the rest before
any other preprocessing. Every per-scan stage is linear in the points that survive: the
raw conversion and range filter, the 0.2 m voxel grid, the alignment and the fitness
score. Azimuth resolution, which is what constrains yaw, is untouched. The 0.2 m voxel
already merges adjacent channels inside ~15 m, so halving the channels removes mostly
far-range redundancy: 63% of the post-voxel source points survive, not 50%.

Neither sensor writes a `ring` field, so the node derives the channel from the cloud
layout. The Ouster cloud is organised 1024×128 with **row = channel** (elevation spreads
0.012° along a row and 12.5° down a column), so `channel = index / width`. The Hesai
cloud is unorganised 230400×1 with the channels interleaved point by point
(**channel = index % 128**; the elevation pattern repeats exactly with period 128), so it
needs `scan_channel_count: 128`. The `bunker` runner profile sets that; with 0 on an
unorganised cloud the node keeps every point and warns once.

| bag | stride | source pts | scans kept | rate | align med / p95 | gap p95 | fitness | node CPU | peak RSS | 2D ATE vs stride 1 (med / p95) | yaw med |
|---|---|---|---|---|---|---|---|---|---|---|---|
| CURTMINI | 1 | 37,281 | 945 / 1195 | 7.9 Hz | 40.7 / 86.4 ms | 0.200 s | 0.065 | **3.69** | 479 MB | — | — |
| CURTMINI | **2** | 23,598 | 1063 | 9.1 Hz | 25.9 / 66.3 ms | 0.200 s | 0.066 | **2.62** | 468 MB | 2.7 / 5.9 cm | 0.03° |
| CURTMINI | 4 | 13,531 | 1091 | 9.3 Hz | 20.6 / 52.5 ms | 0.200 s | 0.067 | **2.13** | 451 MB | 4.1 / 7.5 cm | 0.06° |
| bunker | 1 | 34,211 | 995 / 1200 | 8.3 Hz | 40.4 / 63.7 ms | 0.200 s | 0.085 | **3.57** | 495 MB | — | — |
| bunker | **2** | 19,781 | 1162 | 10.0 Hz | 22.2 / 36.7 ms | 0.100 s | 0.081 | **2.44** | 483 MB | 2.2 / 4.8 cm | 0.05° |
| bunker | 4 | 10,911 | 1194 | 10.0 Hz | 14.6 / 32.7 ms | 0.100 s | 0.081 | **1.82** | 466 MB | 3.2 / 8.2 cm | 0.11° |

Source points are the mean per-scan count reaching the registration after range filter
and voxel grid. Stride 2 is shipped: it saves about a third of the node's CPU, keeps
*more* scans (the bunker bag stops dropping any), and shifts the track by 2–3 cm median
against full resolution — below the ~4 cm the realtime config already sits from a
reference (`jetson_runs.md` §5). Revert with `scan_channel_stride: 1`.

What this does not cover: the 120 s cuts stop before the CURTMINI end-of-run divergence,
so whether fewer source points make that moment better or worse is unmeasured. A fair
check needs the reference trajectories rebuilt (`jetson_runs.md` §5, ~34 min per bag).

Aside: halving *azimuth* is free at the Ouster itself (`512x10` lidar mode) and also
halves the driver's CPU and the 2 MB DDS clouds; channel halving can only happen in the
node.

---

## 4. A sparser map does not help (measured 2026-09-11)

The intuitive lever, and the one the older NDT sweep in
[`downsampled_map_localization_results.md`](downsampled_map_localization_results.md)
supported. With VGICP and the cached crop it is the wrong one. The committed map
variants, run with `MAP=/ws/gt_map/gt_map_usNNN.pcd`:

| map | points | 1.0 m voxels | pts / voxel | CURTMINI scans kept | align med / p95 | fitness med | 2D ATE vs `us050` (med / p95) | yaw med |
|---|---|---|---|---|---|---|---|---|
| `us050` (shipped) | 196,404 | 50,884 | 3.9 | 980 / 1195 | 36.6 / 89.7 ms | 0.065 | — | — |
| `us060` | 138,763 | 46,191 | 3.0 | 1038 | 34.3 / 73.2 ms | 0.085 | 2.2 / 4.8 cm | 0.03° |
| `us070` | 102,905 | 44,411 | 2.3 | 953 | 39.5 / 84.1 ms | 0.110 | 2.7 / 9.2 cm | 0.07° |
| `us100` | 50,884 | 50,884 | 1.0 | 881 | 46.0 / 88.5 ms | 0.205 | 5.9 / 10.8 cm | 0.20° |

bunker: `us050` 1126 scans, 34.7 / 52.0 ms; `us060` 1103, 34.6 / 53.5 ms, 5.6 cm;
`us070` 1063, 37.5 / 55.5 ms, 2.6 cm; `us100` 944, 44.1 / 70.3 ms, 4.1 cm, yaw 0.08°.

Node CPU by map, sampled on a second pass of the same sweep (stride forced to 1, so
comparable with the stride-1 rows in §3):

| map | CURTMINI node CPU | peak RSS | bunker node CPU | peak RSS |
|---|---|---|---|---|
| `us050` | 3.69 cores | 479 MB | 3.57 cores | 495 MB |
| `us060` | 3.64 cores | 462 MB | 3.41 cores | 486 MB |
| `us070` | 3.78 cores | 458 MB | 3.46 cores | 472 MB |
| `us100` | 3.75 cores | 440 MB | 3.52 cores | 468 MB |

A quarter of the map points changes CPU by under 5% in either direction (the run-to-run
spread) and saves ~30-40 MB. Compare stride 2 in §3: −29-32% CPU on the same bags.

VGICP registers against the 1.0 m voxel map, not the point cloud, and `us050` and
`us100` produce the **same 50,884 voxels**. Per-scan cost is source points × (hash lookup
+ Mahalanobis) and never touches the map's point count. A sparser map only lowers the
evidence per voxel (3.9 → 1.0 points, so each covariance comes from one point's k=20
neighbourhood spanning ~2.5 m instead of ~1.3 m), so the covariances degrade, alignment
takes *longer*, fewer scans are kept, CPU does not move, and yaw noise grows sevenfold
by `us100`. The
fitness floor rises with the point spacing (0.065 → 0.205 m²), so scores are not
comparable across maps. What a sparser map does buy is the crop-refresh cost (kNN
covariances over the cropped target every 20 m) and ~28 MB; §5 item 2 gets both without
touching accuracy. Combining a sparser map with a channel stride is therefore just the
stride plus accuracy risk. Keep `gt_map_us050.pcd`.

---

## 5. Remaining unmeasured levers

Found by reading the hot path and verified against the code. Items 2, 4, 6 and 7 below
have since been measured (§8): 7 and the radius-50 form of 2 are free and large, 4 costs
accuracy, 6 does nothing. Items 1, 3, 5 and 8 are still estimates.

1. **`ndt_num_threads` must equal the cores the container actually gets.** The node does
   not clamp its OpenMP pool (`resolveRegistrationThreadCount()` returns a positive value
   as-is), so `8` on a one-core container requests 8 threads for the kd-tree build, the
   covariances, every solver iteration and the fitness score. Under a cpuset that costs
   a few ms per scan in barriers; under a CFS quota (`--cpus=1`) the idle workers
   spin-wait against the same quota and it can be tens of ms. The runner prints which
   of the two you have. Do not use `0` for auto: it resolves to 1 under a cpuset and to
   every core on the board under a bare quota.
2. **Make the crop crop, or delete it.** The crop is 2D with radius
   `local_map_radius + local_map_refresh_distance` = 100 m, and on this corridor-shaped
   site that keeps **97.3%** of `us050` from either bag's start (40 m: 48%; 50 m: 59%;
   60 m: 69%; 80 m: 89%). So every 20 m of travel the node rebuilds the kd-tree and
   k=20 covariances over 191k points to exclude 3% of the map. Either `local_map_radius:
   50` (with `scan_max_range: 50` nothing beyond 72 m of the crop centre can be matched
   anyway; keeps ~59% of the target, a real cut in every refresh) or build the target
   once at configure and remove the refresh from the hot loop. Shipping radius 80 is the
   worst of both. Radius 50 needs an ATE check; it also makes the existing
   `local_map_refresh_distance: 5` rung redundant, which quadruples refresh frequency to
   trim the crop area by only 28%.
3. **Fuse the preprocessing passes.** Conversion, range filter, voxel grid and the timed
   copy are separate passes over the raw cloud; one pass would save an estimated
   8–12 ms per scan, single-threaded, so it transfers one-to-one to a core.
4. **`gicp_corr_randomness: 20 → 10`.** Halves the covariance cost on every crop refresh
   (paid over the whole cropped target). Mild accuracy risk; needs the ATE check.
5. **Dead `source_voxelmap_` in small_gicp v1.0.1.** The pinned library builds a source
   voxel map every scan that `align()` never reads (it exists only for
   `swapSourceAndTarget()`). An estimated 4–8 ms per scan and some memory; fixing it
   means patching the pinned dependency in `docker/Dockerfile`.
6. **`ndt_max_iterations: 50 → 30`.** Bounds the p95 tail on scans that do not converge.
   Only affects those scans.
7. **`fitness_score_max_points: 1000`.** On one core the exact score is ~30 ms of pure
   diagnostics (`reject_above_score_threshold: false`, so nothing consumes it). The ~30%
   high bias comes from the sampling order, not the sample size, so 1000 costs a quarter
   of 4000 for the same bias.
8. **Default `CMAKE_BUILD_TYPE` in the fork's `CMakeLists.txt`.** The line hardcodes
   `-O2 -g` and sets no default build type; our build command passes `Release` and the
   generated flags end `-O3 -DNDEBUG`, but anyone following the fork's own README gets a
   build with Eigen and PCL assertions live in the VGICP loop.

---

## 6. Ruled out

- **Coarser map** — §4.
- **IMU smoother window.** `imu_smoother_` (`use_imu_preintegration: true`) optimises
  two poses (24 dimensions), not `window_size` of them: every accepted scan resets it to
  the observation (`resetImuPreintegrationSmootherToObservation`), so the window only
  grows in sustained fallback. Not a cost.
- **arm64 `-mcpu`, LTO.** NEON is baseline on AArch64, so the gap `-march=native`
  closes on x86 does not exist there; not worth the portability loss.
- **Retry paths, IMU preintegration, diagnostics publishing, TF lookups** — read and
  clean.

---

## 7. How to reproduce

```bash
docker compose exec ros env SET="scan_channel_stride=4" bash /ws/scripts/jetson_bag_test.sh curtmini curt_ch4
docker compose exec ros env MAP=/ws/gt_map/gt_map_us100.pcd bash /ws/scripts/jetson_bag_test.sh bunker bunk_us100
docker compose exec ros python3 /ws/scripts/analysis/traj_eval.py \
  /ws/output/jetson_test/curt_ch1.poses.csv /ws/output/jetson_test/curt_ch4.poses.csv
```

`MAP=` swaps the map and `SET="key=value ..."` sets any scalar parameter, both on the
derived per-run copy only. Run variants back to back and let each run exit fully before
the next: a node that is still alive holds Fast-DDS shared-memory port locks (the next
run then fails to activate with `init_port ... open_and_lock_file failed`, fixed by
`fastdds shm clean`) and re-publishes its latched status to the next run's recorder
(`throughput_summary.py` now drops that row). Run a config twice before believing a
difference smaller than the noise floor above.

---

## 8. Config-only levers, measured (2026-09-11)

Every candidate from §5 that needs no code change, plus two found on the way, run as an
A/B against the shipped config (stride 2, 8 threads) on both bags, CPU and RSS sampled,
scored against the same-day baseline run. Noise floor for this table: the `passive` and
`iter30` rows, which change nothing, sit at 1.4–2.5 mm median / 1.1–1.2 cm p95.

| variant | CURTMINI cores | RSS | align med / p95 | gap p95 | 2D ATE med / p95 | yaw | bunker cores | RSS | align med / p95 | gap p95 | 2D ATE med / p95 | yaw |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| shipped (`base`) | 3.03 | 467 MB | 25.1 / 66.0 ms | 0.200 s | — | — | 2.52 | 481 MB | 19.9 / 35.3 ms | 0.100 s | — | — |
| `OMP_WAIT_POLICY=PASSIVE` | 3.15 | 467 | 26.6 / 65.3 | 0.200 | 1.4 mm / 1.1 cm | 0.002° | 2.54 | 480 | 19.9 / 34.2 | 0.100 | 2.4 mm / 1.2 cm | 0.005° |
| `ndt_num_threads: 4` | 2.36 | 465 | 25.3 / 77.4 | 0.200 | 1.4 mm / 1.2 cm | 0.002° | 2.32 | 483 | 23.4 / 33.1 | 0.100 | 2.5 mm / 1.2 cm | 0.006° |
| `local_map_radius: 50` | 3.07 | 454 | 26.6 / 67.1 | 0.200 | 1.6 mm / 1.2 cm | 0.002° | 2.37 | 474 | 17.2 / 32.7 | 0.100 | 2.5 mm / 1.2 cm | 0.006° |
| `gicp_corr_randomness: 10` | 2.84 | 465 | 21.4 / 53.6 | 0.200 | **3.0 cm / 6.5 cm** | **0.086°** | 2.06 | 487 | 13.0 / 26.4 | 0.100 | **3.0 cm / 9.9 cm** | **0.047°** |
| `ndt_max_iterations: 30` | 3.06 | 464 | 26.2 / 67.9 | 0.200 | 1.5 mm / 1.2 cm | 0.002° | 2.58 | 488 | 19.7 / 34.9 | 0.100 | 2.2 mm / 1.1 cm | 0.005° |
| `fitness_score_max_points: 1000` | **1.79** | 470 | 21.3 / 62.3 | **0.100** | 4.0 mm / 1.5 cm | 0.004° | **1.30** | 485 | 15.9 / 22.1 | 0.100 | 4.9 mm / 1.9 cm | 0.011° |
| SHM `segment_size` 64 MB | 3.10 | **255** | 26.0 / 67.5 | 0.200 | 1.3 mm / 1.3 cm | 0.002° | 2.50 | **274** | 19.6 / 33.2 | 0.100 | 2.1 mm / 1.1 cm | 0.005° |
| fitness 1000 + radius 50 + SHM 64 MB | **1.77** | **250** | 22.2 / 68.8 | **0.100** | 4.0 mm / 1.9 cm | 0.004° | **1.27** | **263** | 15.7 / 22.6 | 0.100 | 4.8 mm / 1.8 cm | 0.011° |
| the same + 4 threads | 2.04 | 243 | 32.5 / 76.3 | 0.200 | 2.3 mm / 1.5 cm | 0.003° | 1.84 | 262 | 26.4 / 42.7 | 0.100 | 3.1 mm / 1.3 cm | 0.006° |

What it says:

- **The exact fitness score is the single largest cost in the node** — 40–48% of its
  CPU, more than the alignment itself. It is a nearest-neighbour query for every source
  point against the cropped target's kd-tree, on 8 threads, for a number that nothing
  gates (`reject_above_score_threshold: false`). Sampling 1000 points gives a score
  within 1.5% of the exact one (0.0673 vs 0.0664 m², not the 30% the earlier note
  feared — that figure was for the old voxel-index ordering on full-resolution clouds),
  so the internal twist EKF's fitness-scaled measurement noise, which clamps at
  0.1 m² anyway, is untouched. The 4–5 mm median shift is the different set of
  accepted scans: the node stops dropping any on CURTMINI.
- **Half the resident memory was the Fast-DDS shared-memory segment.** `pmap` on the
  running node shows one 266 MB `fastrtps_*` mapping, the 256 MB `segment_size` in
  `config/fastdds_shm.xml`, touched end to end as the ring buffer cycles. The Hesai
  cloud is 5.5 MB and the Ouster 2.6 MB, so 64 MB holds a dozen in flight; at 64 MB
  neither bag drops a scan and RSS falls by 212 MB. The rest of the node is ~250 MB:
  ~63 MB map + covariances, ~35 MB heap, ~50 MB of libraries (PCL, VTK, rclcpp,
  Fast-DDS), the remainder thread stacks and DDS bookkeeping. Note the segment is also
  what a killed process leaves behind in `/dev/shm`.
- **`local_map_radius: 50` is free** (nothing beyond 72 m of the crop centre can match
  a 50 m scan) and worth a little on bunker; it replaces the old
  `local_map_refresh_distance: 5` rung, which no longer appears here.
- **`gicp_corr_randomness: 10` is the fastest alignment in the table and the only
  variant that costs accuracy**: 3 cm median, 0.05–0.09° yaw, more than stride 2.
  Leave it at 20.
- **Fewer threads only help while the fitness score is exact.** 4 threads cut CPU 22%
  on CURTMINI at stride 2 because the fitness kd-tree query does not scale past 4; once
  the score is sampled the alignment is what remains, it does scale, and 4 threads is
  slower in both CPU and wall time and drops scans again. Keep `ndt_num_threads` at
  the cores the container really has (§5 item 1).
- `OMP_WAIT_POLICY=PASSIVE` and `ndt_max_iterations: 30` change nothing: libgomp's
  default spin is already short, and the solver rarely reaches 30 iterations.
