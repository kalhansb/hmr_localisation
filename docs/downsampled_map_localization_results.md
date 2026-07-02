# Downsampled point-cloud results

Consolidated results for the two "downsampled point cloud" experiments run in this
workspace. Both started as raw run logs; this document is the written-up summary.

- **Part 1 — Localization-map downsample sweep** (this repo): how heavily can we
  downsample the GLIM ground-truth map (`gt_map`) before NDT localization degrades?
  → **`gt_map_us050.pcd` (0.5 m, 196 k pts) is the chosen default.**
- **Part 2 — SCovox per-scan downsample sweep** (`glim_localisation` repo): how does
  per-scan voxel size affect SCovox map quality (vertical over-fill) vs a GLIM
  reference?

Raw data sources:
- `output/dsbench/` — per-run logs (`sweep.log`, `extra.log`, `long.log`, …) and
  per-run trajectories (`runs/<name>/pose.csv`).
- `output/dsbench/cfg/*.yaml` — the localizer config used for each run.
- `../glim_localisation/output/ds_sweep_results.txt` — SCovox vs GLIM sweep.

---

## Part 1 — Localization-map downsample sweep

### Setup

- **Localizer:** `lidar_localization_ros2`, `NDT_OMP`, `ndt_resolution = 2.0`,
  `ndt_max_iterations = 50`, local-map crop radius `80 m`, scan voxel
  `voxel_leaf_size = 0.2 m`, `score_threshold = 5.0` with rejection enabled.
- **Prediction:** NDT-only, constant-velocity (`predict_pose_from_previous_delta`);
  IMU/odom off (the bag has no lidar↔imu extrinsic — see `cfg/full.yaml` note).
- **Mode:** A — localizer publishes `map → base_link` directly, base frame `os_lidar`.
- **Data:** map-test Ouster bag. Maps are uniform-voxel downsamples of `gt_map.ply`
  (`scripts/downsample_map_pcl … <voxel> uniform`). Suffix = voxel size with the
  decimal dropped (`us050` = 0.5 m, `us005` = 0.05 m, `us0001` = 0.001 m ≈ raw).
- **Main sweep:** play rate 0.5, 120 s bag window (≈240 s wall).
- Config: `cfg/us050.yaml` etc. (all identical except `map_path`).

### Results — main sweep (rate 0.5, 120 s window)

`rel-APE` = position deviation of each downsampled-map run vs the full-map run,
timestamp-interpolated in the shared `map` frame over the overlapping window
(metres). 100 % acceptance (0 rejections) on every run below.

| Map (voxel)      | Points     | File size | Poses | rel-APE median | mean  | p90   | max   |
|------------------|-----------:|----------:|------:|---------------:|------:|------:|------:|
| `gt_map` (full)  | 3,059,991  | 36.7 MB   | 541   | — (reference)  | —     | —     | —     |
| `us0001` (0.001) | 3,058,868  | 48.9 MB   | 544   | 0.003 m        | 0.009 | 0.029 | 0.089 |
| `us005`  (0.05)  | 2,837,469  | 45.4 MB   | 562   | 0.004 m        | 0.010 | 0.029 | 0.087 |
| `us010`  (0.10)  | 2,078,386  | 33.3 MB   | 630   | 0.003 m        | 0.008 | 0.025 | 0.077 |
| `us020`  (0.20)  | 943,289    | 15.1 MB   | 804   | 0.003 m        | 0.006 | 0.009 | 0.079 |
| `us030`  (0.30)  | 493,858    | 7.9 MB    | 936   | 0.015 m        | 0.016 | 0.022 | 0.036 |
| `us040`  (0.40)  | 296,979    | 4.75 MB   | 936   | 0.023 m        | 0.024 | 0.033 | 0.055 |
| **`us050` (0.50)** | **196,404** | **3.14 MB** | **936** | **0.031 m** | **0.032** | **0.042** | **0.070** |

> Note the `us0001`/`us005` files are *larger* than `gt_map.pcd` even at similar
> point counts: the `us*` maps carry an `intensity` field (16 B/pt) while the
> source `gt_map.pcd` is xyz-only (12 B/pt).

### Throughput observation (the real win)

`Poses` rises from **541** (full map) to **936** (≥0.3 m) for the *same* 120 s of
data. The 3 M-point map cannot keep up with real-time playback — NDT's target is
too large, so scans are dropped — whereas the downsampled maps process the entire
stream at 100 % acceptance. Confirmed on the full-bag run (`long.log`, rate 1.0,
1000 s):

| Map              | Accepted | Rejected | Reject % | Poses |
|------------------|---------:|---------:|---------:|------:|
| `gt_map` (full)  | 729      | 225      | 23.6 %   | 730   |
| `us050` (0.5 m)  | 2,335    | 404      | 14.7 %   | 2,336 |

→ `us050` gives **3.2× the pose throughput** *and* a lower reject fraction.

### Conclusion

**`gt_map_us050.pcd` (0.5 m uniform voxel, 196,404 pts) is the default localization
map.** Versus the full map it is **15.6× fewer points** and **11.7× smaller on disk**
(36.7 → 3.14 MB), keeps **100 % pose acceptance** in the rate-0.5 sweep and the
**best real-time throughput**, and tracks the full-map trajectory to **~3 cm median**
(p90 4.2 cm, max 7 cm).

Diminishing returns are clear: below 0.3 m the accuracy gain is sub-centimetre while
footprint balloons; 0.3–0.5 m trades a few cm of deviation for a 6–15× lighter map.
0.5 m is the sweet spot; voxels coarser than 0.5 m were not swept here.

> Empirical check on `us050`: nearest-neighbour spacing median 0.575 m
> (mean 0.63 m), consistent with a 0.5 m voxel grid. Scene extent ≈ 230 × 207 × 52 m.

### Related tuning runs (same `dsbench/` folder)

The `us050_*` / `full_*` configs probe localizer settings *on top of* the chosen map,
not the downsample level itself:
- `*_norej` — `reject_above_score_threshold: false`
- `*_t16` — `ndt_num_threads: 16`
- `*_fit10/20` — tighter `score_threshold`
- `*_r05`, `*_long`, `*_nopred`, `*_recover` — rate / window / prediction variants

See the matching `cfg/*.yaml` and `*.log` for each.

---

## Part 2 — SCovox per-scan downsample sweep

Different experiment, different lever: this downsamples **each incoming scan** before
SCovox integration (`downsample_voxel_size`), to suppress vertical "over-fill" — tall
spurious floating columns in the SCovox map that have no real surface. The reference
is a GLIM map of the same bag. Source: `../glim_localisation/output/ds_sweep_results.txt`
(rate 0.5, 120 s, `max_range 20`).

| `downsample_voxel_size` | SCovox pts | SCovox/GLIM column-height ratio (median) | Floating "SCovox-only" columns |
|------------------------:|-----------:|----------------------------------------:|-------------------------------:|
| 0.05 | 3,314,728 | 17.0× | 79,809 (48.8 %) |
| 0.10 | 3,176,154 | 16.0× | 78,887 (48.2 %) |
| 0.20 | 2,483,271 | 11.0× | 74,019 (45.7 %) |
| **0.50** | **1,051,991** | **2.0×** | **62,320 (41.2 %)** |
| 1.00 | 395,527 | 2.0× | 62,903 (50.1 %) |

**Takeaway:** per-scan downsampling is the lever that collapses the vertical smear —
the column-height ratio vs GLIM falls from 17× (0.05 m) to 2× at **0.5 m**, the sweet
spot (1.0 m gives no further height gain but starts losing real structure). This is
the SCovox vertical-over-fill finding; densifying noisy surfaces fills the z-tails.
