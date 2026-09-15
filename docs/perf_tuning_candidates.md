# Localizer performance: candidates, and what was ruled out

A ranked backlog of changes that might buy per-scan time in the realtime tree config,
plus the leads that were costed and then killed. Nothing here has been applied — the
only thing committed from this round was a documentation correction.

Source line references point into the pinned fork
(`lidar_localization_ros2` at `4e8499a`, reconstructed with
`vcs import src < hmr_localisation.repos`; `src/` is not in git).

---

## 0. The gate — now measured

The baseline exists as of 2026-09-11. It was taken **natively on ROS 2 Humble**, not
in the Jazzy container (see the caveats below), with
`scripts/jetson_bag_test_native.sh` on the `curtmini` bag against `gt_map_us050`,
`SMALL_VGICP`, `ndt_num_threads: 8`, headless.

| run | scans | rate Hz | align med | align p95 | gap med | gap p95 | fit med |
|---|---|---|---|---|---|---|---|
| Jetson AGX Orin, 120 s | 700 | **6.39** | 0.0661 | 0.1170 | 0.100 | 0.201 | **0.0648** |
| Jetson AGX Orin, 30 s | 158 | 6.48 | 0.0643 | 0.1244 | 0.100 | 0.300 | 0.0676 |
| Jetson, **1 core** (`taskset -c 0`, `ndt_num_threads: 1`), 120 s | 300 | **2.62** | 0.1763 | 0.5594 | 0.300 | 0.800 | 0.0644 |
| 8-core x86 (documented, `jetson_runs.md` §4) | — | 6.6 | 0.048 | 0.117 | — | — | 0.0644 |

**Reading it.** The Orin lands within 3% of the x86 host's throughput (6.39 vs
6.6 Hz) despite each alignment costing ~38% more (66 vs 48 ms) — the board makes it
up in the tail, where p95 is identical at 117 ms. `gap_med` of 0.100 s is the
number `jetson_runs.md` says to watch: the node keeps up with the 10 Hz sensor at
the median. `gap_p95` 0.201 s means it drops a scan on the slow tail, which is what
the EKF exists to bridge.

The fitness median of **0.0648 against the documented 0.0644** is the strongest
signal here. It says the localizer is not merely running but *tracking correctly* —
map, seed pose, extrinsics and registration backend all agree with the reference
setup to well under a percent.

**What this changes for the backlog.** The node is CPU-bound but not badly: at
6.4 Hz against a 10 Hz sensor it needs roughly a 1.6× speedup to stop dropping
scans. Alignment is ~66 ms of a ~157 ms budget per processed scan, so alignment
alone is not the whole story — items 2, 3 and 7 (crop refresh, fused preprocessing,
fitness-score cost) all sit in the remaining ~90 ms and are now worth measuring
individually rather than estimating.

### Thread scaling is poor — and that is the most actionable finding here

Going from 1 thread on 1 core to 8 threads on 12 cores buys only:

- **2.44× throughput** (2.62 → 6.39 Hz)
- **2.67× on alignment** (176.3 → 66.1 ms median)

An 8× thread count returning under 3× means the registration is **not
thread-scaling** — it is bound by memory bandwidth or serial sections, not by
available cores. Two consequences:

1. **Raising `ndt_num_threads` will not rescue a slow board.** The advice in
   `jetson_runs.md` not to raise it above 8 is right, but for a better reason than
   "leave cores for other containers": the extra threads would buy almost nothing.
2. **Item #1 is confirmed as high-value on core-restricted hosts, and #9/#10 gain
   in relative worth.** Reducing the *work* (fewer target points, fewer source
   points) attacks a bottleneck that adding threads demonstrably does not.

Accuracy is unaffected by thread count — fitness median is **0.0644 on one core,
identical to the 8-thread run and to the documented x86 figure**. Threads buy
throughput only, never quality, so a core-starved deployment degrades in rate
(`gap_med` 0.300 s = every third scan) rather than in tracking.

The one-core run also confirms the §5 mechanism empirically: the node logged
`ndt_num_threads: 1 (using 1; OpenMP sees 1 cores)`, so under `taskset -c 0`
OpenMP's view really does collapse to one core — meaning `ndt_num_threads: 0`
would have worked equally well there, while the shipped `8` would have spawned
eight threads onto that single core.

> **Caveat — these are Humble numbers, not Jazzy.** Getting a native run required
> seven workarounds (§7), most significantly replaying the bag through an explicit
> QoS override instead of its recorded profiles. Treat this as a sound sanity check
> that the board is in the right performance class, not as a substitute for a
> container run. Re-measure in the Jazzy container before publishing.

---

## 1. Verification results

Two leads were costed in detail and reached opposite verdicts.

### The IMU smoother is not a target

The concern was that `optimize()` runs over a 50-pose window
(`include/lidar_localization/imu_gtsam_smoother.hpp:35`), which would make the
factor graph large enough to matter per scan.

It does not, because the window never fills on the normal path.
`updateImuPreintegrationBackend` calls `imu_smoother_.update()` — which pushes a pose,
taking the count to 2 — and then at
`src/lidar_localization_component.cpp:3025` calls
`resetImuPreintegrationSmootherToObservation` on **every non-fallback scan**. That
reaches `initialize()`, which does `poses_.clear(); poses_.push_back(entry)`
(`imu_gtsam_smoother.hpp:67`).

So `optimize()` sees `n=2`, `total_dim=24` — not the 456×456 system it was costed as.
`window_size = 50` only bites during sustained `fallback_mode`. **Not a target.**

### The local-map crop keeps ~97% of the map

This one held up. Measured directly on `gt_map/gt_map_us050.pcd` (196,404 points) at
the metric the code actually uses — 2D, radius `local_map_radius +
local_map_refresh_distance`, which is `80 + 20 = 100` m as shipped
(`config/gt_ouster_ndt_tree_realtime.yaml:45,50`):

| 2D crop radius | points kept | % of map |
|---|---|---|
| 40 m | 94,340 | 48.0% |
| 50 m | 116,311 | 59.2% |
| 60 m | 135,115 | 68.8% |
| 80 m | 175,149 | 89.2% |
| **100 m (shipped)** | **191,034** | **97.3%** |

The map bounding box is 230 × 207 × 52 m, which makes a 100 m radius sound
selective. It isn't: the corners are empty — zero points within 30 m of either
extreme — because the site is corridor-shaped. From anywhere the robot actually
drives, the 100 m crop is essentially the whole map.

---

## 2. Ranked plan

| # | Change | Where | Gain | Risk |
|---|---|---|---|---|
| 1 | `ndt_num_threads` = actual container cores (1 if one core) | config | Large under a `--cpus=1` quota | None — verified there is no clamp |
| 2 | Either `local_map_radius: 80 → 50`, or drop the crop and build the target once | config / fork | 41% fewer target points, or all refresh cost | Radius 50 needs an ATE check |
| 3 | Fuse the 5 preprocessing passes into one | fork | ~8–12 ms/scan | Code change, testable |
| 4 | `gicp_corr_randomness: 20 → 10` | config | Halves covariance cost per refresh | Mild accuracy risk |
| 5 | Patch out dead `source_voxelmap_` in small_gicp v1.0.1 | pinned dep | Memory + build time | Fork-a-pin |
| 6 | `ndt_max_iterations: 50 → 30` | config | Caps the worst scans | Only affects non-converging scans |
| 7 | `fitness_score_max_points: 1000` | config | ~30 ms → ~2 ms | Free here — the score is pure diagnostics in this config |
| 8 | Set a default `CMAKE_BUILD_TYPE` in `CMakeLists.txt` | fork | Prevents a silent no-NDEBUG build | None |
| 9 | Sparser a priori map (`us060`–`us200`, already committed) | config | Cuts target points directly — the thing the crop fails to do | Accuracy unswept past 0.5 m |
| 10 | Match against a subset of LiDAR channels, not all 128 | fork | Cuts source points ~linearly | Ouster-only; needs an ATE check |
| 11 | Combination of #9 and #10 | config + fork | Compounding — the two hit different sides | Interaction not obviously additive |
| 12 | Fix the wrong "Clamped to the cores…" comment in the config | config | Prevents a real one-core mistake | None |

### On #2 — the two options are mutually exclusive

The measured crop numbers change the call here, so it is worth stating plainly.

At radius 80 the crop machinery re-derives 97% of the map every 20 m of travel —
kd-tree construction plus covariance estimation over 191k points — in order to
exclude 3%. That is the worst of both worlds. Pick one:

- **Commit to the crop being pointless at this site.** Build the target once at
  configure time and delete the refresh path from the hot loop. Removes all refresh
  cost, and is honest about what the geometry says.
- **Make it actually crop.** `local_map_radius: 50` keeps 59% — a real 41% cut in
  every downstream per-scan cost. Needs an ATE check against a reference trajectory
  (`scripts/analysis/traj_eval.py`) before it can be trusted.

Shipping radius 80 is the option that pays for the machinery and gets nothing back.

### On #8 — a real trap, not a nicety

`CMakeLists.txt:21` is `SET(CMAKE_CXX_FLAGS "-O2 -g ${CMAKE_CXX_FLAGS}")`. When the
workspace is built the way this repo's README says — with
`--cmake-args -DCMAKE_BUILD_TYPE=Release` — the Release flags land after it and win,
so the build is `-O3 -DNDEBUG` and everything is fine.

But **the fork's own README omits `-DCMAKE_BUILD_TYPE=Release`**. Following it
produces a build with no `NDEBUG`, silently, with no warning at any point. Setting a
default build type in `CMakeLists.txt` closes that hole for anyone who follows the
upstream instructions.

---

## 3. Sparser map, fewer channels, or both (#9–#11)

> ## ⚠ SUPERSEDED — read [`cpu_optimisation.md`](cpu_optimisation.md) instead
>
> As of upstream `7918b5f` (2026-09-11) all three were measured, and **#9 is
> disproven**. This section is kept only for the reasoning trail; the numbers and
> the shipped configuration live in `cpu_optimisation.md`.
>
> - **#9 sparser map — WRONG, do not do this.** My premise below (that the crop
>   keeps 97% of the map, so a sparser map is the only lever that reduces the
>   target) is a real observation attached to a false conclusion. **VGICP registers
>   against the 1.0 m voxel grid, not the point cloud** — `us050` and `us100`
>   produce the *same* 50,884 voxels, so per-scan cost never touches the map's
>   point count at all. A sparser map only lowers the evidence per voxel
>   (3.9 → 1.0 points), degrading the covariances: alignment gets *slower*
>   (36.6 → 46.0 ms), fewer scans are kept, CPU moves under 5%, and yaw noise grows
>   sevenfold. The fitness floor also rises with point spacing (0.065 → 0.205), so
>   scores are not comparable across maps. Keep `gt_map_us050.pcd`.
> - **#10 channel stride — CORRECT, and now shipped.** The fork gained
>   `scan_channel_stride` (pin `2ed0255`) and the realtime config ships **stride 2**.
>   On the 8-core host: 3.7 → 2.6 cores, alignment median 41 → 26 ms, fewer dropped
>   scans, track moved 2.7 cm median. Stride 4 gives 2.1 cores / 21 ms / 4.1 cm.
>   The Hesai caveat I flagged is handled by `scan_channel_count` (set it to the
>   beams per firing block for an unorganised cloud; with 0 the stride is a no-op
>   and the node warns once).
> - **#11 combination — answered: not worth it.** Since #9 buys no CPU, combining
>   is "the stride plus accuracy risk".
> - **#12 wrong thread comment — FIXED upstream** in `39a7b88`, with the same
>   `resolveRegistrationThreadCount` reasoning recorded in §5 below.
>
> **The pin is load-bearing.** `scan_channel_stride` only exists at `2ed0255`; an
> older node *silently ignores it and runs at full resolution*, so a workspace
> built against `4e8499a` will appear to honour `stride: 2` while doing no such
> thing. Re-run `vcs import src --force < hmr_localisation.repos` and rebuild.

Three levers to try. They matter because they attack the two sides of the
registration cost separately: **#9 cuts target points, #10 cuts source points.**
*(The #9 half of that claim is exactly the error corrected above.)*

### #9 — Sparser a priori map

This is the lever the crop was supposed to be. The crop keeps 97% of the map
(§1), so it does not reduce the target at all; changing the map does, directly and
unconditionally.

The existing sweep in
[`downsampled_map_localization_results.md`](downsampled_map_localization_results.md)
stopped at 0.5 m — it says so explicitly: *"voxels coarser than 0.5 m were not
swept here."* So `us060`–`us200` were committed but their **accuracy was never
measured**. That is the gap to close.

Coarser maps beyond 2.0 m did not exist, so they were generated (uniform sampling,
same `scripts/downsample_map_pcl.cpp` as the originals, built natively against
system PCL 1.12 — no Docker needed):

| map | leaf | points | on disk | vs `us050` | accuracy |
|---|---|---|---|---|---|
| `gt_map.ply` (full) | — | 3,059,991 | 48 MB | 15.6× more | reference |
| `us050` | 0.50 m | 196,404 | 3.0 MB | — | 3 cm median (measured) |
| `us060` | 0.60 m | 138,763 | 2.2 MB | −29% | **unmeasured** |
| `us070` | 0.70 m | 102,905 | 1.6 MB | −48% | **unmeasured** |
| `us080` | 0.80 m | 79,270 | 1.3 MB | −60% | **unmeasured** |
| `us100` | 1.00 m | 50,884 | 796 KB | −74% | **unmeasured** |
| `us150` | 1.50 m | 22,568 | 356 KB | −89% | **unmeasured** |
| `us200` | 2.00 m | 12,641 | 200 KB | −94% | **unmeasured** |
| `us250` | 2.50 m | 8,142 | 128 KB | −96% | new, unmeasured |
| `us300` | 3.00 m | 5,613 | 88 KB | −97% | new, unmeasured |
| `us400` | 4.00 m | 3,227 | 52 KB | −98% | new, unmeasured |
| `us500` | 5.00 m | 2,104 | 36 KB | −99% | new, unmeasured |

Regenerate any of these with:

```bash
g++ -O2 -std=c++17 scripts/downsample_map_pcl.cpp -o /tmp/downsample_map_pcl \
  $(pkg-config --cflags --libs pcl_common-1.12 pcl_io-1.12 pcl_filters-1.12) \
  -lpcl_kdtree -lpcl_search -lpcl_octree
/tmp/downsample_map_pcl gt_map/gt_map.ply gt_map/gt_map_us300.pcd 3.0 uniform
```

`.gitignore` already excludes `gt_map/*.pcd` except the committed `us050`–`us200`,
so the new ones stay out of git on their own.

> **Temper expectations at the coarse end.** `us500` is 2,104 points spread over a
> 230 × 207 × 52 m site — roughly one point per 5 m of corridor. `vgicp_voxel_resolution`
> is 1.0 m, sized for a 0.5 m map at ~8 points per voxel; past about 1.0 m leaf the
> voxels start coming up empty and VGICP has nothing to fit. The interesting band is
> most likely **0.6–1.5 m**, and the 2.5–5.0 m maps are there to find the cliff edge
> rather than to be used. Expect to re-tune `vgicp_voxel_resolution` alongside the
> leaf size — holding it at 1.0 m confounds the sweep.

### #10 — Match against a subset of LiDAR channels

The CURTMINI cloud is an **organized 1024 × 128** Ouster frame, so the 128 rows are
the 128 channels. Keeping every 2nd or 4th row gives 64 or 32 channels and cuts
source points near-linearly, before any other preprocessing runs.

Two practical constraints found while checking this:

- **There is no `ring` field to filter on.** `make_jetson_bag.py` repacks the
  trimmed bags to `x,y,z,intensity,t` (curtmini) and `x,y,z,intensity,timestamp`
  (bunker) — see `jetson_runs.md` §2. Channel selection therefore has to use the
  **row index of the organized cloud** (`height`), not a ring field.
- **It does not apply to the bunker bag at all.** That cloud is `230400 × 1` —
  unorganized, height 1. There is no channel structure left to subset, so #10 is
  an Ouster/curtmini-only lever and cannot be evaluated on the Hesai recording.

The fork has no channel/ring filtering today (the only `stride` in the component is
the fitness-score sampler at `lidar_localization_component.cpp:77–101`), so this is
a code change in the preprocessing path.

> **The risk here is different in kind from #9.** Dropping channels does not thin
> the cloud uniformly — it removes whole elevation bands, which is exactly the
> vertical structure that constrains pitch and z. Uniform decimation (every Nth row)
> is much safer than taking a contiguous block of rows. Check z and pitch error
> specifically, not just translation ATE.

### #11 — Both together

Worth trying because they are not redundant: #9 shrinks the kd-tree/voxel target,
#10 shrinks the query set against it. The per-scan cost is roughly
*source × log(target)*, so the two enter in different terms and should compound.

They are not cleanly separable in the accuracy budget, though — both reduce the
constraint count, and the failure mode (under-constrained fit on a degenerate
corridor) is the same one. **Sweep them independently first**, then combine only the
settings that each survived on their own.

---

## 4. Forcing one core without Docker

`--cpus=1` is a Docker mechanism; the host equivalent is `taskset`. But CPU
affinity alone is **not enough**, because of §5 below:

```bash
taskset -c 0 ros2 launch lidar_localization_ros2 lidar_localization.launch.py ...
```

That pins the process to CPU 0, but the node still requests `ndt_num_threads: 8`
OpenMP threads, which then timeshare one core — *worse* than single-threaded, from
context-switching and OpenMP barrier spinning. **Set `ndt_num_threads: 1` in the
config as well.** Belt and braces:

```bash
OMP_NUM_THREADS=1 taskset -c 0 ros2 launch ...   # with ndt_num_threads: 1
```

Setting `ndt_num_threads: 0` also works — it falls through to `omp_get_max_threads()`,
which respects both `OMP_NUM_THREADS` and the affinity mask — but an explicit `1` is
clearer about intent.

Verify from the node's own startup line, which prints all three numbers:

```
ndt_num_threads: 1 (using 1; OpenMP sees 1 cores)
```

---

## 5. The `ndt_num_threads` comment is wrong (#12)

`config/gt_ouster_ndt_tree_realtime.yaml:28` says the value is *"Clamped to the
cores the container can see."* **It is not clamped.**

```cpp
// registration_backend_policy.hpp:82
inline int resolveRegistrationThreadCount(int requested_threads, int fallback_threads)
{
  if (requested_threads > 0) { return requested_threads; }   // returned as-is
  return fallback_threads > 0 ? fallback_threads : 1;
}
```

Every call site passes `omp_get_max_threads()` as the *second* argument
(`lidar_localization_component.cpp:413, 1131, 1156`), which reads like a `std::min`
at a glance but is only the **fallback for when the parameter is unset**. A positive
request is honoured verbatim, however few cores are actually available.

This is what makes §4 a trap rather than a detail, and it is why item #1 is worth
doing on any core-restricted host: on a one-core box the shipped config asks for 8
threads and gets them.

---

## 6. Ruled out

Costed and rejected: arm64 `-mcpu` tuning, LTO, the retry paths, diagnostics
publishing, and TF lookups.

Build flags were also investigated and found already correct under this repo's
documented build command — see #8 for the caveat that makes it worth a change anyway.

> **"Coarser map" appears here in earlier analysis, and that is now superseded.**
> It was rejected before the crop was measured. Once §1 showed the crop keeps 97% of
> the map, a sparser map became the only lever that actually reduces the target — so
> it is promoted to #9 rather than ruled out. The earlier rejection should be read as
> "coarser map does not help *the crop*", which is true and beside the point.
