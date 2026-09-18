# SCovox mapping node — CPU / memory on the `bunker` bag

Live ROS node (`scovox_mapping_node`), not the offline replay harness.
Measured 2026-09-18. 3 reps per arm, median reported, spread = (max−min)/median.

## Setup

| | |
|---|---|
| bag | `bags/bunker_jetson/bunker_jetson_0.mcap` — 120 s, `/hesai/points` 10 Hz (230 400 pts/scan), `/imu/data` 400 Hz |
| odometry | `hmr_localisation` → `lidar_localization_ros2`, `registration_method: SMALL_VGICP` |
| box | 12 cores, otherwise quiet. Arms 1-2 unpinned; arm 3 pins SCovox to core 4 (`SCOVOX_PIN=4`), localizer left free |
| harness | `scripts/cyclonedds_study/scovox_bag_test.sh` |

The bag carries **no `/tf`**, so SCovox has no pose source of its own and silently
drops every scan without one (`scovox_node.cpp:1960/1970`). The localizer supplies
`map`→`odom`; `odom`→`base_link` is a static identity. All platform motion therefore
lives in `map`→`odom`, so **`integration_frame` must be `map`** — the shipped
`lidar_mapping.yaml` default of `odom` would stack every scan at the origin.

## Carving / TSDF state (both arms)

| setting | state | evidence |
|---|---|---|
| free-space ray carving | **ON**, full ray | `carve_band: -1.0` |
| no-return ray carving | **OFF** | structural — see below |
| TSDF | **OFF** | banner: `TSDF: sdf_trunc=0.000 m space_carving=0` |

`carve_no_return` does not exist in the ROS node — it is a `scovox_frontier` replay
flag only. On the LiDAR path both the downsample branch (`scovox_node.cpp:1817-1821`)
and the per-point branch (`:1881-1884`) `continue` on non-finite **and** out-of-range
returns *before* binning or `integrateHit`, so no free ray is ever cast for a return
that did not come back. `trace_no_return_rays` (`:540`) is an RGB-D debug trace
(used only at `:1349`), not a carve switch.

## Results

| arm | CPU | peak RSS | scan rate | frame p50 |
|---|---|---|---|---|
| res 0.10 m / range 50 m / ds 0.50 m | **0.94 cores** ±0.0% | **559.5 MB** ±0.4% | 3.04 Hz ±1.0% | 328.8 ms ±0.9% |
| res 0.20 m / range 20 m / ds 0.20 m | **0.94 cores** ±0.0% | **213.9 MB** ±0.5% | 7.76 Hz ±0.6% | 124.2 ms ±1.9% |
| ratio | 1.00× | **0.38×** | 2.55× | **0.38×** |
| tuned, **pinned to 1 core** | **0.90 cores** ±1.1% | **215.8 MB** ±1.7% | 7.58 Hz ±0.3% | 125.7 ms ±1.7% |
| cost of pinning | −4.3% | +0.9% | −2.3% | +1.2% |

Localizer running alongside: 2.06–2.20 cores, ~246 MB (its own cost, not SCovox's).
It is never pinned — it wants ~2.1 cores, and pinning it with SCovox would starve it
and degrade SCovox's rate for reasons unrelated to SCovox. `PIN_CORES` (localizer)
and `SCOVOX_PIN` (node under test) are therefore separate knobs.

## Reading the CPU number

**0.94 cores is a LOCK ceiling, not a core ceiling — and the node is not
single-threaded.** It runs 14 threads with an explicit 2-thread executor
(`scovox_node.cpp:3502`, `MultiThreadedExecutor(ExecutorOptions(), 2)`), and the CPU
splits across two busy threads at ~0.48 + ~0.52 cores. Both sit in
`futex_wait_queue_me`, blocked on the map write lock the scan callback takes at
`scovox_node.cpp:1930`. That lock serialises integration, so the second executor
thread buys no parallelism.

Three independent confirmations:

| check | baseline | tuned |
|---|---|---|
| predicted rate `1/frame_ms` vs measured | 3.04 vs 3.04 Hz (−0.0%) | 8.05 vs 7.76 Hz (−3.6%) |
| implied CPU `rate × frame_ms` vs measured | 1.00 vs 0.94 | 0.96 vs 0.94 |
| pinning both threads onto ONE core | — | costs only **2.3%** rate |

The last is the decisive one: two genuinely parallel threads forced onto one core
would lose ~50%, not 2%. The node does ~1 core-second of serialised integration per
wall second and idles the rest, which is why CPU reads ~0.9–0.94 in every arm
regardless of settings. Tuning bought 2.6× throughput and memory and moved CPU not
at all. **Adding cores will not help SCovox here; only `frame_ms` will.** The figures
that track work done per scan are `frame_ms` and peak RSS.

Neither arm keeps up with the sensor: the bag offers ~1200 scans in 120 s, the node
admitted 365 (baseline) and 917 (tuned). The remainder are dropped by DDS at the
subscription queue (best-effort, `KeepLast(10)`), which is correct real-time
behaviour but means the map is built from 30% / 76% of the available scans.

`tf_fallback = 9–10` scans per run missed the exact-stamp TF and were integrated at
the previous pose.

## Caveat, not yet quantified here

The node is built with `SCOVOX_WALKER_TIMERS=1` — it defaults to 1 in
`walker_timers.hpp:29`, so every build carries it unless explicitly zeroed. The
walkers then take two `steady_clock` reads **per ray**. Prior measurement on the
replay binaries put this at +12.7% on a comparable row. Both arms above carry it
equally, so the ratio is unaffected, but the absolute CPU and `frame_ms` figures
are inflated by roughly that much.

## Field-name trap

`tsdf_ms` in the per-scan log is **not** TSDF time. `scovox_map_split.hpp:967-970`:
on the fused walker the combined carve cost is reported under `tsdfTimeUs()` and
`semdirTimeUs()` is 0 by design. With TSDF off, `tsdf_ms ≈ integrate_ms` is the
Beta occupancy full-ray carve — which is why it reads 97% of the frame while the
banner simultaneously says TSDF is off. Both statements are true.

## What the two busy threads actually are

Answered by measurement, after a wrong first guess. The node runs **14 OS threads**
with a 2-thread executor (`scovox_node.cpp:3502`) and two callback groups: the
default group (scan integration, IMU, clock) and `viz_cb_group_` (`:206`), which owns
the once-per-second map-publish timer.

The obvious reading — "one thread maps, the other publishes" — is **wrong**:

| test | result | what it rules out |
|---|---|---|
| disable the viz timer (`scovox_publish_rate:=0.01`) | CPU **0.94 → 0.94 cores**, rate 7.76 → 7.93 Hz | publishing is not the second thread's cost |
| sample thread run-states, 2000 × 2 ms | **94.8% of samples have exactly ONE thread in R**; 2-at-once only 4.3% | any real concurrency |
| per-thread share of R samples | 51.8% / 48.0%, summing to ~100% | fixed roles — they alternate |
| pin both threads to one core | −2.3% rate | parallelism worth having |

So the two threads are **two interchangeable executor workers taking turns on one
serialised job**, not two different jobs. The ~50/50 CPU split is round-robin
hand-off. The default callback group is MutuallyExclusive and the map write lock
(`:1930`) serialises the rest, so only one thread can be doing mapping at any moment.

Consequence: `publish_ms=0.0` in the per-scan log means publishing happens off the
scan thread, not that it is free — but measured, it really is nearly free here
(+2.2% rate when removed), because at `res 0.20 / range 20 m` the map walk is cheap
relative to a 121 ms scan. That will not hold for a larger map.
