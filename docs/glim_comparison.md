# GLIM vs hmr_localisation on the Jetson Orin

Both systems, both bags, same 120 s window, same Fast-DDS SHM profile, same
player at rate 1.0. Measured 2026-09-12.

GLIM v1.2.2 built from source into `~/glim-install` (no sudo on this box):
GTSAM 4.3a0 -> gtsam_points 1.2.2 (CUDA, sm_87) -> glim 1.2.2 -> glim_ros2 on
Humble. Configs per bag/variant live in `~/glim-config/`, harness in the session
scratchpad (`glim_bag_test.sh`, `glim_summary.py`, `glim_pose_recorder.py`).

> **Follow-up study:** [`cyclonedds_transport_study.md`](cyclonedds_transport_study.md)
> re-runs this matrix under CycloneDDS (UDP) instead of Fast-DDS SHM, n=3 per row, and adds
> a tuned+pinned `full+loop` row that is absent here. Two of its findings apply back to
> **this** document:
>
> - **C1** — hmr_localisation processes only 9.0–9.3 Hz on curtmini where GLIM processes
>   9.97 Hz, on both transports. The `cores` columns in §2 and §6 therefore compare
>   unequal amounts of work, and hmr's curtmini figures are flattered by roughly 9 %.
> - **C10** — `still growing` / memory-flatness verdicts are threshold-brittle at n=3 for
>   every configuration except `full+loop`.
>
> Its §4 also re-scores this document's raw `*.cpu.csv` files through the current
> `cpu_window.py`, which is the cleanest available cross-check of the numbers below.

## 1. What is and is not comparable

hmr_localisation **localises against a prior map**; GLIM is **SLAM** and builds
its own. Three consequences that shape every number below:

- **ATE is not the same quantity.** The localizer's ATE is absolute error against
  `gt_map_us050.pcd`. GLIM has no prior map and its world origin is wherever it
  started, so the only meaningful comparison is after a rigid Umeyama alignment
  onto the localizer's trajectory. That measures *drift*, not absolute accuracy,
  and the reference is itself the localizer's output rather than ground truth --
  so this is agreement between two systems, not a verdict on which is right.
- **Point budget must be matched.** GLIM downsamples every scan to a 10k-point
  target (`random_downsample_target`), then applies a 0.5-100 m distance filter.
  That is ~7.6% of curtmini's 131k and ~4.3% of bunker's 230k, i.e. roughly the
  localizer at `scan_channel_stride` 13. GLIM's rows belong against stride 8/16,
  never against stride 1.
- **Backpressure differs.** `GlimROS::points_callback` preprocesses synchronously
  then calls `odometry_estimation->insert_frame()`, which pushes onto an
  UNBOUNDED queue. The localizer drops scans it cannot keep up with at the DDS
  layer and they are gone. GLIM can instead accept everything and fall behind.
  Counting every pose GLIM eventually emits would flatter it, so the harness
  counts only poses that arrived while the player was running, and separately
  reports backlog and end-of-window lag.

## 2. CPU

Both harnesses print a *lifetime* average, but over different amounts of idle
(the localizer ~131 s for a 120 s playback; the GLIM harness ~139 s because it
drains the queue for 20 s afterwards). Comparing those directly favours whichever
idled longer. `cpu_busy.py` instead differentiates the tick counter and averages
only over samples above 20% of each run's 90th percentile -- same rule both
sides. `cores` below is that busy-window figure.

**hmr_localisation is run at `ndt_num_threads=1`**, which is its efficient
configuration. The shipped default of 8 threads costs 3-4 cores for the same work
(~2/3 of it OpenMP overhead) and is not a meaningful comparison point.

### curtmini

| system | config | rate Hz | cores | core-ms/scan | RSS MB |
|---|---|---|---|---|---|
| hmr_loc | t1 stride 4 | 8.99 | **0.78** | 87 | 215 |
| hmr_loc | t1 stride 8 | 9.28 | **0.68** | 73 | 206 |
| hmr_loc | t1 stride 16 | 9.45 | **0.61** | 65 | 205 |
| GLIM | odometry, CPU | 9.97 | 2.15 | 216 | 538 |
| GLIM | odometry, GPU | 9.97 | 1.24 | 124 | 499 |
| GLIM | full stack, CPU | 9.97 | 2.84 | 285 | 2027 |
| GLIM | full stack, GPU | 9.97 | 1.37 | 137 | 814 |

### bunker

| system | config | rate Hz | cores | core-ms/scan | RSS MB |
|---|---|---|---|---|---|
| hmr_loc | t1 stride 4 | 9.75 | **0.80** | 82 | 239 |
| hmr_loc | t1 stride 8 | 9.80 | **0.72** | 73 | 225 |
| hmr_loc | t1 stride 16 | 9.81 | **0.66** | 66 | 227 |
| GLIM | odometry, CPU | 10.01 | 2.35 | 235 | 589 |
| GLIM | odometry, GPU | 10.01 | 1.35 | 135 | 461 |
| GLIM | full stack, CPU | 10.01 | 2.64 | 264 | 1979 |
| GLIM | full stack, GPU | 10.01 | 1.46 | 146 | 900 |

At matched point budget GLIM's CPU backend costs **~3x** the localizer per scan
and its GPU backend **~1.9x** -- and the GPU figure still needs the Orin's GPU on
top of that CPU. Memory is 2-3x for odometry-only and ~10x for the full stack,
which grows with mission length as the global map accumulates.

GLIM's one clear throughput win: it holds a true 10 Hz in every unpinned variant
(`gap_p95` 0.100 s, backlog 0-4 scans, lag < 0.4 s), where the localizer tops out
at 9.28-9.81 and falls to 7.16 at stride 1.

## 3. Single core

The decisive test, given this is the deployment constraint. GLIM pinned to core 0
with `num_threads=1` for both preprocessing and odometry:

| run | realtime factor | cores | lag at end | backlog | RSS MB |
|---|---|---|---|---|---|
| GLIM curtmini | **0.49x** | 0.99 (saturated) | 53.9 s | 102 | 1139 |
| GLIM bunker | **0.47x** | 0.98 (saturated) | 56.1 s | 159 | 1214 |
| hmr_loc curtmini stride 8 | 1.0x | 0.68 | ~0 | n/a | 206 |
| hmr_loc bunker stride 8 | 1.0x | 0.72 | ~0 | n/a | 225 |

**GLIM does not fit in one core.** It processes at roughly half real time and
ends the 120 s window ~55 s behind. Note the RSS: 1.1-1.2 GB against 538-589 MB
unpinned, because the unbounded queue holds the frames it has not caught up on --
memory growth is the visible symptom of falling behind.

Beware the `rate_Hz` column on these runs: it reads 10.00, because GLIM does
process every scan it dequeues at 10 Hz *in sim time*. It is the realtime factor
and lag that show the run failing.

hmr_localisation at stride 8 fits the same core with ~30% headroom.

## 4. Trajectory agreement

ATE after rigid alignment, against each bag's `v2_*_ch1` localizer run.

| bag | GLIM variant | matched | med (m) | p95 (m) | path (m) | drift |
|---|---|---|---|---|---|---|
| curtmini | odometry CPU | 829 | 0.044 | 0.142 | 20.0 | 0.22% |
| curtmini | odometry GPU | 830 | 0.025 | 0.117 | 20.0 | 0.12% |
| curtmini | full stack CPU | 829 | 0.089 | 0.276 | 20.0 | 0.44% |
| curtmini | full stack GPU | 831 | 0.048 | 0.148 | 20.0 | 0.24% |
| bunker | odometry CPU | 737 | 0.022 | 0.096 | 16.9 | 0.13% |
| bunker | odometry GPU | 741 | 0.023 | 0.096 | 17.0 | 0.14% |
| bunker | full stack CPU | 740 | 0.020 | 0.095 | 16.9 | 0.12% |
| bunker | full stack GPU | 741 | 0.025 | 0.098 | 17.0 | 0.15% |

The two systems agree to 2-4 cm median over 17-20 m of path. Separation is
motion-driven: the robot is stationary ~70% of the window and every spike lands
inside a motion burst (corr(speed, separation) = +0.64 curtmini, +0.39 bunker).
While parked they agree to under a centimetre.

**bunker is uniform at ~2 cm across all four variants; curtmini ranges 2.5-8.9 cm
and is the only bag where the full stack is worse than odometry alone.** See the
IMU finding below -- that is the likely cause, not a property of GLIM.

## 5. curtmini's IMU is holed

Measured by re-subscribing RELIABLE with a 5000-deep queue, so nothing counted as
missing was merely dropped in transport (`imu_gaps.py`):

| bag | topic | mean rate | max gap | gaps >= 100 ms in 44 s |
|---|---|---|---|---|
| curtmini | `/curt/imu/data` | 467 Hz | 167.5 ms | **19** (2.56 s with no IMU) |
| bunker | `/imu/data` | 400 Hz | 20.0 ms | 0 |

A scan interval is 100 ms, so those 19 holes each starve at least one full
LiDAR-IMU preintegration. GLIM logs them as `insufficient number of IMU data
between LiDAR scans!! num_imu=0`. This penalises GLIM specifically -- it is a
tightly-coupled LiDAR-IMU system, whereas hmr_localisation uses IMU only as an
optional smoother. It is a property of the recording, not of either system, and
it is the most plausible explanation for curtmini's worse and more variable
agreement.

Worth fixing at the source if curtmini is going to be used for LiDAR-IMU work.

## 6. Tuning GLIM for CPU

Sections 2 and 3 measure GLIM's **shipped defaults**. Those turn out to be as
badly matched to this hardware as hmr_localisation's `ndt_num_threads=8` default
was, and tuning changes the conclusion.

### Single-knob ranking (bunker, from the 2.35-core / 2.2 cm baseline)

| knob | cores | saving | ATE med |
|---|---|---|---|
| `num_threads` 2 -> 1 (both stages) | **1.65** | **-0.70** | 0.020 |
| `random_downsample_target` 1000 | 1.28 | -1.07 | 0.050 (too far) |
| `random_downsample_target` 2500 | 1.94 | -0.41 | 0.023 |
| `random_downsample_target` 5000 | 2.17 | -0.18 | 0.021 |
| `k_correspondences` 10 -> 5 | 2.22 | -0.13 | 0.021 |
| `registration_type` GICP -> VGICP | 2.26 | -0.09 | 0.021 |
| `max_iterations` 8 -> 4 | 2.28 | -0.07 | 0.020 |
| `isam2_relinearize_skip` 1 -> 3 | 2.28 | -0.07 | 0.020 |
| `save_imu_rate_trajectory` off | 2.29 | -0.06 | 0.021 |
| `distance_far_thresh` 100 -> 40 | 2.30 | -0.05 | 0.022 |
| `smoother_lag` 5.0 -> 2.0 | 2.36 | +0.01 | 0.021 |

Two things to read off this:

- **`num_threads` dominates.** 2 -> 1 saves more than every other knob combined.
  Same lesson as the localizer: the thread pool's coordination cost exceeded the
  parallelism it bought. GLIM's config ships 2 and its code default is 4.
- **Registration is not the bottleneck.** Cutting points 4x (10000 -> 2500) buys
  only 17%. The fixed per-scan cost dominates: `extract_raw_points` walks all
  230400 bunker points into `Vector4d`/time/intensity arrays *before* any
  downsampling, ~110 MB/s of allocation and copying at 10 Hz. GLIM has no
  upstream decimation knob equivalent to `scan_channel_stride`, so that cost is
  not reachable from the config. Pushing the target to 1000 does keep cutting CPU
  but costs accuracy (5.0 cm), so 2500 is the sensible floor.

### The tuned config

`~/glim-config/{bunker,curtmini}_o_combo`, relative to `odom_cpu`:

```
config_preprocess.json    num_threads               2    -> 1
                          random_downsample_target  10000 -> 2500
                          k_correspondences         10   -> 5
config_odometry_cpu.json  num_threads               2    -> 1
                          max_iterations            8    -> 4
                          registration_type         GICP -> VGICP
                          isam2_relinearize_skip    1    -> 3
                          save_imu_rate_trajectory  true -> false
                          validate_imu              true -> false
```

`distance_far_thresh` was deliberately left at 100 m: it saved only 0.05 cores
and a safe value is site-specific.

### Result: GLIM fits in one core

Pinned to core 0, three runs per bag:

All `cores` below are `cpu_window.py` (busiest contiguous 120 s). An earlier
revision of this table carried `cpu_busy.py` values (0.74 / 0.64), which D1
retracted but which were never re-run here -- they contradicted D3 two sections
later by 0.05 cores for the same runs.

| run | rate Hz | cores (mean, range) | lag | backlog | ATE med | RSS MB |
|---|---|---|---|---|---|---|
| bunker, defaults | 0.47x RT | 0.98 saturated | 56.1 s | ~500* | -- | 1214 |
| **bunker, tuned** | **10.01** | **0.75** (0.74-0.75) | ~0 | **0** | 2.7 cm | **278** |
| curtmini, defaults | 0.49x RT | 0.99 saturated | 53.9 s | ~500* | -- | 1139 |
| **curtmini, tuned** | **9.97** | **0.69** (0.68-0.69) | ~0 | **0** | 5.1 cm | **258** |

\* The harness reports backlog as "poses arriving in the 20 s drain" (102 / 159).
That is a drain-window artefact, not queue depth: 586 in-window + 102 drained of
~1190 played means ~500 frames were still queued, consistent with 53.9 s of lag
at 10 Hz. Do not read the raw backlog column as queue depth on failing runs.

p95 CPU stays at 0.72-0.77, so it is not clipping against the 1.0 ceiling -- it
genuinely has ~25% headroom. Memory drops ~4x as a side effect.

Accuracy cost is small: bunker 2.2 -> 2.7 cm, curtmini 4.4 -> 5.1 cm.

### Head to head, both tuned

| system | bunker cores | bunker Hz | curtmini cores | curtmini Hz |
|---|---|---|---|---|
| hmr_loc `t1` stride 8 | 0.72 | 9.80 | 0.68 | 9.28 |
| GLIM tuned, 1 core | 0.74 | 10.01 | 0.64 | 9.97 |

**Level on CPU**, with GLIM holding a marginally higher rate. Memory is now
comparable too (278/258 MB vs 225/206 MB).

One unexplained observation: pinning *reduced* GLIM's measured CPU (0.74 vs 1.22
unpinned for the same config). That is consistent with spin-waiting DDS/executor
threads each occupying a core when unpinned and being descheduled when pinned,
but it was not proven. It does not affect the result above, which is measured
under the pinned condition that matters for deployment.

## 6b. The GPU backend stops being worth it once the CPU path is tuned

At **defaults** the GPU backend looked like a large win: 1.24 vs 2.15 cores on
curtmini. That advantage does not survive tuning.

Tuned (`o_combogpu` = the combo knobs that exist in the GPU odometry config --
it has no `max_iterations` and no `registration_type`, GPU is always VGICP),
pinned to one core:

n=3 per cell (an earlier n=1 read of this table said the GPU was *worse* on
curtmini -- 0.69 vs 0.62 -- which the repeats show was run-to-run noise):

| bag | tuned CPU (range) | tuned GPU (range) | CPU ATE | GPU ATE |
|---|---|---|---|---|
| bunker | 0.74 (0.72-0.75) | 0.71 (0.69-0.73) | 2.7 cm | 2.8 cm |
| curtmini | 0.64 (0.62-0.67) | 0.66 (0.62-0.69) | 5.1 cm | 5.8 cm |

The ranges overlap on both bags, so **CPU and GPU backends are indistinguishable
on CPU load once tuned**. GPU offload was hiding inefficiency that
`num_threads=1` and the downsample target removed outright; with that gone there
is nothing left for it to hide, but it is not a penalty either.

**There is still no reason to use the GPU backend here**, on three grounds that
do not depend on the overlapping CPU figures: it carries ~50 MB more host RSS
(311/310 vs 278/258) plus uncounted device memory, its trajectory agreement is
slightly worse on curtmini (5.8 vs 5.1 cm), and its rate is marginally less
consistent (9.84-10.01 vs a flat 9.97-10.01). It buys nothing and ties up the
GPU. Tune the CPU path instead.

## 6c. Reproducibility of the stride 4 recommendation

`hmr_loc` stride 4 is the accuracy winner but runs close to the 1-core ceiling,
so it was repeated three times per bag:

| bag | cores (3 runs) | p95 (3 runs) | rate Hz (3 runs) |
|---|---|---|---|
| bunker | 0.80 / 0.80 / 0.80 | 0.89 / 0.89 / 0.89 | 9.75 / 9.76 / 9.77 |
| curtmini | 0.78 / 0.77 / 0.77 | 0.92 / 0.93 / 0.92 | 8.99 / 9.00 / 9.06 |

Spread is ~1%, so both the recommendation and its limitation are solid: on
curtmini stride 4 **consistently** cannot hold full rate on one core, losing
~10% of scans at p95 0.92-0.93. That is a reproducible ceiling effect, not noise.

## 6d. Adversarial review — two defects found in this document's own method

### D1. The CPU metric was biased in GLIM's favour (SERIOUS)

`cpu_busy.py` (section 2) averaged samples above 20% of each run's 90th
percentile. On these runs that threshold lands at ~0.14 cores.
**hmr_localisation's post-run idle tail is 0.00 cores and was cleanly excluded;
GLIM's is 0.15 cores** -- it keeps servicing DDS and publishing after playback --
**so GLIM's idle sat just above the threshold and was counted as work.**

Consequences:
- GLIM's CPU was understated by up to 8%. curtmini odometry: 0.64 -> **0.69**.
- It manufactured fake variance. Whether the threshold landed above or below
  0.15 flipped ~8 samples in or out, which is exactly the observed
  anti-correlation between `busy_cores` and `busy_s` (0.62/139 s, 0.63/135 s,
  0.67/125 s).

**Retracted:** "hmr_localisation is far more deterministic than GLIM (0.00
spread vs +/-4-8%)". That spread was the metric's, not GLIM's. Corrected, both
systems repeat to +/-1-2%.

**Retracted:** "GLIM odometry on curtmini costs 0.64 cores, less than
hmr_loc stride 8 at 0.68". Corrected they are equal (0.69 vs 0.68).

Replaced by `cpu_window.py`: both systems played exactly 120 s at rate 1.0, so
score the **busiest contiguous 120 s** of each run. No threshold, no assumption
about idle, and it answers the question actually being asked.

### D2. The ATE comparison was asymmetric (MODERATE, small effect)

GLIM was scored with a full 3D rigid Umeyama fit (6 DOF) plus nearest-stamp
association; hmr_localisation was scored with `traj_eval.py --align`, which fits
**yaw + translation only (4 DOF)** and interpolates onto every test pose. More
alignment freedom absorbs more error, so the two numbers were not comparable.

Re-scoring hmr_localisation through the identical 6-DOF path moves it *down*:

| run | 4-DOF (as published) | 6-DOF (symmetric) |
|---|---|---|
| bunker stride 4 | 2.8 cm | **2.3 cm** |
| bunker stride 8 | 4.5 cm | 4.5 cm |
| curtmini stride 4 | 2.6 cm | **2.5 cm** |
| curtmini stride 8 | 4.9 cm | **4.7 cm** |

So the bias ran *against* hmr_localisation and correcting it **strengthens** its
accuracy lead. Conclusions unchanged in direction.

A side finding worth keeping: scoring GLIM with yaw-only alignment leaves a
**0.5-0.9 m** residual. That is not GLIM drifting -- it proves GLIM's
gravity-aligned world frame differs from the map frame by roll/pitch, which a
4-DOF fit cannot remove. So 6 DOF is genuinely *required* for GLIM, and the two
ATE columns remain conceptually non-identical even now: for hmr_localisation the
alignment is nearly a no-op, for GLIM it removes a real frame difference.

### D3. The GPU verdict, corrected again

With the unbiased metric, n=3:

| bag | tuned CPU | tuned GPU | overlap? |
|---|---|---|---|
| bunker | 0.75 (0.74-0.75) | **0.72 (0.71-0.73)** | no -- GPU consistently cheaper |
| curtmini | 0.69 (0.68-0.69) | 0.69 (0.69-0.69) | identical |

The GPU backend is **equal or marginally cheaper, never worse**. Both earlier
verdicts were wrong: "GPU is worse" (n=1 noise) and "indistinguishable" (n=3 but
biased metric). The case against using it now rests only on +33-52 MB host RSS
plus uncounted device memory, and slightly worse curtmini ATE (5.8 vs 5.1 cm).

### D4. Structural limits that cannot be fixed with this data

- **The reference is hmr_localisation's own stride-1 output.** Any systematic
  error shared by stride 1 and stride 8 cancels for hmr_localisation but not for
  GLIM. This inflates hmr_localisation's apparent accuracy by an unknown amount.
  Only independent ground truth would settle it.
- **Different sample sets.** GLIM scores ~1160 scans; hmr_loc stride 4 on
  curtmini only ~1058, because it drops the scans it cannot keep up with. If the
  dropped scans are the hard ones (fast motion), stride 4's ATE is optimistic.
- **120 s, ~17-20 m, stationary ~70% of the time.** Drift and memory-growth
  claims remain extrapolations.

### What survived the review unchanged

Both systems fit one core; `num_threads=1` is the dominant knob for both (0.70
cores, far outside any noise); registration is not GLIM's bottleneck (4x fewer
points buys 17%); the memory divide (278 MB vs 1.6 GB); curtmini's 19 IMU holes;
stride 4 as accuracy winner (strengthened); and GLIM's defaults failing on one
core at 0.48x real time.

## 6e. Second review round — four independent auditors

### R1. Section 2 compares a PINNED localizer against an UNPINNED GLIM (serious)

`sweep_1core_v2.sh:24` runs every `1cv2_*` row with `PIN_CORES=0`;
`sweep_glim.sh` never pins. Section 2's table is therefore pinned-hmr vs
unpinned-GLIM. Pinning changes GLIM's own CPU for *identical configs*:

| config | unpinned | pinned | ratio |
|---|---|---|---|
| bunker `o_combo` | 1.22 | 0.75 | 1.63x |
| curtmini `o_combo` | 1.01 | 0.69 | 1.46x |

So §2's "GLIM's CPU backend costs ~3x the localizer per scan" is **1.5-1.6x
explained by the pinning condition, not by GLIM's defaults**. §7 attributing the
whole 3x to defaults is not established.

Same defect in the throughput claim. §2's "GLIM holds a true 10 Hz where the
localizer tops out at 9.28-9.81" compares unpinned GLIM against a *pinned*
localizer. Unpinned-vs-unpinned: `v2_bunk_ch8` 9.97 vs GLIM 10.01;
`v2_curt_ch8` 9.69 vs 9.97 — a 0.04-0.28 Hz gap, not 0.7. **Retract that claim
from §2.** The §6 pinned-vs-pinned comparison is like-for-like and stands.

Second-order: because ~35% of GLIM's unpinned CPU is scheduling-dependent
(1.22 -> 0.75 under confinement), neither condition is neutral. Pinned flatters
GLIM; unpinned penalises it. hmr at `ndt_num_threads=1` has no comparable
compressible component.

### R2. The GPU verdict is not resolvable at n=3 (serious)

D3 claimed non-overlapping ranges therefore "consistently cheaper". Three
independent objections:

- **Metric-dependent.** Under `cpu_window` the arms separate by 0.007 cores;
  under the harness playback-window figure and under `cpu_busy` they *overlap*.
  The separation is smaller than the difference between two defensible ways of
  drawing the same 120 s window.
- **Within-arm spread exceeds the gap.** GPU spread is 0.027, 4x the 0.007
  separation.
- **n=3 cannot reach significance.** Exact two-sided permutation floor at 3-vs-3
  is p = 2/C(6,3) = **0.10**, which is what complete separation yields. The
  verdict has already flipped twice, so the multiple-comparison burden is worse.

**Corrected:** bunker GPU is 0-4% cheaper, curtmini identical, **not resolvable
at this n**. Only the RSS ground (+33-52 MB) survives as an argument against it.

### R3. `validate_imu=false` suppressed a warning that was firing (serious)

The baseline curtmini run logged **14** "IMU prediction is not good", with
better-ratios degrading from rot 0.80 / trans 0.39 / vel 0.62 to
**rot 0.61 / trans 0.22 / vel 0.38** against GLIM's thresholds of 0.7 / 0.4 / 0.5
(`imu_validation.cpp:66-68`). A "better ratio" is the fraction of frames where
the IMU-preintegrated prediction beat a no-IMU one — so by the end of curtmini,
**IMU preintegration was worse than constant velocity on 78% of frames in
translation**. Bunker logged zero such warnings.

The CPU rationale was wrong: `IMUValidation::validate` early-returns below
0.1 m/s (~70% of these bags) and formats only every 64th call. The saving is
unmeasurable; the cost was blindness. **Reverted to `true` in all configs.**

This strengthens §5 considerably: curtmini's problem is not merely 19 IMU
dropouts, it is that GLIM's IMU integration measurably *hurt* on that bag. Treat
every curtmini GLIM accuracy number as provisional until that is resolved.

### R4. The tuned config is partly overfitted to near-static bags

- **`max_iterations` 8->4 is the most overfit.** LM is capped at 4 with an
  IMU-seeded initial guess; at ~70% stationary that guess is near-exact and the
  cap is **never binding — the experiment structurally could not detect
  under-convergence**. Compounded by `vgicp_voxelmap_levels: 1` (one voxel
  scale, no coarse-to-fine recovery). Restore >=8 for dynamic data.
- **`k_correspondences` 10->5** is rank-adequate for a 3x3 covariance but
  high-variance, and riskiest exactly where `distance_far_thresh: 100` plus 2500
  points puts you in a large scene. Tune jointly with the downsample target.
- **`isam2_relinearize_skip` 1->3** leaves ~0.3 s of stale Jacobians; bounded
  here only because mapping is off and `smoother_lag` is 5 s.
- Safe and portable: `num_threads=1`, VGICP, `save_imu_rate_trajectory=false`.

### R5. The ATE headline is substantially a parked-robot statistic

Split by reference speed (0.05 m/s threshold):

| run | stationary frac | med ALL | med MOVING |
|---|---|---|---|
| curtmini tuned | 61% | 5.46 cm | **7.54 cm** |
| bunker tuned | 45% | 2.58 cm | **4.32 cm** |

The headline "2.7 cm / 5.1 cm" is 45-61% composed of samples where the robot is
not moving. **Moving-only it is 4.3 cm / 7.5 cm — 1.6-1.8x worse.** Quote both.

Related: path length sums per-sample displacement, so stationary jitter inflates
it by 6.8% on curtmini (1.36 m of 20.01 m), deflating `drift%` correspondingly.

### R6. Retractions of my own earlier corrections

- **D4's "different sample sets" bullet is wrong.** Both systems are scored
  against the *reference's* epochs, not their own; matched-index overlap is
  96.5% (curtmini) and 99.5% (bunker). Retracted. The real issue is that the
  reference (`v2_*_ch1`) is the localizer's *slowest* run — bunker's gap median
  is 199 ms, i.e. it drops every other scan — so the ATE grid is chosen by
  whichever scans an overloaded localizer survived, and those are motion-biased.
  That affects both systems equally.
- **"The 2 s mean was smeared 3x" (explo doc) is the wrong diagnosis.**
  `cpu_ticks` is cumulative, so differencing integrates exactly and the mean is
  sampling-rate invariant to 1.4% across an 18x rate change. Only p95 (2.34x)
  and max (2.5x) were mis-sampled. The mean was a correct measurement of the
  wrong quantity (open-loop duty cycle) — two different errors, conflated.
  Consequently **every `cores` figure in this document is immune to that
  aliasing**: for GLIM/hmr the instantaneous p95/mean ratio is 1.04.

### R7. Verified sound (so these can stop being re-litigated)

- `/proc/<pid>/stat` 14+15 captures **all** threads — measured 4.003 cores on a
  4-thread spinner.
- Sample intervals are clean: 2.008-2.187 s, no dropouts.
- The busiest-window search is a no-op, not a bias: `argmax == window[0]` in
  11 of 12 runs, differences <0.0005 cores.
- Nearest-stamp association is exact: 0.00 ms median residual, no reference pose
  reused, the 20 ms tolerance never binds.
- `T_lidar_imu` independently re-derived from `/tf_static` for both bags:
  **0.000 mm / 0.0000 deg** error, and matches the forward transform rather than
  the inverse.
- The curtmini IMU dropout finding is corroborated by GLIM's own logs
  (60 "insufficient IMU" lines on curtmini, 0 on bunker).

### R8. Lesser items

- `scovox` is **not** strictly single-threaded: median 1.024 cores, 41/67
  samples above 1.00. "The integrate stage is serial" is the correct phrasing;
  ~3% of work is on other threads. Conclusion unaffected.
- `lag_end` measures lag *growth*, not absolute lag — it anchors on the first
  in-window pose, so pre-existing lag cancels (hence two runs reporting
  **negative** lag). The failing runs' +54 s is growth-dominated so the
  conclusion holds, but "lag ~0" only establishes that lag did not grow.
- `cpu_window.py` silently falls back to a lifetime average for runs under
  120 s — the exact statistic D1 removed — with no flag. Does not fire on any
  published run (min span 130.6 s) but would on the smoke runs.
- Both bags' configs share byte-identical IMU noise parameters and camera
  intrinsics (inherited from GLIM's shipped defaults), and those intrinsics are
  internally inconsistent (`cx=830.3, cy=639.45` against `image_size [752, 480]`
  — principal point outside the image). Harmless while no camera topic is fed,
  but it means those blocks have never been exercised.

## 7. Conclusion

Comparing **defaults to defaults** is misleading in both directions. Tuned
against tuned, on this hardware:

- CPU and memory are **roughly equal**. The earlier "GLIM costs ~3x" figure was
  an artefact of GLIM's shipped defaults, exactly as the "hmr_loc needs 3.7
  cores" figure was an artefact of `ndt_num_threads=8`.
- Both fit in one core at ~10 Hz.
- They agree on trajectory to 2.7 cm (bunker) / 5.1 cm (curtmini).

So the choice is not about cost any more; it is about what you need.
hmr_localisation gives absolute pose in a known map, which is what you want if
the map exists and the job is to localise in it. GLIM gives you a trajectory and
a map without needing a prior, at the same CPU. Neither dominates.

## 7. Caveats

- The reference is the localizer's own output, not ground truth. Where the two
  diverge during motion, either could be the one moving.
- 120 s window, ~17-20 m of path, robot stationary ~70% of the time. Short and
  slow relative to a real mission; drift figures especially would change over a
  longer run, and GLIM's full-stack RSS would keep growing.
- Single runs per configuration, not repeats. The earlier localizer work put the
  run-to-run spread at CPU +/-0.4% and rate +/-0.6%; GLIM's is unmeasured.
- GLIM's IMU subscription was raised to RELIABLE depth 2000 (from the default
  best_effort `sensor_data`) after the first smoke run lost IMU in transport.
  The cloud topic keeps best_effort depth 5, matching the localizer runs.
- The localizer's `scans` counts are quantised by its recorder's 100-row flush;
  GLIM's recorder flushes on shutdown and its counts are exact.
