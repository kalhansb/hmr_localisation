# Jetson AGX Orin: channel stride and map density (measured 2026-09-11)

The Jetson counterpart to [`cpu_optimisation.md`](cpu_optimisation.md), which was
measured on an 8-core x86 laptop. Same bags, same 120 s windows, same
`SMALL_VGICP` backend at the same pinned small_gicp (v1.0.1, `57c1106`), same
`gt_map_us050` unless stated. Both robot bags.

Run with [`scripts/jetson_bag_test_native.sh`](../scripts/jetson_bag_test_native.sh)
— ROS 2 Humble natively, **not** the Jazzy container (see Caveats).

> **These tables were corrected after an independent audit.** A first version
> reported scan counts of exactly 700/600, which were an artefact:
> `record_alignment_status.py` flushes its CSV every 100 rows, and the run script's
> drain loop reads the file before the recorder's final flush, so the published
> counts were truncated to a flush boundary. Every count below is `wc -l` of the
> **final** CSV. The "cores" column gained a second reading for the same reason —
> see the note under the tables.

**Two CPU columns.** `cores (play)` is averaged over the playback window only, and
is the honest deployment figure. `cores (win)` is averaged over the script's whole
131 s sampler window, which includes ~13 s of post-playback idle and so understates
load by 8–12%. The laptop script has the identical dilution, so **`cores (win)` is
the column to compare against `cpu_optimisation.md`**; `cores (play)` is the one to
size a deployment with.

`scans` is followed by the fraction of the bag's clouds kept (CURTMINI 1195,
bunker 1200). `ATE`/`yaw` are 2D medians against that bag's stride-1 / `us050` run
via `scripts/analysis/traj_eval.py`.

---

## 1. Channel stride

### CURTMINI (Ouster, organised 1024x128, row = channel)

| stride | src pts | cores (play) | cores (win) | RSS | scans | rate | align med / p95 | fit med | ATE vs 1 | yaw |
|---|---|---|---|---|---|---|---|---|---|---|
| 1 | 37,363 | 4.24 | 3.85 | 448 MB | 733 (61.3%) | 6.29 Hz | 66.8 / 118.1 ms | 0.0648 | — | — |
| **2** (shipped) | 23,393 | 4.16 | 3.71 | 432 MB | 1002 (83.8%) | **8.62 Hz** | 46.9 / 86.4 ms | 0.0663 | 2.6 cm | 0.03° |
| 4 | 13,299 | 2.84 | 2.54 | 418 MB | 1102 (92.2%) | **9.48 Hz** | 29.5 / 50.3 ms | 0.0674 | 4.1 cm | 0.07° |

### bunker (Hesai, unorganised 230400x1, `scan_channel_count: 128`)

| stride | src pts | cores (play) | cores (win) | RSS | scans | rate | align med / p95 | fit med | ATE vs 1 | yaw |
|---|---|---|---|---|---|---|---|---|---|---|
| 1 | 33,677 | 3.82 | 3.47 | 478 MB | 645 (53.8%) | 5.53 Hz | 70.5 / 110.5 ms | 0.0849 | — | — |
| **2** (shipped) | 19,132 | 3.70 | 3.36 | 447 MB | 990 (82.5%) | **8.50 Hz** | 42.9 / 66.7 ms | 0.0812 | 2.5 cm | 0.06° |
| 4 | 10,665 | 2.92 | 2.61 | 431 MB | 1153 (96.1%) | **9.91 Hz** | 27.0 / 42.0 ms | 0.0814 | 3.7 cm | 0.10° |

**The stride works on the Orin, and the accuracy cost reproduces the laptop's
closely** — 2.6 vs 2.7 cm at stride 2, 4.1 vs 4.1 cm at stride 4, yaw within 0.01°.
Source point counts agree to within 0.2–3.3% depending on configuration (the
remaining gap is that the two hosts accept different subsets of scans).

**What the stride buys here is throughput, not CPU.** At stride 1 the Orin is
compute-limited: 4.24 cores and only 61% of CURTMINI's clouds processed. Stride 2
cuts per-scan cost to ~70% but CPU barely moves (4.24 → 4.16) because the node
spends the headroom on **+36.7% more scans** (733 → 1002). The arithmetic closes:
1.367 × 0.702 = 0.96, against the measured 4.16/4.24 = 0.98. On bunker the effect is
larger still, +53.5% scans (645 → 990) for a 3% CPU drop.

Only at stride 4 does CPU genuinely fall (2.84 / 2.92 cores), because the node is
near the sensor rate and cannot spend the savings on more work.

> **Recommendation for this board: stride 2 as the default, stride 4 if the board
> is shared.** Stride 2 takes CURTMINI from 61% to 84% of scans for 2.6 cm. Stride 4
> reaches 92% / 96% and is the only setting that returns ~1.3 cores — which matters,
> since the deployment also runs GLIM, the drivers and a planner.
>
> **Note the Orin never fully keeps up.** Even stride 4 is 9.48 / 9.91 Hz against a
> 10 Hz sensor. The laptop's bunker stride 2 reached 97% of scans; the Orin does not
> match that at any stride tested.

---

## 1b. Where the stride breaks (8 / 16 / 32 / 64)

Ouster has 128 channels, so these keep 16, 8, 4 and 2 beams. `ATE`/`yaw` are 2D
medians vs that bag's stride-1 run; `max` is the worst single sample.

### CURTMINI

| stride | beams | src pts | cores (play) | scans | rate | align med | fit med | ATE med / p95 / max | yaw |
|---|---|---|---|---|---|---|---|---|---|
| 4 | 32 | 13,299 | 2.84 | 1102 (92.2%) | 9.48 Hz | 29.5 ms | 0.0674 | 4.1 / 7.7 / — cm | 0.07° |
| 8 | 16 | 7,174 | 1.71 | 1120 (93.7%) | 9.65 Hz | 16.9 ms | 0.0687 | 8.7 / 14.0 / 23.7 cm | 0.07° |
| 16 | 8 | 3,787 | 1.15 | 1132 (94.7%) | 9.76 Hz | 10.2 ms | 0.0704 | 10.7 / 24.1 / 42.4 cm | 0.08° |
| 32 | 4 | 2,024 | 0.77 | 1140 (95.4%) | 9.79 Hz | 6.4 ms | 0.0820 | **35.5 / 57.8 / 81.0 cm** | 0.32° |
| 64 | 2 | 1,183 | 0.63 | 1147 (96.0%) | 9.85 Hz | 4.5 ms | **0.6673** | **85 cm / 11.6 m / 12.6 m** | **10.1°** |

### bunker

| stride | src pts | cores (play) | scans | rate | align med | fit med | ATE med / p95 / max | yaw |
|---|---|---|---|---|---|---|---|---|
| 4 | 10,665 | 2.92 | 1153 (96.1%) | 9.91 Hz | 27.0 ms | 0.0814 | 3.7 / 8.1 / — cm | 0.10° |
| 8 | 5,487 | 1.68 | 1156 (96.3%) | 9.95 Hz | 14.9 ms | 0.0768 | 5.0 / 15.3 / 22.3 cm | 0.08° |
| 16 | 2,758 | 1.05 | 1156 (96.3%) | 9.95 Hz | 7.6 ms | 0.0654 | 7.7 / 23.7 / 57.3 cm | 0.07° |
| 32 | 1,326 | 0.77 | 1182 (98.5%) | 9.93 Hz | 4.6 ms | 0.0668 | 8.2 / 21.0 / **95.4 cm** | 0.15° |
| 64 | 659 | 0.94 | 1134 (94.5%) | 9.76 Hz | 6.8 ms | **10.0794** | **59.9 m / 60.2 m / 60.8 m** | **150.9°** |

**Throughput saturates; only cost keeps falling.** Past stride 4 the rate is pinned
at 9.5–9.95 Hz — the node is sensor-limited, not compute-limited — so further stride
buys CPU and nothing else: **4.24 → 0.63 cores**, a 6.7× reduction, with alignment
down from 66.8 ms to 4.5 ms. RSS barely moves (448 → 403 MB), confirming the map
dominates memory, not the scan.

**The cliff is at 32–64, and it is a cliff.** Fitness holds flat to stride 16
(0.065–0.070, i.e. indistinguishable from full resolution) then breaks:

- **stride 64 has lost the track on both bags.** bunker is the unambiguous failure —
  **59.9 m and 150.9° off**, i.e. somewhere else entirely, facing backwards, with
  fitness 10.08 (123× nominal). CURTMINI degrades less violently but is still gone:
  85 cm median, 11.6 m p95, 10.1° yaw.
- **stride 32 is where CURTMINI becomes unusable** (35.5 cm median, p95 past the
  0.5 m divergence threshold) while bunker still looks healthy on the median
  (8.2 cm) — but with a 95 cm worst sample, so it is surviving rather than tracking.
- **stride 8 is the last setting that is clearly safe on both** (8.7 / 5.0 cm), and
  it is remarkable value: **1.71 cores for 8.7 cm**, vs 4.24 cores at stride 1.

This is the failure mode predicted by the geometry: the stride removes whole
elevation bands while leaving azimuth untouched, so **yaw stays excellent right up
to the break** (0.07–0.08° at stride 16, barely worse than stride 2's 0.03°) and it
is the vertical constraint that collapses. Two beams cannot constrain pitch and z,
and the solution runs away. Note that yaw error only becomes large *after* the
track is lost — it is a symptom, not the cause.

The two sensors fail differently at stride 64 — bunker catastrophically (60 m),
CURTMINI partially (0.85 m) — which is consistent with the Hesai being an
unorganised cloud where `channel = index % 128` is a modular reconstruction rather
than a true row index, so extreme strides sample a far less uniform subset.

> **If CPU is the binding constraint on this board, stride 8 is the aggressive-but-
> defensible setting** — a 2.5× CPU cut below the shipped stride 2, for 8.7 cm and
> no loss of yaw accuracy. **Do not go past 16.** Stride 32 is already unusable on
> the Ouster bag and stride 64 loses the robot.

> **Caveat specific to this table:** the audit agent was running on this machine
> during these eight runs, so the `cores` figures here may be inflated by a few
> percent relative to §1, which was measured on an idle board. The accuracy columns
> are unaffected.

---

## 2. Map density (stride forced to 1, so rows compare with stride-1 above)

### CURTMINI

| map | points | cores (play) | cores (win) | RSS | scans | align med | fit med | ATE vs `us050` | yaw |
|---|---|---|---|---|---|---|---|---|---|
| `us050` | 196,404 | 4.24 | 3.85 | 448 MB | 733 (61.3%) | 66.8 ms | 0.0648 | — | — |
| `us060` | 138,763 | 4.29 | 3.96 | 425 MB | 742 (62.1%) | 66.8 ms | 0.0853 | 2.3 cm | 0.03° |
| `us070` | 102,905 | 4.30 | 3.84 | 414 MB | 709 (59.3%) | 68.3 ms | 0.1097 | 2.9 cm | 0.07° |
| `us100` | 50,884 | 4.30 | 3.90 | 402 MB | 706 (59.1%) | 71.6 ms | 0.2048 | 5.9 cm | 0.19° |

### bunker

| map | points | cores (play) | cores (win) | RSS | scans | align med | fit med | ATE vs `us050` | yaw |
|---|---|---|---|---|---|---|---|---|---|
| `us050` | 196,404 | 3.82 | 3.47 | 478 MB | 645 (53.8%) | 70.5 ms | 0.0849 | — | — |
| `us060` | 138,763 | 3.87 | 3.51 | 445 MB | 650 (54.2%) | 68.9 ms | 0.1044 | 5.6 cm | 0.13° |
| `us070` | 102,905 | 3.93 | 3.51 | 432 MB | 635 (52.9%) | 71.7 ms | 0.1279 | 2.6 cm | 0.04° |
| `us100` | 50,884 | 3.91 | 3.55 | 424 MB | 640 (53.3%) | 73.9 ms | 0.2219 | 4.2 cm | 0.07° |

**The Orin reproduces the laptop's null result.** A quarter of the map points moves
CPU by **1.4% on CURTMINI and 2.9% on bunker**, while alignment gets *slower*
(66.8 → 71.6 ms, 70.5 → 73.9 ms) and scans kept drift slightly **down**
(733 → 706, 645 → 640). The only real saving is 46 MB of RSS on CURTMINI, 54 MB on
bunker.

> **Caveat on "within the noise":** no configuration was repeated on this board, so
> the Orin's own run-to-run spread was never measured. The <3% figure is a spread
> *across different maps*, which is the effect under test — it cannot by itself
> prove the effect is absent. The laptop measured a ~5% spread and reached the same
> conclusion, and the mechanism below is architectural, so the null result is well
> supported — but not by an Orin repeatability measurement.

ATE matches the laptop closely: CURTMINI 2.3/2.9/5.9 cm against 2.2/2.7/5.9 cm,
yaw 0.03/0.07/0.19° against 0.03/0.07/0.20°.

The mechanism is the one upstream identified: **VGICP registers against the 1.0 m
voxel grid, not the point cloud.** `us050` and `us100` yield the same 50,884
voxels, so per-scan cost never touches the map's point count; a sparser map only
lowers the evidence per voxel and degrades the covariances. The fitness floor rises
with point spacing (0.065 → 0.205 CURTMINI, 0.085 → 0.222 bunker), so **scores are
not comparable across maps**.

One oddity, unexplained: bunker's ATE is non-monotonic (`us060` 5.6 cm > `us070`
2.6 cm) — and the laptop shows the same inversion at the same maps. That it
reproduces across hosts suggests these relative ATEs partly characterise the
`us050` *reference run* rather than the maps.

**Keep `gt_map_us050.pcd`.** Combining a sparser map with a channel stride is the
stride plus accuracy risk.

---

## 3. Orin vs the laptop

| | Orin | laptop | ratio |
|---|---|---|---|
| align med, CURTMINI stride 1 | 66.8 ms | 40.7 ms | 1.64× slower |
| align med, CURTMINI stride 2 | 46.9 ms | 25.9 ms | 1.81× slower |
| rate, CURTMINI stride 2 | 8.62 Hz | 9.1 Hz | 0.95× |
| rate, CURTMINI stride 4 | 9.48 Hz | 9.3 Hz | 1.02× |
| scans kept, CURTMINI stride 1 | 61.3% | 79.1% | — |

Per-alignment the Orin is consistently **1.6–1.8× slower**, though this is a
deployment-vs-deployment ratio, not a silicon one: it also spans Humble-era
gcc/PCL/Eigen against the laptop's Jazzy container toolchain, and the Orin's clocks
were not locked. Peak RSS is 17–44 MB lower on the Orin in every configuration.

Note the laptop was **also** dropping scans at stride 1 (945/1195 = 79%), so the
difference between the hosts is quantitative rather than "the laptop kept up and
the Orin did not".

---

## 4. Caveats

These are **ROS 2 Humble** numbers taken natively, because the repo's Docker base
image `osrf/ros:jazzy-desktop` is **amd64-only** and has no arm64 build — see
`docker/Dockerfile.arm64` for the `ros:jazzy-perception` replacement. Seven
host-specific workarounds are documented in `scripts/jetson_bag_test_native.sh`.
The one that could plausibly affect timing is that the bag is replayed through an
explicit `--qos-profile-overrides-path` rather than its recorded profiles. That is
most likely benign: the localizer subscribes to the cloud with
`rclcpp::SensorDataQoS()` (best-effort), so delivery is best-effort in both setups
regardless of the publisher's profile, and `alignment_time_sec` excludes transport
entirely. The rate and scans-kept columns are the ones carrying that risk.

Every configuration was run once, sequentially, on an otherwise idle board after
`fastdds shm clean`. **No repeats, so no error bars** — differences smaller than a
few percent should not be trusted. An earlier attempt was invalidated by a stale
localizer holding SHM locks for four hours; the script now reaps leftovers and
prints the subscription count before playing, so contamination is visible rather
than silent.

`nvpmodel -m 0` / `jetson_clocks` were **not** applied (they need root), so these
are lower bounds — the board was not locked to maximum clocks.
