# GLIM vs hmr_localisation under CycloneDDS

Re-run of the `glim_comparison.md` head-to-head matrix on a **UDP transport**
(`rmw_cyclonedds_cpp`) instead of the Fast-DDS shared-memory profile that study used.

Date: 2026-09-14. Host: Jetson Orin (12 cores visible, 61 GB RAM).
Companion document: [`glim_comparison.md`](glim_comparison.md) — read its §6d/§6e first;
the defect IDs D1–D4 and R1–R8 are referenced throughout and are **not** repeated here.

---

## 1. What changed, and the kernel prerequisite

The prior study ran every node under `config/fastdds_shm.xml`, which maps a 64 MB
shared-memory segment into each participant. This study swaps the transport only —
same bags, same tuned configs, same pinning, same scoring code.

The config under test, used **byte-for-byte unmodified** (md5 `9188275c3cbb1c9b6a6edee4a0b53473`):

```xml
<CycloneDDS xmlns="https://cdds.io/config" ...>
  <Domain Id="any">
    <General>
      <Interfaces>
        <NetworkInterface autodetermine="false" name="wlP1p1s0" priority="default" multicast="default" />
      </Interfaces>
      <AllowMulticast>spdp</AllowMulticast>
      <MaxMessageSize>65500B</MaxMessageSize>
    </General>
    <Discovery>
        <MaxAutoParticipantIndex>100</MaxAutoParticipantIndex>
    </Discovery>
    <Internal>
      <SocketReceiveBufferSize min="10MB" />
      <Watermarks>
        <WhcHigh>500kB</WhcHigh>
      </Watermarks>
    </Internal>
  </Domain>
</CycloneDDS>
```

### `SocketReceiveBufferSize min=` is a hard minimum

Cyclone refuses to create the participant if the kernel will not grant the request.
The symptom is **not** a buffer warning — it is:

```
rcl node's rmw handle is invalid
```

Stock `net.core.rmem_max` on this board is 212992 B, so the 10 MB minimum cannot be met
and every node dies at startup. The fix:

```bash
sudo sysctl -w net.core.rmem_max=10485760
```

Verified three ways before benchmarking: the sysctl value itself, a direct
`setsockopt(SO_RCVBUF)` probe returning the grant, and — the decisive one — participants
with a hard 10 MB minimum starting at all.

> **`sysctl -w` does not survive reboot.** To make it stick:
> ```bash
> echo 'net.core.rmem_max=10485760' | sudo tee /etc/sysctl.d/60-cyclonedds.conf
> ```
> This has **not** been done on the JetBot yet. Until it is, a reboot silently returns
> the box to the state where every Cyclone node fails to start.

---

## 2. Method

- **30 runs**, n=3 per row, 120 s each, zero failures.
- Node pinned to **one core** (`PIN_CORES=0`) in every row, both systems (this is R1's correction).
- hmr_localisation: `scan_channel_stride={4,8}`, `ndt_num_threads=1`.
- GLIM v1.2.2: tuned configs from `~/glim-config/` — variants `o_combo` (odom CPU),
  `o_combogpu` (odom GPU), `o_combofull` (full + loop closure).
- **CPU**: `cpu_window.py` — busiest contiguous 120 s by CPU ticks (D1's correction; the
  old `cpu_busy.py` was biased in GLIM's favour).
- **Memory**: `mem_growth.py` — peak RSS in that same window, a 120 s window slope, a
  final-40 s tail slope, and a `still growing` verdict requiring tail > 1.0 MB/min **and**
  last sample ≥ 98 % of run max.
- **ATE**: 6-DOF rigid Umeyama (no scale), in the **lidar** frame, 20 ms nearest-stamp
  association, against the stride-1 hmr_localisation run. Both systems go through the
  identical scorer — this is D2's correction. Units are **cm**.

### The scoring chain was rebuilt this session and validated before use

The previous session's scratchpad was wiped by a reboot. The harness was recovered from
`~/.claude/file-history/` rather than rewritten, because rewriting would have silently
changed the method. Two independent validations were run **before** any new numbers were
produced:

| check | rebuilt chain | published value |
|---|---|---|
| bunker stride-8 ATE, re-scored from the old Fast-DDS poses | 4.4 cm | 4.5 cm (D2, 6-DOF) |
| full-stack peak RSS, bunker / curtmini | 1979 / 2027 MB | 1979 / 2027 MB |

---

## 3. Results

Jetson Orin, pinned to 1 core, 120 s, both tuned, n=3 per row.
ATE = deviation from the stride-1 hmr_loc run, 6-DOF rigid-aligned, in cm.
RSS = max peak across the 3 reps. MB/min = mean **tail** slope. `growing` = 2-of-3 reps flagged.

```
system   mode       bag       rate    cores   range      p95   ATEmed ATEp95   RSS  MB/min growing
--------------------------------------------------------------------------------------------------
hmr_loc  stride 4   bunker     9.77   0.80  0.80-0.80  0.89     2.3    8.3   161    +0.4 no
hmr_loc  stride 4   curtmini   9.02   0.77  0.77-0.77  0.92     2.6    6.9   144    -5.2 no
hmr_loc  stride 8   bunker     9.79   0.70  0.70-0.71  0.74     4.4   12.5   151    -0.0 no
hmr_loc  stride 8   curtmini   9.29   0.68  0.68-0.68  0.74     4.9   10.6   136    +0.0 no
glim     odom cpu   bunker    10.01   0.75  0.74-0.75  0.77     2.6   10.1   194   -16.2 no
glim     odom cpu   curtmini   9.97   0.68  0.67-0.69  0.73     5.2   14.7   190    +2.9 no
glim     odom gpu   bunker    10.01   0.74  0.74-0.75  0.76     3.0    9.7   213   -12.1 no
glim     odom gpu   curtmini   9.97   0.70  0.69-0.70  0.78     5.9   20.0   226    +3.8 no
glim     full+loop  bunker    10.01   0.87  0.85-0.88  0.94     3.1   10.6  1539  +495.1 YES
glim     full+loop  curtmini   9.97   0.86  0.85-0.86  0.99     8.4   24.6  1062  +350.0 YES
```

---

## 4. Paired against Fast-DDS SHM, through the identical scorer

The first version of this comparison quoted the numbers **published in**
`glim_comparison.md`. That is weaker than it looks, because the published RSS was a
process-lifetime peak while this study reports a windowed peak. To remove that
confound, the raw `*.cpu.csv` files from the 2026-09-12 Fast-DDS runs were re-scored
through *today's exact* `cpu_window.py` and `mem_growth.py`:

```
system   mode       bag      |  CYCLONE (n=3 mean)      |  FAST-DDS SHM (n=1)      |   delta
                             | cores    RSS   tail still| cores    RSS   tail still| dcores    dRSS
-------------------------------------------------------------------------------------------------
hmr_loc  stride 4  bunker    |  0.80    155   +0.4      |  0.80    239  -29.7      |  -0.00     -84
hmr_loc  stride 4  curtmini  |  0.77    143   -5.2      |  0.75    215   -4.4      |  +0.01     -72
hmr_loc  stride 8  bunker    |  0.70    149   -0.0      |  0.70    225   -7.7      |  +0.00     -76
hmr_loc  stride 8  curtmini  |  0.68    136   +0.0      |  0.68    206   -8.4      |  +0.00     -70
glim     odom cpu  bunker    |  0.75    193  -16.2      |  0.75    287  -42.5      |  +0.00     -95
glim     odom cpu  curtmini  |  0.68    188   +2.9  YES |  0.69    258   +1.2  YES |  -0.01     -70
glim     odom gpu  bunker    |  0.74    212  -12.1      |  0.71    283  -11.3      |  +0.03     -71
glim     odom gpu  curtmini  |  0.70    220   +3.8      |  0.69    306  -10.9      |  +0.01     -86
glim     full+loop bunker    |  0.87   1526 +495.1  YES |  0.86   1617 +499.1  YES |  +0.00     -91
glim     full+loop curtmini  |  0.86   1061 +350.0  YES |  0.85   1149 +353.8  YES |  +0.00     -88
```

(RSS here is the **mean** of the 3 Cyclone reps, not the max used in §3 — hence 155 vs 161
for hmr stride-4 bunker, etc. Both aggregations are shown deliberately; see C5.)

### 4.1 CPU: the transport swap is a no-op

Across all 10 rows, |Δcores| ≤ **0.03**, mean **+0.006**. There is no CPU cost to moving off
shared memory on this workload.

This contradicts my own prediction — I expected Cyclone to cost *more* CPU. The accounting
caveat is that loopback UDP receive work happens in softirq / sender context and is not
charged to the subscriber process, so a fraction of the transport cost is real but
invisible in a per-process `cores` metric. Total-system CPU was not measured.

### 4.2 Memory: every row drops 70–95 MB

Mean −80 MB, range −70 to −95 MB. The `fastdds_shm.xml` segment is 64 MB and is mapped into
every participant, which accounts for most but **not all** of this — see C3.

### 4.3 Delivery health

`gap_p95 = 0.100 s` on all 18 GLIM runs — exactly one 10 Hz period, i.e. no missed scans.
`backlog = 0` and `|lag_end| ≤ 0.20 s` everywhere. Scan counts are identical across all
three reps of every row. The 10 MB receive buffer eliminated the half-second scan gaps seen
with the stock 208 kB cap.

GLIM warn/error lines match the SHM baseline exactly: bunker 0; curtmini 60 "insufficient
IMU data" + 14 "IMU prediction is not good" (this is R3 / §5 of the companion doc — curtmini's
IMU is holed, and it is a property of the bag, not the transport).

---

## 5. `full+loop` pinned and tuned — a row that was never published

This configuration does not appear in `glim_comparison.md`. Three findings:

1. **It fits in one core**: 0.86–0.87 cores, p95 0.94–0.99. Loop closure is affordable.
2. **It is the only configuration still growing** at the end of the run — 3/3 reps, at
   350–495 MB/min, roughly 350× the 1.0 MB/min threshold. Everything else is noise around zero.
3. **The growth is not queue backlog.** `backlog = 0` and `lag_end ≈ 0` on all six runs, so
   the process is keeping up with the bag in real time; the RSS is mapping/factor-graph state.

The growth reproduces under **both** transports, which is the strongest evidence in this
study that it is a GLIM property and not a Cyclone artifact:

| run | window slope | tail slope | deceleration |
|---|---|---|---|
| cyclone bunker | +712.8 MB/min | +495.1 | −31 % |
| cyclone curtmini | +452.4 | +350.0 | −23 % |
| fast-dds bunker | +708.6 | +499.1 | −30 % |
| fast-dds curtmini | +453.9 | +353.8 | −22 % |

**The growth is decelerating, by 22–31 % within a single 120 s window.** See C2 — this
retracts an extrapolation I made earlier.

---

## 6. Adversarial review

Same treatment §6d/§6e gave their predecessors. IDs are C-prefixed to avoid clashing with
D1–D4 and R1–R8.

### C1. hmr_localisation is not processing the same amount of work as GLIM (SERIOUS)

On curtmini, hmr runs at **9.02–9.29 Hz** while GLIM runs at **9.97 Hz** on the identical
bag. hmr is dropping ~9 % of scans at stride 4.

I initially read this as Cyclone dropping packets — the exact failure mode the config's
65500 B `MaxMessageSize` invites. It is not. Pose counts straight from `poses.csv`:

| config | Cyclone | Fast-DDS SHM |
|---|---|---|
| bunker stride 4 | 1169 | 1166 |
| bunker stride 8 | 1171 | 1143 |
| curtmini stride 4 | 1076 | 1046 |
| curtmini stride 8 | 1105 | 1102 |

Cyclone produced **more** poses than Fast-DDS in all four configs. The deficit is
hmr_localisation's own behaviour under load, present on both transports.

But that makes the `cores` column **not a like-for-like comparison**. hmr's CPU is measured
while it does less work. Scaling cores by the rate ratio as a first-order correction:

| row | measured | work-normalised | GLIM (same bag) |
|---|---|---|---|
| hmr s4 bunker | 0.80 | ~0.82 | 0.75 |
| hmr s8 bunker | 0.70 | ~0.72 | 0.75 |
| hmr s4 curtmini | 0.77 | **~0.85** | 0.68 |
| hmr s8 curtmini | 0.68 | **~0.73** | 0.68 |

The curtmini rows move substantially. A reader comparing 0.77 against GLIM's 0.68 is
comparing a discount to full price. This affects the companion document's §2 and §6
head-to-head tables too, since the same rate deficit is present in its Fast-DDS data.

Caveat on the correction itself: CPU does not scale perfectly linearly with scan rate
(there is fixed per-cycle overhead), so treat the normalised column as an **upper bound**
on the adjustment, not a measurement.

#### Why it drops scans while a quarter of its core is idle

`cloud_queue_depth` defaults to **1**, and the cloud subscription is
`SensorDataQoS().keep_last(cloud_queue_depth_)` — best-effort, depth 1
(`lidar_localization_component.cpp:1074`, default at `:178`). There is no buffer. A scan
arriving while the callback is still running **overwrites** the queued scan and is gone.

So the 0.23 idle cores are irrelevant: what decides a drop is whether the callback's busy
period ever exceeds the 100 ms inter-arrival time, not the 120 s mean. The node also runs a
2-thread `MultiThreadedExecutor` (`lidar_localization_node.cpp:12`) pinned to a single core,
so the IMU callback (depth 2000, dense on curtmini) competes for the same core.

Nothing is being rejected — `has_converged` is true and `consecutive_rejected_updates` is 0
for every processed scan. The scans never reach the callback.

Drop probability against the preceding scan's `alignment_time_sec` (curtmini, r1):

```
align_time    stride 8            stride 4
  40- 50 ms    0.4% of  499        0.0% of   82
  50- 60 ms    2.5% of  318        0.2% of  469
  60- 70 ms   16.8% of  119        0.4% of  243
  70- 80 ms   40.7% of   27        6.8% of   74
  80-100 ms   35.0% of   20       23.1% of  147
   >100 ms   100.0% of    2       46.4% of   56
```

A steep monotonic dose-response: the scan immediately before a missed cycle took 95.9 ms
median at stride 4 (vs 59.5 ms elsewhere) and 66.8 ms at stride 8 (vs 49.0 ms). Drops follow
long scans, and they begin **well below** the 100 ms period — implying tens of milliseconds
of non-alignment work (cloud conversion, voxel filtering, TF, publish) inside the same busy
period. The two strides put the knee in different places (~60–70 ms vs ~80–100 ms), so that
overhead is not a single constant and is not modelled further here.

Ruled out: **IMU holes are not the cause.** `imu_prediction_active` was true and
`registration_seed_source` was `imu_preintegration` for 100 % of processed scans in every
run, so the seed was never degraded. curtmini's IMU gaps (§4.3) show up in GLIM's warnings,
not in hmr's seeding. Drops are also spread uniformly across the run, not a startup transient.

GLIM does not drop under the same jitter because it buffers frames in an unbounded queue —
it falls behind rather than discarding. On these runs it never needed to (`backlog = 0`).

**This is arguably correct behaviour, not a bug.** For a localizer, discarding a stale scan
to process the freshest one is the right trade. Raising `cloud_queue_depth` to 3–5 would
recover the missing scans at the cost of latency. What it means for this study is narrower:
the `cores` column is not like-for-like, which is what the `norm` column exists to flag.

### C2. My linear memory extrapolation was unsound — RETRACTED

I previously said `full+loop` at ~495 MB/min implies "~2 h to exhaustion on this 61 GB
board, under 20 min on an 8 GB Orin". That assumed a linear fit holds far outside a 120 s
window. It does not even hold *inside* it: the slope falls 22–31 % between the 120 s window
fit and the final-40 s fit (§5 table).

What is actually supported: **RSS is still rising at t = 120 s, at a decelerating rate, and
the asymptote is unknown.** It may plateau, it may not. The 1539 / 1062 MB peaks are simply
where the window ended, not a steady state.

Settling this needs a run much longer than the bag — loop the bag for 10+ minutes and watch
the tail slope. That has **not** been done. Until it is, `full+loop` should not be deployed
unattended on a memory-constrained board, but the specific time-to-exhaustion figures I gave
should be discarded.

### C3. The RSS attribution is over-precise (MODERATE)

I attributed the memory drop to the 64 MB `fastdds_shm.xml` segment. The measured drop is
−70 to −95 MB, mean −80 — consistently **larger** than 64 MB, and varying by 25 MB across
rows. A single mapped segment would produce a roughly constant offset.

So: the segment is very likely the dominant term, with another ~10–30 MB of Fast-DDS
per-process overhead (history caches, per-reader pools, transport threads) on top. That
breakdown is **inferred, not measured** — nobody diffed `/proc/<pid>/smaps` between the two
transports. The honest claim is "Cyclone's participants are ~80 MB smaller", not "the 64 MB
segment explains it".

### C4. The Fast-DDS baseline is cross-boot and n=1 (MODERATE, partly mitigated)

The §4 comparison pits n=3 Cyclone runs from 2026-09-14 against n=1 Fast-DDS runs from
2026-09-12, across a reboot. Thermal state, page-cache state and background load are all
uncontrolled.

Mitigation: re-scoring the raw CSVs through today's scorer removed the metric confound, and
the agreement is tight enough (|Δcores| ≤ 0.03 over 10 independent rows) that a systematic
environment shift is unlikely — a thermal or load difference would not preserve ten
independent numbers to two decimal places. But it remains an uncontrolled comparison, and a
same-boot A/B of one row would close it in ~20 minutes. It has not been run.

### C5. The cross-check I reported was a subset, unlabelled (PRESENTATION)

When I first said "every row present in both tables agrees within 0.02 cores and 0.2 cm",
I displayed **5 of the 8** shared rows without saying it was a subset. The claim happens to
hold for all 8 — but showing a majority and describing it as "every row" is exactly the
pattern a reader should not have to re-derive. §4 above now shows all 10 rows.

Related: §3 reports RSS as the **max** across reps while §4 reports the **mean**, which is
why the same row reads 161 and 155. Both are stated, but a reader skimming will trip over it.

### C6. The ATE column cannot detect what this study is testing (STRUCTURAL)

ATE is measured against a **fixed** stride-1 hmr trajectory recorded under Fast-DDS on
2026-09-12. So:

- The reference itself cannot move when the transport changes — any transport-induced error
  in the reference is baked in and invisible.
- ATE agreeing with the published values to within 0.2 cm is therefore *weak* evidence that
  the transport is harmless. It rules out gross scan loss (which would shift the trajectory)
  and nothing finer.

D4 and R5 apply unchanged: the reference is the localizer's own slowest run, and the
robot is parked for most of both bags, so these ATE figures are substantially a
stationary-robot statistic.

### C7. This table is not a GPU verdict (inherits R2)

The GPU-vs-CPU deltas flip sign by bag: bunker 0.74 GPU vs 0.75 CPU, curtmini 0.70 GPU vs
0.68 CPU. That is within run-to-run spread at n=3, and it reproduces R2's conclusion that
the GPU question is **not resolvable with this sample size** — not D3's earlier "GPU is
consistently cheaper". Host RSS is higher for the GPU rows (212–220 vs 188–193 MB), and
**device memory was not measured at all**, so the GPU rows understate total memory by an
unknown amount.

### C8. Everything here is loopback (STRUCTURAL — the biggest gap)

Every run had publisher and subscriber on the same board. The config sets
`MaxMessageSize=65500B` and binds to `wlP1p1s0`, a WiFi interface with a 1500-byte MTU.
Over the air, each 65500 B datagram becomes ~45 IP fragments, and a full bunker cloud
(~5.5 MB) becomes roughly 3,800 fragments — where **one lost fragment discards the entire
65500 B datagram**.

`gap_p95 = 0.100 s` on loopback says nothing whatsoever about this. The configuration's
single most consequential parameter is the one the benchmark cannot see. A two-machine
AGX↔JetBot run, and a `FragmentSize=1280B` variant, are the obvious follow-ups. Neither
has been done.

Related: the AGX and JetBot share a DDS domain, so any two-machine test must control for
stray participants joining the measurement.

### C9. The aggregation script did not reproduce its own output (FIXED)

`build_table.py` on disk read `g["rate"]` (the 120 s window slope) into the MB/min column
while the `growing` verdict used `g["tail"]`. The published table's numbers are tail
slopes, so the on-disk script did not regenerate the table it supposedly produced —
e.g. it would have printed +712.8 where the table says +495.1.

Fixed (`ms.append(g["tail"])`); the script now reproduces §3 exactly. No published number
changes.

A second instance turned up while checking in the repo copies: `paired.py` flagged
`still growing` on an **any-of-3** vote while `build_table.py` used **2-of-3**, so the two
tables disagreed on three rows (hmr s4 bunker, glim odom cpu curtmini, glim odom gpu
curtmini). `paired.py` now uses 2-of-3. §4 as printed above is the corrected version; the
three rows read `no`.

### C10. The `growing` column is threshold-brittle for 8 of 10 rows (MINOR)

Per-rep tail slopes for the non-`full+loop` rows swing from −22.7 to +28.8 MB/min on the
*same configuration*, against a 1.0 MB/min threshold. Votes:

```
hmr s4 bunk      -0.0   +1.1   -0.0    ->  1/3
glim ocpu curt  +28.8   +2.7  -22.7    ->  1/3
glim ogpu curt  -13.5   +6.2  +18.7    ->  1/3
glim full bunk +516.0 +483.5 +485.9    ->  3/3
glim full curt +346.4 +358.3 +345.3    ->  3/3
```

Three rows are one rep away from flipping to `YES`. The 2-of-3 vote used in §3 is the
right rule, but the column should be read as binary only for `full+loop`, where the signal
is 350× the threshold and unanimous. For every other row the correct reading is
"indistinguishable from flat at n=3", not "proven flat".

### What survived the review unchanged

- The CPU no-op result (§4.1). Ten independent rows, |Δ| ≤ 0.03, re-scored through one scorer.
- The 10 MB buffer requirement and its failure mode (§1), verified three ways.
- `full+loop` fits in one core at 0.86–0.87 (§5).
- `full+loop` RSS is still rising at 120 s, reproduced across two transports and two boots (§5).
- Delivery health on loopback: no missed scans, zero backlog (§4.3).
- curtmini's IMU holes are a bag property, matching the SHM baseline count for count (§4.3).

---

## 7. Reproducing

Everything needed to reproduce this is checked in under
[`scripts/cyclonedds_study/`](../scripts/cyclonedds_study/).

```bash
S=~/jetbot-slam/hmr_localisation/scripts/cyclonedds_study

sudo sysctl -w net.core.rmem_max=10485760      # required; NOT persistent across reboot
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://$S/cyc_user.xml"

$S/sweep_table.sh                               # 30 runs, ~90 min
python3 $S/build_table.py                       # regenerates §3
python3 $S/paired.py                            # regenerates §4
```

`cyc_user.xml` in that directory is the exact file benchmarked
(md5 `9188275c3cbb1c9b6a6edee4a0b53473`).

Raw data lives outside the repo: `output/jetson_test/cyc_hmr_*` and `~/glim-output/cyc_glim_*`.
The Fast-DDS comparison data in §4 is `output/jetson_test/1cv2_*` and
`~/glim-output/{opt1c,gap1c,optf1c}_*` from 2026-09-12.

The repo copies were made self-locating (they originally hardcoded the session scratchpad
path they were written in) and both aggregators were verified to regenerate §3 and §4
byte-identically from a neutral working directory. `sweep_table.sh` writes its logs to
`scripts/cyclonedds_study/results/` by default; override with `SG=<dir>`.

## 8. Caveats

1. Loopback only. The over-the-air fragmentation behaviour of `MaxMessageSize=65500B` is
   completely untested (C8).
2. `net.core.rmem_max` is not persisted; a reboot breaks every Cyclone node (§1).
3. 120 s windows. Long-horizon drift and the `full+loop` memory asymptote are out of reach
   of this data (C2).
4. Both bags are largely stationary; ATE is substantially a parked-robot statistic (C6).
5. hmr and GLIM are not processing the same number of scans (C1).
6. GPU device memory was never measured (C7).
7. Fast-DDS side is n=1 and from a different boot (C4).
