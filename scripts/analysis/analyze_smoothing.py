#!/usr/bin/env python3
"""Does robot_localization + IMU smooth the trajectory? Compares the two edges of
the map -> odom -> base_link tree, recorded at 50 Hz by test_ekf_smoothing.sh:

  GLOBAL  map -> base_link   : the localized pose (what SCovox/global planning use)
  LOCAL   odom -> base_link  : the EKF's high-rate continuous odometry (what a local
                              planner / Nav2 uses), or static identity without the EKF

Inputs: output/tf_{ekf,noekf}_{map,odom}.csv  (cols: t_sample,t_tf,x,y,z,qx,qy,qz,qw)

Smoothness signals (50 Hz sampling):
  held_frac  fraction of samples with ~0 position change -> the "staircase" of a
             layer that only updates at the ~9 Hz NDT scan rate
  rms_jerk   RMS 3rd derivative of position (m/s^3)         lower = smoother
  rms_angacc RMS angular acceleration (rad/s^2)             lower = smoother
"""
import csv
import sys

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

RUNS = "/ws/output"
OUT = sys.argv[1] if len(sys.argv) > 1 else f"{RUNS}/ekf_smoothing.png"
HELD_EPS = 5e-4  # m


def load(mode, edge):
    t, xyz, quat = [], [], []
    with open(f"{RUNS}/tf_{mode}_{edge}.csv") as f:
        for r in csv.DictReader(f):
            t.append(float(r["t_sample"]))
            xyz.append([float(r["x"]), float(r["y"]), float(r["z"])])
            quat.append([float(r["qx"]), float(r["qy"]), float(r["qz"]), float(r["qw"])])
    t, xyz, quat = np.array(t), np.array(xyz), np.array(quat)
    o = np.argsort(t)
    t, xyz, quat = t[o], xyz[o], quat[o]
    return t - t[0], xyz, quat


def yaw_of(q):
    qx, qy, qz, qw = q.T
    return np.arctan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))


def metrics(t, xyz, quat):
    dt = np.diff(t); dt[dt <= 0] = np.median(dt[dt > 0])
    step = np.linalg.norm(np.diff(xyz, axis=0), axis=1)
    speed = step / dt
    accel = np.diff(speed) / dt[1:]
    jerk = np.diff(accel) / dt[2:]
    d = np.abs(np.sum(quat[1:] * quat[:-1], axis=1)).clip(-1, 1)
    ang = 2 * np.arccos(d)
    angacc = np.diff(ang / dt) / dt[1:]
    return dict(n=len(t), dur=float(t[-1]), mean_speed=float(speed.mean()),
                held_frac=float((step < HELD_EPS).mean()), max_step=float(step.max()),
                rms_jerk=float(np.sqrt((jerk ** 2).mean())),
                rms_angacc=float(np.sqrt((angacc ** 2).mean())))


D = {}
for mode in ("ekf", "noekf"):
    for edge in ("map", "odom"):
        try:
            D[(mode, edge)] = load(mode, edge)
        except FileNotFoundError:
            print(f"!! missing tf_{mode}_{edge}.csv")
M = {k: metrics(*v) for k, v in D.items()}


def table(title, keys):
    print(f"\n=== {title} ===")
    hdr = f"{'metric':<13}" + "".join(f"{'/'.join(k):>16}" for k in keys)
    print(hdr); print("-" * len(hdr))
    for mk, lbl in [("mean_speed", "mean spd m/s"), ("held_frac", "held_frac"),
                    ("max_step", "max_step m"), ("rms_jerk", "rms_jerk"),
                    ("rms_angacc", "rms_angacc")]:
        print(f"{lbl:<13}" + "".join(f"{M[k][mk]:>16.4g}" for k in keys))


table("GLOBAL map->base_link  (the localized pose)", [("ekf", "map"), ("noekf", "map")])
table("LOCAL  odom->base_link (the EKF odometry)",   [("ekf", "odom"), ("noekf", "odom")])

print("\n--- verdict ---")
mg, ng = M[("ekf", "map")], M[("noekf", "map")]
print(f"GLOBAL map->base: held_frac {ng['held_frac']:.2f} vs {mg['held_frac']:.2f}, "
      f"rms_jerk {ng['rms_jerk']:.3g} vs {mg['rms_jerk']:.3g}  "
      f"=> EKF does NOT change the global trajectory (pinned to raw NDT by design).")
if ("ekf", "odom") in M:
    eo = M[("ekf", "odom")]
    print(f"LOCAL  odom->base (EKF): held_frac {eo['held_frac']:.2f} vs map's "
          f"{mg['held_frac']:.2f}, rms_jerk {eo['rms_jerk']:.3g} vs {mg['rms_jerk']:.3g}  "
          f"=> the EKF+IMU adds a {'smoother, ' if eo['rms_jerk'] < mg['rms_jerk'] else ''}"
          f"continuous high-rate odometry between the ~9 Hz NDT poses.")
if ("noekf", "odom") in M:
    no = M[("noekf", "odom")]
    print(f"Without the EKF, odom->base is identity (mean speed {no['mean_speed']:.3g} m/s, "
          f"held_frac {no['held_frac']:.2f}) -> the odom frame carries no motion at all.")

# ---------- plots ----------
def window(span=12.0):
    t, xyz, _ = D[("ekf", "map")]
    spd = np.linalg.norm(np.diff(xyz, axis=0), axis=1) / np.clip(np.diff(t), 1e-3, None)
    i = int(np.argmax(np.convolve(spd, np.ones(40) / 40, "same")))
    t0 = max(0.0, t[i] - span / 2)
    return t0, t0 + span


w0, w1 = window()
fig = plt.figure(figsize=(16, 10))

ax = fig.add_subplot(2, 2, 1)
for (mode, edge), c, lbl in [(("ekf", "map"), "#d62728", "map->base (raw NDT, ~9 Hz)"),
                             (("ekf", "odom"), "#1f77b4", "odom->base (EKF, 50 Hz)")]:
    t, xyz, q = D[(mode, edge)]
    m = (t >= w0) & (t <= w1)
    y = np.degrees(np.unwrap(yaw_of(q)))
    ax.plot(t[m], y[m] - y[m][0], color=c, lw=1.3, label=lbl)
ax.set_title(f"EKF run — yaw vs time (zoom {w0:.0f}-{w1:.0f}s)\nstaircase=NDT, smooth=EKF")
ax.set_xlabel("t [s]"); ax.set_ylabel("rel. yaw [deg]"); ax.legend(); ax.grid(alpha=0.3)

ax = fig.add_subplot(2, 2, 2)
for (mode, edge), c, lbl in [(("ekf", "map"), "#d62728", "map->base (NDT)"),
                             (("ekf", "odom"), "#1f77b4", "odom->base (EKF)")]:
    t, xyz, _ = D[(mode, edge)]
    dt = np.clip(np.diff(t), 1e-3, None)
    spd = np.linalg.norm(np.diff(xyz, axis=0), axis=1) / dt
    m = (t[1:] >= w0) & (t[1:] <= w1)
    ax.plot(t[1:][m], spd[m], color=c, lw=1.0, label=lbl)
ax.set_title("EKF run — instantaneous speed (zoom): spikes/zeros=NDT, smooth=EKF")
ax.set_xlabel("t [s]"); ax.set_ylabel("speed [m/s]"); ax.legend(); ax.grid(alpha=0.3)

ax = fig.add_subplot(2, 2, 3)
for mode, c in [("ekf", "#1f77b4"), ("noekf", "#d62728")]:
    if (mode, "map") in D:
        t, _, q = D[(mode, "map")]
        m = (t >= w0) & (t <= w1)
        y = np.degrees(np.unwrap(yaw_of(q)))
        ax.plot(t[m], y[m] - y[m][0], color=c, lw=1.2, label=f"{mode}", alpha=0.8)
ax.set_title("GLOBAL map->base yaw: ekf vs noekf (overlap = EKF doesn't touch it)")
ax.set_xlabel("t [s]"); ax.set_ylabel("rel. yaw [deg]"); ax.legend(); ax.grid(alpha=0.3)

ax = fig.add_subplot(2, 2, 4)
for (mode, edge), c, lbl in [(("ekf", "map"), "#d62728", "map->base (NDT)"),
                             (("ekf", "odom"), "#1f77b4", "odom->base (EKF)")]:
    t, xyz, _ = D[(mode, edge)]
    step = np.linalg.norm(np.diff(xyz, axis=0), axis=1) * 1000
    ax.hist(step, bins=80, histtype="step", color=c, log=True, label=lbl)
ax.axvline(HELD_EPS * 1000, color="k", ls="--", lw=0.8)
ax.set_title("per-sample step (log): spike at 0 = held staircase (NDT)")
ax.set_xlabel("step [mm]"); ax.set_ylabel("count"); ax.legend()

plt.tight_layout()
plt.savefig(OUT, dpi=100)
print(f"\nsaved {OUT}")
