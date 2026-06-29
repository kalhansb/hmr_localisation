#!/usr/bin/env python3
"""Capture one /ouster/points scan and characterize it, to find the source of
SCovox's volumetric over-fill. Reports: organized dims, point count, range
histogram (sensor-frame), no-return structure, per-ring behavior, and how many
points sit near max ranges (a no-return-at-max-range placeholder would show as a
hard spike and would fill the swept volume when integrated as occupied)."""
import sys
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2


class Cap(Node):
    def __init__(self):
        super().__init__("raw_scan_stats")
        qos = QoSProfile(depth=5)
        qos.history = HistoryPolicy.KEEP_LAST
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        qos.durability = DurabilityPolicy.VOLATILE
        self.create_subscription(PointCloud2, "/ouster/points", self.cb, qos)
        self.done = False

    def cb(self, msg):
        if self.done:
            return
        fields = [f.name for f in msg.fields]
        print(f"frame_id={msg.header.frame_id} height={msg.height} width={msg.width} "
              f"is_dense={msg.is_dense} point_step={msg.point_step}")
        print(f"fields={fields}")
        # read xyz WITHOUT skipping nans so we can see no-return structure
        s = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=False)
        x = np.asarray(s["x"], float); y = np.asarray(s["y"], float); z = np.asarray(s["z"], float)
        n = x.size
        finite = np.isfinite(x) & np.isfinite(y) & np.isfinite(z)
        rng = np.sqrt(x*x + y*y + z*z)
        zero = (np.abs(x) < 1e-6) & (np.abs(y) < 1e-6) & (np.abs(z) < 1e-6)
        print(f"\ntotal points         = {n:,}")
        print(f"non-finite (NaN/inf) = {(~finite).sum():,}  ({(~finite).mean()*100:.1f}%)")
        print(f"exact-zero points    = {zero.sum():,}  ({zero.mean()*100:.1f}%)  <- driver no-return as (0,0,0)?")
        good = finite & ~zero
        r = rng[good]
        print(f"valid (finite,nonzero) = {good.sum():,}  ({good.mean()*100:.1f}%)")
        if r.size:
            print(f"\nrange [m] over valid pts: min={r.min():.2f} p50={np.median(r):.2f} "
                  f"mean={r.mean():.2f} p95={np.percentile(r,95):.2f} max={r.max():.2f}")
            # histogram to spot a no-return-at-max spike
            hist, edges = np.histogram(r, bins=[0,1,2,3,5,8,10,12,15,20,30,50,1e9])
            print("range histogram (valid pts):")
            for h, lo, hi in zip(hist, edges[:-1], edges[1:]):
                print(f"  [{lo:>5.0f},{hi:>5.0f}) m : {h:>9,}  ({h/r.size*100:5.1f}%)")
            # how concentrated is the far shell? fraction within 0.5 m of the max
            for cap in (10.0, 12.0, 15.0):
                near = ((r > cap-0.5) & (r <= cap+0.5)).sum()
                print(f"  pts within +-0.5 m of {cap:.0f} m = {near:,} ({near/r.size*100:.1f}%)")
            zg = z[good]
            print(f"\n z [m] (sensor frame) of valid pts: min={zg.min():.2f} p50={np.median(zg):.2f} "
                  f"max={zg.max():.2f}  (>2m up: {(zg>2).mean()*100:.1f}%, < -2m: {(zg<-2).mean()*100:.1f}%)")
            # SINGLE-SCAN column thickness (sensor frame, <=15m to match scovox max_range).
            # If a single scan is already thick -> volumetric scene; if thin -> the
            # accumulated fill is from across-scan accumulation/pose-smear.
            for cap in (15.0, 10.0):
                m = good & (rng <= cap)
                P = np.stack([x[m], y[m], z[m]], 1)
                c = np.unique(np.floor(P/0.2 + 0.5).astype(np.int64), axis=0)
                o = np.lexsort((c[:,2], c[:,1], c[:,0])); c = c[o]
                xy = c[:,:2]; same = np.all(xy[1:]==xy[:-1],axis=1); bnd = np.where(~same)[0]+1
                zc = np.array([len(g) for g in np.split(c[:,2], bnd)])
                print(f" SINGLE-SCAN (<= {cap:.0f} m): cells={len(c):,} cols={len(zc):,} "
                      f"z/col median={np.median(zc):.0f} mean={zc.mean():.1f} %>=5={(zc>=5).mean()*100:.0f}%")
        self.done = True


def main():
    rclpy.init()
    n = Cap()
    waited = 0.0
    while rclpy.ok() and not n.done and waited < 60:
        rclpy.spin_once(n, timeout_sec=0.1); waited += 0.1
    if not n.done:
        print("no /ouster/points received")
    rclpy.shutdown()


main()
