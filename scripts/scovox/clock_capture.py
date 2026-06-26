#!/usr/bin/env python3
# scovox was launched with use_sim_time:=true, so its 0.5 Hz republish timer is
# frozen once the bag (and /clock) stops. This node runs on SYSTEM time (no
# use_sim_time), publishes a forward-advancing /clock to un-freeze scovox's timer,
# and captures the next full /scovox_node/pointcloud to /ws/output/scovox_map.npy.
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2

OUT = "/ws/output/scovox_map.npy"
START_T = 1781893647.0  # just after bag end (1781893646.159)


class Grab(Node):
    def __init__(self):
        super().__init__("clock_grab")
        self.clk = self.create_publisher(Clock, "/clock", 10)
        qos = QoSProfile(depth=1)
        qos.history = HistoryPolicy.KEEP_LAST
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        qos.durability = DurabilityPolicy.VOLATILE
        self.create_subscription(PointCloud2, "/scovox_node/pointcloud", self.cb, qos)
        self.t = START_T
        self.done = False
        self.create_timer(0.01, self.tick)  # wall-clock timer (this node uses system time)

    def tick(self):
        m = Clock()
        m.clock.sec = int(self.t)
        m.clock.nanosec = int((self.t - int(self.t)) * 1e9)
        self.clk.publish(m)
        self.t += 0.1  # 10x sim advance -> scovox's 2 s timer fires within ~0.2 s wall

    def cb(self, msg):
        if self.done:
            return
        s = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        arr = np.stack([s["x"], s["y"], s["z"]], axis=-1).astype(np.float64)
        np.save(OUT, arr)
        mn, mx = arr.min(0), arr.max(0)
        print(f"captured {arr.shape[0]} pts  frame={msg.header.frame_id}")
        print(f"bbox_min={np.round(mn,2)}  bbox_max={np.round(mx,2)}  size={np.round(mx-mn,2)}")
        self.done = True


def main():
    rclpy.init()
    n = Grab()
    waited = 0.0
    while rclpy.ok() and not n.done and waited < 20.0:
        rclpy.spin_once(n, timeout_sec=0.05)
        waited += 0.05
    if not n.done:
        print("NO MESSAGE received on /scovox_node/pointcloud within 20 s")
    rclpy.shutdown()


main()
