#!/usr/bin/env python3
"""Read the first relevant messages from the mcap bag and print frame info,
without ROS/DDS. Uses the pure-python rosbags reader."""
import sys
from pathlib import Path
from rosbags.highlevel import AnyReader

bagdir = Path(sys.argv[1])
want = {"/tf_static", "/tf", "/ouster/points", "/imu/data", "/fix"}
seen_static = False
seen = {}

with AnyReader([bagdir]) as reader:
    print("=== connections ===")
    for c in reader.connections:
        print(f"  {c.topic:45s} {c.msgtype}")
    print("\n=== first messages ===")
    conns = [c for c in reader.connections if c.topic in want]
    for con, ts, raw in reader.messages(connections=conns):
        topic = con.topic
        msg = reader.deserialize(raw, con.msgtype)
        if topic == "/tf_static" and not seen_static:
            seen_static = True
            print("\n--- /tf_static transforms (extrinsics) ---")
            for t in msg.transforms:
                tr, ro = t.transform.translation, t.transform.rotation
                print(f"  {t.header.frame_id}  ->  {t.child_frame_id}")
                print(f"     xyz=({tr.x:+.4f}, {tr.y:+.4f}, {tr.z:+.4f})  "
                      f"quat=({ro.x:+.4f}, {ro.y:+.4f}, {ro.z:+.4f}, {ro.w:+.4f})")
        elif topic == "/tf" and "/tf" not in seen:
            seen["/tf"] = True
            print("\n--- /tf (first dynamic msg) parent->child ---")
            for t in msg.transforms:
                print(f"  {t.header.frame_id} -> {t.child_frame_id}")
        elif topic == "/ouster/points" and topic not in seen:
            seen[topic] = True
            print(f"\n--- /ouster/points ---  frame_id='{msg.header.frame_id}' "
                  f"h={msg.height} w={msg.width} fields={[f.name for f in msg.fields]}")
        elif topic == "/imu/data" and topic not in seen:
            seen[topic] = True
            print(f"--- /imu/data ---  frame_id='{msg.header.frame_id}'")
        elif topic == "/fix" and topic not in seen:
            seen[topic] = True
            print(f"--- /fix ---  frame_id='{msg.header.frame_id}' "
                  f"lat={msg.latitude:.6f} lon={msg.longitude:.6f} alt={msg.altitude:.2f}")
        if seen_static and want.issubset(set(seen) | {"/tf_static"}):
            break
print("\nDONE")
