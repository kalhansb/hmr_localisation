#!/usr/bin/env python3
"""Write an 'active' multi-robot roster with robot2 seeded for a time offset.

robot2 is the SAME platform replayed `--start-offset OFFSET` seconds ahead on the
route, so at its first scan it is already OFFSET seconds along the trajectory --
NOT at the map origin. Seeding it at identity would force a cold global re-lock.
Instead we read the validated single-robot trajectory and use the pose at +OFFSET
as robot2's initial_pose. Swap in a real second bag later and just drop this step.

Usage: seed_robots.py <template_yaml> <trajectory_csv> <offset_s> <out_yaml>
"""
import csv
import sys

import yaml

template, csv_path, offset_s, out = sys.argv[1], sys.argv[2], float(sys.argv[3]), sys.argv[4]

rows = list(csv.reader(open(csv_path)))[1:]
# columns: stamp,x,y,z,qx,qy,qz,qw ; drop the identity seed row (stamp ~0)
traj = [(float(r[0]), [float(v) for v in r[1:8]]) for r in rows if float(r[0]) > 1.0]
if not traj:
    sys.exit(f'no usable poses in {csv_path}')

t0 = traj[0][0]
seed = next((pose for t, pose in traj if t - t0 >= offset_s), traj[-1][1])

cfg = yaml.safe_load(open(template))
for rb in cfg['robots']:
    if rb['name'] == 'robot2':
        rb['initial_pose'] = seed

with open(out, 'w') as f:
    yaml.safe_dump(cfg, f, default_flow_style=None, sort_keys=False)

print(f'robot2 seeded at +{offset_s:.0f}s -> '
      f'x={seed[0]:.2f} y={seed[1]:.2f} z={seed[2]:.2f}  (wrote {out})')
