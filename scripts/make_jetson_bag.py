#!/usr/bin/env python3
"""Cut a recorded bag down to what the localizer needs, small enough to copy to a Jetson.

    make_jetson_bag.py SRC_BAG OUT_BAG CLOUD_TOPIC IMU_TOPIC [--keep-fields x,y,z,intensity,t]

Keeps only CLOUD_TOPIC, IMU_TOPIC and /tf_static, re-packs every PointCloud2 to the
listed fields (a 48-byte Ouster point becomes 20 bytes; the node converts to
PointXYZI anyway, and the per-point time field is only read when deskew is on), and
writes an mcap with zstd chunk compression. Runs inside the container:

    docker compose exec ros python3 /ws/scripts/make_jetson_bag.py \
        /ws/bags/<recording> /ws/bags/curtmini_jetson /ouster/points /curt/imu/data
"""
import argparse
import sys

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message, serialize_message
from sensor_msgs.msg import PointCloud2, PointField

NP_DTYPE = {PointField.INT8: 'i1', PointField.UINT8: 'u1', PointField.INT16: 'i2',
            PointField.UINT16: 'u2', PointField.INT32: 'i4', PointField.UINT32: 'u4',
            PointField.FLOAT32: 'f4', PointField.FLOAT64: 'f8'}


def repack(msg, keep):
    fields = [f for f in msg.fields if f.name in keep]
    src = np.dtype({'names': [f.name for f in msg.fields],
                    'formats': [NP_DTYPE[f.datatype] for f in msg.fields],
                    'offsets': [f.offset for f in msg.fields],
                    'itemsize': msg.point_step})
    dst = np.dtype([(f.name, NP_DTYPE[f.datatype]) for f in fields])
    points = np.frombuffer(bytes(msg.data), dtype=src, count=msg.width * msg.height)
    packed = np.empty(points.shape, dtype=dst)
    for f in fields:
        packed[f.name] = points[f.name]
    out = PointCloud2()
    out.header = msg.header
    out.height, out.width = msg.height, msg.width
    out.is_bigendian, out.is_dense = msg.is_bigendian, msg.is_dense
    out.point_step = dst.itemsize
    out.row_step = dst.itemsize * msg.width
    offset = 0
    for f in fields:
        out.fields.append(PointField(name=f.name, offset=offset, datatype=f.datatype, count=1))
        offset += np.dtype(NP_DTYPE[f.datatype]).itemsize
    out.data = packed.tobytes()
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('src'); ap.add_argument('out'); ap.add_argument('cloud_topic'); ap.add_argument('imu_topic')
    ap.add_argument('--keep-fields', default='x,y,z,intensity,t')
    ap.add_argument('--duration', type=float, default=None, help='seconds from the start of the bag')
    a = ap.parse_args()
    keep = a.keep_fields.split(',')
    topics = [a.cloud_topic, a.imu_topic, '/tf_static']

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=a.src, storage_id='mcap'), rosbag2_py.ConverterOptions('cdr', 'cdr'))
    reader.set_filter(rosbag2_py.StorageFilter(topics=topics))
    writer = rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=a.out, storage_id='mcap', storage_preset_profile='zstd_small'),
                rosbag2_py.ConverterOptions('cdr', 'cdr'))
    for meta in reader.get_all_topics_and_types():
        if meta.name in topics:
            writer.create_topic(meta)

    t0, n_cloud, n_total = None, 0, 0
    while reader.has_next():
        topic, data, t = reader.read_next()
        t0 = t if t0 is None else t0
        if a.duration is not None and (t - t0) * 1e-9 > a.duration:
            break
        if topic == a.cloud_topic:
            data = serialize_message(repack(deserialize_message(data, PointCloud2), keep))
            n_cloud += 1
            if n_cloud % 500 == 0:
                print('%d clouds, %.0f s' % (n_cloud, (t - t0) * 1e-9), file=sys.stderr, flush=True)
        writer.write(topic, data, t)
        n_total += 1
    del writer
    print('wrote %s: %d clouds, %d messages, %.0f s' % (a.out, n_cloud, n_total, (t - t0) * 1e-9))


if __name__ == '__main__':
    main()
