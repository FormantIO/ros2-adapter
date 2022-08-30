import math
import struct

import lzf
from sensor_msgs.msg import LaserScan
from formant.protos.model.v1.media_pb2 import PointCloud
import numpy as np


def ros_laserscan_to_formant_pointcloud(message: LaserScan) -> PointCloud:
    height = 1
    width = len(message.ranges)
    count = height * width

    xs = np.zeros((count,), dtype="float32")
    ys = np.zeros((count,), dtype="float32")
    zs = np.zeros((count,), dtype="float32")

    for i in range(len(message.ranges)):
        xf = 0.0
        yf = 0.0
        zf = 0.0

        r = message.ranges[i]
        a = message.angle_min + message.angle_increment * i
        if a > message.angle_max:
            break
        if r > message.range_min and r < message.range_max:
            xf = r * math.cos(a)
            yf = r * math.sin(a)

        xs[i] = xf
        ys[i] = yf
        zs[i] = zf

    points = np.concatenate((xs, ys, zs)).tobytes()

    buffer = (
        """VERSION 0.7
FIELDS x y z rgb
SIZE 4 4 4 4
TYPE F F F F
WIDTH %s
HEIGHT 1
DATA binary_compressed
"""
        % count
    )
    compressed = lzf.compress(points, 2 * len(points))
    raw = (
        buffer.encode("utf-8")
        + struct.pack("<I", len(compressed))
        + struct.pack("<I", len(points))
        + compressed
    )

    return PointCloud(raw=raw)
