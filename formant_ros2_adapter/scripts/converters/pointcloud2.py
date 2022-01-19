import math
import time
import struct
import array

import lzf
from sensor_msgs.msg import PointCloud2
from formant.protos.model.v1.media_pb2 import PointCloud
import numpy as np

FLOAT_DATA_TYPE = 7
DOUBLE_DATA_TYPE = 8


def ros_pointcloud2_to_formant_pointcloud(message: PointCloud2) -> PointCloud:
    if message.is_bigendian:
        print("unsupported point cloud endianness")

    x_offset = None
    y_offset = None
    z_offset = None
    intensity_offset = None

    x_size = None
    y_size = None
    z_size = None
    intensity_size = None

    for field in message.fields:
        if field.name == "x":
            x_offset = field.offset
            x_size = 8 if field.datatype == DOUBLE_DATA_TYPE else 4
        elif field.name == "y":
            y_offset = field.offset
            y_size = 8 if field.datatype == DOUBLE_DATA_TYPE else 4
        elif field.name == "z":
            z_offset = field.offset
            z_size = 8 if field.datatype == DOUBLE_DATA_TYPE else 4
        elif field.name == "intensity" or "rgb":
            intensity_offset = field.offset
            intensity_size = 8 if field.datatype == DOUBLE_DATA_TYPE else 4

        if field.datatype not in [FLOAT_DATA_TYPE, DOUBLE_DATA_TYPE]:
            print("error: unsupported pointcloud2 datatype")

        if field.count != 1:
            print("error: unsupported pointcloud2 count")

    if (
        x_offset is None
        or y_offset is None
        or z_offset is None
        or x_size is None
        or y_size is None
        or z_size is None
    ):
        print("Error: Missing X, Y, or Z fields")
        return

    count = message.height * message.width

    xs = np.zeros((count, x_size), dtype="b")
    ys = np.zeros((count, y_size), dtype="b")
    zs = np.zeros((count, z_size), dtype="b")
    if intensity_offset is not None:
        intensities = np.zeros((count, intensity_size), dtype="b")

    if message.point_step:
        size = message.point_step
    else:
        size = 4 * x_size

    data = np.reshape(np.array(message.data, dtype="b"), (count, size),)

    xs = data[:, x_offset : x_offset + x_size]
    ys = data[:, y_offset : y_offset + y_size]
    zs = data[:, z_offset : z_offset + z_size]
    if intensity_offset is not None:
        intensities = data[:, intensity_offset : intensity_offset + intensity_size]

    if intensity_offset is None:
        points = np.concatenate((xs, ys, zs)).tobytes()
    else:
        points = np.concatenate((xs, ys, zs, intensities)).tobytes()

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
