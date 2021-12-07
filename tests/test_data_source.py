import random

import rclpy
from std_msgs.msg import (
    Bool,
    Char,
    String,
    Float32,
    Float64,
    Int8,
    Int16,
    Int32,
    Int64,
    UInt8,
    UInt16,
    UInt32,
    UInt64,
)
from sensor_msgs.msg import (
    NavSatFix,
    BatteryState,
    LaserScan,
    PointCloud2,
    CompressedImage,
    RegionOfInterest,
)
from geometry_msgs.msg import Twist, Wrench


class TestNode:
    """
    TestNode creates ROS2 topics for every supported input type
    and publishes data to the topics.

    . ROS data types
    -> Formant types:

        . Bool
        . Message with bool-valued message paths (e.g. RegionOfInterest)
        -> bitset

        . Custom message with numeric-valued message paths (e.g. Wrench)
        -> numericset

        . Char
        . String
        -> text

        . Float32
        . Float64
        . Int8
        . Int16
        . Int32
        . Int64
        . UInt8
        . UInt16
        . UInt32
        . UInt64
        -> numeric

        . NavSatFix
        -> location

        . BatteryState
        -> battery

        . LaserScan
        . PointCloud2
        -> point cloud

        . CompressedImage
        -> image
        -> video

    Localization must be tested separately.
    """

    def __init__(self):
        rclpy.init()

        self.node = rclpy.create_node("formant_ros2_adapter_test_data_source")

        with open("./test-image.jpg", "rb") as f:
            self.test_jpg_bytes = f.read()

        types = [
            Bool,
            Char,
            String,
            Float32,
            Float64,
            Int8,
            Int16,
            Int32,
            Int64,
            UInt8,
            UInt16,
            UInt32,
            UInt64,
            NavSatFix,
            BatteryState,
            CompressedImage,
            RegionOfInterest,
            Twist,
            Wrench,
        ]

        self.publishers = []
        for type_ in types:
            self.publishers.append(
                self.node.create_publisher(type_, "/" + type_.__name__, 10)
            )

        while rclpy.ok():
            self.publish_once()
            rclpy.spin_once(self.node, timeout_sec=1.0)

        self.node.destroy_node()
        rclpy.shutdown()

    def publish_once(self):
        for publisher in self.publishers:
            if publisher.msg_type == NavSatFix:
                publisher.publish(
                    NavSatFix(latitude=-22.90195278, longitude=-43.18056107)
                )
            elif publisher.msg_type in [
                Int8,
                Int16,
                Int32,
                Int64,
                UInt8,
                UInt16,
                UInt32,
                UInt64,
            ]:
                publisher.publish(publisher.msg_type(data=random.randint(0, 8)))
            elif publisher.msg_type in [Float32, Float64]:
                publisher.publish(publisher.msg_type(data=random.random()))
            elif publisher.msg_type == CompressedImage:
                publisher.publish(
                    publisher.msg_type(data=self.test_jpg_bytes, format="jpeg")
                )
            else:
                publisher.publish(publisher.msg_type())


if __name__ == "__main__":
    TestNode()
