import numpy as np
from configuration.config_schema import ConfigSchema
from configuration.publisher_config import PublisherConfig
from rclpy.node import Node
from rclpy.publisher import Publisher
from typing import List, Optional, Dict
from formant.sdk.agent.v1 import Client
from ros2_utils.topic_type_provider import TopicTypeProvider
from geometry_msgs.msg import PoseStamped
from ros2_utils.qos import QOS_PROFILES, qos_profile_system_default
from std_msgs.msg import Bool
from ros2_utils.logger import get_logger
from message_utils.utils import (
    get_ros2_type_from_string,
    get_message_path_value,
)
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
    BatteryState,
    CompressedImage,
    Image,
    Joy,
    LaserScan,
    NavSatFix,
    PointCloud2,
)

from geometry_msgs.msg import (
    Point,
    Point32,
    Polygon,
    Pose,
    PoseStamped,
    PoseArray,
    PoseWithCovariance,
    PoseWithCovarianceStamped,
    Twist,
    TwistStamped,
    Vector3,
    Vector3Stamped,
)

from nav_msgs.msg import Odometry, OccupancyGrid, Path

ROS2_NUMERIC_TYPES = [
    "Float32",
    "Float64",
    "Int8",
    "Int16",
    "Int32",
    "Int64",
    "UInt8",
    "UInt16",
    "UInt32",
    "UInt64",
]


class GenericPublisher:
    def __init__(
        self, fclient: Client, node: Node, topic_type_provider: TopicTypeProvider
    ):
        self._logger = get_logger()
        self._fclient = fclient
        self._node = node
        self._publishers: Dict[str, List[Publisher]] = {}
        self._topic_type_provider = topic_type_provider

    def publish_command(self, formant_stream, msg: str):
        for publisher in self._publishers[formant_stream]:
            ros2_msg_type = publisher.msg_type.__name__
            if ros2_msg_type == "String":
                ros2_msg = String()
                ros2_msg.data = msg
                publisher.publish(ros2_msg)
            elif (ros2_msg_type in ROS2_NUMERIC_TYPES) and msg.isnumeric():
                self._publish_ros2_numeric(publisher, ros2_msg_type, msg)
            else:
                self._logger.warn(
                    "Unsupported ROS2 message type for command: %s" % formant_stream
                )
                continue

    def publish(self, formant_stream, msg):
        for publisher in self._publishers[formant_stream]:
            # Get the ROS2 message type as a string
            ros2_msg_type = publisher.msg_type.__name__

            # Switch on the Formant message type
            if msg.HasField("bitset"):
                # In a bitset, the value either exists (true) or doesn't (false)
                # A button press can only send one bit, so we can just use the first one
                if msg.bitset.bits[0].value:
                    msg_value = True
                else:
                    msg_value = False

                # Select the configured message type
                if ros2_msg_type == "Bool":
                    ros2_msg = Bool()
                    ros2_msg.data = msg_value
                elif ros2_msg_type == "String":
                    ros2_msg = String()
                    ros2_msg.data = str(msg_value)
                else:
                    self._logger.warn(
                        "Unsupported ROS2 message type for bitset: %s" % ros2_msg_type
                    )
                    continue

                publisher.publish(ros2_msg)

            elif msg.HasField("numeric"):
                msg_value = msg.numeric.value
                self._publish_ros2_numeric(publisher, ros2_msg_type, msg_value)

            elif msg.HasField("point"):
                print("WARNING: Point is not yet supported")
            elif msg.HasField("pose"):
                print("WARNING: Pose is not yet supported")
            elif msg.HasField("pose_with_covariance"):
                print("WARNING: Pose_with_covariance is not yet supported")
            elif msg.HasField("twist"):
                if ros2_msg_type == "Twist":
                    ros2_msg = Twist()
                    ros2_msg.linear.x = msg.twist.linear.x
                    ros2_msg.linear.y = msg.twist.linear.y
                    ros2_msg.linear.z = msg.twist.linear.z
                    ros2_msg.angular.x = msg.twist.angular.x
                    ros2_msg.angular.y = msg.twist.angular.y
                    ros2_msg.angular.z = msg.twist.angular.z
                elif ros2_msg_type == "TwistStamped":
                    ros2_msg = TwistStamped()
                    ros2_msg.twist.linear.x = msg.twist.linear.x
                    ros2_msg.twist.linear.y = msg.twist.linear.y
                    ros2_msg.twist.linear.z = msg.twist.linear.z
                    ros2_msg.twist.angular.x = msg.twist.angular.x
                    ros2_msg.twist.angular.y = msg.twist.angular.y
                    ros2_msg.twist.angular.z = msg.twist.angular.z
                elif ros2_msg_type == "Joy":
                    ros2_msg = Joy()
                    ros2_msg.axes = [
                        msg.twist.linear.x,
                        msg.twist.linear.y,
                        msg.twist.linear.z,
                        msg.twist.angular.x,
                        msg.twist.angular.y,
                        msg.twist.angular.z,
                    ]
                else:
                    self._logger.warn(
                        "Unsupported ROS2 message type for twist: %s" % ros2_msg_type
                    )
                    continue

                publisher.publish(ros2_msg)

    def _publish_ros2_numeric(
        self, publisher: Publisher, ros2_msg_type: str, msg_value
    ):
        if ros2_msg_type == "Float32":
            ros2_msg = Float32()
            ros2_msg.data = np.float32(msg_value).item()
        elif ros2_msg_type == "Float64":
            ros2_msg = Float64()
            ros2_msg.data = np.float64(msg_value).item()
        elif ros2_msg_type == "Int8":
            ros2_msg = Int8()
            ros2_msg.data = np.int8(msg_value).item()
        elif ros2_msg_type == "Int16":
            ros2_msg = Int16()
            ros2_msg.data = np.int16(msg_value).item()
        elif ros2_msg_type == "Int32":
            ros2_msg = Int32()
            ros2_msg.data = np.int32(msg_value).item()
        elif ros2_msg_type == "Int64":
            ros2_msg = Int64()
            ros2_msg.data = np.int64(msg_value).item()
        elif ros2_msg_type == "UInt8":
            ros2_msg = UInt8()
            ros2_msg.data = np.uint8(msg_value).item()
        elif ros2_msg_type == "UInt16":
            ros2_msg = UInt16()
            ros2_msg.data = np.uint16(msg_value).item()
        elif ros2_msg_type == "UInt32":
            ros2_msg = UInt32()
            ros2_msg.data = np.uint32(msg_value).item()
        elif ros2_msg_type == "UInt64":
            ros2_msg = UInt64()
            ros2_msg.data = np.uint64(msg_value).item()
        elif ros2_msg_type == "String":
            ros2_msg = String()
            ros2_msg.data = str(msg_value)
        else:
            print(
                "WARNING: Unsupported ROS2 message type for numeric: " + ros2_msg_type
            )
            return

        publisher.publish(ros2_msg)

    def setup_with_config(self, config: ConfigSchema):
        self._config = config
        self._cleanup()
        publisher_configs = config.publishers
        if publisher_configs:
            for publisher_config in publisher_configs:
                try:
                    self._setup_publisher_for_config(publisher_config)
                except ValueError as value_error:
                    self._logger.warn(value_error)
                    continue

    def _setup_publisher_for_config(self, publisher_config: PublisherConfig):
        formant_stream = publisher_config.formant_stream
        topic = publisher_config.topic
        qos_profile = QOS_PROFILES.get(
            publisher_config.qos_profile, qos_profile_system_default
        )
        ros2_type = publisher_config.message_type
        if ros2_type is None:
            ros2_type = self._topic_type_provider.get_type_for_topic(topic)
        if ros2_type is None:
            raise ValueError("No ROS2 type found for %s" % topic)
        self._logger.debug(
            "Setting up publisher %s, %s, %s"
            % (topic, publisher_config.qos_profile, ros2_type)
        )
        new_pub = self._node.create_publisher(
            msg_type=get_ros2_type_from_string(ros2_type),
            topic=topic,
            qos_profile=qos_profile,
        )

        if formant_stream not in self._publishers:
            self._publishers[formant_stream] = []

        self._publishers[formant_stream].append(new_pub)

    def _cleanup(self):
        for publisher in self._publishers.items():
            self._node.destroy_publisher(publisher[1])
        self._publishers = {}
