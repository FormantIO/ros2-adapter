from nav_msgs.msg import Odometry, OccupancyGrid, Path
import threading
from typing import List, Dict, Optional
from rclpy.node import Node
from rclpy.publisher import Publisher
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

from formant.sdk.agent.v1 import Client

from configuration.config_schema import ConfigSchema
from configuration.publisher_config import PublisherConfig
from ros2_utils.topic_type_provider import TopicTypeProvider
from ros2_utils.qos import QOS_PROFILES, qos_profile_system_default
from utils.logger import get_logger
from ros2_utils.message_utils import (
    get_ros2_type_from_string,
    get_message_path_value,
)


class LocalizationPublisher:
    def __init__(
        self, fclient: Client, node: Node, topic_type_provider: TopicTypeProvider
    ):
        self._fclient = fclient
        self._node = node
        self._localization_goal_publisher: Optional[Publisher] = None
        self._localization_cancel_publisher: Optional[Publisher] = None
        self._topic_type_provider = topic_type_provider
        self._config_lock = threading.Lock()

    def publish_goal(self, goal: PoseStamped):
        with self._config_lock:
            if self._localization_goal_publisher is None:
                return False

    def publish_cancel(self, cancel: bool):
        with self._config_lock:
            if self._localization_cancel_publisher is None:
                return False

    def setup_with_config(self, config: ConfigSchema):
        with self._config_lock:
            self._config = config
            self._cleanup()
            localization_config = config.localization
            if localization_config:
                goal_topic = localization_config.goal_publisher_ros2_topic
                if goal_topic:
                    goal_type = self._topic_type_provider.get_class_for_topic(
                        goal_topic, PoseStamped
                    )
                    self._localization_goal_publisher = self._node.create_publisher(
                        goal_type,
                        goal_topic,
                        qos_profile=qos_profile_system_default,
                    )
                cancel_topic = localization_config.cancel_goal_publisher_ros2_topic
                if cancel_topic:
                    cancel_type = self._topic_type_provider.get_class_for_topic(
                        cancel_topic, Bool
                    )
                    self._localization_cancel_publisher = self._node.create_publisher(
                        cancel_type,
                        cancel_topic,
                        qos_profile=qos_profile_system_default,
                    )

    def _cleanup(self):
        self._node.destroy_publisher(self._localization_goal_publisher)
        self._localization_goal_publisher = None
        self._node.destroy_publisher(self._localization_cancel_publisher)
        self._localization_cancel_publisher = None
