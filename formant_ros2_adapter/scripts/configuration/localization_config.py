from typing import Dict, List, Optional

from .get_value import get_value


class LocalizationConfig:
    def __init__(self, config: Dict):
        self.formant_stream: str = get_value(config, "formant_stream", required=True)
        self.base_reference_frame: str = get_value(
            config, "base_reference_frame", required=True
        )
        self.odometry_subscriber_ros2_topic: str = get_value(
            config, "odometry_subscriber_ros2_topic", required=True
        )
        self.map_subscriber_ros2_topic: Optional[str] = get_value(
            config, "map_subscriber_ros2_topic"
        )
        self.point_cloud_subscriber_ros2_topics: Optional[
            List[PointCloudSubscriberConfig]
        ] = get_value(
            config,
            "point_cloud_subscriber_ros2_topics",
            cls=PointCloudSubscriberConfig,
            is_array=True,
        )
        self.path_subscriber_ros2_topic: Optional[str] = get_value(
            config, "path_subscriber_ros2_topic"
        )
        self.goal_subscriber_ros2_topic: Optional[str] = get_value(
            config, "goal_subscriber_ros2_topic"
        )
        self.goal_publisher_ros2_topic: Optional[str] = get_value(
            config, "goal_publisher_ros2_topic"
        )
        self.cancel_goal_publisher_ros2_topic: Optional[str] = get_value(
            config, "cancel_goal_publisher_ros2_topic"
        )


class PointCloudSubscriberConfig:
    def __init__(self, config: Dict):
        self.topic: str = get_value(config, "ros2_topic", required=True)
