from typing import List, Optional, Dict
from rclpy.node import Node
from rclpy.publisher import Publisher

from formant.sdk.agent.v1 import Client

from configuration.config_schema import ConfigSchema
from configuration.publisher_config import PublisherConfig
from .generic_publisher import GenericPublisher
from .localization_publisher import LocalizationPublisher
from ros2_utils.topic_type_provider import TopicTypeProvider
from utils.logger import get_logger


class PublisherCoordinator:
    def __init__(
        self, fclient: Client, node: Node, topic_type_provider: TopicTypeProvider
    ):
        self._logger = get_logger()
        self._topic_type_provider = topic_type_provider
        self._fclient = fclient
        self._node = node
        self.localization_publisher = LocalizationPublisher(
            fclient, node, self._topic_type_provider
        )
        self.generic_publisher = GenericPublisher(
            fclient, node, self._topic_type_provider
        )

    def setup_with_config(self, config: ConfigSchema):
        self._logger.info("Setting up Publisher Coordinator")
        self.localization_publisher.setup_with_config(config)
        self.generic_publisher.setup_with_config(config)
        self._logger.info("Set up Publisher Coordinator")
