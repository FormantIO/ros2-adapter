from rclpy.node import Node

from formant.sdk.agent.v1 import Client

from .basic_subscriber_coodinator import BasicSubscriberCoordinator
from configuration.config_schema import ConfigSchema
from .ingester import Ingester
from .localization_subscriber_coodinator import LocalizationSubscriberCoordinator
from .numeric_set_subscriber_coodinator import NumericSetSubscriberCoordinator
from ros2_utils.topic_type_provider import TopicTypeProvider
from utils.logger import get_logger


class SubscriberCoordinator:
    def __init__(
        self, fclient: Client, node: Node, topic_type_provider: TopicTypeProvider
    ):
        self._logger = get_logger()
        self._fclient = fclient
        self._ingester = Ingester(self._fclient)
        self._node = node
        self._topic_type_provider = topic_type_provider
        self._basic_subscriber_coodinator = BasicSubscriberCoordinator(
            fclient, node, self._ingester, self._topic_type_provider
        )
        self._localization_subscriber_coordinator = LocalizationSubscriberCoordinator(
            fclient, node, self._topic_type_provider
        )
        self._numeric_set_subscriber_coodinator = NumericSetSubscriberCoordinator(
            fclient, node, self._topic_type_provider
        )

    def setup_with_config(self, config: ConfigSchema):
        self._logger.info("Setting up Subscriber Coordinator")
        self._basic_subscriber_coodinator.setup_with_config(config)
        self._localization_subscriber_coordinator.setup_with_config(config)
        self._numeric_set_subscriber_coodinator.setup_with_config(config)
        self._logger.info("Set up Subscriber Coordinator")
