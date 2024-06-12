import os
from rclpy.node import Node
from rclpy.subscription import Subscription
from typing import List, Dict, Optional
import threading
import time
import traceback

from formant.sdk.agent.v1 import Client

from configuration.config_schema import ConfigSchema
from configuration.subscriber_config import SubscriberConfig, MessagePathConfig
from .ingester import Ingester
from .decorators import lock_decorator, handle_value_error
from ros2_utils.qos import QOS_PROFILES, qos_profile_system_default
from ros2_utils.topic_type_provider import TopicTypeProvider
from utils.logger import get_logger
from ros2_utils.message_utils import (
    get_ros2_type_from_string,
    get_message_path_value,
)


FORMANT_OVERRIDE_TIMESTAMP = (
    os.getenv("FORMANT_OVERRIDE_TIMESTAMP", "").lower() == "true"
)
SCHEDULE_SUBSCRIPTIONS_INTERVAL = 1.0


class BasicSubscriberCoordinator:
    def __init__(
        self,
        fclient: Client,
        node: Node,
        ingester: Ingester,
        topic_type_provider: TopicTypeProvider,
    ):
        self._fclient = fclient
        self._node = node
        self._ingester = ingester
        self._topic_type_provider = topic_type_provider
        self._subscriptions: Dict[str, List[Subscription]] = {}
        self._queued_topics: Dict = {}
        self._logger = get_logger()
        self._config_lock = threading.RLock()
        self._subscribe_lock = threading.Lock()

    @lock_decorator("_config_lock")
    def setup_with_config(self, config: ConfigSchema):
        self._config = config
        self._cleanup()

        has_subscribers = self._config.subscribers
        if not has_subscribers:
            return

        self._queued_topics = {
            subscriber.topic: subscriber for subscriber in self._config.subscribers
        }

        self._schedule_subscriptions()

    def _schedule_subscriptions(self):
        t = threading.Timer(SCHEDULE_SUBSCRIPTIONS_INTERVAL, self._setup_subscribers)
        t.daemon = True
        t.start()

    @lock_decorator("_subscriber_lock")
    def _setup_subscribers(self):
        active_topics = set(self._get_active_topics())

        backlog_topics = active_topics.intersection(self._queued_topics.keys)

        for topic in backlog_topics:
            topic_subscriber = self._queued_topics[topic]
            self._setup_subscription(topic_subscriber)
            _ = self._queued_topics.pop(topic)

    @handle_value_error
    def _setup_subscription(self, subscriber_config: SubscriberConfig):
        topic = subscriber_config.topic
        qos_profile = QOS_PROFILES.get(
            subscriber_config.qos_profile, qos_profile_system_default
        )

        ros2_type = subscriber_config.message_type
        if ros2_type is None:
            ros2_type = self._topic_type_provider.get_type_for_topic(topic)

        self._logger.debug(
            "Setting up subscription %s, %s, %s"
            % (topic, subscriber_config.qos_profile, ros2_type)
        )
        new_subscriber = self._node.create_subscription(
            get_ros2_type_from_string(ros2_type),
            topic,
            callback=lambda msg, subscriber_config=subscriber_config: self._handle_message(
                msg, subscriber_config
            ),
            qos_profile=qos_profile,
        )

        if topic not in self._subscriptions:
            self._subscriptions[topic] = []
        self._subscriptions[topic].append(new_subscriber)

    def _get_active_topics(self):
        return [
            topic_name
            for topic_name, _ in self._node.get_topic_names_and_types()
            if self._node.get_publishers_info_by_topic(topic_name)
        ]

    def _handle_message(
        self,
        msg,
        subscriber_config: SubscriberConfig,
        message_path_config: Optional[MessagePathConfig] = None,
        timestamp: Optional[int] = None,
    ):
        with self._config_lock:
            try:
                message_type = type(msg)
                formant_stream = subscriber_config.formant_stream
                topic = subscriber_config.topic
                tags = {}
                if (
                    message_path_config
                    and message_path_config.tag_key
                    and message_path_config.tag_value
                ):
                    tags = {message_path_config.tag_key: message_path_config.tag_value}
                if timestamp is None:
                    timestamp = int(time.time() * 1000)
                    if hasattr(msg, "header"):
                        if not FORMANT_OVERRIDE_TIMESTAMP:
                            header_timestamp = (
                                msg.header.stamp.sec * 1000
                                + msg.header.stamp.nanosec / 1000000
                            )
                            # sanity check to make sure ros header stamp is in epoch time
                            if header_timestamp > 1500000000000:
                                timestamp = int(header_timestamp)

                if message_path_config is None:
                    if subscriber_config.message_paths:
                        for message_path_config in subscriber_config.message_paths:
                            inner_msg = get_message_path_value(
                                msg, message_path_config.path
                            )
                            # handle the inner message with the path config
                            self._handle_message(
                                inner_msg, subscriber_config, message_path_config
                            )
                        return
                self._ingester.ingest(
                    msg, message_type, formant_stream, topic, timestamp, tags
                )
            except Exception as e:
                self._logger.error("Error handling message %s" % traceback.format_exc())

    def _cleanup(self):
        for subscription_item in self._subscriptions.items():
            for subscription in subscription_item[1]:
                self._node.destroy_subscription(subscription)
        self._subscriptions = {}
