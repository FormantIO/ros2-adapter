import os
from rclpy.node import Node
from rclpy.subscription import Subscription
import time
import threading
from typing import List, Dict, Optional, Any
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

from formant.sdk.agent.v1 import Client

from configuration.config_schema import ConfigSchema
from .ingester import Ingester
from ros2_utils.qos import QOS_PROFILES, qos_profile_system_default
from ros2_utils.topic_type_provider import TopicTypeProvider
from utils.logger import get_logger
from ros2_utils.message_utils import (
    get_ros2_type_from_string,
    get_message_path_value,
)
from configuration.numeric_set_config import (
    NumericSetConfig,
    NumericSetSubscriberConfig,
)


FORMANT_OVERRIDE_TIMESTAMP = (
    os.getenv("FORMANT_OVERRIDE_TIMESTAMP", "").lower() == "true"
)


class NumericSetSubscriberCoordinator:
    def __init__(
        self,
        fclient: Client,
        node: Node,
        topic_type_provider: TopicTypeProvider,
    ):
        self._numeric_set_buffer: Dict[str, Dict[str, Any]] = {}

        self._fclient = fclient
        self._node = node
        self._topic_type_provider = topic_type_provider
        self._subscriptions: Dict[str, List[Subscription]] = {}
        self._logger = get_logger()
        self._config_lock = threading.Lock()

    def setup_with_config(self, config: ConfigSchema):
        with self._config_lock:
            self._config = config
            self._cleanup()
            if self._config.numeric_sets:
                for subscriber_config in self._config.numeric_sets:
                    try:
                        self._setup_subscription_for_config(subscriber_config)
                    except ValueError as value_error:
                        self._logger.warn(value_error)
                        continue

    def _setup_subscription_for_config(self, numeric_set_config: NumericSetConfig):
        formant_stream = numeric_set_config.formant_stream
        self._subscriptions[formant_stream] = []
        for subscriber in numeric_set_config.subscribers:
            topic = subscriber.topic
            ros2_type = subscriber.message_type
            if ros2_type is None:
                ros2_type = self._topic_type_provider.get_type_for_topic(topic)
            if ros2_type is None:
                raise ValueError("No ROS2 type found for %s" % topic)
            qos_profile = QOS_PROFILES.get(
                subscriber.qos_profile, qos_profile_system_default
            )

            new_sub = self._node.create_subscription(
                get_ros2_type_from_string(ros2_type),
                topic,
                lambda msg, config=numeric_set_config, subscriber_config=subscriber: self._handle_message(
                    msg, config, subscriber_config
                ),
                qos_profile=qos_profile,
            )

    def _handle_message(
        self,
        msg,
        numeric_set_config: NumericSetConfig,
        subscriber_config: NumericSetSubscriberConfig,
    ):
        with self._config_lock:
            formant_stream = numeric_set_config.formant_stream
            path = subscriber_config.message_path
            if path:
                try:
                    msg = get_message_path_value(msg, path)
                    msg_type = type(msg)
                except:
                    self._logger.error(
                        "Could not find path '%s' in message %s" % (path, str(msg))
                    )
                    return

            msg_type = type(msg)

            label = subscriber_config.label
            unit = subscriber_config.unit if subscriber_config.unit else ""

            # If the message has a data attribute, use that
            if hasattr(msg, "data"):
                msg = msg.data

            # If the message is already a number, use that
            if msg_type in [int, float]:
                value = msg

            # If the message is a ROS2 integer, cast it to an int and use that
            elif msg_type in [Int8, Int16, Int32, Int64, UInt8, UInt16, UInt32, UInt64]:
                value = int(msg)

            # If the message is a ROS2 float, cast it to a float and use that
            elif msg_type in [Float32, Float64]:
                value = float(msg)

            else:
                self._logger.error(
                    "Could not ingest %s: %s" % (formant_stream, str(msg_type))
                )
                return

            # Update the current set with the new value
            if formant_stream not in self.numeric_set_buffer:
                self._numeric_set_buffer[formant_stream] = {}

            self._numeric_set_buffer[formant_stream][label] = (value, unit)

            self._fclient.post_numericset(
                formant_stream, self._numeric_set_buffer[formant_stream]
            )

    def _cleanup(self):
        for subscription_item in self._subscriptions.items():
            for subscription in subscription_item[1]:
                self._node.destroy_subscription(subscription)
        self._subscriptions = {}
        self.numeric_set_buffer = {}
