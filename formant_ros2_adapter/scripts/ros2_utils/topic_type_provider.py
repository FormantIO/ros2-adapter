import time
from typing import Dict
from rclpy.node import Node

from message_utils.utils import get_ros2_type_from_string


class TopicTypeProvider:
    def __init__(self, node: Node):
        self._node = node
        self.ros2_topic_names_and_types: Dict[str, str] = {}
        self.ros2_service_names_and_types: Dict[str, str] = {}

    def update_topic_types(self):
        self.ros2_topic_names_and_types = {}
        ros2_topic_names_and_types = self._node.get_topic_names_and_types()
        print(ros2_topic_names_and_types)
        for topic in ros2_topic_names_and_types:
            # Just use first type
            self.ros2_topic_names_and_types[topic[0]] = topic[1][0]

        self.ros2_service_names_and_types = {}
        ros2_service_names_and_types = self._node.get_service_names_and_types()
        for service in ros2_service_names_and_types:
            self.ros2_service_names_and_types[service[0]] = service[1][0]

    def get_type_for_topic(self, topic):
        print(self.ros2_topic_names_and_types)
        result = self.ros2_topic_names_and_types.get(topic, None)
        while result is None:
            self.update_topic_types()
            print(self.ros2_topic_names_and_types)
            result = self.ros2_topic_names_and_types.get(topic, None)
        return result

    def get_class_for_topic(self, topic, default):
        type_str = self.get_type_for_topic(topic)
        if type_str is None:
            return default
        return get_ros2_type_from_string(type_str)

    def get_service_type_for_name(self, name):
        return self.ros2_service_names_and_types.get(name, None)
