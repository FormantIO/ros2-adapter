from typing import List, Dict
import time
import array
import json
import importlib

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
)
from formant.sdk.agent.v1 import Client as FormantAgentClient
from formant.protos.model.v1.datapoint_pb2 import Datapoint

from converters.laserscan import ros_laserscan_to_formant_pointcloud
from converters.pointcloud2 import ros_pointcloud2_to_formant_pointcloud

QOS_PROFILE = 10


class Adapter:
    """
    Formant <-> ROS2 Adapter
    """

    def __init__(self):
        self.fclient = FormantAgentClient(
            ignore_throttled=True, ignore_unavailable=True,
        )

        self.recv_pc = False

        rclpy.init()
        self.node = rclpy.create_node("formant_ros2_bridge")

        # Mapping from configured ROS2 topic name to ROS2 message type
        self.topic_to_type = {}  # type: Dict[str, Any]

        # Mapping from configured ROS2 topic name to this ROS2 node's subscriber
        self.topic_to_subscription = (
            {}
        )  # type: Dict[str, rclpy.subscription.Subscription]

        with open("./config.json") as f:
            self.config = json.loads(f.read())

        while rclpy.ok():
            self.update_types()
            self.update_subscriptions()
            rclpy.spin_once(self.node, timeout_sec=1.0)

        self.node.destroy_node()
        rclpy.shutdown()

    def get_configured_topics(self):
        return [stream["topic"] for stream in self.config["streams"]]

    def message_callback(self, topic, message):
        """
        Ingests a ROS2 message as a Formant datapoint
        """
        # Uses the topic name to procedurally generate a stream name
        # e.g. "/rover/cmd_vel" -> "rover.cmd_vel"
        # To change the display name, adjust the stream's Alias in stream configuration
        stream = topic[1:].replace("/", ".")

        if type(message) == String:
            self.fclient.post_text(stream, message.data)
        elif type(message) == Char:
            self.fclient.post_text(stream, str(message.data))
        elif type(message) in [
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
        ]:
            self.fclient.post_numeric(stream, message.data)
        elif type(message) == Bool:
            self.fclient.post_bitset(stream, {topic: message.data})
        elif type(message) == NavSatFix:
            self.fclient.post_geolocation(stream, message.latitude, message.longitude)
        elif type(message) == BatteryState:
            self.fclient.post_battery(
                stream,
                message.percentage,
                voltage=message.voltage,
                current=message.current,
                charge=message.charge,
            )
        elif type(message) == CompressedImage:
            if "jpg" in message.format or "jpeg" in message.format:
                content_type = "image/jpg"
            elif "png" in message.format:
                content_type = "image/png"
            else:
                print("Unsupported image format:", message.format)
                return
            self.fclient.post_image(
                stream, bytes(message.data), content_type=content_type
            )
        elif type(message) == LaserScan:
            try:
                self.fclient.agent_stub.PostData(
                    Datapoint(
                        stream=stream,
                        point_cloud=ros_laserscan_to_formant_pointcloud(message),
                        timestamp=int(time.time() * 1000),
                    )
                )
            except Exception as e:
                return
        elif type(message) == PointCloud2:
            try:
                self.fclient.agent_stub.PostData(
                    Datapoint(
                        stream=stream,
                        point_cloud=ros_pointcloud2_to_formant_pointcloud(message),
                        timestamp=int(time.time() * 1000),
                    )
                )
            except Exception as e:
                return
        elif type(message) == Image:
            pass
        elif type(message) == CompressedImage:
            pass
        else:  # Ingest any messages without a direct mapping to a Formant type as JSON
            self.fclient.post_json(stream, message_to_json(message))

    def update_types(self):
        """
        Refreshes this adapter's mapping between configured topics and types
        """
        new_topic_to_type = {}  # type: Dict[str, Any]

        for node_name in self.node.get_node_names():
            for (
                topic_name,
                topic_types,
            ) in self.node.get_publisher_names_and_types_by_node(node_name, ""):
                if topic_name not in self.get_configured_topics():
                    continue
                if len(topic_types) == 0:
                    continue
                # TODO: handle multiple topic types
                message_type = get_message_type_from_string(topic_types[0])
                if message_type is not None:
                    new_topic_to_type[topic_name] = message_type

        self.topic_to_type = new_topic_to_type

    def update_subscriptions(self):
        """
        Creates and maintains subscriptions for configured topics
        """
        # Add new subscriptions
        for topic, message_type in self.topic_to_type.items():
            if topic in self.topic_to_subscription:
                continue
            self.topic_to_subscription[topic] = self.node.create_subscription(
                message_type,
                topic,
                lambda m, t=topic: self.message_callback(t, m),
                QOS_PROFILE,
            )

        # Modify any existing subscriptions whose type has changed
        for topic, subscription in self.topic_to_subscription.items():
            current_message_type = subscription.msg_type
            latest_message_type = self.topic_to_type.get(topic, None)
            if latest_message_type is None:
                continue
            if current_message_type != latest_message_type:
                self.topic_to_subscription[topic] = self.node.create_subscription(
                    latest_message_type,
                    topic,
                    lambda m, t=topic: self.message_callback(t, m),
                    QOS_PROFILE,
                )


def get_message_type_from_string(message_type_string: str):
    """
    Returns a ROS2 message type for the provided ROS2 message type string
    """
    try:
        path = message_type_string.replace("/", ".").split(".")
        module_name = ".".join(path[:-1])
        module = importlib.import_module(module_name)
        return getattr(module, path[-1])
    except:
        print("Couldn't import ROS2 message type from string: ", message_type_string)
        return None


def parse(m):
    if type(m) in [bool, str, int, float, bytes]:
        return m
    elif type(m) in [list, array.array]:
        return [parse(o) for o in m]
    else:
        return {k: parse(getattr(m, k)) for k in m._fields_and_field_types}


def message_to_json(message):
    """
    Converts any ROS2 message into a JSON string
    """
    return json.dumps(parse(message))


if __name__ == "__main__":
    Adapter()
