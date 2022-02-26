#!/usr/bin/env python3

import os
from typing import List, Dict
import time
import array
import json
import importlib


from formant.sdk.agent.v1 import Client as FormantAgentClient
from formant.protos.model.v1.datapoint_pb2 import Datapoint
import grpc
import rclpy
from rclpy.qos import qos_profile_sensor_data
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
    Image,
    CompressedImage,
)
from geometry_msgs.msg import (
    Twist
)
from converters.laserscan import ros_laserscan_to_formant_pointcloud
from converters.pointcloud2 import ros_pointcloud2_to_formant_pointcloud
from message_utils.utils import (
    get_message_type_from_string,
    message_to_json,
    get_message_path_value,
)


class Adapter:
    """
    Formant <-> ROS2 Adapter
    """

    def __init__(self):
        self.fclient = FormantAgentClient(
            ignore_throttled=True, ignore_unavailable=True,
        )

        rclpy.init()
        self.node = rclpy.create_node("formant_ros2_bridge")

        # Mapping from configured ROS2 topic name to ROS2 message type
        self.topic_to_type = {}  # type: Dict[str, Any]

        # Mapping from configured ROS2 topic name to this ROS2 node's subscriber
        self.topic_to_subscription = (
            {}
        )  # type: Dict[str, rclpy.subscription.Subscription]

        self.topic_to_publisher = (
            {}
        )

        # Keeps track of last time published to control publish rate to Formant.
        self.rate_control_for_topics = {}  # type: Dict[str, float]

        current_directory = os.path.dirname(os.path.realpath(__file__))
        with open(f"{current_directory}/config.json") as f:
            self.config = json.loads(f.read())

        # For console output acknowledgement that the script has started running even if it
        # hasn't yet established communication with the Formant agent.
        print("INFO: `main.py` script has started running.")

        # Set up teleoperation
        self.register_control_streams()

        while rclpy.ok():
            self.update_types()
            self.update_subscriptions()
            rclpy.spin_once(self.node, timeout_sec=1.0)

        self.node.destroy_node()
        rclpy.shutdown()

    def get_configured_topics(self):
        return [stream["topic"] for stream in self.config["streams"]]

    def message_callback(self, topic, base_message):
        """
        Ingests a ROS2 message as a Formant datapoint
        """
        configs = [c for c in self.config["streams"] if c["topic"] == topic]
        if len(configs) == 0:
            return

        for config in configs:
            stream = None
            message = None
            message_path_value = None
            message_path_values = None
            bitset = None
            numericset = None
            rate = None

            if "rate" in config:
                # Records time of last publish to Formant.
                if not topic in self.rate_control_for_topics.keys():
                    # Records time of first publish to Formant for this topic.
                    self.rate_control_for_topics[topic] = time.time()
                else:
                    # Rate is in hz
                    rate = config["rate"]
                    time_to_wait = 1.0 / rate
                    time_since_last_publish = (
                        time.time() - self.rate_control_for_topics[topic]
                    )
                    if time_to_wait <= time_since_last_publish:
                        self.rate_control_for_topics[topic] = time.time()
                    else:
                        continue

            if "stream" in config:
                # If the stream is configured in config.json,
                # use that name directly.
                stream = config["stream"]
            else:
                # Otherwise, generate a name from the topic name.
                # e.g. "/rover/cmd_vel" -> "rover.cmd_vel"
                stream = topic[1:].replace("/", ".")

            # handle input types with multiple message paths
            if "formantType" in config and config["formantType"] == "bitset":
                if "messagePaths" not in config:
                    raise ValueError("Missing messagePaths in config for bitset")
                message_path_values = [
                    get_message_path_value(base_message, path)
                    for path in config["messagePaths"]
                ]
                bitset = {
                    k: v for k, v in zip(config["messagePaths"], message_path_values)
                }
                self.fclient.post_bitset(stream, bitset)

            elif "formantType" in config and config["formantType"] == "numericset":
                if "messagePaths" not in config:
                    raise ValueError("Missing messagePaths in config for numericset")
                if "units" not in config:
                    raise ValueError("Missing units in config for numericset")
                message_path_values = [
                    get_message_path_value(base_message, path)
                    for path in config["messagePaths"]
                ]
                units = config["units"]
                if len(units) != len(message_path_values):
                    raise ValueError("len of messagePaths must match len of units")
                numericset = {}
                for i in range(len(message_path_values)):
                    numericset[config["messagePaths"][i]] = (
                        message_path_values[i],
                        units[i],
                    )
                self.fclient.post_numericset(stream, numericset)

            # handle input types with no message path or a single message path
            else:
                if "messagePath" in config:
                    message = get_message_path_value(
                        base_message, config["messagePath"]
                    )
                else:
                    message = base_message

                if type(message) == str:
                    self.fclient.post_text(stream, message)
                elif type(message) in [int, float]:
                    self.fclient.post_numeric(stream, message)
                elif type(message) == String:
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
                    self.fclient.post_geolocation(
                        stream, message.latitude, message.longitude
                    )
                elif type(message) == BatteryState:
                    self.fclient.post_battery(
                        stream,
                        message.percentage,
                        voltage=message.voltage,
                        current=message.current,
                        charge=message.charge,
                    )
                elif type(message) == Image:
                    print(
                        "Error ingesting "
                        + stream
                        + ": "
                        + "Raw image streams are currently unsupported. Please use CompressedImage."
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
                                point_cloud=ros_laserscan_to_formant_pointcloud(
                                    message
                                ),
                                timestamp=int(time.time() * 1000),
                            )
                        )
                    except grpc.RpcError as e:
                        return
                    except Exception as e:
                        print("Error ingesting " + stream + ": " + str(e))
                        return
                elif type(message) == PointCloud2:
                    try:
                        self.fclient.agent_stub.PostData(
                            Datapoint(
                                stream=stream,
                                point_cloud=ros_pointcloud2_to_formant_pointcloud(
                                    message
                                ),
                                timestamp=int(time.time() * 1000),
                            )
                        )
                    except grpc.RpcError as e:
                        return
                    except Exception as e:
                        print("Error ingesting " + stream + ": " + str(e))
                        return
                else:  # Ingest any messages without a direct mapping to a Formant type as JSON
                    self.fclient.post_json(stream, message_to_json(message))

    def update_types(self):
        """
        Refreshes this adapter's mapping between configured topics and types
        """
        new_topic_to_type = {}  # type: Dict[str, Any]

        for (topic_name, topic_types) in self.node.get_topic_names_and_types():
            if topic_name not in self.get_configured_topics():
                continue
            if len(topic_types) == 0:
                continue
            # assumes each topic has only one type
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
                continue  # already subscribed
            self.topic_to_subscription[topic] = self.node.create_subscription(
                message_type,
                topic,
                lambda m, t=topic: self.message_callback(t, m),
                qos_profile_sensor_data,
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
                    qos_profile_sensor_data,
                )

    def register_control_streams(self):
        for control_stream in self.config["control-streams"]:
            if control_stream["formantType"] == 'twist':
                self.topic_to_publisher[control_stream["topic"]] = self.node.create_publisher(
                    Twist, control_stream["topic"], 10
                )
            elif control_stream["formantType"] == "bool":
                self.topic_to_publisher[control_stream["topic"]] = self.node.create_publisher(
                    Bool, control_stream["topic"], 10
                )

        print(self.config["control-streams"])

        self.fclient.register_teleop_callback(self.handle_teleop)

    def handle_teleop(self, msg):
        # try:
        if msg.stream.casefold() == "joystick".casefold():
            pub = self.topic_to_publisher[
                list(
                    filter(
                        lambda stream: stream['formantType'] == 'twist', self.config["control-streams"]
                    )
                )[0]["topic"]
            ]

            self.publish_twist(msg.twist, pub)

        elif msg.stream.casefold() == "buttons".casefold():
            print(self.topic_to_publisher["/" + str(msg.bitset.bits[0].key)])
            # pub = self.topic_to_publisher[msg.bitset.bits.key]

            # print(pub)
            # self.publish_bool(msg)

        # except Exception as e:
        #     self.fclient.post_text("adapter.errors", "Error handling teleop: %s" %  str(e))


    def publish_twist(self, value, publisher):
        msg = Twist()
        msg.linear.x = value.linear.x
        msg.linear.y = value.linear.y
        msg.linear.z = value.linear.z
        msg.angular.x = value.angular.x
        msg.angular.y = value.angular.y
        msg.angular.z = value.angular.z

        publisher.publish(msg)

    def publish_bool(self, msg):
        print("publish bool", msg)

if __name__ == "__main__":
    try:
        Adapter()
    except KeyboardInterrupt:
        exit()
