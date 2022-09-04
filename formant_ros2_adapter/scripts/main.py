#!/usr/bin/env python3

import os
from unittest import case
import cv2
import time
import array
import json
import grpc
import importlib
import jsonschema
import numpy as np
from typing import List, Dict

from formant.sdk.agent.v1 import Client as FormantAgentClient
from formant.protos.model.v1.datapoint_pb2 import Datapoint

import rclpy
from cv_bridge import CvBridge
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import (
    Bool, Char, String, 
    Float32, Float64,
    Int8, Int16, Int32, Int64,
    UInt8, UInt16, UInt32, UInt64
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
    Point, Point32, Polygon,
    Pose, PoseStamped, PoseArray,
    PoseWithCovariance, PoseWithCovarianceStamped,
    Twist, TwistStamped, Vector3, Vector3Stamped
)
from converters.laserscan import ros2_laserscan_to_formant_pointcloud
from converters.pointcloud2 import ros2_pointcloud2_to_formant_pointcloud

from message_utils.utils import (
    get_message_type_from_string,
    message_to_json,
    get_message_path_value,
)

class ROS2Adapter:
    """
    Formant <-> ROS2 Adapter
    """

    def __init__(self):
        # For console output acknowledgement that the script has started running even if it
        # hasn't yet established communication with the Formant agent.
        print("INFO: ROS2 Adapter has started running.")

        # Connect to ROS2
        rclpy.init()
        self.cv_bridge = CvBridge()
        self.ros2_node = rclpy.create_node("formant_ros2_adapter")

        # Set up the config object
        self.config = {}

        # Subscribers and publishers are stored as a dictionary of topics containing a list of objects
        self.ros2_subscribers = {}
        self.ros2_publishers = {}
        self.ros2_service_calls = {}

        # Set up the adapter
        self.fclient = FormantAgentClient(ignore_throttled=True, ignore_unavailable=True)
        self.fclient.register_config_update_callback(self.update_adapter_configuration)
        self.fclient.register_teleop_callback(self.handle_formant_teleop_msg)
        self.fclient.register_command_request_callback(self.handle_formant_command_request_msg)

        # Start spinning
        while rclpy.ok():
            rclpy.spin_once(self.ros2_node, timeout_sec=1.0)

        # Clean up before shutting down
        self.ros2_node.destroy_node()
        rclpy.shutdown()

    def update_adapter_configuration(self):
        # Load config from either the agent's json blob or the config.json file
        try:
            config_blob = json.loads(self.fclient.get_config_blob_data())
        except:
            # Otherwise, load from the config.json file shipped with the adapter
            current_directory = os.path.dirname(os.path.realpath(__file__))
            with open(f"{current_directory}/config.json") as f:
                config_blob = json.loads(f.read())
            
        # Validate configuration based on schema
        with open("config_schema.json") as f:
            self.config_schema = json.load(f)

        # Runt the validation check    
        try:
            jsonschema.validate(config_blob, self.config_schema)
            print("INFO: Validation succeeded.")
        except Exception as e:
            print("WARNING: Validation failed:", e)
            self.fclient.create_event(
                "ROS2 Adapter configuration failed validation",
                notify=False,
                severity="warning", 
            )
            return

        # Set the config object to the validated configuration
        if "ros2_adapter_configuration" in config_blob:
            # Check the json blob for a "ros2_adapter_configuration" section and use it first
            self.config = config_blob["ros2_adapter_configuration"]
        else:
            self.config = {}

        self.update_ros2_information()

        # Fill out the config with default values
        for subscriber_config in self.config["subscribers"]:
            if "formant_stream" not in subscriber_config:
                formant_stream = subscriber_config["ros2_topic"][1:].replace("/", ".")
                subscriber_config["formant_stream"] = formant_stream

            if "ros2_message_type" not in subscriber_config:
                # Get the message type from the topic
                try:
                    subscriber_config["ros2_message_type"] = self.ros2_topic_names_and_types[
                        subscriber_config["ros2_topic"]
                    ]
                except:
                    print("WARNING: setting type bool for unknown topic", subscriber_config["ros2_topic"])
                    subscriber_config["ros2_message_type"] = "std_msgs/msg/Bool"
                    
        self.setup_subscribers()
        self.setup_publishers()

        self.fclient.post_json("adapter.configuration", json.dumps(self.config))
        self.fclient.create_event(
            "ROS2 Adapter started",
            notify=False,
            severity="info", 
        )

    def update_ros2_information(self):
        # Update knowledge about topics and services
        self.ros2_topic_names_and_types = {}
        ros2_topic_names_and_types = self.ros2_node.get_topic_names_and_types()
        for topic in ros2_topic_names_and_types:
            self.ros2_topic_names_and_types[topic[0]] = topic[1][0] # Just use the first one

        self.ros2_service_names_and_types = {}
        ros2_service_names_and_types = self.ros2_node.get_service_names_and_types()
        for service in ros2_service_names_and_types:
            self.ros2_service_names_and_types[service[0]] = service[1][0] # Just use the first one

    def setup_subscribers(self):
        # Remove any existing subscribers
        for subscriber_list in self.ros2_subscribers.values():
            for subscriber in subscriber_list:
                self.ros2_node.destroy_subscription(subscriber)
            
        self.ros2_subscribers = {}

        # Create new subscribers based on the config
        for subscriber_config in self.config["subscribers"]:
            new_sub = self.ros2_node.create_subscription(
                get_message_type_from_string(subscriber_config["ros2_message_type"]),
                subscriber_config["ros2_topic"],
                lambda msg, subscriber_config=subscriber_config: self.handle_ros2_message(
                    msg, 
                    subscriber_config
                ),
                qos_profile_sensor_data,
            )

            if subscriber_config["ros2_topic"] not in self.ros2_subscribers:
                self.ros2_subscribers[subscriber_config["ros2_topic"]] = []

            self.ros2_subscribers[subscriber_config["ros2_topic"]].append(new_sub)

    def setup_publishers(self):
        # Remove any existing publishers
        for publisher_list in self.ros2_publishers.values():
            for publisher in publisher_list:
                self.ros2_node.destroy_publisher(publisher)
        
        self.ros2_publishers = {}

        # Create new publishers based on the config
        for publisher in self.config["publishers"]:
            # If there is no type, get it from the topic... but this shouldn't happen
            if "ros2_message_type" not in publisher:
                if publisher["ros2_topic"] in self.ros2_topic_names_and_types:
                    publisher["ros2_message_type"] = self.ros2_topic_names_and_types[publisher["ros2_topic"]]
                else:
                    print("ERROR: No message type for topic", publisher["ros2_topic"])
                    continue

            new_pub = self.ros2_node.create_publisher(
                get_message_type_from_string(publisher["ros2_message_type"]),
                publisher["ros2_topic"],
                qos_profile_sensor_data,
            )

            if publisher["formant_stream"] not in self.ros2_publishers:
                self.ros2_publishers[publisher["formant_stream"]] = []

            self.ros2_publishers[publisher["formant_stream"]].append(new_pub)

    def handle_ros2_message(self, msg, subscriber_config):
        # Get the message type
        msg_type = type(msg)
        formant_stream = subscriber_config["formant_stream"]
        ros2_topic = subscriber_config["ros2_topic"]

        # Select the part of the message based on the path
        if "ros2_message_paths" in subscriber_config:
            print("WARNING: message paths are not yet implemented")
            for path in subscriber_config["ros2_message_paths"]:
                try:
                    path_msg = get_message_path_value(msg, path["path"])
                except:
                    # If this path does not match, ignore it and log the error
                    print(f"ERROR: Could not find path '{path['path']}' in message {msg_type}")
                    pass

        # TODO: Get the timestamp from the message, or make a new one
        # try:
        #     msg_timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        # except:

        msg_timestamp = int(time.time() * 1000)

        # TODO: collect all the data for localization and send it in one message when it changes
        # OR send every item on its own stream, and then use the new aggregate localization module

        # Handle the message based on its type

        try:
            if msg_type == String:
                if hasattr(msg, "data"):
                    msg = msg.data

                self.fclient.post_text(
                    formant_stream, 
                    msg, 
                    timestamp=msg_timestamp
                )

            elif msg_type == Char:
                if hasattr(msg, "data"):
                    msg = msg.data

                self.fclient.post_text(
                    formant_stream, 
                    str(msg), 
                    timestamp=msg_timestamp
                )

            elif msg_type == Bool:
                if hasattr(msg, "data"):
                    msg = msg.data

                self.fclient.post_bitset(
                    formant_stream, {
                        ros2_topic: msg
                    }, 
                    timestamp=msg_timestamp
                )

            elif msg_type in [
                Float32, Float64,
                Int8, Int16, Int32, Int64,
                UInt8, UInt16, UInt32, UInt64
            ]:
                if hasattr(msg, "data"):
                    msg = msg.data

                self.fclient.post_numeric(
                    formant_stream, 
                    msg,
                    timestamp=msg_timestamp
                )

            elif msg_type == NavSatFix:
                # Convert NavSatFix to a Formant location
                self.fclient.post_geolocation(
                    stream=formant_stream,
                    latitude=msg.latitude,
                    longitude=msg.longitude,
                    altitude=msg.altitude,
                    timestamp=msg_timestamp,
                )
            
            elif msg_type == Image:
                # Convert Image to a Formant image
                cv_image = self.cv_bridge.imgmsg_to_cv2(msg, "passthrough")
                encoded_image = cv2.imencode(".jpg", cv_image)[1].tobytes()

                self.fclient.post_image(
                    stream=formant_stream,
                    value=encoded_image,
                    timestamp=msg_timestamp,
                )

            elif msg_type == CompressedImage:
                # Post the compressed image
                if "jpg" in msg.format or "jpeg" in msg.format:
                    content_type = "image/jpg"
                elif "png" in msg.format:
                    content_type = "image/png"
                else:
                    print("ERROR unsupported image format:", msg.format)
                    return
                self.fclient.post_image(
                    formant_stream, 
                    value=bytes(msg.data), 
                    content_type=content_type,
                    timestamp=msg_timestamp,
                )

            elif msg_type == BatteryState:
                self.fclient.post_battery(
                    formant_stream,
                    msg.percentage,
                    voltage=msg.voltage,
                    current=msg.current,
                    charge=msg.charge,
                )

            elif msg_type == LaserScan:
                # Convert LaserScan to a Formant pointcloud
                try:
                    self.fclient.agent_stub.PostData(
                        Datapoint(
                            stream=formant_stream,
                            point_cloud=ros2_laserscan_to_formant_pointcloud(msg),
                            timestamp=msg_timestamp,
                        )
                    )
                except grpc.RpcError as e:
                    return
                except Exception as e:
                    print("ERROR ingesting " + formant_stream + ": " + str(e))
                    return

            elif msg_type == PointCloud2:
                try:
                    self.fclient.agent_stub.PostData(
                        Datapoint(
                            stream=formant_stream,
                            point_cloud=ros2_pointcloud2_to_formant_pointcloud(msg),
                            timestamp=msg_timestamp,
                        )
                    )
                except grpc.RpcError as e:
                    return
                except Exception as e:
                    print("ERROR ingesting " + formant_stream + ": " + str(e))
                    return

                else:  
                    # Ingest any messages without a direct mapping to a Formant type as JSON
                    self.fclient.post_json(formant_stream, message_to_json(msg))
        except AttributeError as e:
            print("ERROR ingesting " + formant_stream + ": " + str(e))
            
    def handle_formant_teleop_msg(self, msg):
        # Buttons always publish to the "Buttons" stream, so get actual name to use instead
        if msg.stream == "Buttons":
            # Get the key, which is actually the "stream name" equivalent here
            stream_name = msg.bitset.bits[0].key
        else:
            stream_name = msg.stream

        # There can be more than one publisher for a single stream, so loop over them
        if stream_name in self.ros2_publishers:
            for publisher in self.ros2_publishers[stream_name]:
                # Get the ROS2 message type as a string
                ros2_msg_type = publisher.msg_type.__name__

                # Switch on the Formant message type
                if msg.HasField("bitset"):
                    # In a bitset, the value either exists (true) or doesn't (false)
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
                        print("WARNING: Unsupported ROS2 message type for bitset: " + ros2_msg_type)
                        continue

                    publisher.publish(ros2_msg)

                elif msg.HasField("numeric"):
                    msg_value = msg.numeric.value

                    # TODO: this may not be correctly converting all numeric types
                    # Select the configured output type
                    if ros2_msg_type == "Float32":
                        ros2_msg = Float32()
                        ros2_msg.data = float(msg_value)
                    elif ros2_msg_type == "Float64":
                        ros2_msg = Float64()
                        ros2_msg.data = float(msg_value)
                    elif ros2_msg_type == "Int8":
                        ros2_msg = Int8()
                        ros2_msg.data = int(msg_value)
                    elif ros2_msg_type == "Int16":
                        ros2_msg = Int16()
                        ros2_msg.data = int(msg_value)
                    elif ros2_msg_type == "Int32":
                        ros2_msg = Int32()
                        ros2_msg.data = int(msg_value)
                    elif ros2_msg_type == "Int64":
                        ros2_msg = Int64()
                        ros2_msg.data = int(msg_value)
                    elif ros2_msg_type == "UInt8":
                        ros2_msg = UInt8()
                        ros2_msg.data = int(msg_value)
                    elif ros2_msg_type == "UInt16":
                        ros2_msg = UInt16()
                        ros2_msg.data = int(msg_value)
                    elif ros2_msg_type == "UInt32":
                        ros2_msg = UInt32()
                        ros2_msg.data = int(msg_value)
                    elif ros2_msg_type == "UInt64":
                        ros2_msg = UInt64()
                        ros2_msg.data = int(msg_value)
                    elif ros2_msg_type == "String":
                        ros2_msg = String()
                        ros2_msg.data = str(msg_value)
                    else:
                        print("WARNING: Unsupported ROS2 message type for numeric: " + ros2_msg_type)
                        continue
                    
                    publisher.publish(ros2_msg)

                elif msg.HasField("point"):
                    print("WARNING: point is not yet supported")
                elif msg.HasField("pose"):
                    print("WARNING: pose is not yet supported")
                elif msg.HasField("pose_with_covariance"):
                    print("WARNING: pose_with_covariance is not yet supported")
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
                            msg.twist.angular.z
                        ]
                    else:
                        print("WARNING: Unsupported ROS2 message type for twist: " + ros2_msg_type)
                        continue

                    publisher.publish(ros2_msg)

        else:
            print("WARNING: No ROS2 publisher found for stream " + stream_name)

    def handle_formant_command_request_msg(self, msg):
        # Print the message
        print(msg)

        # Post a response
        self.fclient.send_command_response(msg.id, success=True)

    # def handle_teleop(self, msg):
    #     try:
    #         if msg.stream.casefold() == "joystick".casefold():
    #             if not self.joystick_publisher:
    #                 self.joystick_publisher = self.ros2_node.create_publisher(
    #                     Twist, 
    #                     TELEOP_JOYSTICK_TOPIC, 
    #                     10
    #                 )
    #             else:
    #                 self.publish_twist(msg.twist, self.joystick_publisher)

    #         elif msg.stream.casefold() == "buttons".casefold():
    #             button_topic = "/formant/" + str(msg.bitset.bits[0].key)

    #             if not button_topic in self.button_publishers:
    #                 self.button_publishers[button_topic] = self.ros2_node.create_publisher(Bool, button_topic, 10)
    #             else:
    #                 self.publish_bool(msg.bitset.bits[0], self.button_publishers[button_topic])

    #     except Exception as e:
    #         self.fclient.post_text("adapter.errors", "Error handling teleop: %s" %  str(e))

    
    # def publish_twist(self, value, publisher):
    #     msg = Twist()
    #     msg.linear.x = value.linear.x
    #     msg.linear.y = value.linear.y
    #     msg.linear.z = value.linear.z
    #     msg.angular.x = value.angular.x
    #     msg.angular.y = value.angular.y
    #     msg.angular.z = value.angular.z

    #     publisher.publish(msg)

    # def publish_bool(self, value, publisher):
    #     msg = Bool()

    #     if value.value:
    #         msg.data = True
        
    #     publisher.publish(msg)

if __name__ == "__main__":
    try:
        ROS2Adapter()
    except KeyboardInterrupt:
        exit()
