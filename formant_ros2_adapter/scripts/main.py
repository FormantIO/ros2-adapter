#!/usr/bin/env python3

import os
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
from formant.sdk.agent.v1.localization.types import (
    PointCloud as FPointCloud,
    Map as FMap,
    Path as FPath,
    Transform as FTransform,
    Goal as FGoal,
    Odometry as FOdometry,

)
import rclpy
from cv_bridge import CvBridge
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
from nav_msgs.msg import (
    Odometry,
    OccupancyGrid,
    Path
)
from geometry_msgs.msg import (
    Twist,
    PoseStamped
)

from message_utils.utils import (
    get_message_type_from_string,
    message_to_json,
    get_message_path_value,
)

TELEOP_JOYSTICK_TOPIC = "/formant/cmd_vel"
BASE_REFERENCE_FRAME = "map"
class Adapter:
    """
    Formant <-> ROS2 Adapter
    """

    def __init__(self):
        # For console output acknowledgement that the script has started running even if it
        # hasn't yet established communication with the Formant agent.
        print("INFO: `main.py` script has started running.")
        print("")

        # Connect to ROS2
        rclpy.init()
        self.cv_bridge = CvBridge()
        self.node = rclpy.create_node("formant_ros2_adapter")

        # Set up the adapter
        self.fclient = FormantAgentClient(ignore_throttled=True, ignore_unavailable=True)
        self.fclient.register_config_update_callback(self.update_adapter_configuration)
        self.fclient.register_teleop_callback(self.handle_teleop)
        self.fclient.register_command_request_callback(self.handle_command_request)
        self._tf_buffer = None
        self._tf_listener = None
        self._setup_trasform_listener()

        self.fclient.create_event("ROS2 Adapter online", notify=False, severity="info")

        while rclpy.ok():
            self.update_types()
            self.update_subscriptions()
            rclpy.spin_once(self.node, timeout_sec=1.0)

        self.node.destroy_node()
        rclpy.shutdown()

    def _setup_trasform_listener(self):
        try:
            from tf2_ros.buffer import Buffer
            from tf2_ros.transform_listener import TransformListener

            self._tf_buffer = Buffer()
            self._tf_listener = TransformListener(self._tf_buffer)
        except Exception as e:
            print("Error setting up tf2_ros transform listener: %s" % str(e))

    def _lookup_transform(self, msg, base_reference_frame):
        if self._tf_buffer is None or self._tf_listener is None:
            return FTransform()
        try:
            transform = self._tf_buffer.lookup_transform(
                base_reference_frame,
                msg.header.frame_id,
                rclpy.time.Time()
            )
            return FTransform.from_ros_transform_stamped(transform)
        except Exception as e:
            print(
                "Error looking up transform between %s and %s: %s, using identity"
                % (base_reference_frame, msg.header.frame_id, str(e))
            )
        return FTransform()


    def update_adapter_configuration(self):
        # Mapping from configured ROS2 topic name to ROS2 message type
        # type: Dict[str, Any]
        self.topic_to_type = {}

        # Mapping from configured ROS2 topic name to this ROS2 node's subscriber
        # type: Dict[str, rclpy.subscription.Subscription]
        self.topic_to_subscription = {}

        # Keeps track of last time published to control publish rate to Formant.
        # type: Dict[str, float]
        self.rate_control_for_topics = {}  

        # Set up teleoperation
        self.joystick_publisher = None
        self.button_publishers = {}

        # Load config from either the agent's json blob or the config.json file
        try:
            config_blob = json.loads(self.fclient.get_config_blob_data())
        except:
            config_blob = {}
            
        if "ros2_adapter_configuration" in config_blob:
            # Check the json blob for a "ros2_adapter_configuration" section and use it first
            self.config = config_blob["ros2_adapter_configuration"]
        else:
            # Otherwise, load from the config.json file shipped with the adapter
            current_directory = os.path.dirname(os.path.realpath(__file__))
            with open(f"{current_directory}/config.json") as f:
                self.config = json.loads(f.read())

        # Validate configuration based on schema
        with open("schema.json") as f:
            self.config_schema = json.load(f)
        
        print("")
        print("Config Schema:")
        print(self.config_schema)
        print("")

        try:
            jsonschema.validate(self.config, self.config_schema)
            print("Validation succeeded.")
            print("")
        except Exception as e:
            print("Validation failed:")
            print(e)
            print("")

        print("Config:")
        print(str(self.config))
        print("")

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
            localization_stream_name = None
            localization_manager = None
            base_reference_frame = None

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

                if "localization" in config:
                    if config["localization"]:
                        localization_stream_name = stream
                        localization_manager = self.fclient.get_localization_manager(localization_stream_name)
                        base_reference_frame = config.get("base_reference_frame", BASE_REFERENCE_FRAME)


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
                    # Had to install gir1.2-gtk-3.0 package for this to work
                    cv_image = self.cv_bridge.imgmsg_to_cv2(message, "bgr8")
                    encoded_image = cv2.imencode(".jpg", cv_image)[1].tobytes()
                    self.fclient.post_image(stream, encoded_image)

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
                        point_cloud = FPointCloud.from_ros_laserscan(
                                    message
                                )
                        if localization_manager is not None:
                            point_cloud.transform_to_world = self._lookup_transform(message. base_reference_frame)
                            localization_manager.update_point_cloud(point_cloud, cloud_name=topic)
                        else:
                            self.fclient.agent_stub.PostData(
                                Datapoint(
                                    stream=stream,
                                    point_cloud=point_cloud.to_proto(),
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
                        point_cloud = FPointCloud.from_ros(
                                    message
                                )
                        if localization_manager is not None:
                            point_cloud.transform_to_world = self._lookup_transform(message, base_reference_frame)
                            localization_manager.update_point_cloud(point_cloud, cloud_name=topic)
                        else:
                            self.fclient.agent_stub.PostData(
                                Datapoint(
                                    stream=stream,
                                    point_cloud=point_cloud.to_proto(),
                                    timestamp=int(time.time() * 1000),
                                )
                            )
                    except grpc.RpcError as e:
                        return
                    except Exception as e:
                        print("Error ingesting " + stream + ": " + str(e))
                        return
                elif type(message) == Odometry:
                    try:
                        if localization_manager is not None:
                            odometry = FOdometry.from_ros(message)
                            odometry.transform_to_world = self._lookup_transform(message, base_reference_frame)
                            localization_manager.update_odometry(odometry)
                    except grpc.RpcError as e:
                        return
                    except Exception as e:
                        print("Error ingesting " + stream + ": " + str(e))
                        return
                elif type(message) == Path:
                    try:
                        if localization_manager is not None:
                            path = FPath.from_ros(message)
                            path.transform_to_world = self._lookup_transform(message, base_reference_frame)
                            localization_manager.update_path(path)
                    except grpc.RpcError as e:
                        return
                    except Exception as e:
                        print("Error ingesting " + stream + ": " + str(e))
                        return
                elif type(message) == OccupancyGrid:
                    try:
                        if localization_manager is not None:
                            map = FMap.from_ros(message)
                            map.transform_to_world = self._lookup_transform(message, base_reference_frame)
                            localization_manager.update_map(map)
                    except grpc.RpcError as e:
                        return
                    except Exception as e:
                        print("Error ingesting " + stream + ": " + str(e))
                        return

                elif type(message) == PoseStamped:
                    try:
                        if localization_manager is not None:
                            goal = FGoal.from_ros(message)
                            goal.transform_to_world = self._lookup_transform(message, base_reference_frame)
                            localization_manager.update_goal(goal)
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

    def handle_teleop(self, msg):
        try:
            if msg.stream.casefold() == "joystick".casefold():
                if not self.joystick_publisher:
                    self.joystick_publisher = self.node.create_publisher(
                        Twist, 
                        TELEOP_JOYSTICK_TOPIC, 
                        10
                    )
                else:
                    self.publish_twist(msg.twist, self.joystick_publisher)

            elif msg.stream.casefold() == "buttons".casefold():
                button_topic = "/formant/" + str(msg.bitset.bits[0].key)

                if not button_topic in self.button_publishers:
                    self.button_publishers[button_topic] = self.node.create_publisher(Bool, button_topic, 10)
                else:
                    self.publish_bool(msg.bitset.bits[0], self.button_publishers[button_topic])

        except Exception as e:
            self.fclient.post_text("adapter.errors", "Error handling teleop: %s" %  str(e))

    def handle_command_request(self, request):
        print(msg)
        self.fclient.send_command_response(request.id, success=True)

    def publish_twist(self, value, publisher):
        msg = Twist()
        msg.linear.x = value.linear.x
        msg.linear.y = value.linear.y
        msg.linear.z = value.linear.z
        msg.angular.x = value.angular.x
        msg.angular.y = value.angular.y
        msg.angular.z = value.angular.z

        publisher.publish(msg)

    def publish_bool(self, value, publisher):
        msg = Bool()

        if value.value:
            msg.data = True
        
        publisher.publish(msg)


if __name__ == "__main__":
    try:
        Adapter()
    except KeyboardInterrupt:
        exit()
