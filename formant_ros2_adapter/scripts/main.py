#!/usr/bin/env python3


import os
import cv2
import time
import copy
import json
import grpc
import jsonschema
import numpy as np
from errno import EILSEQ
from typing import List, Dict
import traceback

from formant.sdk.agent.v1 import Client as FormantAgentClient
from formant.protos.model.v1.datapoint_pb2 import Datapoint
from formant.protos.model.v1.text_pb2 import Text
from formant.sdk.agent.v1.localization.types import (
    PointCloud as FPointCloud,
    Map as FMap,
    Path as FPath,
    Transform as FTransform,
    Goal as FGoal,
    Odometry as FOdometry,
    Vector3 as FVector3,
    Quaternion as FQuaternion,
)
from formant.sdk.agent.adapter_utils.json_schema_validator import JsonSchemaValidator

import rclpy
from rclpy.parameter import Parameter
from rclpy.qos import (
    QoSProfile,
    HistoryPolicy,
    ReliabilityPolicy,
    DurabilityPolicy,
    LivelinessPolicy,
    qos_profile_unknown,
    qos_profile_system_default,
    qos_profile_sensor_data,
    qos_profile_services_default,
    qos_profile_parameters,
    qos_profile_parameter_events,
    qos_profile_action_status_default,
)

from cv_bridge import CvBridge

from tf2_msgs.msg import TFMessage
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
    BatteryState,
    CompressedImage,
    Image,
    Joy,
    LaserScan,
    NavSatFix,
    PointCloud2,
)

from geometry_msgs.msg import (
    Point,
    PointStamped,
    Point32,
    Polygon,
    Pose,
    PoseStamped,
    PoseArray,
    PoseWithCovariance,
    PoseWithCovarianceStamped,
    Twist,
    TwistStamped,
    Vector3,
    Vector3Stamped,
)

from nav_msgs.msg import Odometry, OccupancyGrid, Path

Costmap = None
try:
    from nav2_msgs.msg import Costmap
except ModuleNotFoundError:
    pass

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from message_utils.utils import (
    get_ros2_type_from_string,
    message_to_json,
    get_message_path_value,
)

FORMANT_OVERRIDE_TIMESTAMP = (
    os.getenv("FORMANT_OVERRIDE_TIMESTAMP", "").lower() == "true"
)

ROS2_NUMERIC_TYPES = [
    "Float32",
    "Float64",
    "Int8",
    "Int16",
    "Int32",
    "Int64",
    "UInt8",
    "UInt16",
    "UInt32",
    "UInt64",
]

qos_profile_example_custom = QoSProfile(
    history=HistoryPolicy.KEEP_ALL,
)

QOS_PROFILES = {
    # Profile details: https://github.com/ros2/rmw/blob/humble/rmw/include/rmw/qos_profiles.h
    "UNKNOWN": qos_profile_unknown,
    "SYSTEM_DEFAULT": qos_profile_system_default,
    "SENSOR_DATA": qos_profile_sensor_data,
    "SERVICES_DEFAULT": qos_profile_services_default,
    "PARAMETERS": qos_profile_parameters,
    "PARAMETER_EVENTS": qos_profile_parameter_events,
    # Profile details: https://github.com/ros2/rcl/blob/rolling/rcl_action/include/rcl_action/default_qos.h
    "ACTION_STATUS_DEFAULT": qos_profile_action_status_default,
    # Example profile created above
    "EXAMPLE_CUSTOM": qos_profile_example_custom,
}

# Seconds
SERVICE_CALL_TIMEOUT = 5


class ROS2Adapter:
    """
    Formant <-> ROS2 Adapter
    """

    def __init__(self):
        # For console output acknowledgement that the script has started running even if it
        # hasn't yet established communication with the Formant agent.
        print("INFO: ROS2 Adapter has started")

        # Set up ROS2
        rclpy.init()
        self.cv_bridge = CvBridge()
        self.ros2_node = rclpy.create_node(
            "formant_ros2_adapter",
            allow_undeclared_parameters=True,
            automatically_declare_parameters_from_overrides=True,
        )

        # Set up the config object
        self.config = {}

        # Subscribers and publishers are stored as a dictionary of topics containing a list of objects
        self.ros2_subscribers = {}
        self.ros2_publishers = {}
        self.ros2_service_clients = {}

        # Set up the localization objects
        ###########################################################################################
        # Currently, localization is implemented completely separately from normal streams        #
        # In the future, it will be removed as individual streams will be assembled in the client #
        ###########################################################################################
        self.tf_buffer = None
        self.tf_listener = None
        self.localization_manager = None
        self.localization_odom_sub = None
        self.localization_map_sub = None
        self.localization_path_sub = None
        self.localization_goal_sub = None
        self.localization_point_cloud_subs = []
        self.localization_goal_pub = None
        self.localization_goal_cancel_pub = None
        self.setup_transform_listener()

        # Subscribers for transform tree ingestion
        self.tf_subscribers = []

        # Set up numeric set objects
        ###############################################################################################
        # Currently, numeric sets are different from individual numerics and contain labels and units #
        # In the future, this will be removed as individual streams will be assembled in the client   #
        ###############################################################################################
        self.numeric_set_subscribers = {}
        self.numeric_set_buffer = {}

        # Set up the adapter
        self.fclient = FormantAgentClient(
            ignore_throttled=True, ignore_unavailable=True
        )

        self.configuration_validator = JsonSchemaValidator(
            self.fclient,
            "ros2_adapter_configuration",
            self.update_adapter_configuration,
            validate=False,
        )

        self.fclient.register_teleop_callback(self.handle_formant_teleop_msg)
        self.fclient.register_command_request_callback(
            self.handle_formant_command_request_msg
        )

        # Start spinning
        print("INFO: Starting to spin ROS2 node")
        while rclpy.ok():
            rclpy.spin_once(self.ros2_node, timeout_sec=1.0)

        # Clean up before shutting down
        if self.ros2_node:
            self.ros2_node.destroy_node()

        rclpy.shutdown()

    def update_adapter_configuration(self, config: Dict):
        print("Received Adapter Configuration")

        # Set the config object to the validated configuration
        if "ros2_adapter_configuration" in config:
            # Check the json blob for a "ros2_adapter_configuration" section and use it first
            self.config = config["ros2_adapter_configuration"]
        else:
            self.config = {}
            print("Adapter Config invalid")
            return

        # Get information about the existing ROS2 system
        self.update_ros2_information()
        print("INFO: Updated ROS2 information")

        # Fill out the config with default values
        for subscriber_config in self.config.get("subscribers", []):
            if "ros2_topic" not in subscriber_config:
                print(
                    "Skipping, 'ros2_topic' not found in: %s" % str(subscriber_config)
                )
                continue
            topic = subscriber_config["ros2_topic"]
            if "formant_stream" not in subscriber_config:
                formant_stream = topic[1:].replace("/", ".")
                subscriber_config["formant_stream"] = formant_stream

            if "ros2_message_type" not in subscriber_config:
                # Get the message type from the topic
                try:
                    subscriber_config[
                        "ros2_message_type"
                    ] = self.ros2_topic_names_and_types[topic]
                except:
                    print(
                        "WARNING: Setting type bool for unknown topic",
                        topic,
                    )
                    subscriber_config["ros2_message_type"] = "std_msgs/msg/Bool"

            if "tags" not in subscriber_config:
                subscriber_config["tags"] = []

        print("INFO: Updated config with default values")
        try:
            # Set each of the app config parameters as a ros2 parameter with a "formant" prefix
            self.setup_ros2_params()
            print("INFO: Finished setting up ROS2 parameters")

            self.setup_subscribers()
            print("INFO: Finished setting up subscribers")

            self.setup_publishers()
            print("INFO: Finished setting up publishers")

            self.setup_service_clients()
            print(
                f"INFO: Finished setting up service calls\n{self.ros2_service_clients}"
            )

            # TODO: once localization visualization has been shifted to universe, this will be removed
            self.setup_localization()
            print("INFO: Finished setting up localization")

            self.setup_tf_ingestion()
            print("INFO: Finished setting up tf ingestion")

            # TODO: once numeric sets can be built from the client, this will be removed
            self.setup_numeric_set_subscribers()
            print("INFO: Finished setting up numeric sets")

            self.fclient.post_json("adapter.configuration", json.dumps(self.config))
            self.fclient.create_event(
                "ROS2 Adapter started",
                notify=False,
                severity="info",
            )
            print("INFO: Posted update event and config")
        except Exception as e:
            print("Exception setting up: %s" % str(e))

    def update_ros2_information(self):
        # Update knowledge about topics and services
        self.ros2_topic_names_and_types = {}
        ros2_topic_names_and_types = self.ros2_node.get_topic_names_and_types()
        for topic in ros2_topic_names_and_types:
            self.ros2_topic_names_and_types[topic[0]] = topic[1][
                0
            ]  # Just use the first one

        self.ros2_service_names_and_types = {}
        ros2_service_names_and_types = self.ros2_node.get_service_names_and_types()
        for service in ros2_service_names_and_types:
            self.ros2_service_names_and_types[service[0]] = service[1][
                0
            ]  # Just use the first one

    def setup_ros2_params(self):
        # TODO: these remain in the ROS2 node after being removed from agent config

        # Loop through all app config params and set them as ros2 parameters
        new_params = []
        for param_key in self.fclient._app_config.keys():
            param_value = self.fclient._app_config[param_key]
            # self.ros2_node.declare_parameter(f"formant.{param_key}", param_value)
            new_params.append(
                Parameter(
                    param_key,
                    Parameter.Type.STRING,
                    self.fclient._app_config[param_key],
                )
            )

        print("INFO: Setting new ROS2 parameters")
        self.ros2_node.set_parameters(new_params)

    def setup_subscribers(self):
        # Remove any existing subscribers
        for subscriber_list in self.ros2_subscribers.values():
            for subscriber in subscriber_list:
                self.ros2_node.destroy_subscription(subscriber)

        self.ros2_subscribers = {}

        # Create new subscribers based on the config
        for subscriber_config in self.config.get("subscribers", []):
            subscriber_topic = subscriber_config["ros2_topic"]
            print(f"Subscriber for topic: {subscriber_topic}")

            if "ros2_qos_profile" in subscriber_config:
                subscriber_qos = QOS_PROFILES.get(
                    subscriber_config["ros2_qos_profile"], qos_profile_system_default
                )
            else:
                subscriber_qos = qos_profile_system_default
            print(f"QoS: {subscriber_qos}, {type(subscriber_qos)}")

            new_sub = self.ros2_node.create_subscription(
                msg_type=get_ros2_type_from_string(
                    subscriber_config["ros2_message_type"]
                ),
                topic=subscriber_topic,
                callback=lambda msg, subscriber_config=subscriber_config: self.handle_ros2_message(
                    msg, subscriber_config
                ),
                qos_profile=subscriber_qos,
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
        for publisher in self.config.get("publishers", []):
            # If there is no type, get it from the topic... but this shouldn't happen
            if "ros2_message_type" not in publisher:
                if publisher["ros2_topic"] in self.ros2_topic_names_and_types:
                    publisher["ros2_message_type"] = self.ros2_topic_names_and_types[
                        publisher["ros2_topic"]
                    ]
                else:
                    print("ERROR: No message type for topic", publisher["ros2_topic"])
                    continue

            publisher_topic = publisher["ros2_topic"]
            print(f"Publisher for topic: {publisher_topic}")

            if "ros2_qos_profile" in publisher:
                publisher_qos = QOS_PROFILES.get(
                    publisher["ros2_qos_profile"], qos_profile_system_default
                )
            else:
                publisher_qos = qos_profile_system_default
            print(f"QoS: {publisher_qos}, {type(publisher_qos)}")

            new_pub = self.ros2_node.create_publisher(
                msg_type=get_ros2_type_from_string(publisher["ros2_message_type"]),
                topic=publisher_topic,
                qos_profile=publisher_qos,
            )

            if publisher["formant_stream"] not in self.ros2_publishers:
                self.ros2_publishers[publisher["formant_stream"]] = []

            self.ros2_publishers[publisher["formant_stream"]].append(new_pub)

    def setup_service_clients(self):
        # Clean up existing service clients before setting up new ones
        for keys in self.ros2_service_clients.keys():
            for service_client in self.ros2_service_clients[keys]:
                service_client.destroy()

        self.ros2_service_clients = {}
        print("INFO: Cleaned up existing service clients")

        # Set up service calls
        for service_client in self.config.get("service_clients", []):
            try:
                service_type_string = self.ros2_service_names_and_types.get(
                    service_client["ros2_service"], service_client["ros2_service_type"]
                )
                service_type = get_ros2_type_from_string(service_type_string)
                print("INFO: Found service of type", service_type_string)
            except Exception as e:
                print(
                    "WARNING: Could not determine service type for service "
                    + service_client["ros2_service"]
                )
                traceback.print_exc()
                continue

            # If a type has been specified, make sure it matches
            if "ros2_service_type" in service_client:
                if service_client["ros2_service_type"] != service_type_string:
                    print(
                        "WARNING: Service "
                        + service_client["ros2_service"]
                        + " does not match specified type"
                    )
                    continue
                else:
                    print(
                        "INFO: Service "
                        + service_client["ros2_service"]
                        + " matches specified type"
                    )

            try:
                new_service_client = self.ros2_node.create_client(
                    srv_type=service_type,
                    srv_name=service_client["ros2_service"],
                    callback_group=None,
                )

                if service_client["formant_stream"] not in self.ros2_service_clients:
                    self.ros2_service_clients[service_client["formant_stream"]] = []

                self.ros2_service_clients[service_client["formant_stream"]].append(
                    new_service_client
                )

                print(
                    "INFO: Set up service client for " + service_client["ros2_service"]
                )
            except Exception as e:
                print(
                    "WARNING: Failed to set up service client for "
                    + service_client["ros2_service"]
                )
                print(e)

    def setup_localization(self):
        print("INFO: Setting up localization")
        # Remove any existing localization subscribers and publishers
        if self.localization_odom_sub is not None:
            self.ros2_node.destroy_subscription(self.localization_odom_sub)
            self.localization_odom_sub = None

        if self.localization_map_sub is not None:
            self.ros2_node.destroy_subscription(self.localization_map_sub)
            self.localization_map_sub = None

        for point_cloud_sub in self.localization_point_cloud_subs:
            self.ros2_node.destroy_subscription(point_cloud_sub)

        self.localization_point_cloud_subs = []

        if self.localization_path_sub is not None:
            self.ros2_node.destroy_subscription(self.localization_path_sub)
            self.localization_path_sub = None

        print("INFO: Destroyed existing localization subscribers")

        if self.localization_goal_pub is not None:
            self.ros2_node.destroy_publisher(self.localization_goal_pub)
            self.localization_goal_pub = None

        if self.localization_goal_cancel_pub is not None:
            self.ros2_node.destroy_publisher(self.localization_goal_cancel_pub)
            self.localization_goal_cancel_pub = None

        print("INFO: Destroyed existing localization publishers")

        # Skip this if there is no localization config
        if "localization" not in self.config:
            print("INFO: No localization configuration")
            return

        localization_config = self.config["localization"]
        print("INFO: Set localization config")

        # Make sure the config has all required fields
        if (
            "formant_stream" not in localization_config
            or "base_reference_frame" not in localization_config
            or "odometry_subscriber_ros2_topic" not in localization_config
        ):
            print("WARNING: Localization config is missing required fields")
            return
        print("INFO: Checked localization config for required fields")

        # Set up the localization manager
        self.localization_manager = self.fclient.get_localization_manager(
            localization_config["formant_stream"]
        )
        print("INFO: Set up localization manager")

        # Set up subscribers
        try:
            odom_type = get_ros2_type_from_string(
                self.ros2_topic_names_and_types[
                    localization_config["odometry_subscriber_ros2_topic"]
                ]
            )
            self.localization_odom_sub = self.ros2_node.create_subscription(
                odom_type,
                localization_config["odometry_subscriber_ros2_topic"],
                self.localization_odom_callback,
                qos_profile_sensor_data,
            )
            print("INFO: Set up localization odom subscriber")
        except Exception as e:
            print("WARNING: Failed to set up localization odom subscriber")
            print(e)

        try:
            map_type = get_ros2_type_from_string(
                self.ros2_topic_names_and_types[
                    localization_config["map_subscriber_ros2_topic"]
                ]
            )
            self.localization_map_sub = self.ros2_node.create_subscription(
                map_type,
                localization_config["map_subscriber_ros2_topic"],
                self.localization_map_callback,
                qos_profile_sensor_data,
            )
            print("INFO: Set up localization map subscriber")
        except Exception as e:
            print("WARNING: Failed to set up localization map subscriber")
            print(e)

        if "point_cloud_subscriber_ros2_topics" in localization_config:
            self.localization_point_cloud_subs = []
            if "point_cloud_subscriber_ros2_topics" in localization_config:
                for point_cloud_subscriber_ros2_topic in localization_config[
                    "point_cloud_subscriber_ros2_topics"
                ]:
                    try:
                        point_cloud_type = get_ros2_type_from_string(
                            self.ros2_topic_names_and_types[
                                point_cloud_subscriber_ros2_topic
                            ]
                        )
                        new_sub = self.ros2_node.create_subscription(
                            point_cloud_type,
                            point_cloud_subscriber_ros2_topic,
                            self.localization_point_cloud_callback,
                            qos_profile_sensor_data,
                        )
                        self.localization_point_cloud_subs.append(new_sub)
                    except Exception as e:
                        print(
                            "WARNING: Failed to create localization point cloud subscriber"
                        )
                        print(e)

            print("INFO: Set up localization point cloud subscribers")

        if "path_subscriber_ros2_topic" in localization_config:
            try:
                path_type = get_ros2_type_from_string(
                    self.ros2_topic_names_and_types[
                        localization_config["path_subscriber_ros2_topic"]
                    ]
                )

                self.localization_path_sub = self.ros2_node.create_subscription(
                    path_type,
                    localization_config["path_subscriber_ros2_topic"],
                    self.localization_path_callback,
                    qos_profile_sensor_data,
                )
                print("INFO: Set up localization path subscriber")
            except Exception as e:
                print("WARNING: Failed to set up localization path subscriber")
                print(e)

        if "goal_subscriber_ros2_topic" in localization_config:
            try:
                goal_sub_type = get_ros2_type_from_string(
                    self.ros2_topic_names_and_types[
                        localization_config["goal_subscriber_ros2_topic"]
                    ]
                )

                self.localization_goal_sub = self.ros2_node.create_subscription(
                    goal_sub_type,
                    localization_config["goal_subscriber_ros2_topic"],
                    self.localization_goal_callback,
                    qos_profile_sensor_data,
                )
                print("INFO: Set up localization goal subscriber")
            except Exception as e:
                print("WARNING: Failed to set up localization goal subscriber")
                print(e)

        if "goal_publisher_ros2_topic" in localization_config:
            try:
                # If the topic exists, use its type
                if (
                    localization_config["goal_publisher_ros2_topic"]
                    in self.ros2_topic_names_and_types
                ):
                    goal_pub_type = get_ros2_type_from_string(
                        self.ros2_topic_names_and_types[
                            localization_config["goal_publisher_ros2_topic"]
                        ]
                    )
                else:
                    # Otherwise, use the default type
                    goal_pub_type = PoseStamped

                self.localization_goal_pub = self.ros2_node.create_publisher(
                    goal_pub_type,
                    localization_config["goal_publisher_ros2_topic"],
                    qos_profile_system_default,
                )
                print("INFO: Set up localization goal publisher")
            except Exception as e:
                print("WARNING: Failed to set up localization goal publisher")
                print(e)

        if "cancel_goal_publisher_ros2_topic" in localization_config:
            try:
                # If the topic exists, use its type
                if (
                    localization_config["cancel_goal_publisher_ros2_topic"]
                    in self.ros2_topic_names_and_types
                ):
                    cancel_goal_pub_type = get_ros2_type_from_string(
                        self.ros2_topic_names_and_types[
                            localization_config["cancel_goal_publisher_ros2_topic"]
                        ]
                    )
                else:
                    # Otherwise, use the default type
                    cancel_goal_pub_type = Bool

                self.localization_goal_cancel_pub = self.ros2_node.create_publisher(
                    cancel_goal_pub_type,
                    localization_config["cancel_goal_publisher_ros2_topic"],
                    qos_profile_system_default,
                )
                print("INFO: Set up localization cancel goal publisher")
            except Exception as e:
                print("WARNING: Failed to set up localization cancel goal publisher")
                print(e)

    def setup_tf_ingestion(self):
        print("INFO: Setting up tf ingestion")
        if "transform_tree" not in self.config:
            print("INFO: TF ingestion not setup.")
            return
        for subscriber in self.tf_subscribers:
            try:
                subscriber.destroy_subscriber()
            except:
                pass
        self.tf_subscribers = []

        print("INFO: Destroyed existing tf subscribers")
        base_reference_frame = self.config["transform_tree"]["base_reference_frame"]
        print("INFO: tf base reference frame: %s" % base_reference_frame)

        self.fclient.set_base_frame_id(base_reference_frame)

        for topic in ["/tf", "/tf_static"]:
            self.ros2_node.create_subscription(
                TFMessage,
                topic,
                self.tf_callback,
                qos_profile_sensor_data,
            )
        print("INFO: Subscribed to tf topics")

    def setup_numeric_set_subscribers(self):
        print("INFO: Setting up numeric sets")

        # Remove any existing numeric set subscribers and clear the buffer
        for subscriber_list in self.numeric_set_subscribers.values():
            for subscriber in subscriber_list:
                try:
                    subscriber.destroy_subscriber()
                except:
                    pass
        self.numeric_set_subscribers = {}
        self.numeric_set_buffer = {}

        print("INFO: Destroyed existing numeric set subscribers")

        # Create new numeric set subscribers
        if "numeric_sets" not in self.config:
            print("INFO: No numeric sets to set up")
            return

        for numeric_set_config in self.config["numeric_sets"]:
            formant_stream = numeric_set_config["formant_stream"]
            self.numeric_set_subscribers[formant_stream] = []
            print("INFO: Setting up numeric set for formant stream: " + formant_stream)

            for ros2_subscriber_config in numeric_set_config["ros2_subscribers"]:
                try:
                    ros2_topic = ros2_subscriber_config["ros2_topic"]

                    if ros2_topic in self.ros2_topic_names_and_types:
                        ros2_message_type = get_ros2_type_from_string(
                            self.ros2_topic_names_and_types[ros2_topic]
                        )
                    else:
                        print("WARNING: ROS2 topic " + ros2_topic + " does not exist")
                        continue

                    # Create a new subscriber
                    new_numeric_set_sub = self.ros2_node.create_subscription(
                        ros2_message_type,
                        ros2_topic,
                        lambda msg, stream=formant_stream, config=ros2_subscriber_config: self.handle_ros2_numeric_set_message(
                            msg, stream, config
                        ),
                        qos_profile_sensor_data,
                    )

                    self.numeric_set_subscribers[formant_stream].append(
                        new_numeric_set_sub
                    )
                except Exception as e:
                    print("WARNING: Failed to create numeric set subscriber")
                    print(e)

    def tf_callback(self, msg):
        for tf in msg.transforms:
            parent_frame = tf.header.frame_id
            child_frame = tf.child_frame_id
            tx = tf.transform.translation.x
            ty = tf.transform.translation.y
            tz = tf.transform.translation.z
            rx = tf.transform.rotation.x
            ry = tf.transform.rotation.y
            rz = tf.transform.rotation.z
            rw = tf.transform.rotation.w
            self.fclient.post_transform_frame(
                parent_frame, child_frame, tx, ty, tz, rx, ry, rz, rw
            )

    # To do: use Agent SDK localization_manager for PoseWithCovarianceStamp
    def localization_odom_callback(self, msg):
        msg_type = type(msg)
        if msg_type == Odometry:
            odometry = FOdometry.from_ros(msg)
            odometry.transform_to_world = self.lookup_transform(
                msg, self.config["localization"]["base_reference_frame"]
            )
            self.localization_manager.update_odometry(odometry)
        elif msg_type == PoseWithCovarianceStamped:
            print("PoseWithCovarianceStamped message type")

            # ROS types
            ros_pose = msg.pose.pose

            # Formant types
            odometry = FOdometry(pose=FTransform.from_ros_pose(ros_pose))
            odometry.transform_to_world = self.lookup_transform(
                msg, self.config["localization"]["base_reference_frame"]
            )

            self.localization_manager.update_odometry(odometry)
        else:
            print("WARNING: Unknown odom type", msg_type)

    # To do: use Agent SDK localization_manager for Costmap
    def localization_map_callback(self, msg):
        msg_type = type(msg)
        if msg_type is OccupancyGrid:
            formant_map = FMap.from_ros(msg)
            formant_map.transform_to_world = self.lookup_transform(
                msg, self.config["localization"]["base_reference_frame"]
            )
            self.localization_manager.update_map(formant_map)
        elif Costmap is not None and msg_type is Costmap:
            print("Costmap message type")

            # ROS types
            ros_resolution = msg.metadata.resolution
            ros_width = msg.metadata.size_x
            ros_height = msg.metadata.size_y
            ros_origin = msg.metadata.origin
            ros_origin_position = ros_origin.position
            ros_origin_orientation = ros_origin.orientation

            # Formant types
            formant_map = FMap(
                resolution=ros_resolution,
                width=ros_width,
                height=ros_height,
                origin=FTransform.from_ros_pose(ros_origin),
                occupancy_grid_data=msg.data,
            )

            self.localization_manager.update_map(formant_map)
        else:
            print("WARNING: Unknown map type", msg_type)

    def localization_point_cloud_callback(self, msg):
        # Check to see if the point cloud is in laser scan or pointcloud2 format
        msg_type = type(msg)
        if msg_type == LaserScan:
            point_cloud = FPointCloud.from_ros_laserscan(msg)

        elif msg_type == PointCloud2:
            point_cloud = FPointCloud.from_ros(msg)

        else:
            print("ERROR: Unknown point cloud type", msg_type)
            return

        point_cloud.transform_to_world = self.lookup_transform(
            msg, self.config["localization"]["base_reference_frame"]
        )

        if msg.header.frame_id != None:
            point_cloud_name = msg.header.frame_id
        else:
            point_cloud_name = "point_cloud"  # TODO: should get something unique here, like the topic name

        self.localization_manager.update_point_cloud(point_cloud, point_cloud_name)

    def localization_path_callback(self, msg):
        msg_type = type(msg)
        if msg_type == Path:
            path = FPath.from_ros(msg)
            path.transform_to_world = self.lookup_transform(
                msg, self.config["localization"]["base_reference_frame"]
            )
            self.localization_manager.update_path(path)
        else:
            print("WARNING: Unknown path type", msg_type)

    def localization_goal_callback(self, msg):
        msg_type = type(msg)
        if msg_type == PoseStamped:
            goal = FGoal.from_ros(msg)
            goal.transform_to_world = self.lookup_transform(
                msg, self.config["localization"]["base_reference_frame"]
            )
            self.localization_manager.update_goal(goal)
        else:
            print("WARNING: Unknown goal type", msg_type)

    def handle_ros2_message(self, msg, subscriber_config):
        try:
            # Get the message type
            msg_type = type(msg)
            formant_stream = subscriber_config["formant_stream"]
            ros2_topic = subscriber_config["ros2_topic"]

            # Select the part of the message based on the path
            if "ros2_message_paths" in subscriber_config:
                for path_config in subscriber_config["ros2_message_paths"]:
                    try:
                        path_msg = get_message_path_value(msg, path_config["path"])

                        # Write subscriber_config for this message
                        path_subscriber_config = copy.deepcopy(subscriber_config)
                        del path_subscriber_config["ros2_message_paths"]

                        # Pass tags to the path subscriber config
                        if "tag_key" in path_config and "tag_value" in path_config:
                            path_subscriber_config["tags"] = {
                                path_config["tag_key"]: path_config["tag_value"]
                            }

                        # Recursively call this function with the message in the path
                        self.handle_ros2_message(path_msg, path_subscriber_config)

                    except:
                        # If this path does not match, ignore it and log the error
                        print(
                            f"ERROR: Could not find path '{path_config['path']}' in message {msg_type}"
                        )
                        pass

            msg_timestamp = int(time.time() * 1000)
            if hasattr(msg, "header"):
                if not FORMANT_OVERRIDE_TIMESTAMP:
                    header_timestamp = (
                        msg.header.stamp.sec * 1000 + msg.header.stamp.nanosec / 1000000
                    )
                    # sanity check to make sure ros header stamp is in epoch time
                    if header_timestamp > 1500000000000:
                        msg_timestamp = int(header_timestamp)

            # Handle the message based on its type
            try:
                if msg_type in [str, String, Char]:
                    if hasattr(msg, "data"):
                        msg = msg.data

                    self.fclient.post_text(
                        formant_stream,
                        str(msg),
                        tags=subscriber_config["tags"],
                        timestamp=msg_timestamp,
                    )

                elif msg_type in [Bool, bool]:
                    if hasattr(msg, "data"):
                        msg = msg.data

                    self.fclient.post_bitset(
                        formant_stream,
                        {ros2_topic: msg},
                        tags=subscriber_config["tags"],
                        timestamp=msg_timestamp,
                    )

                elif msg_type in [
                    int,
                    float,
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
                    if hasattr(msg, "data"):
                        msg = msg.data

                    self.fclient.post_numeric(
                        formant_stream,
                        msg,
                        tags=subscriber_config["tags"],
                        timestamp=msg_timestamp,
                    )

                elif msg_type == NavSatFix:
                    # Convert NavSatFix to a Formant location
                    self.fclient.post_geolocation(
                        stream=formant_stream,
                        latitude=msg.latitude,
                        longitude=msg.longitude,
                        altitude=msg.altitude,
                        tags=subscriber_config["tags"],
                        timestamp=msg_timestamp,
                    )

                elif msg_type == Image:
                    # Convert Image to a Formant image
                    cv_image = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")
                    encoded_image = cv2.imencode(".jpg", cv_image)[1].tobytes()

                    self.fclient.post_image(
                        stream=formant_stream,
                        value=encoded_image,
                        tags=subscriber_config["tags"],
                        timestamp=msg_timestamp,
                    )

                elif msg_type == CompressedImage:
                    # Post the compressed image
                    if "jpg" in msg.format or "jpeg" in msg.format:
                        content_type = "image/jpg"
                    elif "png" in msg.format:
                        content_type = "image/png"
                    else:
                        print("WARNING: Image format", msg.format, "not supported")
                        return
                    self.fclient.post_image(
                        formant_stream,
                        value=bytes(msg.data),
                        content_type=content_type,
                        tags=subscriber_config["tags"],
                        timestamp=msg_timestamp,
                    )

                elif msg_type == BatteryState:
                    self.fclient.post_battery(
                        formant_stream,
                        msg.percentage,
                        voltage=msg.voltage,
                        current=msg.current,
                        charge=msg.charge,
                        tags=subscriber_config["tags"],
                        timestamp=msg_timestamp,
                    )

                elif msg_type == LaserScan:
                    # Convert LaserScan to a Formant pointcloud
                    try:
                        self.fclient.agent_stub.PostData(
                            Datapoint(
                                stream=formant_stream,
                                point_cloud=FPointCloud.from_ros_laserscan(
                                    msg
                                ).to_proto(),
                                tags=subscriber_config["tags"],
                                timestamp=msg_timestamp,
                            )
                        )
                    except grpc.RpcError as e:
                        return
                    except Exception as e:
                        print(
                            "ERROR: Could not ingest " + formant_stream + ": " + str(e)
                        )
                        return

                elif msg_type == PointCloud2:
                    try:
                        self.fclient.agent_stub.PostData(
                            Datapoint(
                                stream=formant_stream,
                                point_cloud=FPointCloud.from_ros(msg).to_proto(),
                                tags=subscriber_config["tags"],
                                timestamp=msg_timestamp,
                            )
                        )
                    except grpc.RpcError as e:
                        return
                    except Exception as e:
                        print(
                            "ERROR: Could not ingest " + formant_stream + ": " + str(e)
                        )
                        return

                else:
                    # Ingest any messages without a direct mapping to a Formant type as JSON
                    self.fclient.post_json(
                        formant_stream,
                        message_to_json(msg),
                        tags=subscriber_config["tags"],
                        timestamp=msg_timestamp,
                    )
            except AttributeError as e:
                print("ERROR: Could not ingest " + formant_stream + ": " + str(e))
        except Exception as e:
            print("Error handling ros2 msg: %s" % str(traceback.format_exc()))

    def handle_ros2_numeric_set_message(self, msg, formant_stream, subscriber_config):
        # Get the ROS2 topic name
        ros2_topic = subscriber_config["ros2_topic"]

        # Get the original message type
        msg_type = type(msg)

        path = ""
        # If there is a path, get the value from the path
        if "ros2_message_path" in subscriber_config:
            path = subscriber_config["ros2_message_path"]

            try:
                path_msg = get_message_path_value(msg, path)
                msg = path_msg

                # Re-set the message type if the path is a different type
                msg_type = type(msg)
            except:
                # If this path does not match, ignore it and log the error
                print(f"ERROR: Could not find path '{path}' in message {msg_type}")
                return

        # Get the label and unit
        label = subscriber_config.get("label", path)
        unit = subscriber_config.get("unit", "")

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
            print("ERROR: Could not ingest " + formant_stream + ": " + str(e))
            return

        # Update the current set with the new value
        if formant_stream not in self.numeric_set_buffer:
            self.numeric_set_buffer[formant_stream] = {}

        self.numeric_set_buffer[formant_stream][label] = (value, unit)

        self.fclient.post_numericset(
            formant_stream, self.numeric_set_buffer[formant_stream]
        )

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
                    # A button press can only send one bit, so we can just use the first one
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
                        print(
                            "WARNING: Unsupported ROS2 message type for bitset: "
                            + ros2_msg_type
                        )
                        continue

                    publisher.publish(ros2_msg)

                elif msg.HasField("numeric"):
                    msg_value = msg.numeric.value
                    self.publish_ros2_numeric(publisher, ros2_msg_type, msg_value)

                elif msg.HasField("point"):
                    point = Point(x=msg.point.x, y=msg.point.y, z=msg.point.z)
                    if ros2_msg_type == "Point":
                        ros2_msg = point
                    elif ros2_msg == "PointStamped":
                        ros2_msg = PointStamped(point=point)
                    else:
                        self._logger.warn("Unsupported Point Type: %s" % ros2_msg_type)
                    publisher.publish(ros2_msg)

                elif msg.HasField("pose"):
                    print("WARNING: Pose is not yet supported")
                elif msg.HasField("pose_with_covariance"):
                    print("WARNING: Pose_with_covariance is not yet supported")
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
                            msg.twist.angular.z,
                        ]
                    else:
                        print(
                            "WARNING: Unsupported ROS2 message type for twist: "
                            + ros2_msg_type
                        )
                        continue

                    publisher.publish(ros2_msg)

        # Handle service calls
        elif stream_name in self.ros2_service_clients:
            print(f"INFO: Calling service {stream_name}")
            for service_client in self.ros2_service_clients[stream_name]:
                # Get parameter from bitset (button) or numeric (slider/dial) stream
                # Other teleop stream types are not supported
                command_text = ""
                if msg.HasField("bitset"):
                    if msg.bitset.bits[0].value is True:
                        command_text = "True"
                elif msg.HasField("numeric"):
                    # This works, don't need to cast as string or anything
                    command_text = msg.numeric.value
                else:
                    print(
                        "WARNING: Invalid teleop control datapoint "
                        "(requires bitset or numeric)"
                    )
                    continue
                success, service_call_result = self.ros2_service_call(
                    service_client, stream_name, command_text
                )

                # To do: something here to inform Formant like with commands?
                if success is True:
                    print(f"INFO: Service call {stream_name} succeeded")
                else:
                    print(f"WARNING: Service call {stream_name} failed")

    def ros2_service_call(self, service_client, service_command, command_text):
        # Return values
        success = False
        service_result = ""

        # Check if the specified service if it exists
        if service_command not in self.ros2_service_clients:
            service_result = (
                "WARNING: Service not configured for formant stream: "
                + f"{service_request}"
            )
            # Future optimization: streamline code so only have these two lines once
            print(service_result)
            return success, service_result

        # Create the service request
        service_request = service_client.srv_type.Request()
        service_request_slots = list(
            service_request.get_fields_and_field_types().values()
        )

        # We only handle single-param requests for now
        print(f"INFO: Service request slots: {service_request_slots}")
        if len(service_request_slots) > 1:
            service_result = (
                "WARNING: Unsupported service request type for command: "
                + f"{service_command}"
            )
            print(service_result)
            return success, service_result

        # If the service has no parameters, just call it
        if service_request_slots == []:
            pass

        # If the service has a single boolean parameter, call it with "true"
        elif service_request_slots == ["boolean"]:
            # Get the name of the attribute to set from the service request
            service_request_attribute = list(
                service_request.get_fields_and_field_types().keys()
            )[0]

            # Check to see if a parameter was passed in the command text
            if command_text == "":
                service_request_value = True
            elif command_text in ["true", "True", "TRUE", "t", "T", "1"]:
                service_request_value = True
            elif command_text in ["false", "False", "FALSE", "f", "F", "0"]:
                service_request_value = False
            else:
                service_result = (
                    "WARNING: Invalid parameter for boolean service "
                    + f"{service_command}: "
                    + f"{command_text}"
                )
                print(service_result)
                return success, service_result
            # Set the attribute on the request to true
            setattr(
                service_request,
                service_request_attribute,
                service_request_value,
            )

        # If the service has a single string parameter, call it with the command text
        elif service_request_slots == ["string"]:
            # If the command text is empty, don't call the service
            if command_text == "":
                service_result = (
                    "WARNING: "
                    + "Command text is empty but service requires a string parameter"
                )
                return success, service_result
            service_request_attribute = list(
                service_request.get_fields_and_field_types().keys()
            )[0]
            setattr(service_request, service_request_attribute, command_text)

        # If the service has a string list parameter, send the sequence itself
        # (not a string of it)
        elif service_request_slots == ["sequence<string>"]:
            command_text_json = {}
            try:
                command_text_json = json.loads(command_text)
            except json.decoder.JSONDecodeError:
                service_result = (
                    "WARNING: Invalid parameter for string sequence service: "
                    + "not a JSON"
                )
                print(service_result)
                return success, service_result
            if type(command_text_json) is not list:
                service_result = (
                    "WARNING: Invalid parameter for string sequence service: "
                    + "not a list"
                )
                print(service_result)
                return success, service_result
            service_request_attribute = list(
                service_request.get_fields_and_field_types().keys()
            )[0]
            setattr(service_request, service_request_attribute, command_text_json)

        # If the service has a single numeric parameter, call it with the command text
        # Float32, Float64, Int8, Int16, Int32, Int64, UInt8, UInt16, UInt32, UInt64
        elif service_request_slots[0] in [
            "float",
            "float32",
            "float64",
            "int",
            "int8",
            "int16",
            "int32",
            "int64",
            "uint8",
            "uint16",
            "uint32",
            "uint64",
        ]:
            # If the command text is empty, don't call the service
            if command_text == "":
                service_result = (
                    "WARNING: "
                    + "Command text is empty but service requires a numeric parameter"
                )
                print(service_result)
                return success, service_result
            # If the command text is not numeric, don't call the service
            # (this instead of isnumeric() to support decimal numbers)
            try:
                number_check = float(command_text)
            except ValueError:
                service_result = (
                    "WARNING: "
                    + "Command text is not numeric but service requires a numeric parameter"
                )
                print(service_result)
                return success, service_result
            # Get the name of the attribute to set from the service request
            service_request_attribute = list(
                service_request.get_fields_and_field_types().keys()
            )[0]
            # Cast the command value to the type determined by the service request slot
            slot_type = service_request_slots[0]

            # To do:
            # Using numpy types seems like a good idea, but I can't get it working
            # Error:
            # AssertionError: The 'input' field must be of type 'int'
            # if slot_type == "float32":
            #     service_request_value = np.float32(command_text)
            # elif slot_type == "float64":
            #     service_request_value = np.float64(command_text)
            # elif slot_type == "int8":
            #     service_request_value = np.int8(command_text)
            # elif slot_type == "int16":
            #     service_request_value = np.int16(command_text)
            # elif slot_type == "int32":
            #     service_request_value = np.int32(command_text)
            # elif slot_type == "int64":
            #     service_request_value = np.int64(command_text)
            # elif slot_type == "uint8":
            #     service_request_value = np.uint8(command_text)
            # elif slot_type == "uint16":
            #     service_request_value = np.uint16(command_text)
            # elif slot_type == "uint32":
            #     service_request_value = np.uint32(command_text)
            # elif slot_type == "uint64":
            #     service_request_value = np.uint64(command_text)

            if "int" in slot_type:
                # If user sends a float, convert to int
                service_request_value = int(float(command_text))
            elif "float" in slot_type:
                service_request_value = float(command_text)
            else:
                service_result = (
                    "WARNING: Unsupported parameter type for numeric service "
                    + f"{service_command}: "
                    + f"{slot_type}"
                )
                print(service_result)
                return success, service_result
            # Set the attribute on the request to the command text
            setattr(
                service_request,
                service_request_attribute,
                service_request_value,
            )

        else:
            service_result = (
                "WARNING: Unsupported ROS 2 service parameters for command "
                + f"{service_command}"
            )
            print(service_result)
            return success, service_result

        # Check if the service is available
        if service_client.wait_for_service(SERVICE_CALL_TIMEOUT) == False:
            service_result = "WARNING: Timeout waiting for service"
            print(service_result)
            return success, service_result
        # Call the service if the paramaters are valid
        if service_result == "":
            service_result = service_client.call(service_request)
            success = True
            print(f"INFO: Service call result: {service_result}")

        # success is True or False
        # service_result is response from the call if success, error message if fail
        return success, service_result

    def handle_formant_command_request_msg(self, msg):
        print(f"INFO: Formant command received:\n{msg}")

        # Publish message on topic if a publisher exists for this command
        if msg.command in self.ros2_publishers:
            for publisher in self.ros2_publishers[msg.command]:
                # Get the ROS2 message type as a string
                ros2_msg_type = publisher.msg_type.__name__
                if ros2_msg_type == "String":
                    ros2_msg = String()
                    ros2_msg.data = msg.text
                    publisher.publish(ros2_msg)
                    self.fclient.send_command_response(msg.id, success=True)
                elif (ros2_msg_type in ROS2_NUMERIC_TYPES) and msg.text.isnumeric():
                    self.publish_ros2_numeric(publisher, ros2_msg_type, msg.text)
                    self.fclient.send_command_response(msg.id, success=True)
                else:
                    print(
                        "WARNING: Unsupported ROS2 message type for command: "
                        f"{ros2_msg_type}"
                    )
                    self.fclient.send_command_response(msg.id, success=False)
                    continue
        print("INFO: %s" % str(self.ros2_service_clients))
        # Call a service if a service exists for this command
        if msg.command in self.ros2_service_clients:
            print(f"INFO: Calling service {msg.command}")
            for service_client in self.ros2_service_clients[msg.command]:
                success, service_call_result = self.ros2_service_call(
                    service_client, msg.command, msg.text
                )

                if success is True:
                    print(f"INFO: Service call {msg.command} succeeded")
                else:
                    print(f"WARNING: Service call {msg.command} failed")
                msg_timestamp = int(time.time() * 1000)
                self.fclient.send_command_response(
                    request_id=msg.id,
                    success=success,
                    datapoint=Datapoint(
                        stream="ros2.service_call.response",
                        text=Text(value=str(service_call_result)),
                        timestamp=msg_timestamp,
                    ),
                )

    def publish_ros2_numeric(self, publisher, ros2_msg_type, msg_value):
        if ros2_msg_type == "Float32":
            ros2_msg = Float32()
            ros2_msg.data = np.float32(msg_value).item()
        elif ros2_msg_type == "Float64":
            ros2_msg = Float64()
            ros2_msg.data = np.float64(msg_value).item()
        elif ros2_msg_type == "Int8":
            ros2_msg = Int8()
            ros2_msg.data = np.int8(msg_value).item()
        elif ros2_msg_type == "Int16":
            ros2_msg = Int16()
            ros2_msg.data = np.int16(msg_value).item()
        elif ros2_msg_type == "Int32":
            ros2_msg = Int32()
            ros2_msg.data = np.int32(msg_value).item()
        elif ros2_msg_type == "Int64":
            ros2_msg = Int64()
            ros2_msg.data = np.int64(msg_value).item()
        elif ros2_msg_type == "UInt8":
            ros2_msg = UInt8()
            ros2_msg.data = np.uint8(msg_value).item()
        elif ros2_msg_type == "UInt16":
            ros2_msg = UInt16()
            ros2_msg.data = np.uint16(msg_value).item()
        elif ros2_msg_type == "UInt32":
            ros2_msg = UInt32()
            ros2_msg.data = np.uint32(msg_value).item()
        elif ros2_msg_type == "UInt64":
            ros2_msg = UInt64()
            ros2_msg.data = np.uint64(msg_value).item()
        elif ros2_msg_type == "String":
            ros2_msg = String()
            ros2_msg.data = str(msg_value)
        else:
            print(
                "WARNING: Unsupported ROS2 message type for numeric: " + ros2_msg_type
            )
            return

        publisher.publish(ros2_msg)

    def setup_transform_listener(self):
        try:
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self.ros2_node)

        except Exception as e:
            print("ERROR: Could not set up tf2_ros transform listener: %s" % str(e))

    def lookup_transform(self, msg, base_reference_frame):
        if self.tf_buffer is None or self.tf_listener is None:
            return FTransform()
        try:
            transform = self.tf_buffer.lookup_transform(
                base_reference_frame, msg.header.frame_id, rclpy.time.Time()
            )
            return FTransform.from_ros_transform_stamped(transform)
        except Exception as e:
            pass
            print(
                "ERROR: Could not look up transform between %s and %s: %s, using identity"
                % (base_reference_frame, msg.header.frame_id, str(e))
            )
        return FTransform()


if __name__ == "__main__":
    try:
        ROS2Adapter()
    except KeyboardInterrupt:
        exit()
    except Exception as e:
        print(e)
        exit()
