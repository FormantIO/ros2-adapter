from geometry_msgs.msg import TransformStamped, PoseWithCovarianceStamped, PoseStamped
from nav_msgs.msg import Odometry, OccupancyGrid, Path
import rclpy
from rclpy.node import Node
from rclpy.subscription import Subscription
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_msgs.msg import TFMessage
import threading
from typing import List, Dict, Optional
from sensor_msgs.msg import (
    LaserScan,
    PointCloud2,
)

from formant.sdk.agent.v1 import Client
from formant.sdk.agent.v1.localization.localization_manager import LocalizationManager
from formant.sdk.agent.v1.localization.types import (
    PointCloud as FPointCloud,
    Map as FMap,
    Path as FPath,
    Transform as FTransform,
    Goal as FGoal,
    Odometry as FOdometry,
)

from configuration.config_schema import ConfigSchema
from configuration.localization_config import LocalizationConfig
from configuration.subscriber_config import SubscriberConfig
from configuration.transform_tree_config import TransformTreeConfig
from utils.logger import get_logger
from ros2_utils.qos import QOS_PROFILES, qos_profile_system_default
from ros2_utils.topic_type_provider import TopicTypeProvider


Costmap = None
try:
    from nav2_msgs.msg import Costmap
except ModuleNotFoundError:
    pass


class LocalizationSubscriberCoordinator:
    def __init__(
        self, fclient: Client, node: Node, topic_type_provider: TopicTypeProvider
    ):
        self._localization_manager: Optional[LocalizationManager] = None
        self._fclient = fclient
        self._node = node
        self._topic_type_provider = topic_type_provider
        self._subscriptions: List[Subscription] = []
        self._logger = get_logger()
        self._setup_transform_listener()
        self.ros2_topic_names_and_types: Dict[str, str] = {}
        self._config_lock = threading.Lock()

    def _setup_transform_listener(self):
        try:
            self._logger.info("Setting up tf2_ros transform listener")
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self._node)
            self._logger.info("Set up tf2_ros transform listener")

        except Exception as e:
            self._logger.warn(
                "Could not set up tf2_ros transform listener: %s" % str(e)
            )

    def _lookup_transform(self, msg, base_reference_frame: str):
        if self.tf_buffer is None or self.tf_listener is None:
            return FTransform()
        try:
            transform = self.tf_buffer.lookup_transform(
                base_reference_frame, msg.header.frame_id, rclpy.time.Time()
            )
            return FTransform.from_ros_transform_stamped(transform)
        except Exception as e:
            pass
            self._logger.warn(
                "Could not look up transform between %s and %s: %s, using identity"
                % (base_reference_frame, msg.header.frame_id, str(e))
            )
        return FTransform()

    def setup_with_config(self, config: ConfigSchema):
        with self._config_lock:
            self._config = config
            self._cleanup()
            if self._config.transform_tree:
                self._setup_tf(self._config.transform_tree)
            if self._config.localization:
                self._setup_localization(self._config.localization)

    def _setup_localization(self, localization_config: LocalizationConfig):
        self._logger.info("Setting up localization subscribers")
        self._localization_manager = self._fclient.get_localization_manager(
            localization_config.formant_stream
        )
        # Setup Odom
        self._logger.info("Setting up odometry")
        odom_topic = localization_config.odometry_subscriber_ros2_topic
        odom_type = self._topic_type_provider.get_class_for_topic(odom_topic, Odometry)
        odom_sub = self._node.create_subscription(
            odom_type,
            odom_topic,
            self._odom_callback,
            qos_profile=qos_profile_system_default,
        )
        self._subscriptions.append(odom_sub)
        self._logger.info("Set up odometry")

        # Setup Map
        map_topic = localization_config.map_subscriber_ros2_topic
        if map_topic:
            self._logger.info("Setting up map")
            map_type = self._topic_type_provider.get_class_for_topic(
                map_topic, OccupancyGrid
            )
            map_sub = self._node.create_subscription(
                map_type,
                map_topic,
                self._map_callback,
                qos_profile=qos_profile_system_default,
            )
            self._subscriptions.append(map_sub)
            self._logger.info("Set up map")

        # Setup Path
        path_topic = localization_config.path_subscriber_ros2_topic
        if path_topic:
            self._logger.info("Setting up path")
            path_type = self._topic_type_provider.get_class_for_topic(path_topic, Path)
            path_sub = self._node.create_subscription(
                path_type,
                path_topic,
                self._path_callback,
                qos_profile=qos_profile_system_default,
            )
            self._subscriptions.append(path_sub)
            self._logger.info("Set up path")

        # Setup Pointcloud
        pointcloud_configs = localization_config.point_cloud_subscriber_ros2_topics
        if pointcloud_configs:
            self._logger.info("Setting up point cloud")
            for pointcloud_config in pointcloud_configs:
                pointcloud_type = self._topic_type_provider.get_class_for_topic(
                    pointcloud_config.topic, Path
                )
                pointcloud_sub = self._node.create_subscription(
                    pointcloud_type,
                    pointcloud_config.topic,
                    lambda msg, topic=pointcloud_config.topic: self._point_cloud_callback(
                        msg, topic
                    ),
                    qos_profile=qos_profile_system_default,
                )
                self._subscriptions.append(pointcloud_sub)
                self._logger.info("Set up point cloud")

        self._logger.info("Set up localization subscribers")

    def _odom_callback(self, msg):
        with self._config_lock:
            msg_type = type(msg)
            if msg_type == Odometry:
                odometry = FOdometry.from_ros(msg)
            elif msg_type == PoseWithCovarianceStamped:
                odometry = FOdometry(pose=FTransform.from_ros_pose(msg.pose.pose))
            else:
                self._logger.warn("Unknown odom type: %s" % msg_type)
                return

            odometry.transform_to_world = self._lookup_transform(
                msg, self._config.localization.base_reference_frame
            )

            self._localization_manager.update_odometry(odometry)

    def _map_callback(self, msg):
        with self._config_lock:
            msg_type = type(msg)
            if msg_type is OccupancyGrid:
                formant_map = FMap.from_ros(msg)
            elif Costmap is not None and msg_type is Costmap:
                # ROS types
                ros_resolution = msg.metadata.resolution
                ros_width = msg.metadata.size_x
                ros_height = msg.metadata.size_y
                ros_origin = msg.metadata.origin

                # Formant types
                formant_map = FMap(
                    resolution=ros_resolution,
                    width=ros_width,
                    height=ros_height,
                    origin=FTransform.from_ros_pose(ros_origin),
                    occupancy_grid_data=msg.data,
                )
            else:
                self._logger.warn("Unknown map type %s" % msg_type)
                return

            formant_map.transform_to_world = self._lookup_transform(
                msg, self._config.localization.base_reference_frame
            )
            self._localization_manager.update_map(formant_map)

    def _path_callback(self, msg):
        with self._config_lock:
            msg_type = type(msg)
            if msg_type == Path:
                path = FPath.from_ros(msg)
            else:
                self._logger.warn("Unknown path type: %s" % msg_type)
                return

            path.transform_to_world = self._lookup_transform(
                msg, self._config.localization.base_reference_frame
            )
            self._localization_manager.update_path(path)

    def _goal_callback(self, msg):
        with self._config_lock:
            msg_type = type(msg)
            if msg_type == PoseStamped:
                goal = FGoal.from_ros(msg)
            else:
                self._logger.warn("Unknown goal type: %s" % msg_type)
                return
            goal.transform_to_world = self._lookup_transform(
                msg, self._config.localization.base_reference_frame
            )
            self._localization_manager.update_goal(goal)

    def _point_cloud_callback(self, msg, topic_name):
        with self._config_lock:
            msg_type = type(msg)
            if msg_type == LaserScan:
                point_cloud = FPointCloud.from_ros_laserscan(msg)
            elif msg_type == PointCloud2:
                point_cloud = FPointCloud.from_ros(msg)
            else:
                self._logger.warn("Unknown point cloud type: %s" % msg_type)
                return

            point_cloud.transform_to_world = self._lookup_transform(
                msg, self._config.localization.base_reference_frame
            )

            self._localization_manager.update_point_cloud(point_cloud, topic_name)

    def _setup_tf(self, tf_config: TransformTreeConfig):
        self._logger.debug(
            "Setting up tf config with base_reference_frame: %s"
            % tf_config.base_reference_frame
        )
        self._fclient.set_base_frame_id(tf_config.base_reference_frame)
        for topic in ["/tf", "/tf_static"]:
            new_sub = self._node.create_subscription(
                TFMessage, topic, self.tf_callback, qos_profile_system_default
            )
            self._subscriptions.append(new_sub)

    def tf_callback(self, msg: TFMessage):
        with self._config_lock:
            tf: TransformStamped
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
                self._fclient.post_transform_frame(
                    parent_frame, child_frame, tx, ty, tz, rx, ry, rz, rw
                )

    def _setup_subscription_for_config(self, subscriber_config: SubscriberConfig):
        pass

    def _cleanup(self):
        for subscription in self._subscriptions:
            self._node.destroy_subscription(subscription)
        self._subscriptions = []
