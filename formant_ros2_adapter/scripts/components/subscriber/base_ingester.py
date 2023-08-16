from cv_bridge import CvBridge
import cv2
import grpc
from typing import Dict
from sensor_msgs.msg import (
    BatteryState,
    CompressedImage,
    Image,
    LaserScan,
    NavSatFix,
    PointCloud2,
)
from .types import STRING_TYPES, BOOL_TYPES, NUMERIC_TYPES, OTHER_DATA_TYPES

from formant.sdk.agent.v1 import Client
from formant.protos.model.v1.datapoint_pb2 import Datapoint
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

from utils.logger import get_logger
from ros2_utils.message_utils import (
    get_ros2_type_from_string,
    message_to_json,
    get_message_path_value,
)

"""
A Handle Exceptions Class would be nice
"""


class BaseIngester:
    def __init__(self, _fclient: Client):
        self._fclient = _fclient
        self.cv_bridge = CvBridge()
        self._logger = get_logger()

    def prepare(
        self,
        msg,
        msg_type: type,
        formant_stream: str,
        topic: str,
        msg_timestamp: int,
        tags: Dict,
    ):
        msg = self._preprocess(msg, msg_type)

        if msg_type in STRING_TYPES:
            msg = self._fclient.prepare_text(
                stream=formant_stream, value=msg, tags=tags, timestamp=msg_timestamp
            )

        elif msg_type in BOOL_TYPES:
            self._fclient.prepare_bitset(
                stream=formant_stream, value=msg, tags=tags, timestamp=msg_timestamp
            )
        elif msg_type in NUMERIC_TYPES:
            self._fclient.prepare_numeric(
                stream=formant_stream, value=msg, tags=tags, timestamp=msg_timestamp
            )

        elif msg_type == NavSatFix:

            self._fclient.prepare_geolocation(
                stream=formant_stream,
                latitude=msg.latitude,
                longitude=msg.longitude,
                altitude=msg.altitude,
                tags=tags,
                timestamp=msg_timestamp,
            )

        elif msg_type == Image:
            self._fclient.prepare_image(
                stream=formant_stream,
                value=msg,
                tags=tags,
                timestamp=msg_timestamp,
            )
        elif msg_type == CompressedImage:
            self._fclient.prepare_image(
                stream=formant_stream,
                value=msg["value"],
                content_type=msg["content_type"],
                tags=tags,
                timestamp=msg_timestamp,
            )

        elif msg_type == BatteryState:
            self._fclient.prepare_battery(
                stream=formant_stream,
                percentage=msg.percentage,
                voltage=msg.voltage,
                current=msg.current,
                charge=msg.charge,
                tags=tags,
                timestamp=msg_timestamp,
            )

        elif msg_type == LaserScan:
            msg = Datapoint(
                stream=formant_stream,
                point_cloud=FPointCloud.from_ros_laserscan(msg).to_proto(),
                tags=tags,
                timestamp=msg_timestamp,
            )

        elif msg_type == PointCloud2:
            Datapoint(
                stream=formant_stream,
                point_cloud=FPointCloud.from_ros(msg).to_proto(),
                tags=tags,
                timestamp=msg_timestamp,
            )

        else:
            self._fclient.prepare_json(
                stream=formant_stream,
                value=msg,
                tags=tags,
                timestamp=msg_timestamp,
            )
        return msg

    def _preprocess(self, msg, msg_type: type):

        if msg_type in STRING_TYPES:
            msg = self._prepare_string(msg)
        elif msg_type in BOOL_TYPES or msg_type in NUMERIC_TYPES:
            msg = self._prepare_attr_data(msg)
        elif msg_type == Image:
            msg = self._prepare_image(msg)

        elif msg_type == CompressedImage:
            msg = self._prepare_compressed_image(msg)

        elif msg_type not in OTHER_DATA_TYPES:
            msg = message_to_json(msg)

        return msg

    def _prepare_string(self, msg):
        msg = self._prepare_attr_data(msg)
        msg = str(msg)
        return msg

    def _prepare_image(self, msg):
        cv_image = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")
        encoded_image = cv2.imencode(".jpg", cv_image)[1].tobytes()
        return encoded_image

    def _prepare_compressed_image(self, msg):
        if "jpg" in msg.format or "jpeg" in msg.format:
            content_type = "image/jpg"
        elif "png" in msg.format:
            content_type = "image/png"
        else:
            self._logger.warn("Image format", msg.format, "not supported")
            return
        return {"value": bytes(msg.data), "content_type": content_type}

    def _prepare_attr_data(self, msg):
        if hasattr(msg, "data"):
            msg = msg.data
        return msg
