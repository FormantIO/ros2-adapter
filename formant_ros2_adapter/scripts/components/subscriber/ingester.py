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
from .base_ingester import BaseIngester
from .types import STRING_TYPES, BOOL_TYPES, NUMERIC_TYPES


class Ingester(BaseIngester):
    def ingest(
        self,
        msg,
        msg_type: type,
        formant_stream: str,
        topic: str,
        msg_timestamp: int,
        tags: Dict,
    ):
        msg = self.prepare(msg, msg_type, formant_stream, topic, msg_timestamp, tags)

        try:
            self._fclient.post_data(msg)
        except grpc.RpcError as e:
            return
        except Exception as e:
            self._logger.error("Could not ingest " + formant_stream + ": " + str(e))
            return
