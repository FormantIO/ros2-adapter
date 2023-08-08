from cv_bridge import CvBridge
import cv2
import grpc
from typing import Dict
from sensor_msgs.msg import (
    BatteryState,
    CompressedImage,
    Image,
    Joy,
    LaserScan,
    NavSatFix,
    PointCloud2,
)
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
        msg = self.prepare(msg, msg_type)
        # Handle the message based on its type
        try:
            if msg_type in [str, String, Char]:
                self._fclient.post_text(
                    formant_stream,
                    msg,
                    tags=tags,
                    timestamp=msg_timestamp,
                )

            elif msg_type in [Bool, bool]:
                self._fclient.post_bitset(
                    formant_stream,
                    {topic: msg},
                    tags=tags,
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
                self._fclient.post_numeric(
                    formant_stream,
                    msg,
                    tags=tags,
                    timestamp=msg_timestamp,
                )

            elif msg_type == NavSatFix:
                # Convert NavSatFix to a Formant location
                self._fclient.post_geolocation(
                    stream=formant_stream,
                    latitude=msg.latitude,
                    longitude=msg.longitude,
                    altitude=msg.altitude,
                    tags=tags,
                    timestamp=msg_timestamp,
                )

            elif msg_type == Image:
                self._fclient.post_image(
                    stream=formant_stream,
                    value=msg,
                    tags=tags,
                    timestamp=msg_timestamp,
                )

            elif msg_type == CompressedImage:
                self._fclient.post_image(
                    formant_stream,
                    value=msg["value"],
                    content_type=msg["content_type"],
                    tags=tags,
                    timestamp=msg_timestamp,
                )

            elif msg_type == BatteryState:
                self._fclient.post_battery(
                    formant_stream,
                    msg.percentage,
                    voltage=msg.voltage,
                    current=msg.current,
                    charge=msg.charge,
                    tags=tags,
                    timestamp=msg_timestamp,
                )

            elif msg_type == LaserScan:
                # Convert LaserScan to a Formant pointcloud
                try:
                    self._fclient.agent_stub.PostData(
                        Datapoint(
                            stream=formant_stream,
                            point_cloud=FPointCloud.from_ros_laserscan(msg).to_proto(),
                            tags=tags,
                            timestamp=msg_timestamp,
                        )
                    )
                except grpc.RpcError as e:
                    return
                except Exception as e:
                    self._logger.error(
                        "Could not ingest " + formant_stream + ": " + str(e)
                    )
                    return

            elif msg_type == PointCloud2:
                try:
                    self._fclient.agent_stub.PostData(
                        Datapoint(
                            stream=formant_stream,
                            point_cloud=FPointCloud.from_ros(msg).to_proto(),
                            tags=tags,
                            timestamp=msg_timestamp,
                        )
                    )
                except grpc.RpcError as e:
                    return
                except Exception as e:
                    self._logger.error(
                        "Could not ingest " + formant_stream + ": " + str(e)
                    )
                    return

            else:
                # Ingest any messages without a direct mapping to a Formant type as JSON
                self._fclient.post_json(
                    formant_stream,
                    msg,
                    tags=tags,
                    timestamp=msg_timestamp,
                )

        except AttributeError as e:
            self._logger.error("Could not ingest " + formant_stream + ": " + str(e))
