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


class Ingester:
    def __init__(self, _fclient: Client):
        self._fclient = _fclient
        self.cv_bridge = CvBridge()
        self._logger = get_logger()

    def ingest(
        self,
        msg,
        msg_type: type,
        formant_stream: str,
        topic: str,
        msg_timestamp: int,
        tags: Dict,
    ):

        # Handle the message based on its type
        try:
            if msg_type in [str, String, Char]:
                if hasattr(msg, "data"):
                    msg = msg.data

                self._fclient.post_text(
                    formant_stream,
                    str(msg),
                    tags=tags,
                    timestamp=msg_timestamp,
                )

            elif msg_type in [Bool, bool]:
                if hasattr(msg, "data"):
                    msg = msg.data

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
                if hasattr(msg, "data"):
                    msg = msg.data

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
                # Convert Image to a Formant image
                cv_image = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")
                encoded_image = cv2.imencode(".jpg", cv_image)[1].tobytes()

                self._fclient.post_image(
                    stream=formant_stream,
                    value=encoded_image,
                    tags=tags,
                    timestamp=msg_timestamp,
                )

            elif msg_type == CompressedImage:
                # Post the compressed image
                if "jpg" in msg.format or "jpeg" in msg.format:
                    content_type = "image/jpg"
                elif "png" in msg.format:
                    content_type = "image/png"
                else:
                    self._logger.warn("Image format", msg.format, "not supported")
                    return
                self._fclient.post_image(
                    formant_stream,
                    value=bytes(msg.data),
                    content_type=content_type,
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
                    message_to_json(msg),
                    tags=tags,
                    timestamp=msg_timestamp,
                )

        except AttributeError as e:
            self._logger.error("Could not ingest " + formant_stream + ": " + str(e))
