import time
from typing import List, Optional

from formant.sdk.agent.v1 import Client
from formant.protos.model.v1.datapoint_pb2 import Datapoint
from formant.protos.model.v1.text_pb2 import Text

from configuration.config_schema import ConfigSchema
from ..publisher.publisher_coodinator import PublisherCoordinator
from ..services.service_call_result import ResultType
from ..services.service_coordinator import ServiceCoordinator
from utils.logger import get_logger


class FormantControl:
    def __init__(
        self,
        fclient: Client,
        publisher_coodinator: PublisherCoordinator,
        service_coordinator: ServiceCoordinator,
    ):
        self._logger = get_logger()
        self._localization_stream: Optional[str] = None
        self._fclient = fclient
        self._publisher_coodinator = publisher_coodinator
        self._service_coordinator = service_coordinator

    def setup_with_config(
        self,
        config: ConfigSchema,
    ):
        self._logger.info("Setting up Formant Control")
        self._config = config

        self._cleanup()
        formant_streams = get_all_formant_streams(config)
        formant_streams.append("Buttons")

        self._logger.info(
            "Registering callback for Formant Streams: %s" % formant_streams
        )
        self._localization_stream = get_localization_stream(config)
        self._fclient.register_command_request_callback(
            self._handle_command, command_filter=formant_streams
        )

        self._fclient.register_teleop_callback(
            self._handle_teleop, stream_filter=formant_streams
        )
        self._logger.info("Set up Formant Control")

    def _handle_teleop(self, msg):
        try:
            stream_name = msg.stream
            self._logger.debug("Teleop message received: %s" % stream_name)
            if stream_name == "Buttons":
                stream_name = msg.bitset.bits[0].key
            self._publisher_coodinator.generic_publisher.publish(stream_name, msg)

            if stream_name == self._localization_stream:
                try:
                    self._publisher_coodinator.localization_publisher.publish_goal(
                        msg.pose
                    )
                except Exception as e:
                    self._logger.warn("Error publishing goal: %s" % e)

            service_param = None
            if msg.HasField("bitset"):
                if msg.bitset.bits[0].value is True:
                    service_param = "True"
            elif msg.HasField("numeric"):
                service_param = msg.numeric.value
            if service_param is not None:
                self._service_coordinator.call_service(stream_name, service_param)
        except Exception as e:
            self._logger.warn("Error in teleop callback: %s" % str(e))

    def _handle_command(self, msg):
        self._logger.info("Command received %s" % str(msg))
        formant_stream = msg.command

        # To do: this shouldn't always happen
        service_call_result = self._service_coordinator.call_service(
            formant_stream, msg.text
        )
        for result in service_call_result:
            if result.type == ResultType.CONFIG_NOT_FOUND:
                continue
            success = result.type == ResultType.SUCCESS
            msg_timestamp = int(time.time() * 1000)
            self._fclient.send_command_response(
                request_id=msg.id,
                success=success,
                datapoint=Datapoint(
                    stream="ros2.service_call.response",
                    text=Text(value=str(service_call_result)),
                    timestamp=msg_timestamp,
                ),
            )

        # To do: this shouldn't always happen
        self._publisher_coodinator.generic_publisher.publish_command(
            formant_stream, msg.text
        )

    def _cleanup(self):
        self._fclient.unregister_command_request_callback(self._handle_command)
        self._fclient.unregister_teleop_callback(self._handle_teleop)


# To do: make logger available outside of class
def get_all_formant_streams(config: ConfigSchema):
    formant_streams: List[str] = []
    if config.service_clients:
        for service_client in config.service_clients:
            formant_streams.append(service_client.formant_stream)
    if config.publishers:
        for publisher in config.publishers:
            formant_streams.append(publisher.formant_stream)
    if (
        config.localization
        and config.localization.goal_publisher_ros2_topic is not None
    ):
        formant_streams.append(config.localization.formant_stream)
    return formant_streams


def get_localization_stream(config: ConfigSchema):
    if (
        config.localization
        and config.localization.goal_publisher_ros2_topic is not None
    ):
        return config.localization.formant_stream
    return None
