from rclpy.node import Node
import time
import traceback

from formant.sdk.agent.v1 import Client
from formant.sdk.agent.adapter_utils import JsonSchemaValidator

from components.formant_control.formant_control import FormantControl
from components.parameters.parameter_coordinator import ParameterCoordinator
from components.publisher.publisher_coodinator import PublisherCoordinator
from components.services.service_coordinator import ServiceCoordinator
from components.subscriber.subscriber_coordinator import SubscriberCoordinator
from configuration.config_schema import ConfigSchema
from ros2_utils.topic_type_provider import TopicTypeProvider
from utils.logger import get_logger


INITIAL_WAIT = 5


class ROS2Adapter:
    def __init__(self, fclient: Client, node: Node):
        self._logger = get_logger()
        self._fclient = fclient
        self._node = node
        self._topic_type_provider = TopicTypeProvider(self._node)
        self._subscriber_coordinator = SubscriberCoordinator(
            self._fclient, self._node, self._topic_type_provider
        )
        self._service_coordinator = ServiceCoordinator(
            self._fclient, self._node, self._topic_type_provider
        )
        self._parameter_coordinator = ParameterCoordinator(self._fclient, self._node)
        self._publisher_coordinator = PublisherCoordinator(
            self._fclient, self._node, self._topic_type_provider
        )
        self._formant_control = FormantControl(
            self._fclient, self._publisher_coordinator, self._service_coordinator
        )

        self.configuration_validator = JsonSchemaValidator(
            self._fclient,
            "ros2_adapter_configuration",
            self.setup_with_config,
            validate=False,
        )

        self._logger.info("ROS 2 adapter finished initializing")

    def setup_with_config(self, config):
        try:
            self._logger.info("Received Config")
            if "ros2_adapter_configuration" in config:
                self.config = ConfigSchema(config["ros2_adapter_configuration"])
                self._logger.info("Config Parsed")
            else:
                self.config = ConfigSchema({})

            self._logger.info("Waiting for %s for topics to register" % INITIAL_WAIT)
            time.sleep(INITIAL_WAIT)

            self._topic_type_provider.update_topic_types()
            self._subscriber_coordinator.setup_with_config(self.config)
            self._service_coordinator.setup_with_config(self.config)
            self._parameter_coordinator.setup_with_config(self.config)
            self._publisher_coordinator.setup_with_config(self.config)
            self._formant_control.setup_with_config(self.config)

            self._logger.info("ROS 2 adapter finished setting up with new config")
        except Exception as e:
            self._logger.error(traceback.format_exc())
