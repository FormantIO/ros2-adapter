import rclpy
from rclpy.node import Node
from formant.sdk.agent.v1 import Client
from formant.sdk.agent.adapter_utils import JsonSchemaValidator
from configuration.config_schema import ConfigSchema
from components.subscriber.subscriber_coordinator import SubscriberCoordinator
from components.services.service_coordinator import ServiceCoordinator
from components.publisher.publisher_coodinator import PublisherCoordinator
from components.parameters.parameter_coordinator import ParameterCoordinator
from components.formant_control.formant_control import FormantControl
from ros2_utils.topic_type_provider import TopicTypeProvider
from ros2_utils.logger import get_logger
import traceback
import time

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

    def setup_with_config(self, config):
        try:
            self._logger.info("Received Config")
            self._logger.debug(config)
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
        except Exception as e:
            self._logger.error(traceback.format_exc())


if __name__ == "__main__":
    rclpy.init()
    node = rclpy.create_node(
        "formant_ros2_adapter",
        allow_undeclared_parameters=True,
        automatically_declare_parameters_from_overrides=True,
    )
    fclient = Client(ignore_throttled=True)
    ROS2Adapter(fclient, node)
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=1.0)
    except KeyboardInterrupt:
        exit()
        # Clean up before shutting down
    if node:
        node.destroy_node()

    rclpy.shutdown()