from rclpy.node import Node
from rclpy.parameter import Parameter

from formant.sdk.agent.v1 import Client

from configuration.config_schema import ConfigSchema
from utils.logger import get_logger


class ParameterCoordinator:
    def __init__(
        self,
        fclient: Client,
        node: Node,
    ):
        self._logger = get_logger()
        self._fclient = fclient
        self._node = node

    def setup_with_config(self, config: ConfigSchema):
        try:
            for item in self._fclient._app_config.items():
                try:
                    self._node.set_parameters(
                        [
                            Parameter(
                                item[0],
                                Parameter.Type.STRING,
                                item[1],
                            )
                        ]
                    )
                except Exception as e:
                    self._logger.info("Error setting up parameter %s" % (str(item)))
        except Exception as e:
            self._logger.warn("Issue getting app config %s" % str(e))
