from typing import Dict

from .get_value import get_value


class ServiceClientConfig:
    def __init__(self, config: Dict):
        self.formant_stream: str = get_value(config, "formant_stream", required=True)
        self.service: str = get_value(config, "ros2_service", required=True)
        self.service_type: str = get_value(config, "ros2_service_type", required=True)
