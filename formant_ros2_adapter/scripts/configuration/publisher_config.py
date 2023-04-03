from typing import Dict, Optional

from .get_value import get_value


class PublisherConfig:
    def __init__(self, config: Dict):
        self.formant_stream: str = get_value(config, "formant_stream", required=True)
        self.formant_datapoint_type: Optional[str] = get_value(
            config, "formant_datapoint_type"
        )
        self.topic: str = get_value(config, "ros2_topic", required=True)
        self.message_type: str = get_value(config, "ros2_message_type", required=True)
        self.qos_profile: Optional[str] = get_value(config, "ros2_qos_profile")
