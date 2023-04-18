from typing import Dict, List, Optional

from .get_value import get_value


class NumericSetConfig:
    def __init__(self, config: Dict):
        self.formant_stream: str = get_value(config, "formant_stream", required=True)
        self.subscribers: List[NumericSetSubscriberConfig] = get_value(
            config,
            "ros2_subscribers",
            cls=NumericSetSubscriberConfig,
            required=True,
            is_array=True,
        )


class NumericSetSubscriberConfig:
    def __init__(self, config: Dict):
        self.topic: str = get_value(config, "ros2_topic", required=True)
        self.message_path: Optional[str] = get_value(
            config, "ros2_message_path", is_array=True
        )
        self.message_type: Optional[str] = get_value(config, "ros2_message_type")
        self.qos_profile: Optional[str] = get_value(config, "ros2_qos_profile")
        self.label: str = get_value(config, "label", required=True)
        self.unit: Optional[str] = get_value(config, "unit")
