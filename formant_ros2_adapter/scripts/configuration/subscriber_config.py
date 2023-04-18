from typing import Dict, List, Optional

from .get_value import get_value


class SubscriberConfig:
    def __init__(self, config: Dict):
        self.topic: str = get_value(config, "ros2_topic", required=True)
        self.message_paths: Optional[List[MessagePathConfig]] = get_value(
            config, "ros2_message_paths", cls=MessagePathConfig, is_array=True
        )
        self.message_type: Optional[str] = get_value(config, "ros2_message_type")
        self.qos_profile: Optional[str] = get_value(config, "ros2_qos_profile")
        formant_stream = get_value(config, "formant_stream")
        self.formant_stream: str = (
            formant_stream
            if formant_stream is not None
            else self.topic[1:].replace("/", ".")
        )


class MessagePathConfig:
    def __init__(self, config: Dict):
        self.path: str = get_value(config, "path", required=True)
        self.tag_key: Optional[str] = get_value(config, "tag_key")
        self.tag_value: Optional[str] = get_value(config, "tag_value")
