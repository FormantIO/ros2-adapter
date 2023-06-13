from typing import Dict, List, Optional

from .get_value import get_value
from .localization_config import LocalizationConfig
from .numeric_set_config import NumericSetConfig
from .publisher_config import PublisherConfig
from .service_client_config import ServiceClientConfig
from .subscriber_config import SubscriberConfig
from .transform_tree_config import TransformTreeConfig


class ConfigSchema:
    def __init__(self, config: Dict):
        self.subscribers: Optional[List[SubscriberConfig]] = get_value(
            config, "subscribers", cls=SubscriberConfig, is_array=True
        )
        self.publishers: Optional[List[PublisherConfig]] = get_value(
            config, "publishers", cls=PublisherConfig, is_array=True
        )
        self.service_clients: Optional[List[ServiceClientConfig]] = get_value(
            config, "service_clients", cls=ServiceClientConfig, is_array=True
        )
        self.localization: Optional[LocalizationConfig] = get_value(
            config, "localization", cls=LocalizationConfig
        )
        self.transform_tree: Optional[TransformTreeConfig] = get_value(
            config, "transform_tree", cls=TransformTreeConfig
        )
        self.numeric_sets: Optional[List[NumericSetConfig]] = get_value(
            config, "numeric_sets", cls=NumericSetConfig, is_array=True
        )
