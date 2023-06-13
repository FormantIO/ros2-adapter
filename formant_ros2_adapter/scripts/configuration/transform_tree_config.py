from typing import Dict

from .get_value import get_value


class TransformTreeConfig:
    def __init__(self, config: Dict):
        self.base_reference_frame: str = get_value(
            config, "base_reference_frame", required=True
        )
