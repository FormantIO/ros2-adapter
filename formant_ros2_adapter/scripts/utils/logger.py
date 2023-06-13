import logging

logger = logging.getLogger("formant_ros2_adapter")
logger.setLevel(logging.DEBUG)


def get_logger():
    """Return instance of formant logger"""
    return logger
