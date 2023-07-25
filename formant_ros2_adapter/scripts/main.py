#!/usr/bin/python3

import rclpy
import time

from formant.sdk.agent.v1 import Client

from ros2_adapter import ROS2Adapter
from utils.logger import get_logger


FCLIENT_WAIT = 2


if __name__ == "__main__":
    logger = get_logger()
    rclpy.init()
    node = rclpy.create_node(
        "formant_ros2_adapter",
        allow_undeclared_parameters=True,
        automatically_declare_parameters_from_overrides=True,
    )
    logger.info("Creating Formant agent client")
    # To do: a cleaner solution would have ignore_unavailable=True and
    # something implemented in the client to avoid a race condition
    fclient = Client(ignore_throttled=True, ignore_unavailable=True)
    logger.info("Waiting %s seconds for Formant agent client" % FCLIENT_WAIT)
    ROS2Adapter(fclient, node)
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=1.0)
    except KeyboardInterrupt:
        pass
    if node:
        node.destroy_node()

    rclpy.shutdown()
