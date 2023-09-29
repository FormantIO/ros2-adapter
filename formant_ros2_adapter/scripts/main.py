#!/usr/bin/python3

import os
import rclpy

from formant.sdk.agent.v1 import Client
from ros2_adapter import ROS2Adapter
from utils.logger import get_logger
import os

ROS2_DOMAIN_ID = os.environ.get("ROS_DOMAIN_ID", None)
if ROS2_DOMAIN_ID:
    ROS2_DOMAIN_ID = int(ROS2_DOMAIN_ID)

FCLIENT_WAIT = 2

if __name__ == "__main__":
    logger = get_logger()
    rclpy.init(domain_id=ROS2_DOMAIN_ID)


    # Get the config directory from an environment variable
    config_dir = os.environ.get("CONFIG_DIR")

    # If the config environment variable is set and exists, change the working directory
    # Workaround for the agent adapter looking for "config.py" in the current working directory only
    if config_dir and os.path.isdir(config_dir):
        os.chdir(config_dir)

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
