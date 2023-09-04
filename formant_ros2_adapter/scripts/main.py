#!/usr/bin/python3

import rclpy

from utils.logger import get_logger
from ros2_utils.message_utils import get_ros2_type_from_string
from ros2_utils.qos import qos_profile_system_default


FCLIENT_WAIT = 2
logger = get_logger()


def handle_message(msg):
    logger.info(msg)


if __name__ == "__main__":
    rclpy.init()
    node = rclpy.create_node(
        "formant_ros2_adapter",
        allow_undeclared_parameters=True,
        automatically_declare_parameters_from_overrides=True,
    )
    logger.info("Creating Formant agent client")
    # To do: a cleaner solution would have ignore_unavailable=True and
    # something implemented in the client to avoid a race condition
    logger.info("Waiting %s seconds for Formant agent client" % FCLIENT_WAIT)
    new_subscriber = node.create_subscription(
        get_ros2_type_from_string("interfaces/msg/AgvReport"),
        "/agv/a8/report",
        callback=handle_message,
        qos_profile=qos_profile_system_default,
    )
    logger.info("subscriber setup")
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=1.0)
    except KeyboardInterrupt:
        pass
    if node:
        node.destroy_node()

    rclpy.shutdown()
