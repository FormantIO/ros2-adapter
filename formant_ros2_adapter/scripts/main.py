import rclpy

from formant.sdk.agent.v1 import Client

from ros2_adapter import ROS2Adapter


if __name__ == "__main__":
    rclpy.init()
    node = rclpy.create_node(
        "formant_ros2_adapter",
        allow_undeclared_parameters=True,
        automatically_declare_parameters_from_overrides=True,
    )
    fclient = Client(ignore_throttled=True, ignore_unavailable=True)
    ROS2Adapter(fclient, node)
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=1.0)
    except KeyboardInterrupt:
        pass
    if node:
        node.destroy_node()

    rclpy.shutdown()
