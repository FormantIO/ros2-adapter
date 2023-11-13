# Uses this as a template: 
# https://github.com/ros2/examples/tree/humble/rclpy/services/minimal_service


# This interface is included with rclpy, not custom
from example_interfaces.srv import SetBool

import rclpy

g_node = None


def set_bool_callback(request, response):
    global g_node
    g_node.get_logger().info(f"Incoming request:\n{request}")

    response.success = request.data
    g_node.get_logger().info(f"Response:\n{response.success}, {type(response.success)}")

    return response


def main(args=None):
    global g_node
    rclpy.init(args=args)

    g_node = rclpy.create_node('formant_bool')

    srv = g_node.create_service(SetBool, 'set_bool', set_bool_callback)
    g_node.get_logger().info(f"Starting service /set_bool")
    while rclpy.ok():
        rclpy.spin_once(g_node)

    g_node.destroy_service(srv)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
