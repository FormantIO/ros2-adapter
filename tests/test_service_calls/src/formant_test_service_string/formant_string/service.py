# Uses this as a template: 
# https://github.com/ros2/examples/tree/humble/rclpy/services/minimal_service


from formant_test_interfaces.srv import SingleString

import rclpy

g_node = None


def set_string_callback(request, response):
    global g_node
    g_node.get_logger().info(f"Incoming service call:\n{request}")

    response.output = request.input
    g_node.get_logger().info(f"Response:\n{response.output}, {type(response.output)}")

    return response


def main(args=None):
    global g_node
    rclpy.init(args=args)

    g_node = rclpy.create_node('formant_string')

    srv = g_node.create_service(SingleString, 'set_string', set_string_callback)
    g_node.get_logger().info("Starting service /set_string")
    while rclpy.ok():
        rclpy.spin_once(g_node)

    g_node.destroy_service(srv)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
