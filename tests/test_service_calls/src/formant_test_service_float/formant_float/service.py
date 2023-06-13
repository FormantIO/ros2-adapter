# Uses this as a template: 
# https://github.com/ros2/examples/tree/humble/rclpy/services/minimal_service


from formant_test_interfaces.srv import SingleFloat

import rclpy

g_node = None


def set_float_callback(request, response):
    global g_node
    g_node.get_logger().info(f"Incoming request:\n{request}")

    response.output = request.input
    g_node.get_logger().info(f"Response:\n{response.output}, {type(response.output)}")

    return response


def main(args=None):
    global g_node
    rclpy.init(args=args)

    g_node = rclpy.create_node('formant_float')

    srv = g_node.create_service(SingleFloat, 'set_float', set_float_callback)
    g_node.get_logger().info("Starting service /set_float")
    while rclpy.ok():
        rclpy.spin_once(g_node)

    g_node.destroy_service(srv)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
