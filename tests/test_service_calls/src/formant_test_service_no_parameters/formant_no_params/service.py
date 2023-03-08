# Uses this as a template: 
# https://github.com/ros2/examples/tree/humble/rclpy/services/minimal_service


from formant_test_interfaces.srv import NoParameters 

import rclpy

g_node = None


def no_params_callback(request, response):
    global g_node
    g_node.get_logger().info(f"Incoming request:\n{request}")

    response.message = "I don't need you!"
    g_node.get_logger().info(f"Response:\n{response}, {type(response)}")

    return response


def main(args=None):
    global g_node
    rclpy.init(args=args)

    g_node = rclpy.create_node('formant_no_params')

    srv = g_node.create_service(NoParameters, 'call_no_params', no_params_callback)
    g_node.get_logger().info("Starting Formant no parameters service")
    while rclpy.ok():
        rclpy.spin_once(g_node)

    g_node.destroy_service(srv)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
