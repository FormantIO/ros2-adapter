#!/usr/bin/python3
import rclpy
from rclpy.node import Node

class EmptyNode(Node):
    def __init__(self):
        super().__init__('empty_node')
        self.get_logger().info('EmptyNode is running...')

def main(args=None):
    rclpy.init(args=args)
    node = EmptyNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()