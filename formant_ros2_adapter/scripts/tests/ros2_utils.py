import rclpy
from rclpy.node import Node


class GeneralPublisher(Node):
    def __init__(self, topic_name, message_type, message_data, context):
        super().__init__("general_publisher", context=context)
        self.publisher_ = self.create_publisher(message_type, topic_name, 10)
        self.message_data = message_data
        self.timer_period = 0.5  # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = self.message_data
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg}"')
        self.i += 1
        if self.i > 1:  # Publish twice and then shut down
            self.timer.cancel()  # Cancel the timer
            self.destroy_node()  # Destroy the node
            self.context.shutdown()  # Shutdown the context
