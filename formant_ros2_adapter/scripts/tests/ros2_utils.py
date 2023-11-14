import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from formant_test_interfaces.srv import SingleInt


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


class TeleopTextListener(Node):
    def __init__(self, ros2_topic, message, context):
        super().__init__("teleop_text_listener", context=context)
        self.subscription = self.create_subscription(
            String, ros2_topic, self.listener_callback, 10
        )
        self.ros2_topic = ros2_topic
        self.message = message
        self.text_received = False

    def listener_callback(self, message):
        if message.data == self.message:
            self.text_received = True
            self.destroy_node()
            self.context.shutdown()


class SingleIntServiceListener(Node):
    def __init__(self, ros2_topic, value, context):
        super().__init__("single_int_service_listener", context=context)
        self.value = value
        self.value_recv = True
        self.service = self.create_service(SingleInt, ros2_topic, self.service_callback)
        print("ASdf")

    def service_callback(self, request, response):
        if str(request.input) == self.value:
            self.value_recv = True
            self.destroy_node()
            self.context.shutdown()
