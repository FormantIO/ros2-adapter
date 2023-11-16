import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from formant_test_interfaces.srv import (
    NoParameters,
    SingleInt,
    SingleFloat,
    SingleString,
)
from example_interfaces.srv import SetBool


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


class GeneralServiceListener(Node):
    def __init__(self, test_vals, context):
        super().__init__("general_service_listener", context=context)
        self.test_vals = test_vals
        self.completed = {key: False for key in self.test_vals}
        self.is_completed = False
        self.running_services = []
        self.ros2_service_type_mapping = {
            "call_no_params": NoParameters,
            "set_bool": SetBool,
            "set_int": SingleInt,
            "set_float": SingleFloat,
            "set_string": SingleString,
        }
        for ros2_service in self.test_vals:
            self.create_service_with_key(ros2_service)
    def create_service_with_key(self, ros2_service):
        callback_type = self.ros2_service_type_mapping.get(ros2_service, None)
        service = self.create_service(
            callback_type, ros2_service, 
            lambda request, response: self.service_callback(request, response, ros2_service)
        )
        self.running_services.append(service)

    def service_callback(self, request, response, ros2_service):
        print(request, response)
        expected_value = self.test_vals[ros2_service]
        actual_value = response.message
        is_expected = expected_value == actual_value
        if is_expected:
            self.completed[ros2_service] = True
        
        if all(self.completed.values()):
            self.is_completed = True
            self.destroy_node()
            self.context.shutdown()
        return response
        
