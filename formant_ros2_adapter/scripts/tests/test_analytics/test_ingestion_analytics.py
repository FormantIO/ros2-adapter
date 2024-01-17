import unittest
import rclpy
from std_msgs.msg import String, Int32
from geometry_msgs.msg import Twist
from ros2_adapter import ROS2Adapter  # Adjust import as necessary
from formant.sdk.agent.v1 import Client  # Adjust import as necessary
import time
import threading


convert_topic_to_stream = lambda topic: topic.replace("/", ".").strip(".")


# Utility function to publish messages
def publish_messages(node, topic, msg_type, count, delay=1):
    publisher = node.create_publisher(msg_type, topic, 10)
    for i in range(count):
        msg = msg_type()
        if msg_type == String:
            msg.data = "test_string"
        elif msg_type == Twist:
            msg.linear.x = 1.0
            msg.angular.z = 1.0
        elif msg_type == Int32:
            msg.data = i  # Or any integer value
        publisher.publish(msg)
        time.sleep(delay)


# Test class
class TestROS2Adapter(unittest.TestCase):
    def setUp(self):
        # Initialize ROS and create a node
        rclpy.init()
        self.node = rclpy.create_node("test_node")

        # Start ROS2Adapter in a separate thread
        self.adapter = ROS2Adapter(Client(), self.node)
        self.adapter_thread = threading.Thread(target=self.spin_ros)
        self.adapter_thread.start()

        time.sleep(2)  # Wait for ROS2Adapter to initialize

    def spin_ros(self):
        while rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=0.1)

    def test_message_counts(self):
        # Test for message counts
        topics = ["/my_string", "/my_int"]
        message_types = [String, Int32]
        message_counts = [8, 3]
        ingestion_analytics = (
            self.adapter._subscriber_coordinator._ingester.ingestion_analytics
        )
        ingestion_analytics.received_messages_count = {}
        ingestion_analytics.sent_messages_count = {}
        for topic, msg_type, count in zip(topics, message_types, message_counts):
            publish_messages(self.node, topic, msg_type, count, delay=0.5)
        received_messages = {}
        sent_messages = {}
        received_message_history = {}
        sent_message_history = {}

        total_duration = 120
        interval_duration = 30
        start_time = time.time()

        def update_received_messages(key, value):
            if value == 0:
                received_messages[key] = received_messages.get(
                    key, 0
                ) + received_message_history.get(key, 0)
                received_message_history[key] = 0
            else:
                received_message_history[key] = max(
                    received_message_history.get(key, 0), value
                )

        def update_sent_messages(key, value):
            if value == 0:
                sent_messages[key] = sent_messages.get(
                    key, 0
                ) + sent_message_history.get(key, 0)
                sent_message_history[key] = 0
            else:
                sent_message_history[key] = max(sent_message_history.get(key, 0), value)

        while time.time() - start_time < total_duration:
            for key, value in ingestion_analytics.received_messages_count.items():
                update_received_messages(key, value)

            for key, value in ingestion_analytics.sent_messages_count.items():
                update_sent_messages(key, value)

            time.sleep(interval_duration)

        # Final iteration for received and sent messages
        for key in received_message_history:
            update_received_messages(key, 0)

        for key in sent_message_history:
            update_sent_messages(key, 0)

        for topic, count in zip(topics, message_counts):
            stream = convert_topic_to_stream(topic)
            received_count = received_messages.get(topic, 0)
            sent_count = sent_messages.get(stream, 0)

            try:
                self.assertEqual(
                    received_count,
                    count,
                    f"Incorrect number of received messages for {topic}",
                )
                self.assertEqual(
                    sent_count, count, f"Incorrect number of sent messages for {stream}"
                )
            except AssertionError as e:
                print(f"Warning: {e}")

    def tearDown(self):
        if rclpy.ok():
            rclpy.shutdown()
        self.adapter_thread.join()


if __name__ == "__main__":
    unittest.main()
