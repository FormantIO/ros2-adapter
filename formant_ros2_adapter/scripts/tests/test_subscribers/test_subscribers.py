import unittest
import datetime
import rclpy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from agent_mock import AgentMockServicer, serve
from ros2_utils import GeneralPublisher
import subprocess
import time
import json


class TestSubscriber(unittest.TestCase):
    def setUp(self):
        self.servicer = AgentMockServicer()
        self.server = serve(self.servicer)

        command = "source /opt/ros/*/setup.bash && python3 ../main.py --test"
        self.f = open("output.txt", "w")

        self.adapter_process = subprocess.Popen(
            command,
            shell=True,
            executable="/bin/bash",
            stdout=self.f,
            stderr=self.f,
        )

        time.sleep(10)

    def test_post_string(self):
        print("l")
        message = String()
        message.data = "test_string"
        context = rclpy.Context()
        rclpy.init(context=context)

        publisher = GeneralPublisher("/my_string", String, message, context)
        executor = rclpy.executors.SingleThreadedExecutor(context=context)
        executor.add_node(publisher)
        try:
            executor.spin()
        except Exception as e:
            print(f"Error while spinning: {e}")

        time.sleep(1)

        count_test_string = sum(
            1
            for item in self.servicer.post_datapoints
            if hasattr(item, "datapoints")
            and any(
                dp.HasField("text") and dp.text.value == "test_string"
                for dp in item.datapoints
            )
        )

        self.assertEqual(count_test_string, 2)

    def test_post_velocity(self):
        message = Twist()
        message.linear.x = 1.0
        message.angular.y = 1.0

        context = rclpy.Context()
        rclpy.init(context=context)
        publisher = GeneralPublisher("/my_velocity", Twist, message, context)
        executor = rclpy.executors.SingleThreadedExecutor(context=context)
        executor.add_node(publisher)
        try:
            executor.spin()
        except Exception as e:
            print(f"Error while spinning: {e}")
        time.sleep(1)

        nested_json_load = lambda x: json.loads(json.loads(x))
        linear_collected = False
        angular_collected = False

        for request_datapoint in self.servicer.post_datapoints:
            linear_collected = (
                any(
                    nested_json_load(dp.json.value)["x"] == 1.0
                    for dp in request_datapoint.datapoints
                    if dp.stream == "my.velocity.linear"
                )
                or linear_collected
            )

            angular_collected = (
                any(
                    nested_json_load(dp.json.value)["y"] == 1.0
                    for dp in request_datapoint.datapoints
                    if dp.stream == "my.velocity.angular"
                )
                or angular_collected
            )

        self.assertTrue(linear_collected, "Linear velocity not collected!")
        self.assertTrue(angular_collected, "Angular velocity not collected!")

    def tearDown(self):
        # Kill the terminal process
        self.adapter_process.terminate()
        self.adapter_process.wait()  # Wait for the process to terminate

        self.server.stop(0)
        self.f.close()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    unittest.main()
