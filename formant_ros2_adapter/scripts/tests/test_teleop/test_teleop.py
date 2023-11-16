import unittest
import datetime
import rclpy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from agent_mock import AgentMockServicer, serve
from ros2_utils import TeleopTextListener
import subprocess
import time
import json
from models_mock import Command


class TestTeleop(unittest.TestCase):
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

    def test_publish(self):
        stream_name = "burger.text"
        value = "sample_text"
        ros2_topic = "/teleop_text"

        self.servicer.command_requests.append(
            Command(id="_", name=stream_name, value=value)
        )
        context = rclpy.Context()
        rclpy.init(context=context)

        publisher = TeleopTextListener(ros2_topic, value, context)
        executor = rclpy.executors.SingleThreadedExecutor(context=context)
        executor.add_node(publisher)
        try:
            executor.spin()
        except Exception as e:
            print(f"Error while spinning: {e}")

        self.assertTrue(publisher.text_received)

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
