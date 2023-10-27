import unittest
import datetime
import rclpy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from agent_mock import AgentMockServicer, serve
from ros2_utils import GeneralPublisher
import subprocess
import time


class TestAgentMockServicer(unittest.TestCase):
    def setUp(self):
        rclpy.init()

        self.servicer = AgentMockServicer()
        self.server = serve(self.servicer)

        terminal_command = "gnome-terminal"
        script_to_run = "source /opt/ros/*/setup.bash && python3 ../main.py --test"
        self.adapter_process = subprocess.Popen(
            script_to_run,
            shell=True,
            executable="/bin/bash",
            cwd="./",
        )
        time.sleep(30)

    def test_post_string(self):
        topic = "/my_string"
        message_data = String()
        message_data.data = "key: value"

        start_time = datetime.datetime.utcnow().isoformat()
        print(f"Start Time: {start_time}")

        general_publisher = GeneralPublisher(topic, String, message_data)

        end_time = datetime.datetime.utcnow().isoformat()
        print(f"End Time: {end_time}")

        time.sleep(5)
        print(self.servicer.post_datapoints)

    def tearDown(self):
        # Kill the terminal process
        self.adapter_process.terminate()
        self.adapter_process.wait()  # Wait for the process to terminate

        self.server.stop(0)

        rclpy.shutdown()


if __name__ == "__main__":
    unittest.main()
