import unittest
import rclpy
from agent_mock import AgentMockServicer, serve
import subprocess
import time

ROS_BAG_PATH = "./test_localization/rosbag2_turtlebot3_localization"


class TestLocalization(unittest.TestCase):
    def setUp(self):
        rclpy.init()

        self.servicer = AgentMockServicer()
        self.server = serve(self.servicer)

        command = "source /opt/ros/*/setup.bash && python3 ../main.py --test"

        self.adapter_process = subprocess.Popen(
            command,
            shell=True,
            executable="/bin/bash",
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )
        time.sleep(10)

    def test_localization(self):
        bag_command = "source /opt/ros/*/setup.bash && ros2 bag play %s" % (
            ROS_BAG_PATH
        )
        self.bag_process = subprocess.Popen(
            bag_command,
            shell=True,
            executable="/bin/bash",
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )

        self.bag_process.wait()

        length_datapoints = len(self.servicer.post_datapoints)
        self.assertGreater(length_datapoints, 50)

    def tearDown(self):
        self.adapter_process.terminate()
        self.adapter_process.kill()

        self.bag_process.terminate()
        self.bag_process.kill()
        self.server.stop(0)

        rclpy.shutdown()


if __name__ == "__main__":
    unittest.main()
