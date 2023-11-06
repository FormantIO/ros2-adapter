import unittest
import rclpy
from agent_mock import AgentMockServicer, serve
import subprocess
import time

ROS_BAG_PATH = "./test_tf/rosbag2_turtlebot3_tf"


class TestTF(unittest.TestCase):
    def setUp(self):
        rclpy.init()

        self.servicer = AgentMockServicer()
        self.server = serve(self.servicer)

        command = "source /opt/ros/*/setup.bash && python3 ../main.py --test"

        self.adapter_process = subprocess.Popen(
            command,
            shell=True,
            executable="/bin/bash",
            stdout=open("adapter_process_stdout.txt", "w"),
            stderr=open("adapter_process_stderr.txt", "w"),
        )
        time.sleep(20)

    def test_tf(self):
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
        # Wait for the process to complete and capture the output and errors
        stdout, stderr = self.bag_process.communicate()

        # Open a text file in write mode (this will overwrite existing files)
        with open("ros_bag_output.txt", "w") as f:
            # Write stdout and stderr to the file
            if stdout:
                f.write("Standard Output:\n")
                f.write(stdout.decode())  # Decoding may be needed for binary output
            if stderr:
                f.write("\nStandard Error:\n")
                f.write(stderr.decode())  # Decoding may be needed for binary output

        self.bag_process.wait()

        length_datapoints = len(self.servicer.transform_frames)
        print(length_datapoints)
        self.assertGreater(length_datapoints, 1081)

    def tearDown(self):
        self.adapter_process.terminate()
        self.adapter_process.kill()

        self.bag_process.terminate()
        self.bag_process.kill()
        self.server.stop(0)

        rclpy.shutdown()


if __name__ == "__main__":
    unittest.main()
