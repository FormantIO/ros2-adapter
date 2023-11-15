import unittest
import time
import rclpy
from agent_mock import AgentMockServicer, serve
import subprocess
import time
import os
from ros2_utils import SingleIntServiceListener, GeneralServiceListener
from models_mock import Command

ADAPTER_NAME = "ros2_adapter_configuration"

TEST_VALUES = {
    "call_no_params": "",
    "set_bool": "True",
    "set_int": "1",
    "set_float": "1.0",
    "set_string": "exampleString",
}


class TestServiceCall(unittest.TestCase):
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

    def test_all(self):
        service_clients = self.servicer.config[ADAPTER_NAME]["service_clients"]
        for service_client in service_clients:
            formant_stream = service_client["formant_stream"]
            test_value = TEST_VALUES[service_client["ros2_service"].replace("/", "")]
            self.servicer.command_requests.append(
                Command(
                    id="_",
                    name=formant_stream,
                    value=test_value,
                )
            )

        context = rclpy.Context()
        rclpy.init(context=context)

        publisher = GeneralServiceListener(TEST_VALUES, context)
        executor = rclpy.executors.MultiThreadedExecutor(context=context)
        executor.add_node(publisher)
        try:
            executor.spin()
        except Exception as e:
            print(f"Error while spinning: {e}")

        self.assertTrue(publisher.value_recv)

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
