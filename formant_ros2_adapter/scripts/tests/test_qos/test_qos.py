import subprocess
import unittest
import re
import time


class QoSTest(unittest.TestCase):
    def setUp(self):
        command = "source /opt/ros/*/setup.bash && python3 ../main.py"
        self.adapter_process = subprocess.Popen(
            command,
            shell=True,
            executable="/bin/bash",
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
        )
        time.sleep(10)

    def test_qos_compatibility(self):
        self._publish_qos_commands()
        """
        time.sleep(5)
        self.adapter_process.terminate()
        self.adapter_process.wait()
        self.adapter_thread.join()
        print(self.adapter_output)
        """

    def _publish_qos_commands(self):
        SUBSCRIBER_COMMAND = 'ros2 topic pub /my_string std_msgs/msg/String "data: {key: value}" --qos-profile sensor_data'
        subscriber_warning = self._run_command(SUBSCRIBER_COMMAND)
        is_subscriber_warning = self._regex_match(subscriber_warning)
        self.assertTrue(is_subscriber_warning)

        PUBLISHER_COMMAND = (
            "ros2 topic echo /my_command_for_publisher --qos-reliability reliable"
        )
        publisher_warning = self._run_command(PUBLISHER_COMMAND)
        is_publisher_warning = self._regex_match(publisher_warning)
        self.assertTrue(is_publisher_warning)

    def _run_command(self, command, wait_time=5):
        process = subprocess.Popen(
            command,
            shell=True,
            executable="/bin/bash",
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
        )
        time.sleep(wait_time)
        process.terminate()
        process.kill()
        process.wait()
        _, stderr = process.communicate()
        return stderr

    def _regex_match(self, message):
        pattern = r"""
        \[WARN\]                        # Match the literal [WARN]
        \s+\[\d+\.\d+\]                 # Match whitespace, followed by timestamp in square brackets
        \s+\[[^\]]+\]:                  # Match whitespace, followed by node name in square brackets and colon
        \s+New\s+(?:subscription|publisher)    # Match ' New subscription' or ' New publisher'
        \s+discovered\s+on\s+topic\s+'[^']+'  # Match ' discovered on topic ' followed by anything between single quotes
        .+?                             # Match any characters non-greedily until the next part of the pattern
        Last\s+incompatible\s+policy:\s+RELIABILITY # Match ' Last incompatible policy: RELIABILITY'
        """

        # Search for a match using the VERBOSE flag to ignore whitespace and comments in the pattern
        match = re.search(pattern, message, re.VERBOSE)

        # Return True if a match is found, else False
        return bool(match)

    def tearDown(self):
        self.adapter_process.terminate()
        self.adapter_process.kill()


if __name__ == "__main__":
    unittest.main()
