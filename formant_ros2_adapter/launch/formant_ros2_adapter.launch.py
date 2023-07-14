import os

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get the path to the package
    package_name = "formant_ros2_adapter"
    config_dir = os.path.join(get_package_share_directory(package_name), "config")

    # Create the node, setting the config_dir environment variable
    # Merge the current environment variables with the new one
    env = os.environ.copy()
    env["CONFIG_DIR"] = config_dir

    node = Node(package=package_name, executable="main.py", name="formant_ros2_adapter", output="both", env=env)

    return LaunchDescription([node])
