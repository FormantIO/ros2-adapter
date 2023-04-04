# Test ROS 2 services

This directory contains a set of ROS 2 interfaces and service servers that can be used to test the service call functionality of the Formant ROS 2 adapter. They can also easily be duplicated and modified for different uses. The instructions and examples are based on these tutorials:

https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Custom-ROS2-Interfaces.html
https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html

and use this as a template:

https://github.com/ros2/examples/tree/humble/rclpy/services/minimal_service

## Setup instructions

Set up a colcon workspace if you haven't already, and put the contents of "src" into the workspace's "src". Build all of the packages by running `colcon build`, or individual packages with `colcon build --packages-select {package name, e.g. formant_int}`.

Source the packages with source `{workspace}/install/setup.bash`.

Run `ros2 pkg list`, `ros2 pkg executables {package}`, `ros2 interface list`, and `ros2 interface show {interface}` to see the new services and how to use them ([this example config](https://github.com/FormantIO/ros2-adapter/blob/qa-adapter/tests/test_service_calls/config_test_service_calls.json) also shows the names of the services and interfaces).

## Testing
1. Run the ROS 2 adapter with the service calls configured (e.g. [this example config](https://github.com/FormantIO/ros2-adapter/blob/qa-adapter/tests/test_service_calls/config_test_service_calls.json))
2. Run the service server (e.g. `ros2 run formant_int service`)
3. Send a command or publish to a topic from Formant whose name matches the `formant_stream` field in the config (e.g. `ros2_service_test_set_int`)

## Creating more examples

New interfaces can easily be created by following the "Custom-ROS2-Interfaces" tutorial mentioned above.

To create new service call servers, you can take a shortcut and copy/modify one of the existing examples. You will notice that they are all almost the same structurally, but there are a few things that need to be changed:

* "package.xml": `name` field, maybe `exec_depend` depending on the dependencies
* "setup.py": `package_name`, `entry_points` fields
* "setup.cfg": `script_dir` and `install_dir` fields
* "resource" folder: change name of file inside to match packge
* "service.py": what the service actually does
