#!/bin/bash

source /opt/ros/*/setup.bash
ros2 topic pub -t 10 -w 0 /qa_string std_msgs/msg/String "data: {hello: world}"