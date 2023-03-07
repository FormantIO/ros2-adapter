#!/bin/bash

source /opt/ros/*/setup.bash  # this adapter is meant to work with any ROS2 distribution

# if you use custom messages, source your workspace here

export PYTHONUNBUFFERED=true

cd /app/formant_ros2_adapter/scripts/
python3 main.py