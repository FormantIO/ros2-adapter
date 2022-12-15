#!/bin/bash

source /opt/ros/*/setup.bash  # this adapter is meant to work with any ROS2 distribution
python3 -m pip install --force-reinstall --upgrade formant-1.105.50_release_1.105_075d6b4c-py2.py3-none-any.whl

# if you use custom messages, source your workspace here
python3 -m pip install -r requirements.txt
cd formant_ros2_adapter/scripts/
python3 main.py