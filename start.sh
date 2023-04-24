#!/bin/bash

source /opt/ros/*/setup.bash  # this adapter is meant to work with any ROS2 distribution

if [ -z ${FORMANT_ROS2_WS+x} ]; then
    echo "FORMANT_ROS2_WS unset"
else
    echo "sourcing $FORMANT_ROS2_WS"
    source $FORMANT_ROS2_WS
fi
export PYTHONUNBUFFERED=true
python3 -m pip install -r requirements.txt
cd formant_ros2_adapter/scripts/
python3 main.py