#!/bin/bash


DEVICE="nicolas-container"
TOPIC="/my_string"
STREAM="my.string"

source /opt/ros/*/setup.bash

# How to get date formant required for fctl
start_time=$(date -u +"%Y-%m-%dT%H:%M:%S")
ros2 topic pub -t 2 -w 0 $TOPIC std_msgs/msg/String "data: {key: value}"
end_time=$(date -u +"%Y-%m-%dT%H:%M:%S")

# Verify that the data just published was ingested with fctl
fctl query --device $DEVICE --stream $STREAM --start $start_time --end $end_time
