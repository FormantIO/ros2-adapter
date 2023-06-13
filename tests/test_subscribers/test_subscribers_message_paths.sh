#!/bin/bash


DEVICE="nicolas-container"
TOPIC="/my_velocity"
STREAM_1="my.velocity.linear"
STREAM_2="my.velocity.angular"

source /opt/ros/*/setup.bash

# How to get date formant required for fctl
start_time=$(date -u +"%Y-%m-%dT%H:%M:%S")
ros2 topic pub -t 2 -w 0 $TOPIC geometry_msgs/msg/Twist "{linear: {x: 1, y: 0, z: 0}, angular: {x: 0, y: 1, z: 0}}"
end_time=$(date -u +"%Y-%m-%dT%H:%M:%S")

# Verify that the data just published was ingested with fctl
fctl query --device $DEVICE --stream $STREAM_1 --start $start_time --end $end_time
fctl query --device $DEVICE --stream $STREAM_2 --start $start_time --end $end_time