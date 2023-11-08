#!/bin/bash

DEVICE="nicolas-container"
TOPIC="/my_string"
STREAM="my.string"

source /opt/ros/*/setup.bash

# How to get date formant required for fctl
start_time=$(date -u +"%Y-%m-%dT%H:%M:%S")

# Publish multiple messages in quick succession to trigger batching
number_of_messages=20
for i in $(seq 1 $number_of_messages); do
    ros2 topic pub -t 2 -w 0 $TOPIC std_msgs/msg/String "data: {key: value_$i}" &
    sleep 0.01  # Sleep for 10ms between messages
done

wait  # Wait for all background jobs to finish

end_time=$(date -u +"%Y-%m-%dT%H:%M:%S")

# Give some time for the BatchIngester to process and send the data
# This time should be greater than the ingest_interval in BatchIngester
sleep 1