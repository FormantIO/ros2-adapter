#!/bin/bash

# Function to generate random float between min and max
random_float() {
    local min=$1
    local max=$2
    echo "scale=2; $min + ($max - $min) * $RANDOM / 32767" | bc
}

# String messages
string_messages=(
    "Hello from the adapter!"
    "Adapter is running smoothly"
    "Greetings from your friendly adapter"
    "Connection established via adapter"
    "Adapter status: Amazing"
)

while true; do
    # String publish
    random_msg=${string_messages[$RANDOM % ${#string_messages[@]}]}
    ros2 topic pub --once /stringpub std_msgs/msg/String "data: \"$random_msg\""
    
    # Char publish (ASCII value between 33 and 126 for printable characters)
    ros2 topic pub --once /charpub std_msgs/msg/Char "data: $(($RANDOM % 94 + 33))"
    
    # UInt8 (0-255)
    ros2 topic pub --once /uint8pub std_msgs/msg/UInt8 "data: $(($RANDOM % 256))"
    
    # UInt16 (0-65535)
    ros2 topic pub --once /uint16pub std_msgs/msg/UInt16 "data: $(($RANDOM % 65536))"
    
    # UInt32 (using multiple RANDOM calls for larger range)
    ros2 topic pub --once /uint32pub std_msgs/msg/UInt32 "data: $(($RANDOM * $RANDOM))"
    
    # UInt64 (using multiple RANDOM calls for larger range)
    ros2 topic pub --once /uint64pub std_msgs/msg/UInt64 "data: $(($RANDOM * $RANDOM * $RANDOM))"
    
    # Int16 (-32768 to 32767)
    ros2 topic pub --once /int16pub std_msgs/msg/Int16 "data: $(($RANDOM - 32768))"
    
    # Int32 (using multiple RANDOM calls for larger range)
    ros2 topic pub --once /int32pub std_msgs/msg/Int32 "data: $((($RANDOM - 16384) * $RANDOM))"
    
    # Int64 (using multiple RANDOM calls for larger range)
    ros2 topic pub --once /int64pub std_msgs/msg/Int64 "data: $((($RANDOM - 16384) * $RANDOM * $RANDOM))"
    
    # Float32 (random float between -100 and 100)
    ros2 topic pub --once /float32pub std_msgs/msg/Float32 "data: $(random_float -100 100)"
    
    # Float64 (random float between -1000 and 1000)
    ros2 topic pub --once /float64pub std_msgs/msg/Float64 "data: $(random_float -1000 1000)"
    
    sleep 2
done
