#!/bin/bash

# Create and activate virtual environment if it doesn't exist
VENV_DIR="./venv"
if [ ! -d "$VENV_DIR" ]; then
    echo "Creating virtual environment..."
    python3 -m venv $VENV_DIR
fi

# Activate virtual environment
source $VENV_DIR/bin/activate

# Source ROS2
source /opt/ros/*/setup.bash

# Optional ROS environment variables
#export ROS_DOMAIN_ID=1
#export ROS_LOCALHOST_ONLY=1 #un-comment for local only ingestion
#export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp


# Check for FORMANT_ROS2_WS
if [ -z ${FORMANT_ROS2_WS+x} ]; then
    echo "FORMANT_ROS2_WS unset"
else
    echo "sourcing $FORMANT_ROS2_WS"
    source $FORMANT_ROS2_WS
fi

export PYTHONUNBUFFERED=true

# Install requirements in virtual environment
python3 -m pip install -r requirements.txt

cd formant_ros2_adapter/scripts/
python3 main.py

# Deactivate virtual environment when done
deactivate

