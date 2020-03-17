#!/bin/bash
set -e

# Use our ROS Catkin workspace (sets up PYTHONPATH etc)
source "/opt/ros/melodic/install/setup.bash"
exec "$@"
