#!/bin/bash
source /opt/ros/jazzy/setup.bash
source ~/msds_ws/install/setup.bash
export ROS_DOMAIN_ID=2
ros2 launch msds_modes standby.launch.py

# chmod +x launch.sh
