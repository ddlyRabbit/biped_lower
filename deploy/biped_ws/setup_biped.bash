#!/bin/bash
# Source this file to set up the biped workspace
# Usage: source ~/biped_lower/deploy/biped_ws/setup_biped.bash

# ROS2 Jazzy
source /opt/ros/jazzy/setup.bash

# Biped workspace
source ~/biped_lower/deploy/biped_ws/install/setup.bash

# Ensure biped_msgs and other packages are found
export AMENT_PREFIX_PATH=$HOME/biped_lower/deploy/biped_ws/install/biped_msgs:$HOME/biped_lower/deploy/biped_ws/install/biped_description:$HOME/biped_lower/deploy/biped_ws/install/biped_driver:$HOME/biped_lower/deploy/biped_ws/install/biped_driver_cpp:$HOME/biped_lower/deploy/biped_ws/install/biped_control:$HOME/biped_lower/deploy/biped_ws/install/biped_bringup:$HOME/biped_lower/deploy/biped_ws/install/biped_teleop:$HOME/biped_lower/deploy/biped_ws/install/biped_tools:$AMENT_PREFIX_PATH
