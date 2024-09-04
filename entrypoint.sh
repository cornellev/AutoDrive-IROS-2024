#!/bin/bash
set -e

# Setup development environment
source /opt/ros/foxy/setup.bash
source /home/autodrive_devkit/ros2_ws/install/setup.bash

# Launch AutoDRIVE Devkit (ROS 2 API)
ros2 launch autodrive_f1tenth simulator_bringup_rviz.launch.py
