#!/bin/bash
set -e

# # Setup development environment
source /opt/ros/foxy/setup.bash
source /home/autodrive_devkit/install/setup.bash

# # Launch Competition Code
ros2 launch autodrive_iros_2024 launch_prod.py