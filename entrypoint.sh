#!/bin/bash
set -e

# # Setup development environment
source /opt/ros/foxy/setup.bash
source /home/autodrive_devkit/install/setup.bash

# # Launch Competition Code
ls /home/autodrive_devkit
ls /home/autodrive_devkit/install
ls /home/autodrive_devkit/install/share
ls /home/autodrive_devkit/install/share/autodrive_iros_2024

ros2 launch autodrive_iros_2024 launch.py