#!/bin/bash
cd /home/ubuntu/edna2023/
source /opt/ros/humble/setup.bash
source ./install/setup.bash
export ROS_NAMESPACE=real
ros2 launch edna_bringup real.launch.py