#!/bin/bash
source /opt/ros/${ROS_DISTRO}/setup.bash

colcon build --symlink-install --paths src/*

source install/setup.bash
ros2 launch edna_bringup real.launch.py
