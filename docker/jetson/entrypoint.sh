#!/bin/bash
source /opt/ros/${ROS_DISTRO}/setup.bash

colcon build --symlink-install

source install/setup.bash
ros2 launch edna_bringup real.launch.py