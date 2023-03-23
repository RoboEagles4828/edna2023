#!/bin/bash
source /opt/ros/${ROS_DISTRO}/setup.bash

colcon build --symlink-install --event-handlers --paths src/* --packages-skip edna_tests edna_debugger
source install/setup.bash
ros2 launch edna_bringup real.launch.py