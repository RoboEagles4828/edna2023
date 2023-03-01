#!/bin/bash
source /opt/ros/${ROS_DISTRO}/setup.bash
if [ -d "/home/${USERNAME}/workspaces/isaac_ros-dev/install" ]; then
    source /home/${USERNAME}/workspaces/isaac_ros-dev/install/setup.bash
fi
$@