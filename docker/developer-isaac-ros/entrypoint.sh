#!/bin/bash
#
# Copyright (c) 2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

source /opt/ros/${ROS_DISTRO}/setup.bash
if [ -d "/home/${USERNAME}/workspaces/isaac_ros-dev/install" ]; then
    source /home/${USERNAME}/workspaces/isaac_ros-dev/install/setup.bash
fi
$@