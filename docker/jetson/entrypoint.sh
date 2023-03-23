#!/bin/bash
source /opt/ros/${ROS_DISTRO}/setup.bash

# Check the rebuild level
if test -f ".rsyncmetadata"; then
    echo "RSync metadata found, checking what to rebuild..."
    CHANGED_FILES=$(<.rsyncmetadata)
    if egrep -q "\.cpp\b|\.hpp\b|\.c\b|\.h\b|\.yaml\b" <<< $CHANGED_FILES; then
        # Figure out what packages to rebuild...
        REBUILD_PACKAGES=$(grep 'src/' <<< "$CHANGED_FILES" | tr '/' ' ' | awk '{print $2}' | nl | sort -u -k2 | sort -n | cut -f2- | tr '\n' ' ')
        echo "Rebuilding Packages: $REBUILD_PACKAGES"
        colcon build --symlink-install --paths src/* --packages-select $REBUILD_PACKAGES
    else
        echo "No critical source files changed, not rebuilding."
    fi 
    rm .rsyncmetadata
else
    echo "RSync metadata not found, rebuilding all packages..."
    colcon build --symlink-install --paths src/*
fi

source install/setup.bash
ros2 launch edna_bringup real.launch.py