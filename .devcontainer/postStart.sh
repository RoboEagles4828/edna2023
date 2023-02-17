#!/bin/bash

# Change permissions on any joystick devices
sudo chmod a+rw /dev/input/js*
source /opt/ros/humble/setup.bash

echo -e "\n----JOYSTICKS CONNECTED----"
ros2 run joy joy_enumerate_devices

echo -e "\n----ISAAC SIM COMMANDS----"
echo "isaac-setup     : Setup Isaac Sim Shaders"
echo "isaac           : Start Isaac Sim"
echo "isaac rm        : Stop and delete isaac sim container"

echo -e "\n----COMMANDS----"
echo "launch          : Launch sim teleop code"
echo "launch isaac    : Launch sim teleop code"
echo "launch real     : Launch real teleop code"
echo "launch test_hw  : Launch rviz with teleop code"
echo "restart-ros2    : restart ros2 daemon"

echo -e "\n----Build Command----"
echo -e "\nctrl + shift + b  : Build"