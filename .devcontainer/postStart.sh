#!/bin/bash

# Change permissions on any joystick devices
sudo chmod a+rw /dev/input/js*
source /opt/ros/humble/setup.bash

echo "JOYSTICKS CONNECTED: "
ros2 run joy joy_enumerate_devices
