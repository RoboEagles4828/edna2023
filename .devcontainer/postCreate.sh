#!/bin/bash

# Configure omniverse cache 
sudo cp ./.devcontainer/omniverse.toml /root/.nvidia-omniverse/config/omniverse.toml

# Run the postinstall script for isaac sim
# Check to see if post install has already been run
if [ ! -d root/.cache/ov/Kit ]; then
    echo "ISAAC SIM POST INSTALL RUNNING... THIS WILL TAKE ABOUT 10min"
    sudo exec /isaac-sim/omni.isaac.sim.post.install.sh
fi                                                                                                                                                                                                                                                                                                      

# Setup ZSH
# echo 'alias isaac=/isaac-sim/isaac-sim.sh' >> ~/.zshrc

# Setup ROS2 Humble as the default bridge extension
basekit_file="/isaac-sim/apps/omni.isaac.sim.base.kit"
sudo sed -i 's/ros_bridge/ros2_bridge-humble/' $basekit_file