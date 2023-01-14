#!/bin/bash
USERNAME=vscode

# Take ownership of persistent data folders
sudo chown -R $USERNAME:$USERNAME /home/$USERNAME/.cache
sudo chown -R $USERNAME:$USERNAME /home/$USERNAME/.local
sudo chown -R $USERNAME:$USERNAME /home/$USERNAME/.nv
sudo chown -R $USERNAME:$USERNAME /home/$USERNAME/.nvidia-omniverse

# Take ownership of isaac-sim folders
# (Might want to uncomment later (untested))
# sudo chown $USERNAME:$USERNAME /isaac-sim/kit
# sudo chown -R $USERNAME:$USERNAME /isaac-sim/kit/logs
# sudo chown $USERNAME:$USERNAME /isaac-sim/exts
# sudo chown -R $USERNAME:$USERNAME /isaac-sim/exts/omni.isaac.synthetic_recorder

# Configure omniverse cache 
cp ./.devcontainer/omniverse.toml /home/$USERNAME/.nvidia-omniverse/config/omniverse.toml

# Run the postinstall script for isaac sim
# Check to see if post install has already been run
if [ ! -d ~/.cache/ov/Kit ]; then
    echo "ISAAC SIM POST INSTALL RUNNING... THIS WILL TAKE ABOUT 10min"
    sudo bash /isaac-sim/omni.isaac.sim.post.install.sh
fi                                                                                                                                                                                                                                                                                                      

# Setup ZSH
echo 'alias isaac=/isaac-sim/isaac-sim.sh' >> ~/.zshrc

# Setup ROS2 Humble as the default bridge extension
basekit_file="/isaac-sim/apps/omni.isaac.sim.base.kit"
sudo sed -i 's/ros_bridge/ros2_bridge-humble/' $basekit_file