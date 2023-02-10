#!/bin/bash
git config devcontainers-theme.show-dirty 1
sed -i 's/ZSH_THEME="devcontainers"/ZSH_THEME="eastwood"/' ~/.zshrc

echo "source /opt/ros/humble/setup.zsh" >> ~/.zshrc
echo "export ROS_DISTRO=humble" >> ~/.zshrc
echo "export ROS_DOMAIN_ID=0" >> ~/.zshrc

# Update
sudo apt-get update
rosdep update --rosdistro=humble
