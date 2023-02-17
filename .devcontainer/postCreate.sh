#!/bin/bash
git config devcontainers-theme.show-dirty 1
sed -i 's/ZSH_THEME="devcontainers"/ZSH_THEME="eastwood"/' ~/.zshrc

echo "source /opt/ros/humble/setup.zsh" >> ~/.zshrc
echo "source /workspaces/edna2023/install/setup.zsh" >> ~/.zshrc
echo "export ROS_DISTRO=humble" >> ~/.zshrc
echo "export ROS_DOMAIN_ID=0" >> ~/.zshrc

# Update apt list and rosdep
sudo apt-get update
rosdep update --rosdistro=humble
rosdep install --from-paths src --ignore-src -r -y