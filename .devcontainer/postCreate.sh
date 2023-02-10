#!/bin/bash
git config devcontainers-theme.show-dirty 1
sed -i 's/ZSH_THEME="devcontainers"/ZSH_THEME="eastwood"/' ~/.zshrc

sudo mkdir -p /usr/local/share/middleware_profiles
sudo cp /workspaces/edna2023/docker/developer/config/rtps_udp_profile.xml /usr/local/share/middleware_profiles/
echo "source /opt/ros/humble/setup.zsh" >> ~/.zshrc
echo "export ROS_DISTRO=humble" >> ~/.zshrc
echo "export ROS_DOMAIN_ID=0" >> ~/.zshrc

# Update
sudo apt-get update
rosdep update --rosdistro=humble

# Take ownership of docker.socket
# sudo chown root:docker /var/run/docker.sock