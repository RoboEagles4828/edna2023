#!/bin/bash
git config devcontainers-theme.show-dirty 1
sed -i 's/ZSH_THEME="devcontainers"/ZSH_THEME="eastwood"/' ~/.zshrc

echo "source /opt/ros/humble/setup.zsh" >> ~/.zshrc
echo "if [ ! -d /workspaces/edna2023/install ]; then" >> ~/.zshrc
echo "echo -e 'No install folder found remember to build with ctrl + shift + b\n';" >> ~/.zshrc
echo "else source /workspaces/edna2023/install/setup.zsh; fi" >> ~/.zshrc
echo "export ROS_DISTRO=humble" >> ~/.zshrc
echo "export RCUTILS_COLORIZED_OUTPUT=1" >> ~/.zshrc

# Update apt list and rosdep
sudo apt-get update
rosdep update --rosdistro=humble
rosdep install --from-paths src --ignore-src -r -y
isaac rm