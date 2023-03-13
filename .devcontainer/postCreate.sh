#!/bin/bash
git config devcontainers-theme.show-dirty 1
sed -i 's/ZSH_THEME="devcontainers"/ZSH_THEME="eastwood"/' ~/.zshrc

echo "source /opt/ros/humble/setup.zsh" >> ~/.zshrc
echo "source /home/admin/workspaces/rtab_ws/install/setup.zsh" >> ~/.zshrc
echo "if [ ! -d /workspaces/edna2023/install ]; then" >> ~/.zshrc
echo "echo -e 'No install folder found remember to build with ctrl + shift + b\n';" >> ~/.zshrc
echo "else source /workspaces/edna2023/install/setup.zsh; fi" >> ~/.zshrc
echo "export ROS_DISTRO=humble" >> ~/.zshrc
echo "export RCUTILS_COLORIZED_OUTPUT=1" >> ~/.zshrc
echo 'complete -o nospace -o default -F _python_argcomplete "ros2"' >> ~/.zshrc
echo 'alias python="python3"' >> ~/.zshrc


# Update apt list and rosdep
# Skipping since this will be done recently in docker file
# sudo apt-get update
# rosdep update --rosdistro=humble
rosdep install --from-paths src --ignore-src -r -y 
isaac rm &
isaac-ros rm &
rm -rf /workspaces/edna2023/install /workspaces/edna2023/build /workspaces/edna2023/log