#!/bin/bash
ORANGE='\033[0;33m'
RED='\033[0;31m'
GREEN='\033[0;32m'
NC='\033[0m'
SCRIPT_PATH=$(dirname "$0")
devenv_path="$SCRIPT_PATH/../.devcontainer/.env"

# Graphics Driver Check
#hasDriver=$(nvidia-smi | grep "Version: 525")
#if [[ -z "$hasDriver" ]]; then
#  echo -e "${RED}Please install nvidia driver 525 before this install script${NC}"
#  exit 1
# fi

echo -e "${ORANGE}INSTALLING APT PACKAGES${NC}"
sudo apt-get update
sudo apt-get install \
    apt-transport-https \
    ca-certificates \
    curl \
    gnupg-agent \
    software-properties-common \
    git-lfs -y

# Stop ubuntu pop ups that applications are not responding
gsettings set org.gnome.mutter check-alive-timeout 60000

# Setup a unique domain 
# Avoids conflicts with others on the same network
if [ -z "$(cat $devenv_path | grep ROS_NAMESPACE)" ]; then
  echo -e "${ORANGE}SETTING ROS_NAMESPACE${NC}"
  read -p "Enter a name for your ROS_NAMESPACE: " ros_namespace
  echo "ROS_NAMESPACE=${ros_namespace}" >> $devenv_path
else
  echo -e "${GREEN}ROS_NAMESPACE ALREADY SET${NC}"
fi

# Docker
if [[ -z "$(which docker)" ]]; then
  echo -e "${ORANGE}INSTALLING DOCKER${NC}"
  curl https://get.docker.com | sh \
  && sudo systemctl --now enable docker
  sudo usermod -aG docker $USER
else
  echo -e "${GREEN}DOCKER ALREADY INSTALLED${NC}"
fi


# Nividia Docker
if ! dpkg -s nvidia-docker2 > /dev/null; then
  echo -e "${ORANGE}INSTALLING NVIDIA DOCKER${NC}"
  distribution=$(. /etc/os-release;echo $ID$VERSION_ID) \
  && curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg \
  && curl -s -L https://nvidia.github.io/libnvidia-container/$distribution/libnvidia-container.list | \
    sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
    sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list
  sudo apt-get update
  sudo apt-get install -y nvidia-docker2
  sudo systemctl restart docker
else
  echo -e "${GREEN}NVIDIA DOCKER ALREADY INSTALLED${NC}"
fi