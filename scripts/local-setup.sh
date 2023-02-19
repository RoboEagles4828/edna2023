#!/bin/bash
ORANGE='\033[0;33m'
RED='\033[0;31m'
GREEN='\033[0;32m'
NC='\033[0m'

# Graphics Driver Check
hasDriver=$(nvidia-smi | grep "Version: 525")
if [[ -z "$hasDriver" ]]; then
  echo -e "${RED}Please install nvidia driver 525 before this install script${NC}"
  exit 1
fi

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
requre_reload=false
if [ -z "$(cat ~/.bashrc | grep ROS_DOMAIN_ID)" ]; then
  echo -e "${ORANGE}SETTING ROS_DOMAIN_ID${NC}"
  read -p "Enter a unique number that is not zero for ROS_DOMAIN_ID: " domain_id
  echo "export ROS_DOMAIN_ID=${domain_id}" >> ~/.bashrc
  requre_reload=true
elif [ "$ROS_DOMAIN_ID" == "0" ]; then
  echo -e "${RED}ROS_DOMAIN_ID is set to 0, please change it to a unique number${NC}"
  read -p "Enter a unique number for ROS_DOMAIN_ID: " domain_id
  sed -i "s/ROS_DOMAIN_ID=0/ROS_DOMAIN_ID=${domain_id}/g" ~/.bashrc
  requre_reload=true
else
  echo -e "${GREEN}ROS_DOMAIN_ID ALREADY SET${NC}"
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

if [ requre_reload ]; then
  echo "Reload VS Code to apply changes"
fi
