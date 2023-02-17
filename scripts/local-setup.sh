#!/bin/bash
ORANGE='\033[0;33m'
RED='\033[0;31m'
GREEN='\033[0;32m'
NC='\033[0m'

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
    git-lfs

# Git lfs for ros-isaac
# git lfs install
# Stop ubuntu pop ups that applications are not responding
gsettings set org.gnome.mutter check-alive-timeout 60000

if [[ -z "$(which docker)" ]]; then
  echo -e "${ORANGE}INSTALLING DOCKER${NC}"
  curl https://get.docker.com | sh \
  && sudo systemctl --now enable docker
  sudo usermod -aG docker $USER
else
  echo -e "${GREEN}DOCKER ALREADY INSTALLED${NC}"
fi


if [[ -z "$(apt list 2> /dev/null | grep nvidia-docker2)" ]]; then
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
