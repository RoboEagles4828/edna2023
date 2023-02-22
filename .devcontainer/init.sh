#!/bin/bash
# Create the directory where isaac sim will save to.
mkdir -p ~/docker/isaac-sim

# Find nvidia and vulkan driver config files
nvidia_driver_config_path="scripts/config/nvidia_driver_config"
nvidia_layers=""
nvidia_icd=""
_10_nvidia=""

nvidia_layer_loc1=$(find /etc/vulkan -name nvidia_layers.json)
nvidia_layer_loc2=$(find /usr/share/vulkan -name nvidia_layers.json)
if [[ -n "$nvidia_layer_loc1" ]]; then nvidia_layers=$nvidia_layer_loc1
elif [[ -n "$nvidia_layer_loc2" ]]; then nvidia_layers=$nvidia_layer_loc2
fi

nvidia_icd_loc1=$(find /usr/share/vulkan -name nvidia_icd.json)
nvidia_icd_loc2=$(find /etc/vulkan -name nvidia_icd.json)
if [[ -n "$nvidia_icd_loc1" ]]; then nvidia_icd="$nvidia_icd_loc1"
elif [[ -n "$nvidia_icd_loc2" ]]; then nvidia_icd="$nvidia_icd_loc2"
fi

_10_nvidia_loc1=$(find /usr/share/glvnd -name 10_nvidia.json)
if [[ -n "$_10_nvidia_loc1" ]]; then _10_nvidia="$_10_nvidia_loc1"
fi

echo -e "\
nvidia_layers=${nvidia_layers}\n\
nvidia_icd=${nvidia_icd}\n\
_10_nvidia=${_10_nvidia}" > "${nvidia_driver_config_path}"


XSOCK=/tmp/.X11-unix
XAUTH=/tmp/.docker.xauth
touch $XAUTH
echo "Display to use: $DISPLAY"
xauth nlist $DISPLAY | sed -e 's/^..../ffff/' | xauth -f $XAUTH nmerge -