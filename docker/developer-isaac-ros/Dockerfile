FROM nvcr.io/nvidia/isaac/ros:x86_64-humble-nav2_7356480e8a4f3484b21ae13ec5c959ee
RUN mkdir -p /opt/install-scripts

################
# Config Setup #
################

# Copy middleware profiles
RUN mkdir -p /usr/local/share/middleware_profiles
COPY config/*_profile.xml /usr/local/share/middleware_profiles/

######################
# User configuration #
######################

ARG USERNAME=admin
ARG USER_UID=1000
ARG USER_GID=1000

# Install prerequisites
RUN apt-get update && apt-get install -y \
        sudo \
&& rm -rf /var/lib/apt/lists/* \
&& apt-get clean

# Reuse triton-server user as 'admin' user if exists
RUN if [ $(getent group triton-server) ]; then \
        groupmod --gid ${USER_GID} -n ${USERNAME} triton-server ; \
        usermod -l ${USERNAME} -u ${USER_UID} -m -d /home/${USERNAME} triton-server ; \
        mkdir -p /home/${USERNAME} ; \
        sudo chown ${USERNAME}:${USERNAME} /home/${USERNAME} ; \
    fi

# Create the 'admin' user if not already exists
RUN if [ ! $(getent passwd ${USERNAME}) ]; then \
        groupadd --gid ${USER_GID} ${USERNAME} ; \
        useradd --uid ${USER_UID} --gid ${USER_GID} -m ${USERNAME} ; \
    fi

# Update 'admin' user
RUN echo ${USERNAME} ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/${USERNAME} \
    && chmod 0440 /etc/sudoers.d/${USERNAME} \
    && adduser ${USERNAME} video && adduser ${USERNAME} sudo

# Add docker group
RUN groupadd --system docker
RUN usermod -aG docker ${USERNAME}

ENV USERNAME=${USERNAME}
ENV USER_GID=${USER_GID}
ENV USER_UID=${USER_UID}

####################################
# Install and Build Isaac ROS Pkgs #
####################################
USER ${USERNAME}

# Download
COPY install-scripts/install-isaac-pkgs /opt/install-scripts/install-isaac-pkgs
RUN sudo chmod +x /opt/install-scripts/install-isaac-pkgs
RUN /opt/install-scripts/install-isaac-pkgs

# Build
RUN sudo rm /usr/lib/x86_64-linux-gnu/libcuda.so.1
RUN sudo ln -s /usr/local/cuda/lib64/stubs/libcuda.so /usr/lib/x86_64-linux-gnu/libcuda.so.1
RUN cd /home/${USERNAME}/workspaces/isaac_ros-dev && source /opt/ros/${ROS_DISTRO}/setup.bash && colcon build --symlink-install

#####################
# Interactive Setup #
#####################

WORKDIR /home/${USERNAME}/workspaces/isaac_ros-dev
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /home/${USERNAME}/.bashrc
RUN echo "source /home/${USERNAME}/workspaces/isaac_ros-dev/install/setup.bash" >> /home/${USERNAME}/.bashrc
COPY entrypoint.sh /home/${USERNAME}/entrypoint.sh
RUN sudo chmod +x /home/${USERNAME}/entrypoint.sh