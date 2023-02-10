FROM nvcr.io/nvidia/isaac/ros:x86_64-humble-nav2_7356480e8a4f3484b21ae13ec5c959ee
RUN mkdir -p /opt/install-scripts

################
# Config Setup #
################

# Copy middleware profiles
RUN mkdir -p /usr/local/share/middleware_profiles
COPY config/*_profile.xml /usr/local/share/middleware_profiles/

####################
# ROS Dependencies #
####################

# Copy and isntall docker
COPY install-scripts/install-ros-deps /opt/install-scripts/install-ros-deps
RUN chmod +x /opt/install-scripts/install-ros-deps
RUN /opt/install-scripts/install-ros-deps

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
