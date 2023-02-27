FROM ros:humble-ros-base
RUN mkdir -p /opt/install-scripts
RUN mkdir -p /opt/workspace

################
# Config Setup #
################

# Copy middleware profiles
RUN mkdir -p /usr/local/share/middleware_profiles
COPY config/*_profile.xml /usr/local/share/middleware_profiles/


####################
# Preload ROS Deps #
####################

# Copy and install workspace rosdeps
COPY install-scripts/install-rosdeps /opt/install-scripts/install-rosdeps
RUN chmod +x /opt/install-scripts/install-rosdeps
COPY src /tmp/src
RUN /opt/install-scripts/install-rosdeps

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


COPY entrypoint.sh /opt/entrypoint.sh
USER ${USERNAME}
WORKDIR /opt/workspace
ENTRYPOINT [ "/bin/bash", "/opt/entrypoint.sh" ]