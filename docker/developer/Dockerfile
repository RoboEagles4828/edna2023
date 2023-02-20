FROM osrf/ros:humble-desktop
RUN mkdir -p /opt/install-scripts

################
# Config Setup #
################

# Copy middleware profiles
RUN mkdir -p /usr/local/share/middleware_profiles
COPY config/*_profile.xml /usr/local/share/middleware_profiles/

##################
# Common Utility #
##################

# Copy and install common
COPY install-scripts/install-common /opt/install-scripts/install-common
RUN chmod +x /opt/install-scripts/install-common
RUN /opt/install-scripts/install-common

##############
# Docker CLI #
##############

# Copy and isntall docker
COPY install-scripts/install-docker /opt/install-scripts/install-docker
RUN chmod +x /opt/install-scripts/install-docker
RUN /opt/install-scripts/install-docker

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


# Fix for Docker Permissions
RUN echo -e "#!/bin/sh\n\
    sudoIf() { if [ \"\$(id -u)\" -ne 0 ]; then sudo \"\$@\"; else \"\$@\"; fi }\n\
    SOCKET_GID=\$(stat -c '%g' /var/run/docker.sock) \n\
    if [ \"${SOCKET_GID}\" != '0' ]; then\n\
        if [ \"\$(cat /etc/group | grep :\${SOCKET_GID}:)\" = '' ]; then sudoIf groupadd --gid \${SOCKET_GID} docker-host; fi \n\
        if [ \"\$(id ${USERNAME} | grep -E \"groups=.*(=|,)\${SOCKET_GID}\(\")\" = '' ]; then sudoIf usermod -aG \${SOCKET_GID} ${USERNAME}; fi\n\
    fi\n\
    exec \"\$@\"" > /usr/local/share/docker-init.sh \
    && chmod +x /usr/local/share/docker-init.sh

# VS Code overrides ENTRYPOINT and CMD when executing `docker run` by default.
# Setting the ENTRYPOINT to docker-init.sh will configure non-root access to
# the Docker socket if "overrideCommand": false is set in devcontainer.json.
# The script will also execute CMD if you need to alter startup behaviors.
ENTRYPOINT [ "/usr/local/share/docker-init.sh" ]
CMD [ "sleep", "infinity" ]