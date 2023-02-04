FROM ghcr.io/roboeagles4828/galactic-isaac:latest

COPY utility-scripts ${UTILITY_SCRIPT_DIR}


RUN sudo chown -c edna:edna /isaac-sim/*
RUN sudo chown -c -R edna:edna /isaac-sim/exts
RUN sudo chown -c -R edna:edna /isaac-sim/kit
RUN sudo chown -c -R edna:edna /isaac-sim/jupyter_kernel
RUN sudo chown -c -R edna:edna /isaac-sim/data

RUN sudo mkdir -p /home/edna/.nvidia-omniverse/config
RUN sudo mkdir -p /home/edna/.nvidia-omniverse/logs
COPY omniverse.toml /home/edna/.nvidia-omniverse/config/omniverse.toml