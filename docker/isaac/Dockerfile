FROM nvcr.io/nvidia/isaac-sim:2022.2.0

ENV INSTALL_SCRIPT_DIR=/opt/install-scripts
COPY install-scripts ${INSTALL_SCRIPT_DIR}
COPY utility-scripts ${UTILITY_SCRIPT_DIR}

ENV PATH="${UTILITY_SCRIPT_DIR}:${INSTALL_SCRIPT_DIR}:${PATH}"

RUN install-common
USER edna
RUN install-ros2

ENTRYPOINT [ "/bin/zsh" ]
