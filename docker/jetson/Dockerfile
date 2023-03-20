ARG VERSION=2
FROM jetson-base:${VERSION}

RUN mkdir -p /opt/workspace
COPY entrypoint.sh /opt/entrypoint.sh
USER ${USERNAME}
WORKDIR /opt/workspace
ENTRYPOINT [ "/bin/bash", "/opt/entrypoint.sh" ]