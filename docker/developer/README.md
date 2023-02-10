# Docker for Development

This directory has the configuration for building the image used in the edna devcontainer.
It uses the osrf ros humble image as a base and installs edna related software on top.


When loading the devcontainer the common utils feature is used to update the user GID/UID to match local and setup zsh and other terminal nice to haves.

**Common Utils**: `devcontainers/features/common-utils` \
**URL**: https://github.com/devcontainers/features/tree/main/src/common-utils


# Build

1. Docker Login to github registry. [Guide](https://docs.github.com/en/packages/working-with-a-github-packages-registry/working-with-the-container-registry#authenticating-with-a-personal-access-token-classic) \
This will be used to push the image to the registry
2. Run `./build` to build the image. \
The script will prompt you if you would like to do a quick test or push to the registry.