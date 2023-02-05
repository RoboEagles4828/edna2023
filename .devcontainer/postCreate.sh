#!/bin/bash
git config devcontainers-theme.show-dirty 1
sed -i 's/ZSH_THEME="devcontainers"/ZSH_THEME="eastwood"/' ~/.zshrc