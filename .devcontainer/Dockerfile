############################################
# Klipper / Katapult development container #
# Build MCU firmware on x86 (laptop) hosts #
############################################

# ---- Base image -------------------------------------------------------------
FROM ubuntu:22.04

# ---- Non-interactive apt + minimal locales ----------------------------------
ARG DEBIAN_FRONTEND=noninteractive
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        ca-certificates build-essential make git \
        gcc-arm-none-eabi binutils-arm-none-eabi \
        python3 python3-venv python3-pip \
        ncurses-bin ncurses-dev \
        openssh-client less nano && \
    apt-get clean && rm -rf /var/lib/apt/lists/*

# ---- Create the unprivileged user used by VS Code ---------------------------
RUN useradd -ms /bin/bash vscode && \
    echo "vscode ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

USER vscode
WORKDIR /workspace