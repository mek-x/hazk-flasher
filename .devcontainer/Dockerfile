FROM mcr.microsoft.com/devcontainers/python:3.11

# 1. Install PlatformIO Core
RUN pip install -U platformio

# 2. Install udev (for device detection rules)
RUN apt-get update && apt-get install -y udev

# 3. (Optional for RHEL based systems) Add the GID 18 group and add the user to it.
RUN groupadd -g 18 host-serial || true \
    && usermod -aG $(getent group 18 | cut -d: -f1) vscode

# 4. Add the 'vscode' user to dialout/plugdev groups
# This allows access to /dev/ttyUSB0 without being root
RUN usermod -aG dialout,plugdev vscode
