FROM mcr.microsoft.com/devcontainers/python:3.11

# 1. Install PlatformIO Core
RUN pip install -U platformio

# 2. Install udev (for device detection rules)
RUN apt-get update && apt-get install -y udev

# 3. Add the 'vscode' user to dialout/plugdev groups
# This allows access to /dev/ttyUSB0 without being root
RUN usermod -aG dialout,plugdev vscode
