FROM ros:jazzy-ros-base

RUN apt-get update && apt-get install -y \
    ros-jazzy-ros-gz \
    mesa-utils \
    libgl1 \
    libglx-mesa0 \
    libgl1-mesa-dri \
    ca-certificates \
    curl \
    && rm -rf /var/lib/apt/lists/*


RUN apt-get update && apt-get install -y \
    git \
    build-essential \
    python3-pip \
    python3-numpy \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install --break-system-packages \
        empy==3.3.4 pexpect future dronecan \
        pymavlink pyserial defusedxml pynmeagps \
    && pip3 install --break-system-packages --no-deps MAVProxy

ARG USERNAME=ubuntu
RUN echo "$USERNAME ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

ENV XDG_RUNTIME_DIR=/tmp/runtime-$USERNAME
RUN mkdir -p $XDG_RUNTIME_DIR && \
    chown -R $USERNAME:$USERNAME $XDG_RUNTIME_DIR && \
    chmod 0700 $XDG_RUNTIME_DIR

USER $USERNAME

ARG ARDUSUB_VERSION=Sub-4.7.2
RUN git clone --branch ${ARDUSUB_VERSION} --depth 1 \
        --recurse-submodules --shallow-submodules \
        https://github.com/ArduPilot/ardupilot.git /home/$USERNAME/ardupilot \
    && cd /home/$USERNAME/ardupilot \
    && ./waf configure --board sitl \
    && ./waf sub

WORKDIR /home/$USERNAME/workspace

RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
RUN echo "export GZ_SIM_RESOURCE_PATH=/home/ubuntu/workspace/models:$GZ_SIM_RESOURCE_PATH" >> ~/.bashrc

RUN echo 'export PATH=$PATH:$HOME/ardupilot/Tools/autotest' >> ~/.bashrc