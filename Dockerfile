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


    USER root
RUN apt-get update && apt-get install -y \
    rapidjson-dev \
    libopencv-dev \
    libgstreamer1.0-dev \
    libgstreamer-plugins-base1.0-dev \
    && rm -rf /var/lib/apt/lists/*
USER $USERNAME

ENV GZ_VERSION=harmonic
ARG ARDUPILOT_GAZEBO_COMMIT=80dc9664dda7efd6c5d1a71a79198d1d6b708c0e
RUN git clone https://github.com/ArduPilot/ardupilot_gazebo.git /home/$USERNAME/ardupilot_gazebo \
    && cd /home/$USERNAME/ardupilot_gazebo \
    && git checkout ${ARDUPILOT_GAZEBO_COMMIT} \
    && mkdir build && cd build \
    && bash -c "source /opt/ros/jazzy/setup.bash && cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo && make -j4"

WORKDIR /home/$USERNAME/workspace

RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
RUN echo "export GZ_SIM_RESOURCE_PATH=/home/ubuntu/workspace/models:$GZ_SIM_RESOURCE_PATH" >> ~/.bashrc
RUN echo 'export GZ_SIM_SYSTEM_PLUGIN_PATH=$HOME/ardupilot_gazebo/build:$GZ_SIM_SYSTEM_PLUGIN_PATH' >> ~/.bashrc
RUN echo 'export PATH=$PATH:$HOME/ardupilot/Tools/autotest' >> ~/.bashrc