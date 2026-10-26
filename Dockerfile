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

ARG USERNAME=ubuntu
RUN echo "$USERNAME ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

ENV XDG_RUNTIME_DIR=/tmp/runtime-$USERNAME
RUN mkdir -p $XDG_RUNTIME_DIR && \
    chown -R $USERNAME:$USERNAME $XDG_RUNTIME_DIR && \
    chmod 0700 $XDG_RUNTIME_DIR

USER $USERNAME
WORKDIR /home/$USERNAME/workspace

RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
RUN echo "export GZ_SIM_RESOURCE_PATH=/home/ubuntu/workspace/models:$GZ_SIM_RESOURCE_PATH" >> ~/.bashrc