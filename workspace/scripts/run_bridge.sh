#!/usr/bin/env bash
#
# Start the ROS 2 <-> Gazebo bridge for the simulated sensors (front camera).
#
# Gazebo and ROS 2 have separate topic systems that do not see each other.
# ros_gz_bridge (installed with ros-jazzy-ros-gz, see the Dockerfile) listens
# on Gazebo topics and republishes the messages on ROS 2 topics. Which topics
# it bridges, and under which ROS names, is set in config/ros_gz_bridge.yaml.
#



set -e

# This script does not read ~/.bashrc, so load the ROS 2 environment here
# (provides the ros2 command and the ros_gz_bridge package).
source /opt/ros/jazzy/setup.bash

# ROS 2 programs only see each other when they use the same domain ID.
# The BlueSpark stack (bluespark repo, docker-compose*.yml) uses 42, so the
# bridge must publish in domain 42 too, otherwise vision_node receives nothing.
export ROS_DOMAIN_ID=42

# exec replaces this shell with the bridge, so Ctrl+C goes straight to it.
exec ros2 run ros_gz_bridge parameter_bridge \
    --ros-args -p config_file:="$HOME/workspace/config/ros_gz_bridge.yaml"