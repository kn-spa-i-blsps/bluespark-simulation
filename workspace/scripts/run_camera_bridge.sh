#!/usr/bin/env bash
# Bridge the underwater camera image and camera info from Gazebo to ROS 2:
#   /bluerov2/front_camera/underwater/image -> /bluerov2/front_camera/underwater/image_raw
#   /bluerov2/front_camera/camera_info      -> /bluerov2/front_camera/camera_info
# Run inside the container: bash scripts/run_camera_bridge.sh
set -e

source /opt/ros/jazzy/setup.bash
exec ros2 run ros_gz_bridge parameter_bridge \
    /bluerov2/front_camera/underwater/image@sensor_msgs/msg/Image[gz.msgs.Image \
    /bluerov2/front_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo \
    --ros-args -r /bluerov2/front_camera/underwater/image:=/bluerov2/front_camera/underwater/image_raw
