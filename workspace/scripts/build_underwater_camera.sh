#!/usr/bin/env bash
# Build underwater_camera_proc into workspace/bin (incremental, fast when nothing changed).
# Run inside the container: bash scripts/build_underwater_camera.sh
set -e

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

# Gazebo Harmonic libraries come from the ROS 2 Jazzy vendor packages.
source /opt/ros/jazzy/setup.bash

cmake -S "$WORKSPACE_DIR/src/underwater_camera_proc" -B "$WORKSPACE_DIR/build" \
    -DCMAKE_BUILD_TYPE=Release
cmake --build "$WORKSPACE_DIR/build" -j"$(nproc)"
