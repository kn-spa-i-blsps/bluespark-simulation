#!/usr/bin/env bash
# Start the underwater camera post-processing node (rebuilds it first if the
# sources changed). Gazebo must already be running.
# Run inside the container: bash scripts/run_underwater_camera.sh [config.yaml]
set -e

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

bash "$WORKSPACE_DIR/scripts/build_underwater_camera.sh" > /dev/null
source /opt/ros/jazzy/setup.bash
exec "$WORKSPACE_DIR/bin/underwater_camera_proc" \
    "${1:-$WORKSPACE_DIR/params/underwater_camera.yaml}"
