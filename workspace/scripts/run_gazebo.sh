#!/usr/bin/env bash
#
# Start Gazebo with the BlueSpark pool world (the first step of every
# simulation session: ArduSub, the camera and the bridge all need it running).
#
# Usage, inside the bs_sim container:
#   bash scripts/run_gazebo.sh                  pool world, simulation running (-r)
#   bash scripts/run_gazebo.sh worlds/x.sdf     another world
#   bash scripts/run_gazebo.sh worlds/x.sdf -s  extra arguments go to `gz sim`
#                                               (e.g. -s = server only, no GUI)
# Then, in another terminal: bash scripts/run_ardusub.sh
#
set -e

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# World paths are relative to the workspace, like in the README.
cd "$WORKSPACE_DIR"

WORLD="${1:-worlds/bs_pool.sdf}"
shift || true

# This script does not read ~/.bashrc, so set here what the Dockerfile puts
# there. The ROS 2 environment provides the gz command itself (Gazebo Harmonic
# comes with ROS Jazzy's vendor packages), then: where Gazebo finds our models
# (model://bluerov2, ...) and the ArduPilotPlugin (ardupilot_gazebo) that
# connects the BlueROV2 to ArduSub.
source /opt/ros/jazzy/setup.bash
export GZ_SIM_RESOURCE_PATH="$WORKSPACE_DIR/models:${GZ_SIM_RESOURCE_PATH:-}"
export GZ_SIM_SYSTEM_PLUGIN_PATH="$HOME/ardupilot_gazebo/build:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"

# -r: start with the simulation running, not paused.
# exec replaces this shell with Gazebo, so Ctrl+C goes straight to it.
exec gz sim -r "$WORLD" "$@"
