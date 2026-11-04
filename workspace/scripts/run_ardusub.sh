#!/usr/bin/env bash
#
# Start ArduSub SITL connected to the Gazebo simulation.
#
# ArduSub SITL is the ArduSub firmware compiled for Linux (see the Dockerfile).
# Here it plays the role of the Pixhawk: it gets the vehicle state from Gazebo
# through ArduPilotPlugin (JSON over UDP port 9002) and sends motor PWM back.
#
# Usage, inside the bs_sim container:
#   1. Terminal 1: gz sim -r worlds/bs_pool.sdf      (Gazebo must run first)
#   2. Terminal 2: bash scripts/run_ardusub.sh
#   3. Wait for the MAVProxy prompt "MANUAL>". Quick check:
#        arm throttle     arm the vehicle
#        rc 5 1600        forward (channel 5 = forward/back, 1500 = neutral)
#        rc 5 1500        stop
#        disarm
#
# sim_vehicle.py starts two programs: ArduSub itself (in the background) and
# MAVProxy (in this terminal). MAVProxy also forwards all MAVLink traffic to
# udp 127.0.0.1:14550, which is where MAVROS connects from the bluespark
# container:
#   ros2 launch bluespark_bringup governer.launch.py fcu_url:=udp://:14550@ run_autonomy:=false
#
# Any extra arguments are passed on to sim_vehicle.py
# (see: $HOME/ardupilot/Tools/autotest/sim_vehicle.py --help).
#
set -e

# sim_vehicle.py keeps its working files (parameter storage eeprom.bin, logs)
# in the current directory, so run it from the ArduSub folder.
cd "$HOME/ardupilot/ArduSub"

# Full path because this script does not read ~/.bashrc (where PATH is set).
#   -v ArduSub        vehicle type
#   -f vectored       frame: BlueROV2 with 6 thrusters (FRAME_CONFIG = vectored)
#   --model JSON      use an external physics engine over JSON (Gazebo via
#                     ArduPilotPlugin) instead of ArduSub's built-in sub model
#   -L RATBeach       start location from Tools/autotest/locations.txt; ArduSub
#                     needs geographic coordinates, the place itself is irrelevant
#   -w                wipe stored parameters, start from defaults every time so
#                     everyone gets the same configuration
#   --add-param-file  then apply the BlueSpark simulation parameters
#                     (workspace/params/bluespark_sitl.parm)
# exec replaces this shell with sim_vehicle.py, so Ctrl+C goes straight to it.
exec "$HOME/ardupilot/Tools/autotest/sim_vehicle.py" \
    -v ArduSub -f vectored --model JSON -L RATBeach -w \
    --add-param-file="$HOME/workspace/params/bluespark_sitl.parm" \
    "$@"
