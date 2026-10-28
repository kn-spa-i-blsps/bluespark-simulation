#!/usr/bin/env bash
# Start ArduSub SITL connected to Gazebo through the ardupilot_gazebo plugin.
# Start Gazebo first: gz sim -r worlds/bs_pool.sdf
# Extra arguments are passed on to sim_vehicle.py.
set -e
cd "$HOME/ardupilot/ArduSub"
exec "$HOME/ardupilot/Tools/autotest/sim_vehicle.py" \
    -v ArduSub -f vectored --model JSON -L RATBeach -w \
    --add-param-file="$HOME/workspace/params/bluespark_sitl.parm" \
    "$@"