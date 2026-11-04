# bluespark-simulation

## Running with ArduSub SITL

The image contains ArduSub SITL (ArduPilot `Sub-4.7.2`) and the
[`ardupilot_gazebo`](https://github.com/ArduPilot/ardupilot_gazebo) plugin.
The BlueROV2 model loads the plugin, so ArduSub drives the thrusters in Gazebo
and receives the simulated IMU and pose (JSON over UDP, port 9002).

The first image build takes 15-30 min because ArduSub is compiled from source.
After changing the `Dockerfile`, rebuild with `docker compose down && ./start_docker.sh`.

1. Terminal 1 (container): `gz sim -r worlds/bs_pool.sdf`
2. Terminal 2 (container): `bash scripts/run_ardusub.sh`
   Starts ArduSub and MAVProxy; wait for the `MANUAL>` prompt.
   Quick check: `arm throttle`, `rc 5 1600` (forward), `rc 5 1500`, `disarm`.
3. MAVLink is forwarded to `udp 127.0.0.1:14550`. To run the BlueSpark ROS 2
   stack against the simulation (repo `bluespark`, branch `pytrees_mvp`,
   `docker-compose.dev.yml`):

   ```bash
   ros2 launch bluespark_bringup governer.launch.py fcu_url:=udp://:14550@ run_autonomy:=false
   ```

Notes:
- Start Gazebo before ArduSub.
- `workspace/params/bluespark_sitl.parm` sets `MAV_GCS_SYSID=1` and
  `MAV_GCS_SYSID_HI=255` so ArduSub accepts RC override from MAVROS. Parameters
  are reset to defaults plus this file on every start (`-w`).
- The model is generated from a template. After editing `model.sdf.in` or
  `configs.yaml`, run (in the container):
  `python3 scripts/generate_model.py models/bluerov2/model.sdf.in models/bluerov2/model.sdf models/bluerov2/configs.yaml`

## Acknowledgements

This project uses 3D underwater simulation models from Gazebo Fuel created by **Xinyuan**, **Ryan Summers** and *possibly* other authors(`monkescripts`) and BlueRov2 models from **Clyde McQueen** and **Blue Robotics**. 

For full asset details, links, and BibTeX citations, see [ACKNOWLEDGEMENTS.md](./ACKNOWLEDGEMENTS.md).