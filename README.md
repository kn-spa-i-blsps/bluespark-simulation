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

## Object detection dataset

`worlds/dataset_pool.sdf` is a "photo studio": the pool, the labelled obstacles
and a free camera (`models/dataset_camera`), without the drone. Gazebo's
`boundingbox_camera` saves every frame with the 2D boxes of the labelled objects;
a script moves the camera to random poses between frames.

1. In one terminal (container), start the world **paused** (no `-r`):
   `gz sim worlds/dataset_pool.sdf`
2. In another terminal (container):
   `python3 scripts/generate_dataset.py --count 500`
   Frames go to `workspace/dataset/raw` (`images/*.png`, `boxes/*.csv`).
3. Convert to a YOLO dataset (train/val split, `data.yaml`):
   `python3 scripts/convert_to_yolo.py`
   Result in `workspace/dataset/yolo`.

Notes:
- Classes: a Label system on each obstacle in `dataset_pool.sdf` sets its class
  id; the names are `CLASS_NAMES` in `convert_to_yolo.py`. Gazebo merges all
  labelled parts of one model into one box, so a part that needs its own class
  must be a separate model.
- Camera poses (distance, depth, aim) are set at the top of `generate_dataset.py`.
  `--seed` repeats the same poses.
- `workspace/dataset/raw` must be empty or missing before a run: Gazebo numbers
  the saved frames from 0 after every start and would overwrite older ones.
  `generate_dataset.py` stops if it finds frames there.
- `workspace/dataset/` is in `.gitignore`.
- Underwater effects (issue #4) are not applied yet.

## Acknowledgements

This project uses 3D underwater simulation models from Gazebo Fuel created by **Xinyuan**, **Ryan Summers** and *possibly* other authors(`monkescripts`) and BlueRov2 models from **Clyde McQueen** and **Blue Robotics**. 

For full asset details, links, and BibTeX citations, see [ACKNOWLEDGEMENTS.md](./ACKNOWLEDGEMENTS.md).