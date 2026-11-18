# bluespark-simulation

## Running with ArduSub SITL

The image contains ArduSub SITL (ArduPilot `Sub-4.7.2`) and the
[`ardupilot_gazebo`](https://github.com/ArduPilot/ardupilot_gazebo) plugin.
The BlueROV2 model loads the plugin, so ArduSub drives the thrusters in Gazebo
and receives the simulated IMU and pose (JSON over UDP, port 9002).

The first image build takes 15-30 min because ArduSub is compiled from source.
After changing the `Dockerfile`, rebuild with `docker compose down && ./start_docker.sh`.

1. Terminal 1 (container): `bash scripts/run_gazebo.sh`
   (= `gz sim -r worlds/bs_pool.sdf` with the environment set; another world:
   `bash scripts/run_gazebo.sh worlds/<x>.sdf`, extra arguments go to `gz sim`)
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

## Front camera in ROS 2

The BlueROV2 front camera (sensor `front_camera` in the model) publishes on
Gazebo topics, which ROS 2 does not see. `scripts/run_bridge.sh` starts
`ros_gz_bridge` (installed with `ros-jazzy-ros-gz`), which republishes them on
the ROS 2 topics the BlueSpark stack uses for the real front camera, so
`vision_node` can run on the simulated image without changes:

| Gazebo topic                              | ROS 2 topic                 |
|-------------------------------------------|-----------------------------|
| `/bluerov2/front_camera/underwater/image` | `/front/camera/image_raw`   |
| `/bluerov2/front_camera/camera_info`      | `/front/camera/camera_info` |

1. In one terminal (container): `bash scripts/run_gazebo.sh`
2. In another terminal (container): `bash scripts/run_underwater_camera.sh`
3. In another terminal (container): `bash scripts/run_bridge.sh`
4. Check, in a fourth terminal (container):
   `export ROS_DOMAIN_ID=42 && ros2 topic hz /front/camera/image_raw`
   (about 30 Hz).

Notes:
- The bridged image is the processed one published by `underwater_camera_proc`
  (see "Realistic Underwater Camera Simulation" below). Without
  `scripts/run_underwater_camera.sh` running, `/front/camera/image_raw`
  receives nothing.
- The bridge publishes in ROS domain 42 (`ROS_DOMAIN_ID`), the same as the
  BlueSpark stack; programs in different domains do not see each other.
- The topic list is in `workspace/config/ros_gz_bridge.yaml`.
- Gazebo publishes `rgb8` (it cannot produce BGR), while the real camera
  publishes `bgr8`. `vision_node` converts based on the image encoding since
  the `fix/vision-rgb8-encoding` change in the `bluespark` repo; with an older
  version red and blue are swapped.
- ArduSub is not needed for the camera; both can run at the same time.

## Realistic Underwater Camera Simulation

Gazebo Harmonic does not natively simulate underwater light attenuation/turbidity in OGRE 2.
To achieve realistic underwater imagery, the BlueROV2 uses an `rgbd_camera` sensor paired with
a real-time Gazebo Transport post-processing node (`underwater_camera_proc`):

- **Physical Model**: Jaffe-McGlamery / Beer-Lambert optical attenuation per RGB channel:
  $$I_\lambda(d) = I_{0,\lambda} \cdot e^{-\eta_\lambda d} + B_\lambda \cdot (1 - e^{-\eta_\lambda d})$$
  where red attenuates significantly faster than blue/green ($\eta_R \gg \eta_G > \eta_B$).
- **Range Correction**: planar depth $z$ is converted to distance along each pixel's ray $d$.
  Pixels with nothing within the camera's far clip (open water) get exactly the water colour $B$.
- **Optical Degradations**: Forward scattering blur (distance-dependent softening) and sensor noise.
- **Configuration**: Water parameters (attenuation coefficients, backscatter color, blur, noise) are in
  `params/underwater_camera.yaml`. Camera geometry is read from `camera_info`, and the visible range
  is the camera's `<clip>` in `model.sdf.in`; nothing camera-related is duplicated in the YAML.
- **Build**: `run_underwater_camera.sh` rebuilds the node automatically when the source changed
  (or build manually with `bash scripts/build_underwater_camera.sh`).

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
