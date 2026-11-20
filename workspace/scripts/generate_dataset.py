#!/usr/bin/env python3
"""
Take photos of the obstacles from random camera poses, with bounding boxes.

Needs worlds/dataset_pool.sdf running and PAUSED (in another terminal):
    gz sim worlds/dataset_pool.sdf
Then:
    python3 scripts/generate_dataset.py --count 500
    python3 scripts/generate_dataset.py --count 500 --underwater   # with water

For every photo the script:
  1. picks a random camera pose in front of / behind the gate (random_pose),
  2. moves the camera there (Gazebo service set_pose),
  3. advances the paused simulation by --steps physics steps, which is exactly
     one camera frame (update_rate 10 Hz = 100 steps of 1 ms),
  4. waits for the boxes and the image of that frame (same time stamp) and
     saves them to dataset/raw:
         images/image_0000000.png
         boxes/boxes_0000000.csv    label,x_center,y_center,width,height (pixels)
     The numbers go on after the files already there, so runs add up.
scripts/convert_to_yolo.py turns them into a YOLO dataset.

The image is the clean image of the camera (models/dataset_camera/model.sdf).
With --underwater it is the same image after the underwater camera
post-processing (src/underwater_camera_proc, issue #4) with the water of
params/underwater_camera.yaml, so it looks like the drone's front camera.
"""

import argparse
import math
import random
import subprocess
import tempfile
import threading
import time
from pathlib import Path

import cv2
import numpy as np
import yaml
from gz.msgs10 import image_pb2
from gz.msgs10.annotated_axis_aligned_2d_box_v_pb2 import AnnotatedAxisAligned2DBox_V
from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.pose_pb2 import Pose
from gz.msgs10.world_control_pb2 import WorldControl
from gz.transport13 import Node

WORLD = "dataset_pool"
CAMERA = "dataset_camera"
WORKSPACE = Path.home() / "workspace"
RAW_DIR = WORKSPACE / "dataset/raw"

# Topics of the camera (models/dataset_camera/model.sdf)
BOXES_TOPIC = "/dataset_camera/boxes"
RGBD_TOPIC = "/dataset_camera/rgbd"  # .../image, .../depth_image, .../camera_info
UNDERWATER_TOPIC = "/dataset_camera/underwater/image"

# Underwater camera post-processing (issue #4)
UNDERWATER_BUILD = WORKSPACE / "scripts/build_underwater_camera.sh"
UNDERWATER_RUN = WORKSPACE / "scripts/run_underwater_camera.sh"
UNDERWATER_CONFIG = WORKSPACE / "params/underwater_camera.yaml"

# Gate in worlds/dataset_pool.sdf: base at (0, 0, -2), 3.05 m wide along x,
# 1.52 m tall, opening facing +/- y. GATE_CENTER is the middle of the frame.
GATE_CENTER = (0.0, 0.0, -2.0 + 0.76)

# Where the camera may be. Water is between the floor (z = -2) and the surface (z = 0).
DISTANCE = (1.0, 8.0)     # from the gate plane, along y [m]
SIDEWAYS = (-3.0, 3.0)    # along the gate, x [m]
DEPTH = (-1.8, -0.2)      # z [m]
AIM_JITTER = (1.5, 0.6)   # look at a point up to this far from the gate centre (x, z) [m]
MAX_ROLL = math.radians(10)
EMPTY_SHARE = 0.1         # share of photos looking away (no obstacle, negative examples)


def random_pose(rng):
    """Return (x, y, z, roll, pitch, yaw) of a camera somewhere near the gate."""
    side = rng.choice((-1, 1))  # in front of or behind the gate
    x = GATE_CENTER[0] + rng.uniform(*SIDEWAYS)
    y = GATE_CENTER[1] + side * rng.uniform(*DISTANCE)
    z = rng.uniform(*DEPTH)

    # Aim at a random point around the gate centre, so the gate is not always
    # in the middle of the image and is sometimes cut off by the image border.
    tx = GATE_CENTER[0] + rng.uniform(-AIM_JITTER[0], AIM_JITTER[0])
    tz = GATE_CENTER[2] + rng.uniform(-AIM_JITTER[1], AIM_JITTER[1])
    dx, dy, dz = tx - x, GATE_CENTER[1] - y, tz - z

    # The camera looks along its own +x axis: yaw turns it left/right,
    # a positive pitch tilts it down (hence the minus sign).
    yaw = math.atan2(dy, dx)
    pitch = -math.atan2(dz, math.hypot(dx, dy))
    if rng.random() < EMPTY_SHARE:
        yaw += rng.uniform(math.radians(90), math.radians(270))  # look away
    roll = rng.uniform(-MAX_ROLL, MAX_ROLL)
    return x, y, z, roll, pitch, yaw


def quaternion(roll, pitch, yaw):
    """Roll/pitch/yaw (as in SDF poses) to a quaternion (x, y, z, w)."""
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy)


def stamp(msg):
    """Sim time of a message [ns]; the frames of one render share it."""
    return msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nsec


class Camera:
    """Moves the dataset camera and collects the boxes and images it publishes."""

    def __init__(self, image_topic):
        self.node = Node()
        self.lock = threading.Lock()
        self.boxes = {}   # time stamp -> [(label, x_min, y_min, x_max, y_max)]
        self.images = {}  # time stamp -> image (BGR, as OpenCV saves it)
        self.newest = -1  # newest time stamp received
        # Subscribing also makes Gazebo render the sensors: it skips sensors
        # that nobody listens to.
        if not (self.node.subscribe(AnnotatedAxisAligned2DBox_V, BOXES_TOPIC, self._on_boxes)
                and self.node.subscribe(image_pb2.Image, image_topic, self._on_image)):
            raise SystemExit("Cannot subscribe to the camera topics.")

    def _on_boxes(self, msg):
        boxes = [(b.label, b.box.min_corner.x, b.box.min_corner.y,
                  b.box.max_corner.x, b.box.max_corner.y) for b in msg.annotated_box]
        with self.lock:
            self.boxes[stamp(msg)] = boxes
            self.newest = max(self.newest, stamp(msg))

    def _on_image(self, msg):
        if msg.pixel_format_type not in (image_pb2.RGB_INT8, image_pb2.BGR_INT8):
            print(f"\nSkipping an image in pixel format {msg.pixel_format_type}")
            return
        image = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step)
        image = image[:, :msg.width * 3].reshape(msg.height, msg.width, 3)
        if msg.pixel_format_type == image_pb2.RGB_INT8:
            image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        with self.lock:
            self.images[stamp(msg)] = image
            self.newest = max(self.newest, stamp(msg))

    def _call(self, service, request, request_type):
        ok, reply = self.node.request(f"/world/{WORLD}/{service}", request,
                                      request_type, Boolean, 3000)
        if not (ok and reply.data):
            raise SystemExit(f"Gazebo service {service} failed. "
                             "Is worlds/dataset_pool.sdf running?")

    def move(self, x, y, z, roll, pitch, yaw):
        pose = Pose(name=CAMERA)
        pose.position.x, pose.position.y, pose.position.z = x, y, z
        o = pose.orientation
        o.x, o.y, o.z, o.w = quaternion(roll, pitch, yaw)
        self._call("set_pose", pose, Pose)

    def take_photo(self, steps, timeout=30.0, retry=3.0):
        """Run one camera frame of simulation; return its (boxes, image).

        Frames received before the step (older poses) are ignored. A frame is
        a boxes message and an image with the same time stamp, so the boxes
        always belong to the image they are saved with.
        """
        with self.lock:
            after = self.newest
        deadline = time.time() + timeout
        while time.time() < deadline:
            self._call("control", WorldControl(pause=True, multi_step=steps), WorldControl)
            # Step again if no frame comes within `retry` s: at the start the
            # subscriptions (or the underwater post-processing) may still be
            # getting ready. The camera pose stays the same.
            step_deadline = min(time.time() + retry, deadline)
            while time.time() < step_deadline:
                with self.lock:
                    done = [s for s in self.boxes if s > after and s in self.images]
                    if done:
                        s = max(done)
                        frame = self.boxes[s], self.images[s]
                        self.boxes = {k: v for k, v in self.boxes.items() if k > s}
                        self.images = {k: v for k, v in self.images.items() if k > s}
                        return frame
                time.sleep(0.02)
        raise SystemExit("No frame from the camera. Is worlds/dataset_pool.sdf running "
                         "and paused?")


def start_underwater():
    """Start the underwater post-processing on the dataset camera.

    Uses the water of params/underwater_camera.yaml (the drone's front camera)
    with the topics switched to the dataset camera.
    """
    missing = [p for p in (UNDERWATER_BUILD, UNDERWATER_RUN, UNDERWATER_CONFIG) if not p.exists()]
    if missing:
        raise SystemExit("--underwater needs the underwater camera post-processing "
                         f"(issue #4), missing: {', '.join(str(p) for p in missing)}")
    config = yaml.safe_load(UNDERWATER_CONFIG.read_text())
    config["topics"].update({
        "input_rgb_topic": f"{RGBD_TOPIC}/image",
        "input_depth_topic": f"{RGBD_TOPIC}/depth_image",
        "input_camera_info_topic": f"{RGBD_TOPIC}/camera_info",
        "output_image_topic": UNDERWATER_TOPIC,
    })
    with tempfile.NamedTemporaryFile("w", prefix="dataset_underwater_", suffix=".yaml",
                                     delete=False) as f:
        yaml.safe_dump(config, f)
    print("Building the underwater camera post-processing...")
    subprocess.run(["bash", str(UNDERWATER_BUILD)], check=True, stdout=subprocess.DEVNULL)
    return subprocess.Popen(["bash", str(UNDERWATER_RUN), f.name], stdout=subprocess.DEVNULL)


def next_number():
    """Number for the next frame: one after the highest already in RAW_DIR."""
    numbers = [int(p.stem.split("_")[1]) for p in (RAW_DIR / "boxes").glob("boxes_*.csv")]
    return max(numbers, default=-1) + 1


def save_frame(number, boxes, image):
    """Save one frame in the layout scripts/convert_to_yolo.py reads."""
    for folder in ("images", "boxes"):
        (RAW_DIR / folder).mkdir(parents=True, exist_ok=True)
    if not cv2.imwrite(str(RAW_DIR / f"images/image_{number:07d}.png"), image):
        raise SystemExit(f"Cannot write images to {RAW_DIR}")
    lines = ["label,x_center,y_center,width,height"]
    for label, x0, y0, x1, y1 in boxes:
        lines.append(f"{label},{(x0 + x1) / 2:.6f},{(y0 + y1) / 2:.6f},"
                     f"{x1 - x0:.6f},{y1 - y0:.6f}")
    # Written after the image: a boxes file always has its image.
    (RAW_DIR / f"boxes/boxes_{number:07d}.csv").write_text("\n".join(lines) + "\n")


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[1])
    parser.add_argument("--count", type=int, default=500, help="number of photos")
    parser.add_argument("--underwater", action="store_true",
                        help="add the water of the drone's front camera (issue #4)")
    parser.add_argument("--steps", type=int, default=100,
                        help="physics steps per photo = one camera frame (default 100)")
    parser.add_argument("--seed", type=int, default=None,
                        help="random seed, to get the same poses again")
    args = parser.parse_args()

    rng = random.Random(args.seed)
    underwater = start_underwater() if args.underwater else None
    try:
        camera = Camera(UNDERWATER_TOPIC if underwater else f"{RGBD_TOPIC}/image")
        first = next_number()
        for i in range(args.count):
            camera.move(*random_pose(rng))
            save_frame(first + i, *camera.take_photo(args.steps))
            print(f"\r{i + 1}/{args.count} photos", end="", flush=True)
    finally:
        if underwater:
            underwater.terminate()
    print(f"\nSaved in {RAW_DIR}. Next: python3 scripts/convert_to_yolo.py")


if __name__ == "__main__":
    main()
