#!/usr/bin/env python3
"""
Take photos of the obstacles from random camera poses, with bounding boxes.

Needs worlds/dataset_pool.sdf running and PAUSED (in another terminal):
    gz sim worlds/dataset_pool.sdf
Then:
    python3 scripts/generate_dataset.py --count 500

For every photo the script:
  1. picks a random camera pose in front of / behind the gate (random_pose),
  2. moves the camera there (Gazebo service set_pose),
  3. advances the paused simulation by --steps physics steps, which is exactly
     one camera frame (update_rate 10 Hz = 100 steps of 1 ms),
  4. waits until Gazebo has saved that frame.
Gazebo itself writes the image and its boxes to dataset/raw (see the <save> in
models/dataset_camera/model.sdf). scripts/convert_to_yolo.py turns them into a
YOLO dataset.

Gazebo skips rendering a sensor that has no subscribers, even with <save> on,
so the script keeps one subscribed to the camera topic while it runs.
"""

import argparse
import math
import random
import subprocess
import time
from pathlib import Path

WORLD = "dataset_pool"
CAMERA = "dataset_camera"
RAW_DIR = Path.home() / "workspace/dataset/raw"  # <save><path> of the camera
BOXES_TOPIC = f"/{CAMERA}/boxes"                 # <topic> of the camera

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


def gz_service(name, reqtype, request):
    """Call a Gazebo service with the gz command line tool."""
    subprocess.run(
        ["gz", "service", "-s", f"/world/{WORLD}/{name}",
         "--reqtype", reqtype, "--reptype", "gz.msgs.Boolean",
         "--timeout", "3000", "--req", request],
        check=True, capture_output=True,
    )


def subscribe_to_camera():
    """Start a background subscriber to the camera topic, so Gazebo renders it.

    Without any subscriber Gazebo does not render the sensor at all and no
    frame is ever saved. The messages themselves are not needed (DEVNULL).
    """
    return subprocess.Popen(["gz", "topic", "-e", "-t", BOXES_TOPIC],
                            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def move_camera(x, y, z, roll, pitch, yaw):
    qx, qy, qz, qw = quaternion(roll, pitch, yaw)
    gz_service("set_pose", "gz.msgs.Pose",
               f'name: "{CAMERA}" position {{x: {x} y: {y} z: {z}}} '
               f"orientation {{x: {qx} y: {qy} z: {qz} w: {qw}}}")


def saved_frames():
    """Number of frames Gazebo has saved (the box file is written after the image)."""
    return len(list((RAW_DIR / "boxes").glob("*.csv")))


def take_photo(steps, timeout=15.0, retry=3.0, settle=0.3):
    """Run one camera frame of simulation and wait for Gazebo to save it."""
    before = saved_frames()
    deadline = time.time() + timeout
    while saved_frames() == before:
        if time.time() > deadline:
            raise TimeoutError("No frame saved. Is dataset_pool.sdf running and paused?")
        # Step again if no frame comes within `retry` s: at the start the camera
        # subscriber may still be connecting. The camera pose stays the same.
        gz_service("control", "gz.msgs.WorldControl", f"pause: true multi_step: {steps}")
        step_deadline = time.time() + retry
        while saved_frames() == before and time.time() < step_deadline:
            time.sleep(0.05)
    # Rendering runs in its own thread, so wait until no more files appear
    # before the camera moves again (the very first step can render 2 frames).
    count = saved_frames()
    while True:
        time.sleep(settle)
        if saved_frames() == count:
            return
        count = saved_frames()


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[1])
    parser.add_argument("--count", type=int, default=500, help="number of photos")
    parser.add_argument("--steps", type=int, default=100,
                        help="physics steps per photo = one camera frame (default 100)")
    parser.add_argument("--seed", type=int, default=None,
                        help="random seed, to get the same poses again")
    args = parser.parse_args()

    # Gazebo numbers the saved frames from 0 after every start, so frames left
    # from an earlier run could be overwritten (and would not be counted).
    if saved_frames():
        raise SystemExit(f"{RAW_DIR} already has {saved_frames()} frames from an earlier "
                         "run. Move or delete that folder first.")

    rng = random.Random(args.seed)
    subscriber = subscribe_to_camera()
    try:
        for i in range(1, args.count + 1):
            move_camera(*random_pose(rng))
            take_photo(args.steps)
            print(f"\r{i}/{args.count} photos", end="", flush=True)
    finally:
        subscriber.terminate()
    print(f"\nSaved in {RAW_DIR}. Next: python3 scripts/convert_to_yolo.py")


if __name__ == "__main__":
    main()
