#!/usr/bin/env python3
"""
Turn the frames saved by Gazebo (scripts/generate_dataset.py) into a YOLO dataset.

Input, written by the dataset camera (models/dataset_camera/model.sdf):
    dataset/raw/images/image_0000001.png
    dataset/raw/boxes/boxes_0000001.csv    label,x_center,y_center,width,height (pixels)
Output:
    dataset/yolo/images/{train,val}/image_0000001.png
    dataset/yolo/labels/{train,val}/image_0000001.txt   class x_center y_center width height (0-1)
    dataset/yolo/data.yaml                               paths and class names for training

Usage, in the container:  python3 scripts/convert_to_yolo.py
"""

import argparse
import csv
import random
import shutil
import struct
from pathlib import Path

# Class names, indexed by the Gazebo label (Label system in worlds/dataset_pool.sdf).
CLASS_NAMES = ["gate"]

DATASET_DIR = Path.home() / "workspace/dataset"


def png_size(path):
    """(width, height) of a PNG, read from its header (bytes 16-24)."""
    with open(path, "rb") as f:
        return struct.unpack(">II", f.read(24)[16:24])


def yolo_lines(boxes_csv, width, height, min_px):
    """YOLO label lines for one frame. Boxes smaller than min_px are dropped."""
    lines = []
    with open(boxes_csv) as f:
        for box in csv.DictReader(f):
            label = int(box["label"])
            w, h = float(box["width"]), float(box["height"])
            if label >= len(CLASS_NAMES) or w < min_px or h < min_px:
                continue
            # Pixels -> fractions of the image size, as YOLO expects.
            lines.append(f"{label} {float(box['x_center']) / width:.6f} "
                         f"{float(box['y_center']) / height:.6f} "
                         f"{w / width:.6f} {h / height:.6f}")
    return lines


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[1])
    parser.add_argument("--val", type=float, default=0.2, help="share of photos for validation")
    parser.add_argument("--min-px", type=float, default=4, help="drop boxes smaller than this")
    parser.add_argument("--seed", type=int, default=0, help="seed of the train/val split")
    args = parser.parse_args()

    raw, out = DATASET_DIR / "raw", DATASET_DIR / "yolo"
    images = sorted((raw / "images").glob("image_*.png"))
    if not images:
        raise SystemExit(f"No images in {raw / 'images'}. Run generate_dataset.py first.")
    if out.exists():
        shutil.rmtree(out)  # always rebuild from raw, so no stale files stay behind

    for kind in ("images", "labels"):
        for split in ("train", "val"):
            (out / kind / split).mkdir(parents=True)

    random.Random(args.seed).shuffle(images)
    n_val = round(len(images) * args.val)
    counts = {"train": 0, "val": 0, "boxes": 0}

    for i, image in enumerate(images):
        split = "val" if i < n_val else "train"
        boxes_csv = raw / "boxes" / image.name.replace("image_", "boxes_").replace(".png", ".csv")
        if not boxes_csv.exists():
            continue  # image saved without its boxes (Gazebo stopped in between)
        lines = yolo_lines(boxes_csv, *png_size(image), args.min_px)

        shutil.copy2(image, out / "images" / split / image.name)
        # An image without objects gets an empty label file: a negative example.
        label_file = out / "labels" / split / image.with_suffix(".txt").name
        label_file.write_text("".join(line + "\n" for line in lines))
        counts[split] += 1
        counts["boxes"] += len(lines)

    (out / "data.yaml").write_text(
        f"path: {out}\ntrain: images/train\nval: images/val\n"
        f"names:\n" + "".join(f"  {i}: {n}\n" for i, n in enumerate(CLASS_NAMES)))
    print(f"{counts['train']} train + {counts['val']} val images, "
          f"{counts['boxes']} boxes -> {out}")


if __name__ == "__main__":
    main()
