#!/usr/bin/env python3
"""
Turn the frames saved by scripts/generate_dataset.py into a YOLO dataset.

Input, written by scripts/generate_dataset.py:
    dataset/raw/images/image_0000000.png
    dataset/raw/boxes/boxes_0000000.csv    label,x_center,y_center,width,height (pixels)
Output:
    dataset/yolo/images/{train,val}/image_0000000.png
    dataset/yolo/labels/{train,val}/image_0000000.txt   class x_center y_center width height (0-1)
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


def read_boxes(boxes_csv):
    """Boxes of one frame: [(label, x_center, y_center, width, height)], in pixels."""
    with open(boxes_csv) as f:
        return [(int(b["label"]), float(b["x_center"]), float(b["y_center"]),
                 float(b["width"]), float(b["height"])) for b in csv.DictReader(f)]


def yolo_line(box, width, height):
    """YOLO label line of one box: pixels -> fractions of the image size."""
    label, x, y, w, h = box
    return f"{label} {x / width:.6f} {y / height:.6f} {w / width:.6f} {h / height:.6f}"


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[1])
    parser.add_argument("--val", type=float, default=0.2, help="share of photos for validation")
    parser.add_argument("--min-px", type=float, default=4,
                        help="skip images with a box smaller than this in both width and height")
    parser.add_argument("--seed", type=int, default=0, help="seed of the train/val split")
    args = parser.parse_args()

    raw, out = DATASET_DIR / "raw", DATASET_DIR / "yolo"
    images = sorted((raw / "images").glob("image_*.png"))
    if not images:
        raise SystemExit(f"No images in {raw / 'images'}. Run generate_dataset.py first.")

    frames, no_boxes = [], 0
    for image in images:
        boxes_csv = raw / "boxes" / image.name.replace("image_", "boxes_").replace(".png", ".csv")
        if not boxes_csv.exists():
            no_boxes += 1  # image saved without its boxes (generate_dataset.py stopped in between)
            continue
        frames.append((image, read_boxes(boxes_csv)))

    # A label without a class name cannot go into the labels, but the object is
    # in the images: YOLO would learn it as background. Stop before writing anything.
    unknown = sorted({box[0] for _, boxes in frames for box in boxes
                      if not 0 <= box[0] < len(CLASS_NAMES)})
    if unknown:
        raise SystemExit(f"Labels {unknown} have no class name. Add them to CLASS_NAMES in "
                         "scripts/convert_to_yolo.py (index = label in worlds/dataset_pool.sdf).")

    if out.exists():
        shutil.rmtree(out)  # always rebuild from raw, so no stale files stay behind

    for kind in ("images", "labels"):
        for split in ("train", "val"):
            (out / kind / split).mkdir(parents=True)

    random.Random(args.seed).shuffle(frames)
    n_val = round(len(frames) * args.val)
    counts = {"train": 0, "val": 0, "boxes": 0, "small": 0}

    for i, (image, boxes) in enumerate(frames):
        # A box too small to learn from (an object far away or almost all
        # hidden) is not just left out: the object is still in the image, and
        # without its label YOLO would learn it as background. So the whole
        # image is skipped. Only boxes small in both directions count: a thin
        # box, like a gate post at the image border, is a clearly visible object.
        if any(w < args.min_px and h < args.min_px for _, _, _, w, h in boxes):
            counts["small"] += 1
            continue
        split = "val" if i < n_val else "train"
        size = png_size(image)
        lines = [yolo_line(box, *size) for box in boxes]

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
    if counts["small"]:
        print(f"Skipped {counts['small']} images with a box under {args.min_px:g} px "
              "(--min-px).")
    if no_boxes:
        print(f"Skipped {no_boxes} images without a boxes file.")


if __name__ == "__main__":
    main()
