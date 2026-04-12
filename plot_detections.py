#!/usr/bin/env python3
"""
Plot YOLO bounding boxes on detected images from the Grove AI SD card.

Usage:
    python plot_detections.py path/to/SESSION_XXXX/DETECT/
    python plot_detections.py path/to/SESSION_XXXX/DETECT/ --save output_dir/

Each det_XXXX.jpg should have a matching det_XXXX.txt with lines:
    <class_idx> <confidence> <x> <y> <width> <height>
"""

import argparse
import os
import glob

from PIL import Image
import matplotlib.pyplot as plt
import matplotlib.patches as patches

# Distinct colors for up to 20 classes
COLORS = [
    "#FF3838", "#FF9D97", "#FF701F", "#FFB21D", "#CFD231",
    "#48F90A", "#92CC17", "#3DDB86", "#1A9334", "#00D4BB",
    "#2C99A8", "#00C2FF", "#344593", "#6473FF", "#0018EC",
    "#8438FF", "#520085", "#CB38FF", "#FF95C8", "#FF37C7",
]


def parse_annotation(txt_path):
    """Parse a detection annotation file. Returns list of dicts."""
    detections = []
    with open(txt_path, "r") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            parts = line.split()
            if len(parts) < 6:
                continue
            detections.append({
                "class_idx": int(parts[0]),
                "confidence": float(parts[1]),
                "x": int(parts[2]),
                "y": int(parts[3]),
                "w": int(parts[4]),
                "h": int(parts[5]),
            })
    return detections


def plot_image(img_path, detections, save_path=None):
    """Draw bounding boxes on image and display or save."""
    img = Image.open(img_path)
    fig, ax = plt.subplots(1, figsize=(10, 8))
    ax.imshow(img)

    for det in detections:
        color = COLORS[det["class_idx"] % len(COLORS)]
        rect = patches.Rectangle(
            (det["x"], det["y"]), det["w"], det["h"],
            linewidth=2, edgecolor=color, facecolor="none"
        )
        ax.add_patch(rect)
        label = f"cls:{det['class_idx']} {det['confidence']:.2f}"
        ax.text(
            det["x"], det["y"] - 4, label,
            color="white", fontsize=9, fontweight="bold",
            bbox=dict(facecolor=color, alpha=0.7, edgecolor="none", pad=1),
        )

    ax.set_title(os.path.basename(img_path))
    ax.axis("off")
    plt.tight_layout()

    if save_path:
        fig.savefig(save_path, dpi=150, bbox_inches="tight")
        plt.close(fig)
    else:
        plt.show()


def main():
    parser = argparse.ArgumentParser(description="Plot YOLO detections on images")
    parser.add_argument("detect_dir", help="Path to SESSION_XXXX/DETECT/ folder")
    parser.add_argument("--save", default=None, help="Output directory for annotated images")
    args = parser.parse_args()

    if args.save and not os.path.exists(args.save):
        os.makedirs(args.save)

    jpg_files = sorted(glob.glob(os.path.join(args.detect_dir, "det_*.jpg")))
    if not jpg_files:
        print(f"No det_*.jpg files found in {args.detect_dir}")
        return

    for jpg_path in jpg_files:
        base = os.path.splitext(jpg_path)[0]
        txt_path = base + ".txt"

        if not os.path.exists(txt_path):
            print(f"Skipping {os.path.basename(jpg_path)} (no matching .txt)")
            continue

        detections = parse_annotation(txt_path)
        print(f"{os.path.basename(jpg_path)}: {len(detections)} detection(s)")

        save_path = None
        if args.save:
            save_path = os.path.join(args.save, os.path.basename(jpg_path))

        plot_image(jpg_path, detections, save_path)

    if args.save:
        print(f"Annotated images saved to {args.save}")


if __name__ == "__main__":
    main()
