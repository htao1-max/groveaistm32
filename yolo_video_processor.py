"""
YOLOv8n Video Processor
=======================
Processes an MP4 file using a YOLOv8n model with 2 classes.
Draws bounding boxes and labels on each frame and saves the annotated video.

Usage:
    python yolo_video_processor.py --source video.mp4 --model best.pt --output result.mp4

Requirements:
    pip install ultralytics opencv-python
"""

import argparse
import sys
import time
from pathlib import Path

import cv2
from ultralytics import YOLO


def process_video(
    source: str,
    model_path: str = "best.pt",
    output: str = "output.mp4",
    conf_threshold: float = 0.25,
    iou_threshold: float = 0.45,
    show: bool = False,
):
    # ── Load model ───────────────────────────────────────────────
    print(f"Loading model: {model_path}")
    model = YOLO(model_path)

    class_names = model.names  # e.g. {0: 'classA', 1: 'classB'}
    print(f"Classes: {class_names}")

    # ── Open source video ────────────────────────────────────────
    cap = cv2.VideoCapture(source)
    if not cap.isOpened():
        sys.exit(f"Error: cannot open video '{source}'")

    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
    total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))

    print(f"Source  : {source}")
    print(f"Size    : {width}x{height} @ {fps:.1f} FPS")
    print(f"Frames  : {total_frames}")

    # ── Set up output writer ─────────────────────────────────────
    fourcc = cv2.VideoWriter_fourcc(*"mp4v")
    writer = cv2.VideoWriter(output, fourcc, fps, (width, height))

    # ── Colours per class (BGR) ──────────────────────────────────
    colors = {
        0: (0, 255, 0),   # green  – class 0
        1: (0, 120, 255),  # orange – class 1
    }

    # ── Frame loop ───────────────────────────────────────────────
    frame_idx = 0
    start = time.time()

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        # Run inference
        results = model.predict(
            frame,
            conf=conf_threshold,
            iou=iou_threshold,
            verbose=False,
        )

        # Draw detections
        for result in results:
            for box in result.boxes:
                cls_id = int(box.cls[0])
                conf = float(box.conf[0])
                x1, y1, x2, y2 = map(int, box.xyxy[0])

                color = colors.get(cls_id, (255, 255, 255))
                label = f"{class_names[cls_id]} {conf:.2f}"

                # Bounding box
                cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)

                # Label background
                (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 1)
                cv2.rectangle(frame, (x1, y1 - th - 8), (x1 + tw + 4, y1), color, -1)
                cv2.putText(
                    frame, label, (x1 + 2, y1 - 4),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1, cv2.LINE_AA,
                )

        writer.write(frame)
        frame_idx += 1

        # Progress
        if frame_idx % 100 == 0 or frame_idx == total_frames:
            elapsed = time.time() - start
            proc_fps = frame_idx / elapsed if elapsed > 0 else 0
            print(f"  [{frame_idx}/{total_frames}] {proc_fps:.1f} FPS")

        # Optional live preview
        if show:
            cv2.imshow("YOLOv8 Detection", frame)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                print("Preview closed by user.")
                break

    # ── Cleanup ──────────────────────────────────────────────────
    cap.release()
    writer.release()
    if show:
        cv2.destroyAllWindows()

    elapsed = time.time() - start
    print(f"\nDone – {frame_idx} frames in {elapsed:.1f}s ({frame_idx / elapsed:.1f} FPS)")
    print(f"Saved to: {Path(output).resolve()}")


# ── CLI ──────────────────────────────────────────────────────────
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Process MP4 with YOLOv8n (2-class model)")
    parser.add_argument("--source", required=True, help="Path to input .mp4 file")
    parser.add_argument("--model", default="best.pt", help="Path to YOLOv8n weights (default: best.pt)")
    parser.add_argument("--output", default="output.mp4", help="Path to output .mp4 file (default: output.mp4)")
    parser.add_argument("--conf", type=float, default=0.25, help="Confidence threshold (default: 0.25)")
    parser.add_argument("--iou", type=float, default=0.45, help="IoU threshold for NMS (default: 0.45)")
    parser.add_argument("--show", action="store_true", help="Show live preview window")
    args = parser.parse_args()

    process_video(
        source=args.source,
        model_path=args.model,
        output=args.output,
        conf_threshold=args.conf,
        iou_threshold=args.iou,
        show=args.show,
    )
