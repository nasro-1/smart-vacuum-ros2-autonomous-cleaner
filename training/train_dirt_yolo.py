#!/usr/bin/env python3
"""Train a YOLOv8 model for dirt/trash detection.

Usage:
    python train_dirt_yolo.py --data datasets/dirt_detection/data.yaml
    python train_dirt_yolo.py --data datasets/dirt_detection/data.yaml --epochs 100 --batch 32

The trained weights will be saved to:
    <project>/<name>/weights/best.pt
"""

import argparse
import os
import sys


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Train YOLOv8 for dirt/trash detection."
    )
    parser.add_argument(
        "--data",
        default=os.path.join("datasets", "dirt_detection", "data.yaml"),
        help="Path to YOLO data.yaml (default: datasets/dirt_detection/data.yaml)",
    )
    parser.add_argument(
        "--model",
        default="yolov8n.pt",
        help="Pretrained base model (default: yolov8n.pt)",
    )
    parser.add_argument("--epochs", type=int, default=60,  help="Training epochs")
    parser.add_argument("--imgsz",  type=int, default=640, help="Input image size")
    parser.add_argument("--batch",  type=int, default=16,  help="Batch size")
    parser.add_argument(
        "--project",
        default=os.path.join("models"),
        help="Output directory for training runs (default: models/)",
    )
    parser.add_argument(
        "--name",
        default="dirt_yolo",
        help="Run name inside --project (default: dirt_yolo)",
    )
    return parser.parse_args()


def main():
    args = parse_args()

    # Validate dataset path early
    if not os.path.exists(args.data):
        print(
            f"[ERROR] Dataset config not found: {args.data}\n"
            "Create it in YOLO format (train/val images + labels).\n"
            "See datasets/dirt_detection/data.yaml.example for reference.",
            file=sys.stderr,
        )
        sys.exit(1)

    try:
        from ultralytics import YOLO
    except ImportError:
        print(
            "[ERROR] ultralytics is not installed.\n"
            "Run: pip install ultralytics",
            file=sys.stderr,
        )
        sys.exit(1)

    model = YOLO(args.model)
    model.train(
        data=args.data,
        epochs=args.epochs,
        imgsz=args.imgsz,
        batch=args.batch,
        project=args.project,
        name=args.name,
    )

    best_weights = os.path.join(args.project, args.name, "weights", "best.pt")
    print("\nTraining finished.")
    print(f"Best weights saved to: {best_weights}")
    print(
        f"\nTo use in the detector node:\n"
        f"  ros2 run dirt_cleaner dirt_detector_node --ros-args -p weights:={best_weights}"
    )


if __name__ == "__main__":
    main()