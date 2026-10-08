#!/bin/bash
set -euo pipefail

source /opt/ros/jazzy/setup.bash
source /home/lxmix/turtlebot3_ws/install/setup.bash

DATASET_ROOT="${1:-/home/lxmix/turtlebot3_ws/datasets/dirt_detection}"
STAMP="$(date +%Y%m%d_%H%M%S)"
OUT_DIR="${DATASET_ROOT}/images_${STAMP}"

mkdir -p "${OUT_DIR}"

echo "Saving images to: ${OUT_DIR}"
echo "Press Ctrl+C to stop capture."

ros2 run image_view image_saver \
  --ros-args \
  -r image:=/camera/image_raw \
  -p filename_format:="${OUT_DIR}/frame_%06d.jpg"
