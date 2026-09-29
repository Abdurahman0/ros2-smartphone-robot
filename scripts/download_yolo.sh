#!/usr/bin/env bash
# Download the YOLOv4-tiny model the object_detector needs.
#
# The weights are ~24 MB, too large to keep in git, so they are downloaded here instead
# and git-ignored. YOLOv4-tiny is the small variant that runs on a CPU; it recognises the
# 80 everyday object classes of the COCO dataset (person, laptop, bottle, chair, ...).
#
# Run once, inside the container:  bash scripts/download_yolo.sh
set -euo pipefail
DIR="${1:-$HOME/ros2_ws/models/yolo}"
mkdir -p "$DIR"; cd "$DIR"
base_cfg="https://raw.githubusercontent.com/AlexeyAB/darknet/master"
rel="https://github.com/AlexeyAB/darknet/releases/download/darknet_yolo_v4_pre"

[ -f yolov4-tiny.cfg ]     || curl -sL -o yolov4-tiny.cfg     "$base_cfg/cfg/yolov4-tiny.cfg"
[ -f coco.names ]          || curl -sL -o coco.names          "$base_cfg/data/coco.names"
[ -f yolov4-tiny.weights ] || curl -sL -o yolov4-tiny.weights "$rel/yolov4-tiny.weights"

echo "Model ready in $DIR:"
ls -lh "$DIR"
