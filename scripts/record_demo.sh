#!/usr/bin/env bash
# Record a demo video of RViz, rendered on a virtual display.
#
# Same trick as capture_rviz.sh: RViz draws onto Xvfb, so the recording is a fixed
# resolution, needs no physical screen, and can be reproduced by anyone with the repo.
#
# Usage:  scripts/record_demo.sh <rviz_config> <output.mp4> [seconds]
# Run INSIDE the container, with a simulation already running.

set -euo pipefail

CONFIG="${1:?usage: record_demo.sh <rviz_config> <output.mp4> [seconds]}"
OUTPUT="${2:?usage: record_demo.sh <rviz_config> <output.mp4> [seconds]}"
DURATION="${3:-60}"
DISPLAY_NUM=":99"
RES="1600x1000"

# ROS's setup.bash reads variables it has not defined, so it trips `set -u`.
# Disable nounset just for the sourcing, then restore it.
set +u
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
set -u

cleanup() { kill "${RVIZ_PID:-}" "${XVFB_PID:-}" 2>/dev/null || true; }
trap cleanup EXIT

if ! DISPLAY="$DISPLAY_NUM" xdpyinfo >/dev/null 2>&1; then
  Xvfb "$DISPLAY_NUM" -screen 0 "${RES}x24" -nolisten tcp >/tmp/xvfb.log 2>&1 &
  XVFB_PID=$!
  sleep 3
fi

DISPLAY="$DISPLAY_NUM" LIBGL_ALWAYS_SOFTWARE=1 \
  rviz2 -d "$CONFIG" --ros-args -p use_sim_time:=true >/tmp/rviz_record.log 2>&1 &
RVIZ_PID=$!
sleep 25   # let RViz start and the displays connect

echo "Recording ${DURATION}s to ${OUTPUT}..."
# 10 fps is plenty for a robotics demo and keeps the file small enough to attach.
ffmpeg -loglevel error -y \
  -f x11grab -framerate 10 -video_size "$RES" -i "${DISPLAY_NUM}.0" \
  -t "$DURATION" -c:v libx264 -preset veryfast -crf 28 -pix_fmt yuv420p \
  "$OUTPUT"
echo "Done: $(du -h "$OUTPUT" | cut -f1)"
