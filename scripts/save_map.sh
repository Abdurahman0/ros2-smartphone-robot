#!/usr/bin/env bash
# Save the current SLAM map to src/smartphone_robot/maps/<name>.{pgm,yaml}.
#
# RViz has no "export map" button -- a map is saved with nav2_map_server's map_saver, which
# subscribes to /map and writes an image (.pgm) plus its metadata (.yaml). This wraps that
# in one short command. Run it (inside the container) WHILE a mapping launch is still
# running, so /map is being published.
#
#   bash scripts/save_map.sh            # saves as maps/indoor.{pgm,yaml}
#   bash scripts/save_map.sh house      # saves as maps/house.{pgm,yaml}
set -euo pipefail
NAME="${1:-indoor}"
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash

# Save into the SOURCE tree (symlink-installed), so the map is version-controlled and the
# launch files find it, not into the throwaway install/ copy.
DIR=~/ros2_ws/src/smartphone_robot/maps
mkdir -p "$DIR"; cd "$DIR"

echo "Saving /map to $DIR/${NAME}.{pgm,yaml} ..."
ros2 run nav2_map_server map_saver_cli -f "$NAME" --ros-args -p use_sim_time:=true
echo
echo "Saved:"
ls -lh "$DIR/${NAME}.pgm" "$DIR/${NAME}.yaml"
