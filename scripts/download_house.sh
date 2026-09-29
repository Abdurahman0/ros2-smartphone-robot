#!/usr/bin/env bash
# Download the 3D house model that worlds/house.world uses.
#
# worlds/house.world references "OpenRobotics/House 1" from Gazebo Fuel by URI. Gazebo can
# fetch it on first load, but doing it explicitly here makes the world start reliably and
# offline afterwards. The model (~2.5 MB of meshes and textures) is cached in ~/.gz/fuel,
# outside the repo, so it is not committed.
#
# Run once, inside the container:  bash scripts/download_house.sh
set -euo pipefail
source /opt/ros/jazzy/setup.bash
echo "Downloading OpenRobotics/House 1 from Gazebo Fuel..."
gz fuel download -u "https://fuel.gazebosim.org/1.0/OpenRobotics/models/House 1"
echo "Done. Cached in ~/.gz/fuel"
