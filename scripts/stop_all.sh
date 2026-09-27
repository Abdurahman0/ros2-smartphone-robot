#!/usr/bin/env bash
# Stop every simulation / ROS process started by this project, cleanly.
#
# Why this exists: Ctrl-C on `ros2 launch` does not always reap every child, and the
# survivors are not harmless. Two ros_gz_bridge processes both republish /clock and
# /scan with slightly different timing, which makes TF lookups fail with
# "Transform data too old" and the Nav2 controller abort -- a failure that looks like
# a tuning bug and is not one. Always run this between simulation runs.
#
# Run INSIDE the container.

set -uo pipefail

echo "Stopping launch files..."
pkill -INT -f "ros2 launch" 2>/dev/null
sleep 5

# NOTE: pkill -x matches /proc/<pid>/comm, which the kernel truncates to 15
# characters -- "parameter_bridge" is stored as "parameter_bridg" and -x never
# matches it. Use -f, which matches the full command line.
echo "Force-stopping stragglers..."
for pattern in "ros2 launch" "gz sim" "parameter_bridge" "image_bridge" \
               "robot_state_publisher" "rviz2" "slam_toolbox" "wander_node" \
               "nav2" "map_server" "amcl" "_server" "lifecycle_manager" \
               "collision_monitor" "velocity_smoother" "waypoint_follower" \
               "opennav_docking" "Xvfb"; do
  pkill -9 -f "$pattern" 2>/dev/null
done
sleep 2

# Stale FastDDS shared-memory segments left by killed processes cause
# "RTPS_TRANSPORT_SHM Error ... open_and_lock_file failed" on the next run.
echo "Clearing stale DDS shared memory..."
find /dev/shm -maxdepth 1 -name 'fastrtps*' -delete 2>/dev/null

REMAINING=$(ps -eo comm --no-headers | grep -cvE '^(bash|ps|grep|sh|sleep|find)$' || true)
echo "Done. Remaining non-shell processes: ${REMAINING}"
ps -eo pid,pcpu,comm --no-headers | grep -vE '(bash|ps|grep|sleep|find)$' || true
