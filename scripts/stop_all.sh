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

# NOTE: match on EXECUTABLE PATHS, not on bare words.
#
# This list used to contain patterns like "amcl" and "nav2", and `pkill -f` matches the
# whole command line -- so running an experiment script named experiment_amcl_beams.sh
# made this script kill the experiment that had just called it. It failed instantly and
# silently, which took a while to work out.
#
# Every ROS node lives under a predictable lib path, so matching those is both precise
# and complete.
for pattern in \
    "gz sim" \
    "/opt/ros/jazzy/lib/ros_gz_bridge/" \
    "/opt/ros/jazzy/lib/ros_gz_image/" \
    "/opt/ros/jazzy/lib/ros_gz_sim/" \
    "/opt/ros/jazzy/lib/nav2_" \
    "/opt/ros/jazzy/lib/slam_toolbox/" \
    "/opt/ros/jazzy/lib/robot_state_publisher/" \
    "/opt/ros/jazzy/lib/joint_state_publisher" \
    "/opt/ros/jazzy/lib/rviz2/" \
    "/opt/ros/jazzy/lib/opennav_docking/" \
    "/ros2_ws/install/smartphone_robot/lib/" \
    "/opt/ros/jazzy/bin/ros2 launch" \
    "Xvfb" ; do
  pkill -9 -f "$pattern" 2>/dev/null
done
sleep 2

# Stale FastDDS shared-memory segments left by killed processes cause
# "RTPS_TRANSPORT_SHM Error ... open_and_lock_file failed" on the next run.
echo "Clearing stale DDS shared memory..."
find /dev/shm -maxdepth 1 -name 'fastrtps*' -delete 2>/dev/null

# Verify rather than assume. A single surviving gz sim or ros_gz_bridge is enough to
# corrupt the next run, so keep going until the check is actually clean.
for attempt in 1 2 3; do
  LEFT=$(pgrep -fc 'gz sim|/opt/ros/jazzy/lib/ros_gz|/opt/ros/jazzy/bin/ros2 launch' 2>/dev/null | head -1)
  LEFT=${LEFT:-0}
  [ "$LEFT" -eq 0 ] && break
  echo "  attempt ${attempt}: ${LEFT} still alive, killing again..."
  pkill -9 -f 'gz sim' 2>/dev/null
  pkill -9 -f '/opt/ros/jazzy/lib/ros_gz' 2>/dev/null
  pkill -9 -f '/opt/ros/jazzy/bin/ros2 launch' 2>/dev/null
  sleep 3
done

CLOCKS=$(pgrep -fc 'gz sim' 2>/dev/null | head -1)
CLOCKS=${CLOCKS:-0}
if [ "$CLOCKS" -ne 0 ]; then
  echo "WARNING: ${CLOCKS} gz sim process(es) still running. Two simulations means two"
  echo "         /clock publishers and simulated time that jumps backwards."
  ps -eo pid,etimes,args --no-headers | grep '[g]z sim'
  exit 1
fi

echo "Done. No simulation processes remain."
ps -eo pid,pcpu,comm --no-headers | grep -vE '(bash|ps|grep|sleep|find)$' || true
