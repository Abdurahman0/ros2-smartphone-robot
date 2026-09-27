#!/usr/bin/env bash
# Experiment: how many laser beams does AMCL need to localise well?
#
# AMCL subsamples the 360 returns the LiDAR provides down to `max_beams` before matching
# them against the map. Fewer beams is cheaper but carries less information -- and
# heading is what suffers first, because distinguishing two similar orientations needs
# returns spread around the robot.
#
# For each setting this runs the SAME four navigation goals and records AMCL's estimate
# against Gazebo's ground truth, so the numbers are comparable. Everything else -- map,
# world, spawn pose, goals, controller -- is held fixed.
#
# Usage (inside the container, nothing else running):
#     bash scripts/experiment_beam_count.sh 60 180 360
#
# Results land in results/ as one CSV per setting, plus a summary printed at the end.

set -uo pipefail
set +u; source /opt/ros/jazzy/setup.bash; source ~/ros2_ws/install/setup.bash; set -u

WS=~/ros2_ws
SHARE=$(ros2 pkg prefix smartphone_robot)/share/smartphone_robot
RESULTS="$WS/results"
mkdir -p "$RESULTS"

# The same goals every time. Chosen to cross the room diagonally, pass the L-shaped
# partition, and end near a wall, so localisation is exercised rather than flattered.
GOALS_X=(5.2 0.5 5.4 0.4)
GOALS_Y=(5.2 5.5 0.6 0.4)

for BEAMS in "$@"; do
  echo
  echo "############### max_beams = ${BEAMS} ###############"
  bash "$WS/scripts/stop_all.sh" >/dev/null 2>&1
  sleep 4

  # A copy of the parameters with only this one value changed.
  PARAMS="/tmp/nav2_beams_${BEAMS}.yaml"
  python3 - "$SHARE/config/nav2_params.yaml" "$PARAMS" "$BEAMS" <<'PY'
import sys, re
src, dst, beams = sys.argv[1], sys.argv[2], sys.argv[3]
text = open(src).read()
text = re.sub(r'^(\s*)max_beams:\s*\d+', rf'\g<1>max_beams: {beams}', text, count=1, flags=re.M)
open(dst, 'w').write(text)
PY

  ros2 launch smartphone_robot navigation.launch.py \
      rviz:=false params_file:="$PARAMS" >/tmp/exp_${BEAMS}.log 2>&1 &
  sleep 55

  ros2 run smartphone_robot localization_eval --ros-args \
      -p use_sim_time:=true \
      -p output_csv:="$RESULTS/amcl_${BEAMS}beams.csv" >/tmp/eval_${BEAMS}.log 2>&1 &
  EVAL=$!
  sleep 8

  for i in "${!GOALS_X[@]}"; do
    printf '  goal (%s, %s) ... ' "${GOALS_X[$i]}" "${GOALS_Y[$i]}"
    timeout 90 ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
      "{pose: {header: {frame_id: map}, pose: {position: {x: ${GOALS_X[$i]}, y: ${GOALS_Y[$i]}}, orientation: {w: 1.0}}}}" \
      2>/dev/null | grep -oE 'SUCCEEDED|ABORTED|CANCELED' | tail -1
  done

  kill -INT "$EVAL" 2>/dev/null
  sleep 5
  grep -A10 'LOCALISATION ERROR' /tmp/eval_${BEAMS}.log || echo '  (no summary)'
done

bash "$WS/scripts/stop_all.sh" >/dev/null 2>&1
echo
echo "CSV files in $RESULTS"
