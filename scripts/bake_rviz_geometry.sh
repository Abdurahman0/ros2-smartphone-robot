#!/usr/bin/env bash
# Let RViz write its own Window Geometry (the QMainWindow State blob) for a config, so
# docked panels -- especially an Image display -- get a saved position and actually show.
#
# RViz stores panel sizes and dock positions in a Qt-serialised blob that a hand-written
# config does not contain. Without it, a newly added Image display is created but docked
# nowhere visible. This runs RViz on a virtual display with a window manager (needed so the
# window can be focused and sent Ctrl+S), maximises it, saves, and splices the resulting
# geometry block back into the hand-written config, keeping the comments.
#
# Usage (inside container):  bash scripts/bake_rviz_geometry.sh <name> [W] [H]
set -uo pipefail
NAME="${1:?usage: bake_rviz_geometry.sh <name> [W] [H]}"; W="${2:-1920}"; H="${3:-1040}"
SRC="$HOME/ros2_ws/src/smartphone_robot/rviz/${NAME}.rviz"
TMP="/tmp/geom_${NAME}.rviz"; DISP=":99"

set +u; source /opt/ros/jazzy/setup.bash; source ~/ros2_ws/install/setup.bash; set -u
cp "$SRC" "$TMP"

# Fresh virtual display + window manager, killing any stale ones first.
pkill -9 -f "Xvfb $DISP" 2>/dev/null || true
pkill -9 -f openbox 2>/dev/null || true
sleep 1
Xvfb "$DISP" -screen 0 "${W}x${H}x24" -nolisten tcp >/tmp/xvfb_bake.log 2>&1 &
sleep 3
DISPLAY="$DISP" openbox >/tmp/openbox_bake.log 2>&1 &
sleep 2

DISPLAY="$DISP" LIBGL_ALWAYS_SOFTWARE=1 rviz2 -d "$TMP" >/tmp/bake_${NAME}.log 2>&1 &
RV=$!
sleep 30

export DISPLAY="$DISP"
WIN=$(xdotool search --name "geom_${NAME}" | tail -1)
if [ -z "${WIN:-}" ]; then echo "RViz window not found"; kill $RV 2>/dev/null; exit 1; fi
xdotool windowsize "$WIN" "$W" "$H"; xdotool windowmove "$WIN" 0 0; sleep 2
xdotool windowactivate --sync "$WIN"; sleep 2
xdotool key --clearmodifiers ctrl+s; sleep 5
kill $RV 2>/dev/null; sleep 2

python3 - "$SRC" "$TMP" <<'PY'
import sys, pathlib
src, tmp = pathlib.Path(sys.argv[1]), pathlib.Path(sys.argv[2])
s, t = src.read_text(), tmp.read_text()
if 'QMainWindow State' not in t:
    print(f'  {src.name}: geometry NOT saved'); raise SystemExit(1)
geom = t[t.index('Window Geometry:'):]
head = s[:s.index('Window Geometry:')]
src.write_text(head + geom)
print(f'  {src.name}: geometry baked ({len(geom)} bytes)')
PY
pkill -9 -f "Xvfb $DISP" 2>/dev/null || true
pkill -9 -f openbox 2>/dev/null || true
