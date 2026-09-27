#!/usr/bin/env bash
# Let RViz itself write the Window Geometry block, then splice it back into the
# hand-written config so the comments survive.
set -uo pipefail
NAME="$1"; W="${2:-1920}"; H="${3:-1040}"
SRC="$HOME/ros2_ws/src/smartphone_robot/rviz/${NAME}.rviz"
TMP="/tmp/geom_${NAME}.rviz"

set +u; source /opt/ros/jazzy/setup.bash; source ~/ros2_ws/install/setup.bash; set -u
cp "$SRC" "$TMP"

DISPLAY=:99 LIBGL_ALWAYS_SOFTWARE=1 rviz2 -d "$TMP" >/tmp/bake_${NAME}.log 2>&1 &
RV=$!
sleep 32

export DISPLAY=:99
WIN=$(xdotool search --name "geom_${NAME}.rviz" | tail -1)
xdotool windowsize "$WIN" "$W" "$H"; xdotool windowmove "$WIN" 0 0; sleep 2
xdotool windowactivate --sync "$WIN" >/dev/null 2>&1; sleep 2
xdotool key --clearmodifiers ctrl+s; sleep 5
kill $RV 2>/dev/null; sleep 2

python3 - "$SRC" "$TMP" <<'PY'
import sys, pathlib
src, tmp = pathlib.Path(sys.argv[1]), pathlib.Path(sys.argv[2])
s, t = src.read_text(), tmp.read_text()
if 'QMainWindow State' not in t:
    print(f'  {src.name}: SAQLANMADI'); raise SystemExit(1)
# Window Geometry is the final top-level block in an RViz config.
geom = t[t.index('Window Geometry:'):]
head = s[:s.index('Window Geometry:')]
note = ("# Window Geometry below was written by RViz itself, not by hand. 'QMainWindow State'\n"
        "# is a Qt-serialised blob describing dock sizes and positions; without it RViz falls\n"
        "# back to distributing panels evenly, which is why a config can look completely\n"
        "# different on another screen size. Regenerate with scripts/bake_rviz_geometry.sh.\n")
src.write_text(head + note + geom)
print(f'  {src.name}: geometry ko\'chirildi ({len(geom)} bayt)')
PY
