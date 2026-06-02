#!/bin/bash
# Hands-free FINAL PROJECT map reset via synthetic clicks on display :20 (no sudo).
# Clicking FINAL PROJECT while already in it does NOT reload, so we switch to
# RACING2026 first, then back to FINAL PROJECT → fresh scene, car at spawn,
# rosbridge connection preserved.
#
# Button pixel coords (1854x850 screen, MAP tab open):
#   FINAL PROJECT (110,315)   RACING2026 (110,358)
# If the panel layout changes, re-measure with a screenshot.
export DISPLAY=:20
export XAUTHORITY=/home/kyle/.Xauthority
CLICK="python3 $(dirname "$0")/click20.py"

echo "[reset] switching to RACING2026 ..."
$CLICK 110 358
sleep 3
echo "[reset] switching back to FINAL PROJECT ..."
$CLICK 110 315
sleep 5
echo "[reset] done — car should be at spawn on a fresh FINAL PROJECT map."

# For RETURN (Nav2) tests the SLAM map must be rebuilt for the new layout,
# AND navigation must restart afterwards (SLAM restart de-syncs Nav2's costmaps).
if [ "${1:-}" = "--slam" ]; then
  echo "[reset] restarting SLAM + navigation (Nav2) for a clean costmap ..."
  docker restart kylefp-slam-1 >/dev/null 2>&1
  sleep 8
  docker restart kylefp-navigation-1 >/dev/null 2>&1
  sleep 8
  echo "[reset] SLAM + navigation restarted."
else
  echo "[reset] (grab-only) SLAM left as-is. For RETURN tests run: reset_map.sh --slam"
fi
