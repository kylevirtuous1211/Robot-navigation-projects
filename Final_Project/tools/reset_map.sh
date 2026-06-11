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

# Post-scene-reset container restarts. Two modes:
#   --pin  : reproducible-localization ritual (Task 2). Re-origin the laser scan_matcher
#            odom to the spawn, THEN re-anchor SLAM, THEN resync Nav2 — in that order.
#            Needed because slam_toolbox anchors `map` to odom, and scan_matcher odom
#            accumulates forever (never resets on its own), so the map frame floats every
#            session. Verified 2026-06-11: after this, /amcl_pose at spawn reads ~(0,0,0).
#            (slam_toolbox's map_start_pose does NOT pin a fresh map — odom reset is the fix.)
#   --slam : older flag — rebuild SLAM + restart Nav2 only. Does NOT reset odom, so map
#            coords still float. Use --pin when you need reproducible coordinates.
case "${1:-}" in
  --pin)
    echo "[reset] PIN localization (car must be at spawn from the scene reset above):"
    echo "[reset]   1/3 restart robot_bringup (scan_matcher odom -> 0 at spawn) ..."
    docker restart kylefp-robot_bringup-1 >/dev/null 2>&1
    sleep 8
    echo "[reset]   2/3 restart slam (map re-anchors to odom=0) ..."
    docker restart kylefp-slam-1 >/dev/null 2>&1
    sleep 8
    echo "[reset]   3/3 restart navigation (Nav2 costmap resync) ..."
    docker restart kylefp-navigation-1 >/dev/null 2>&1
    sleep 8
    echo "[reset] PIN done — verify /amcl_pose at spawn reads ~(0,0,0):"
    echo "[reset]   docker exec kylefp-tfshim bash -lc 'source /opt/ros/humble/setup.bash && ROS_DOMAIN_ID=7 ros2 topic echo /amcl_pose --once'"
    ;;
  --slam)
    echo "[reset] restarting SLAM + navigation (Nav2) for a clean costmap ..."
    docker restart kylefp-slam-1 >/dev/null 2>&1
    sleep 8
    docker restart kylefp-navigation-1 >/dev/null 2>&1
    sleep 8
    echo "[reset] SLAM + navigation restarted."
    echo "[reset] NOTE: --slam does NOT reset scan_matcher odom, so map coords still float."
    echo "[reset]       For reproducible (0,0,0)-pinned coords (Task 2 dock) use: reset_map.sh --pin"
    ;;
  *)
    echo "[reset] (grab-only) SLAM left as-is."
    echo "[reset]   RETURN/Nav2 tests: reset_map.sh --slam | pinned localization: reset_map.sh --pin"
    ;;
esac
