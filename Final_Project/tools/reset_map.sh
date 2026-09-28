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

# Run a ros2 CLI command inside an always-up container on the kylefp net (ROS_DOMAIN_ID=7).
# NOTE: the `--no-daemon` on the ros2 calls below is REQUIRED — the long-lived containers
# carry a wedged ros2 daemon (fails with xmlrpc `!rclpy.ok()`), so daemon-routed calls
# (topic list/echo) error out instantly. `--no-daemon` does its own DDS discovery (~2s) and
# works reliably. Pick a topic with a CONTINUOUS publisher: e.g. /amcl_pose (tfshim) — NOT
# /odom, which scan_matcher only publishes on movement so `echo` can't even find its type.
ros2_in() { docker exec kylefp-tfshim bash -lc "source /opt/ros/humble/setup.bash && ROS_DOMAIN_ID=7 $*"; }

echo "[reset] switching to RACING2026 ..."
$CLICK 110 358
sleep 3
echo "[reset] switching back to FINAL PROJECT ..."
$CLICK 110 315
sleep 5
echo "[reset] done — car should be at spawn on a fresh FINAL PROJECT map."

# Post-scene-reset container restarts. Two modes:
#   --pin  : reproducible-localization ritual (Task 2/3, combined task23). Re-origin the
#            laser scan_matcher odom to the spawn, THEN re-anchor SLAM — in that order.
#            Needed because slam_toolbox anchors `map` to odom, and scan_matcher odom
#            accumulates forever (never resets on its own), so the map frame floats every
#            session. Verified 2026-06-11: after this, /amcl_pose at spawn reads ~(0,0,0).
#            (slam_toolbox's map_start_pose does NOT pin a fresh map — odom reset is the fix.)
#            Nav2 is NOT restarted: every mission (Task 1/2/3 + combined) drives purely on
#            /amcl_pose + vision (publish_raw_car_control), never the Nav2 stack, so the Nav2
#            costmap resync is dead weight here. Task 1's RETURN is a go-to-point on /amcl_pose.
#   --slam : older flag — rebuild SLAM + restart Nav2 only. Does NOT reset odom, so map
#            coords still float. Use --pin when you need reproducible coordinates.
case "${1:-}" in
  --pin)
    echo "[reset] PIN localization (car must be at spawn from the scene reset above):"
    echo "[reset]   1/2 restart robot_bringup (scan_matcher odom -> 0 at spawn) ..."
    docker restart -t 3 kylefp-robot_bringup-1 >/dev/null 2>&1
    sleep 6   # let scan_matcher come back with odom re-origined to 0 before slam reads it
    echo "[reset]   2/2 restart slam (map re-anchors to odom=0) ..."
    docker restart -t 3 kylefp-slam-1 >/dev/null 2>&1
    echo "[reset] Nav2 NOT restarted - no mission uses it (pose + vision only)."
    # One call both WAITS and VERIFIES: blocks until the full TF chain (slam map->odom +
    # scan_matcher odom->base, republished by tfshim) produces a fresh /amcl_pose — i.e. slam
    # has re-anchored to odom=0 — then prints the spawn pose. Returns as soon as ready (~2-5s),
    # capped at 20s. Replaces the old blind `sleep 8` AND the manual verify command.
    echo "[reset] waiting for /amcl_pose to republish — spawn should read ~(0,0,0):"
    ros2_in "timeout 20 ros2 topic echo --no-daemon /amcl_pose --once --field pose.pose.position" 2>/dev/null \
      || echo "[reset]   ! /amcl_pose not seen within 20s — verify manually"
    echo "[reset] PIN done."
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
