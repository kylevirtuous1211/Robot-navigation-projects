#!/bin/bash
# Launch the Final Project Task 2 autonomous mission (headless) on the
# isolated kylefp stack (ROS_DOMAIN_ID=7, network kylefp_my_bridge_network).
#
# Task 2 = BRIDGE_APPROACH pose dock to the ramp foot (needs reset_map.sh --pin) -> SNAP_90 (rotate to
# the bridge axis so the off-bridge decoy bear leaves the frame, then creep forward to dock onto the
# ramp) -> VISUAL_CLIMB (lower the open claw, align-then-go up the bridge on the bear's delta_x until
# close/scooped; stuck-guard reverses if no depth progress) -> GRIP (press + scoop_grab) ->
# pose-RETURN to start. Bear picker is YOLO_TARGET_PICK=onbridge: the detection node subscribes to
# /yolo/bridge_info and only reports the bear horizontally aligned with the bridge (ignores off-bridge
# decoys). NOTE: changing that env requires recreating the kylefp-yolo container (see below), not just
# re-running this script.
#
# Prereqs (already running):
#   - The full stack is up -> ../../../start_stack.sh  (one launcher for all 3 tasks;
#     provides /yolo/target_info (bear), /yolo/bridge_info + /yolo/road_info, /amcl_pose)
#   - reset_map.sh --pin has been run at spawn (BRIDGE_APPROACH pose dock needs the pinned map frame)
#   - Unity in FINAL PROJECT scene, CAR + ARM Mode = AI, RosBridge port 9091 (Connected)
#
# Watch the live "[Task2]" state log (transitions + per-tick bear/pose readout) to tune the knobs
# at the top of task2_mission.py (CROSS_MIN_SEC, DOCK_NEAR_DIST, ALIGN_PX, ...). Ctrl-C stops.
#
# Sanity before running (inside any container on the net):
#   ros2 topic hz /yolo/target_info   # bear detection (drives VISUAL_DOCK / CROSS / APPROACH_FAR)
#   ros2 topic echo /amcl_pose --once # confirm ~ (0,0,0) at spawn after reset_map.sh --pin
cd "$(dirname "$0")" || exit 1
docker rm -f kylefp-mission-task2 >/dev/null 2>&1
exec docker run -it --rm --name kylefp-mission-task2 \
  --network kylefp_my_bridge_network --gpus all \
  -e ROS_DOMAIN_ID=7 -e PYTHONUNBUFFERED=1 \
  -v "$(pwd)/src:/workspaces/src" \
  ghcr.io/screamlab/pros_car_docker_image:latest \
  bash -lc "cd /workspaces && colcon build && source install/setup.bash && ros2 run pros_car_py task2_auto"
