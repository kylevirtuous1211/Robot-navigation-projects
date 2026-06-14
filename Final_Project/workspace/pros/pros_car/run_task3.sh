#!/bin/bash
# Launch the Final Project Task 3 autonomous mission (headless) on the
# isolated kylefp stack (ROS_DOMAIN_ID=7, network kylefp_my_bridge_network).
#
# Task 3 = door knob: SEARCH -> APPROACH -> OBSERVE (>=5s, Locate & Observe) ->
# UNLOCK (arm knob_poke) -> CLEAR (drive body forward to push the door open).
#
# Prereqs (already running):
#   - The full stack is up -> ../../../start_stack.sh  (one launcher for all 3 tasks;
#     the knob detection container remaps its output to /yolo/target_info_knob)
#   - Unity in FINAL PROJECT scene, CAR + ARM Mode = AI, RosBridge port 9091 (Connected)
#
# Watch the live "[Task3]" state log (transitions + per-tick knob found/dist/dx) to
# tune the knobs at the top of task3_mission.py. Ctrl-C stops; re-run to retry.
#
# Sanity before running (inside any container on the net):
#   ros2 topic hz /yolo/target_info_knob   # should box the KNOB
cd "$(dirname "$0")" || exit 1
docker rm -f kylefp-mission-task3 >/dev/null 2>&1
exec docker run -it --rm --name kylefp-mission-task3 \
  --network kylefp_my_bridge_network --gpus all \
  -e ROS_DOMAIN_ID=7 -e PYTHONUNBUFFERED=1 -e NOBUILD="${NOBUILD:-}" \
  -v "$(pwd)/src:/workspaces/src" \
  -v kylefp_colcon_build:/workspaces/build \
  -v kylefp_colcon_install:/workspaces/install \
  ghcr.io/screamlab/pros_car_docker_image:latest \
  bash -lc 'cd /workspaces && { { [ -n "$NOBUILD" ] && [ -f install/setup.bash ]; } || colcon build --symlink-install; } && source install/setup.bash && ros2 run pros_car_py task3_auto'
