#!/bin/bash
# Launch the Final Project Task 2 autonomous mission (headless) on the
# isolated kylefp stack (ROS_DOMAIN_ID=7, network kylefp_my_bridge_network).
#
# Task 2 = road-led onto the bridge (follow /yolo/road_info, bias toward the bridge via
# /yolo/bridge_info delta_x), committed climb (distance via /amcl_pose; Unity has no IMU),
# then reuse Task 1's bear pipeline (APPROACH->OBSERVE->CREEP->GRIP->RETURN to start).
#
# Prereqs (already running):
#   - The full stack is up -> ../../../start_stack.sh  (one launcher for all 3 tasks;
#     provides /yolo/target_info (bear), /yolo/bridge_info + /yolo/road_info, /amcl_pose)
#   - Unity in FINAL PROJECT scene, CAR + ARM Mode = AI, RosBridge port 9091 (Connected)
#
# Watch the live "[Task2]" state log (transitions + per-tick road/bridge/bear readout)
# to tune the knobs at the top of task2_mission.py. Ctrl-C stops; re-run to retry.
#
# Sanity before running (inside any container on the net):
#   ros2 topic hz /yolo/road_info     # road-follow centroid
#   ros2 topic hz /yolo/bridge_info   # bridge steering bias + close-area gate
#   ros2 topic hz /yolo/target_info   # bear detection
cd "$(dirname "$0")" || exit 1
docker rm -f kylefp-mission-task2 >/dev/null 2>&1
exec docker run -it --rm --name kylefp-mission-task2 \
  --network kylefp_my_bridge_network --gpus all \
  -e ROS_DOMAIN_ID=7 -e PYTHONUNBUFFERED=1 \
  -v "$(pwd)/src:/workspaces/src" \
  ghcr.io/screamlab/pros_car_docker_image:latest \
  bash -lc "cd /workspaces && colcon build && source install/setup.bash && ros2 run pros_car_py task2_auto"
