#!/bin/bash
# Launch the Final Project Task 1 autonomous mission (headless) on the
# isolated kylefp stack (ROS_DOMAIN_ID=7, network kylefp_my_bridge_network).
#
# Prereqs (already running):
#   - kylefp stack up (final_project_unity: robot+slam+nav), kylefp-yolo, kylefp-tfshim
#   - Unity in FINAL PROJECT scene, CAR + ARM Mode = AI, RosBridge port 9091 (Connected)
#
# Ctrl-C stops the mission. Re-run this script to attempt again.
cd "$(dirname "$0")" || exit 1
docker rm -f kylefp-mission >/dev/null 2>&1
exec docker run -it --rm --name kylefp-mission \
  --network kylefp_my_bridge_network --gpus all \
  -e ROS_DOMAIN_ID=7 -e PYTHONUNBUFFERED=1 \
  -v "$(pwd)/src:/workspaces/src" \
  ghcr.io/screamlab/pros_car_docker_image:latest \
  bash -lc "cd /workspaces && colcon build && source install/setup.bash && ros2 run pros_car_py task1_auto"
