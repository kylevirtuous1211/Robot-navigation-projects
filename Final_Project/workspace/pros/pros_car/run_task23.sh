#!/bin/bash
# Launch the Final Project COMBINED Task 2 + Task 3 mission (headless) on the
# isolated kylefp stack (ROS_DOMAIN_ID=7, network kylefp_my_bridge_network).
#
# Runs the full Task 2 (bridge: mount -> climb -> grip bear -> descend -> return
# + release) then the full Task 3 (door knob: waypoints to the door -> knob pursuit
# -> lever press while driving through) back-to-back in one session
# (entry point: task23_auto / combined_mission.py).
#
# Prereqs:
#   - The full stack is up -> ../../../start_stack.sh  (one launcher for all 3 tasks;
#     bear /yolo/target_info, knob /yolo/target_info_knob, bridge-seg all live)
#   - Unity in FINAL PROJECT scene, CAR + ARM Mode = AI, RosBridge port 9091 (Connected)
#   - Run ONCE first:  ../../../tools/reset_map.sh --pin   (car at spawn -> /amcl_pose ~ (0,0,0);
#     pins the frame that BOTH tasks' absolute waypoints are measured in)
#
# Watch the live log: [combined] markers around each task, plus [Task2]/[Task3]
# state transitions. The car turns around after Task 2 and drives the Task 3
# waypoints to the door. Ctrl-C stops; re-run to retry.
cd "$(dirname "$0")" || exit 1
docker rm -f kylefp-mission-task23 >/dev/null 2>&1
exec docker run -it --rm --name kylefp-mission-task23 \
  --network kylefp_my_bridge_network --gpus all \
  -e ROS_DOMAIN_ID=7 -e PYTHONUNBUFFERED=1 -e NOBUILD="${NOBUILD:-}" \
  -v "$(pwd)/src:/workspaces/src" \
  -v kylefp_colcon_build:/workspaces/build \
  -v kylefp_colcon_install:/workspaces/install \
  ghcr.io/screamlab/pros_car_docker_image:latest \
  bash -lc 'cd /workspaces && { { [ -n "$NOBUILD" ] && [ -f install/setup.bash ]; } || colcon build --symlink-install; } && source install/setup.bash && ros2 run pros_car_py task23_auto'
