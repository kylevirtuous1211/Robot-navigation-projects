#!/bin/bash
# =============================================================================
# Final Project — bring up the ENTIRE isolated Task 1 stack in one shot.
#
# Brings up (all detached / background — no extra terminals needed):
#   - kylefp compose stack : robot_bringup + slam + navigation + lidar_trans + rosbridge(9091)
#   - kylefp-yolo          : YOLO bear detection (domain 7)
#   - kylefp-tfshim        : TF -> /amcl_pose shim (domain 7)
#   - kylefp-foxglove      : Foxglove bridge (ws://localhost:8766)
#   - Unity sim            : on Chrome Remote Desktop display :20
#
# After this, you only:
#   1) In Unity: log in -> FINAL PROJECT -> CAR+ARM Mode = AI ->
#      RosBridge PORT = 9091 -> press Reload (must show "Connected")
#   2) Run the mission:  ./workspace/pros/pros_car/run_task1.sh
#
# Safe to re-run: it removes/recreates the helper containers each time.
# =============================================================================
set -e
ROOT="/home/kyle/Desktop/Robot-navigation-projects"
PROS="$ROOT/Final_Project/workspace/pros"
NET="kylefp_my_bridge_network"

echo "==> 1/5 compose stack (robot + slam + nav + rosbridge:9091)"
cd "$PROS/pros_app/docker/compose"
docker compose -p kylefp \
  -f docker-compose_robot_unity.yml \
  -f docker-compose_slam_unity.yml \
  -f docker-compose_navigation_unity.yml up -d

echo "==> 2/5 YOLO perception (domain 7)"
cd "$PROS/ros2_yolo_integration"
docker rm -f kylefp-yolo >/dev/null 2>&1 || true
docker run -d --name kylefp-yolo --network "$NET" --gpus all \
  -e ROS_DOMAIN_ID=7 -e YOLO_DEVICE=cuda -v "$(pwd)/src:/workspaces/src" \
  pros_cameraapi:cu128 \
  bash -lc "cd /workspaces && colcon build && source install/setup.bash && ros2 run yolo_example_pkg yolo_node"

echo "==> 3/5 TF -> /amcl_pose shim (domain 7)"
cd "$PROS/pros_car"
docker rm -f kylefp-tfshim >/dev/null 2>&1 || true
docker run -d --name kylefp-tfshim --network "$NET" --gpus all \
  -e ROS_DOMAIN_ID=7 -v "$(pwd)/src:/workspaces/src" \
  ghcr.io/screamlab/pros_car_docker_image:latest \
  bash -lc "cd /workspaces && colcon build && source install/setup.bash && ros2 run pros_car_py tf_to_amcl_pose"

echo "==> 4/5 Foxglove bridge (ws://localhost:8766)"
docker rm -f kylefp-foxglove >/dev/null 2>&1 || true
docker run -d --name kylefp-foxglove --network "$NET" \
  -e ROS_DOMAIN_ID=7 -p 8766:8765 \
  us-central1-docker.pkg.dev/foxglove-images/images/foxglove_bridge:ros-humble-v3.2.6

echo "==> 5/5 Unity sim on display :20"
if pgrep -f pros_twin_unity_tsai >/dev/null 2>&1; then
  echo "    Unity already running, skipping."
else
  cd "$ROOT/Final_Project/pros_twin_linux/pros_twin_unity_linux"  # symlink -> V5 build
  DISPLAY=:20 XAUTHORITY=/home/kyle/.Xauthority \
    __NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia \
    setsid ./pros_twin_unity_tsai_run_linux.x86_64 -force-vulkan >/tmp/kylefp_unity.log 2>&1 &
  echo "    Unity launched (log: /tmp/kylefp_unity.log)"
fi

echo ""
echo "============================================================"
echo "Stack up. Now in Unity (Chrome Remote Desktop):"
echo "  login -> FINAL PROJECT -> CAR & ARM Mode = AI"
echo "  RosBridge PORT = 9091 -> press Reload (must say Connected)"
echo ""
echo "Then run the mission:"
echo "  $PROS/pros_car/run_task1.sh"
echo "Foxglove viewer: ws://localhost:8766"
echo "============================================================"
docker ps --filter name=kylefp- --format '  {{.Names}}: {{.Status}}' | sort
