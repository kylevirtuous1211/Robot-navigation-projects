#!/bin/bash
# =============================================================================
# Final Project — bring up the ENTIRE isolated stack for ALL THREE tasks in one shot.
#
# This is the single, consolidated launcher (the merged-demo pipeline): every
# perception node runs at once, so you can run Task 1, Task 2 and Task 3 back-to-back
# in one Unity session WITHOUT restarting any container — just run the matching
# run_taskN.sh each time.
#
# Perception topics (all live simultaneously, no interference):
#   - /yolo/target_info       <- kylefp-yolo       (detection, YOLO_TARGET=bear)   Task 1 + Task 2 bear
#   - /yolo/target_info_knob  <- kylefp-yolo-knob  (detection, YOLO_TARGET=knob,   Task 3 knob
#                                                    outputs remapped off the bear topics)
#   - /yolo/bridge_info       <- kylefp-yolo-seg   (segmentation, bridge geometry) Task 2 bridge bias
#   - /yolo/road_info         <- kylefp-yolo-seg   (segmentation, road centroid)   Task 2 road-follow
#     (Note: Unity has no IMU — Task 2 is road-led, not IMU pitch based.)
#
# Brings up (all detached / background — no extra terminals needed):
#   - kylefp compose stack : robot_bringup + slam + navigation + lidar_trans + rosbridge(9091)
#   - kylefp-yolo          : YOLO bear detection            -> /yolo/target_info
#   - kylefp-yolo-knob     : YOLO knob detection (remapped) -> /yolo/target_info_knob
#   - kylefp-yolo-seg      : YOLO bridge segmentation       -> /yolo/bridge_info
#   - kylefp-tfshim        : TF -> /amcl_pose shim
#   - kylefp-foxglove      : Foxglove bridge (ws://localhost:8766)
#   - Unity sim            : on Chrome Remote Desktop display :20
#
# After this, you only:
#   1) In Unity: log in -> FINAL PROJECT -> CAR+ARM Mode = AI ->
#      RosBridge PORT = 9091 -> press Reload (must show "Connected")
#   2) Run any mission (stack stays up between them):
#        ./workspace/pros/pros_car/run_task1.sh
#        ./workspace/pros/pros_car/run_task2.sh
#        ./workspace/pros/pros_car/run_task3.sh
#
# Safe to re-run: it removes/recreates the helper containers each time.
# =============================================================================
set -e
ROOT="/home/kyle/Desktop/Robot-navigation-projects"
PROS="$ROOT/Final_Project/workspace/pros"
NET="kylefp_my_bridge_network"

echo "==> 1/7 compose stack (robot + slam + nav + rosbridge:9091)"
cd "$PROS/pros_app/docker/compose"
docker compose -p kylefp \
  -f docker-compose_robot_unity.yml \
  -f docker-compose_slam_unity.yml \
  -f docker-compose_navigation_unity.yml up -d

echo "==> 2/7 YOLO detection — bear -> /yolo/target_info (domain 7)"
cd "$PROS/ros2_yolo_integration"
docker rm -f kylefp-yolo >/dev/null 2>&1 || true
docker run -d --name kylefp-yolo --network "$NET" --gpus all \
  -e ROS_DOMAIN_ID=7 -e YOLO_DEVICE=cuda -e YOLO_TARGET=bear -v "$(pwd)/src:/workspaces/src" \
  pros_cameraapi:cu128 \
  bash -lc "cd /workspaces && colcon build && source install/setup.bash && ros2 run yolo_example_pkg yolo_node"

echo "==> 3/7 YOLO detection — knob -> /yolo/target_info_knob (remapped, domain 7)"
docker rm -f kylefp-yolo-knob >/dev/null 2>&1 || true
# Same detection.pt, YOLO_TARGET=knob; remap ALL outputs off the bear topics so the two
# detection nodes never clash. Task 3 subscribes to /yolo/target_info_knob.
docker run -d --name kylefp-yolo-knob --network "$NET" --gpus all \
  -e ROS_DOMAIN_ID=7 -e YOLO_DEVICE=cuda -e YOLO_TARGET=knob -v "$(pwd)/src:/workspaces/src" \
  pros_cameraapi:cu128 \
  bash -lc "cd /workspaces && colcon build && source install/setup.bash && \
    ros2 run yolo_example_pkg yolo_node --ros-args \
      -r /yolo/target_info:=/yolo/target_info_knob \
      -r /yolo/target_marker:=/yolo/target_marker_knob \
      -r /yolo/detection/compressed:=/yolo/detection_knob/compressed \
      -r /camera/x_multi_depth_values:=/camera/x_multi_depth_values_knob"

echo "==> 4/7 YOLO segmentation — bridge -> /yolo/bridge_info (domain 7)"
docker rm -f kylefp-yolo-seg >/dev/null 2>&1 || true
docker run -d --name kylefp-yolo-seg --network "$NET" --gpus all \
  -e ROS_DOMAIN_ID=7 -e YOLO_DEVICE=cuda -v "$(pwd)/src:/workspaces/src" \
  pros_cameraapi:cu128 \
  bash -lc "cd /workspaces && colcon build && source install/setup.bash && ros2 run yolo_example_pkg yolo_seg_node"

echo "==> 5/7 TF -> /amcl_pose shim (domain 7)"
cd "$PROS/pros_car"
docker rm -f kylefp-tfshim >/dev/null 2>&1 || true
docker run -d --name kylefp-tfshim --network "$NET" --gpus all \
  -e ROS_DOMAIN_ID=7 -v "$(pwd)/src:/workspaces/src" \
  ghcr.io/screamlab/pros_car_docker_image:latest \
  bash -lc "cd /workspaces && colcon build && source install/setup.bash && ros2 run pros_car_py tf_to_amcl_pose"

echo "==> 6/7 Foxglove bridge (ws://localhost:8766)"
docker rm -f kylefp-foxglove >/dev/null 2>&1 || true
docker run -d --name kylefp-foxglove --network "$NET" \
  -e ROS_DOMAIN_ID=7 -p 8766:8765 \
  us-central1-docker.pkg.dev/foxglove-images/images/foxglove_bridge:ros-humble-v3.2.6

echo "==> 7/7 Unity sim on display :20"
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
echo "Full stack up (all 3 tasks). Now in Unity (Chrome Remote Desktop):"
echo "  login -> FINAL PROJECT -> CAR & ARM Mode = AI"
echo "  RosBridge PORT = 9091 -> press Reload (must say Connected)"
echo ""
echo "Verify perception in Foxglove (ws://localhost:8766) / ros2 topic hz:"
echo "  /yolo/target_info        (bear)   | /yolo/target_info_knob   (knob)"
echo "  /yolo/bridge_info        (bridge) | /yolo/road_info           (road-follow)"
echo ""
echo "Then run any mission (stack stays up between tasks):"
echo "  $PROS/pros_car/run_task1.sh   # bear: search -> grab -> return"
echo "  $PROS/pros_car/run_task2.sh   # bridge -> grab bear -> return"
echo "  $PROS/pros_car/run_task3.sh   # door knob: observe -> unlock -> clear"
echo "Foxglove viewer: ws://localhost:8766"
echo "============================================================"
docker ps --filter name=kylefp- --format '  {{.Names}}: {{.Status}}' | sort
