#!/bin/bash
# =============================================================================
# Final Project — bring up the ENTIRE isolated stack for ALL THREE tasks in one shot.
#
# Now a single `docker compose up -d` (four overlays under project `kylefp`) brings up
# every container; Unity is the one host-side step (it's a GUI binary, not a container).
# Every perception node runs at once, so you can run Task 1, Task 2 and Task 3 back-to-back
# in one Unity session without restarting the perception containers — just run reset_map.sh --pin
# (restarts only odometry + SLAM) and the matching run_taskN.sh.
#
# Perception topics (all live simultaneously, no interference):
#   - /yolo/target_info       <- kylefp-yolo       (detection, YOLO_TARGET=bear)   Task 1 + Task 2 bear
#   - /yolo/target_info_knob  <- kylefp-yolo-knob  (detection, YOLO_TARGET=knob,   Task 3 knob
#                                                    outputs remapped off the bear topics)
#   - /yolo/bridge_info       <- kylefp-yolo-seg   (segmentation, bridge geometry) Task 2 bridge bias
#   - /yolo/road_info         <- kylefp-yolo-seg   (segmentation, road centroid)   Task 2 road-follow
#     (Note: Unity has no IMU — Task 2 is road-led, not IMU pitch based.)
#
# Containers (all detached): kylefp compose stack (robot_bringup + slam + navigation +
#   lidar_trans + rosbridge:9091) + kylefp-yolo / -yolo-knob / -yolo-seg + kylefp-tfshim +
#   kylefp-foxglove (ws://localhost:8766). Plus the Unity sim on display :20.
#
# After this, you only:
#   1) In Unity: log in -> FINAL PROJECT -> CAR+ARM Mode = AI ->
#      RosBridge PORT = 9091 -> press Reload (must show "Connected")
#   2) Run any mission (stack stays up between them):
#        ./workspace/pros/pros_car/run_task1.sh
#        ./workspace/pros/pros_car/run_task2.sh
#        ./workspace/pros/pros_car/run_task3.sh
#
# Safe to re-run: `up -d` recreates only what changed.
# =============================================================================
set -e
ROOT="/home/kyle/Desktop/Robot-navigation-projects"
PROS="$ROOT/Final_Project/workspace/pros"

echo "==> 1/2 full container stack (robot + slam + nav + rosbridge:9091 + perception + foxglove)"
cd "$PROS/pros_app/docker/compose"
docker compose -p kylefp \
  -f docker-compose_robot_unity.yml \
  -f docker-compose_slam_unity.yml \
  -f docker-compose_navigation_unity.yml \
  -f docker-compose_perception_unity.yml up -d

echo "==> 2/2 Unity sim on display :20"
if pgrep -f pros_twin_unity_tsai >/dev/null 2>&1; then
  echo "    Unity already running, skipping."
else
  cd "$ROOT/Final_Project/pros_twin_linux/pros_twin_unity_linux"  # symlink -> V5.4 build
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
echo "  $PROS/pros_car/run_task3.sh   # door knob: waypoints -> unlock -> clear"
echo "Foxglove viewer: ws://localhost:8766"
echo "============================================================"
docker ps --filter name=kylefp- --format '  {{.Names}}: {{.Status}}' | sort
