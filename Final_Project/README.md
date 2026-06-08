# Final Project — Task 1 Autonomous Mission

Autonomous **Task 1** for the Unity rover challenge: find a bear, approach and hold
(Locate & Observe, 10 pts), grip it, and drive back to the start (Recovery, 20 pts) —
fully autonomous, no manual Foxglove clicking or keyboard driving.

Strategy: **reactive visual servoing** (YOLO `/yolo/target_info` + depth) for search →
approach → grip, then **online SLAM + Nav2** to return to the recorded start pose. The map
is randomized each run, so the costmap is built live as the rover explores — no map pre-pass.

This builds on the HW4 stack (copied into `workspace/pros/`), reusing the trained
`detection.pt` (bear/knob) and the existing wheel/arm/Nav2 primitives.

## Quick start (recommended)

Two scripts, no multiple terminals — everything comes up detached.

**Only if the stack is down** (after a reboot, or you removed the containers) — run once:

```bash
~/Desktop/Robot-navigation-projects/Final_Project/start_stack.sh
```

One consolidated launcher brings up **everything for all three tasks at once** in the background:
the compose stack (robot + online SLAM + Nav2 + rosbridge on **9091**), **all three YOLO containers**
(bear → `/yolo/target_info`, knob → `/yolo/target_info_knob`, bridge → `/yolo/bridge_info`), the
`tf → /amcl_pose` shim, Foxglove (`ws://localhost:8766`), and the Unity sim on the Chrome Remote
Desktop display.

**Then in Unity** (on your Chrome Remote Desktop): log in → **FINAL PROJECT** →
**CAR & ARM Mode = AI** → **RosBridge PORT = 9091** → press **Reload** (must show "Connected").

**Every mission run** (the stack stays up between tasks — run them in any order, no restarts):

```bash
~/Desktop/Robot-navigation-projects/Final_Project/workspace/pros/pros_car/run_task1.sh   # bear
~/Desktop/Robot-navigation-projects/Final_Project/workspace/pros/pros_car/run_task2.sh   # bridge + bear
~/Desktop/Robot-navigation-projects/Final_Project/workspace/pros/pros_car/run_task3.sh   # door knob
```

That's it. Ctrl-C stops the mission; re-run to try again (new random map each time).

> **Note:** If your stack is already up, skip `start_stack.sh` entirely and just run the `run_taskN.sh`
> you want.

The section below documents the equivalent manual steps (raw `docker run` commands) if you
ever need to bring pieces up individually or debug the stack.

## Run — isolated stack (verified, shared GPU box)

When another user already runs a stack on the default `ROS_DOMAIN_ID=1` / port 9090, bring up
your own isolated stack instead: project `kylefp`, `ROS_DOMAIN_ID=7`, rosbridge on host port
**9091**, foxglove on **8766** (`.env` files already set to domain 7; slam compose maps 9091).

```bash
# 1. Stack: robot + online SLAM + Nav2 (project kylefp, own network)
cd ~/Desktop/Robot-navigation-projects/Final_Project/workspace/pros/pros_app/docker/compose
docker compose -p kylefp \
  -f docker-compose_robot_unity.yml \
  -f docker-compose_slam_unity.yml \
  -f docker-compose_navigation_unity.yml up -d

# 2. YOLO perception (domain 7, mounts your edited src)
cd ~/Desktop/Robot-navigation-projects/Final_Project/workspace/pros/ros2_yolo_integration
docker run -d --name kylefp-yolo --network kylefp_my_bridge_network --gpus all \
  -e ROS_DOMAIN_ID=7 -e YOLO_DEVICE=cuda -v "$(pwd)/src:/workspaces/src" \
  pros_cameraapi:cu128 \
  bash -lc "cd /workspaces && colcon build && source install/setup.bash && ros2 run yolo_example_pkg yolo_node"

# 3. TF -> /amcl_pose shim (domain 7)
cd ~/Desktop/Robot-navigation-projects/Final_Project/workspace/pros/pros_car
docker run -d --name kylefp-tfshim --network kylefp_my_bridge_network --gpus all \
  -e ROS_DOMAIN_ID=7 -v "$(pwd)/src:/workspaces/src" \
  ghcr.io/screamlab/pros_car_docker_image:latest \
  bash -lc "cd /workspaces && colcon build && source install/setup.bash && ros2 run pros_car_py tf_to_amcl_pose"

# 4. Foxglove bridge on domain 7 (view at ws://localhost:8766)
docker run -d --name kylefp-foxglove --network kylefp_my_bridge_network \
  -e ROS_DOMAIN_ID=7 -p 8766:8765 \
  us-central1-docker.pkg.dev/foxglove-images/images/foxglove_bridge:ros-humble-v3.2.6

# 5. Unity sim on the Chrome Remote Desktop display, then in-sim set
#    CAR + ARM Mode = AI and RosBridge PORT = 9091, press Reload (must show "Connected").
cd ~/Desktop/Robot-navigation-projects/HW4/pros_twin_linux/pros_twin_unity_linux
DISPLAY=:20 XAUTHORITY=/home/kyle/.Xauthority \
  __NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia \
  ./pros_twin_unity_tsai_run_linux.x86_64 -force-vulkan &
```

### Launch the Task 1 mission

```bash
~/Desktop/Robot-navigation-projects/Final_Project/workspace/pros/pros_car/run_task1.sh
```

Runs `task1_auto` headless (builds + `ros2 run pros_car_py task1_auto`) in your terminal so you
see the live `[Task1]` state log. **Ctrl-C** stops it; re-run to try again. Equivalent raw command:

```bash
cd ~/Desktop/Robot-navigation-projects/Final_Project/workspace/pros/pros_car
docker run -it --rm --name kylefp-mission \
  --network kylefp_my_bridge_network --gpus all \
  -e ROS_DOMAIN_ID=7 -e PYTHONUNBUFFERED=1 -v "$(pwd)/src:/workspaces/src" \
  ghcr.io/screamlab/pros_car_docker_image:latest \
  bash -lc "cd /workspaces && colcon build && source install/setup.bash && ros2 run pros_car_py task1_auto"
```

## Verify (Foxglove `ws://localhost:8766` for the isolated stack; `9090` for the default one)
- `/map` grows as the rover moves (SLAM); Nav2 active; `/amcl_pose` published by the shim.
- `/yolo/detection/compressed` boxes the bear; `/yolo/target_marker` sits on the bear in 3D.
- Mission reaches the bear, holds 5 s (Locate & Observe), grips, and Nav2-returns (Recovery).
- Re-open the **FINAL PROJECT** tab 2–3× (new random map) to confirm robustness.

## Known integration check
`final_project_unity.sh` runs `navigation_unity.xml`, which passes a `map` arg to
nav2_bringup's `navigation_launch.py`. If that launch rejects the arg, remove the
`<arg name="map" .../>` line in `pros_app/docker/compose/demo/navigation_unity.xml` — the
Nav2 costmap only needs the `/map` topic that slam_toolbox publishes, not a map server.

## Tasks 2 & 3 (implemented)

Both run hands-free like Task 1: bring up the task's stack, then run its mission script and watch the
live `[TaskN]` state log (every transition + a throttled per-tick sensor readout for fast tuning).

All three tasks share one stack (`start_stack.sh`); just run the matching `run_taskN.sh`.

- **Task 2 (bridge):** `run_task2.sh` (`task2_auto`). A bear is **guaranteed at the top-middle** of the
  bridge, so it's deterministic — no visual bear search. The segmentation node publishes
  `/yolo/bridge_info` (bridge `delta_x`, area, bottom-edge, `symmetry`) and `/yolo/road_info`. Task 2
  **homes on the bridge** by its left/right position (bridge-on-left → steer left; rotates in place when
  far off so it never drives in crooked), **squares up** at the ramp foot using `symmetry` (≈0 = facing
  the ramp head-on), then **lowers the open bulldozer claw and drives straight up the centerline** — the
  claw scoops the top-middle bear as it arrives (distance gauged via `/amcl_pose`, **no IMU** since Unity
  has none) — **closes the gripper** (with a small anti-rollback press), and pose-**RETURNs to start**.
  Claw-down the whole climb means the bear can't roll out.
- **Task 3 (door):** `run_task3.sh` (`task3_auto`). A dedicated detection container runs with
  `YOLO_TARGET=knob`, its output remapped to `/yolo/target_info_knob` so it coexists with the bear
  node — the trained `knob` class arrives through the exact same format Task 1 uses for the bear.
  Locate & Observe reuses APPROACH/OBSERVE (≥5 s, 10 pts); **Unlock** = `arm_controller.knob_poke()`
  (reach + press, tunable pose); **Clear** = drive the body forward (`CLEAR_PUSH_SEC`) to push the
  door fully open.

Per-mission tuning knobs live at the top of each `taskN_mission.py.__init__` (alignment tolerances,
Task 2's `BRIDGE_ALIGN_PX`/`BRIDGE_SYM_TOL`/`BRIDGE_CLOSE_AREA`/`CLIMB_DISTANCE`/`GRIP_PRESS_SPEED`). They need in-sim
calibration — the verbose logs make that fast. `YOLO_TARGET` defaults to `bear`, so Task 1 is unaffected.
