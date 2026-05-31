# Final Project — Task 1 Autonomous Mission

Autonomous **Task 1** for the Unity rover challenge: find a bear, approach and hold
(Locate & Observe, 10 pts), grip it, and drive back to the start (Recovery, 20 pts) —
fully autonomous, no manual Foxglove clicking or keyboard driving.

Strategy: **reactive visual servoing** (YOLO `/yolo/target_info` + depth) for search →
approach → grip, then **online SLAM + Nav2** to return to the recorded start pose. The map
is randomized each run, so the costmap is built live as the rover explores — no map pre-pass.

This builds on the HW4 stack (copied into `workspace/pros/`), reusing the trained
`detection.pt` (bear/knob) and the existing wheel/arm/Nav2 primitives.

## What was added / changed (vs. the HW4 stack)

| File | Change |
|------|--------|
| `ros2_yolo_integration/.../object_detect.py` | `bear`-only; subscribes `/camera/image/camera_info`; back-projects the bear bbox-center + depth into a 3D point and publishes `/yolo/target_marker` (replaces the manual Foxglove `/clicked_point`). Also fixes the `delta_x` image-center bug. |
| `pros_car/.../task1_mission.py` | **New.** `Task1Mission` — SEARCH → APPROACH → OBSERVE(5s) → GRIP → RETURN(Nav2) state machine. |
| `pros_car/.../tf_to_amcl_pose.py` | **New.** Republishes `map → base_footprint` TF as `/amcl_pose` so the existing Nav2 follower works without AMCL. |
| `pros_car/.../mode_manager.py`, `mode_app.py`, `main2.py` | Register the **"Task 1 Mission"** menu entry. |
| `pros_app/final_project_unity.sh` | **New.** One-shot launcher: robot_unity + slam_unity + navigation_unity (online SLAM + Nav2). Also added to `control.py`'s menu. |

Tunables live at the top of `Task1Mission.__init__` (`APPROACH_STOP_DIST` ≈ the in-sim
*N*, `OBSERVE_SECONDS`, `ALIGN_PX`, `GRIP_WAIT`, `RETURN_ARRIVE_DIST`).

## Run

**0. Simulator** (NVIDIA offload — see HW4 README for the full env vars):
```bash
cd pros_twin_linux/pros_twin_unity_linux/
./pros_twin_unity_tsai_run_linux.x86_64 -force-vulkan
# log in with TA username/key → EASY → FINAL PROJECT scene → AI mode
```

**1. Stack — online SLAM + Nav2** (Terminal 1):
```bash
cd workspace/pros/pros_app
python3 ./control.py -s        # choose ./final_project_unity.sh
```

**2. YOLO perception** (Terminal 2):
```bash
cd workspace/pros/ros2_yolo_integration
./yolo_activate.sh
r                              # colcon build + source
ros2 run yolo_example_pkg yolo_node
```

**3. Pose shim** (Terminal 3):
```bash
cd workspace/pros/pros_car
./car_control.sh
r
ros2 run pros_car_py tf_to_amcl_pose
```

**4. Mission** (Terminal 4):
```bash
cd workspace/pros/pros_car
./car_control.sh
r
ros2 run pros_car_py robot_control
# select "Task 1 Mission"  (press q to stop)
```

Watch live state in Terminal 4's log (SEARCH → APPROACH → OBSERVE → GRIP → RETURN), and
the in-sim TASK panel ticking Locate & Observe then Recovery.

## Verify (Foxglove `ws://localhost:9090`)
- `/map` grows as the rover moves (SLAM); Nav2 active; `/amcl_pose` published by the shim.
- `/yolo/detection/compressed` boxes the bear; `/yolo/target_marker` sits on the bear in 3D.
- Mission reaches the bear, holds 5 s (Locate & Observe), grips, and Nav2-returns (Recovery).
- Re-open the **FINAL PROJECT** tab 2–3× (new random map) to confirm robustness.

## Known integration check
`final_project_unity.sh` runs `navigation_unity.xml`, which passes a `map` arg to
nav2_bringup's `navigation_launch.py`. If that launch rejects the arg, remove the
`<arg name="map" .../>` line in `pros_app/docker/compose/demo/navigation_unity.xml` — the
Nav2 costmap only needs the `/map` topic that slam_toolbox publishes, not a map server.

## Roadmap — Tasks 2 & 3
- **Task 2 (bridge):** use `segmentation.pt` (`bridge` mask) to center on the bridge, drive up
  (Ascent) / over / down (Descent), then reuse APPROACH→GRIP→RETURN for the bridge bear.
- **Task 3 (door):** `knob` is already a trained class. Locate & Observe reuses APPROACH; Unlock
  = arm interaction with the knob; Clear = drive the body forward to push the door fully open.
