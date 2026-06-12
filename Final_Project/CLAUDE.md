# Final Project — Autonomous Unity Rover (CLAUDE.md)

Operating spec for working on and running this project. Read this before touching the stack.

## Goal

Autonomous **Task 1** of the Unity rover challenge, fully hands-free (no Foxglove clicking, no
keyboard driving):

1. **SEARCH** — find a bear with YOLO.
2. **APPROACH / OBSERVE** — drive up and hold still ≥5 s in front of it → **Locate & Observe (10 pts)**.
3. **GRIP** — auto-grip with the arm.
4. **RETURN** — Nav2 back to the recorded start pose → **Recovery (20 pts)**.

Strategy: **reactive visual servoing** (YOLO `/yolo/target_info` + depth) for search→approach→grip,
then **online SLAM + Nav2** to return. The map is **randomized each run**, so the costmap is built
live — there is no map pre-pass.

**Tasks 2 (bridge) and 3 (door) are now implemented** as their own state machines, same hands-free
pattern as Task 1:
- **Task 2** — bridge (no IMU — Unity has none; deterministic scene + spawn). **One bear on the
  bridge.** Pose-based coarse dock to the ramp foot, then bear vision climbs to it (bear detection is
  far more stable near the ramp than the bridge/road masks). Flow: **BRIDGE_APPROACH** — drive to
  hand-measured `DOCK_POSE = (0.90, 0.38) yaw≈90.7°` in the **pinned `/amcl_pose` (0,0,0)-spawn frame**
  (see the localization-pin gotcha below — only valid after `reset_map.sh --pin`) with Task 1's
  RETURN-style controller → **SNAP_90** — rotate to the bridge axis (~`DOCK_YAW_RAD` 90.7°) so the
  bridge centers and the off-bridge decoy rotates out of frame, then creep straight forward
  `SNAP_FWD_SEC` to fix translation / dock onto the ramp → **VISUAL_CLIMB** — lower the open bulldozer
  claw, then **align-then-go**: if `|delta_x| > VCLIMB_ALIGN_PX` rotate in place to center the bear,
  else drive full-thrust straight up the bridge (re-checked every frame). Two defenses keep it on the
  bridge bear, not the decoy: (1) `YOLO_TARGET_PICK=onbridge` — the detection node subscribes to
  `/yolo/bridge_info` and reports only the bear horizontally aligned with the bridge (latched through
  segmentation dropouts; falls back to nearest when no bridge in view); (2) SNAP_90 having rotated the
  decoy out of frame. Commit to GRIP when depth ≤ `VCLIMB_GRIP_DIST` for N frames, seen-then-lost-at-
  close (scooped), or on `VCLIMB_TIMEOUT`; a stuck-guard reverses briefly if bear depth stops
  decreasing → **GRIP** — press forward + `scoop_grab` (close + lift) → **DESCEND** — keep driving
  down the far side: full-thrust over the crest for `DESCEND_MIN_SEC`, then controlled, ending on a
  **vision stop** (far-side road mask `area_frac ≥ DESCEND_ROAD_AREA`, or bridge mask
  `area_frac ≤ DESCEND_BRIDGE_AREA`, held `DESCEND_DONE_CONFIRM` frames) with `DESCEND_MAX_SEC` as a
  fallback — not a blind timer → pose-**RETURN to start** and release. On-bridge `/amcl_pose` freezes,
  so VISUAL_CLIMB/DESCEND use vision only, no pose.
- **Task 3** — door knob: Locate & Observe the knob (`detection.pt` `knob` class via
  `YOLO_TARGET=knob` → `/yolo/target_info_knob`), **UNLOCK** with an arm poke
  (`arm_controller.knob_poke`), then **CLEAR** by driving the body forward to push the door open.

`detection.pt` has `bear`/`knob` classes; each detection node tracks one target (`YOLO_TARGET` env,
default `bear`). The consolidated `start_stack.sh` runs two detection nodes — bear on
`/yolo/target_info` and knob remapped to `/yolo/target_info_knob` — plus the `segmentation.pt`
(`bridge`/`road`) node, so all three tasks' perception is live at once.

## TA rules that constrain how we demo (source: `docs/ta-qa.md`)

- **No task ordering** — Task 1/2/3 in any order; only completion matters.
- **All tasks in ONE session** — highest score is taken.
- **Difficulty does not affect scoring** — always use **Easy** (gives LiDAR + Depth Map + RGB).
- **Off-road is allowed** — the rover need not stay on the road.
- **Scoring**: Canva slides p.108–109; pick one map to demo; each map has its own high score.
  Scores are server-timestamped and ranking uses that time.
- **Unity-pass but server-fail**: record video as proof, discuss with TA.



## The stack (isolated — `kylefp`)

Runs on its own ROS domain/network so it can coexist with another user on the shared GPU box.

| Setting | Value | Why |
|---|---|---|
| Compose project | `kylefp` | isolates containers + network `kylefp_my_bridge_network` |
| `ROS_DOMAIN_ID` | **7** | off the shared default `1` (set in all `.env` files) |
| rosbridge | host **9091** → container 9090 | Unity connects here; set in `docker-compose_slam_unity.yml` |
| Foxglove | `ws://localhost:8766` (host 8766 → 8765) | viewer |
| Unity display | `:20` (Chrome Remote Desktop), `__NV_PRIME_RENDER_OFFLOAD=1` | NVIDIA offload |

Containers brought up: compose stack (`kylefp-*-1`: robot_bringup, slam, navigation, lidar_trans,
rosbridge) + `kylefp-yolo` (bear) + `kylefp-yolo-knob` (knob, remapped) + `kylefp-yolo-seg` (bridge)
+ `kylefp-tfshim` + `kylefp-foxglove`, plus the Unity binary — all from one `start_stack.sh`.

## Running it

**Bring the whole stack up (detached) — only if it's down** (after reboot / removed containers). One
consolidated launcher brings up **everything for all three tasks at once** (bear + knob + bridge
perception all live, on separate topics — see Key ROS topics):
```bash
~/Desktop/Robot-navigation-projects/Final_Project/start_stack.sh
```
Then **in Unity** (Chrome Remote Desktop): log in → **FINAL PROJECT** → **CAR & ARM Mode = AI** →
**RosBridge PORT = 9091** → press **Reload** (must show "Connected").

**Run the mission (every attempt) — the stack stays up between tasks, no restarts:**
```bash
~/Desktop/Robot-navigation-projects/Final_Project/workspace/pros/pros_car/run_task1.sh   # task1_auto
~/Desktop/Robot-navigation-projects/Final_Project/workspace/pros/pros_car/run_task2.sh   # task2_auto
~/Desktop/Robot-navigation-projects/Final_Project/workspace/pros/pros_car/run_task3.sh   # task3_auto
```
Runs `taskN_auto` headless in your terminal — watch the live `[TaskN]` state log (every state
transition + a throttled per-tick sensor readout, tuned for fast knob calibration). **Ctrl-C** stops;
re-run to retry (new random map each time). Because all perception runs at once, you can run Task 1 →
Task 2 → Task 3 back-to-back in one session without touching containers.

Manual per-piece `docker run` commands (for debugging individual containers) are in the README.

## Build / edit / test loop

- Source lives under `workspace/pros/<pkg>/src` and is **bind-mounted** into the containers at
  `/workspaces/src`. Editing a `.py` on the host changes it in the container.
- ROS packages must be **rebuilt + re-sourced** to take effect — the run scripts already do
  `colcon build && source install/setup.bash && ros2 run ...` on each launch.
  - YOLO node: `yolo_example_pkg` → `ros2 run yolo_example_pkg yolo_node`
  - Car/mission: `pros_car_py` → entry points below.
- After editing the YOLO or mission node, **restart that container** (or re-run `run_task1.sh` for
  the mission) so the rebuild picks up changes.

`pros_car_py` console entry points (`setup.py`):
`robot_control` (menu UI), `task1_auto` / `task2_auto` / `task3_auto` (the autonomous missions),
`tf_to_amcl_pose` (pose shim), `lidar_trans`, plus arm/serial helpers.

## Code map

| Path | Role |
|---|---|
| `workspace/pros/pros_car/src/pros_car_py/pros_car_py/task1_mission.py` | **`Task1Mission`** state machine (SEARCH→APPROACH→OBSERVE→CREEP→GRIP→RETURN→DONE). Tunables at top of `__init__`. |
| `.../pros_car_py/task2_mission.py` | **`Task2Mission`** (no IMU; bridge bear + off-bridge decoy): BRIDGE_APPROACH (pose dock to `DOCK_X/Y`) → SNAP_90 (rotate to bridge axis so decoy leaves frame, then creep forward to dock onto ramp) → VISUAL_CLIMB (lower open claw, align-then-go up the bridge on the bear's `delta_x`; commit to GRIP on close-depth / seen-then-lost-at-close / timeout; stuck-guard reverse) → GRIP (press + `scoop_grab`) → DESCEND (drive down the far side; vision-terminated — far-side road mask appears / bridge mask shrinks, `DESCEND_MAX_SEC` fallback) → RETURN-to-start. Bridge-bear selection is in the YOLO node (`YOLO_TARGET_PICK=onbridge`). On-bridge `/amcl_pose` freezes so VISUAL_CLIMB/DESCEND are vision-only. Tunables (`SNAP_FWD_SEC`, `VCLIMB_SPEED`, `VCLIMB_ALIGN_PX`, `VCLIMB_GRIP_DIST`, `VCLIMB_MAX_TRACK_DIST`, `DESCEND_ROAD_AREA`, `DESCEND_BRIDGE_AREA`) at top of `__init__`. |
| `.../pros_car_py/task3_mission.py` | **`Task3Mission`**: door knob (SEARCH→APPROACH→OBSERVE→UNLOCK→CLEAR). Knob via `YOLO_TARGET=knob`. Tunables at top of `__init__`. |
| `.../pros_car_py/tf_to_amcl_pose.py` | republishes `map→base_footprint` TF as `/amcl_pose` so Nav2 follower works without AMCL. |
| `.../pros_car_py/ros_communicator.py` | pub/sub hub: `publish_car_control`, `publish_raw_car_control`, `get_latest_amcl_pose`, `get_latest_bridge_info`, `get_latest_road_info`, `get_latest_knob_target_info`, etc. |
| `.../pros_car_py/nav_processing.py`, `nav2_utils.py` | Nav2 plan-following + geometry helpers. |
| `.../pros_car_py/arm_controller_2D.py` | scoop grab (`scoop_pose/grab/release`), auto-grip (`auto_control(key='g')`), door-knob poke (`knob_poke()`). |
| `workspace/pros/ros2_yolo_integration/.../object_detect.py` | YOLO **detection** node: single target by `YOLO_TARGET` env (`bear` default / `knob`); back-projects bbox-center + depth → `/yolo/target_marker`; publishes `/yolo/target_info`. |
| `workspace/pros/ros2_yolo_integration/.../segment_detect.py` | YOLO **segmentation** node (`yolo_seg_node`): `bridge`/`road` masks → `/yolo/segmentation/compressed`; publishes `/yolo/bridge_info` (bridge geometry: delta_x, area, bottom-edge dx, symmetry) + `/yolo/road_info` (road centroid) for Task 2. |
| `start_stack.sh` | **single consolidated launcher**: one-shot detached bring-up of the entire stack + all three YOLO containers (bear / knob-remapped / bridge-seg) + Unity. Run any task against it, no restarts. |
| `workspace/pros/pros_car/run_task1.sh` / `run_task2.sh` / `run_task3.sh` | build + run the mission headless. |
| `tools/reset_map.sh`, `tools/click20.py` | hands-free map reset via synthetic clicks on `:20`. |

## Key ROS topics

- `/camera/image/compressed`, `/camera/depth/compressed`, `/camera/x_multi_depth_values` — sensor in.
- `/yolo/detection/compressed` — annotated image (every frame). `/yolo/target_info`
  (`Float32MultiArray`: found, distance, delta_x, area_frac, bottom_frac — every frame) = **bear**
  (`kylefp-yolo`, `YOLO_TARGET=bear`). `/yolo/target_marker` (3D Marker — **only** when a graspable
  target has valid depth).
- `/yolo/target_info_knob` — same format, **knob** for Task 3 (`kylefp-yolo-knob`, `YOLO_TARGET=knob`,
  outputs remapped off the bear topics so both detection nodes coexist). Read via
  `data_processor.get_knob_target_info()`. Companion remaps: `/yolo/target_marker_knob`,
  `/yolo/detection_knob/compressed`.
- `/yolo/segmentation/compressed` — bridge/road mask overlay (Task 2). `/yolo/bridge_info`
  (`Float32MultiArray`: found, delta_x, area_frac, centroid_y_frac, bottom_edge_dx, symmetry,
  aspect_ratio — every frame) is published; Task 2 currently uses only `found` and `sign(delta_x)`
  for a coarse "bridge-is-on-the-left/right" intersection-turn hint (the other fields proved too
  noisy when close to the bridge and are not consulted by the mission). `/yolo/road_info` (found, delta_x, area_frac)
  is the road-follow centroid. Read via `data_processor.get_bridge_info()` / `get_road_info()`.
- **No IMU in the Unity sim.** `/imu/data` is a *hardware-only* topic (`docker-compose_imu.yml` →
  `pros_imu` reading `/dev/imu_usb`), not launched and not simulated — so Task 2 is **road-led**, not
  pitch-based, and judges "crossed" by `/amcl_pose` distance (`CLIMB_DISTANCE`).
- `/map` (SLAM), `/amcl_pose` (from the shim), `/tf` (~48 Hz), `/car_C_front_wheel`,
  `/car_C_rear_wheel` (wheel speed commands).

## Mission tuning (top of `Task1Mission.__init__`)

Sensor/geometry reality drives these: depth saturates below ~0.45 m, arm reach ≈0.19 m.
- `APPROACH_STOP_DIST = 0.50` — stop at nearest reliable depth to do Locate & Observe.
- `ALIGN_PX = 35.0` — center tolerance so the bear sits on the gripper's center axis.
- `OBSERVE_SECONDS = 5.5` — hold time (>5 s scores, with margin).
- `CREEP_DRIVE_SECONDS = 1.3` — blind forward push to get the bear into arm reach after observing.
- `GRIP_WAIT = 15.0`, `MAX_GRIP_ATTEMPTS = 3`.
Tune these against the in-sim "N units" and the gripper geometry.

## Operational gotchas (learned the hard way)

- **Foxglove "no Hz" is usually normal.** `foxglove_bridge` only streams (and shows a rate for)
  topics something is **subscribed** to. Foxglove auto-subscribes to `/tf` + `/tf_static`, so only
  those show Hz in the Topics list by default. A blank Hz ≠ "not publishing." To check a topic, open
  a panel on it (Image panel for cameras) or run `ros2 topic hz <topic>` inside a container.
- **Camera not flowing → everything YOLO is silent.** The YOLO node is callback-driven by
  `/camera/image/compressed`; if Unity isn't connected in AI mode on 9091, no `/yolo/*` publishes.
  Canary topics: `/yolo/detection/compressed` and `/yolo/target_info` (publish every frame).
- **Map reset for retries:** `tools/reset_map.sh` switches RACING2026↔FINAL PROJECT to force a fresh
  scene (clicking FINAL PROJECT while already in it does NOT reload). For RETURN/Nav2 tests use
  `reset_map.sh --slam` to also restart SLAM + navigation (SLAM restart de-syncs Nav2 costmaps).
- **`/amcl_pose` map frame floats every session → pin it for pose-based nav (Task 2).** Odometry
  comes from a laser scan-matcher (`scan_matcher`, `ros2_laser_scan_matcher`, in `robot_bringup`)
  that integrates forever and never resets; `slam_toolbox` (`mode: mapping`, no loaded map) anchors
  `map` to that odom on its first scan. So the *same physical spawn* reads wildly different
  coordinates each session (observed: `(2.87,-3.46)`, `(-3.1,-4.3)`, `(-4.21,-4.24)`), and any
  hard-coded absolute pose (e.g. Task 2's `DOCK_X/Y/YAW`) silently goes stale. **Fix / ritual:** with
  the car at spawn, run **`reset_map.sh --pin`** — it re-origins the scan_matcher odom (restart
  `robot_bringup`) → re-anchors SLAM → resyncs Nav2, *in that order*, so spawn ≡ `map (0,0,0)` and the
  whole deterministic scene gets reproducible coordinates. Verify: `/amcl_pose` at spawn ≈ `(0,0,0)`.
  Note `slam_toolbox`'s `map_start_pose` does **not** pin a fresh map (verified) — the odom reset is
  the actual lever. Task 2's dock constants were measured in this pinned frame. Full rationale:
  `docs/superpowers/specs/2026-06-11-task2-localization-pin-spawn-design.md`.
- **Nav2 `map` arg:** if `navigation_launch.py` rejects the `map` arg, remove the `<arg name="map">`
  line in `pros_app/docker/compose/demo/navigation_unity.xml` — Nav2 only needs the `/map` topic.

## Conventions

- Don't change the domain/port/network values piecemeal — they must stay consistent across all
  `.env` files, the slam compose, and the Unity RosBridge port (9091).
- Don't commit the Unity binary (`pros_twin_linux*/`, `*.zip`) or the large PDFs — already gitignored.
- Commit style: `type(Final_Project): summary` (e.g. `feat`, `fix`, `chore`, `docs`).
- Reference docs: `docs/ta-qa.md` (rules), `docs/Unity_final_project_spec.pdf`,
  `docs/5_29_update.pdf`. Demo videos in `TA_demo_videos/`.
</content>
