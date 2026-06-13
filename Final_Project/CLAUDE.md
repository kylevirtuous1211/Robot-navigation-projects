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

**Tasks 2 (bridge) and 3 (door) are implemented and verified working end-to-end** as their own
state machines, same hands-free pattern as Task 1. **Task 2 scores full points** (Ascent +
Descent + Recovery: mount → climb → grip → descend the far stairs → detour return around the bridge).
**Task 3 is verified working end-to-end** (waypoint drive to the door → visual dock on the knob →
Locate & Observe ≥5 s → lever press-down, held → drive through the doorway).
**Task 1 is expected to finish too** (same hands-free pattern). Task 2's scoring has **no Locate &
Observe** (only Ascent/Descent/Recovery), but it still runs an **OBSERVE phase** before gripping — not
for points, but to **rotate and face/center the bear** so the grip and the descent line up straight
(skipping it lets the car grab at a bad angle and wedge on the bridge crest):
- **Task 2** — bridge (no IMU — Unity has none; deterministic scene + spawn). **One bear on the
  bridge.** A measured waypoint path mounts the bridge centred, then it climbs on the **bridge
  segmentation mask** (stable up the incline) and grips the bear at the top. Flow: **BRIDGE_APPROACH** — pose-based
  `DRIVE_WP` through the measured `WAYPOINTS` ascent path (lined up with the bridge mouth → straight up
  the bridge axis `+y` a little onto the bridge, so the car mounts **centred** and doesn't catch the
  side), in the **pinned `/amcl_pose` (0,0,0)-spawn frame** (only valid after `reset_map.sh --pin`);
  last waypoint reached (or pose freezes on the bridge — a stuck-guard handles it) → **VISUAL_CLIMB**.
  (Empty `WAYPOINTS` falls back to the old path: pose dock to `DOCK_POSE = (0.90, 0.38) yaw≈90.7°` →
  **SNAP_90** — rotate to the bridge axis so the bridge centers and the decoy rotates out of frame,
  creep forward `SNAP_FWD_SEC` → VISUAL_CLIMB.) → **VISUAL_CLIMB** — lower the open bulldozer
  claw, then drive full-thrust up the bridge centred on the **bridge-mask centroid** via
  `_bridge_center_steer` (a `BRIDGE_DX_DEADBAND` absorbs the ~+90 px steady-state incline bias so a
  fixed offset doesn't curve the car into the side rail). Two defenses keep it on the bridge bear, not
  the decoy: (1) `YOLO_TARGET_PICK=onbridge` — the detection node subscribes to `/yolo/bridge_info` and
  reports only the bear horizontally aligned with the bridge (latched through segmentation dropouts;
  falls back to nearest when no bridge in view); (2) SNAP_90 having rotated the decoy out of frame.
  Hand off to **OBSERVE** when the bear is within `VCLIMB_OBSERVE_DIST` (≈0.65 m), seen-then-lost-at-close
  (scooped), or on `VCLIMB_TIMEOUT` → **OBSERVE** — rotate in place to face/center the bear
  (`OBSERVE_ALIGN_PX`, `OBSERVE_FACE_TIMEOUT`), hold `OBSERVE_SECONDS` so the car is squared up to the
  bear (alignment, not scoring) → **GRIP** — press forward + `scoop_grab`
  (close + lift) → **SNAP_DESCEND** — rotate to center the **bridge mask** (`b_dx`, same as the climb;
  falls back to the road mask `/yolo/road_info` `delta_x` when the bridge isn't detected — `/amcl_pose`
  freezes on the bridge so no yaw up there) so the car points straight down the stairs → **DESCEND** —
  full-thrust down, steering on the **bridge mask centre-line** (road mask only as fallback when the
  bridge isn't detected looking down the stairs), over the wide flat "fat" top and down the stairs;
  stop only when the **road fills the frame** (`road area_frac ≥ DESCEND_ROAD_AREA`,
  set high ~0.55 because the far road is already visible ~0.33 from the top) held `DESCEND_DONE_CONFIRM`
  frames, `DESCEND_MAX_SEC` fallback → pose-**RETURN** — drive the `RETURN_WAYPOINTS` detour **around**
  the bridge (never straight back *over* it carrying the bear), then go-to-point to the start pose and
  release (empty `RETURN_WAYPOINTS` = straight back, the old behavior). Return runs at full
  `RETURN_DRIVE_SPEED` through the detour (no near-waypoint slowdown), and when it stalls on the
  road↔ground lip it fires a **straight full-thrust burst** (`RETURN_BURST_SPEED`, both wheels equal)
  to clear the step before falling back to skipping the waypoint.
  On-bridge `/amcl_pose` freezes, so VISUAL_CLIMB/SNAP_DESCEND/DESCEND use vision only, no pose.
- **Task 3** — door knob (lever handle): pose-based **DRIVE_WP** through measured `WAYPOINTS`
  (pinned `(0,0,0)` spawn frame, same machinery as Task 2's BRIDGE_APPROACH) to the door, then
  **SEARCH→APPROACH** visual dock on the knob (`detection.pt` `knob` class via `YOLO_TARGET=knob`
  → `/yolo/target_info_knob`), **OBSERVE** ≥5 s (Locate & Observe), **UNLOCK** by lever press
  (`knob_raise` → forward nudge → `knob_press_down` → `knob_retract`), then **CLEAR** by driving
  straight through the now-open door. Empty `WAYPOINTS` falls back to the old rotate-in-place SEARCH.

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
~/Desktop/Robot-navigation-projects/Final_Project/workspace/pros/pros_car/run_task23.sh  # task23_auto (Task 2 → Task 3 combined)
```

**Combined Task 2 + Task 3 in one run (`task23_auto` / `combined_mission.py`):** `run_task23.sh`
runs the **full Task 2** (mount → climb → grip → descend → return + release) then the **full Task 3**
(drive to the door → dock on the knob → Locate & Observe ≥5 s → lever press → drive through) back-to-back
in a single session, scoring all points from both. It reuses the `Task2Mission`/`Task3Mission` classes
unchanged via a thin orchestrator (one shared `RosCommunicator` + spin thread, run sequentially).
Run **`reset_map.sh --pin` once first** — both tasks' absolute waypoints live in that pinned spawn frame.
After Task 2 returns to spawn (facing ~−180°), Task 3's `DRIVE_WP` spins to turn around and drives to the
door; the knob visual-dock + OBSERVE correct for any pinned-frame drift accumulated over the Task 2 run.
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
| `.../pros_car_py/task2_mission.py` | **`Task2Mission`** (no IMU; bridge bear + off-bridge decoy): BRIDGE_APPROACH (pose `DRIVE_WP` through the `WAYPOINTS` ascent path — up the bridge axis a little onto the bridge so it mounts centred; last WP / pose-freeze stuck-guard → VISUAL_CLIMB. Empty `WAYPOINTS` = old fallback: DRIVE_DOCK to `DOCK_X/Y` → SNAP_90 rotate-to-axis + creep) → VISUAL_CLIMB (lower open claw, full-thrust up the bridge centred on the bridge-mask centroid via `_bridge_center_steer`+`BRIDGE_DX_DEADBAND`; hand off to OBSERVE on near-bear `VCLIMB_OBSERVE_DIST` / seen-then-lost-at-close / timeout) → OBSERVE (rotate to face/center the bear + hold `OBSERVE_SECONDS` — alignment so the grip/descent line up straight, not Locate & Observe scoring) → GRIP (press + `scoop_grab`) → SNAP_DESCEND (rotate to center the bridge mask, road-mask fallback — no yaw on the bridge) → DESCEND (full-thrust down, bridge-mask-centred with road-mask fallback, stop when road fills the frame `road area_frac ≥ DESCEND_ROAD_AREA`, `DESCEND_MAX_SEC` fallback) → RETURN (drive the `RETURN_WAYPOINTS` detour around the bridge, then go-to-point to the start pose + release; empty list = straight back). Bridge-bear selection is in the YOLO node (`YOLO_TARGET_PICK=onbridge`). On-bridge `/amcl_pose` freezes so VISUAL_CLIMB/OBSERVE/SNAP_DESCEND/DESCEND are vision-only. Tunables (`SNAP_FWD_SEC`, `VCLIMB_SPEED`, `BRIDGE_DX_DEADBAND`, `VCLIMB_OBSERVE_DIST`, `OBSERVE_SECONDS`, `SNAP_ROAD_PX`, `DESCEND_ROAD_AREA`, `RETURN_WAYPOINTS`) at top of `__init__`. |
| `.../pros_car_py/task3_mission.py` | **`Task3Mission`**: door knob (SEARCH→APPROACH→OBSERVE→UNLOCK→CLEAR). Knob via `YOLO_TARGET=knob`. Tunables at top of `__init__`. |
| `.../pros_car_py/tf_to_amcl_pose.py` | republishes `map→base_footprint` TF as `/amcl_pose` so Nav2 follower works without AMCL. |
| `.../pros_car_py/ros_communicator.py` | pub/sub hub: `publish_car_control`, `publish_raw_car_control`, `get_latest_amcl_pose`, `get_latest_bridge_info`, `get_latest_road_info`, `get_latest_knob_target_info`, etc. |
| `.../pros_car_py/nav_processing.py`, `nav2_utils.py` | Nav2 plan-following + geometry helpers. |
| `.../pros_car_py/arm_controller_2D.py` | scoop grab (`scoop_pose/grab/release`), auto-grip (`auto_control(key='g')`), door-knob poke (`knob_poke()`). |
| `workspace/pros/ros2_yolo_integration/.../object_detect.py` | YOLO **detection** node: single target by `YOLO_TARGET` env (`bear` default / `knob`); back-projects bbox-center + depth → `/yolo/target_marker`; publishes `/yolo/target_info`. |
| `workspace/pros/ros2_yolo_integration/.../segment_detect.py` | YOLO **segmentation** node (`yolo_seg_node`): `bridge`/`road` masks → `/yolo/segmentation/compressed`; publishes `/yolo/bridge_info` (bridge geometry: delta_x, area, bottom-edge dx, symmetry) + `/yolo/road_info` (road centroid) for Task 2. |
| `start_stack.sh` | **single consolidated launcher**: one-shot detached bring-up of the entire stack + all three YOLO containers (bear / knob-remapped / bridge-seg) + Unity. Run any task against it, no restarts. |
| `.../pros_car_py/combined_mission.py` | **`task23_auto`** orchestrator: runs full Task 2 then full Task 3 in one session, reusing both mission classes (one shared `RosCommunicator`). |
| `workspace/pros/pros_car/run_task1.sh` / `run_task2.sh` / `run_task3.sh` / `run_task23.sh` | build + run the mission headless (`run_task23.sh` = Task 2 → Task 3 combined). |
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
