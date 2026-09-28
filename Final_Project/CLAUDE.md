# Final Project — Autonomous Unity Rover (CLAUDE.md)

Operating spec for working on and running this project. Read this before touching the stack.

## Goal

Hands-free autonomous missions for the Unity rover challenge: no Foxglove clicking, no keyboard driving.
Each task is its own state machine in `workspace/pros/pros_car/src/pros_car_py/pros_car_py/taskN_mission.py`, and all three run against one shared stack.
The descriptions below follow the code (the README has a diagram per task).
All missions drive with `publish_raw_car_control` / named actions on `/amcl_pose` + vision; none uses Nav2.

- **Task 1 (bear):** SEARCH → APPROACH → OBSERVE → CREEP → GRIP → RETURN → DONE.
  SEARCH rotates in place (nudging forward for ~0.8 s every 12 s) until YOLO confirms a bear for 3 frames.
  APPROACH servos on `/yolo/target_info` `delta_x` and depth, and enters OBSERVE when centred (`ALIGN_PX` 70) within `APPROACH_STOP_DIST` (0.5 m), or when the bear is lost after the car got within `COMMIT_DOCK_DIST` (0.7 m), since the gripper then occludes it.
  OBSERVE holds still for `OBSERVE_SECONDS` = 5.5 s → **Locate & Observe** (needs >5 s).
  CREEP runs a blocking `scoop_pose()` (arm down, claw open), then pushes forward for `BULLDOZER_PUSH_SEC`; GRIP runs `scoop_grab()` (close + lift).
  RETURN is a go-to-point on `/amcl_pose` back to the start pose recorded at launch, then `scoop_release()` → **Recovery**.
- **Task 2 (bridge + bear):** BRIDGE_APPROACH → VISUAL_CLIMB → OBSERVE → GRIP → SNAP_DESCEND → DESCEND → RETURN → DONE.
  Scoring is Ascent + Descent + Recovery (no Locate & Observe); there is one bear on the bridge and an off-bridge decoy.
  BRIDGE_APPROACH drives the 10 measured `WAYPOINTS` in the pinned `(0,0,0)` spawn frame (needs `reset_map.sh --pin`); a stuck guard on the last waypoint also hands over, because `/amcl_pose` freezes on the bridge.
  VISUAL_CLIMB runs `scoop_pose()` and climbs at `VCLIMB_SPEED` for `VCLIMB_CLIMB_SEC` (3 s; the timer is the only exit), steering on the bear when it is within `VCLIMB_MAX_TRACK_DIST` (4 m), else on the bridge mask `delta_x`, else straight.
  OBSERVE rotates to face a near bear (≤ `OBSERVE_NEAR_DIST`, `OBSERVE_ALIGN_PX`, `OBSERVE_FACE_TIMEOUT`) and holds `OBSERVE_SECONDS` (2 s), for grip alignment only.
  GRIP presses forward for `GRIP_PRESS_SEC`, then `scoop_grab()`.
  SNAP_DESCEND rotates until the bridge mask (road mask if no bridge) is centred within `SNAP_ROAD_PX` for `SNAP_ROAD_CONFIRM` (3) frames, or until `SNAP_DESCEND_TIMEOUT` (6 s); DESCEND drives down steering on the bridge mask (road fallback) until both masks are gone, the road fills the view (`DESCEND_ROAD_AREA`), or `DESCEND_MAX_SEC` (5 s).
  RETURN drives the 12 `RETURN_WAYPOINTS` around the bridge (skipping one it is stuck on), then go-to-point to the start and `scoop_release()`.
  The bear picker lives in the YOLO node: `YOLO_TARGET_PICK=onbridge` makes it subscribe to `/yolo/bridge_info` and report only the bear aligned with the bridge.
- **Task 3 (door knob):** DRIVE_WP → UNLOCK → CLEAR → DONE.
  A blocking `knob_stow()` runs first.
  DRIVE_WP drives the 13 measured `WAYPOINTS` to the door in the pinned frame (needs `--pin`), skipping stuck intermediate waypoints; being stuck at the last one also hands over.
  UNLOCK is one blocking macro: rotate until the heading is within `UNLOCK_ALIGN_DEG` of +x, pursue the knob on `/yolo/target_info_knob` for up to `UNLOCK_NUDGE_SEC` (3 s), `knob_raise()`, then start `knob_press_down()` in a background thread.
  CLEAR drives at `CLEAR_SPEED` holding yaw 0 for `CLEAR_PUSH_SEC` (15 s) while the lever stays pressed, pushing through the door.
  **This path has no >5 s hold in front of the door**, so it does not do Locate & Observe; the SEARCH → APPROACH → OBSERVE (5.5 s) → UNLOCK branch only runs when `WAYPOINTS` is empty.
  `knob_poke()` and `knob_retract()` exist in the arm controller but are not called.

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

**Combined Task 2 + Task 3 in one run (`task23_auto` / `combined_mission.py`):** `run_task23.sh` runs the full Task 2, waits 2 s, then runs the full Task 3, reusing the `Task2Mission`/`Task3Mission` classes unchanged (one shared `RosCommunicator` + spin thread).
Task 3 runs however Task 2 ended (success, timeout or the 360 s watchdog).
Run **`reset_map.sh --pin` once first**, because both tasks' absolute waypoints live in that pinned spawn frame.
After Task 2 returns to spawn (facing ~−180°), Task 3's `DRIVE_WP` spins to turn around and drives to the door.
DRIVE_WP and the UNLOCK/CLEAR heading hold trust the pinned frame, so drift from the Task 2 run carries over; only UNLOCK's knob pursuit (up to 3 s) corrects the lateral offset by vision.
Missions run headless in your terminal: watch the live `[TaskN]` state log (every transition, plus a throttled per-tick sensor readout in Tasks 2 and 3).
**Ctrl-C** stops; re-run to retry.
Because all perception runs at once, Task 1 → Task 2 → Task 3 can run back-to-back in one session; `reset_map.sh --pin` restarts only odometry and SLAM, never the perception containers.

Each `run_taskN.sh` is the `docker run` command for its mission, with its prerequisites in the header comment.

## Build / edit / test loop

- Source lives under `workspace/pros/<pkg>/src` and is **bind-mounted** into the containers at
  `/workspaces/src`. Editing a `.py` on the host changes it in the container.
- ROS packages must be **rebuilt + re-sourced** to take effect — the run scripts already do
  `colcon build --symlink-install && source install/setup.bash && ros2 run ...` on each launch.
  - YOLO node: `yolo_example_pkg` → `ros2 run yolo_example_pkg yolo_node`
  - Car/mission: `pros_car_py` → entry points below.
- After editing the YOLO or mission node, **restart that container** (or re-run `run_task1.sh` for
  the mission) so the rebuild picks up changes.

### Fast iteration (demo-time) — persisted build cache + `NOBUILD=1`

The mission containers run `--rm`, which used to wipe `/workspaces/{build,install}` and force a
**cold full build of all 8 packages every launch** (incl. the two slow CMake interface pkgs
`action_interface`, `custome_interfaces`). The `run_task*.sh` scripts now fix this:
- **Build cache persists** in two named volumes (`kylefp_colcon_build`, `kylefp_colcon_install`),
  so colcon goes **incremental** — unchanged packages (the CMake interface pkgs, untouched python)
  are skipped on every run after the first.
- **`--symlink-install`** makes the install tree symlink back to `src`, so a pure-Python edit
  (`task2_mission.py` and the other mission/tunable files in `pros_car_py`) is live in the install
  tree with no recompile.

Three iteration speeds (pick per edit):

| Command | Behavior | Use when |
|---|---|---|
| `./run_task2.sh` | incremental `colcon build --symlink-install` | default — always correct |
| `NOBUILD=1 ./run_task2.sh` | **skips colcon entirely → instant start** | edited only `.py` logic in `pros_car_py` |
| `./run_task2.sh` (plain) | full incremental build | changed `setup.py` entry points, added a new file, or touched a CMake / `*_interface` package |

`NOBUILD=1` is self-guarding: it falls back to a real build if no install tree exists yet (e.g. fresh
volume), so the **first run after a reboot/volume-wipe still builds**. Caveat: `NOBUILD=1` runs
whatever is already installed — if you skip the build after changing `setup.py` or a CMake package you
silently run stale code. Rule: `.py` logic edit → `NOBUILD=1` safe; structure/entry-point/interface
change → run plain once.

If the cache ever gets into a stale/bad state, nuke it and let the next run rebuild fresh:
`docker volume rm kylefp_colcon_build kylefp_colcon_install`. **Prime it with one warm-up run before
a demo** so the slow first build isn't on the clock.

`pros_car_py` console entry points (`setup.py`):
`robot_control` (menu UI), `task1_auto` / `task2_auto` / `task3_auto` (the autonomous missions), `task23_auto` (Task 2 → Task 3 combined),
`tf_to_amcl_pose` (pose shim), `lidar_trans`, plus arm/serial helpers.

## Code map

| Path | Role |
|---|---|
| `workspace/pros/pros_car/src/pros_car_py/pros_car_py/task1_mission.py` | **`Task1Mission`** state machine (SEARCH→APPROACH→OBSERVE→CREEP→GRIP→RETURN→DONE); RETURN is a go-to-point on `/amcl_pose`. Tunables at top of `__init__`. |
| `.../pros_car_py/task2_mission.py` | **`Task2Mission`** (no IMU; bridge bear + off-bridge decoy): BRIDGE_APPROACH (pose `DRIVE_WP` through `WAYPOINTS`) → VISUAL_CLIMB (timed `VCLIMB_CLIMB_SEC` climb, steer on bear, else bridge mask) → OBSERVE (face the bear, `OBSERVE_SECONDS`) → GRIP (press + `scoop_grab`) → SNAP_DESCEND / DESCEND (bridge mask, road fallback) → RETURN (`RETURN_WAYPOINTS` detour, then go-to-point + release). On-bridge `/amcl_pose` freezes, so the bridge states are vision-only. Tunables at top of `__init__`. |
| `.../pros_car_py/task3_mission.py` | **`Task3Mission`**: door knob (DRIVE_WP→UNLOCK→CLEAR; SEARCH→APPROACH→OBSERVE only when `WAYPOINTS` is empty). Knob via `YOLO_TARGET=knob`. Tunables at top of `__init__`. |
| `.../pros_car_py/tf_to_amcl_pose.py` | republishes `map→base_footprint` TF as `/amcl_pose` (no AMCL runs); every mission navigates on it. |
| `.../pros_car_py/ros_communicator.py` | pub/sub hub: `publish_car_control`, `publish_raw_car_control`, `get_latest_amcl_pose`, `get_latest_bridge_info`, `get_latest_road_info`, `get_latest_knob_target_info`, etc. |
| `.../pros_car_py/nav_processing.py`, `nav2_utils.py` | Nav2 plan-following + geometry helpers (the missions only use the geometry helpers, e.g. `calculate_angle_point`). |
| `.../pros_car_py/arm_controller_2D.py` | scoop grab (`scoop_pose/grab/release`), auto-grip (`auto_control(key='g')`), door-knob moves (`knob_stow/raise/press_down`; `knob_poke`/`knob_retract` are unused). |
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
  aspect_ratio; every frame) is published.
  Task 2 steers VISUAL_CLIMB, SNAP_DESCEND and DESCEND on its `found` and `delta_x`; the other fields are not consulted.
  `/yolo/road_info` (found, delta_x, area_frac) is the road-mask fallback for SNAP_DESCEND/DESCEND, and `area_frac` ends DESCEND.
  Read via `data_processor.get_bridge_info()` / `get_road_info()`.
- **No IMU in the Unity sim.** `/imu/data` is a *hardware-only* topic (`docker-compose_imu.yml` →
  `pros_imu` reading `/dev/imu_usb`), not launched and not simulated.
  Task 2 is therefore vision-led on the bridge (timed climb, mask-steered descent), not pitch-based.
- `/map` (SLAM), `/amcl_pose` (from the shim), `/tf` (~48 Hz), `/car_C_front_wheel`,
  `/car_C_rear_wheel` (wheel speed commands).

## Mission tuning (top of `Task1Mission.__init__`)

Sensor/geometry reality drives these: depth saturates below ~0.45 m, arm reach ≈0.19 m.
- `APPROACH_STOP_DIST = 0.50`: stop at the nearest reliable depth to do Locate & Observe.
- `ALIGN_PX = 70.0`: center tolerance, loose enough that rotate-in-place does not overshoot and hunt.
- `COMMIT_DOCK_DIST = 0.7`: losing the bear after getting this close counts as docked (the gripper hides it).
- `OBSERVE_SECONDS = 5.5`: hold time (>5 s scores, with margin).
- `BULLDOZER_PUSH_SEC = 1.0` at `CREEP_SPEED = 90`: forward push after `scoop_pose()` to get the bear into the open claw, re-centring on the bear while it is still visible.
- `RETURN_ARRIVE_DIST = 0.60`, `RETURN_TIMEOUT = 120`: go-to-point arrival radius and safety timeout.
Tune these against the in-sim "N units" and the gripper geometry.

## Operational gotchas (learned the hard way)

- **Foxglove "no Hz" is usually normal.** `foxglove_bridge` only streams (and shows a rate for)
  topics something is **subscribed** to. Foxglove auto-subscribes to `/tf` + `/tf_static`, so only
  those show Hz in the Topics list by default. A blank Hz ≠ "not publishing." To check a topic, open
  a panel on it (Image panel for cameras) or run `ros2 topic hz <topic>` inside a container.
- **Camera not flowing → everything YOLO is silent.** The YOLO node is callback-driven by
  `/camera/image/compressed`; if Unity isn't connected in AI mode on 9091, no `/yolo/*` publishes.
  Canary topics: `/yolo/detection/compressed` and `/yolo/target_info` (publish every frame).
- **Map reset for retries:** `tools/reset_map.sh` switches RACING2026↔FINAL PROJECT to force a fresh scene (clicking FINAL PROJECT while already in it does NOT reload).
  `reset_map.sh --slam` (restart SLAM + Nav2) is a legacy flag from the old Nav2 return; no mission needs it now.
- **`/amcl_pose` map frame floats every session → pin it for pose-based nav (Tasks 2 and 3).**
  Odometry comes from a laser scan-matcher (`scan_matcher`, `ros2_laser_scan_matcher`, in `robot_bringup`) that integrates forever and never resets, and `slam_toolbox` (`mode: mapping`, no loaded map) anchors `map` to that odom on its first scan.
  So the *same physical spawn* reads wildly different coordinates each session (observed: `(2.87,-3.46)`, `(-3.1,-4.3)`, `(-4.21,-4.24)`), and any hard-coded absolute pose (e.g. the Task 2/3 `WAYPOINTS`) silently goes stale.
  **Fix / ritual:** with the car at spawn, run **`reset_map.sh --pin`**: it re-origins the scan_matcher odom (restart `robot_bringup`), then re-anchors SLAM (restart `slam`), *in that order*, so spawn ≡ `map (0,0,0)` and the whole deterministic scene gets reproducible coordinates.
  It **auto-verifies**: after the slam restart it blocks on a single `/amcl_pose` echo that returns the instant the full TF chain re-anchors (slam `map→odom` + scan_matcher `odom→base`, via tfshim) and prints the spawn pose, so you eyeball `(0,0,0)` without a separate command.
  For demo-time speed, that wait plus `docker restart -t 3` replaces the old fixed `sleep 8` after slam with the actual settle time (~2-5 s); bringup keeps a short fixed `sleep 6`, purely so its odom is at 0 before slam reads it.
  Two gotchas are baked in.
  The ros2 calls use **`--no-daemon`**, because the long-lived containers carry a wedged ros2 daemon (`xmlrpc !rclpy.ok()`) that makes daemon-routed `topic echo`/`list` fail instantly.
  The readiness signal is `/amcl_pose` (continuously published by tfshim), **not** `/odom`, which scan_matcher only publishes on movement, so `echo` can't even determine its type when the car is idle.
  `--pin` does not restart Nav2 (`navigation`): every mission drives purely on `/amcl_pose` + vision (`publish_raw_car_control`), never the Nav2 stack.
  Task 1 returns to the pose it records at launch, so it works with or without `--pin`.
  Note that `slam_toolbox`'s `map_start_pose` does **not** pin a fresh map (verified); the odom reset is the actual lever.
  The Task 2/3 `WAYPOINTS` were measured in this pinned frame.
  Full rationale: `docs/superpowers/specs/2026-06-11-task2-localization-pin-spawn-design.md`.
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
