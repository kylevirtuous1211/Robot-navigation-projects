# Task 3 (door knob) — Finish & Verify Design

**Date:** 2026-06-13
**Status:** Approved design, ready for implementation plan
**Topic:** Add a Task-2-style waypoint front-end to the Task 3 mission, bring it
up against the live Unity sim, and field-tune it to reliably score.

## Problem

Task 3's code is structurally complete and committed (`dbb00c3`), and every
dependency is wired up:

| Dependency | Status |
| --- | --- |
| `Task3Mission` state machine (SEARCH→APPROACH→OBSERVE→UNLOCK→CLEAR→DONE) | committed |
| `data_processor.get_knob_target_info()` → `/yolo/target_info_knob` | committed |
| `ros_communicator` knob subscription + `publish_car_control` / `publish_raw_car_control` + `get_latest_amcl_pose` | committed |
| `arm_controller_2D.knob_poke()` | committed |
| YOLO knob container (`YOLO_TARGET=knob`, outputs remapped) in `docker-compose_perception_unity.yml` | present (file untracked) |
| `run_task3.sh` + `setup.py` `task3_auto` entry point | committed |

But **Task 3 has never been run against the sim even once.** CLAUDE.md marks
only Task 2 as "verified working end-to-end"; Task 3 is merely "implemented."

"Finish" therefore means two things:

1. **A small code change** — replace the current "rotate-in-place SEARCH then
   visual approach" front-end with a **Task-2-style pose-based waypoint drive**
   that coarse-navigates the rover near the knob, *then* hands off to visual
   docking. The knob detection, OBSERVE, UNLOCK, and CLEAR logic are kept.
2. **Bring-up and field-tuning** — verify each layer against the live sim and
   tune the parameters/poses so the mission reliably scores.

## Revised mission flow

```
DRIVE_WP  → pose-based: drive in order through WAYPOINTS (measured in the pinned
            (0,0,0) spawn frame) to get near the door knob. Reuses Task 2's
            BRIDGE_APPROACH machinery: get_latest_amcl_pose() go-to-point per
            waypoint with an arrive_dist tolerance and a position "stuck" guard.
VISUAL_DOCK → visual servo on the knob (/yolo/target_info_knob: delta_x center +
            distance approach) to dock close and centered. This is the existing
            APPROACH logic, entered after the waypoints instead of after a
            rotate-in-place SEARCH. A short rotate-to-acquire fallback runs if the
            knob is not yet framed at the last waypoint.
OBSERVE   → stop and hold ≥5 s on the knob (Locate & Observe, scoring).
UNLOCK    → open-door by LEVER PRESS (not a turn): raise the arm to its highest
            pose, drive forward toward the knob to position the gripper over the
            lever handle, then lower the arm to its lowest pose to press the lever
            down and unlatch the door.
CLEAR     → drive the body straight forward through the now-open door (full points).
DONE      → stop.
```

`WAYPOINTS` empty falls back to the old behavior (rotate-in-place SEARCH →
visual APPROACH), mirroring how Task 2's empty `WAYPOINTS` falls back to its old
dock path. This keeps the vision-only path available for debugging.

### Measured WAYPOINTS (pinned spawn frame)

Measured in the pinned `(0,0,0)` frame; the path runs forward in `+x` to
`(2.0, 0)`, turns and runs `+y` to `(2.0, 1.67)`, then nudges `+x` to the door at
`(2.43, 1.66)`. `arrive_dist` per point is tuned in Phase 4 (placeholder values
below; the last, in-front-of-the-knob point is tightest):

```python
# [x, y, arrive_dist]  (arrive_dist tuned in Phase 4)
WAYPOINTS = [
    [0.5,  0.0,  0.20],
    [1.0,  0.0,  0.20],
    [1.5,  0.0,  0.20],
    [2.0,  0.0,  0.20],
    [2.0,  0.5,  0.20],
    [2.0,  1.0,  0.20],
    [2.0,  1.5,  0.20],
    [2.0,  1.67, 0.15],
    [2.43, 1.66, 0.10],   # in front of the knob → hand off to VISUAL_DOCK
]
```

## Door mechanic (confirmed)

- The Task 3 "knob" is a **lever handle** on a wooden gate (see the demo
  screenshot). The door opens by **pressing the lever down**, not turning it:
  raise the arm to its highest pose, drive forward so the gripper sits over the
  lever, then lower the arm to its lowest pose to push the lever down and
  unlatch. The body then drives straight through. A 2-DOF arm (shoulder/elbow)
  handles this vertical press directly. The `UNLOCK → CLEAR` design is the correct
  shape; UNLOCK is implemented as this raise → approach → lower-press sequence.
- The **door is at a fixed location relative to spawn** in the chosen demo map
  (confirmed), exactly like the Task 2 bridge. So waypoints measured once in theu
  pinned frame stay valid run-to-run. This is the same assumption Task 2 relies
  on and is what makes the waypoint front-end viable.

## Approach: Hybrid verify-then-tune (Approach C) + waypoint front-end

Smoke-test the single highest-risk unknown first (does the model actually detect
`knob`?), because that failure invalidates everything downstream. Then measure
the approach waypoints, then collapse the rest into a tight full-mission tuning
loop. Arm poses, stop distance, and push time are all best tuned from real
end-to-end runs.

Rejected alternatives for the overall strategy:

- **A — Staged bottom-up bring-up of every layer in isolation:** maximally
  de-risking but many separate manual steps; most per-layer isolation buys
  nothing once detection is known good.
- **B — Full-mission first, fix from the log:** fastest when things mostly work,
  but on a cold start perception + nav + arm + door can fail tangled together.

## Roles & constraints

- **The human drives the sim.** Unity runs as a host GUI binary on display `:20`
  (Chrome Remote Desktop), AI mode, RosBridge port 9091. Claude cannot launch or
  observe the sim directly.
- **Claude edits code/params/poses between runs** and reads the human's
  `[Task3]` state-log output and scene reports.
- **Tuning surface:** the `WAYPOINTS` list and other constants at the top of
  `Task3Mission.__init__` (`task3_mission.py`), and the `KNOB_*` constants in
  `ArmController.__init__` (`arm_controller_2D.py`).
- **Task 3 is now a pose + vision hybrid** (was vision-only): `DRIVE_WP`
  consumes `/amcl_pose` (from the `tf_to_amcl_pose` shim), so the **pinned frame
  is now required**, not just a ritual. `VISUAL_DOCK`/`OBSERVE`/`UNLOCK`/`CLEAR`
  remain vision + blind drive.
- **Code reuse:** the `DRIVE_WP` go-to-point drive should reuse the same
  pose-based helper Task 2's `BRIDGE_APPROACH` uses. Whether to extract a shared
  waypoint-drive helper or replicate the loop inside `Task3Mission` (matching the
  project's self-contained per-task convention) is decided in the implementation
  plan.

## Start pose: standalone, not chained

Task 3 runs **standalone from a fresh spawn**, not chained off the end of Task 2.
Each run does its own `reset_map.sh --pin`, so the car starts at spawn `(0,0,0)`
and `WAYPOINTS` are measured from spawn. This keeps Task 3 independent and
repeatable (no dependence on Task 2 succeeding or its accumulated odom drift) and
matches the TA rules (tasks in any order, highest score taken).

Note: seeding the car's pose by publishing `/initialpose` is **not supported by
this stack** — there is no AMCL. `tf_to_amcl_pose` simply republishes the
`map → base_footprint` TF as `/amcl_pose`, and nothing consumes `/initialpose`
to relocalize. The frame origin is set only by `reset_map.sh --pin` (the odom
re-origin). Chaining Task 3 onto Task 2's end pose (e.g. `(0.315, -0.088,
yaw ≈ -176°)`) would instead require running Task 3 in the same pinned session
without re-pinning — explicitly out of scope here.

## Standard per-run loop

`reset_map.sh --pin` is **required** before every run (the waypoints are in the
pinned frame):

```bash
~/Desktop/Robot-navigation-projects/Final_Project/tools/reset_map.sh --pin
~/Desktop/Robot-navigation-projects/Final_Project/workspace/pros/pros_car/run_task3.sh
```

`reset_map.sh --pin` forces a fresh scene, re-origins odom, and re-anchors SLAM
so spawn ≡ `map (0,0,0)`. Verify `/amcl_pose` at spawn ≈ `(0,0,0)` before
trusting the waypoints.

## Phases

### Phase 0 — Pre-flight (~2 min)

- `start_stack.sh` up; Unity in FINAL PROJECT + AI mode + 9091 "Connected".
- Confirm `kylefp-yolo-knob` is running (`docker ps --filter name=kylefp-`).
- `reset_map.sh --pin`; verify `/amcl_pose` at spawn ≈ `(0,0,0)`.

### Phase 1 — Perception smoke test (gates everything)

With the rover facing the door, inside any container on the net:

```bash
ros2 topic echo /yolo/target_info_knob   # expect [1, dist, dx, area, bottom] when knob in frame
```

Also eyeball `/yolo/detection_knob/compressed` in Foxglove for the bounding box.

- **Pass** (knob detected, reasonably stable) → Phase 2.
- **Fail** (nothing / unstable / wrong object) → branch: verify `detection.pt`
  actually has a class literally named `knob` matching `YOLO_TARGET=knob`. **This
  is the biggest risk.** If the model does not know `knob`, scope expands:
  relabel/retrain, or fall back to detecting the door body or an ArUco marker.
  Surface and decide then; do not proceed on flaky detection.

### Phase 2 — Approach waypoints (provided)

The approach path has been measured (see *Measured WAYPOINTS* above): forward in
`+x` to `(2.0, 0)`, then `+y` to `(2.0, 1.67)`, then `+x` to the knob at
`(2.43, 1.66)`. These `x, y` values go straight into `Task3Mission.WAYPOINTS`;
only the per-point `arrive_dist` tolerances remain to be tuned (Phase 4). If a
fresh `reset_map.sh --pin` shows the path is off, re-measure by driving from
spawn and echoing `/amcl_pose`.

### Phase 3 — Arm bench test (lever press in isolation)

With the rover parked roughly at the intended dock distance from the lever,
run the UNLOCK arm sequence alone and observe:

- Does the **highest** arm pose clear the lever on the forward approach (gripper
  ends up above the lever handle, not jammed into it)?
- Does the **lowest** arm pose press the lever **down** far enough to unlatch the
  door?

This replaces the old forward-poke `knob_poke` semantics with a vertical press.
Tunables: the highest/lowest arm poses (shoulder/elbow angles), the forward-nudge
distance/time that positions the gripper over the lever between raise and lower,
and the hold time at the bottom. Gripper open/close likely does not matter for a
press; default to whatever keeps the end-effector profile best for pushing the
lever. Decide in the implementation plan whether to repurpose `knob_poke` or add
a dedicated `knob_press` routine.

### Phase 4 — Drive + dock loop (DRIVE_WP → VISUAL_DOCK → OBSERVE)

Full `run_task3.sh`, tuning from the `[Task3]` log:

- DRIVE_WP: confirm the rover follows `WAYPOINTS` and arrives near the knob;
  tune `arrive_dist` per point and the stuck-guard. Confirm `/amcl_pose` stays
  valid along the whole approach (no freeze like on the bridge).
- VISUAL_DOCK: `APPROACH_STOP_DIST` (currently `0.45`, which sits at depth
  saturation; the knob is higher than a bear so depth may read oddly — fallback
  is to stop on bbox `area` / `bottom_frac`), `ALIGN_PX` (centering), and
  `COMMIT_DOCK_DIST` (near-distance loss → "docked" handling). Plus the
  rotate-to-acquire fallback if the knob is not framed at the last waypoint.
- OBSERVE: `OBSERVE_SECONDS` (`5.5`, already > 5 s with margin; likely fine).

Goal: the rover reliably stops centered at a distance where the Phase-3 arm
reach can touch the knob.

### Phase 5 — Full mission: lever press + drive through (UNLOCK → CLEAR)

End-to-end run. Confirm UNLOCK lowers the arm onto the lever and unlatches the
door, then raises/retracts the arm clear of the gate. Tune `CLEAR_PUSH_SEC` and
`CLEAR_SPEED` so the body drives straight through the now-unlatched door (pushing
the gate aside as it passes) and clears it fully for full points.

### Phase 6 — Reliability + commit

- 3 back-to-back runs (`reset_map.sh --pin && run_task3.sh` ×3) on the demo map.
  Confirm the fixed-location waypoints + dock repeat reliably.
- Mark Task 3 "verified working end-to-end" in CLAUDE.md.
- Commit the new waypoint code + tuned params, **plus** the currently-untracked
  `docker-compose_perception_unity.yml` and the modified `start_stack.sh` that
  the knob container depends on, using the `feat(Final_Project):` commit style.

## Top risks & contingencies

1. **`knob` not detectable by the model** — biggest risk; checked first and
   cheaply in Phase 1. Contingency: verify the class name; if absent, relabel/
   retrain, or fall back to door-body detection or an ArUco marker. Potential
   scope expansion.
2. **`/amcl_pose` drift / not pinned** — waypoints are in the pinned frame, so a
   missed `reset_map.sh --pin` (or a floating map frame) sends DRIVE_WP to the
   wrong place. Contingency: the Phase-0 spawn ≈ `(0,0,0)` check is mandatory
   before trusting waypoints; the empty-`WAYPOINTS` vision-only fallback path
   stays available for debugging.
3. **Lever press misses or under-presses** — the raised arm catches the lever on
   approach, or the lowest pose doesn't push it down far enough to unlatch.
   Contingency: tune the highest/lowest poses and the forward-nudge distance in
   Phase 3; if reach is short, add a small extra forward body nudge while the arm
   is down.
4. **Depth unreliable at knob height** — `APPROACH_STOP_DIST` at depth
   saturation. Contingency: switch the VISUAL_DOCK stop criterion to bbox `area`
   / `bottom_frac`.

## Success criteria

- `run_task3.sh` completes DRIVE_WP→VISUAL_DOCK→OBSERVE→UNLOCK→CLEAR→DONE
  hands-free.
- OBSERVE holds ≥ 5 s on the knob (Locate & Observe points).
- The door unlocks and the rover drives through it.
- Repeatable across 3 consecutive runs on the demo map.
