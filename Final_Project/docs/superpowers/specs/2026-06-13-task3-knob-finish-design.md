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
UNLOCK    → open-door script: arm interacts with the knob (knob_poke) to unlock.
CLEAR     → drive the body forward through the now-open door.
DONE      → stop.
```

`WAYPOINTS` empty falls back to the old behavior (rotate-in-place SEARCH →
visual APPROACH), mirroring how Task 2's empty `WAYPOINTS` falls back to its old
dock path. This keeps the vision-only path available for debugging.

## Door mechanic (confirmed)

- The Task 3 door is **locked**: the arm must interact with the knob to unlock
  it, **then** the body pushes the door open. The `UNLOCK → CLEAR` design is the
  correct shape and is kept. The exact Unity unlock trigger (gripper contact vs.
  press/hold vs. gripper-close) is unknown and is discovered empirically in the
  arm bench test.
- The **door is at a fixed location relative to spawn** in the chosen demo map
  (confirmed), exactly like the Task 2 bridge. So waypoints measured once in the
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

### Phase 2 — Measure approach waypoints

After `reset_map.sh --pin`, manually drive the rover (via the `robot_control`
menu UI / Foxglove teleop) from spawn along a clean path to just in front of the
door knob. While driving, `ros2 topic echo /amcl_pose` and record a short
ordered list of `[x, y, arrive_dist]` points into `Task3Mission.WAYPOINTS` — the
same procedure used to measure Task 2's `WAYPOINTS`. The last point should leave
the rover roughly facing the knob at a distance where `VISUAL_DOCK` can acquire
it.

### Phase 3 — Arm bench test (`knob_poke` in isolation)

With the rover parked roughly at the intended dock distance from the knob,
trigger `knob_poke()` alone and observe:

- Does `KNOB_REACH_POSE` place the gripper **at knob height and touching the
  knob**?
- Does Unity register the **unlock** (door becomes openable)?

Tune `KNOB_REACH_POSE` `[shoulder, elbow, gripper]`, `KNOB_HOLD_WAIT`, and the
`close_gripper` flag. **Risk:** the 2-DOF arm cannot axially *rotate* a knob, so
the Unity unlock is most likely contact- or press-triggered. Discover the actual
trigger here and shape `knob_poke` to match (may require a small forward body
nudge during the poke to make contact).

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

### Phase 5 — Full mission + door push (UNLOCK → CLEAR)

End-to-end run. Tune `CLEAR_PUSH_SEC` and `CLEAR_SPEED` so the body pushes the
unlocked door fully open and drives through. Confirm the arm retracts to
`KNOB_RETRACT_POSE` before the push so it does not snag the door frame.

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
3. **Arm cannot trigger the unlock** — 2-DOF arm can't rotate a knob. Contingency:
   discover the Unity trigger mechanic in Phase 3 and adapt `knob_poke` (contact
   nudge, hold, or gripper-close).
4. **Depth unreliable at knob height** — `APPROACH_STOP_DIST` at depth
   saturation. Contingency: switch the VISUAL_DOCK stop criterion to bbox `area`
   / `bottom_frac`.

## Success criteria

- `run_task3.sh` completes DRIVE_WP→VISUAL_DOCK→OBSERVE→UNLOCK→CLEAR→DONE
  hands-free.
- OBSERVE holds ≥ 5 s on the knob (Locate & Observe points).
- The door unlocks and the rover drives through it.
- Repeatable across 3 consecutive runs on the demo map.
