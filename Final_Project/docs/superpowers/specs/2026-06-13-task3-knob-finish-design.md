# Task 3 (door knob) — Finish & Verify Design

**Date:** 2026-06-13
**Status:** Approved design, ready for implementation plan
**Topic:** Bring the already-written Task 3 mission up against the live Unity sim and field-tune it to reliably score.

## Problem

Task 3's code is structurally complete and committed (`dbb00c3`), and every
dependency is wired up:

| Dependency | Status |
| --- | --- |
| `Task3Mission` state machine (SEARCH→APPROACH→OBSERVE→UNLOCK→CLEAR→DONE) | committed |
| `data_processor.get_knob_target_info()` → `/yolo/target_info_knob` | committed |
| `ros_communicator` knob subscription + `publish_car_control` / `publish_raw_car_control` | committed |
| `arm_controller_2D.knob_poke()` | committed |
| YOLO knob container (`YOLO_TARGET=knob`, outputs remapped) in `docker-compose_perception_unity.yml` | present (file untracked) |
| `run_task3.sh` + `setup.py` `task3_auto` entry point | committed |

But **Task 3 has never been run against the sim even once.** CLAUDE.md marks
only Task 2 as "verified working end-to-end"; Task 3 is merely "implemented."
Several parameters and arm poses are explicitly flagged "must tune in sim."

So "finish" does **not** mean writing missing code. It means: bring each layer
up against the live sim, confirm it works, and field-tune the parameters so the
mission reliably scores.

## Door mechanic (confirmed)

The Task 3 door is **locked**: the arm must interact with the knob (poke/turn)
to unlock it, **then** the body pushes the door open. The existing
`UNLOCK → CLEAR` design is therefore the correct shape and is kept as-is. The
exact Unity trigger for the unlock (gripper contact vs. press/hold vs.
gripper-close) is unknown and is discovered empirically in Phase 2.

## Approach: Hybrid verify-then-tune (Approach C)

Smoke-test the single highest-risk unknown first (does the model actually detect
`knob`?), because that failure invalidates everything downstream and could force
a perception-strategy change. Once detection is confirmed, collapse the rest
into a tight full-mission tuning loop, since arm poses, stop distance, and push
time are all best tuned from real end-to-end runs anyway.

Rejected alternatives:

- **A — Staged bottom-up bring-up of every layer in isolation:** maximally
  de-risking but many separate manual steps; most of the per-layer isolation
  buys nothing once detection is known good.
- **B — Full-mission first, fix from the log:** fastest when things mostly work,
  but on a cold start perception + arm + door + tuning can fail tangled
  together with no clean signal for which layer broke.

## Roles & constraints

- **The human drives the sim.** Unity runs as a host GUI binary on display `:20`
  (Chrome Remote Desktop), AI mode, RosBridge port 9091. Claude cannot launch or
  observe the sim directly.
- **Claude edits params/poses between runs** and reads the human's `[Task3]`
  state-log output and scene reports.
- **Tuning surface is two files only:** the constants at the top of
  `Task3Mission.__init__` (`task3_mission.py`) and the `KNOB_*` constants in
  `ArmController.__init__` (`arm_controller_2D.py`).
- **Task 3 is vision-only** — its state machine consumes no `/amcl_pose` or
  Nav2. No pose-frame correctness is required by the mission logic itself.

## Standard per-run loop

Every iteration (including the Phase 1 perception echo and the Phase 2 arm bench
test, which need a fresh FINAL PROJECT scene with the door present) is wrapped by
the reset ritual:

```bash
~/Desktop/Robot-navigation-projects/Final_Project/tools/reset_map.sh --pin
~/Desktop/Robot-navigation-projects/Final_Project/workspace/pros/pros_car/run_task3.sh
```

`reset_map.sh --pin` forces a **fresh random scene each retry** and re-origins
odom + re-anchors SLAM to a clean state. Task 3's logic does not consume the
pinned pose, but running the pin is harmless and keeps the operational ritual
identical to Task 1/2.

## Phases

### Phase 0 — Pre-flight (~2 min)

- `start_stack.sh` up; Unity in FINAL PROJECT + AI mode + 9091 "Connected".
- Confirm `kylefp-yolo-knob` is running (`docker ps --filter name=kylefp-`).

### Phase 1 — Perception smoke test (gates everything)

With the rover facing the door (after `reset_map.sh --pin`), inside any container
on the net:

```bash
ros2 topic echo /yolo/target_info_knob   # expect [1, dist, dx, area, bottom] when knob in frame
```

Also eyeball `/yolo/detection_knob/compressed` in Foxglove for the bounding box.

- **Pass** (knob detected, reasonably stable) → Phase 2.
- **Fail** (nothing / unstable / wrong object) → branch: verify `detection.pt`
  actually has a class literally named `knob` matching `YOLO_TARGET=knob`. **This
  is the biggest risk.** If the model does not know `knob`, finishing expands in
  scope: relabel/retrain the model, or fall back to detecting the door body or an
  ArUco marker on the knob. Surface and decide then; do not proceed on a flaky
  detection.

### Phase 2 — Arm bench test (`knob_poke` in isolation)

With the rover parked roughly at the intended stop distance from the knob,
trigger `knob_poke()` alone (via the `robot_control` menu UI or a one-off call)
and observe:

- Does `KNOB_REACH_POSE` place the gripper **at knob height and touching the
  knob**?
- Does Unity register the **unlock** (door becomes openable)?

Tune `KNOB_REACH_POSE` `[shoulder, elbow, gripper]`, `KNOB_HOLD_WAIT`, and the
`close_gripper` flag. **Risk:** the 2-DOF arm (shoulder/elbow/gripper) cannot
axially *rotate* a knob, so the Unity unlock is most likely contact- or
press-triggered. Discover the actual trigger here and shape `knob_poke` to match
— this may require a small forward body nudge during the poke to make contact.

### Phase 3 — Drive loop (SEARCH → APPROACH → OBSERVE)

Run the full `run_task3.sh` and tune from the `[Task3]` log:

- SEARCH: rotation direction and `SEARCH_CONFIRM` frame count.
- APPROACH: `APPROACH_STOP_DIST` (currently `0.45`, which sits right at depth
  saturation; the knob is higher than a bear so depth may read oddly — fallback
  is to stop on bbox `area` / `bottom_frac` instead of depth), `ALIGN_PX`
  (centering tolerance), and `COMMIT_DOCK_DIST` (near-distance loss → "docked"
  handling).
- OBSERVE: `OBSERVE_SECONDS` (`5.5`, already > 5 s with margin; likely fine).

Goal: the rover reliably stops centered at a distance where the Phase-2 arm
reach can actually touch the knob.

### Phase 4 — Full mission + door push (UNLOCK → CLEAR)

End-to-end run. Tune `CLEAR_PUSH_SEC` and `CLEAR_SPEED` so the body pushes the
now-unlocked door fully open and drives through. Confirm the arm retracts to
`KNOB_RETRACT_POSE` before the push so it does not snag the door frame.

### Phase 5 — Reliability + commit

- 3 back-to-back runs (`reset_map.sh --pin && run_task3.sh` ×3) on fresh random
  maps. Door/knob location varies per map → confirm the SEARCH rotation finds the
  door from spawn each time.
- Mark Task 3 "verified working end-to-end" in CLAUDE.md.
- Commit the tuned params, **plus** the currently-untracked
  `docker-compose_perception_unity.yml` and the modified `start_stack.sh` that
  the knob container depends on, using the `feat(Final_Project):` commit style.

## Top risks & contingencies

1. **`knob` not detectable by the model** — biggest risk; checked first and
   cheaply in Phase 1. Contingency: verify the class name; if absent, relabel/
   retrain, or fall back to door-body detection or an ArUco marker. This is a
   potential scope expansion.
2. **Arm cannot trigger the unlock** — 2-DOF arm can't rotate a knob. Contingency:
   discover the Unity trigger mechanic in Phase 2 and adapt `knob_poke` (contact
   nudge, hold, or gripper-close) accordingly.
3. **Depth unreliable at knob height** — `APPROACH_STOP_DIST` at depth saturation.
   Contingency: switch the APPROACH stop criterion to bbox `area` / `bottom_frac`.

## Success criteria

- `run_task3.sh` completes SEARCH→APPROACH→OBSERVE→UNLOCK→CLEAR→DONE hands-free.
- OBSERVE holds ≥ 5 s on the knob (Locate & Observe points).
- The door unlocks and the rover drives through it.
- Repeatable across 3 consecutive fresh-map runs.
