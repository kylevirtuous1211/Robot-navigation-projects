# Task 2 — Reproducible Localization by Re-origining Odometry at Spawn (DESIGN)

**Status:** Approved design, **core mechanism verified live (2026-06-11)**. Ready for
implementation planning (script + docs + dock re-measure).

**Date:** 2026-06-11

## Problem

Task 2 navigates to a hand-measured absolute dock pose read from `/amcl_pose`. Those
coordinates are not reproducible across sessions: the same physical dock spot read
`(2.57,-2.77) yaw≈-179°`, `(-3.1,-4.3) yaw≈-105°`, and (live baseline this session)
`(-4.21,-4.24) yaw≈-58°` — three different rigid anchors of the whole `map` frame at
the same physical spawn. Every run therefore requires re-measuring the dock pose, and
a stale pose sends `CLIMB` off-axis.

## Root cause (verified live)

The world is fully deterministic (bridge, road, walls, even bears identical every
load) and the Unity restart button always returns the car to the same physical spawn.
The instability is purely in the frame anchor:

- Odometry comes from a **laser scan-matcher** — node `scan_matcher`
  (`ros2_laser_scan_matcher`, launched by `demo/robot_bringup_unity.xml`, hosted in
  `kylefp-robot_bringup-1`). There is no wheel/IMU odometry.
- `scan_matcher` integrates pose from node startup and **never resets**. With
  `robot_bringup` up for 11 days, `/odom` had accumulated to `(-3.62, -11.02)`.
- `slam_toolbox` (`mode: mapping`, no loaded map) anchors `map` to the current odom
  pose on its first scan, and while the robot is stationary it applies **no**
  correction (`map→odom` stays ≈identity; scan-match updates only after
  `minimum_travel_distance = 0.5 m`). So `/amcl_pose ≈ /odom` = the giant accumulated
  value, different every session.

**Disproven mechanism:** setting `map_start_pose: [0.0, 0.0, 0.0]` does **not** fix
this. Verified live — after setting it and restarting SLAM, `/amcl_pose` returned the
*identical* polluted value. `slam_toolbox` ignores `map_start_pose` for a fresh map
(it is honored only when continuing a loaded `map_file_name`).

## Goal

Make `/amcl_pose` reproducible across sessions so a once-measured dock pose stays
valid, without maintaining a serialized-map artifact.

## Approach (chosen, verified)

Re-origin the **odometry** to the fixed spawn, so the whole chain anchors at
`(0,0,0)` there. Because the spawn is deterministic, resetting `scan_matcher` while the
car sits at spawn makes `odom origin ≡ spawn`, and restarting SLAM then anchors `map`
onto `odom=0`. Result: `spawn ≡ map origin (0,0,0)` every session; the deterministic
world's coordinates become reproducible. Stays `mode: mapping`; no saved map.

(Runner-up, deferred: serialized map + `mode: localization`. Additionally removes
within-run loop-closure drift, at the cost of building/committing/maintaining a map
file. Escalation path if drift proves to matter.)

## Section 1 — Mechanism (verified live)

The reset, with the car at spawn, in order:

1. **Restart `robot_bringup`** → `scan_matcher` re-origins → `/odom = (0,0,0)`.
   *Verified:* `/odom` → `(1.1e-16, 3.1e-16)` ≈ 0.
2. **Restart `slam`** → `map` re-anchors to `odom=0`.
   *Verified:* `/amcl_pose` → `(2e-7, -1.7e-5)` ≈ 0, identity orientation; holds at
   `~1e-16` at rest with no drift.

`map_start_pose: [0.0, 0.0, 0.0]` is left set in `slam_params.yaml` as a documented
**no-op** (harmless; would only take effect under a future `map_file_name` +
`mode: localization`). The real lever is the odom re-origin above.

## Section 2 — Operational procedure (the reset ritual)

Per-run, the canonical Task-2 reset ritual:

1. **Unity restart** → car returns to the fixed spawn.
2. **Restart `robot_bringup`, then `slam`** (car at spawn) — order matters: odom must
   be zeroed *before* SLAM re-anchors. Encode this in a script
   (e.g. extend `tools/reset_map.sh` with a `--pin-spawn` path or add a dedicated
   `tools/reset_localization.sh`) so it is one command and the order can't be missed.
   Note: `navigation` depends on SLAM; restarting SLAM de-syncs Nav2 costmaps, so the
   script should also restart `navigation` (as `reset_map.sh --slam` already does).
3. Run the Task-2 mission.

Avoid: restarting SLAM while the car is *not* at spawn, or a mid-session Unity restart
without re-running the ritual — either re-floats or desyncs the frame. Documented in
`Final_Project/CLAUDE.md`.

## Section 3 — Verification, re-measure, caveats

**Acceptance test — pin (DONE for spawn):**
- `/amcl_pose` at spawn after the ritual reads `≈(0,0,0)`, stable at rest. ✅ verified.
- *Remaining (needs driving):* drive to the bridge, record the dock pose; re-run the
  ritual; drive again → the dock pose reads the same within tolerance (< ~0.1 m / few
  °). This confirms reproducibility end-to-end and that scan-match drift over the
  Task-2 path is acceptable. To be run by the user with the mission.

**Re-measure the dock once:** the current `DOCK_X/Y/DOCK_YAW_RAD` in `task2_mission.py`
were captured in old floating frames. After the ritual, measure the dock pose one final
time in the pinned `(0,0,0)` frame and update those constants. They are then valid
across all future sessions.

**Residual caveat:** mapping mode builds the map live, so scan-match/loop-closure may
nudge coordinates mid-run. Expected small for a short Task-2 run; the remaining
acceptance test above measures it. If it ever causes a failure, escalate to the
serialized-map + `mode: localization` approach.

## Scope / out of scope

- **In scope:** the reset-ritual script (odom→SLAM→nav, in order), the ritual
  documented in CLAUDE.md, the `slam_params.yaml` comment, and the one-time dock
  re-measure + constant update.
- **Out of scope:** the visual-dock hybrid (separate approved spec,
  `2026-06-11-task2-visual-dock-design.md`); serialized-map/localization mode
  (escalation path only).

## Testing

In-sim, no unit harness. Spawn pin is verified (Section 1/3). The end-to-end dock
reproducibility test (Section 3) is the remaining pass/fail criterion, run with the
mission.
