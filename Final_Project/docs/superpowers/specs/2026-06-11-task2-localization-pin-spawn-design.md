# Task 2 — Reproducible Localization by Pinning the Map Origin to Spawn (DESIGN)

**Status:** Approved design, ready for implementation planning.

**Date:** 2026-06-11

## Problem

Task 2 navigates to a hand-measured absolute dock pose read from `/amcl_pose`. Those
coordinates are not reproducible across sessions: the same physical dock spot read
`(2.57, -2.77) yaw≈-179°` one session and `(-3.1, -4.3) yaw≈-105°` the next — a
~5.9 m / ~74° rigid shift of the whole `map` frame. Every run therefore requires
re-measuring the dock pose, and a stale pose sends `CLIMB` off-axis.

## Root cause

The world is fully deterministic (verified in-sim: bridge, road, walls, and even the
bears are identical every load), and the Unity **restart button always returns the
car to the same physical spawn**. The instability is purely in the frame anchor:

`slam_toolbox` runs in `mode: mapping` with no loaded map (`demo/slam_params.yaml`,
launched via compose → `demo/slam.xml` → `online_sync_launch.py`). On its first scan
it anchors the `map` frame to the **current odometry pose**. Wheel odometry keeps
accumulating across a Unity restart (teleporting the car back to spawn does not zero
the encoders), so each SLAM init anchors `map` to a different odom value. Same
physical spawn → different `map` coordinates each session.

## Goal

Make `/amcl_pose` reproducible across sessions so a once-measured dock pose stays
valid, **without** maintaining a serialized-map artifact.

## Approach (chosen)

Pin the `map` origin to the fixed spawn. Because the spawn is deterministic, forcing
SLAM's first scan to a constant `map` pose makes `spawn ≡ map origin` every session,
so the entire static world (bridge, dock) gets reproducible coordinates. Stays in
`mode: mapping`; no saved map.

(Runner-up, deferred: serialized map + `mode: localization`. It additionally removes
within-run loop-closure drift, at the cost of building/committing/maintaining a map
file. Kept as the escalation path if drift proves to matter.)

## Section 1 — Config change

In `demo/slam_params.yaml` (the file `online_sync_launch.py` actually loads),
uncomment and set:

```yaml
map_start_pose: [0.0, 0.0, 0.0]
```

This forces SLAM's first scan to `map (0,0,0)` regardless of accumulated odometry.
With the car always physically at the spawn at SLAM init, **spawn ≡ map origin
(0,0,0)** every session, and the deterministic world's coordinates become
reproducible. `mode` stays `mapping`; `map_file_name` stays unset.

## Section 2 — Operational procedure

The pin only holds if SLAM initializes while the car is at the spawn. The Unity
restart guarantees the spawn, so the canonical per-run reset ritual is:

1. **Unity restart** → car returns to the fixed spawn.
2. **`reset_map.sh --slam`** → SLAM restarts fresh with the car at spawn → origin
   pinned at `(0,0,0)`.
3. Run the Task-2 mission.

Within a single SLAM session, SLAM is never restarted; a fresh, correctly-pinned
frame comes only from this reset-at-spawn sequence. Restarting SLAM while the car is
*not* at spawn (or a mid-session Unity restart without a SLAM reset) re-floats or
desyncs the frame and must be avoided. This ritual is documented in
`Final_Project/CLAUDE.md`.

## Section 3 — Verification, re-measure, caveats

**Verify the pin (acceptance test):**
- Echo `/amcl_pose` at the spawn across two independent reset cycles → both read
  `≈(0, 0, 0)`.
- Drive to the bridge, record the dock pose; reset (ritual above); drive again →
  the dock pose reads the same within tolerance (target: < ~0.1 m / few °).
- That test is the proof that localization is reproducible.

**Re-measure the dock once:** the current `DOCK_X/Y/DOCK_YAW_RAD` in
`task2_mission.py` were captured in an old floating frame. After the pin is verified,
measure the dock pose one final time in the pinned `(0,0,0)` frame and update those
constants. They are then valid across all future sessions.

**Residual caveat:** mapping mode still builds the map live, so loop-closure may nudge
coordinates slightly mid-run. For a short Task-2 run this is expected to be small. If
it ever causes a failure, escalate to the serialized-map + `mode: localization`
approach, which optimizes a fixed map offline and removes this drift.

**Fallback if `map_start_pose` is not honored:** some `slam_toolbox` builds apply
`map_start_pose` only when continuing a *loaded* map, not for a fresh map. If the
acceptance test shows the spawn not reading `(0,0,0)`, plan B is to zero odometry at
spawn (restart the odom/driver node, or otherwise reset the odom source) immediately
before SLAM init — same net effect: spawn anchors the frame at a constant. Resolved
empirically during implementation.

## Scope / out of scope

- **In scope:** `slam_params.yaml` change, the reset ritual documented in CLAUDE.md,
  the verification test, and the one-time dock re-measure + constant update.
- **Out of scope:** the visual-dock hybrid (separate approved spec,
  `2026-06-11-task2-visual-dock-design.md`); serialized-map/localization mode (kept as
  the escalation path only).

## Testing

In-sim, no unit harness. The acceptance test in Section 3 (spawn reads `(0,0,0)`;
dock pose stable across resets) is the pass/fail criterion. Secondary check: run the
existing Task-2 mission end-to-end after the re-measure and confirm `CLIMB` starts
pointed up the bridge.
