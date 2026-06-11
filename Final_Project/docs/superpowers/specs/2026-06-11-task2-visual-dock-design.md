# Task 2 — Walk-then-Visual-Dock Hybrid (DESIGN / DEFERRED)

**Status:** Approved design, **deferred**. Implement *after* the persistent-map /
localization work (that spec is the prerequisite — it removes the per-session
re-measure burden that this design otherwise inherits).

**Date:** 2026-06-11

## Problem

Task 2 docks in front of the bridge using a hand-measured absolute pose
(`DOCK_X/Y/YAW`) read from `/amcl_pose`, then commits to a straight `CLIMB`. Two
failures observed:

1. **Wrong heading at commit** — at the docked pose the bridge appeared at
   `bridge_info.delta_x ≈ +307` (hard right of frame), i.e. the robot was *not*
   pointed up the bridge, so `CLIMB` drove off-axis.
2. **Frame fragility** — `/amcl_pose` comes from online SLAM in `mode: mapping`
   with no loaded map, so the `map` origin re-initialises every session. The same
   physical dock spot read `(2.57, -2.77) yaw≈-179°` one session and
   `(-3.1, -4.3) yaw≈-105°` the next (a ~5.9 m / ~74° rigid frame shift). Absolute
   dock coordinates are only valid within the session they were measured.

This design fixes (1) directly and mitigates (2) for the heading. The persistent-map
spec addresses (2) for position.

## Approach (chosen)

Keep pose-based nav for **coarse** position + heading, then add a **vision phase**
that refines heading against the actual bridge (camera frame → independent of the
`map` origin). Square-up is realised as **center + gate**, not active lateral
servoing (a differential-drive robot rotating in place can null `delta_x` but cannot
remove perspective `symmetry`, which needs lateral translation).

## Section 1 — State-machine structure

`BRIDGE_APPROACH` keeps its phased structure; a vision phase is inserted **after**
the pose phases:

```
BRIDGE_APPROACH
  DRIVE_DOCK    (pose) → drive to medium-range DOCK_X/Y          [unchanged controller]
  YAW_ALIGN     (pose) → coarse heading to DOCK_YAW_RAD          [loosened to ~8°: get bridge in frame]
  VISUAL_DOCK   (NEW)  → center on bridge + gate on square-up, else search → fallback
→ CLIMB  [unchanged logic; CLIMB_DISTANCE re-measured for the backed-off start]
→ GRIP → RETURN → DONE
```

Pose `DOCK_YAW` is the **safety baseline**: YAW_ALIGN already parks at it, so the
fallback path is simply "stop refining and commit." Vision is best-effort refinement
layered on top — it can only help, never strand the mission.

## Section 2 — VISUAL_DOCK control law

Reads `bridge_info = [found, delta_x, area_frac, centroid_y_frac, bottom_edge_dx,
symmetry, aspect_ratio]`, each smoothed with a **rolling median** over the last
`VDOCK_SMOOTH_N` (~5) frames to suppress the close-range noise.

- **CENTER (active):** rotate in place proportional to smoothed `delta_x` until
  `|delta_x| ≤ VDOCK_CENTER_PX` for `VDOCK_CONFIRM` frames. Rotation-direction sign
  is a one-line tunable (`VDOCK_ROT_SIGN`) — verified in-sim, since camera-sign
  bugs are easy to introduce.
- **GATE (passive commit check):** once centered, commit to `CLIMB` **only if**
  `|symmetry| ≤ VDOCK_SYM_TOL` **and** `aspect_ratio ≥ VDOCK_ASPECT_MIN` (truly
  facing the ramp head-on, not a side wall: aspect ~0.3 = wall, ~≥0.7 = ramp). We do
  **not** servo symmetry by translating — if it fails, the position is off and we
  hand off to fallback, we do not wander near the ramp foot.

## Section 3 — Fallback, tunables, what changes

- **Gate fails / bridge lost → `ROTATE_SEARCH`:** sweep ±`VDOCK_SEARCH_DEG` to
  reacquire a well-formed bridge (found + high aspect). Reacquired & gated → `CLIMB`.
  Sweep exhausted or `VISUAL_DOCK_TIMEOUT` → **fall back** to the already-aligned
  pose `DOCK_YAW` and climb anyway.
- **New tunables** (top of `__init__`, grouped): `VDOCK_SMOOTH_N`, `VDOCK_CENTER_PX`,
  `VDOCK_CENTER_GAIN`, `VDOCK_CONFIRM`, `VDOCK_SYM_TOL`, `VDOCK_ASPECT_MIN`,
  `VDOCK_SEARCH_DEG`, `VISUAL_DOCK_TIMEOUT`, `VDOCK_ROT_SIGN`.
- **Re-measure:** `DOCK_X/Y` move back to medium range (~1–1.5 m from the ramp
  foot); `CLIMB_DISTANCE` grows to cover the gap + bridge. Both stay in-sim tunables.
- **Unchanged:** DRIVE_DOCK controller (the 0.10 arrive-dist tightening + near-zone
  "drive straight, ease steering" fix), CLIMB/GRIP/RETURN, the `_norm_angle` yaw
  handling.

## Testing

- In-sim only (no unit harness for the mission state machine). Validate per phase via
  the `[Task2]` log: bridge centered (`delta_x → 0`), gate values (`symmetry`,
  `aspect_ratio`) at commit, and that `CLIMB` now starts pointed up the bridge.
- Fallback: force bridge out of frame at the dock and confirm rotate-search →
  pose-yaw commit fires instead of stalling.

## Dependencies

- `/yolo/bridge_info` (segment_detect.py) — already live via `start_stack.sh`.
- Persistent-map / localization spec — prerequisite for reusable `DOCK_X/Y`.
