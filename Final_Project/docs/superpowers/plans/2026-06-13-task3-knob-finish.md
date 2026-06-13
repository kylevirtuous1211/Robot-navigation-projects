# Task 3 (door knob) — Finish Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace Task 3's rotate-in-place front-end with a Task-2-style pose-based waypoint drive that coarse-navigates to the door, and rewrite UNLOCK as a lever-press (raise → nudge forward → press down → retract), then field-tune it in the Unity sim.

**Architecture:** Task 3 becomes a pose+vision hybrid. A new `DRIVE_WP` state drives in order through measured `WAYPOINTS` (pinned `(0,0,0)` spawn frame) using `get_latest_amcl_pose()` go-to-point — the exact machinery Task 2's `BRIDGE_APPROACH` uses (`cal_distance` / `calculate_angle_point` + arc-drive + stuck-guard). After the last waypoint it hands off to the existing `SEARCH → APPROACH(visual dock) → OBSERVE` path. `UNLOCK` drives the lever-press arm sequence; `CLEAR` drives straight through.

**Tech Stack:** ROS 2 Humble, Python, `pros_car_py` (`task3_mission.py`, `arm_controller_2D.py`, `nav2_utils.py`), Docker (`kylefp` stack), Unity sim.

**Testing reality:** This is ROS+sim integration code with no existing unit-test harness for missions (the package's `test/` dirs are flake8/pep257/copyright only, matching Task 1/2). So automated verification per code task = host `python3 -m py_compile` (syntax) + in-container `colcon build` (imports/build); behavioral verification = the in-sim runbook phases (Tasks 5–9), executed by the human at the Unity GUI. This mirrors how Task 1/2 were validated.

---

## File Structure

- **Modify** `workspace/pros/pros_car/src/pros_car_py/pros_car_py/arm_controller_2D.py` — add lever-press poses + `knob_raise()` / `knob_press_down()` / `knob_retract()`.
- **Modify** `workspace/pros/pros_car/src/pros_car_py/pros_car_py/task3_mission.py` — add `WAYPOINTS` + `DRIVE_WP` state + approach tunables; start in `DRIVE_WP`; rewrite `UNLOCK` as the lever-press sequence.
- **Modify** `Final_Project/CLAUDE.md` — update the Task 3 description to the waypoint + lever-press flow.
- **Reuse (no change)** `nav2_utils.py` (`cal_distance`, `calculate_angle_point`), `ros_communicator.py` (`get_latest_amcl_pose`, `publish_raw_car_control`).

---

## Task 1: Add lever-press arm primitives

**Files:**
- Modify: `workspace/pros/pros_car/src/pros_car_py/pros_car_py/arm_controller_2D.py` (poses in `__init__` near the existing `KNOB_*` block ~line 54-62; methods after `knob_poke` ~line 205)

- [ ] **Step 1: Add lever-press pose constants in `__init__`**

Find the existing Task 3 block in `__init__` (the `KNOB_REACH_POSE` / `KNOB_RETRACT_POSE` / `KNOB_HOLD_WAIT` lines) and add immediately after it:

```python
        # ---- Task 3 門把「下壓開門」(lever press) 參數 (現場依 Unity 把手幾何微調) ----
        # 「最高」待命姿勢 [shoulder, elbow, gripper]：手臂抬高,爪落在門把上方,前進時不撞把手。
        self.KNOB_RAISE_POSE = [-180.0, 0.0, 90.0]
        # 「最低」下壓姿勢 [shoulder, elbow, gripper]：手臂下降把 lever 壓下去 → 解門閂。
        self.KNOB_PRESS_POSE = [-150.0, -75.0, 90.0]
        # 下壓到底後停留 (s),讓 Unity 的門閂觸發。
        self.KNOB_PRESS_HOLD = 1.0
```

- [ ] **Step 2: Add the three arm methods**

Insert after the existing `knob_poke` method (right before `_execute_grab_sequence`):

```python
    def knob_raise(self):
        """Task 3 lever press 步驟 1：抬手臂到最高待命姿勢 (爪在門把上方,前進不撞把手)。阻塞。"""
        print("🚪 抬手臂到最高 ...")
        self._smooth_move_to(list(self.KNOB_RAISE_POSE), step=5.0, delay=0.1)

    def knob_press_down(self):
        """Task 3 lever press 步驟 3：下降手臂把門把/lever 壓下 → 解門閂,停留 KNOB_PRESS_HOLD。阻塞。"""
        print("🚪 下壓門把 (lever press) ...")
        self._smooth_move_to(list(self.KNOB_PRESS_POSE), step=5.0, delay=0.1)
        time.sleep(self.KNOB_PRESS_HOLD)

    def knob_retract(self):
        """Task 3 lever press 步驟 4：壓完把手臂收回最高,避免擋住車身前進穿門。阻塞。"""
        print("🚪 收回手臂 (抬高) ...")
        self._smooth_move_to(list(self.KNOB_RAISE_POSE), step=5.0, delay=0.1)
```

- [ ] **Step 3: Syntax check (host)**

Run: `python3 -m py_compile workspace/pros/pros_car/src/pros_car_py/pros_car_py/arm_controller_2D.py`
Expected: no output, exit 0 (file compiles).

- [ ] **Step 4: Commit**

```bash
git add Final_Project/workspace/pros/pros_car/src/pros_car_py/pros_car_py/arm_controller_2D.py
git commit -m "feat(Final_Project): Task 3 lever-press arm primitives (raise/press_down/retract)"
```

---

## Task 2: Add WAYPOINTS + DRIVE_WP state to Task3Mission

**Files:**
- Modify: `workspace/pros/pros_car/src/pros_car_py/pros_car_py/task3_mission.py`

- [ ] **Step 1: Import the nav helpers**

At the top of the file, the imports are `import threading` / `import time`. Add below them:

```python
from pros_car_py.nav2_utils import calculate_angle_point, cal_distance
```

- [ ] **Step 2: Add the `DRIVE_WP` state constant**

In the `# ---- 狀態 ----` block (where `SEARCH = "SEARCH"` etc. are defined), add as the first state:

```python
    DRIVE_WP = "DRIVE_WP"
```

- [ ] **Step 3: Add WAYPOINTS + approach tunables in `__init__`**

In `__init__`, right after the `# ---- 搜尋 ----` block constants (after `self.SEARCH_CONFIRM = 3`), add:

```python
        # ---- 位姿式上門 waypoint (pinned (0,0,0) spawn frame; reset_map.sh --pin 後量測) ----
        # 依序開過這些點到門把前,再交給 SEARCH→APPROACH 視覺 dock。格式 [x, y, arrive_dist]。
        # 空 list → 退回舊行為 (原地 SEARCH 旋轉找門把)。
        self.WAYPOINTS = [
            [0.5,  0.0,  0.20],
            [1.0,  0.0,  0.20],
            [1.5,  0.0,  0.20],
            [2.0,  0.0,  0.20],
            [2.0,  0.5,  0.20],
            [2.0,  1.0,  0.20],
            [2.0,  1.5,  0.20],
            [2.0,  1.67, 0.15],
            [2.43, 1.66, 0.10],   # 門把正前方 → 交給視覺 dock
        ]
        # DRIVE_WP 行進參數 (沿用 Task2 BRIDGE_APPROACH 調好的值)
        self.APPROACH_DRIVE_SPEED = 200.0   # 全速 (dist >= APPROACH_FAR_DIST)
        self.APPROACH_TURN_GAIN = 7.0       # 角度 → wheel-diff 比例 (deg → speed)
        self.APPROACH_SPIN_DEG = 15.0       # 方位角差 > 此值 → 原地轉
        self.APPROACH_FAR_DIST = 1.0        # < 此距離降到 70%
        self.STUCK_MOVE_TOL = 0.03          # N 幀內位移 < 此值 (m) 視為沒動
        self.STUCK_TICKS = 15               # ~1.5s 沒動 → 末點 stuck guard 觸發
        self.WP_TIMEOUT = 60.0              # DRIVE_WP 總逾時保險 (s)
```

- [ ] **Step 4: Start in DRIVE_WP when waypoints exist**

In `start()`, change the line `self.state = self.SEARCH` to:

```python
        self.state = self.DRIVE_WP if self.WAYPOINTS else self.SEARCH
```

- [ ] **Step 5: Initialise DRIVE_WP loop vars**

In `_run`, after the existing `dbg_tick = 0` line, add:

```python
        wp_idx = 0
        wp_deadline = time.time() + self.WP_TIMEOUT
        stuck_anchor_xy = None
        stuck_anchor_tick = 0
```

- [ ] **Step 6: Add the DRIVE_WP state handler**

In `_run`, the state machine currently starts with `if self.state == self.SEARCH:`. Insert this block immediately BEFORE that `if`, as the new first branch (change the existing `if self.state == self.SEARCH` to `elif self.state == self.SEARCH`):

```python
            # ---------------- DRIVE_WP (位姿式開到門把前) ----------------
            if self.state == self.DRIVE_WP:
                pose_msg = self.ros_communicator.get_latest_amcl_pose()
                if pose_msg is None:
                    self._publish("STOP")
                    if time.time() > wp_deadline:
                        self._transition(self.SEARCH, "DRIVE_WP 無 /amcl_pose 逾時 → 退回 SEARCH")
                        search_deadline = time.time() + self.SEARCH_TIMEOUT
                        found_streak = 0
                    self._dbg_line(dbg_tick, found, dist, dx)
                    time.sleep(self.TICK)
                    continue

                p = pose_msg.pose.pose.position
                o = pose_msg.pose.pose.orientation
                car_xy = [p.x, p.y]
                wp_x, wp_y, wp_arrive = self.WAYPOINTS[wp_idx]
                wp_target = [wp_x, wp_y]
                dist_wp = cal_distance(car_xy, wp_target)
                last_wp = (wp_idx == len(self.WAYPOINTS) - 1)

                if dist_wp < wp_arrive:
                    wp_idx += 1
                    stuck_anchor_xy = None
                    if wp_idx >= len(self.WAYPOINTS):
                        self._publish("STOP")
                        self._transition(self.SEARCH,
                                         f"通過最後 WP (dist={dist_wp:.2f}m) → 視覺取得門把")
                        search_deadline = time.time() + self.SEARCH_TIMEOUT
                        found_streak = 0
                    else:
                        print(f"[Task3] 通過 WP {wp_idx}/{len(self.WAYPOINTS)} (dist={dist_wp:.2f}m)")
                else:
                    # 末點卡住 guard:連 STUCK_TICKS 幀沒動 → 視為到位 → 交給視覺
                    if last_wp:
                        if (stuck_anchor_xy is None
                                or cal_distance(car_xy, stuck_anchor_xy) > self.STUCK_MOVE_TOL):
                            stuck_anchor_xy = list(car_xy)
                            stuck_anchor_tick = dbg_tick
                        elif dbg_tick - stuck_anchor_tick >= self.STUCK_TICKS:
                            self._publish("STOP")
                            self._transition(self.SEARCH,
                                             f"末 WP 卡住 ({self.STUCK_TICKS}f 沒動) → 視覺取得門把")
                            search_deadline = time.time() + self.SEARCH_TIMEOUT
                            found_streak = 0
                            time.sleep(self.TICK)
                            continue
                    ang = calculate_angle_point(o.z, o.w, car_xy, wp_target)
                    if abs(ang) > self.APPROACH_SPIN_DEG:
                        self._publish("COUNTERCLOCKWISE_ROTATION_SLOW" if ang > 0
                                      else "CLOCKWISE_ROTATION_SLOW")
                    else:
                        base = self.APPROACH_DRIVE_SPEED
                        if dist_wp < self.APPROACH_FAR_DIST:
                            base *= 0.70
                        turn = self.APPROACH_TURN_GAIN * ang
                        turn = max(-base, min(base, turn))
                        left = base - turn
                        right = base + turn
                        self.ros_communicator.publish_raw_car_control([left, right, left, right])
                    if dbg_tick % self.DBG_EVERY == 0:
                        print(f"[Task3][DRIVE_WP] WP {wp_idx + 1}/{len(self.WAYPOINTS)} "
                              f"car=({car_xy[0]:.2f},{car_xy[1]:.2f}) dist={dist_wp:.2f}")

                if time.time() > wp_deadline:
                    self._publish("STOP")
                    self._transition(self.SEARCH, "DRIVE_WP 逾時 → 退回 SEARCH")
                    search_deadline = time.time() + self.SEARCH_TIMEOUT
                    found_streak = 0
```

Note: `found`, `dist`, `dx` referenced in the `pose_msg is None` `_dbg_line` call are already computed at the top of the loop (the `info = self.data_processor.get_knob_target_info()` block runs before the state branches), so they are in scope.

- [ ] **Step 7: Syntax check (host)**

Run: `python3 -m py_compile workspace/pros/pros_car/src/pros_car_py/pros_car_py/task3_mission.py`
Expected: no output, exit 0.

- [ ] **Step 8: Commit**

```bash
git add Final_Project/workspace/pros/pros_car/src/pros_car_py/pros_car_py/task3_mission.py
git commit -m "feat(Final_Project): Task 3 pose-based DRIVE_WP waypoint front-end (reuses Task 2 machinery)"
```

---

## Task 3: Rewrite UNLOCK as the lever-press sequence

**Files:**
- Modify: `workspace/pros/pros_car/src/pros_car_py/pros_car_py/task3_mission.py`

- [ ] **Step 1: Replace the UNLOCK tunable**

In `__init__`, find the `# ---- 解鎖 (手臂門把互動) ----` block with `self.UNLOCK_CLOSE_GRIPPER = True`. Replace that whole block with:

```python
        # ---- 解鎖 (手臂下壓開門 lever press) ----
        # 抬手臂後往門把再前進的時間 (s),讓爪落在 lever 正上方,再下壓。
        self.UNLOCK_NUDGE_SEC = 0.8
```

- [ ] **Step 2: Rewrite the UNLOCK state handler**

Replace the entire `elif self.state == self.UNLOCK:` block with:

```python
            # ---------------- UNLOCK (抬手 → 前進到門把 → 下壓 lever 開門) ----------------
            elif self.state == self.UNLOCK:
                self._publish("STOP")
                print("[Task3] UNLOCK：抬手臂 → 前進到門把 → 下壓 lever 開門")
                # 1. 抬手臂到最高 (爪移到門把上方,前進不撞把手)
                self.arm_controller.knob_raise()
                # 2. 往門把再前進一小段,讓爪落在 lever 正上方
                nudge_start = time.time()
                while time.time() - nudge_start < self.UNLOCK_NUDGE_SEC:
                    if stop_event.is_set():
                        break
                    self._publish("FORWARD_SLOW")
                    time.sleep(self.TICK)
                self._publish("STOP")
                # 3. 下壓 lever → 解門閂
                self.arm_controller.knob_press_down()
                # 4. 收回 (抬高) 避免擋住車身穿門
                self.arm_controller.knob_retract()
                clear_start = time.time()
                self._transition(self.CLEAR, "下壓開門完成 → 車身直行穿門")
```

- [ ] **Step 3: Update the module docstring (optional but keeps it honest)**

At the top of the file, the docstring lists `UNLOCK → 手臂互動：伸臂去碰/壓門把 (arm_controller.knob_poke)`. Change that one line to:

```
  UNLOCK   → 手臂下壓開門：抬手臂到最高 → 前進到門把 → 下壓 lever (arm_controller.knob_press_down) → 收回
```

- [ ] **Step 4: Syntax check (host)**

Run: `python3 -m py_compile workspace/pros/pros_car/src/pros_car_py/pros_car_py/task3_mission.py`
Expected: no output, exit 0.

- [ ] **Step 5: Commit**

```bash
git add Final_Project/workspace/pros/pros_car/src/pros_car_py/pros_car_py/task3_mission.py
git commit -m "feat(Final_Project): Task 3 UNLOCK = lever press (raise->nudge->press_down->retract)"
```

---

## Task 4: In-container build check + docs

**Files:**
- Modify: `Final_Project/CLAUDE.md`

- [ ] **Step 1: Build the package in the container to catch import errors**

This needs the `kylefp` stack up (it pulls the same image `run_task3.sh` uses). Run:

```bash
docker run --rm \
  -v "$HOME/Desktop/Robot-navigation-projects/Final_Project/workspace/pros/pros_car/src:/workspaces/src" \
  ghcr.io/screamlab/pros_car_docker_image:latest \
  bash -lc "cd /workspaces && colcon build --symlink-install --packages-select pros_car_py 2>&1 | tail -20"
```

Expected: `Finished <<< pros_car_py` with no Python import/syntax errors. (If the image must be pulled first, that's fine.)

- [ ] **Step 2: Update the Task 3 line in CLAUDE.md**

Find the `- **Task 3** — door knob:` bullet under "Tasks 2 (bridge) and 3 (door)". Replace it with:

```markdown
- **Task 3** — door knob (lever handle): pose-based **DRIVE_WP** through measured `WAYPOINTS`
  (pinned `(0,0,0)` spawn frame, same machinery as Task 2's BRIDGE_APPROACH) to the door, then
  **SEARCH→APPROACH** visual dock on the knob (`/yolo/target_info_knob`), **OBSERVE** ≥5 s
  (Locate & Observe), **UNLOCK** by lever press (`knob_raise` → forward nudge → `knob_press_down`
  → `knob_retract`), then **CLEAR** by driving straight through the now-open door.
```

- [ ] **Step 3: Commit**

```bash
git add Final_Project/CLAUDE.md
git commit -m "docs(Final_Project): CLAUDE.md Task 3 = waypoint drive + lever-press"
```

---

## Task 5 (sim runbook): Pre-flight + perception smoke test

> Executed by the human at the Unity GUI. Claude reads the output and adjusts code/params.

- [ ] **Step 1: Bring the stack up and pin**

```bash
~/Desktop/Robot-navigation-projects/Final_Project/start_stack.sh
# Unity: FINAL PROJECT + AI mode + RosBridge 9091 "Connected"
~/Desktop/Robot-navigation-projects/Final_Project/tools/reset_map.sh --pin
```

- [ ] **Step 2: Confirm knob container + pinned frame**

```bash
docker ps --filter name=kylefp- --format '{{.Names}}: {{.Status}}'   # kylefp-yolo-knob present
ros2 topic echo /amcl_pose --once                                    # at spawn ≈ (0,0,0)
```

- [ ] **Step 3: Smoke-test knob detection (the gating risk)**

Point the rover at the door, then:

```bash
ros2 topic echo /yolo/target_info_knob   # expect [1, dist, dx, area, bottom] when door in frame
```
Also open `/yolo/detection_knob/compressed` in Foxglove.
- PASS → Task 6.
- FAIL (no/unstable/wrong detection) → STOP and report: verify `detection.pt` has a class literally named `knob`. This may force a perception fallback (relabel/retrain, door-body detection, or ArUco) — a scope change to decide with Claude before continuing.

---

## Task 6 (sim runbook): Arm lever-press bench test

- [ ] **Step 1: Park the rover ~dock distance in front of the lever**

Drive manually (the `robot_control` menu UI / Foxglove teleop) until the gripper would be roughly over the lever.

- [ ] **Step 2: Trigger the lever-press sequence in isolation**

From inside the `pros_car` container, in a Python shell or a scratch one-off, call `knob_raise()`, then move the body forward a touch, then `knob_press_down()`, then `knob_retract()` — or simply run the full mission and watch the UNLOCK phase. Observe:
- Does `KNOB_RAISE_POSE` clear the lever on approach (gripper above it, not jammed)?
- Does `KNOB_PRESS_POSE` push the lever **down** far enough to unlatch?

- [ ] **Step 3: Report angles to tune**

Report what you see; Claude adjusts `KNOB_RAISE_POSE`, `KNOB_PRESS_POSE`, `KNOB_PRESS_HOLD`, and `UNLOCK_NUDGE_SEC` in source, then rebuild via `run_task3.sh`.

---

## Task 7 (sim runbook): Drive + dock loop (DRIVE_WP → SEARCH → APPROACH → OBSERVE)

- [ ] **Step 1: Run the mission**

```bash
~/Desktop/Robot-navigation-projects/Final_Project/tools/reset_map.sh --pin
~/Desktop/Robot-navigation-projects/Final_Project/workspace/pros/pros_car/run_task3.sh
```

- [ ] **Step 2: Watch the `[Task3]` log and report**

- DRIVE_WP: does it follow the 9 waypoints and arrive near the door? Does `/amcl_pose` stay valid the whole way (no freeze)? Report any waypoint it overshoots/undershoots → Claude tunes that point's `arrive_dist` (and `WAYPOINTS` x/y if the path is wrong).
- SEARCH→APPROACH: does it acquire and center the knob and stop at a reachable distance? Report `dist`/`dx` at stop → Claude tunes `APPROACH_STOP_DIST` / `ALIGN_PX` (or switches the stop criterion to bbox `area`/`bottom_frac` if depth is unreliable at knob height).
- OBSERVE: confirm it holds ≥5 s.

---

## Task 8 (sim runbook): Full mission — lever press + drive through

- [ ] **Step 1: Full end-to-end run** (`reset_map.sh --pin && run_task3.sh`).

- [ ] **Step 2: Verify + report**

- UNLOCK: arm lowers onto the lever, door unlatches, arm retracts clear.
- CLEAR: the body drives straight through and clears the door. Report whether `CLEAR_PUSH_SEC` / `CLEAR_SPEED` push far enough → Claude tunes.

---

## Task 9 (sim runbook): Reliability + final commit

- [ ] **Step 1: 3 back-to-back runs** (`reset_map.sh --pin && run_task3.sh` ×3) on the demo map; confirm the waypoints + dock + unlock + drive-through repeat reliably.

- [ ] **Step 2: Mark verified + commit everything**

Claude updates CLAUDE.md to mark Task 3 "verified working end-to-end", then commits the tuned params plus the currently-untracked `docker-compose_perception_unity.yml` and modified `start_stack.sh` (the knob container Task 3 depends on):

```bash
git add Final_Project/CLAUDE.md \
        Final_Project/workspace/pros/pros_app/docker/compose/docker-compose_perception_unity.yml \
        Final_Project/start_stack.sh \
        Final_Project/workspace/pros/pros_car/src/pros_car_py/pros_car_py/task3_mission.py \
        Final_Project/workspace/pros/pros_car/src/pros_car_py/pros_car_py/arm_controller_2D.py
git commit -m "feat(Final_Project): Task 3 verified end-to-end (tuned waypoints + lever-press)"
```

---

## Success criteria

- `run_task3.sh` completes `DRIVE_WP → SEARCH → APPROACH → OBSERVE → UNLOCK → CLEAR → DONE` hands-free.
- OBSERVE holds ≥ 5 s on the knob (Locate & Observe points).
- The lever presses down, the door unlatches, and the rover drives through.
- Repeatable across 3 consecutive runs on the demo map.
