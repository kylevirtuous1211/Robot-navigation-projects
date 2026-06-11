"""
Task2Mission — Final Project Task 2 自動任務 (對準上橋 + 推土機鏟取橋頂的熊 + 返航)
=======================================================================

關鍵前提 (場景保證)：
  **橋頂正中央一定有一隻熊。** 所以不需要視覺搜尋/觀察熊 —— 只要「對準橋面正中、把開著的
  推土機鏟爪降下、直直開上橋」，熊就會被鏟進爪中，到頂後關爪夾起即可。

策略 (無 IMU；地圖種子固定)：
  視覺伺服 (road+bridge segmentation) 在貼近坡腳時 area/sym/aspect/dx 都太不穩,且地面其他
  熊無法與「橋上的目標熊」分辨 → 棄用視覺伺服。改用「位姿式直接導航」：場景的最佳 docking
  pose 已在 sim 內實測,直接以 /amcl_pose 駕到該點並對齊 yaw,然後降爪直開上橋 (兩隻熊都會
  被籠住)。

完整流程 (對應計分項目)：

  BRIDGE_APPROACH → 兩階段:
                    DRIVE     — 用 RETURN-style controller 開到 (DOCK_X, DOCK_Y),計算方位
                                角差,大則原地轉、小則 arc 前進。dist < DOCK_ARRIVE_DIST 連續
                                N 幀 → YAW_ALIGN。
                    YAW_ALIGN — 原地轉到 yaw = DOCK_YAW_RAD (10° 容差) → CLIMB。
  CLIMB         → 先把開著的推土機鏟爪降下 (scoop_pose, 等於把熊「關」在爪裡的籠子)，
                  固定前推、依 bridge dx (若有) 微修置中、直直開上橋；用 /amcl_pose 量行進距離。
  GRIP          → 到橋頂：小幅前頂(抗坡面後滑、把熊壓在爪中) + 關爪夾起 (scoop_grab)。
  RETURN        → 位姿式直線返航回起點 → 放下 bear (Recovery)。
  DONE          → 停車結束。

重用既有元件：
  - 差速前進/轉向   : ros_communicator.publish_raw_car_control / publish_car_control
  - 橋面導引        : data_processor.get_bridge_info / get_road_info (segment_detect 發布)
  - 鏟取/夾取        : arm_controller.scoop_pose() + scoop_grab() + scoop_release()
  - 起點/行進距離     : ros_communicator.get_latest_amcl_pose (由 tf_to_amcl_pose 提供)

⚠️ 參數需在 Unity 場景內實跑微調。日誌刻意「很吵」，方便快速對到正確門檻值。
   注意：此版本不做 Locate & Observe 的 5 秒停留 (改用「降爪直開鏟取」)；若 Task 2 仍需該
   10 分，可在到頂後、關爪前插入一段停留 (爪已降下=已籠住熊，停留期間不怕滾走)。
"""

import threading
import time
import math

from pros_car_py.nav2_utils import calculate_angle_point, cal_distance


class Task2Mission:
    # ---- 狀態 ----
    BRIDGE_APPROACH = "BRIDGE_APPROACH"
    CLIMB = "CLIMB"
    GRIP = "GRIP"
    RETURN = "RETURN"
    DONE = "DONE"

    def __init__(
        self,
        ros_communicator,
        data_processor,
        nav_processing,
        car_controller,
        arm_controller,
    ):
        self.ros_communicator = ros_communicator
        self.data_processor = data_processor
        self.nav_processing = nav_processing
        self.car_controller = car_controller
        self.arm_controller = arm_controller

        # ====================================================================
        # 可調參數 (需依 Unity 場景微調)
        # ====================================================================
        self.TICK = 0.1                  # 控制週期 (s)
        self.DEBUG = True                # 是否印每幀狀態列 (調參時開著)
        self.DBG_EVERY = 5               # 每幾幀印一次狀態列 (~0.5s)

        # ---- BRIDGE_APPROACH：位姿式直接導航 (pose-based) ----
        # 簡化策略:不再做視覺伺服。場景的坡腳位置固定 (per 地圖種子),手動實測一個 docking pose
        # (in front of bridge ramp, 面向坡口),用 Task 1 RETURN 同款 controller 開到那點再對齊 yaw,
        # 然後 CLIMB → 兩隻熊都會被籠住 (一隻在坡腳、一隻在橋頂中央)。
        #
        # 流程:
        #   1) Drive 到 (DOCK_X, DOCK_Y) — 計算方位角差,大則原地轉,小則直行/arc。
        #   2) 到位 (dist < DOCK_ARRIVE_DIST) → align yaw 到 DOCK_YAW_RAD。
        #   3) yaw 對準 (|Δyaw| < DOCK_YAW_TOL) → CLIMB。
        #
        # ── Waypoints 路徑 (順序執行,場景實測;地圖種子固定) ──
        # 從 /amcl_pose 截下的 pose 列表,task 會依序經過每一個 waypoint,最後一個為精準 dock。
        # 中間 waypoint: 只判斷「到位置 < arrive_dist」就切到下一個,不對齊 yaw (避免不必要停留)。
        # 最後 waypoint: 精準對位 (< DOCK_ARRIVE_DIST) + yaw 對齊 (< DOCK_YAW_TOL) → CLIMB。
        #
        # 若 controller 把車開到撞牆,代表 waypoint 之間的直線跨越了不可行地形 — 加新的中繼
        # waypoint (手動把車開到安全的路徑點,echo /amcl_pose,把座標填進這個 list) 即可路由。
        #
        # 格式: (x, y, arrive_dist)  — 最後一個額外含 yaw 對齊
        # 不使用中繼 waypoints — 直接開往實測 dock pose (清空即進 DRIVE_DOCK)。
        self.WAYPOINTS = []
        # 實測最終 docking pose —— 在「釘住的 (0,0,0) spawn frame」量測 (from /amcl_pose;
        # quat z=0.7115656192, w=0.7026196479)。此 frame 由 reset ritual 維持:Unity restart →
        # 重啟 robot_bringup (重置 scan_matcher odom→0) → 重啟 slam (map 重錨到 odom=0),車在 spawn。
        # 跑過 ritual 後此座標跨 session 可重現,不需每次重量。若沒跑 ritual 就直接用,map frame 會浮動、
        # 座標失效 (見 docs/superpowers/specs/2026-06-11-task2-localization-pin-spawn-design.md)。
        self.DOCK_X = 0.9022             # 最終 dock x (含 yaw 對齊)
        self.DOCK_Y = 0.4282             # 最終 dock y
        # yaw = 2*atan2(z, w) = 2*atan2(0.71157, 0.70262) ≈ 1.5834 rad (90.7°)。不在 ±180° 邊界。
        self.DOCK_YAW_RAD = 1.5834           # rad ≈ 90.7°
        # Controller knobs — 三段速度 (FAR / MID / NEAR) 讓終點精準對齊不衝過頭
        self.APPROACH_DRIVE_SPEED = 110.0    # 全速 (dist >= APPROACH_FAR_DIST 時)
        self.APPROACH_TURN_GAIN = 7.0        # 角度 → wheel-diff 比例 (deg → speed)
        self.APPROACH_SPIN_DEG = 15.0        # 方位角差 > 此值 → 原地轉 (略嚴,讓接近時更端正)
        self.APPROACH_FAR_DIST = 1.0         # < 此距離降到 70% (避免衝過頭)
        self.APPROACH_NEAR_DIST = 0.40       # < 此距離再降到 35% (crawl,精準對位)
        self.DOCK_ARRIVE_DIST = 0.20         # 到位距離容差 (m). 0.10 在此幾何下達不到 (go-to-point 近目標時
                                             # bearing 病態,車會畫圈/卡死);位置不需極精準 —— YAW_ALIGN 會精對
                                             # 朝向、CLIMB 期間再依 bridge dx 置中,故 0.20 足矣。
        self.DOCK_ARRIVE_CONFIRM = 4         # 連續 N 幀達標才視為到位 (略增,濾抖動)
        # 卡死偵測 (near-goal stuck guard): 在 STUCK_NEAR_DIST 範圍內,若位置 N 幀內幾乎沒動
        # → 視為實際到位 (可能被小階差/curb 卡住,前進不了那剩下幾公分)。
        self.STUCK_NEAR_DIST = 0.35          # 此距離內才啟動 stuck 偵測。需 > DOCK_ARRIVE_DIST,涵蓋近目標的
                                             #   stall 帶 (實測車會在 ~0.24m 卡住),卡住即視為到位 → YAW_ALIGN。
        self.STUCK_MOVE_TOL = 0.03           # N 幀內位移 < 此值 (m) 視為沒動
        self.STUCK_TICKS = 15                # ~1.5s 沒動 → 觸發
        self.DOCK_YAW_TOL = math.radians(4)  # yaw 對準容差 (~4°,從 10° → 4°)
        self.DOCK_YAW_CONFIRM = 3            # 連續 N 幀達標才切 CLIMB (濾抖動)
        self.BRIDGE_APPROACH_TIMEOUT = 90.0  # 全程逾時保險

        # ---- CLIMB：降鏟爪 + committed 直開上橋 (用 /amcl_pose 量距離) ----
        self.CLIMB_SPEED = 120.0          # 上橋前推輪速
        self.CLIMB_GAIN = 0.25            # 上橋時依橋面 dx 的微修轉向量 (爪已降，少修為宜)
        self.CLIMB_DISTANCE = 2.5         # 行進這麼多 (m) 視為到橋頂中央 (橋長，需量測微調)
        self.CLIMB_TIMEOUT = 30.0         # 上橋保險上限 (s)

        # ---- GRIP：到頂關爪 (含抗後滑前頂) ----
        self.GRIP_PRESS_SPEED = 40.0      # 關爪期間的小幅前頂輪速 (抗坡面後滑、把熊壓在爪中)；
                                          # 若會把車頂過頭，設 0 改純煞停 (靠輪速 0 的馬達保持力)
        self.GRIP_PRESS_SEC = 1.0         # 關爪前先前頂這麼久 (s)

        # ---- RETURN (位姿式直線返航；與 Task 1 相同，跨回橋故 timeout 放寬) ----
        self.RETURN_ARRIVE_DIST = 0.40
        self.RETURN_ARRIVE_CONFIRM = 4
        self.RETURN_TIMEOUT = 150.0
        self.RETURN_SPIN_DEG = 20.0
        self.RETURN_DRIVE_SPEED = 110.0
        self.RETURN_TURN_GAIN = 7
        self.GOTO_FAR_DIST = 1.0

        # ---- 執行緒狀態 ----
        self._thread = None
        self._stop_event = threading.Event()
        self._running = False

        self.state = self.BRIDGE_APPROACH
        self.start_pose = None  # [x, y]，任務起點 (RETURN 目標)
        self.start_yaw = 0.0

    # ==========================================================
    # 對外介面
    # ==========================================================
    def start(self):
        if self._running:
            return
        self._stop_event.clear()
        self.state = self.BRIDGE_APPROACH
        self._thread = threading.Thread(
            target=self._run, args=(self._stop_event,), daemon=True
        )
        self._thread.start()
        self._running = True

    def stop(self):
        if self._running:
            self._stop_event.set()
            self._thread.join(timeout=3.0)
            self._running = False
        self._publish("STOP")

    # ==========================================================
    # 主狀態機
    # ==========================================================
    def _run(self, stop_event):
        print("[Task2] 任務開始 (對準上橋 → 鏟取橋頂的熊 → 返航)。")
        self.start_pose = self._capture_start_pose()
        if self.start_pose is None:
            print("[Task2] ⚠️ 取不到 /amcl_pose，RETURN 將無法返航、CLIMB 距離量測也會失效。")
        else:
            print(f"[Task2] 起點記錄為 {self.start_pose} (yaw={math.degrees(self.start_yaw):.0f}°)")

        approach_deadline = time.time() + self.BRIDGE_APPROACH_TIMEOUT
        climb_start_pose = None
        climb_entry_time = 0.0
        scoop_prepared = False               # CLIMB: 鏟爪只在進 CLIMB 時降一次
        # BRIDGE_APPROACH (pose-based, waypoints) 狀態
        dock_target = [self.DOCK_X, self.DOCK_Y]
        wp_idx = 0                           # 當前 intermediate waypoint index (進 list 之後變 ≥ len → DOCK)
        dock_arrived_streak = 0              # 連續到位幀數 (final dock)
        yaw_aligned_streak = 0               # 連續 yaw 達標幀數
        # near-goal stuck guard
        stuck_anchor_xy = None               # 進入近區後的「卡死」基準位置
        stuck_anchor_tick = -10000           # anchor 設置的 tick (用以計時)
        approach_phase = "DRIVE_WP" if self.WAYPOINTS else "DRIVE_DOCK"
        dock_dbg = 0
        dbg_tick = 0

        while not stop_event.is_set():
            dbg_tick += 1
            # --- 感測讀取 ---
            rinfo = self.data_processor.get_road_info()            # [found,dx,area] or None
            r_found = bool(rinfo and rinfo[0] == 1)
            r_dx = rinfo[1] if rinfo else 0.0

            binfo = self.data_processor.get_bridge_info()          # [found,dx,area,...] or None
            b_found = bool(binfo and binfo[0] == 1)
            b_dx = binfo[1] if binfo else 0.0
            # bear: 只給 debug 印出,不參與導航/commit (image 空間無法分辨「橋頂的」vs「地面其他」)
            tinfo = self.data_processor.get_yolo_target_info()
            t_found = bool(tinfo and tinfo[0] == 1)
            t_dist = tinfo[1] if tinfo else 0.0
            t_dx = tinfo[2] if tinfo else 0.0

            # ---------------- BRIDGE_APPROACH (pose-based: drive to dock pose, align yaw) ----------------
            if self.state == self.BRIDGE_APPROACH:
                pose_msg = self.ros_communicator.get_latest_amcl_pose()
                if pose_msg is None:
                    self._publish("STOP")
                    if time.time() > approach_deadline:
                        self._transition(self.DONE, "BRIDGE_APPROACH 逾時 (無 /amcl_pose)")
                    time.sleep(self.TICK)
                    continue

                p = pose_msg.pose.pose.position
                o = pose_msg.pose.pose.orientation
                car_xy = [p.x, p.y]
                cur_yaw = self._norm_angle(2.0 * math.atan2(o.z, o.w))

                # ---- Phase 1a: DRIVE_WP → 經過中繼 waypoints (依序,不對齊 yaw) ----
                if approach_phase == "DRIVE_WP":
                    wp_x, wp_y, wp_arrive_dist = self.WAYPOINTS[wp_idx]
                    wp_target = [wp_x, wp_y]
                    dist_wp = cal_distance(car_xy, wp_target)
                    if dist_wp < wp_arrive_dist:
                        wp_idx += 1
                        if wp_idx >= len(self.WAYPOINTS):
                            approach_phase = "DRIVE_DOCK"
                            print(f"[Task2] 通過最後中繼點 (dist={dist_wp:.2f}m) → 開往 dock")
                        else:
                            print(f"[Task2] 通過 WP {wp_idx}/{len(self.WAYPOINTS)} "
                                  f"(dist={dist_wp:.2f}m) → WP {wp_idx + 1}")
                    else:
                        ang = calculate_angle_point(o.z, o.w, car_xy, wp_target)
                        if abs(ang) > self.APPROACH_SPIN_DEG:
                            self._publish("COUNTERCLOCKWISE_ROTATION_SLOW" if ang > 0
                                          else "CLOCKWISE_ROTATION_SLOW")
                            action = f"SPIN({ang:+.0f}°)"
                        else:
                            # 中繼段不需降到 crawl,只用 FAR/MID 兩段
                            base = self.APPROACH_DRIVE_SPEED
                            if dist_wp < self.APPROACH_FAR_DIST:
                                base *= 0.70
                            turn = self.APPROACH_TURN_GAIN * ang
                            turn = max(-base, min(base, turn))
                            left = base - turn
                            right = base + turn
                            self.ros_communicator.publish_raw_car_control([left, right, left, right])
                            action = f"ARC(base={base:.0f},turn={turn:+.0f})"
                        dock_dbg += 1
                        if dock_dbg % 10 == 1:
                            print(f"[Task2] WP {wp_idx + 1}/{len(self.WAYPOINTS)} "
                                  f"car=({car_xy[0]:.2f},{car_xy[1]:.2f}) "
                                  f"dist={dist_wp:.2f} ang={ang:+.0f}° → {action}")

                # ---- Phase 1b: DRIVE_DOCK → 到最終 dock 位置 (精準對位) ----
                elif approach_phase == "DRIVE_DOCK":
                    dist = cal_distance(car_xy, dock_target)
                    if dist < self.DOCK_ARRIVE_DIST:
                        dock_arrived_streak += 1
                        self._publish("STOP")
                        if dock_arrived_streak >= self.DOCK_ARRIVE_CONFIRM:
                            approach_phase = "YAW_ALIGN"
                            yaw_aligned_streak = 0
                            print(f"[Task2] 已到 dock 位置 (dist={dist:.2f}m) → 對準 yaw")
                    else:
                        dock_arrived_streak = 0
                        # near-goal stuck guard: 進近區後若 STUCK_TICKS 幀內位移 < STUCK_MOVE_TOL,
                        # 視為實際到位 (被 curb / 小階差卡住,前進不了那剩下幾公分)
                        if dist < self.STUCK_NEAR_DIST:
                            if stuck_anchor_xy is None:
                                stuck_anchor_xy = list(car_xy)
                                stuck_anchor_tick = dbg_tick
                            else:
                                move = cal_distance(car_xy, stuck_anchor_xy)
                                if move > self.STUCK_MOVE_TOL:
                                    # 有在動,重置 anchor
                                    stuck_anchor_xy = list(car_xy)
                                    stuck_anchor_tick = dbg_tick
                                elif (dbg_tick - stuck_anchor_tick) >= self.STUCK_TICKS:
                                    print(f"[Task2] ⚠ DOCK 卡死偵測 (dist={dist:.2f}m, "
                                          f"{self.STUCK_TICKS}f 沒動) → 視為到位 → 對準 yaw")
                                    approach_phase = "YAW_ALIGN"
                                    yaw_aligned_streak = 0
                                    self._publish("STOP")
                                    continue
                        else:
                            stuck_anchor_xy = None
                        ang = calculate_angle_point(o.z, o.w, car_xy, dock_target)
                        near = dist < self.APPROACH_NEAR_DIST
                        # 近區 (< NEAR_DIST):方位角對微小側偏極敏感 (dist→0 時 bearing 會暴衝),
                        # 此時「原地轉」只空轉、不縮短距離 → 一律改為「直行 + 弱轉向」爬進去。
                        if (not near) and abs(ang) > self.APPROACH_SPIN_DEG:
                            self._publish("COUNTERCLOCKWISE_ROTATION_SLOW" if ang > 0
                                          else "CLOCKWISE_ROTATION_SLOW")
                            action = f"SPIN({ang:+.0f}°)"
                        else:
                            # 三段速度: 遠 → 全速; 中 → 70%; 近 → 35% (crawl 精準對位)
                            base = self.APPROACH_DRIVE_SPEED
                            turn = self.APPROACH_TURN_GAIN * ang
                            if near:
                                base *= 0.35
                                turn *= 0.4          # 近區弱化轉向,讓車直直爬進,而非原地畫圈卡住
                                # 再把 turn 夾在 ±0.5*base:確保兩輪都保有前進分量,某輪不會歸零 → 不會原地
                                # 空轉卡死 (實測 ang≈-18° 時 turn 會飽和到 -base、right 輪=0 → 凍住)。
                                turn = max(-0.5 * base, min(0.5 * base, turn))
                            elif dist < self.APPROACH_FAR_DIST:
                                base *= 0.70
                            turn = max(-base, min(base, turn))
                            left = base - turn
                            right = base + turn
                            self.ros_communicator.publish_raw_car_control([left, right, left, right])
                            action = f"ARC(base={base:.0f},turn={turn:+.0f})"
                        dock_dbg += 1
                        if dock_dbg % 10 == 1:
                            print(f"[Task2] DOCK car=({car_xy[0]:.2f},{car_xy[1]:.2f}) "
                                  f"dist={dist:.2f} ang={ang:+.0f}° → {action}")

                # ---- Phase 2: YAW_ALIGN → 對準目標方向 ----
                elif approach_phase == "YAW_ALIGN":
                    # 角度正規化到 [-π,π]:在 ±180° 邊界連續,不會正負跳動 → 防 wiggle。
                    yaw_err = self._norm_angle(self.DOCK_YAW_RAD - cur_yaw)
                    if abs(yaw_err) < self.DOCK_YAW_TOL:
                        yaw_aligned_streak += 1
                        self._publish("STOP")
                        if yaw_aligned_streak >= self.DOCK_YAW_CONFIRM:
                            climb_start_pose = car_xy
                            climb_entry_time = time.time()
                            scoop_prepared = False
                            self._transition(
                                self.CLIMB,
                                f"已對準 yaw (current={math.degrees(cur_yaw):.1f}°, "
                                f"target={math.degrees(self.DOCK_YAW_RAD):.1f}°, "
                                f"err={math.degrees(yaw_err):+.1f}°) → 上橋")
                            continue
                    else:
                        yaw_aligned_streak = 0
                        # yaw_err > 0 = 需逆時針轉 (CCW),< 0 = 順時針 (CW)
                        self._publish("COUNTERCLOCKWISE_ROTATION_SLOW" if yaw_err > 0
                                      else "CLOCKWISE_ROTATION_SLOW")
                    dock_dbg += 1
                    if dock_dbg % 10 == 1:
                        print(f"[Task2] YAW_ALIGN cur={math.degrees(cur_yaw):.1f}° "
                              f"target={math.degrees(self.DOCK_YAW_RAD):.1f}° "
                              f"err={math.degrees(yaw_err):+.1f}°")

                if time.time() > approach_deadline:
                    self._transition(self.DONE, "BRIDGE_APPROACH 逾時")

            # ---------------- CLIMB (降鏟爪 + committed 直開上橋) ----------------
            elif self.state == self.CLIMB:
                # 1) 進 CLIMB 先把開著的推土機鏟爪降到貼地 (阻塞到位)。爪降下=把橋頂的熊
                #    一路「籠住」，開上去就鏟進爪中，途中也不怕熊滾走。
                if not scoop_prepared:
                    self._publish("STOP")
                    self.arm_controller.scoop_pose()   # 阻塞：降臂 + 開爪
                    scoop_prepared = True
                    climb_start_pose = self._current_xy()   # 鏟爪就緒後才起算行進距離
                    climb_entry_time = time.time()
                travelled = (cal_distance(self._current_xy(), climb_start_pose)
                             if (climb_start_pose and self._current_xy()) else 0.0)
                # 2) 到頂 (行進夠遠) 或逾時 → 關爪。
                if travelled >= self.CLIMB_DISTANCE:
                    self._publish("STOP")
                    self._transition(self.GRIP, f"已到橋頂中央 (行進 {travelled:.2f}m) → 關爪夾起")
                    continue
                if time.time() - climb_entry_time > self.CLIMB_TIMEOUT:
                    self._publish("STOP")
                    self._transition(self.GRIP, "上橋逾時保險 → 關爪夾起")
                    continue
                # 3) 固定前推、依橋面 dx 微修置中，直直開上橋。
                steer = self.CLIMB_GAIN * b_dx if b_found else 0.0
                self._arc(self.CLIMB_SPEED, steer)

            # ---------------- GRIP (抗後滑前頂 + 關爪夾起) ----------------
            elif self.state == self.GRIP:
                # 小幅前頂一下：抗坡面後滑、把熊壓在爪中 (期間輪速保持，scoop_grab 阻塞時仍前頂)。
                if self.GRIP_PRESS_SPEED > 0:
                    print(f"[Task2] GRIP：小幅前頂 {self.GRIP_PRESS_SEC:.1f}s (抗後滑) 後關爪")
                    self.ros_communicator.publish_raw_car_control(
                        [self.GRIP_PRESS_SPEED] * 4
                    )
                    time.sleep(self.GRIP_PRESS_SEC)
                print("[Task2] GRIP：關爪夾住 bear + 抬起搬運")
                self.arm_controller.scoop_grab()   # 阻塞：關爪 + 等黏合 + 抬起 (期間輪速維持上面的前頂)
                self._publish("STOP")
                if self.start_pose is not None:
                    print(f"[Task2] RETURN 目標(起點) = {self.start_pose}")
                self._return_entry_time = time.time()
                self._arrive_streak = 0
                self._return_dbg = 0
                self._transition(self.RETURN, "夾取完成 → 位姿式返航 (跨回橋)")

            # ---------------- RETURN (= Task 1 邏輯) ----------------
            elif self.state == self.RETURN:
                pose_msg = self.ros_communicator.get_latest_amcl_pose()
                if self.start_pose is None or pose_msg is None:
                    self._publish("STOP")
                    if self.start_pose is None:
                        self._transition(self.DONE, "無起點，結束")
                    time.sleep(self.TICK)
                    continue
                if time.time() - self._return_entry_time > self.RETURN_TIMEOUT:
                    self._publish("STOP")
                    self._transition(self.DONE, "返航逾時")
                    continue

                p = pose_msg.pose.pose.position
                o = pose_msg.pose.pose.orientation
                car = [p.x, p.y]
                dist = cal_distance(car, self.start_pose)

                if dist < self.RETURN_ARRIVE_DIST:
                    self._arrive_streak += 1
                    self._publish("STOP")
                    if self._arrive_streak >= self.RETURN_ARRIVE_CONFIRM:
                        print(f"[Task2] 已穩定到站 (dist={dist:.2f}) → 放下 bear")
                        self.arm_controller.scoop_release()
                        self._transition(self.DONE, "已放下 bear (Recovery 完成)")
                    time.sleep(self.TICK)
                    continue
                self._arrive_streak = 0

                ang = calculate_angle_point(o.z, o.w, car, self.start_pose)
                if abs(ang) > self.RETURN_SPIN_DEG:
                    action = ("COUNTERCLOCKWISE_ROTATION_SLOW" if ang > 0
                              else "CLOCKWISE_ROTATION_SLOW")
                    self._publish(action)
                else:
                    base = self.RETURN_DRIVE_SPEED
                    if dist < self.GOTO_FAR_DIST:
                        base *= 0.7
                    turn = self.RETURN_TURN_GAIN * ang
                    turn = max(-base, min(base, turn))
                    left = base - turn
                    right = base + turn
                    self.ros_communicator.publish_raw_car_control([left, right, left, right])
                    action = f"ARC(base={base:.0f},turn={turn:+.0f})"

                self._return_dbg += 1
                if self._return_dbg % 10 == 1:
                    print(f"[Task2] RETURN car=({car[0]:.2f},{car[1]:.2f}) "
                          f"dist={dist:.2f} ang={ang:+.0f}° → {action}")

            # ---------------- DONE ----------------
            elif self.state == self.DONE:
                self._publish("STOP")
                break

            self._dbg_line(dbg_tick, r_found, r_dx, b_found, b_dx, t_found, t_dist, t_dx)
            time.sleep(self.TICK)

        self._publish("STOP")
        print("[Task2] 任務執行緒結束。")

    # ==========================================================
    # 子流程 / 工具
    # ==========================================================
    def _arc(self, base, steer):
        """差速 arc 前進：steer>0 右轉(往 +dx)、<0 左轉。clamp 到 [-base, base]。
        輪序 [rear_L, rear_R, front_L, front_R]。"""
        steer = max(-base, min(base, steer))
        left = base + steer
        right = base - steer
        self.ros_communicator.publish_raw_car_control([left, right, left, right])

    @staticmethod
    def _norm_angle(a):
        """把角度正規化到 [-π, π] — 等價於 atan2(sin,cos),在 ±180° 邊界連續。
        目標 yaw 接近 ±π 時,用此函式算誤差才不會因雜訊正負跳動造成原地抖動 (wiggle)。
        (注意:2*atan2(z,w) 的值域是 (-2π, 2π],也需先正規化才能和 DOCK_YAW_RAD 一致比較。)"""
        return math.atan2(math.sin(a), math.cos(a))

    def _current_xy(self):
        pose_msg = self.ros_communicator.get_latest_amcl_pose()
        if pose_msg is None:
            return None
        p = pose_msg.pose.pose.position
        return [p.x, p.y]

    def _capture_start_pose(self, retries=30, delay=0.2):
        for _ in range(retries):
            pose_msg = self.ros_communicator.get_latest_amcl_pose()
            if pose_msg is not None:
                p = pose_msg.pose.pose.position
                o = pose_msg.pose.pose.orientation
                self.start_yaw = self._norm_angle(2.0 * math.atan2(o.z, o.w))
                return [p.x, p.y]
            time.sleep(delay)
        return None

    def _transition(self, new_state, reason):
        old = self.state
        self.state = new_state
        print(f"[Task2] {old} → {new_state}  ({reason})")

    def _dbg_line(self, tick, r_found, r_dx, b_found, b_dx, t_found, t_dist, t_dx):
        if not self.DEBUG or tick % self.DBG_EVERY != 0:
            return
        print(f"[Task2][{self.state}] road(F={int(r_found)} dx={r_dx:+.0f}) | "
              f"bridge(F={int(b_found)} dx={b_dx:+.0f}) | "
              f"bear(F={int(t_found)} dist={t_dist:.2f} dx={t_dx:+.0f})")

    def _publish(self, action_key):
        self.ros_communicator.publish_car_control(
            action_key, publish_rear=True, publish_front=True
        )


def main(args=None):
    """Headless 進入點：直接跑 Task 2 任務並等到結束。
    用法 (容器內)：ros2 run pros_car_py task2_auto
    需統一 pipeline 已起 (start_stack.sh)：提供 /yolo/bridge_info、/yolo/road_info、/amcl_pose。
    """
    import threading
    import rclpy
    from pros_car_py.ros_communicator import RosCommunicator
    from pros_car_py.data_processor import DataProcessor
    from pros_car_py.nav_processing import Nav2Processing
    from pros_car_py.car_controller import CarController
    from pros_car_py.arm_controller_2D import ArmController

    rclpy.init(args=args)
    ros_communicator = RosCommunicator()
    spin_thread = threading.Thread(
        target=rclpy.spin, args=(ros_communicator,), daemon=True
    )
    spin_thread.start()

    data_processor = DataProcessor(ros_communicator)
    nav_processing = Nav2Processing(ros_communicator, data_processor)
    car_controller = CarController(ros_communicator, nav_processing)
    arm_controller = ArmController(ros_communicator, data_processor)

    mission = Task2Mission(
        ros_communicator,
        data_processor,
        nav_processing,
        car_controller,
        arm_controller,
    )

    print("[task2_auto] 啟動 Task 2 任務 (headless)。Ctrl-C 可中止。")
    mission.start()
    try:
        while mission._running and mission._thread.is_alive():
            mission._thread.join(timeout=0.5)
    except KeyboardInterrupt:
        print("[task2_auto] 收到中止訊號。")
    finally:
        mission.stop()
        ros_communicator.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
