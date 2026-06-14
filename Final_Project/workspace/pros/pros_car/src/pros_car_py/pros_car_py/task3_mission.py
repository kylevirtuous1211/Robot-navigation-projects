"""
Task3Mission — Final Project Task 3 自動任務 (門把：定位觀察 → 解鎖 → 推開門)
=======================================================================

完整流程 (對應計分項目)：

  SEARCH   → 原地旋轉直到 YOLO 偵測到 knob (門把)
             ※ knob 透過 /yolo/target_info_knob 傳來 (統一 pipeline 中由 YOLO_TARGET=knob 的
               偵測容器 remap 輸出；bear 同時在 /yolo/target_info，兩者互不干擾)。格式與 bear 相同。
  APPROACH → 依 /yolo/target_info 的 delta_x 對準、依 distance 前進靠近門把
  OBSERVE  → 停在門把前並保持靜止 >= 5 秒 (Locate & Observe, 10 pts)
  UNLOCK   → 手臂下壓開門：抬手臂 → 前進到門把 → 下壓 lever (arm_controller.knob_press_down) → 不收回 (維持壓著穿門)
  CLEAR    → 車身直線前推一段，把門整個推開
  DONE     → 停車結束

Task 3 沒有 bear、不需返航 (RETURN)。計分靠 Locate & Observe + 把門推開。

設計上盡量「重用既有元件」：
  - 前進/旋轉指令   : ros_communicator.publish_car_control + ACTION_MAPPINGS
  - 對準/靠近邏輯    : 與 Task 1 APPROACH 相同 (delta_x 置中 + depth 靠近)
  - 門把互動        : arm_controller.knob_poke()  (新增的門把姿勢原語)

執行緒模型與 Task1Mission 相同：背景 daemon thread + stop_event。

⚠️ 參數需在 Unity 場景內實跑微調。日誌刻意「很吵」，方便快速對到正確門檻值。
"""

import threading
import time

from pros_car_py.nav2_utils import calculate_angle_point, cal_distance


class Task3Mission:
    # ---- 狀態 ----
    DRIVE_WP = "DRIVE_WP"
    SEARCH = "SEARCH"
    APPROACH = "APPROACH"
    OBSERVE = "OBSERVE"
    UNLOCK = "UNLOCK"
    CLEAR = "CLEAR"
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
        self.DBG_EVERY = 10              # 每幾幀印一次狀態列 (~1s)

        # ---- 搜尋 ----
        self.SEARCH_TIMEOUT = 240.0      # 找不到 knob 的保險上限 (s)
        self.SEARCH_CONFIRM = 3          # 連續偵測到 N 幀才認定 (濾假偵測)

        # ---- 位姿式上門 waypoint (pinned (0,0,0) spawn frame; reset_map.sh --pin 後量測) ----
        # 依序開過這些點到門把前,再交給 SEARCH→APPROACH 視覺 dock。格式 [x, y, arrive_dist]。
        # 空 list → 退回舊行為 (原地 SEARCH 旋轉找門把)。
        # 門把 (lever) 實測位置 ≈ (3.3, 1.69)。末點停在門把前 ~0.4m 當 dock,再交給視覺/壓桿。
        self.WAYPOINTS = [
            [0.5,  0.0,  0.20],
            [1.0,  0.0,  0.20],
            [1.5,  0.0,  0.20],
            [2.0,  0.0,  0.20],
            [2.0,  0.5,  0.20],
            [2.0,  1.0,  0.20],
            [2.0,  1.5,  0.20],
            [2.0,  1.69, 0.15],
            [2.5,  1.69, 0.15],
            [2.6,  1.69, 0.12],
            [2.7,  1.69, 0.12],
            [2.8,  1.69, 0.12],
            [2.9,  1.69, 0.12],   # 門把 (3.3,1.69) 前 ~0.4m → 交給視覺 dock / 壓桿
        ]
        # DRIVE_WP 行進參數 (沿用 Task2 BRIDGE_APPROACH 調好的值)
        self.APPROACH_DRIVE_SPEED = 500.0   # 全速 (dist >= APPROACH_FAR_DIST) — 平地全速 (app 更新 ×2.5)
        self.APPROACH_TURN_GAIN = 7.0       # 角度 → wheel-diff 比例 (deg → speed)
        self.APPROACH_SPIN_DEG = 15.0       # 方位角差 > 此值 → 原地轉 (僅遠區)
        self.APPROACH_FAR_DIST = 1.0        # < 此距離降到 70%
        self.APPROACH_NEAR_DIST = 0.35      # < 此距離「不原地轉」,改直行+弱轉向爬進 (防近目標 pivot-stall)
        self.STUCK_MOVE_TOL = 0.03          # N 幀內位移 < 此值 (m) 視為沒動
        self.STUCK_TICKS = 15               # ~1.5s 沒動 → 末點 stuck guard 觸發
        self.WP_TIMEOUT = 240.0             # DRIVE_WP 總逾時保險 (s) — 設大,真卡死才兜底 (逾時=放棄,不亂解鎖)

        # ---- 靠近 (= Task 1 APPROACH 風格) ----
        # 門把比 bear 高、且要「停在伸臂可碰到」的距離。深度 <0.45m 會觸底失效，
        # 所以停在最近可靠深度，再靠手臂往前頂 (knob_poke) 補足。
        self.APPROACH_STOP_DIST = 0.45   # 到此距離就停 (m) → 開始 Locate & Observe
        self.ALIGN_PX = 50.0             # 置中容差 (px)：門把較小，比 bear 嚴一點
        self.LOST_CONFIRM = 5            # 連續遺失 N 幀才判定
        self.COMMIT_DOCK_DIST = 0.7      # 已靠近到此距離才遺失 → 視為到位 (被手臂/車身遮擋)

        # ---- 觀察 ----
        self.OBSERVE_SECONDS = 5.5       # 靜止觀察時間 (>5s 才拿分，留裕度)

        # ---- 解鎖 (手臂下壓開門 lever press) ----
        # 趴到門把前 (arm 收著=門把可見) → 抬手 → 下壓 lever。趴門時★朝門把溫和 pure-pursuit★:邊開邊把門把
        # 帶到正前方收斂 (dx→0),爪最後落 lever 正上方。(收斂≠原地轉置中——原地轉會在遠處先轉出大角度、之後斜著
        #  撞門壓不到;溫和邊開邊修則角度小且越近越正。)橫向起點靠 WAYPOINTS 停在門把同一 y,pursuit 收掉殘餘偏移。
        self.UNLOCK_NUDGE_SEC = 3.0       # 趴到門把前的前進上限 (s);到門把很近 / 撞到門 會提早停
        self.UNLOCK_NUDGE_SPEED = 200.0   # 趴門前進輪速 (raw);慢一點讓 pursuit 收斂跟得上
        self.UNLOCK_DOCK_DIST = 0.30      # ★arm 收著時★門把深度 <= 此值 → 已到門把正前方 → 停 (再抬手壓桿)。
                                          #   設小:確保夠近 (爪夠得到 lever);若還沒到門就停就調大,壓不到 lever 就調小。
        # 進門前先把車頭對正門軸 (+x = orientation/yaw 0),垂直起步,避免一開始就斜。
        self.UNLOCK_ALIGN_DEG = 6.0       # 車頭與 +x 夾角 <= 此值算對正
        self.UNLOCK_ALIGN_TIMEOUT = 4.0   # 對正逾時保險 (s):轉不到位也往下走
        # 趴門「朝門把 pure-pursuit」:依 knob dx 溫和轉向收斂 (邊開邊修,gain 小→角度小、越近越正)。
        self.KNOB_PURSUE_GAIN = 0.45      # knob dx → wheel-diff 比例 (溫和;太大會在遠處轉出大角度斜撞門)
        self.KNOB_PURSUE_CLAMP = 90.0     # pursuit 轉向差速上限 (raw):限制最大角度,保兩輪都前進
        self.KNOB_PURSUE_DEADBAND = 12.0  # |knob dx| 在此內 → 直行 (已夠正,不再修)
        # CLEAR 穿門 (爪已抬起擋住門把,無 knob 可追) → 用 /amcl_pose 維持車頭垂直門面 (yaw=0) 直行穿門。
        self.UNLOCK_HOLD_GAIN = 7.0       # yaw 誤差 (deg) → wheel-diff 比例 (= DRIVE_WP 同款)
        self.UNLOCK_HOLD_CLAMP = 90.0     # 維持轉向差速上限 (raw):保兩輪都前進、溫和修正不甩出門

        # ---- 推開門 (車身前推) ----
        self.CLEAR_PUSH_SEC = 15.0       # 直線前推穿門的時間 (s) — 拉長確保整台車過門
        self.CLEAR_SPEED = 300.0         # 前推輪速 (平地全速,確保完全穿過門) (app 更新 ×2.5)

        # ---- 執行緒狀態 ----
        self._thread = None
        self._stop_event = threading.Event()
        self._running = False

        self.state = self.SEARCH

    # ==========================================================
    # 對外介面
    # ==========================================================
    def start(self):
        if self._running:
            return
        self._stop_event.clear()
        self.state = self.DRIVE_WP if self.WAYPOINTS else self.SEARCH
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
        print("[Task3] 任務開始 (門把：定位觀察 → 解鎖 → 推開門)。")
        # 先把手臂收到鏡頭視野外,避免爪擋住相機 → DRIVE_WP/SEARCH/APPROACH 的 knob 偵測才穩。
        self.arm_controller.knob_stow()
        search_deadline = time.time() + self.SEARCH_TIMEOUT
        observe_start = None
        clear_start = 0.0
        found_streak = 0
        lost_streak = 0
        last_valid_dist = None
        dbg_tick = 0
        wp_idx = 0
        wp_deadline = time.time() + self.WP_TIMEOUT
        stuck_anchor_xy = None
        stuck_anchor_tick = 0

        while not stop_event.is_set():
            dbg_tick += 1
            # knob 走獨立 topic /yolo/target_info_knob (統一 pipeline 中 bear 同時在 /yolo/target_info)。
            info = self.data_processor.get_knob_target_info()  # [found,dist,dx,area,bottom] or None
            found = bool(info and info[0] == 1)
            dist = info[1] if info else 0.0
            dx = info[2] if info else 0.0

            # ---------------- DRIVE_WP (位姿式開到門把前) ----------------
            if self.state == self.DRIVE_WP:
                pose_msg = self.ros_communicator.get_latest_amcl_pose()
                if pose_msg is None:
                    self._publish("STOP")
                    if time.time() > wp_deadline:
                        self._transition(self.DONE, "DRIVE_WP 無 /amcl_pose 逾時 → 放棄 (沒到門口,不亂解鎖)")
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
                        self._transition(self.UNLOCK,
                                         f"通過最後 WP (dist={dist_wp:.2f}m) → 直接解鎖+穿門")
                    else:
                        print(f"[Task3] 通過 WP {wp_idx}/{len(self.WAYPOINTS)} (dist={dist_wp:.2f}m)")
                else:
                    # 先決定要「原地轉」還是「前進」。近目標 (dist < NEAR_DIST) 不原地轉
                    # (dist→0 時 bearing 會暴衝 → pivot-stall),改直行 + 弱轉向爬進。
                    ang = calculate_angle_point(o.z, o.w, car_xy, wp_target)
                    near = dist_wp < self.APPROACH_NEAR_DIST
                    spinning = (not near) and abs(ang) > self.APPROACH_SPIN_DEG

                    # 卡死 guard:只在「想前進但沒動」時計時;轉彎 (原地轉) 不算卡死,否則
                    # 90° 轉角會被誤判成卡住而提早跳點 (實測 WP5/6/9 被亂跳)。
                    if spinning:
                        stuck_anchor_xy = None
                    elif (stuck_anchor_xy is None
                            or cal_distance(car_xy, stuck_anchor_xy) > self.STUCK_MOVE_TOL):
                        stuck_anchor_xy = list(car_xy)
                        stuck_anchor_tick = dbg_tick
                    elif dbg_tick - stuck_anchor_tick >= self.STUCK_TICKS:
                        self._publish("STOP")
                        if last_wp:
                            self._transition(self.UNLOCK,
                                             f"末 WP 卡住 ({self.STUCK_TICKS}f 沒動) → 直接解鎖+穿門")
                        else:
                            wp_idx += 1
                            print(f"[Task3] WP {wp_idx}/{len(self.WAYPOINTS)} 卡住 "
                                  f"({self.STUCK_TICKS}f 沒動,dist={dist_wp:.2f}) → 跳下一點")
                        stuck_anchor_xy = None
                        time.sleep(self.TICK)
                        continue

                    if spinning:
                        self._publish("COUNTERCLOCKWISE_ROTATION_SLOW" if ang > 0
                                      else "CLOCKWISE_ROTATION_SLOW")
                    else:
                        # 直行 + 弱轉向,turn 夾在 ±0.5*base 確保兩輪都保有前進分量,不會原地空轉。
                        base = self.APPROACH_DRIVE_SPEED
                        turn = self.APPROACH_TURN_GAIN * ang
                        if near:
                            base *= 0.40
                            turn *= 0.4
                            turn = max(-0.5 * base, min(0.5 * base, turn))
                        elif dist_wp < self.APPROACH_FAR_DIST:
                            base *= 0.70
                        turn = max(-base, min(base, turn))
                        left = base - turn
                        right = base + turn
                        self.ros_communicator.publish_raw_car_control([left, right, left, right])
                    if dbg_tick % self.DBG_EVERY == 0:
                        print(f"[Task3][DRIVE_WP] WP {wp_idx + 1}/{len(self.WAYPOINTS)} "
                              f"car=({car_xy[0]:.2f},{car_xy[1]:.2f}) dist={dist_wp:.2f} ang={ang:+.0f}")

                if time.time() > wp_deadline:
                    self._publish("STOP")
                    self._transition(self.DONE, "DRIVE_WP 逾時 (沒走完 waypoints) → 放棄,不亂解鎖")

            # ---------------- SEARCH (備援:WAYPOINTS 為空時才會用到) ----------------
            elif self.state == self.SEARCH:
                found_streak = found_streak + 1 if found else 0
                if found_streak >= self.SEARCH_CONFIRM:
                    self._transition(self.APPROACH, f"穩定偵測到 knob (dist={dist:.2f})")
                    last_valid_dist = None
                    lost_streak = 0
                elif time.time() > search_deadline:
                    self._transition(self.DONE, "搜尋逾時，找不到 knob")
                else:
                    self._publish("CLOCKWISE_ROTATION_SLOW")

            # ---------------- APPROACH (= Task 1 邏輯) ----------------
            elif self.state == self.APPROACH:
                if not found:
                    lost_streak += 1
                    self._publish("STOP")
                    if lost_streak >= self.LOST_CONFIRM:
                        if (last_valid_dist is not None
                                and last_valid_dist <= self.COMMIT_DOCK_DIST):
                            self._publish("STOP")
                            observe_start = time.time()
                            self._transition(self.OBSERVE,
                                             f"近距離遺失(被遮擋, last={last_valid_dist:.2f}) → 視為到位")
                        else:
                            self._transition(self.SEARCH, "目標遺失(尚遠) → 退回 SEARCH")
                            found_streak = 0
                            search_deadline = time.time() + self.SEARCH_TIMEOUT
                    time.sleep(self.TICK)
                    continue

                lost_streak = 0
                if dist > 0.0:
                    last_valid_dist = dist
                is_close = (0.0 < dist <= self.APPROACH_STOP_DIST) or (
                    dist == -1.0 and last_valid_dist is not None and last_valid_dist < 1.0
                )
                if is_close and abs(dx) <= self.ALIGN_PX:
                    self._publish("STOP")
                    observe_start = time.time()
                    self._transition(self.OBSERVE, f"已到位 (dist={dist:.2f}, dx={dx:.0f})")
                elif abs(dx) > self.ALIGN_PX:
                    if dx > 0:
                        self._publish("CLOCKWISE_ROTATION_SLOW")
                    else:
                        self._publish("COUNTERCLOCKWISE_ROTATION_SLOW")
                elif not is_close:
                    self._publish("FORWARD_SLOW")
                else:
                    self._publish("STOP")

            # ---------------- OBSERVE ----------------
            elif self.state == self.OBSERVE:
                self._publish("STOP")
                if time.time() - observe_start >= self.OBSERVE_SECONDS:
                    self._transition(self.UNLOCK, "觀察完成 → 解鎖門把")

            # ---------------- UNLOCK (對正門軸 → 置中門把 → 趴到門把前 → 抬手 + 下壓 lever 開門) ----------------
            # 定位 (對正/置中/趴到門把前) 全在 arm 收著時完成 —— 抬手後爪會擋住門把偵測,故先趴好位置再抬手壓桿。
            elif self.state == self.UNLOCK:
                self._publish("STOP")
                print("[Task3] UNLOCK：對正門軸 → 置中門把 → 趴到門把前 → 抬手 + 下壓 lever 開門")
                # 0. 先把車頭對正門軸 (+x = yaw 0) → 垂直進門,避免斜著進門卡到門框 (左輪卡門柱)。
                align_start = time.time()
                while time.time() - align_start < self.UNLOCK_ALIGN_TIMEOUT:
                    if stop_event.is_set():
                        break
                    pose_msg = self.ros_communicator.get_latest_amcl_pose()
                    if pose_msg is None:
                        break
                    p = pose_msg.pose.pose.position
                    o = pose_msg.pose.pose.orientation
                    ang = calculate_angle_point(o.z, o.w, [p.x, p.y], [p.x + 1.0, p.y])
                    if abs(ang) <= self.UNLOCK_ALIGN_DEG:
                        break
                    self._publish("COUNTERCLOCKWISE_ROTATION_SLOW" if ang > 0
                                  else "CLOCKWISE_ROTATION_SLOW")
                    time.sleep(self.TICK)
                self._publish("STOP")
                print(f"[Task3] UNLOCK：車頭已對正門軸 (+x, |ang|<={self.UNLOCK_ALIGN_DEG:.0f}°) → 朝門把 pursuit 趴到門把前")
                # 1. ★趴到門把正前方 (arm 仍收著 = 門把清楚可見)★:★朝門把 (knob dx) 溫和 pure-pursuit 邊開邊修★,
                #    把門把收斂到正前方 (dx→0),爪最後落 lever 正上方。看不到門把 → 直行。
                #    開到門把很近 (dist <= UNLOCK_DOCK_DIST) 或「曾靠近後掉框=撞到門/門把進爪下」或逾時 → 停。
                nudge_start = time.time()
                k_was_close = False
                while time.time() - nudge_start < self.UNLOCK_NUDGE_SEC:
                    if stop_event.is_set():
                        break
                    info = self.data_processor.get_knob_target_info()
                    k_found = bool(info and info[0] > 0.5)
                    k_dist = info[1] if info else 0.0
                    k_dx = info[2] if info else 0.0
                    if k_found and 0.0 < k_dist <= self.UNLOCK_DOCK_DIST:
                        break                      # 到門把正前方 → 停
                    if k_was_close and not k_found:
                        break                      # 曾靠近後掉框 = 已撞到門/門把在爪下 → 到位
                    if k_found and 0.0 < k_dist <= self.UNLOCK_DOCK_DIST * 1.6:
                        k_was_close = True
                    self._knob_pursue(self.UNLOCK_NUDGE_SPEED, k_found, k_dx)   # 朝門把溫和收斂
                    time.sleep(self.TICK)
                self._publish("STOP")
                print("[Task3] UNLOCK：已到門把正前方 → 抬手 + 下壓 lever")
                # 2. 抬手臂到 READY (Wrist 177/Finger 閉合/Elbow 8) + 下壓 lever 開門 (不收回,維持壓著穿門,避免門閂彈回)
                self.arm_controller.knob_raise()
                self.arm_controller.knob_press_down()
                clear_start = time.time()
                self._transition(self.CLEAR, "下壓開門完成 (不收手) → 車身直行穿門")

            # ---------------- CLEAR (維持車頭垂直門面直行穿門) ----------------
            elif self.state == self.CLEAR:
                clear_elapsed = time.time() - clear_start
                if clear_elapsed >= self.CLEAR_PUSH_SEC:
                    self._publish("STOP")
                    self._transition(self.DONE, f"門已推開 (前推 {clear_elapsed:.1f}s)")
                else:
                    # ★維持車頭垂直門面 (yaw=0) 直行穿門★ (此時爪已抬起擋住門把偵測,故用 pose 維持垂直,
                    #   不靠 knob 視覺):垂直對著門中心直穿,整車不卡門框。
                    self._drive_door_axis(self.CLEAR_SPEED)

            # ---------------- DONE ----------------
            elif self.state == self.DONE:
                self._publish("STOP")
                break

            self._dbg_line(dbg_tick, found, dist, dx)
            time.sleep(self.TICK)

        self._publish("STOP")
        print("[Task3] 任務執行緒結束。")

    # ==========================================================
    # 工具
    # ==========================================================
    def _transition(self, new_state, reason):
        old = self.state
        self.state = new_state
        print(f"[Task3] {old} → {new_state}  ({reason})")

    def _knob_pursue(self, base, k_found, k_dx):
        """前進並朝門把 (knob dx) 溫和轉向收斂 (pure pursuit):邊開邊把門把帶到正前方,爪最後落 lever 正上方。
        knob 偏右 (dx>0) → 右轉 (左輪快/右輪慢);|dx| 在死區內 或 門把不在框 → 直行。gain 小 → 角度小、越近越正。"""
        if k_found and abs(k_dx) > self.KNOB_PURSUE_DEADBAND:
            steer = self.KNOB_PURSUE_GAIN * k_dx
            steer = max(-self.KNOB_PURSUE_CLAMP, min(self.KNOB_PURSUE_CLAMP, steer))
        else:
            steer = 0.0
        left = base + steer
        right = base - steer
        self.ros_communicator.publish_raw_car_control([left, right, left, right])
        return steer

    def _drive_door_axis(self, base):
        """前進並用 /amcl_pose 維持車頭「垂直門面」(對齊門軸 +x, yaw=0):垂直撞門/穿門,
        爪正對 lever 壓得到、整車不卡門框。轉向只做弱 yaw 修正 (不靠 knob dx,避免門把橫向偏時把車轉斜)。
        UNLOCK 趴門 + CLEAR 穿門共用。取不到 pose → 純直行。"""
        pose_msg = self.ros_communicator.get_latest_amcl_pose()
        turn = 0.0
        if pose_msg is not None:
            p = pose_msg.pose.pose.position
            o = pose_msg.pose.pose.orientation
            # 車頭與門軸 +x 的夾角 (deg);ang>0 → 車偏右需左修 (= DRIVE_WP 同款 arc 慣例)
            ang = calculate_angle_point(o.z, o.w, [p.x, p.y], [p.x + 1.0, p.y])
            turn = self.UNLOCK_HOLD_GAIN * ang
            turn = max(-self.UNLOCK_HOLD_CLAMP, min(self.UNLOCK_HOLD_CLAMP, turn))
        left = base - turn
        right = base + turn
        self.ros_communicator.publish_raw_car_control([left, right, left, right])
        return turn

    def _dbg_line(self, tick, found, dist, dx):
        if not self.DEBUG or tick % self.DBG_EVERY != 0:
            return
        print(f"[Task3][{self.state}] knob(found={int(found)} dist={dist:.2f} dx={dx:+.0f})")

    def _publish(self, action_key):
        self.ros_communicator.publish_car_control(
            action_key, publish_rear=True, publish_front=True
        )


def main(args=None):
    """Headless 進入點：直接跑 Task 3 任務並等到結束。
    用法 (容器內)：ros2 run pros_car_py task3_auto
    需統一 pipeline 已起 (start_stack.sh)，knob 偵測 remap 至 /yolo/target_info_knob。
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

    mission = Task3Mission(
        ros_communicator,
        data_processor,
        nav_processing,
        car_controller,
        arm_controller,
    )

    print("[task3_auto] 啟動 Task 3 任務 (headless)。Ctrl-C 可中止。")
    mission.start()
    try:
        while mission._running and mission._thread.is_alive():
            mission._thread.join(timeout=0.5)
    except KeyboardInterrupt:
        print("[task3_auto] 收到中止訊號。")
    finally:
        mission.stop()
        ros_communicator.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
