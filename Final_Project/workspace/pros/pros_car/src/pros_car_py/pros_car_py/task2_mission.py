"""
Task2Mission — Final Project Task 2 自動任務 (對準上橋 + 推土機鏟取橋頂的熊 + 返航)
=======================================================================

關鍵前提 (場景保證)：
  **橋頂正中央一定有一隻熊。** 所以不需要視覺搜尋/觀察熊 —— 只要「對準橋面正中、把開著的
  推土機鏟爪降下、直直開上橋」，熊就會被鏟進爪中，到頂後關爪夾起即可。

策略 (無 IMU；Unity sim 無 /imu/data)：
  道路導航 (road-led)：用 /yolo/road_info 沿路前進；在路口用 /yolo/bridge_info 的「左/右」
  哪一側做轉彎提示。橋面遮罩的 area/sym/aspect/精準 dx 在貼近時太不穩,完全棄用。
  走到路盡頭 (road 連續遺失) = 走到橋頭 → 直接切 CLIMB,不再做幾何對準。

完整流程 (對應計分項目)：

  BRIDGE_APPROACH → 沿路 (road_dx 置中) 前進；若 bridge 偏在某一側 (路口),加固定 steer
                    朝那邊靠。road 連續遺失達門檻 + 近期看過 bridge → 走到橋頭 → CLIMB。
  CLIMB         → 先把開著的推土機鏟爪降下 (scoop_pose, 等於把熊「關」在爪裡的籠子)，
                  再固定前推、依 bridge dx 微修置中、直直開上橋；用 /amcl_pose 量行進距離到頂。
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

        # ---- BRIDGE_APPROACH：道路導航 + 路口「朝橋轉」(三段式) ----
        # 三段強度,依 bridge 在畫面的偏離程度切換模式 (橋面遮罩只用 found + sign(dx),不用其他幾何):
        #   |b_dx| >= BRIDGE_TURN_PX     → **原地轉**朝向 bridge (橋偏太遠 = 該轉的路口)。
        #                                  road 此時被忽略 — 先把車頭朝橋,再回去走路。
        #   BRIDGE_HINT_PX <= |b_dx|     → 跟道路 + 加固定 steer 朝 bridge 側 (中等強度偏轉)。
        #     < BRIDGE_TURN_PX
        #   |b_dx| <  BRIDGE_HINT_PX     → 純依 road_dx 走 (橋大致在前方/路上,不需偏轉)。
        # 另外: 如果上一個看到 bridge 是「far off 一側」狀態,然後突然 found=0 (我們剛經過/丟失),
        # 在 TURN_HOLD_TICKS 內持續往那一側「原地轉」,直到 re-acquire bridge 或時間到。
        # 這是處理「bridge 在左 -> 1 變 0 -> 我們應該左轉」的關鍵 (使用者口述邏輯)。
        self.APPROACH_SPEED = 110.0          # 沿路前進輪速
        self.ROAD_GAIN = 0.25                # steer = ROAD_GAIN * road_dx (沿路置中項)
        self.BRIDGE_HINT_PX = 60.0           # |bridge dx| > 此值 → 加 hint steer (中等偏轉)
        self.BRIDGE_HINT_STEER = 35.0        # 中等偏轉強度 (加在 steer 上、用 bridge dx 的 sign)
        self.BRIDGE_TURN_PX = 150.0          # |bridge dx| > 此值 → 原地轉 (路口必須轉的訊號)
        # 「橋剛從 far-off 變不見」的轉向延續：bridge 是 (>= BRIDGE_TURN_PX) 那一側,然後變 found=0,
        # 仍用該方向原地轉，最多 TURN_HOLD_TICKS 幀,讓車頭真的轉到能再看到橋的方位。
        self.TURN_HOLD_TICKS = 25            # ~2.5s
        # road 持續看不到時的「上橋切換」判定：表示我們走到路盡頭 (= 橋頭)。
        self.ROAD_LOST_COMMIT = 8            # road 連續遺失 N 幀 → 切 CLIMB (需近期看過 bridge)
        self.ROAD_FLICKER_TOL = 3            # road 短暫遺失 (<=此幀) 仍前進
        self.BRIDGE_APPROACH_TIMEOUT = 120.0
        # 雙雙不見時的最後手段搜尋方向
        self.SEARCH_SPIN = "CLOCKWISE_ROTATION_SLOW"

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
        scoop_prepared = False         # CLIMB: 鏟爪只在進 CLIMB 時降一次
        road_lost_streak = 0           # BRIDGE_APPROACH: road 連續遺失幀 (達門檻 → 走到路盡頭 = CLIMB)
        last_bridge_dx = 0.0           # BRIDGE_APPROACH: 最後看到 bridge 的 dx
        last_bridge_seen_tick = -10000 # BRIDGE_APPROACH: 最後一次看到 bridge 的 tick
        # 「far-off bridge 剛丟失 → 持續往該方向原地轉」的狀態：
        turn_hold_side = 0             # -1=左、+1=右、0=無 (last time bridge was far off, this side)
        turn_hold_remaining = 0        # 剩餘原地轉的 tick 數 (bridge re-acquire 或歸 0 即止)
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
            # 註：area/sym/aspect 在貼近時雜訊太大,road-led 版本不再使用,只用 b_found + sign(b_dx)。

            # ---------------- BRIDGE_APPROACH (道路導航 + 路口朝橋轉) ----------------
            if self.state == self.BRIDGE_APPROACH:
                # 記住最後看到 bridge 的方位 (用 sign 做轉向決策)
                if b_found:
                    last_bridge_dx = b_dx
                    last_bridge_seen_tick = dbg_tick
                    # 若 bridge 在 far-off 一側 → 設定 turn-hold (給「丟失後仍持續轉」用)
                    if abs(b_dx) >= self.BRIDGE_TURN_PX:
                        turn_hold_side = -1 if b_dx < 0 else 1
                        turn_hold_remaining = self.TURN_HOLD_TICKS
                    else:
                        # bridge 在「相對前方」(non-far-off) → 不需 turn-hold,清掉
                        turn_hold_side = 0
                        turn_hold_remaining = 0
                else:
                    # bridge 看不到時, turn-hold 倒數 (是 far-off 剛丟才會啟動的計數)
                    if turn_hold_remaining > 0:
                        turn_hold_remaining -= 1
                    else:
                        turn_hold_side = 0

                # ===== 決策優先序：路口大轉 > turn-hold > 道路置中 + 中等偏轉 > road lost commit =====

                # (1) bridge 看到且偏太遠 → 原地轉 (90° 路口必須直接轉)
                if b_found and abs(b_dx) >= self.BRIDGE_TURN_PX:
                    if b_dx > 0:
                        self._publish("CLOCKWISE_ROTATION_SLOW")
                    else:
                        self._publish("COUNTERCLOCKWISE_ROTATION_SLOW")
                    # 路口轉中 → road 計數不算 lost
                    if r_found:
                        road_lost_streak = 0

                # (2) bridge 看不到但剛剛還是 far-off (turn_hold 仍有效) → 繼續往該方向轉
                #     這是「bridge 從 1 變 0,該側」場景的關鍵 — 持續轉直到 re-acquire 或 hold 結束
                elif (not b_found) and turn_hold_remaining > 0 and turn_hold_side != 0:
                    if turn_hold_side > 0:
                        self._publish("CLOCKWISE_ROTATION_SLOW")
                    else:
                        self._publish("COUNTERCLOCKWISE_ROTATION_SLOW")
                    if r_found:
                        road_lost_streak = 0

                # (3) road 有看到 → 沿路前進 (中等偏轉或純置中)
                elif r_found:
                    road_lost_streak = 0
                    steer = self.ROAD_GAIN * r_dx
                    if b_found and abs(b_dx) >= self.BRIDGE_HINT_PX:
                        steer += self.BRIDGE_HINT_STEER * (1.0 if b_dx > 0 else -1.0)
                    self._arc(self.APPROACH_SPEED, steer)

                # (4) road 不見 → flicker 容忍 / commit / 搜尋
                else:
                    road_lost_streak += 1
                    if road_lost_streak <= self.ROAD_FLICKER_TOL:
                        # 短暫遺失,繼續直行
                        self._arc(self.APPROACH_SPEED, 0.0)
                    elif road_lost_streak >= self.ROAD_LOST_COMMIT:
                        # 持續遺失 → 走到路盡頭。只要近期看過 bridge,commit 上橋。
                        recently_saw_bridge = (dbg_tick - last_bridge_seen_tick) <= 30  # ~3s
                        if recently_saw_bridge or b_found:
                            self._publish("STOP")
                            climb_start_pose = self._current_xy()
                            climb_entry_time = time.time()
                            scoop_prepared = False
                            self._transition(
                                self.CLIMB,
                                f"道路盡頭 (road lost {road_lost_streak}f, "
                                f"last bridge dx={last_bridge_dx:+.0f}) → 上橋")
                            continue
                        else:
                            # 沒 road + 近期沒 bridge → 原地搜尋
                            self._publish(self.SEARCH_SPIN)
                    else:
                        # 中段遺失,慢慢直行
                        self._publish("FORWARD_SLOW")

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

            self._dbg_line(dbg_tick, r_found, r_dx, b_found, b_dx)
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
                self.start_yaw = 2.0 * math.atan2(o.z, o.w)
                return [p.x, p.y]
            time.sleep(delay)
        return None

    def _transition(self, new_state, reason):
        old = self.state
        self.state = new_state
        print(f"[Task2] {old} → {new_state}  ({reason})")

    def _dbg_line(self, tick, r_found, r_dx, b_found, b_dx):
        if not self.DEBUG or tick % self.DBG_EVERY != 0:
            return
        print(f"[Task2][{self.state}] road(found={int(r_found)} dx={r_dx:+.0f}) | "
              f"bridge(found={int(b_found)} dx={b_dx:+.0f})")

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
