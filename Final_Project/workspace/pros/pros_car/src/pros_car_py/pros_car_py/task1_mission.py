"""
Task1Mission — Final Project Task 1 自動任務 (反應式視覺伺服 + 位姿式返航)
=======================================================================

完整流程 (對應計分項目)：

  SEARCH   → 原地旋轉直到 YOLO 偵測到 bear
  APPROACH → 依 /yolo/target_info 的 delta_x 對準、依 distance 前進靠近
             (Locate & Observe, 10 pts 的前置)
  OBSERVE  → 停在 bear 前方並保持靜止 >= 5 秒 (Locate & Observe, 10 pts)
  CREEP    → 推土機式鏟取：手臂降到貼地鏟取姿勢(開爪)，再用車身固定前推把 bear 推進爪中
  GRIP     → 關爪夾住 + 抬起搬運 (不靠 IK 構到目標，改用車身定位)
  RETURN   → 位姿式直線返航回起點 (/amcl_pose 對準直行 + 最終轉向) → 放下 bear，不靠 Nav2 (Recovery, 20 pts)
  DONE     → 停車結束

設計上盡量「重用既有元件」：
  - 前進/旋轉指令     : ros_communicator.publish_car_control + ACTION_MAPPINGS
  - 鏟取/夾取         : arm_controller.scoop_pose() + scoop_grab()
  - 起點/車身定位      : ros_communicator.get_latest_amcl_pose (由 tf_to_amcl_pose 提供)

執行緒模型與 car_controller.auto_control 相同：背景 daemon thread + stop_event。
"""

import threading
import time
import math

from pros_car_py.nav2_utils import calculate_angle_point, cal_distance


class Task1Mission:
    # ---- 狀態 ----
    SEARCH = "SEARCH"
    APPROACH = "APPROACH"
    OBSERVE = "OBSERVE"
    CREEP = "CREEP"
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

        # ---- 可調參數 (需依 Unity 場景的 "N units" 與夾爪幾何微調) ----
        self.TICK = 0.1                  # 控制週期 (s)
        # 深度感測在 ~0.45m 以下會失效(觸底)，而手臂可達範圍只有 ~0.19m。
        # 因此：APPROACH 先停在「最近的有效深度 (~0.5m)」完成 Locate&Observe，
        # 之後再 CREEP「盲推前進」一小段把 bear 推進手臂可達範圍才夾取。
        self.APPROACH_STOP_DIST = 0.50   # 到最近可靠深度就停 (m) → Locate&Observe
        self.ALIGN_PX = 70.0             # 置中容差 (px)：放寬以免原地轉向過衝、左右來回獵擺；
                                         # 推土機夾取較寬容，CREEP 前推時仍會再依 dx 修正置中
        self.LOST_CONFIRM = 5            # APPROACH 連續遺失 N 幀才判定 (容忍 YOLO 短暫掉幀)
        self.COMMIT_DOCK_DIST = 0.7      # 先前已靠近到此距離才遺失 → 視為被遮擋(到位)，直接夾取而非回 SEARCH
        self.OBSERVE_SECONDS = 5.5       # 靜止觀察時間 (>5s 才拿分，留 0.5s 餘裕，與 Task 3 相同)
        # CREEP = 推土機式夾取：手臂先降到貼地「鏟取」姿勢(開爪)，再用車身固定前推，
        # 把 bear 推進開著的低位爪中 → 不靠手臂去構到超出 0.19m 可達範圍的點。
        self.BULLDOZER_PUSH_SEC = 1.0    # 鏟取姿勢就緒後直線前推時間 (s) — 主要微調旋鈕
        self.CREEP_SPEED = 90.0          # CREEP 前推輪速 (比 FORWARD_SLOW=150 慢，更溫和地把
                                         # bear 推進爪中)；速度變慢=同時間推得更短，要保持距離就加大上面的時間

        # RETURN：位姿式直線返航 (不靠 Nav2/costmap)。用 /amcl_pose 算出朝起點的方位，
        # 先轉向對準再直行，到站後原地轉到「起始朝向相反」把熊放回。全程用具名輪速指令
        # (與去程相同，確定能動)，不經 cmd_vel→輪速 換算。
        self.RETURN_ARRIVE_DIST = 0.60   # 到起點的容許半徑 (m)：只要進到目標區就放下，不必硬塞到
                                         # 圓心 (車會在最後幾 cm 卡住/原地打轉)；爪子再向前伸 ~0.2m
        self.RETURN_ARRIVE_CONFIRM = 4   # 需「停在半徑內」連續這麼多幀才算真的到站 (擋高速衝過誤判)
        self.RETURN_TIMEOUT = 120.0      # 返航保險上限 (s)
        # 返航用「比例式 arc」控制：邊前進邊依航向誤差差速轉彎，避免原地轉/直行來回切換的頓挫。
        self.RETURN_SPIN_DEG = 20.0      # 航向誤差 > 此值先原地轉 (大角度 arc 轉不過來)，以下邊開邊轉
        self.RETURN_DRIVE_SPEED = 110.0  # arc 前進基礎輪速 (放慢 → 同差速下轉彎半徑更小)
        self.RETURN_TURN_GAIN = 7     # 每 1° 航向誤差的左右輪差速 (調大 → 轉更急，不再大彎)
        self.GOTO_FAR_DIST = 1.0         # 距起點 < 此值放慢前進，較好收尾 (m)

        # ---- 搜尋強化 (避免假偵測 & 找不到就放棄) ----
        self.SEARCH_TIMEOUT = 240.0      # 找不到 bear 的保險上限 (s)，加大以多轉幾圈
        self.SEARCH_CONFIRM = 3          # 連續偵測到 N 幀才認定真的找到 (濾除單幀假偵測)
        self.CLOSE_CONFIRM = 3           # 連續 N 幀都「夠近」才進 OBSERVE
        self.SEARCH_NUDGE_SEC = 12.0     # 轉這麼久仍沒看到 → 前進一下換視角
        self.SEARCH_NUDGE_TICKS = 8      # 前進的幀數 (約 0.8s)

        # ---- 執行緒狀態 ----
        self._thread = None
        self._stop_event = threading.Event()
        self._running = False

        self.state = self.SEARCH
        self.start_pose = None  # [x, y]，任務起點 (RETURN 返航目標)
        self.start_yaw = 0.0    # 起始朝向 (rad)，返航目標朝向設為其相反 (yaw+pi)

    # ==========================================================
    # 對外介面 (給 Task1Mode 呼叫)
    # ==========================================================
    def start(self):
        if self._running:
            return
        self._stop_event.clear()
        self.state = self.SEARCH
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
        print("[Task1] 任務開始。")
        # 記錄任務起點 (供 RETURN 使用)
        self.start_pose = self._capture_start_pose()
        if self.start_pose is None:
            print("[Task1] ⚠️ 取不到 /amcl_pose，RETURN 將無法返航。")
        else:
            print(f"[Task1] 起點記錄為 {self.start_pose}")

        search_deadline = time.time() + self.SEARCH_TIMEOUT
        observe_start = None
        creep_start = 0.0
        creep_arm_prepared = False  # 鏟取姿勢只在進 CREEP 時擺一次
        found_streak = 0       # SEARCH: 連續偵測幀數
        close_streak = 0       # (保留) APPROACH 計數
        lost_streak = 0        # APPROACH: 連續遺失目標幀數
        last_valid_dist = None # APPROACH: 最近一次有效深度
        search_nudge_until = 0.0   # SEARCH: 前進換視角的截止時間
        last_rotate_time = time.time()
        return_entry_time = 0.0
        return_dbg = 0         # RETURN: 診斷列印節流計數
        arrive_streak = 0      # RETURN: 連續「停在到站半徑內」的幀數

        while not stop_event.is_set():
            info = self.data_processor.get_yolo_target_info()  # [found,dist,dx,area,bottom] or None
            found = bool(info and info[0] == 1)
            dist = info[1] if info else 0.0
            dx = info[2] if info else 0.0
            bottom_frac = info[4] if (info and len(info) >= 5) else 0.0

            # ---------------- SEARCH ----------------
            if self.state == self.SEARCH:
                found_streak = found_streak + 1 if found else 0
                if found_streak >= self.SEARCH_CONFIRM:
                    print(f"[Task1] 穩定偵測到 bear (dist={dist:.2f}) → APPROACH")
                    self.state = self.APPROACH
                    close_streak = 0
                    last_valid_dist = None
                elif time.time() > search_deadline:
                    print("[Task1] ⚠️ 搜尋逾時，結束任務。")
                    self.state = self.DONE
                elif time.time() < search_nudge_until:
                    # 換視角：前進一小段
                    self._publish("FORWARD_SLOW")
                else:
                    # 持續原地旋轉掃描；每隔一段時間前進一下換視角
                    self._publish("CLOCKWISE_ROTATION_SLOW")
                    if time.time() - last_rotate_time > self.SEARCH_NUDGE_SEC:
                        search_nudge_until = time.time() + self.SEARCH_NUDGE_TICKS * self.TICK
                        last_rotate_time = time.time()

            # ---------------- APPROACH ----------------
            elif self.state == self.APPROACH:
                if not found:
                    lost_streak += 1
                    close_streak = 0
                    self._publish("STOP")
                    if lost_streak >= self.LOST_CONFIRM:
                        # 關鍵：若「先前已很靠近」才遺失，代表 bear 被爪子/車身擋住
                        # (= 已在正前方可夾位置) → 直接進 OBSERVE 開始夾取，不要回 SEARCH。
                        if (
                            last_valid_dist is not None
                            and last_valid_dist <= self.COMMIT_DOCK_DIST
                        ):
                            print(
                                f"[Task1] 近距離遺失(被遮擋, last={last_valid_dist:.2f}) "
                                f"→ 視為到位 → OBSERVE"
                            )
                            self._publish("STOP")
                            observe_start = time.time()
                            self.state = self.OBSERVE
                        else:
                            print("[Task1] 目標遺失 (距離尚遠) → 退回 SEARCH")
                            self.state = self.SEARCH
                            found_streak = 0
                            search_deadline = time.time() + self.SEARCH_TIMEOUT
                            last_rotate_time = time.time()
                    time.sleep(self.TICK)
                    continue

                lost_streak = 0
                if dist > 0.0:
                    last_valid_dist = dist
                # 是否「夠近」：有效深度 <= 門檻，或過近(-1)且先前確實已接近
                is_close = (0.0 < dist <= self.APPROACH_STOP_DIST) or (
                    dist == -1.0
                    and last_valid_dist is not None
                    and last_valid_dist < 1.0
                )

                if is_close and abs(dx) <= self.ALIGN_PX:
                    # 夠近且對準 → 直接到位 (不需多幀確認，避免繞圈)
                    self._publish("STOP")
                    observe_start = time.time()
                    print(f"[Task1] 已到位 (dist={dist:.2f}, dx={dx:.0f}) → OBSERVE")
                    # 注意：不在此抬手臂 — 抬起的爪子會擋住相機，害 CREEP 看不到 bear。
                    # 手臂維持低姿(不擋鏡頭)直到 GRIP 取得目標後才動作。
                    self.state = self.OBSERVE
                elif abs(dx) > self.ALIGN_PX:
                    # 先「原地轉向」對準，不前進 → 避免在熊周圍繞圈
                    if dx > 0:
                        self._publish("CLOCKWISE_ROTATION_SLOW")
                    else:
                        self._publish("COUNTERCLOCKWISE_ROTATION_SLOW")
                elif not is_close:
                    # 已對準但還不夠近 → 直行前進
                    self._publish("FORWARD_SLOW")
                else:
                    self._publish("STOP")

            # ---------------- OBSERVE ----------------
            elif self.state == self.OBSERVE:
                self._publish("STOP")
                if time.time() - observe_start >= self.OBSERVE_SECONDS:
                    print("[Task1] 觀察完成 → CREEP (推土機式鏟取)")
                    self.state = self.CREEP

            # ---------------- CREEP (推土機式：降臂鏟取姿勢 + 車身前推) ----------------
            elif self.state == self.CREEP:
                # 1) 進 CREEP 先把手臂降到貼地、爪面平行的鏟取姿勢並開爪 (阻塞到位)。
                #    手臂維持不動，靠車身把 bear 推進開著的低位爪中。
                if not creep_arm_prepared:
                    self._publish("STOP")
                    self.arm_controller.scoop_pose()   # 阻塞直到爪子降到鏟取姿勢
                    creep_arm_prepared = True
                    creep_start = time.time()          # 鏟取姿勢就緒後才起算前推
                creep_elapsed = time.time() - creep_start
                # 2) 像推土機一樣固定前推，把 bear 推進爪中；前推中仍對準 (看得到時)。
                if creep_elapsed >= self.BULLDOZER_PUSH_SEC:
                    self._publish("STOP")
                    print(f"[Task1] 前推完成 (推土機 {creep_elapsed:.1f}s) → GRIP")
                    self.state = self.GRIP
                elif found and dx > self.ALIGN_PX:
                    self._publish("CLOCKWISE_ROTATION_SLOW")
                elif found and dx < -self.ALIGN_PX:
                    self._publish("COUNTERCLOCKWISE_ROTATION_SLOW")
                else:
                    # 慢速前推 (直接發四輪等速，比 FORWARD_SLOW 更慢更溫和)。
                    self.ros_communicator.publish_raw_car_control(
                        [self.CREEP_SPEED, self.CREEP_SPEED,
                         self.CREEP_SPEED, self.CREEP_SPEED]
                    )

            # ---------------- GRIP (關爪夾住 + 抬起) ----------------
            elif self.state == self.GRIP:
                self._publish("STOP")
                print("[Task1] GRIP：關爪夾住 bear + 抬起搬運")
                # bear 已被車身推進開著的低位爪中 → 直接關爪、等黏合、抬起 (阻塞)。
                self.arm_controller.scoop_grab()
                print("[Task1] 夾取完成 → RETURN (位姿式直線返航回起點)")
                if self.start_pose is not None:
                    print(f"[Task1] RETURN 目標(起點) = {self.start_pose}")
                return_entry_time = time.time()
                self.state = self.RETURN

            # ---------------- RETURN (位姿式直線返航：朝起點直行，到站放下 bear) ----------------
            elif self.state == self.RETURN:
                pose_msg = self.ros_communicator.get_latest_amcl_pose()
                if self.start_pose is None or pose_msg is None:
                    self._publish("STOP")
                    if self.start_pose is None:
                        self.state = self.DONE
                    time.sleep(self.TICK)
                    continue

                if time.time() - return_entry_time > self.RETURN_TIMEOUT:
                    print("[Task1] ⚠️ 返航逾時，停止。")
                    self._publish("STOP")
                    self.state = self.DONE
                    continue

                p = pose_msg.pose.pose.position
                o = pose_msg.pose.pose.orientation
                car = [p.x, p.y]
                dist = cal_distance(car, self.start_pose)

                # 到站判定：必須「停在半徑內」連續 RETURN_ARRIVE_CONFIRM 幀才算真到站，
                # 避免高速衝過起點時單幀 dist<半徑 就誤判完成、邊滑邊放掉 bear。
                if dist < self.RETURN_ARRIVE_DIST:
                    arrive_streak += 1
                    self._publish("STOP")   # 先煞停，讓它真的定在起點而非滑過
                    if arrive_streak >= self.RETURN_ARRIVE_CONFIRM:
                        print(f"[Task1] 已穩定到站 (dist={dist:.2f}) → 放下 bear")
                        self.arm_controller.scoop_release()  # 降臂→開爪→抬空爪
                        print("[Task1] 已放下 bear → DONE (Recovery 完成)")
                        self.state = self.DONE
                    time.sleep(self.TICK)
                    continue
                arrive_streak = 0   # 滑出半徑 → 重新計數，繼續開回去

                # 航向誤差 (calculate_angle_point: >0→需逆時針/左轉、<0→順時針/右轉)。
                ang = calculate_angle_point(o.z, o.w, car, self.start_pose)
                if abs(ang) > self.RETURN_SPIN_DEG:
                    # 角度太大 → 先原地轉對準 (arc 轉不過來)。
                    action = ("COUNTERCLOCKWISE_ROTATION_SLOW" if ang > 0
                              else "CLOCKWISE_ROTATION_SLOW")
                    self._publish(action)
                else:
                    # 比例式 arc：邊前進邊差速轉彎，平順不停頓。
                    base = self.RETURN_DRIVE_SPEED
                    if dist < self.GOTO_FAR_DIST:
                        base *= 0.7              # 接近起點放慢，較好收尾
                    turn = self.RETURN_TURN_GAIN * ang
                    turn = max(-base, min(base, turn))   # 夾住，內輪最多到 0 (不強烈反轉)
                    # ang>0 需左轉(逆時針)：左輪慢、右輪快。
                    left = base - turn
                    right = base + turn
                    self.ros_communicator.publish_raw_car_control(
                        [left, right, left, right]
                    )
                    action = f"ARC(base={base:.0f},turn={turn:+.0f})"

                # 診斷：每 ~1s 印一次車況，判斷是卡住/獵擺/繞圈/amcl 失更新。
                return_dbg += 1
                if return_dbg % 10 == 1:
                    print(
                        f"[Task1] RETURN car=({car[0]:.2f},{car[1]:.2f}) "
                        f"start=({self.start_pose[0]:.2f},{self.start_pose[1]:.2f}) "
                        f"dist={dist:.2f} ang={ang:+.0f}° → {action}"
                    )

            # ---------------- DONE ----------------
            elif self.state == self.DONE:
                self._publish("STOP")
                break

            time.sleep(self.TICK)

        self._publish("STOP")
        print("[Task1] 任務執行緒結束。")

    # ==========================================================
    # 子流程
    # ==========================================================
    def _capture_start_pose(self, retries=30, delay=0.2):
        """讀取目前的 /amcl_pose 作為任務起點，並記錄起始朝向 (self.start_yaw, rad)。
        回傳 [x, y]。"""
        for _ in range(retries):
            pose_msg = self.ros_communicator.get_latest_amcl_pose()
            if pose_msg is not None:
                p = pose_msg.pose.pose.position
                o = pose_msg.pose.pose.orientation
                # 只有 yaw 的四元數：yaw = 2*atan2(z, w)
                self.start_yaw = 2.0 * math.atan2(o.z, o.w)
                return [p.x, p.y]
            time.sleep(delay)
        return None

    def _publish(self, action_key):
        self.ros_communicator.publish_car_control(
            action_key, publish_rear=True, publish_front=True
        )


def main(args=None):
    """Headless 進入點：不開 urwid 選單，直接跑 Task 1 任務並等到結束。

    用法 (容器內)：ros2 run pros_car_py task1_auto
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

    mission = Task1Mission(
        ros_communicator,
        data_processor,
        nav_processing,
        car_controller,
        arm_controller,
    )

    print("[task1_auto] 啟動 Task 1 任務 (headless)。Ctrl-C 可中止。")
    mission.start()
    try:
        # 等任務執行緒自己跑到 DONE 結束
        while mission._running and mission._thread.is_alive():
            mission._thread.join(timeout=0.5)
    except KeyboardInterrupt:
        print("[task1_auto] 收到中止訊號。")
    finally:
        mission.stop()
        ros_communicator.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
