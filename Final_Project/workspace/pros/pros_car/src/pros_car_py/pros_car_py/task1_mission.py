"""
Task1Mission — Final Project Task 1 自動任務 (反應式視覺伺服 + Nav2 返航)
=======================================================================

完整流程 (對應計分項目)：

  SEARCH   → 原地旋轉直到 YOLO 偵測到 bear
  APPROACH → 依 /yolo/target_info 的 delta_x 對準、依 distance 前進靠近
             (Locate & Observe, 10 pts 的前置)
  OBSERVE  → 停在 bear 前方並保持靜止 >= 5 秒 (Locate & Observe, 10 pts)
  GRIP     → 透過 arm_controller 自動夾取 (/yolo/target_marker 由 YOLO 節點自動產生)
  RETURN   → 用 Nav2 導航回任務起點 (Recovery, 20 pts)
  DONE     → 停車結束

設計上盡量「重用既有元件」：
  - 前進/旋轉指令     : ros_communicator.publish_car_control + ACTION_MAPPINGS
  - 自動夾取          : arm_controller.auto_control(key='g')
  - Nav2 返航跟隨      : nav_processing.get_action_from_nav2_plan_no_dynamic_p_2_p
  - 起點/車身定位      : ros_communicator.get_latest_amcl_pose (由 tf_to_amcl_pose 提供)

執行緒模型與 car_controller.auto_control 相同：背景 daemon thread + stop_event。
"""

import threading
import time


class Task1Mission:
    # ---- 狀態 ----
    SEARCH = "SEARCH"
    APPROACH = "APPROACH"
    OBSERVE = "OBSERVE"
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
        self.APPROACH_STOP_DIST = 0.6    # 距 bear 多近視為到位 (m)，對應 "within N units"
        self.ALIGN_PX = 60.0             # delta_x 在此範圍內視為對準 (px, 以真實中心計)
        self.OBSERVE_SECONDS = 5.5       # 靜止觀察時間 (>5s 才拿分，留 0.5s 餘裕)
        self.GRIP_WAIT = 15.0            # 等待手臂完成夾取排程的時間 (s)
        self.RETURN_ARRIVE_DIST = 0.5    # 回到起點的容許半徑 (m)
        self.SEARCH_TIMEOUT = 120.0      # 找不到 bear 的保險上限 (s)

        # ---- 執行緒狀態 ----
        self._thread = None
        self._stop_event = threading.Event()
        self._running = False

        self.state = self.SEARCH
        self.start_pose = None  # [x, y]，任務起點 (Nav2 返航目標)

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
            print("[Task1] ⚠️ 取不到 /amcl_pose，RETURN 將無法使用 Nav2 返航。")
        else:
            print(f"[Task1] 起點記錄為 {self.start_pose}")

        search_deadline = time.time() + self.SEARCH_TIMEOUT
        observe_start = None
        return_started = False

        while not stop_event.is_set():
            info = self.data_processor.get_yolo_target_info()  # [found, dist, dx] or None
            found = bool(info and info[0] == 1)
            dist = info[1] if info else 0.0
            dx = info[2] if info else 0.0

            # ---------------- SEARCH ----------------
            if self.state == self.SEARCH:
                if found:
                    print("[Task1] 偵測到 bear → APPROACH")
                    self.state = self.APPROACH
                elif time.time() > search_deadline:
                    print("[Task1] ⚠️ 搜尋逾時，結束任務。")
                    self.state = self.DONE
                else:
                    self._publish("CLOCKWISE_ROTATION_SLOW")

            # ---------------- APPROACH ----------------
            elif self.state == self.APPROACH:
                if not found:
                    # 目標暫時消失，退回搜尋
                    self._publish("CLOCKWISE_ROTATION_SLOW")
                    self.state = self.SEARCH
                    search_deadline = time.time() + self.SEARCH_TIMEOUT
                elif dist == -1.0 or (0.0 < dist <= self.APPROACH_STOP_DIST):
                    # dist == -1 表示過近(<~0.4m)，視為已到位
                    self._publish("STOP")
                    observe_start = time.time()
                    print("[Task1] 已到位 → OBSERVE (保持靜止 5s)")
                    self.state = self.OBSERVE
                elif dx > self.ALIGN_PX:
                    self._publish("CLOCKWISE_ROTATION_SLOW")
                elif dx < -self.ALIGN_PX:
                    self._publish("COUNTERCLOCKWISE_ROTATION_SLOW")
                else:
                    self._publish("FORWARD_SLOW")

            # ---------------- OBSERVE ----------------
            elif self.state == self.OBSERVE:
                self._publish("STOP")
                if time.time() - observe_start >= self.OBSERVE_SECONDS:
                    print("[Task1] 觀察完成 → GRIP")
                    self.state = self.GRIP

            # ---------------- GRIP ----------------
            elif self.state == self.GRIP:
                self._publish("STOP")
                if self._do_grip(stop_event):
                    print("[Task1] 夾取完成 → RETURN")
                    self.state = self.RETURN
                    self.nav_processing.reset_nav_process()
                    return_started = False
                else:
                    print("[Task1] ⚠️ 夾取失敗 (無目標 Marker)，結束任務。")
                    self.state = self.DONE

            # ---------------- RETURN ----------------
            elif self.state == self.RETURN:
                if self.start_pose is None:
                    self._publish("STOP")
                    self.state = self.DONE
                else:
                    action = self.nav_processing.get_action_from_nav2_plan_no_dynamic_p_2_p(
                        goal_coordinates=self.start_pose
                    )
                    self._publish(action if action else "STOP")
                    return_started = True
                    if self.nav_processing.get_finish_flag():
                        print("[Task1] 已回到起點 → DONE (Recovery 完成)")
                        self.nav_processing.reset_nav_process()
                        self.state = self.DONE

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
    def _do_grip(self, stop_event):
        """觸發手臂自動夾取，並等待排程完成。"""
        if self.ros_communicator.latest_yolo_marker is None:
            return False
        # arm_controller.auto_control 會讀 latest_yolo_marker，做 TF 轉換，
        # 並在背景執行 _execute_grab_sequence (open→move→close→retract)。
        self.arm_controller.auto_control(key="g", mode="auto_arm_human")
        # 等待夾取排程跑完 (分段 sleep 以便能即時中止)
        waited = 0.0
        while waited < self.GRIP_WAIT and not stop_event.is_set():
            time.sleep(0.2)
            waited += 0.2
        return True

    def _capture_start_pose(self, retries=30, delay=0.2):
        """讀取目前的 /amcl_pose 作為任務起點 [x, y]。"""
        for _ in range(retries):
            pose_msg = self.ros_communicator.get_latest_amcl_pose()
            if pose_msg is not None:
                p = pose_msg.pose.pose.position
                return [p.x, p.y]
            time.sleep(delay)
        return None

    def _publish(self, action_key):
        self.ros_communicator.publish_car_control(
            action_key, publish_rear=True, publish_front=True
        )
