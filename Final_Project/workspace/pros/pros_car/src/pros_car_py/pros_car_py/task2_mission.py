"""
Task2Mission — Final Project Task 2 自動任務
  (位姿粗對位 → 視覺伺服上橋走近橋上的熊並鏟入 → 關爪夾起 → 返航)
=======================================================================

關鍵前提 (場景)：橋上有一隻目標熊,橋外另有散落的誘餌熊。YOLO 端 YOLO_TARGET_PICK=onbridge
  (object_detect.py 訂閱 /yolo/bridge_info,只發布「橫向對齊橋面」的熊) 確保 VISUAL_CLIMB 追的是橋上那隻。

策略 (無 IMU；地圖種子固定)：
  位姿式 waypoints 把車帶上橋口 → 朝熊 bbox 視覺伺服爬橋固定秒數 → 夾起 → 對正下橋 → 位姿式返航。
  (橋上 /amcl_pose 會凍,故爬橋/夾取/對正下橋只用視覺、不用 pose。)

完整流程 (對應計分項目)：

  BRIDGE_APPROACH → 位姿式 DRIVE_WP:依序開過 WAYPOINTS (上橋路徑:對到橋口 → 沿橋軸 +y 開一點上橋,置中、不卡
                    橋側),末點到位 (或上橋 pose 凍住 stuck guard) → VISUAL_CLIMB。需先 reset_map.sh --pin 釘住 map frame。
  VISUAL_CLIMB   → 進場先 scoop_pose() 把鏟爪降下+開爪 (阻塞一次),之後朝「熊 bbox 的 dx」大力轉向並全速爬橋,
                    爬滿 VCLIMB_CLIMB_SEC 秒 → OBSERVE。看不到熊就直走。(只用計時 + 物件偵測;深度太抖不用來判停。)
  OBSERVE        → 停下原地轉把橋上的熊置中 (面向它),持住 OBSERVE_SECONDS 秒 (對齊,Task2 無 Locate&Observe 計分) → GRIP。
                    熊被低位爪遮/掉鏡頭或對中逾時 → 直接持住。
  GRIP           → 小幅前頂 (把熊鏟進爪中、抗坡面後滑) + scoop_grab() 關爪+抬起。
  SNAP_DESCEND   → 夾完原地轉對正 (朝下對側直):優先用「橋面質心 (b_dx,= 上橋同款)」,橋面看不到才退用前方路面
                    (r_dx);橋上 pose 凍,不用 yaw。對正/逾時 → DESCEND。
  DESCEND        → 夾住後全速前進,優先沿「橋面中線 (b_dx)」置中、橋面看不到才退用路面 (r_dx),翻過坡頂 fat part、
                    下階梯;不減速 (避免卡在階差)。
                    結束:橋面+路面 mask 皆消失 (已離橋到平地) 連續 N 幀為主、road 占滿畫面為輔,MAX_SEC 兜底;
                    順便下橋讓 /amcl_pose 解凍。之後 → RETURN。
  RETURN         → 位姿式返航:先依序開過 RETURN_WAYPOINTS 繞過橋的一側 (避免背著熊直線穿回陡橋),繞行完成
                    再直線回起點 → 放下 bear (Recovery)。RETURN_WAYPOINTS 空 = 直線回 (舊行為)。
  DONE           → 停車結束。

重用既有元件：
  - 差速前進/轉向   : ros_communicator.publish_raw_car_control / publish_car_control / _arc
  - bear 視覺       : data_processor.get_yolo_target_info ([found,dist,dx,area,bottom])
  - 鏟取/夾取/放下   : arm_controller.scoop_pose() + scoop_grab() + scoop_release()
  - 起點/朝向        : ros_communicator.get_latest_amcl_pose (由 tf_to_amcl_pose 提供)

⚠️ 參數需在 Unity 場景內實跑微調。日誌刻意「很吵」，方便快速對到正確門檻值。
"""

import threading
import time
import math

from pros_car_py.nav2_utils import calculate_angle_point, cal_distance


class Task2Mission:
    # ---- 狀態 ----
    BRIDGE_APPROACH = "BRIDGE_APPROACH"   # 位姿式 DRIVE_WP 依序開過上橋 waypoints
    VISUAL_CLIMB = "VISUAL_CLIMB"         # 降爪 + 朝熊 bbox 轉向爬橋固定 N 秒
    OBSERVE = "OBSERVE"                   # 停下面向橋上的熊 + 持住觀察 (Locate & Observe 計分),再夾
    GRIP = "GRIP"                         # 前推 + 關爪夾起
    SNAP_DESCEND = "SNAP_DESCEND"         # 夾完以「前方路面」對正 (朝下對側直),再下坡
    DESCEND = "DESCEND"                   # 夾住後沿路面中線過 fat 頂、下階梯到平地
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

        # ---- BRIDGE_APPROACH：位姿式 DRIVE_WP 依序開過上橋 waypoints → VISUAL_CLIMB ----
        # 場景的坡腳位置固定 (per 地圖種子)。在「釘住的 (0,0,0) spawn frame」(reset_map.sh --pin) 量測一串 waypoint,
        # 依序開過 (沿橋軸 +y 開上橋口、置中、不卡橋側),最後一點到位 (或上橋 pose 凍住的 stuck guard) → VISUAL_CLIMB。
        # 量測法:手動把車開到路徑點,echo /amcl_pose 把座標填進此 list。撞牆代表兩點直線跨越不可行地形 → 加中繼點。
        # 格式 [x, y, arrive_dist] (pinned frame)。
        self.WAYPOINTS = [
            [0.899, 0.0, 0.12],   # 對到橋口
            [0.899, 0.2, 0.12],   # 沿橋軸 (+y) 上橋口
            [0.899, 0.4, 0.12],   # 沿橋軸 (+y) 上橋口
            [0.899, 0.5, 0.12],   # 沿橋軸 (+y) 上橋口
            [0.899, 0.616, 0.12],   # 沿橋軸 (+y) 上橋口
            [0.899, 0.74, 0.12],   # 沿橋軸 (+y) 上橋口
            [0.899, 0.800, 0.18],   # 開一點上橋 (置中) → VISUAL_CLIMB
            [0.899, 0.82, 0.12],   # 沿橋軸 (+y) 上橋口
            [0.899, 0.85, 0.12],   # 沿橋軸 (+y) 上橋口
            [0.899, 0.9, 0.12],   # 沿橋軸 (+y) 上橋口
        ]
        # DRIVE_WP controller knobs (中繼段:遠 → 全速;近 waypoint → 70%;方位角大 → 原地轉)
        self.APPROACH_DRIVE_SPEED = 300.0    # 平地全速 (dist >= APPROACH_FAR_DIST 時) — 近 waypoint 仍自動降速 (app 更新 ×2.5)
        self.APPROACH_TURN_GAIN = 7.0        # 角度 → wheel-diff 比例 (deg → speed)
        self.APPROACH_SPIN_DEG = 15.0        # 方位角差 > 此值 → 原地轉 (略嚴,讓接近時更端正)
        self.APPROACH_FAR_DIST = 1.0         # < 此距離降到 70% (避免衝過頭)
        # 上橋後 /amcl_pose 會凍:走最後一個 waypoint 時若位置連 STUCK_TICKS 幀沒動 (pose 凍/卡橋口) → 視為已上橋 → VISUAL_CLIMB。
        self.STUCK_MOVE_TOL = 0.03           # N 幀內位移 < 此值 (m) 視為沒動
        self.STUCK_TICKS = 15                # ~1.5s 沒動 → 觸發
        self.BRIDGE_APPROACH_TIMEOUT = 120.0  # 位姿粗對位全程逾時保險

        # ---- VISUAL_CLIMB：降爪 + 朝熊 bbox 大力轉向、全速爬橋固定 N 秒 → OBSERVE ----
        # 只用兩個東西:① 計時 (VCLIMB_CLIMB_SEC) ② 物件偵測 (bear bbox dx)。看得到熊 → 朝熊 bbox dx 大力轉並全速爬;
        # 看不到熊 → 直走。深度 (t_dist) 只當「近到可追」門檻,不用來判停 (太抖)。不靠 /amcl_pose (橋上 pose 會凍)。
        self.VCLIMB_SPEED = 300.0         # 上橋前進輪速 (full thrust;太慢會卡在坡面) (app 更新 ×2.5)
        self.VCLIMB_CLIMB_SEC = 3         # ★停止條件★:朝熊全速爬這麼多秒 → 直接 OBSERVE (不靠深度/置中提早停)。
        self.VCLIMB_MAX_TRACK_DIST = 4.0  # 只追深度 <= 此距離 (m) 的熊。深度 -1 (過近觸底) 仍算;超過視為遠誘餌不追。
        self.BRIDGE_DX_DEADBAND = 70.0    # b_dx 在 ±此值內不轉向 (DESCEND 沿路面置中用;吸收下坡時質心穩態偏移)。
        # 朝熊轉向 (P 控,扣 deadband):熊偏一邊就大力轉去朝它,不要只滑過去。
        #   實測:gain 0.35/clamp 0.3/deadband 30 → dx=-110 只轉 ~28,車幾乎直走滑過熊。加大如下。
        self.VCLIMB_BEAR_GAIN = 0.9       # 追熊轉向增益 (扣 deadband 後 P 控) → 真的轉去朝熊
        self.VCLIMB_BEAR_CLAMP = 0.7      # 追熊 steer 上限 = 此比例×base (0.7×300=210);放大才轉得動大偏差
        self.VCLIMB_BEAR_DEADBAND = 15.0  # 追熊轉向死區 (px):稍微偏就開始修,朝熊對得更準

        # ---- OBSERVE：停下面向橋上的熊 + 持住觀察 (對齊;Task2 無 Locate&Observe 計分),再進 GRIP ----
        # VISUAL_CLIMB 爬滿固定秒數 → 進 OBSERVE。先原地轉把熊置中 (面向它),再停住持住 OBSERVE_SECONDS 秒,
        # 然後 GRIP。熊看不到 (被爪遮/掉鏡頭) 或對中逾時 → 直接持住。
        self.OBSERVE_SECONDS = 2.0        # 觀察持住秒數 (Task2 無 Locate&Observe 計分,持住只為對齊→2s 足夠)
        self.OBSERVE_ALIGN_PX = 40.0      # 面向熊的置中容差 (|bear dx| <= 此值算面向);收緊 60→40 真的對準才停
        self.OBSERVE_NEAR_DIST = 1.5      # ★只用「近熊」(dist <= 此值,m) 對中★;遠處誘餌熊 (~2m) 忽略,
                                          #   避免 bbox 抖到遠熊 (dx 小) 就誤判「已面向」而不轉。
        self.OBSERVE_FACE_TIMEOUT = 10.0  # 對中熊逾時保險 (s):放寬 6→10,給足時間真的轉到熊置中再持住

        # ---- GRIP：到頂/過橋後,前頂一段把熊鏟進低位開爪中 + 關爪夾起 (爪已在 VISUAL_CLIMB 降下且全程開著) ----
        self.GRIP_PRESS_SPEED = 150.0     # 關爪前的前頂輪速 (full thrust;坡頂要更大力頂得動、把熊鏟進爪);0=純煞停 (app 更新 ×2.5)
        self.GRIP_PRESS_SEC = 1.0         # 關爪前先前頂這麼久 (s):熊掉出鏡頭時常在爪前 ~0.8m,需多頂一段才鏟進爪

        # ---- SNAP_DESCEND：夾完後以「前方路面 (road_info delta_x)」對正,朝下對側直,再 DESCEND ----
        # 橋上 /amcl_pose 會凍,不能用 yaw;改用穩定可見的路面質心 (r_dx→0=朝正前方下坡方向) 原地轉對正。
        # 坡頂 fat part 上原地轉可能轉不太動 → SNAP_DESCEND_TIMEOUT 兜底直接進 DESCEND (下坡時 road 轉向會邊走邊修)。
        self.SNAP_ROAD_PX = 45.0          # 路面置中容差 (|road dx| <= 此值算對正)
        self.SNAP_ROAD_CONFIRM = 3        # 連續 N 幀路面置中才算對正 → DESCEND
        self.SNAP_DESCEND_TIMEOUT = 6.0   # 對正逾時保險 (s) → 仍進 DESCEND

        # ---- DESCEND：夾住後全速前進,翻過坡頂 fat part、下階梯到對側平地,順便下橋讓 /amcl_pose 解凍 ----
        # 沿「路面 (road_info delta_x)」中線持續前進 (橋面看下對側階梯時偵測不到,road 才穩);全速衝過 fat part + 下階梯。
        # 結束條件:「路面占滿畫面 (area_frac >= DESCEND_ROAD_AREA)」才算真的下到地面 —— 坡頂就看得到遠處路面 (~0.33),
        # 故門檻要拉高 (~0.55),否則會在 fat part 上就誤判到底、停住卡死。MIN_SEC 前不可結束,MAX_SEC 兜底。
        self.DESCEND_MIN_SEC = 0.5        # commit 窗 (s):此前全速衝過 fat part + 階梯,視覺尚不可結束 (防坡頂誤判)。
                                          #   實測下坡只 ~2-3s,故縮短 4→2。
        self.DESCEND_MAX_SEC = 5.0        # 結束逾時 (s) 兜底:實測 ~2-3s 已下到地面,9s 太久 → 縮到 5s。
        self.DESCEND_ROAD_AREA = 0.55     # 路面 area_frac >= 此值 → 路面占滿畫面=已下到地面 → 結束 (備用)。
                                          #   實測下坡時 road mask 反而會掉到 0 (不會占滿),故此判底常失效。
        self.DESCEND_DONE_CONFIRM = 5     # 連續 N 幀滿足「road 占滿」條件才結束 (濾 segmentation 抖動)
        # ★主要結束依據★:下橋後 bridge+road 兩個 mask 都偵測不到 (F=0) 並持續 → 已駛離橋/階梯到平地。
        #   實測 road area 0.24→0.17→0 後一直 0,正是「離開橋面」的訊號。MIN_SEC 後才允許,過此連續 N 幀 → 結束。
        self.DESCEND_LOST_CONFIRM = 8     # 連續 N 幀 bridge+road 皆 F=0 (~0.8s) → 判定已離橋到平地 → 結束

        self.DESCEND_STEER_GAIN = 0.35    # 沿路面中線置中的差速增益 (扣 BRIDGE_DX_DEADBAND 後 P 控,= VCLIMB 同款)
        self.DESCEND_STEER_CLAMP = 0.3    # steer 夾在 ±此比例*base

        # ---- RETURN (位姿式返航) ----
        # 先「繞行」過橋的中繼點 (RETURN_WAYPOINTS) 再直線回起點 —— 避免直線穿回橋 (背著熊翻陡橋會失敗/卡側牆)。
        # RETURN_WAYPOINTS = [[x, y, arrive_dist], ...]:在 pinned (reset_map.sh --pin) frame 量到的繞行點,依序開過,
        # 路由繞過橋的某一側回到起點。空 list = 不繞行、直線回起點 (舊行為)。座標待在 sim 量測後填入。
        # 量測自下橋後的 pinned frame (繞橋的 +x 側通道回起點;orientation 不需要,follower 自算 bearing)。
        # arrive_dist=0.20:點間距密,0.40 會離點還 ~0.4m 就判到位切角 (撞橋角);0.20 夠緊不切角 (各段 >0.25)、
        # 又配合提速後 (RETURN_DRIVE_SPEED=200) 不會因每幀位移大而衝過 0.15 的小半徑判不到、繞著點打轉。
        self.RETURN_WAYPOINTS = [
            [0.83, 3.03, 0.20],   # 下橋落點 (遠端)
            [0.914, 3.011, 0.20],   # 繞到橋遠端外
            [1.4, 3.011, 0.20],
            [1.836, 3.011, 0.20],  
            [1.836, 2.074, 0.20],   # 沿 +x 側通道往 -y (與橋平行,不穿橋)
            [1.862, 1.606, 0.20],
            [1.862, 1.306, 0.20],
            [1.862, 1.006, 0.20],
            [1.862, 0.359, 0.20],   # 轉向:往 -x 回起點側
            [1.149, 0.308, 0.20],
            [0.562, 0.0359, 0.20],  # 接近起點 → 之後 GOTO_START 直線回 (0,0)
            [0.562, 0.024, 0.20],
        ]
        self.RETURN_ARRIVE_DIST = 0.40
        self.RETURN_ARRIVE_CONFIRM = 4
        self.RETURN_TIMEOUT = 150.0      # 繞行較長,逾時放寬
        self.RETURN_SPIN_DEG = 20.0
        self.RETURN_DRIVE_SPEED = 500.0  # 平地返航全速 (app 更新 ×2.5;近起點仍有減速 crawl)。base 大於 GAIN*ang 才不會被 clamp 成「單輪歸零」原地頂、
                                         #   卡在繞行點 (實測 ang=14° 時 7*14=98,base 太小→turn 夾到 base→內輪=0 卡死)。
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
        print("[Task2] 任務開始 (粗對位 → 視覺對準近熊 → snap 90° → 過橋 → 夾取遠熊 → 返航)。")
        self.start_pose = self._capture_start_pose()
        if self.start_pose is None:
            print("[Task2] ⚠️ 取不到 /amcl_pose，BRIDGE_APPROACH/RETURN 將失效。")
        else:
            print(f"[Task2] 起點記錄為 {self.start_pose} (yaw={math.degrees(self.start_yaw):.0f}°)")

        approach_deadline = time.time() + self.BRIDGE_APPROACH_TIMEOUT
        # BRIDGE_APPROACH (pose-based, DRIVE_WP) 狀態
        wp_idx = 0                           # 當前上橋 waypoint index (進 list 之後變 ≥ len → VISUAL_CLIMB)
        # 上橋末點 stuck guard (pose 凍住即視為已上橋)
        stuck_anchor_xy = None               # 「卡死」基準位置
        stuck_anchor_tick = -10000           # anchor 設置的 tick (用以計時)
        # VISUAL_CLIMB 狀態 (進 VISUAL_CLIMB 第一幀初始化)
        vclimb_scoop_prepared = False        # 鏟爪只在進 VISUAL_CLIMB 時降一次
        vclimb_entry_time = 0.0
        vclimb_dbg = 0
        # OBSERVE 狀態 (進 OBSERVE 時初始化)
        observe_entry = 0.0                  # 進 OBSERVE 的時間 (對中熊逾時用)
        observe_centered = False             # 是否已面向熊 (或放棄對中) → 開始持住
        observe_start = 0.0                  # 持住計時起點
        observe_dbg = 0
        # SNAP_DESCEND 狀態 (進 SNAP_DESCEND 時於 GRIP 設定)
        snap_descend_entry = 0.0
        snap_road_streak = 0                 # 連續路面置中幀數
        snap_descend_dbg = 0
        # DESCEND 狀態 (進 DESCEND 時於 SNAP_DESCEND 設定)
        descend_entry_time = 0.0
        descend_dbg = 0
        descend_done_streak = 0              # 連續滿足「路面占滿畫面 (到地面)」條件的幀數
        descend_lost_streak = 0              # 連續 bridge+road 皆 F=0 的幀數 (→ 已離橋到平地)
        # RETURN 狀態
        return_wp_idx = 0                    # 當前繞行 waypoint index (>= len(RETURN_WAYPOINTS) → 直線回起點)
        return_stuck_anchor = None           # 繞行卡死偵測基準位置 (只在 arc 直行段計,轉向段不算)
        return_stuck_tick = 0
        dock_dbg = 0
        dbg_tick = 0

        while not stop_event.is_set():
            dbg_tick += 1
            # --- 感測讀取 ---
            rinfo = self.data_processor.get_road_info()            # [found,dx,area] or None
            r_found = bool(rinfo and rinfo[0] == 1)
            r_dx = rinfo[1] if rinfo else 0.0
            r_area = rinfo[2] if rinfo and len(rinfo) > 2 else 0.0   # 對側平地路面占比 (DESCEND 判到底)

            # bridge_info / road_info 僅供 debug 列;導引改用 bear 視覺伺服 (VISUAL_CLIMB)。
            binfo = self.data_processor.get_bridge_info()          # or None
            b_found = bool(binfo and binfo[0] == 1)
            b_dx = binfo[1] if binfo else 0.0
            # bear: [found, distance, delta_x, area, bottom]。VISUAL_CLIMB 吃它 (朝熊 bbox dx 轉向)。
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

                # DRIVE_WP → 依序開過上橋 waypoints (不對齊 yaw),末點到位 / 上橋 pose 凍住 → VISUAL_CLIMB
                wp_x, wp_y, wp_arrive_dist = self.WAYPOINTS[wp_idx]
                wp_target = [wp_x, wp_y]
                dist_wp = cal_distance(car_xy, wp_target)
                last_wp = (wp_idx == len(self.WAYPOINTS) - 1)
                if dist_wp < wp_arrive_dist:
                    wp_idx += 1
                    stuck_anchor_xy = None      # 換點 → 重置 stuck 基準
                    if wp_idx >= len(self.WAYPOINTS):
                        # 上橋 waypoints 已把車帶到橋上中線、對到橋軸 → 直接視覺伺服上橋
                        self._transition(self.VISUAL_CLIMB,
                                         f"通過最後上橋 WP (dist={dist_wp:.2f}m) → 視覺伺服上橋")
                        continue
                    print(f"[Task2] 通過上橋 WP {wp_idx}/{len(self.WAYPOINTS)} "
                          f"(dist={dist_wp:.2f}m) → WP {wp_idx + 1}")
                else:
                    # 上橋後 /amcl_pose 會凍:走最後一個上橋點時若位置連 STUCK_TICKS 幀沒動 (pose 凍/卡橋口),
                    # 視為「已上到橋上」→ 交給 VISUAL_CLIMB 視覺伺服爬,避免卡在 DRIVE_WP 永遠到不了末點。
                    if last_wp:
                        if stuck_anchor_xy is None or cal_distance(car_xy, stuck_anchor_xy) > self.STUCK_MOVE_TOL:
                            stuck_anchor_xy = list(car_xy)
                            stuck_anchor_tick = dbg_tick
                        elif dbg_tick - stuck_anchor_tick >= self.STUCK_TICKS:
                            self._transition(self.VISUAL_CLIMB,
                                             f"上橋 WP 位置凍住 ({self.STUCK_TICKS}f 沒動,疑上橋 pose 凍) → 視覺伺服上橋")
                            continue
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

                if time.time() > approach_deadline:
                    self._transition(self.DONE, "BRIDGE_APPROACH 逾時")

            # ---------------- VISUAL_CLIMB (降爪 → 朝熊 bbox 轉向爬橋固定 N 秒 → OBSERVE) ----------------
            # 只用兩個東西:① 計時 (爬滿 VCLIMB_CLIMB_SEC 秒就停) ② 物件偵測 (bear bbox dx,大力轉去朝熊)。
            # 看得到熊 → 朝熊 bbox dx 轉並全速爬;看不到熊 → 直走 (不靠橋面 segmentation)。深度只當「是否近到可追」門檻。
            elif self.state == self.VISUAL_CLIMB:
                # 1) 進場先把鏟爪降到貼地+開爪 (阻塞一次)。爪降下=熊會被一路鏟進爪中。
                if not vclimb_scoop_prepared:
                    self._publish("STOP")
                    self.arm_controller.scoop_pose()   # 阻塞：降臂 + 開爪
                    vclimb_scoop_prepared = True
                    vclimb_entry_time = time.time()
                elapsed = time.time() - vclimb_entry_time

                # 2) 停止 = 爬滿固定秒數 → OBSERVE (不靠深度/置中提早停,避免被深度抖動提早/過晚停)。
                if elapsed > self.VCLIMB_CLIMB_SEC:
                    self._publish("STOP")
                    observe_entry = time.time(); observe_centered = False
                    self._transition(self.OBSERVE,
                                     f"朝熊爬橋 {self.VCLIMB_CLIMB_SEC:.0f}s 到 → 停下觀察")
                    continue

                # 3) 轉向:看得到熊 (深度可追) → 朝熊 bbox dx 大力轉並全速爬;看不到熊 → 直走。
                if t_found and t_dist <= self.VCLIMB_MAX_TRACK_DIST:
                    steer = self._bridge_center_steer(self.VCLIMB_SPEED, t_dx,
                                                      self.VCLIMB_BEAR_GAIN,
                                                      self.VCLIMB_BEAR_CLAMP,
                                                      self.VCLIMB_BEAR_DEADBAND)
                    self._arc(self.VCLIMB_SPEED, steer)
                    steer_str = f"→bear dx={t_dx:+.0f} dist={t_dist:.2f} steer={steer:+.0f}"
                else:
                    self._arc(self.VCLIMB_SPEED, 0.0)
                    steer_str = "no-bear straight"

                vclimb_dbg += 1
                if vclimb_dbg % 10 == 1:
                    print(f"[Task2] VISUAL_CLIMB t={elapsed:.1f}/{self.VCLIMB_CLIMB_SEC:.0f}s "
                          f"bear(F={int(t_found)} dist={t_dist:.2f} dx={t_dx:+.0f}) → {steer_str}")

            # ---------------- OBSERVE (停下面向橋上的熊 + 持住觀察 Locate & Observe,再夾) ----------------
            elif self.state == self.OBSERVE:
                if not observe_centered:
                    # 面向階段:只用「近熊」(dist <= OBSERVE_NEAR_DIST) 對中,忽略遠處誘餌熊。
                    # 近熊偏太多 → 轉去對中;近熊已置中 或 對中逾時 → 開始持住;
                    # 只有遠熊/暫時看不到近熊 → 停住「等」,不因此提早結束面向 (修:抖到遠熊就誤判已面向)。
                    t_near = t_found and 0.0 < t_dist <= self.OBSERVE_NEAR_DIST
                    faced_timeout = (time.time() - observe_entry) >= self.OBSERVE_FACE_TIMEOUT
                    if t_near and abs(t_dx) > self.OBSERVE_ALIGN_PX and not faced_timeout:
                        self._publish("CLOCKWISE_ROTATION_SLOW" if t_dx > 0
                                      else "COUNTERCLOCKWISE_ROTATION_SLOW")
                        action = f"FACE(bear_dx={t_dx:+.0f})"
                    elif (t_near and abs(t_dx) <= self.OBSERVE_ALIGN_PX) or faced_timeout:
                        observe_centered = True
                        observe_start = time.time()
                        self._publish("STOP")
                        action = "FACED → 持住" + ("(逾時)" if faced_timeout else "")
                    else:
                        self._publish("STOP")
                        action = "WAIT(無近熊)"
                else:
                    # 持住階段:停住觀察 OBSERVE_SECONDS 秒 → GRIP
                    self._publish("STOP")
                    held = time.time() - observe_start
                    if held >= self.OBSERVE_SECONDS:
                        self._transition(self.GRIP, f"觀察完成 (面向熊 + 持住 {held:.1f}s) → 關爪夾起")
                        continue
                    action = f"HOLD({held:.1f}/{self.OBSERVE_SECONDS:.1f}s)"
                observe_dbg += 1
                if observe_dbg % 10 == 1:
                    print(f"[Task2] OBSERVE bear(F={int(t_found)} dist={t_dist:.2f} dx={t_dx:+.0f}) → {action}")

            # ---------------- GRIP (前頂把熊壓進爪中 + 關爪夾起;爪已在 VISUAL_CLIMB 降下) ----------------
            elif self.state == self.GRIP:
                # 前頂一段：抗坡面後滑、把熊壓進爪中。★每幀重發輪速★,確保整段真的持續前進
                # (單發 publish + sleep 指令不會維持,車不會動 → 鏟不到熊)。
                if self.GRIP_PRESS_SPEED > 0:
                    print(f"[Task2] GRIP：前頂 {self.GRIP_PRESS_SEC:.1f}s (壓熊進爪/抗後滑) 後關爪")
                    press_start = time.time()
                    while time.time() - press_start < self.GRIP_PRESS_SEC:
                        if stop_event.is_set():
                            break
                        self.ros_communicator.publish_raw_car_control([self.GRIP_PRESS_SPEED] * 4)
                        time.sleep(self.TICK)
                print("[Task2] GRIP：關爪夾住 bear + 抬起搬運")
                self.arm_controller.scoop_grab()   # 阻塞：關爪 + 等黏合 + 抬起
                self._publish("STOP")
                snap_descend_entry = time.time()
                snap_road_streak = 0
                snap_descend_dbg = 0
                self._transition(self.SNAP_DESCEND, "夾取完成 → 以前方路面對正 (朝下對側直)")

            # ---------------- SNAP_DESCEND (夾完以「前方路面」對正朝向,再下坡;橋上 pose 凍,不用 yaw) ----------------
            elif self.state == self.SNAP_DESCEND:
                def _to_descend(reason):
                    nonlocal descend_entry_time, descend_dbg, descend_done_streak, descend_lost_streak
                    self._publish("STOP")
                    descend_entry_time = time.time()
                    descend_dbg = 0
                    descend_done_streak = 0
                    descend_lost_streak = 0
                    self._transition(self.DESCEND, reason)

                if time.time() - snap_descend_entry > self.SNAP_DESCEND_TIMEOUT:
                    _to_descend("對正逾時 (坡頂原地轉不太動) → 直接下坡 (下坡再以橋面/路面邊走邊修)")
                    continue
                # 對正基準:優先用「橋面 segmentation 質心」(b_dx,= 上橋同款);橋面看不到時退用前方路面 (r_dx)。
                if b_found:
                    c_dx, c_found, c_lbl = b_dx, True, "bridge"
                elif r_found:
                    c_dx, c_found, c_lbl = r_dx, True, "road"
                else:
                    c_dx, c_found, c_lbl = 0.0, False, "none"
                if c_found and abs(c_dx) > self.SNAP_ROAD_PX:
                    # 還沒置中 → 原地轉把它轉到正前方 (c_dx>0 在右 → 順時針)
                    snap_road_streak = 0
                    self._publish("CLOCKWISE_ROTATION_SLOW" if c_dx > 0
                                  else "COUNTERCLOCKWISE_ROTATION_SLOW")
                    action = f"CENTER({c_lbl}_dx={c_dx:+.0f})"
                elif c_found:
                    snap_road_streak += 1
                    self._publish("STOP")
                    action = f"OK({c_lbl}_dx={c_dx:+.0f}) {snap_road_streak}/{self.SNAP_ROAD_CONFIRM}"
                    if snap_road_streak >= self.SNAP_ROAD_CONFIRM:
                        _to_descend(f"{c_lbl} 對正完成 → 下對側")
                        continue
                else:
                    # 這幀橋面/路面都沒看到 → 原地等 (timeout 兜底)
                    snap_road_streak = 0
                    self._publish("STOP")
                    action = "WAIT_MASK"
                snap_descend_dbg += 1
                if snap_descend_dbg % 10 == 1:
                    print(f"[Task2] SNAP_DESCEND bridge(F={int(b_found)} dx={b_dx:+.0f}) "
                          f"road(F={int(r_found)} dx={r_dx:+.0f}) → {action}")

            # ---------------- DESCEND (沿路面中線翻 fat 頂、下階梯到平地;路面占滿畫面=到地面 → RETURN) ----------------
            elif self.state == self.DESCEND:
                elapsed = time.time() - descend_entry_time

                def _to_return(reason):
                    # 下橋結束 → 位姿式返航 (橋頂 GRIP 已夾住熊,一路背著回起點)。
                    self._publish("STOP")
                    if self.start_pose is not None:
                        print(f"[Task2] RETURN 目標(起點) = {self.start_pose}")
                    self._return_entry_time = time.time()
                    self._arrive_streak = 0
                    self._return_dbg = 0
                    self._transition(self.RETURN, reason)

                # 兜底逾時:視覺沒判到底也最多前進這麼久 → 返航
                if elapsed > self.DESCEND_MAX_SEC:
                    _to_return("下坡逾時 (MAX_SEC) → 直接返航")
                    continue
                # 到底判定 (過 commit 窗後才允許):路面占滿畫面 (area_frac 高) = 已下到地面,連續 N 幀 → RETURN。
                #   坡頂就看得到遠處路面 (~0.33),故門檻拉高 (~0.55),才不會在 fat part 上就誤判到底卡死。
                if elapsed > self.DESCEND_MIN_SEC:
                    # ★主要判底★:bridge+road 兩個 mask 都偵測不到並持續 → 已駛離橋/階梯到平地 → 結束。
                    if not b_found and not r_found:
                        descend_lost_streak += 1
                    else:
                        descend_lost_streak = 0
                    if descend_lost_streak >= self.DESCEND_LOST_CONFIRM:
                        _to_return("橋+路面 mask 皆消失 (已離橋到平地) → 直接返航")
                        continue
                    # 備用判底:路面占滿畫面 (實測常失效,但保留)。
                    if r_found and r_area >= self.DESCEND_ROAD_AREA:
                        descend_done_streak += 1
                    else:
                        descend_done_streak = 0
                    if descend_done_streak >= self.DESCEND_DONE_CONFIRM:
                        _to_return("路面占滿畫面 (已下到地面) → 直接返航")
                        continue

                # 全速衝過 fat part + 下階梯 (不減速,避免卡在坡頂的階差)。轉向基準:優先沿「橋面中線 (b_dx,= 上橋同款)」
                # 置中;橋面看不到 (下階梯時 segmentation 常偵測不到) 才退用前方路面 (r_dx);兩者都沒有 → 直走。
                base = self.VCLIMB_SPEED
                if b_found:
                    steer = self._bridge_center_steer(base, b_dx,
                                                      self.DESCEND_STEER_GAIN, self.DESCEND_STEER_CLAMP)
                    self._arc(base, steer)
                    ds = f"bridge_dx={b_dx:+.0f} steer={steer:+.0f}"
                elif r_found:
                    steer = self._bridge_center_steer(base, r_dx,
                                                      self.DESCEND_STEER_GAIN, self.DESCEND_STEER_CLAMP)
                    self._arc(base, steer)
                    ds = f"road_dx={r_dx:+.0f} steer={steer:+.0f}"
                else:
                    self._arc(base, 0.0)
                    ds = "no-mask straight"
                descend_dbg += 1
                if descend_dbg % 10 == 1:
                    print(f"[Task2] DESCEND t={elapsed:.1f}/{self.DESCEND_MAX_SEC:.0f}s base={base:.0f} "
                          f"bridge(F={int(b_found)} dx={b_dx:+.0f}) road(F={int(r_found)} area={r_area:.3f}) "
                          f"lost={descend_lost_streak}/{self.DESCEND_LOST_CONFIRM} "
                          f"done={descend_done_streak}/{self.DESCEND_DONE_CONFIRM} → {ds}")

            # ---------------- RETURN (繞行 waypoints 繞過橋 → 直線回起點) ----------------
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

                # ---- Phase A: DETOUR_WP → 依序開過繞行中繼點 (繞過橋的一側,不直線穿回橋);背著熊不放下 ----
                if return_wp_idx < len(self.RETURN_WAYPOINTS):
                    wp_x, wp_y, wp_arrive = self.RETURN_WAYPOINTS[return_wp_idx]
                    wp_target = [wp_x, wp_y]
                    dwp = cal_distance(car, wp_target)
                    if dwp < wp_arrive:
                        return_wp_idx += 1
                        return_stuck_anchor = None
                        if return_wp_idx >= len(self.RETURN_WAYPOINTS):
                            print(f"[Task2] RETURN 通過最後繞行點 (dist={dwp:.2f}m) → 直線回起點")
                        else:
                            print(f"[Task2] RETURN 通過繞行點 {return_wp_idx}/{len(self.RETURN_WAYPOINTS)} "
                                  f"(dist={dwp:.2f}m) → 下一繞行點")
                    else:
                        ang = calculate_angle_point(o.z, o.w, car, wp_target)
                        if abs(ang) > self.RETURN_SPIN_DEG:
                            self._publish("COUNTERCLOCKWISE_ROTATION_SLOW" if ang > 0
                                          else "CLOCKWISE_ROTATION_SLOW")
                            action = f"WP_SPIN({ang:+.0f}°)"
                            return_stuck_anchor = None      # 原地轉向時位置本就不變,不算卡死
                        else:
                            # arc 直行段:位置連 STUCK_TICKS 幀沒動 = 真的卡住 (撞到/pose 凍,非轉向) → 跳下一繞行點,
                            # 不要乾耗到 RETURN_TIMEOUT。(只在 arc 段判,故不會誤把正常原地轉向當卡死。)
                            if return_stuck_anchor is None or cal_distance(car, return_stuck_anchor) > self.STUCK_MOVE_TOL:
                                return_stuck_anchor = list(car)
                                return_stuck_tick = dbg_tick
                            elif dbg_tick - return_stuck_tick >= self.STUCK_TICKS:
                                print(f"[Task2] RETURN 繞行點 {return_wp_idx + 1}/{len(self.RETURN_WAYPOINTS)} "
                                      f"卡死 ({self.STUCK_TICKS}f arc 沒動,dist={dwp:.2f}) → 跳下一點")
                                return_wp_idx += 1
                                return_stuck_anchor = None
                                time.sleep(self.TICK)
                                continue
                            base = self.RETURN_DRIVE_SPEED
                            if dwp < self.GOTO_FAR_DIST:
                                base *= 0.7
                            turn = max(-base, min(base, self.RETURN_TURN_GAIN * ang))
                            left = base - turn
                            right = base + turn
                            self.ros_communicator.publish_raw_car_control([left, right, left, right])
                            action = f"WP_ARC(base={base:.0f},turn={turn:+.0f})"
                        self._return_dbg += 1
                        if self._return_dbg % 10 == 1:
                            print(f"[Task2] RETURN 繞行 WP {return_wp_idx + 1}/{len(self.RETURN_WAYPOINTS)} "
                                  f"car=({car[0]:.2f},{car[1]:.2f}) dist={dwp:.2f} ang={ang:+.0f}° → {action}")
                    time.sleep(self.TICK)
                    continue

                # ---- Phase B: GOTO_START → 繞行完成 (或無繞行點) → 直線回起點 + 放下熊 ----
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

    def _bridge_center_steer(self, base, dx, gain, clamp, deadband=None):
        """依某個畫面偏移 dx (VISUAL_CLIMB 傳 bear bbox dx、DESCEND 傳橋面/路面質心 dx) 算置中差速。
        先扣 deadband (預設 BRIDGE_DX_DEADBAND;VISUAL_CLIMB 傳較緊的 VCLIMB_BEAR_DEADBAND 大力朝熊),
        扣完只對剩餘偏差做 P 控制,再 clamp 限制最大彎度。"""
        db = self.BRIDGE_DX_DEADBAND if deadband is None else deadband
        err = (dx - math.copysign(db, dx)) if abs(dx) > db else 0.0
        steer = gain * err
        cap = clamp * base
        return max(-cap, min(cap, steer))

    @staticmethod
    def _norm_angle(a):
        """把角度正規化到 [-π, π] — 等價於 atan2(sin,cos),在 ±180° 邊界連續。
        RETURN 算 yaw 誤差時用此函式,避免雜訊在 ±π 邊界正負跳動造成原地抖動 (wiggle)。"""
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
