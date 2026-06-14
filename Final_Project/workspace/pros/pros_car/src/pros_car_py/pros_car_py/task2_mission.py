"""
Task2Mission — Final Project Task 2 自動任務
  (位姿粗對位 → 視覺伺服上橋走近橋上的熊並鏟入 → 關爪夾起 → 返航)
=======================================================================

關鍵前提 (場景)：橋上有一隻目標熊,橋外另有散落的誘餌熊。兩道防線只鎖橋上那隻:
  ① YOLO 端 YOLO_TARGET_PICK=onbridge (object_detect.py 訂閱 /yolo/bridge_info,只發布「橫向對齊橋面」的熊)。
  ② 任務端 SNAP_90 先轉到橋軸,橋面置中、誘餌熊被轉出畫面 → VISUAL_CLIMB 視野裡自然只剩橋上那隻。

策略 (無 IMU；地圖種子固定)：
  位姿粗導航到坡腳 → SNAP_90 轉到橋軸 (誘餌轉出畫面) → bear 視覺「先對準、再前進」走近橋上的熊
  (橋上 /amcl_pose 會凍,故爬橋只用視覺、不用 pose)。

完整流程 (對應計分項目)：

  BRIDGE_APPROACH → 位姿式 DRIVE_WP:依序開過 WAYPOINTS (上橋路徑:對到橋口 → 沿橋軸 +y 開一點上橋,置中、不卡
                    橋側),末點到位 (或上橋 pose 凍住) → 直接 VISUAL_CLIMB。需先 reset_map.sh --pin 釘住 map frame。
                    WAYPOINTS 空時退回舊路徑:DRIVE_DOCK 開到 (DOCK_X, DOCK_Y) → SNAP_90。
  SNAP_90 (fallback) → 上橋前確保兩條件:①橋面置中 ②朝向 ~90°。(A) 原地轉把「橋面 segmentation 質心」轉到畫面正中,
                    車自然朝 ~90° (yaw sanity 確認);成立 N 幀 → (B) 沿橋軸直行 SNAP_FWD_SEC 秒貼上橋口 → VISUAL_CLIMB。
                    (WAYPOINTS 上橋路徑已取代此段;僅 WAYPOINTS 空時用。)
  VISUAL_CLIMB   → 進場先 scoop_pose() 把鏟爪降下+開爪 (阻塞一次),之後沿「橋面中線 (bridge_dx)」全速直行過橋
                    (bridge_dx 穩,bear 深度太抖不用來轉向)。bear 只判時機:走近到 VCLIMB_OBSERVE_DIST (仍清楚可見),
                    或「曾靠近 (<=COMMIT_DIST) 後持續看不到=已鏟入爪中」,或 VCLIMB_TIMEOUT (到頂) → OBSERVE。
  OBSERVE        → 停下原地轉把橋上的熊置中 (面向它),再停住持住 OBSERVE_SECONDS 秒 (Locate & Observe 計分) → GRIP。
                    熊被低位爪遮/掉鏡頭或對中逾時 → 直接持住。
  GRIP           → 小幅前頂 (把熊鏟進爪中、抗坡面後滑) + scoop_grab() 關爪+抬起。
  SNAP_DESCEND   → 夾完原地轉對正 (朝下對側直):優先用「橋面質心 (b_dx,= 上橋同款)」,橋面看不到才退用前方路面
                    (r_dx);橋上 pose 凍,不用 yaw。對正/逾時 → DESCEND。
  DESCEND        → 夾住後全速前進,優先沿「橋面中線 (b_dx)」置中、橋面看不到才退用路面 (r_dx),翻過坡頂 fat part、
                    下階梯;不減速 (避免卡在階差)。
                    結束:路面占滿畫面 (road area_frac >= DESCEND_ROAD_AREA = 已下到地面) 連續 N 幀,MAX_SEC 兜底;
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
    BRIDGE_APPROACH = "BRIDGE_APPROACH"   # 位姿粗對位 (DRIVE_WP/DRIVE_DOCK)
    SNAP_90 = "SNAP_90"                   # 原地轉到橋軸 (~90°),橋面置中、誘餌熊轉出畫面
    VISUAL_CLIMB = "VISUAL_CLIMB"         # 降爪 + 視覺伺服上橋走近橋上的熊並鏟入
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
        # 格式: [x, y, arrive_dist] (pinned frame)。非空 → BRIDGE_APPROACH 走 DRIVE_WP 依序開過這些點,
        # 最後一點到位 → 直接進 VISUAL_CLIMB (跳過 DRIVE_DOCK/SNAP_90)。空 = 舊行為 (DRIVE_DOCK → SNAP_90)。
        #
        # 「上橋」路徑 (實測,下面 DOCK 同一 pinned frame):從對到橋口的點沿橋軸 (+y) 直行、開一點上橋,
        # 讓車置中上橋、不卡在橋側 (取代原本 SNAP_90 原地轉+貼坡口,那一段會卡側邊)。arrive_dist 取小 (點間距很短),
        # 末點較大以涵蓋上橋後 pose 凍的範圍 (DRIVE_WP 另有「位置凍住」guard 兜底 → VISUAL_CLIMB)。
        self.WAYPOINTS = [
            [0.899, 0.0, 0.12],   # 對到橋口 (= dock 位置)
            [0.899, 0.2, 0.12],   # 沿橋軸 (+y) 上橋口
            [0.899, 0.4, 0.12],   # 沿橋軸 (+y) 上橋口
            [0.899, 0.616, 0.12],   # 沿橋軸 (+y) 上橋口
            [0.899, 0.800, 0.18],   # 開一點上橋 (置中) → VISUAL_CLIMB
        ]
        # 實測最終 docking pose —— 在「釘住的 (0,0,0) spawn frame」量測 (from /amcl_pose;
        # quat z=0.7115656192, w=0.7026196479)。此 frame 由 reset ritual 維持:Unity restart →
        # 重啟 robot_bringup (重置 scan_matcher odom→0) → 重啟 slam (map 重錨到 odom=0),車在 spawn。
        # 跑過 ritual 後此座標跨 session 可重現,不需每次重量。若沒跑 ritual 就直接用,map frame 會浮動、
        # 座標失效 (見 docs/superpowers/specs/2026-06-11-task2-localization-pin-spawn-design.md)。
        self.DOCK_X = 0.9022             # 最終 dock x (含 yaw 對齊)
        self.DOCK_Y = 0.3782             # 最終 dock y
        # yaw = 2*atan2(z, w) = 2*atan2(0.71157, 0.70262) ≈ 1.5834 rad (90.7°)。不在 ±180° 邊界。
        self.DOCK_YAW_RAD = 1.5834           # rad ≈ 90.7°
        # Controller knobs — 三段速度 (FAR / MID / NEAR) 讓終點精準對齊不衝過頭
        self.APPROACH_DRIVE_SPEED = 300.0    # 平地全速 (dist >= APPROACH_FAR_DIST 時) — 近 waypoint 仍自動降速 (app 更新 ×2.5)
        self.APPROACH_TURN_GAIN = 7.0        # 角度 → wheel-diff 比例 (deg → speed)
        self.APPROACH_SPIN_DEG = 15.0        # 方位角差 > 此值 → 原地轉 (略嚴,讓接近時更端正)
        self.APPROACH_FAR_DIST = 1.0         # < 此距離降到 70% (避免衝過頭)
        self.APPROACH_NEAR_DIST = 0.40       # < 此距離再降到 35% (crawl,精準對位)
        self.DOCK_ARRIVE_DIST = 0.10         # 到位距離容差 (m). 0.10 在此幾何下達不到 (go-to-point 近目標時
                                             # bearing 病態,車會畫圈/卡死);位置不需極精準 —— 朝向與精細置中
                                             # 都交給 VISUAL_CLIMB 的 bear 視覺 (align-then-go),故放寬即可。
        self.DOCK_ARRIVE_CONFIRM = 4         # 連續 N 幀達標才視為到位 (略增,濾抖動)
        # 卡死偵測 (near-goal stuck guard): 在 STUCK_NEAR_DIST 範圍內,若位置 N 幀內幾乎沒動
        # → 視為實際到位 (可能被小階差/curb 卡住,前進不了那剩下幾公分)。
        self.STUCK_NEAR_DIST = 0.35          # 此距離內才啟動 stuck 偵測。需 > DOCK_ARRIVE_DIST,涵蓋近目標的
                                             #   stall 帶 (實測車會在 ~0.24m 卡住),卡住即視為到位 → VISUAL_CLIMB。
        self.STUCK_MOVE_TOL = 0.03           # N 幀內位移 < 此值 (m) 視為沒動
        self.STUCK_TICKS = 15                # ~1.5s 沒動 → 觸發
        self.BRIDGE_APPROACH_TIMEOUT = 120.0  # 位姿粗對位全程逾時保險

        # ---- SNAP_90：到 dock 後原地轉到橋軸 (~DOCK_YAW_RAD≈90.7°,pinned frame) ----
        # 為何需要 (實測):dock 處車朝向不定,橋面常落在畫面邊緣 (bridge_dx≈-290),橋外的誘餌熊反而置中/較近,
        # 純視覺/最近挑選會選到誘餌。先轉到橋軸 → 橋面置中、誘餌熊被轉出畫面 → VISUAL_CLIMB 自然只看到橋上那隻。
        # 坡腳處 /amcl_pose 仍有效 (上橋後才凍),故可用位姿精轉。
        # 進 VISUAL_CLIMB 前要同時滿足兩條件 (才不會上橋爬偏/追錯熊):
        #   ① 橋面置中 (|bridge_dx| <= SNAP_CENTER_PX)  ② 朝向接近 90° 橋軸。
        # 做法:原地轉「把橋面 segmentation 質心 (穩定) 轉到畫面正中」。橋面置中時,橋中線上的熊也跟著置中,
        # 且車自然朝向 ~90°;再用 yaw sanity 確認。兩條件都成立 N 幀才上橋。(不用 bear 深度——太抖。)
        self.SNAP_CENTER_PX = 70              # 橋面置中容差 (px):|bridge_dx| <= 此值算置中
        self.SNAP_YAW_SANITY = math.radians(25)  # 置中後 yaw 需在 DOCK_YAW±此範圍 (sanity;排除異常)
        self.SNAP_YAW_CONFIRM = 3             # 連續 N 幀「置中且 yaw 合理」才算對準 (濾抖)
        self.SNAP_90_TIMEOUT = 25.0           # 對準逾時保險 (s) → 仍前進一段再入 VISUAL_CLIMB
        # 對準後、進 VISUAL_CLIMB 前,先沿橋軸直行一小段「貼上橋口」(修正平移/docking),再交給視覺。
        self.SNAP_FWD_SEC = 1.2               # 對準後直行前進的時間 (s);0=不前進
        self.SNAP_FWD_SPEED = 300.0           # 此段前進輪速 (比上橋慢,溫和貼進坡口) (app 更新 ×2.5)

        # ---- VISUAL_CLIMB：降爪 + 沿「橋面 segmentation 中線」全速過橋,走近橋上的熊並鏟入 ----
        # 轉向用穩定的 bridge_info delta_x (b_dx) 維持在橋中線、全速直行過橋 (不用 bear 深度轉向——太抖)。
        # bear 只用來判夾取時機:走近到 GRIP_DIST,或「曾靠近 (<=COMMIT_DIST) 後持續看不到=已鏟入爪中」→ GRIP;
        # 都沒觸發就一路過橋到 VCLIMB_TIMEOUT (視為已過橋/到頂) 再夾。不靠 /amcl_pose (橋上 pose 會凍)。
        self.VCLIMB_SPEED = 300.0         # 上橋前進輪速 (full thrust;太慢會卡在坡面) (app 更新 ×2.5)
        self.VCLIMB_STEER_GAIN = 0.35     # 置中差速增益:steer = GAIN * (扣 deadband 後的 bridge_dx)。
                                          #   調小 (0.6→0.35):增益太大會一路把車帶去撞側牆 (低增益→修正溫和,不硬轉)。
        self.VCLIMB_STEER_CLAMP = 0.3     # steer 夾在 ±此比例*base (調小,限制最大彎度,避免硬轉撞牆)
        self.BRIDGE_DX_DEADBAND = 70.0    # b_dx 在 ±此值內不轉向 (DESCEND 用;吸收下坡時橋面質心的穩態偏移)。
        self.VCLIMB_STEER_DEADBAND = 35.0 # VISUAL_CLIMB 專用、較緊的 deadband:上橋要更貼著橋中線校正朝向。
                                          #   實測上橋常停在 bridge_dx≈+55 (熊 dx≈+131=車偏左,是真偏移不是 camera 偏置),
                                          #   70 太寬會把這真偏移當雜訊不修 → 車爬偏。設 35 讓它把朝向校回橋中線;
                                          #   增益仍低 (0.35) 故修正溫和、不會像舊版 (gain 0.6) 一路撞牆。爬偏才調大。
        self.VCLIMB_OBSERVE_DIST = 0.65   # 走近橋上的熊到此距離 (m) 連續 N 幀 → 停下「面向 + 觀察」(OBSERVE)。
                                          #   調小 (0.9→0.65) = 上橋爬更久/更靠近熊才停 (爬到更上面);熊此時仍可見可面向。
        self.VCLIMB_GRIP_DIST = 0.55      # 熊深度 <= 此距離 (m) 連續 N 幀 → 已到熊前 → GRIP (保留;OBSERVE 觸發實際用 OBSERVE_DIST)
        self.VCLIMB_MAX_TRACK_DIST = 4.0  # 只追深度 <= 此距離 (m) 的熊 (判夾取時機用)。深度 -1 (過近觸底) 仍算。
        self.VCLIMB_GRIP_CONFIRM = 3      # 連續 N 幀夠近才 GRIP (濾抖動)
        self.VCLIMB_COMMIT_DIST = 0.70    # 曾靠近到此距離 (m) 後持續看不到 = 已鏟入爪中 → GRIP。
                                          #   設嚴一點 (0.7),避免在 ~1m「熊掉出鏡頭」就誤判到位 (爪只構得到 ~0.2m)。
        self.VCLIMB_REACH_LOST = 10       # 靠近後連續看不到熊這麼多幀 (~1.0s) → GRIP
        self.VCLIMB_TIMEOUT = 6.0        # (簡化版未用) 舊複雜邏輯的過橋逾時。
        self.VCLIMB_CLIMB_SEC = 3.0      # ★簡化版★:全速沿橋中線爬這麼多秒 → 直接 OBSERVE (測試用,移除追熊 handoff)。
        # (移除 bear 深度卡死偵測:橋上 bear 深度常凍在 ~1m 不隨車前進而變,會誤判卡死、亂倒退浪費過橋時間。
        #  改靠 full thrust + bridge 中線轉向 open-loop 過橋;真的物理卡死就靠 VCLIMB_TIMEOUT 兜底。)

        # ---- OBSERVE：停下面向橋上的熊 + 持住觀察 (Locate & Observe 計分),再進 GRIP ----
        # VISUAL_CLIMB 走近熊 (<= VCLIMB_OBSERVE_DIST) 或曾靠近後看不到 → 進 OBSERVE。先原地轉把熊置中 (面向它),
        # 再停住持住 OBSERVE_SECONDS 秒 (>5s 給分餘裕),然後 GRIP。熊看不到 (被爪遮/掉鏡頭) 時不轉,直接持住。
        self.OBSERVE_SECONDS = 5.5        # 觀察持住秒數 (>5s 給分餘裕)
        self.OBSERVE_ALIGN_PX = 60.0      # 面向熊的置中容差 (|bear dx| <= 此值算面向)
        self.OBSERVE_NEAR_DIST = 1.5      # ★只用「近熊」(dist <= 此值,m) 對中★;遠處誘餌熊 (~2m) 忽略,
                                          #   避免 bbox 抖到遠熊 (dx 小) 就誤判「已面向」而不轉。
        self.OBSERVE_FACE_TIMEOUT = 6.0   # 對中熊逾時保險 (s):轉不到位也進持住,避免在坡頂一直空轉

        # ---- GRIP：到頂/過橋後,前頂一段把熊鏟進低位開爪中 + 關爪夾起 (爪已在 VISUAL_CLIMB 降下且全程開著) ----
        self.GRIP_PRESS_SPEED = 300.0     # 關爪前的前頂輪速 (full thrust;坡頂要更大力頂得動、把熊鏟進爪);0=純煞停 (app 更新 ×2.5)
        self.GRIP_PRESS_SEC = 3.0         # 關爪前先前頂這麼久 (s):熊掉出鏡頭時常在爪前 ~0.8m,需多頂一段才鏟進爪

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
        self.DESCEND_MIN_SEC = 4.0        # commit 窗 (s):此前全速衝過 fat part + 階梯,視覺尚不可結束 (防坡頂誤判)
        self.DESCEND_MAX_SEC = 9.0        # 結束逾時 (s):實測 ~8s 已下到地面;下坡時 road/bridge mask 偵測不穩、
                                          #   road-fill 判底常失效,故縮短到 9s 當主要結束依據 → RETURN。
        self.DESCEND_ROAD_AREA = 0.55     # 路面 area_frac >= 此值 → 路面占滿畫面=已下到地面 → 結束。
                                          #   坡頂看遠處路面只 ~0.33,故設高 (~0.55);用 log 的 road_area 頂/底值微調。
        self.DESCEND_DONE_CONFIRM = 5     # 連續 N 幀滿足「到底」條件才結束 (濾 segmentation 抖動)
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
        # BRIDGE_APPROACH (pose-based, waypoints) 狀態
        dock_target = [self.DOCK_X, self.DOCK_Y]
        wp_idx = 0                           # 當前 intermediate waypoint index (進 list 之後變 ≥ len → DOCK)
        dock_arrived_streak = 0              # 連續到位幀數 (final dock)
        # near-goal stuck guard
        stuck_anchor_xy = None               # 進入近區後的「卡死」基準位置
        stuck_anchor_tick = -10000           # anchor 設置的 tick (用以計時)
        approach_phase = "DRIVE_WP" if self.WAYPOINTS else "DRIVE_DOCK"
        # SNAP_90 狀態
        snap_phase = "ROTATE"                # ROTATE (轉到橋軸) → FORWARD (直行貼坡口)
        snap_aligned_streak = 0
        snap_deadline = 0.0
        snap_fwd_deadline = 0.0
        # VISUAL_CLIMB 狀態 (進 VISUAL_CLIMB 第一幀初始化)
        vclimb_scoop_prepared = False        # 鏟爪只在進 VISUAL_CLIMB 時降一次
        vclimb_entry_time = 0.0
        vclimb_lost_streak = 0               # 連續看不到熊的幀數
        vclimb_grip_streak = 0               # 連續「夠近」的幀數
        vclimb_last_valid_dist = None        # 最近一次熊的有效深度 (判斷是否曾靠近)
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
            b_area = binfo[2] if binfo and len(binfo) > 2 else 0.0  # 橋面占比 (DESCEND 判橋已在身後)
            # bear: [found, distance, delta_x, area, bottom]。VISUAL_CLIMB 吃它 (置中 + 到位判定)。
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

                # ---- Phase 1a: DRIVE_WP → 經過中繼 waypoints (依序,不對齊 yaw) ----
                if approach_phase == "DRIVE_WP":
                    wp_x, wp_y, wp_arrive_dist = self.WAYPOINTS[wp_idx]
                    wp_target = [wp_x, wp_y]
                    dist_wp = cal_distance(car_xy, wp_target)
                    last_wp = (wp_idx == len(self.WAYPOINTS) - 1)
                    if dist_wp < wp_arrive_dist:
                        wp_idx += 1
                        stuck_anchor_xy = None      # 換點 → 重置 stuck 基準
                        if wp_idx >= len(self.WAYPOINTS):
                            # 上橋 waypoints 已把車帶到橋上中線、對到橋軸 → 直接視覺伺服上橋 (跳過 DRIVE_DOCK/SNAP_90)
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

                # ---- Phase 1b: DRIVE_DOCK → 到最終 dock 位置 (精準對位) ----
                elif approach_phase == "DRIVE_DOCK":
                    dist = cal_distance(car_xy, dock_target)
                    if dist < self.DOCK_ARRIVE_DIST:
                        dock_arrived_streak += 1
                        self._publish("STOP")
                        if dock_arrived_streak >= self.DOCK_ARRIVE_CONFIRM:
                            snap_phase = "ROTATE"
                            snap_aligned_streak = 0
                            snap_deadline = time.time() + self.SNAP_90_TIMEOUT
                            self._transition(
                                self.SNAP_90,
                                f"已到 dock 位置 (dist={dist:.2f}m) → 轉到橋軸 (~90°)")
                            continue
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
                                          f"{self.STUCK_TICKS}f 沒動) → 視為到位 → 轉到橋軸")
                                    self._publish("STOP")
                                    snap_phase = "ROTATE"
                                    snap_aligned_streak = 0
                                    snap_deadline = time.time() + self.SNAP_90_TIMEOUT
                                    self._transition(self.SNAP_90, "DOCK 卡死 → 轉到橋軸 (~90°)")
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

                if time.time() > approach_deadline:
                    self._transition(self.DONE, "BRIDGE_APPROACH 逾時")

            # ---------------- SNAP_90 (把橋上的熊轉到正中 + yaw~90 → 直行貼坡口) ----------------
            elif self.state == self.SNAP_90:
                # ---- Phase B: FORWARD → 沿橋軸直行一小段,修正平移、貼上橋口,再交給 VISUAL_CLIMB ----
                if snap_phase == "FORWARD":
                    if time.time() < snap_fwd_deadline:
                        self._arc(self.SNAP_FWD_SPEED, 0.0)
                    else:
                        self._publish("STOP")
                        vclimb_scoop_prepared = False
                        self._transition(self.VISUAL_CLIMB, "貼坡口完成 → 視覺伺服上橋")
                    continue

                # ---- Phase A: ALIGN → 原地轉把「橋上的熊」轉到正中 (yaw 自然 ~90,再 sanity 確認) ----
                pose_msg = self.ros_communicator.get_latest_amcl_pose()
                if pose_msg is None or time.time() > snap_deadline:
                    self._publish("STOP")
                    reason = "SNAP_90 無 /amcl_pose" if pose_msg is None else "SNAP_90 對準逾時"
                    snap_phase = "FORWARD"
                    snap_fwd_deadline = time.time() + self.SNAP_FWD_SEC
                    self._transition(self.SNAP_90, reason + " → 直行貼坡口")
                    continue
                o = pose_msg.pose.pose.orientation
                cur_yaw = self._norm_angle(2.0 * math.atan2(o.z, o.w))
                yaw_err = self._norm_angle(self.DOCK_YAW_RAD - cur_yaw)
                yaw_sane = abs(yaw_err) <= self.SNAP_YAW_SANITY
                # 用「橋面 segmentation 質心」對準 (比 bear 深度穩太多):b_dx → 0 = 橋面置中。橋面置中時
                # 橋上的熊 (在橋中線上) 也跟著置中,且車自然朝向橋軸 ~90° (再用 yaw sanity 確認)。

                if b_found and abs(b_dx) > self.SNAP_CENTER_PX:
                    # 橋面還沒置中 → 原地轉把它轉到中間 (b_dx>0 橋在右 → 順時針)
                    snap_aligned_streak = 0
                    self._publish("CLOCKWISE_ROTATION_SLOW" if b_dx > 0
                                  else "COUNTERCLOCKWISE_ROTATION_SLOW")
                    action = f"CENTER(bridge_dx={b_dx:+.0f} yaw={math.degrees(cur_yaw):.0f}°)"
                elif b_found and yaw_sane:
                    # 橋面置中 + yaw 合理 → 兩條件成立
                    snap_aligned_streak += 1
                    self._publish("STOP")
                    action = f"OK(bridge_dx={b_dx:+.0f} yaw={math.degrees(cur_yaw):.0f}°) {snap_aligned_streak}/{self.SNAP_YAW_CONFIRM}"
                    if snap_aligned_streak >= self.SNAP_YAW_CONFIRM:
                        snap_phase = "FORWARD"
                        snap_fwd_deadline = time.time() + self.SNAP_FWD_SEC
                        print(f"[Task2] snap 對準完成 (橋面置中 bridge_dx={b_dx:+.0f}, "
                              f"yaw={math.degrees(cur_yaw):.1f}°) → 直行貼坡口 {self.SNAP_FWD_SEC:.1f}s")
                        continue
                elif b_found:
                    # 橋面置中了但 yaw 還不在 ~90 → 朝 90 轉 (順便維持橋面大致置中)
                    snap_aligned_streak = 0
                    self._publish("COUNTERCLOCKWISE_ROTATION_SLOW" if yaw_err > 0
                                  else "CLOCKWISE_ROTATION_SLOW")
                    action = f"YAW(err={math.degrees(yaw_err):+.0f}° bridge_dx={b_dx:+.0f})"
                else:
                    # 這幀沒看到橋面:yaw 還沒到 ~90 就先轉去把橋帶進畫面,否則原地等偵測
                    snap_aligned_streak = 0
                    if not yaw_sane:
                        self._publish("COUNTERCLOCKWISE_ROTATION_SLOW" if yaw_err > 0
                                      else "CLOCKWISE_ROTATION_SLOW")
                        action = f"SEEK_YAW(err={math.degrees(yaw_err):+.0f}°)"
                    else:
                        self._publish("STOP")
                        action = "WAIT_BRIDGE"
                dock_dbg += 1
                if dock_dbg % 10 == 1:
                    print(f"[Task2] SNAP_90 {action}  (target yaw={math.degrees(self.DOCK_YAW_RAD):.0f}°, "
                          f"bridge F={int(b_found)} dx={b_dx:+.0f})")

            # ---------------- VISUAL_CLIMB (簡化版:降爪 → 沿橋中線爬 N 秒 → OBSERVE) ----------------
            elif self.state == self.VISUAL_CLIMB:
                # 1) 進場先把鏟爪降到貼地+開爪 (阻塞一次)。爪降下=熊會被一路鏟進爪中。
                if not vclimb_scoop_prepared:
                    self._publish("STOP")
                    self.arm_controller.scoop_pose()   # 阻塞：降臂 + 開爪
                    vclimb_scoop_prepared = True
                    vclimb_entry_time = time.time()
                # 2) 簡化邏輯 (測試用):全速沿橋中線爬 VCLIMB_CLIMB_SEC 秒 → 直接 OBSERVE。
                #    不追熊深度、不判夾取時機 (移除舊的 bear handoff / timeout 複雜邏輯)。
                elapsed = time.time() - vclimb_entry_time
                if elapsed > self.VCLIMB_CLIMB_SEC:
                    self._publish("STOP")
                    observe_entry = time.time(); observe_centered = False
                    self._transition(self.OBSERVE,
                                     f"爬橋 {self.VCLIMB_CLIMB_SEC:.0f}s 到 → 停下觀察")
                    continue
                # 3) 轉向:橋面中線 (b_dx) 置中,全速直行;橋面沒偵測就直走。
                if b_found:
                    steer = self._bridge_center_steer(self.VCLIMB_SPEED, b_dx,
                                                      self.VCLIMB_STEER_GAIN,
                                                      self.VCLIMB_STEER_CLAMP,
                                                      self.VCLIMB_STEER_DEADBAND)
                    self._arc(self.VCLIMB_SPEED, steer)
                    steer_str = f"bridge_dx={b_dx:+.0f} steer={steer:+.0f}"
                else:
                    self._arc(self.VCLIMB_SPEED, 0.0)
                    steer_str = "no-bridge straight"

                vclimb_dbg += 1
                if vclimb_dbg % 10 == 1:
                    print(f"[Task2] VISUAL_CLIMB t={elapsed:.1f}/{self.VCLIMB_CLIMB_SEC:.0f}s → {steer_str}")

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
                    nonlocal descend_entry_time, descend_dbg, descend_done_streak
                    self._publish("STOP")
                    descend_entry_time = time.time()
                    descend_dbg = 0
                    descend_done_streak = 0
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
                    # 還沒置中 → 原地轉把它轉到正前方 (c_dx>0 在右 → 順時針),同 SNAP_90 慣例
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
                    self._publish("STOP")
                    if self.start_pose is not None:
                        print(f"[Task2] RETURN 目標(起點) = {self.start_pose}")
                    self._return_entry_time = time.time()
                    self._arrive_streak = 0
                    self._return_dbg = 0
                    self._transition(self.RETURN, reason)

                # 兜底逾時:視覺沒判到底也最多前進這麼久 → RETURN
                if elapsed > self.DESCEND_MAX_SEC:
                    _to_return("下坡逾時 (MAX_SEC) → 位姿式返航")
                    continue
                # 到底判定 (過 commit 窗後才允許):路面占滿畫面 (area_frac 高) = 已下到地面,連續 N 幀 → RETURN。
                #   坡頂就看得到遠處路面 (~0.33),故門檻拉高 (~0.55),才不會在 fat part 上就誤判到底卡死。
                if elapsed > self.DESCEND_MIN_SEC:
                    if r_found and r_area >= self.DESCEND_ROAD_AREA:
                        descend_done_streak += 1
                    else:
                        descend_done_streak = 0
                    if descend_done_streak >= self.DESCEND_DONE_CONFIRM:
                        _to_return("路面占滿畫面 (已下到地面) → 位姿式返航")
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

    def _bridge_center_steer(self, base, b_dx, gain, clamp, deadband=None):
        """依「橋面 segmentation 質心」b_dx 算置中差速,維持在橋中線。
        先扣 deadband (預設 BRIDGE_DX_DEADBAND;VISUAL_CLIMB 傳較緊的 VCLIMB_STEER_DEADBAND 校準朝向),
        扣完只對剩餘偏差做 P 控制,再 clamp 限制最大彎度。"""
        db = self.BRIDGE_DX_DEADBAND if deadband is None else deadband
        err = (b_dx - math.copysign(db, b_dx)) if abs(b_dx) > db else 0.0
        steer = gain * err
        cap = clamp * base
        return max(-cap, min(cap, steer))

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
