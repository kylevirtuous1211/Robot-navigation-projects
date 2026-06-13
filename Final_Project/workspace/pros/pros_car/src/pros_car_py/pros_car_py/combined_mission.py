"""Combined Task 2 (bridge) → Task 3 (door knob) end-to-end mission.
=================================================================

一次跑完 Task 2 + Task 3，拿兩個任務的全部分數。

用法 (容器內)：
    ros2 run pros_car_py task23_auto
或直接：
    ~/Desktop/Robot-navigation-projects/Final_Project/workspace/pros/pros_car/run_task23.sh

前置 (和單跑 Task 2 / Task 3 相同)：
  1. start_stack.sh 全感知上線 (bear / knob / bridge-seg 三個 YOLO + slam/nav + tfshim)。
  2. Unity 進 FINAL PROJECT、CAR+ARM = AI、RosBridge 9091 Connected。
  3. reset_map.sh --pin (車在 spawn → /amcl_pose ≈ (0,0,0)，把釘住的座標系定好)。
     兩個任務的絕對座標 (Task2 的 DOCK/WAYPOINTS、Task3 的 WAYPOINTS) 都在這個釘住的 frame。

流程：
  ▶ TASK2 (bridge)  完整跑 Task2Mission：上橋 → 爬橋 → 夾熊 → 下橋 → 返航放熊 (Ascent/Descent/Recovery)。
                    Task2 RETURN 是位姿式回到 spawn，結束時 /amcl_pose 已解凍且有效。
  (settle 2s)
  ▶ TASK3 (door)    完整跑 Task3Mission：knob_stow 收手臂 → DRIVE_WP 開到門把 (絕對 waypoint,
                    會先原地轉掉頭，因為 Task2 結束時車頭朝後) → 視覺對準 → 觀察 ≥5s (Locate & Observe)
                    → 壓桿開門 → 直行穿門。

設計：直接「重用」兩個已測過的 state machine class，不重寫。只共用一份 ROS 元件 (RosCommunicator +
spin thread + data_processor/nav_processing/car_controller/arm_controller)，依序跑 Task2 → Task3。
就算 Task2 沒拿滿分 (timeout 也會走到 DONE)，仍然續跑 Task3 → 盡量拿到所有分。
"""

import threading
import time

import rclpy

from pros_car_py.ros_communicator import RosCommunicator
from pros_car_py.data_processor import DataProcessor
from pros_car_py.nav_processing import Nav2Processing
from pros_car_py.car_controller import CarController
from pros_car_py.arm_controller_2D import ArmController
from pros_car_py.task2_mission import Task2Mission
from pros_car_py.task3_mission import Task3Mission


def _run_to_done(mission, label, watchdog_sec=None):
    """啟動一個任務 state machine 並阻塞等到它跑完 (DONE → _running=False)。

    watchdog_sec: 真‧卡死兜底 (秒)。各任務自己已有狀態逾時 (e.g. Task2 RETURN 150s),此處
    只在「整個任務怎樣都不結束」時強制停止、往下走 (避免連跑時無限掛住)。設大一點,不要切到正常慢跑。
    """
    print(f"[combined] ▶ {label} 開始")
    start = time.time()
    mission.start()
    try:
        while mission._running and mission._thread.is_alive():
            mission._thread.join(timeout=0.5)
            if watchdog_sec is not None and time.time() - start > watchdog_sec:
                print(f"[combined] ⏱ {label} 超過 {watchdog_sec:.0f}s watchdog → 強制停止,往下走")
                break
    finally:
        mission.stop()
    print(f"[combined] ✔ {label} 結束 ({time.time() - start:.0f}s)")


def main(args=None):
    """Headless 進入點：依序跑 Task 2 → Task 3，共用同一份 ROS 元件。"""
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

    deps = (
        ros_communicator,
        data_processor,
        nav_processing,
        car_controller,
        arm_controller,
    )

    # watchdog 只是「真卡死」兜底 (各任務內部已有狀態逾時);設大,不切正常慢跑。
    TASK2_WATCHDOG_SEC = 360.0
    TASK3_WATCHDOG_SEC = 240.0

    print("[task23_auto] 啟動 Task 2 → Task 3 連跑 (headless)。Ctrl-C 可中止。")
    try:
        _run_to_done(Task2Mission(*deps), "TASK2 (bridge)", TASK2_WATCHDOG_SEC)
        # 兩任務間稍微 settle:Task2 放完熊、車停穩、/amcl_pose 解凍穩定後再進 Task3。
        time.sleep(2.0)
        _run_to_done(Task3Mission(*deps), "TASK3 (door knob)", TASK3_WATCHDOG_SEC)
        print("[combined] 🎉 Task 2 + Task 3 全部完成。")
    except KeyboardInterrupt:
        print("[combined] 收到中止訊號。")
    finally:
        ros_communicator.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
