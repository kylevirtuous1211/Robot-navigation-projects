# Control arm depending on self.move_real_and_virtual
from action_interface.action import ArmGoal
import time
import math
from typing import Tuple, List
from arm_control_pkg.utils import get_yaw_from_quaternion, normalize_angle
import random
class ArmAutoController:
    def __init__(
        self, arm_params, arm_commute_node, pybulletRobotController, arm_agnle_control
    ):
        self.arm_params = arm_params.get_arm_params()
        self.pybullet_robot_controller = pybulletRobotController
        self.arm_commute_node = arm_commute_node
        self.arm_agnle_control = arm_agnle_control
        self.depth = 100.0

    def catch2(self, should_cancel=lambda: False):
        self.arm_agnle_control.arm_index_change(0, 100.0)
        self.arm_commute_node.publish_arm_angle()
        time.sleep(0.5)
        self.arm_agnle_control.arm_index_change(0, 105.0)
        self.arm_commute_node.publish_arm_angle()
        time.sleep(0.5)

        angles = [105, 45, 145, 180, 70]
        
        for i in range(8):
            self.arm_agnle_control.arm_all_change(angles)
            self.arm_commute_node.publish_arm_angle()
            time.sleep(0.5)

            angles[1] += 5
            angles[2] -= 5
        time.sleep(1.0)
        self.grap()
        time.sleep(0.5)
        self.init_pose(grap=True)
        time.sleep(1.0)

        self.arm_agnle_control.arm_index_change(0, 0.0)
        self.arm_commute_node.publish_arm_angle()
        time.sleep(0.5)

        self.arm_agnle_control.arm_index_change(0, 5.0)
        self.arm_commute_node.publish_arm_angle()
        time.sleep(1.0)

        angles = [5, 80, 100, 180, 10]
        self.arm_agnle_control.arm_all_change(angles)
        self.arm_commute_node.publish_arm_angle()
        time.sleep(0.5)

        self.arm_agnle_control.arm_index_change(4, 70.0)
        self.arm_commute_node.publish_arm_angle()
        time.sleep(0.5)

        self.init_pose(grap=False)

        # self.arm_agnle_control.arm_all_change([105, 45, 145, 180, 70])
        # self.arm_commute_node.publish_arm_angle()
        # time.sleep(0.5)
        # self.arm_agnle_control.arm_all_change([105, 50, 140, 180, 70])
        # self.arm_commute_node.publish_arm_angle()
        # time.sleep(0.5)
        # self.arm_agnle_control.arm_all_change([105, 55, 135, 180, 70])
        # self.arm_commute_node.publish_arm_angle()
        # time.sleep(0.5)
        # self.arm_agnle_control.arm_all_change([105, 65, 125, 180, 70])
        # self.arm_commute_node.publish_arm_angle()
        # time.sleep(0.5)
        # self.arm_agnle_control.arm_all_change([105, 85, 100, 180, 70])
        # self.arm_commute_node.publish_arm_angle()
        # time.sleep(0.5)
        # time.sleep(0.5)
        return ArmGoal.Result(success=True, message="success")
        # self.arm_agnle_control.arm_all_change([])

    def catch(self, should_cancel=lambda: False):
        label = "tennis"
        while self.depth > 0.4:
            print(self.depth)
            try:
                self.depth = self.arm_commute_node.get_latest_object_coordinates(label=label)[0]
            except:
                continue
        while 1:
            if should_cancel():
                return ArmGoal.Result(success=False, message="Canceled by user")
            if self.follow_obj(label=label)  == True:
                break
            # if self.follow_obj(label="ball") == True:
            #     break

        # reset depth
        self.depth = 100.0
        # obj_pos = self.pybullet_robot_controller.markPointInFrontOfEndEffector(
        #     distance=0.4,z_offset = 0.05
        # )
        data = self.arm_commute_node.get_latest_object_coordinates(label=label)
        depth = data[0]
        obj_pos = self.pybullet_robot_controller.markPointInFrontOfEndEffector(
            distance=depth + 0.05,z_offset=0.15
        )
        robot_angle = self.pybullet_robot_controller.generateInterpolatedTrajectory(
            target_position=obj_pos,steps=10
        )
        for i in robot_angle:
            self.move_real_and_virtual(radian=i)
            time.sleep(0.1)
        self.grap()
        time.sleep(1.0)
        self.init_pose(grap=True)
        time.sleep(1.0)
        self.seek_arucode()
        time.sleep(0.5)
        self.init_pose()
        # self.rotate_car()
        # self.rotate_wrist()
        # time.sleep(0.2)
        # for i in range(10):
        #     self.arm_commute_node.publish_pos()
        #     time.sleep(0.1)

        return ArmGoal.Result(success=True, message="success")

    def seek_arucode(self, joint_idx: int = 0, sleep_s: float = 0.2):
        """
        掃描指定關節 (joint_idx) 從 0~180 度，邊掃邊讀取 ArUco 深度；
        一旦讀到深度，就把末端移到前方該距離的位置。
        """
        self.arm_commute_node.clear_arucode_topic()
        # 防呆：確保 joint_idx 在範圍內
        joint_positions, _, _ = self.pybullet_robot_controller.getJointStates()  # 弧度 list，長度 = 可控關節數
        dof = len(joint_positions)
        if dof == 0 or joint_idx < 0 or joint_idx >= dof:
            self.get_logger().error(f"[seek_arucode] 無效的 joint_idx={joint_idx} 或無可控關節（dof={dof}）")
            return

        for deg in range(0, 181, 10):  # 0..180（含 180）
            # 1) UI/角度控制端用「度」
            self.arm_agnle_control.arm_index_change(joint_idx, deg)

            # 2) 取目前整組關節（弧度），只改第 joint_idx
            q = list(self.pybullet_robot_controller.getJointStates()[0])  # 再取一次最新值
            q[joint_idx] = math.radians(deg)

            # 3) 真實 + 模擬 同步（注意：這裡要丟「整組」角度）
            self.move_real_and_virtual(radian=q)

            time.sleep(0.5)

            # 4) 檢查是否拿到深度
            arucode_depth = self.arm_commute_node.get_latest_arucode_depth()  # 單位：公尺（前面 publish 的就是 m）
            print("arucode:", arucode_depth)
            if arucode_depth is not None and arucode_depth > 0.0:
                time.sleep(1)
                arucode_depth = self.arm_commute_node.get_latest_arucode_depth()
                # 5) 依目前末端姿態，沿本地 X 前方 arucode_depth 的目標點（你函式已用四元數算本地軸了）
                obj_pos = self.pybullet_robot_controller.markPointInFrontOfEndEffector(
                    distance=arucode_depth - 0.15, z_offset=0.05, visualize=True
                )

                # 6) 插值產生軌跡並執行
                traj = self.pybullet_robot_controller.generateInterpolatedTrajectory(
                    target_position=obj_pos, steps=5
                )
                for qstep in traj:   # qstep 應該就是「整組關節弧度」
                    self.move_real_and_virtual(radian=qstep)
                    time.sleep(0.3)
                self.arm_agnle_control.arm_index_change(4, 70)
                self.arm_commute_node.publish_arm_angle()
                self.arm_commute_node.clear_arucode_signal()  # 清除信號，避免重複讀取
                break


    def rotate_wrist(self):
        self.arm_agnle_control.arm_index_change(3, 90)
        self.arm_commute_node.publish_arm_angle()

    def rotate_car(self):
        # 取得當前車體朝向
        _, rotation = self.arm_commute_node.get_car_position_and_orientation()
        current_yaw = get_yaw_from_quaternion(rotation)

        # 設定目標朝向（反向 180 度）
        target_yaw = normalize_angle(current_yaw + math.pi)

        # 開始旋轉
        self.arm_commute_node.publish_control(vel=[5.0, -5.0, 5.0, -5.0])

        while True:
            _, rotation = self.arm_commute_node.get_car_position_and_orientation()
            yaw = get_yaw_from_quaternion(rotation)
            yaw_error = normalize_angle(target_yaw - yaw)

            print(f"Current Yaw: {math.degrees(yaw):.2f}, Target: {math.degrees(target_yaw):.2f}, Error: {math.degrees(yaw_error):.2f}")

            if abs(yaw_error) < math.radians(5):  # 誤差小於 5 度即停止
                break
            time.sleep(0.1)

        for i in range(5):
            # 停止轉動
            self.arm_commute_node.publish_control(vel=[0.0, 0.0, 0.0, 0.0])
            time.sleep(0.1)

    def car2_position(self):
        # 給 car2 的座標
        pass

    def arm_wave(self, should_cancel=lambda: False):
        while 1:
            if should_cancel():
                return ArmGoal.Result(success=False, message="Canceled by user")
            axis0 = round(random.uniform(30.0, 150.0), 1)
            axis1 = round(random.uniform(0.0, 70.0), 1)
            axis2 = round(random.uniform(0.0, 130.0), 1)
            axis3 = round(random.uniform(90.0, 180.0), 1)
            axis4 = round(random.uniform(0.0, 70.0), 1)
            angles_deg = [axis0, axis1, axis2, axis3, axis4]
            angles_rad = [math.radians(a) for a in angles_deg]
            self.move_real_and_virtual(radian=angles_rad)
            time.sleep(1.0)

    def object_follow(self, should_cancel=lambda: False):
        while 1:
            if should_cancel():
                return ArmGoal.Result(success=False, message="Canceled by user")
            self.follow_obj(label="tennis", step=5)

    def radians_to_degrees(self, radians_list):
        """Converts a list of angles from radians to degrees."""
        if not isinstance(radians_list, (list, tuple)):
            # Handle potential errors if input is not a list/tuple
            print("Error: Input must be a list or tuple of radians.")
            return []  # Or raise an error
        try:
            degrees_list = [math.degrees(rad) for rad in radians_list]
            return degrees_list
        except TypeError as e:
            print(
                f"Error converting radians to degrees: {e}. Ensure all elements are numbers."
            )
            return []  # Or raise an error

    def grap(self):
        self.arm_agnle_control.arm_index_change(4, 10.0)
        self.arm_commute_node.publish_arm_angle()

    def init_pose(self, grap=False):
        angle = self.arm_agnle_control.arm_default_change()
        if grap:
            self.arm_agnle_control.arm_index_change(4, 10.0)
            self.arm_commute_node.publish_arm_angle()
            time.sleep(1.0)
        self.arm_commute_node.publish_arm_angle()
        joints_reset_degrees = angle
        joints_reset_radians = [math.radians(angle) for angle in joints_reset_degrees]
        self.pybullet_robot_controller.setJointPosition(position=joints_reset_radians)
        return ArmGoal.Result(success=True, message="success")

    def test(self):
        # self.rotate_car()
        self.arm_commute_node.publish_pos()
        return ArmGoal.Result(success=True, message="success")

    def look_up(self):
        self.arm_agnle_control.arm_index_change(2, 140)
        self.arm_commute_node.publish_arm_angle()

    def _is_at_target(
        self,
        depth: float,
        y: float,
        z: float,
        target_depth: float,
        depth_thresh: float,
        lateral_thresh: float,
    ) -> bool:
        """
        判斷當前 (depth, y, z) 是否已經進入允收範圍。
        """
        return (
            # abs(depth - target_depth) <= depth_thresh
            abs(y) <= 0.02
            and abs(z) <= 0.1
        )

    def follow_obj(self, label="ball", target_depth=0.3, step=10):
        # 參數設定
        depth_threshold = 0.05
        lateral_threshold = 0.05
        x_adjust_factor = 0.3
        y_adjust_factor = 0.3
        z_adjust_factor = 0.3

        # 1. 讀座標
        data = self.arm_commute_node.get_latest_object_coordinates(label=label)
        if not data or len(data) < 3:
            return ArmGoal.Result(success=False, message="No object detected")
        current_depth, obj_y, obj_z = data

        # 2. 初次檢查
        if self._is_at_target(
            current_depth,
            obj_y,
            obj_z,
            target_depth,
            depth_threshold,
            lateral_threshold,
        ):
            print("檢查到已經在目標位置")
            return True

        # 3. 計算偏移
        depth_diff = current_depth - target_depth
        x_offset = depth_diff * x_adjust_factor
        y_offset = obj_y * y_adjust_factor
        z_offset = obj_z * z_adjust_factor

        # 4. 設定絕對深度（可選）
        target_pos = self.pybullet_robot_controller.offset_from_end_effector(
            x_offset=x_offset,
            y_offset=y_offset,
            z_offset=z_offset,
            visualize=True,
            mark_color=[0, 1, 0],
        )
        target_pos[0] = 0.2  # 若要固定深度

        # 5. 生成與執行軌跡
        traj = self.pybullet_robot_controller.generateInterpolatedTrajectory(
            target_position=target_pos, steps=step
        )
        if not traj:
            return True

        for angle in traj:
            self.move_real_and_virtual(radian=angle)
            time.sleep(0.05)

            # 6. 每步驟都再檢查一次
            new_data = self.arm_commute_node.get_latest_object_coordinates(label=label)
            if new_data and len(new_data) >= 3:
                nd, ny, nz = new_data
                if self._is_at_target(
                    nd, ny, nz, target_depth, depth_threshold, lateral_threshold
                ):
                    print("中途已達到目標位置，提早停止")
                    return True
                    break

        # return True

    def ik_move_func(self):
        # use ik move to obj position, but not excute
        # This must use imu data
        imu_data = self.arm_commute_node.get_latest_imu_data()
        obj_position_data = self.arm_commute_node.get_latest_object_coordinates(
            label="fire"
        )
        extrinsics = self.pybullet_robot_controller.calculate_imu_extrinsics(
            imu_world_quaternion=imu_data, link_name="camera_1", visualize=False
        )
        obj_pos_in_pybullet = self.pybullet_robot_controller.transform_object_to_world(
            T_world_to_imu=extrinsics,
            object_coords_imu=obj_position_data,
            visualize=True,
        )
        print(obj_pos_in_pybullet)
        is_close_pos = self.pybullet_robot_controller.is_link_close_to_position(
            link_name="base_link", target_position=obj_pos_in_pybullet, threshold=0.8
        )
        if is_close_pos:
            robot_angle = self.pybullet_robot_controller.generateInterpolatedTrajectory(
                target_position=obj_pos_in_pybullet, steps=10
            )
            for i in robot_angle:
                self.move_real_and_virtual(radian=i)
                time.sleep(0.2)
        else:
            print("not close to the object")

    def move_real_and_virtual(self, radian):
        # for synchronous move real and virtual robot
        self.pybullet_robot_controller.setJointPosition(position=radian)
        degree = self.radians_to_degrees(radian)
        # degree[-1] = 90
        self.arm_agnle_control.arm_all_change(degree)
        self.arm_commute_node.publish_arm_angle()

    def move_forward_backward(self, direction="forward", distance=0.1):
        """
        控制手臂向前或向後移動。

        Args:
            direction (str): 移動方向，"forward" 或 "backward"
            distance (float): 移動距離（以米為單位），對於後退方向會自動轉換為負值

        Returns:
            ArmGoal.Result: 包含操作結果的對象
        """
        # 根據方向確定距離值（前進為正，後退為負）
        actual_distance = distance if direction == "forward" else -abs(distance)
        if direction == "forward":
            z_offset = 0.05
        else:
            z_offset = -0.05
        # 標記目標點位置
        obj_pos = self.pybullet_robot_controller.markPointInFrontOfEndEffector(
            distance=actual_distance, z_offset=z_offset
        )

        # 生成插值軌跡
        robot_angle = self.pybullet_robot_controller.generateInterpolatedTrajectory(
            target_position=obj_pos, steps=5
        )

        # 執行運動
        for i in robot_angle:
            # 如果需要同步真實機械臂，可以用 move_real_and_virtual
            # 否則只移動模擬中的機械臂
            self.move_real_and_virtual(radian=i)
            time.sleep(0.1)

        return ArmGoal.Result(success=True, message=f"Successfully moved {direction}")

    def move_end_effector_direction(self, direction="up"):
        # generate
        pos = self.pybullet_robot_controller.move_ee_relative_example(
            direction=direction,
            distance=0.05,
        )
        robot_angle = self.pybullet_robot_controller.generateInterpolatedTrajectory(
            target_position=pos, steps=5
        )
        for i in robot_angle:
            self.move_real_and_virtual(radian=i)
            time.sleep(0.1)
        return ArmGoal.Result(success=True, message="success")
