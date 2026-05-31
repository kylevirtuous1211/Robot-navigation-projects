# ############################################
# A Robot controller for kinematics, dynamics
# and control based on pyBullet framework
#
# Author : Deepak Raina @ IIT Delhi
# Version : 0.1
# ############################################

# Input:
# 1. robot_type: specify urdf file initials eg. if urdf file name is 'ur5.urdf', specify 'ur5'
# 2. controllable_joints: joint indices of controllable joints. If not specified, by default all joints indices except first joint (first joint is fixed joint between robot stand and base)
# 3. end-eff_index: specify the joint indices for end-effector link. If not specified, by default the last controllable_joints is considered as end-effector joint
# 4. time_Step: time step for simulation

import pybullet as p
import numpy as np
import time
import random
import os
from ament_index_python.packages import get_package_share_directory
import xml.etree.ElementTree as ET
import math
from scipy.spatial.transform import Rotation as R
import pybullet_data


class PybulletRobotController:
    def __init__(self, arm_params, arm_angle_control_node):
        self.arm_params = arm_params.get_arm_params()
        self.arm_angle_control_node = arm_angle_control_node
        # self.robot_type = "ur5"
        robot_description_path = get_package_share_directory("robot_description")
        self.urdf_path = os.path.join(robot_description_path, "urdf", "target.urdf")
        self.robot_id = None
        self.num_joints = None
        self.controllable_joints = int(
            self.arm_params["pybullet"]["controllable_joints"]
        )
        self.end_eff_index = int(self.arm_params["pybullet"]["end_eff_index"])
        self.time_step = float(self.arm_params["pybullet"]["time_step"])
        self.previous_ee_position = None
        self.initial_height = float(self.arm_params["pybullet"]["initial_height"])
        self.createWorld(
            GUI=self.arm_params["pybullet"]["gui"],
            view_world=self.arm_params["pybullet"]["view_world"],
        )

        # synchronize the robot with the initial position
        self.set_initial_joint_positions()
        # self.draw_link_axes(link_name="camera_1")
        self.mimic_pairs = {}  # {主控_joint_index: 被控_joint_index}
        self.marker_ids = []
        self.transformed_object_marker_ids = []

    def markPointInFrontOfEndEffector(
        self, distance=0.3, z_offset=0.1, y_offset=0.0, color=[0, 1, 1], visualize=True
    ):
        """
        根據 end-effector 當前四元數，將 (distance, y_offset, z_offset)
        視為在「本地座標系 (X, Y, Z)」下的偏移，轉成世界座標後加到當前 EE 位置。
        - distance: 沿本地 X（前方）
        - y_offset: 沿本地 Y（側向，+為左）
        - z_offset: 沿本地 Z（垂直，+為上）

        Returns:
            list[x, y, z]: 目標點世界座標
        """
        # 初始化/清理視覺化 ID
        if visualize and not hasattr(self, "front_marker_ids"):
            self.front_marker_ids = []
        if visualize:
            for mid in self.front_marker_ids:
                try:
                    p.removeUserDebugItem(mid)
                except:
                    pass
            self.front_marker_ids.clear()

        try:
            ee_state = p.getLinkState(
                self.robot_id, self.end_eff_index, computeForwardKinematics=True
            )
            position = np.array(ee_state[0])    # 世界座標
            orientation = ee_state[1]           # 四元數 (x,y,z,w)

            # 四元數 → 旋轉矩陣（世界座標中的本地基底）
            Rm = np.array(p.getMatrixFromQuaternion(orientation)).reshape(3, 3)
            local_x = Rm[:, 0]   # 本地 X（前）
            local_y = Rm[:, 1]   # 本地 Y（左）
            local_z = Rm[:, 2]   # 本地 Z（上）

            # 把本地偏移映射到世界
            offset_vec = distance * local_x + y_offset * local_y + z_offset * local_z
            target_point = position + offset_vec

        except Exception as e:
            print(f"Error computing EE forward point: {e}")
            try:
                ee_state = p.getLinkState(self.robot_id, self.end_eff_index)
                return list(ee_state[0])
            except:
                return None

        if visualize:
            line_len = 0.05
            # 畫三條短線（十字 + z）
            self.front_marker_ids.append(
                p.addUserDebugLine(
                    (target_point - np.array([line_len, 0, 0])).tolist(),
                    (target_point + np.array([line_len, 0, 0])).tolist(),
                    color, lineWidth=2
                )
            )
            self.front_marker_ids.append(
                p.addUserDebugLine(
                    (target_point - np.array([0, line_len, 0])).tolist(),
                    (target_point + np.array([0, line_len, 0])).tolist(),
                    color, lineWidth=2
                )
            )
            self.front_marker_ids.append(
                p.addUserDebugLine(
                    (target_point - np.array([0, 0, line_len])).tolist(),
                    (target_point + np.array([0, 0, line_len])).tolist(),
                    color, lineWidth=2
                )
            )
        return list(target_point)


    def draw_link_axes(self, link_name=None, axis_length=0.2):
        """
        在指定的 link（默认末端执行器）的位置画出它的 local XYZ 轴（红: X, 绿: Y, 蓝: Z），
        并删除上一次画的指示，避免残留，同时用一个小圆点标出 link 的位置。

        Args:
            link_name (str or None): 要显示的 link 名称；若为 None，则用 end_eff_index。
            axis_length (float): 每个轴的长度
        """
        # 先删除上次的线 & 点
        if not hasattr(self, "link_axes_lines"):
            self.link_axes_lines = []
        for lid in self.link_axes_lines:
            p.removeUserDebugItem(lid)
        self.link_axes_lines.clear()

        # 找到要绘制的 link index
        if link_name is None:
            link_idx = self.end_eff_index
        else:
            link_idx = None
            for jid in range(self.num_joints):
                info = p.getJointInfo(self.robot_id, jid)
                if info[12].decode("utf-8") == link_name:
                    link_idx = jid
                    break
            if link_idx is None:
                self.get_logger().warn(
                    f"Link '{link_name}' not found, using end-effector."
                )
                link_idx = self.end_eff_index

        # 取得该 link 的 world pose
        ls = p.getLinkState(self.robot_id, link_idx)
        pos, orn = np.array(ls[0]), ls[1]

        # 转 quaternion→rotation matrix
        R_mat = np.array(p.getMatrixFromQuaternion(orn)).reshape(3, 3)
        x_axis, y_axis, z_axis = R_mat[:, 0], R_mat[:, 1], R_mat[:, 2]

        # 算各轴终点
        x_end = pos + x_axis * axis_length
        y_end = pos + y_axis * axis_length
        z_end = pos + z_axis * axis_length

        # 画三条线并保存 id
        self.link_axes_lines.append(
            p.addUserDebugLine(pos.tolist(), x_end.tolist(), [1, 0, 0], lineWidth=3)
        )
        self.link_axes_lines.append(
            p.addUserDebugLine(pos.tolist(), y_end.tolist(), [0, 1, 0], lineWidth=3)
        )
        self.link_axes_lines.append(
            p.addUserDebugLine(pos.tolist(), z_end.tolist(), [0, 0, 1], lineWidth=3)
        )

        # --- 新增：在 pos 处画一个小“球”点 ---
        # addUserDebugPoints(points, colors, pointSize)
        self.link_axes_lines.append(
            p.addUserDebugPoints(
                [pos.tolist()],  # 位置列表
                [[1, 1, 0]],  # 颜色列表：黄色
                pointSize=30,  # 点大小，调大一些看着像小球
            )
        )

    def is_link_close_to_position(self, link_name, target_position, threshold):
        """
        計算指定 link 的當前位置與目標位置之間的距離，並判斷是否小於閾值。
        Handles base_link using getBasePositionAndOrientation.

        Args:
            link_name (str): 要檢查的 link 的名稱 (e.g., "base_link", "camera_1").
            target_position (list or tuple): 目標世界座標 [x, y, z]。
            threshold (float): 距離閾值（單位：公尺）。

        Returns:
            bool: 如果 link 的當前位置與目標位置的距離小於 threshold，則返回 True，否則返回 False。
                  如果找不到 link 或輸入無效，也返回 False。
        """
        # --- Input Validation ---
        if not isinstance(target_position, (list, tuple)) or len(target_position) < 3:
            print(
                "錯誤: target_position 必須是包含至少三個元素 [x, y, z] 的列表或元組。"
            )
            return False
        if not isinstance(threshold, (int, float)) or threshold < 0:
            print("錯誤: threshold 必須是一個非負數值。")
            return False

        target_pos_np = np.array(target_position[0:3])

        # --- Get Current Link Position ---
        current_link_pos_np = None
        if link_name == "base_link":
            try:
                # Use the correct function for the base link
                base_pos, _ = p.getBasePositionAndOrientation(self.robot_id)
                current_link_pos_np = np.array(base_pos)
                print(f"獲取 base_link 位置: {current_link_pos_np}")
            except Exception as e:
                print(f"獲取 base_link 位置時發生錯誤: {e}")
                return False
        else:
            # --- Find Link Index for non-base links ---
            # _find_link_index now returns the JOINT index
            joint_idx = self._find_link_index(link_name)
            if joint_idx is None:
                # _find_link_index already prints an error/warning
                return False
            # --- Get Current Link Position using the JOINT index ---
            try:
                # p.getLinkState uses the JOINT index whose child link we want
                link_state = p.getLinkState(
                    self.robot_id, joint_idx, computeForwardKinematics=True
                )
                current_link_pos_np = np.array(link_state[0])  # worldLinkFramePosition
                print(
                    f"獲取 link '{link_name}' (關節索引 {joint_idx}) 位置: {current_link_pos_np}"
                )
            except Exception as e:
                # Catch potential errors if joint_idx is somehow invalid, though _find_link_index should prevent this
                print(
                    f"獲取 link '{link_name}' (關節索引 {joint_idx}) 狀態時發生錯誤: {e}"
                )
                return False

        if current_link_pos_np is None:
            # Safeguard
            print(f"未能確定 link '{link_name}' 的當前位置。")
            return False

        # --- Calculate Distance ---
        distance = np.linalg.norm(current_link_pos_np - target_pos_np)

        # --- Compare with Threshold ---
        is_close = distance < threshold
        # print(f"Link '{link_name}' 當前位置: {current_link_pos_np}, 目標位置: {target_pos_np}") # Already printed above
        print(
            f"計算距離: {distance:.4f} 公尺, 閾值: {threshold} 公尺. 是否在閾值內: {is_close}"
        )

        return is_close

    def _find_link_index(self, link_name):
        """
        Helper function to find the index of a link by its name.
        Returns the joint index whose child link matches the name.
        Does NOT handle "base_link".
        """
        # Iterate through joints to find other links
        if self.num_joints is None:  # Ensure num_joints is initialized
            self.num_joints = p.getNumJoints(self.robot_id)

        for jid in range(self.num_joints):
            try:
                info = p.getJointInfo(self.robot_id, jid)
                # Link name is stored in the 12th element of getJointInfo
                if info[12].decode("utf-8") == link_name:
                    return jid  # Return the JOINT index
            except Exception as e:
                print(f"Error getting info for joint {jid}: {e}")
                # Continue to next joint or handle error appropriately

        # If loop finishes without finding the link
        print(f"Warning: Link '{link_name}' not found by iterating through joints.")
        return None  # Return None if not found

    def calculate_ee_relative_target_positions(self, distance):
        """
        計算相對於當前末端執行器 **局部座標系** 的上、下、左、右、後退目標世界座標。

        Args:
            distance (float): 要在末端執行器局部座標系的各個方向上移動的距離（單位：公尺）。
                              - 上/下: 沿局部 Z 軸
                              - 左/右: 沿局部 Y 軸 (假設 Y 軸指向左側)
                              - 後退: 沿局部 X 軸的反方向 (假設 X 軸指向前方)

        Returns:
            dict: 一個包含五個目標世界座標的字典，鍵為 'up', 'down', 'left', 'right', 'backward'。
                  每個值是一個包含 [x, y, z] 座標的列表。
                  如果無法獲取末端執行器狀態，則返回 None。
        """
        try:
            # 1. 獲取當前末端執行器的世界座標和姿態
            ee_state = p.getLinkState(
                self.robot_id, self.end_eff_index, computeForwardKinematics=True
            )
            current_position = np.array(ee_state[0])  # worldLinkFramePosition
            current_orientation = ee_state[
                1
            ]  # worldLinkFrameOrientation (quaternion x,y,z,w)
        except Exception as e:
            print(f"獲取末端執行器狀態時發生錯誤: {e}")
            return None

        # 2. 將姿態四元數轉換為旋轉矩陣
        try:
            rotation_matrix = np.array(
                p.getMatrixFromQuaternion(current_orientation)
            ).reshape(3, 3)
        except Exception as e:
            print(f"從四元數轉換旋轉矩陣時發生錯誤: {e}")
            return None

        # 3. 提取末端執行器的局部座標軸在世界座標系中的方向向量
        #   - X 軸通常是向前 (索引 0)
        #   - Y 軸通常是向左 (索引 1)
        #   - Z 軸通常是向上 (索引 2)
        local_x_axis = rotation_matrix[:, 0]  # Assuming X is forward
        local_y_axis = rotation_matrix[:, 1]  # Assuming Y is left
        local_z_axis = rotation_matrix[:, 2]  # Assuming Z is up

        # 4. 計算相對於末端執行器局部的目標世界座標
        target_positions = {
            "up": (current_position + local_z_axis * distance).tolist(),
            "down": (current_position - local_z_axis * distance).tolist(),
            "left": (current_position + local_y_axis * distance).tolist(),
            "right": (current_position - local_y_axis * distance).tolist(),
            "backward": (
                current_position - local_x_axis * distance
            ).tolist(),  # Add backward movement
            # 如果需要向前移動，可以添加:
            # 'forward': (current_position + local_x_axis * distance).tolist(),
        }

        print(
            f"基於末端執行器當前位置 {current_position.tolist()} 和姿態計算相對目標點:"
        )
        for direction, pos in target_positions.items():
            print(f"  - {direction} (local): {pos}")

        return target_positions

    def move_ee_relative_example(
        self, direction, distance, visualize=True, execute_move=False
    ):
        """
        計算相對於末端執行器的局部目標世界座標，可選地視覺化並執行移動。

        Args:
            direction (str): 'up', 'down', 'left', 'right', or 'backward'.
            distance (float): 移動距離。
            visualize (bool): 是否在 PyBullet 中標記計算出的目標點。預設為 True。
            execute_move (bool): 是否計算 IK 並嘗試移動手臂到目標位置。預設為 False。

        Returns:
            list or None: 計算出的目標世界座標 [x, y, z]，如果無法計算或方向無效則返回 None。
        """
        target_positions = self.calculate_ee_relative_target_positions(distance)
        target_pos_world = None  # Initialize to None

        if target_positions and direction in target_positions:
            target_pos_world = target_positions[direction]
            print(
                f"計算出相對於末端執行器 '{direction}' 方向的目標世界座標: {target_pos_world}"
            )

            # 可選：視覺化標記目標點
            if visualize:
                self.markTarget(
                    target_pos_world, color=[0, 1, 1]
                )  # Cyan color for target
                print("目標點已標記 (青色)。")

            # 可選：執行移動
            if execute_move:
                print("嘗試執行移動...")
                # 可選：檢查可達性 (假設 is_position_reachable 存在)
                # if hasattr(self, 'is_position_reachable') and self.is_position_reachable(target_pos_world):
                #     print("目標位置可達，計算 IK...")
                # else:
                #     print("警告: 未檢查或目標位置可能不可達。繼續嘗試 IK...")

                # 計算 IK (只關心位置)
                joint_angles = self.solveInversePositionKinematics(target_pos_world)

                if joint_angles:
                    print(
                        f"IK 解算成功: {joint_angles[:len(self.controllable_joints)]}"
                    )
                    # 設置關節角度 (假設 setJointPosition 存在且有效)
                    self.setJointPosition(joint_angles[: len(self.controllable_joints)])
                    print("已執行 setJointPosition。")
                else:
                    print("IK 求解失敗，未執行移動。")
            # else: # if not execute_move
            #     print("未請求執行移動 (execute_move=False)。")

        else:
            print(f"無法計算目標位置或方向 '{direction}' 無效。")
            # target_pos_world remains None

        return target_pos_world  # 返回計算出的世界座標或 None

    def offset_from_end_effector(
        self, x_offset, y_offset, z_offset, visualize=False, mark_color=[1, 0, 1]
    ):
        """
        基於 end-effector 當前姿態，將指定的 local Y/Z 軸偏差轉換為世界座標位置，
        並用 IK 解出對應的關節角度（弧度），同時標記該點。

        Args:
            y_offset (float): 右手座標系中 Y 軸的偏差量（向左為正）
            z_offset (float): 右手座標系中 Z 軸的偏差量（向上為正）
            mark_color (list): 用於標記目標位置的顏色，預設為紫色 [1, 0, 1]

        Returns:
            list: 可行的 IK 解（弧度），若無解則回傳 None
        """
        # Step 1: 取得末端位置與朝向
        ee_state = p.getLinkState(self.robot_id, self.end_eff_index)
        position = np.array(ee_state[0])  # 世界座標
        orientation = ee_state[1]  # 四元數

        # Step 2: 四元數 → 旋轉矩陣
        rot_matrix = np.array(p.getMatrixFromQuaternion(orientation)).reshape(3, 3)

        # Step 3: 抓出 local Y/Z 軸方向向量
        local_x_axis = rot_matrix[:, 0]
        local_y_axis = rot_matrix[:, 1]
        local_z_axis = rot_matrix[:, 2]

        # Step 4: 偏移計算
        offset_vector = (
            x_offset * local_x_axis + y_offset * local_y_axis + z_offset * local_z_axis
        )
        new_position = position + offset_vector

        if visualize:
            # Step 5: 標記該位置
            self.markTarget(new_position, color=mark_color)
        return new_position
        # # Step 6: 解 IK
        # ik_solution = self.solveInversePositionKinematics(list(new_position))

        # if ik_solution:
        #     return ik_solution[: len(self.controllable_joints)]
        # else:
        #     print("❌ 無法計算偏移後的 IK 解")
        #     return None

    def getJointStates(self):
        joint_states = p.getJointStates(self.robot_id, self.controllable_joints)
        joint_positions = [state[0] for state in joint_states]
        joint_velocities = [state[1] for state in joint_states]
        joint_torques = [state[3] for state in joint_states]
        return joint_positions, joint_velocities, joint_torques

    def move_end_effector_laterally(self, distance=0.3):
        """
        根據末端執行器目前旋轉姿態，沿其「右手方向（local-side）」平移一定距離。

        Args:
            distance (float): 要平移的距離，正值向右，負值向左。
        """
        # Step 1: 取得當前末端執行器位置與姿態
        ee_state = p.getLinkState(self.robot_id, self.end_eff_index)
        position = np.array(ee_state[0])  # 世界座標
        orientation = ee_state[1]  # 四元數 (x, y, z, w)

        # Step 2: 四元數 → 旋轉矩陣
        rotation_matrix = np.array(p.getMatrixFromQuaternion(orientation)).reshape(3, 3)

        # Step 3: 抓出旋轉矩陣的「右手方向」單位向量
        right_direction = rotation_matrix[:, 1]  # 通常 local-Y 是側邊 (右手) 方向

        # Step 4: 根據方向向量計算新的目標位置
        target_position = position + right_direction * distance

        # Step 5: 傳入原姿態，保持手的朝向不變
        target_pose = list(target_position) + list(
            p.getEulerFromQuaternion(orientation)
        )
        joint_angles = self.solveInversePositionKinematics(target_pose)

        if joint_angles:
            print("計算出的關節角度:", joint_angles)
            return joint_angles
        else:
            print("無法計算可行的 IK 解。")

    def generateInterpolatedTrajectory(self, target_position, steps=50):
        """
        Generate joint angle trajectory based on target position.

        Args:
            target_position (list or tuple): A [x, y, z] world coordinate of length 3.
            steps (int): Number of interpolation steps, default is 50.

        Returns:
            list: Joint angles (in angle) corresponding to each interpolation point.
        """
        current_position = self.solveForwardPositonKinematics(self.getJointStates()[0])[
            0:3
        ]

        # 計算每一步的位移向量
        step_vector = (np.array(target_position) - np.array(current_position)) / steps
        self.markTarget(target_position)

        # 用於存儲每一步的關節角度（以弧度表示）
        joint_angles_in_radians = []

        # 逐步靠近目標
        for i in range(steps):
            # 計算當前目標位置
            intermediate_position = np.array(current_position) + (i + 1) * step_vector
            # 計算 IK 解
            joint_angles = self.solveInversePositionKinematics(intermediate_position)

            # 將當前關節角度應用到機器人
            if joint_angles and len(joint_angles) >= len(self.controllable_joints):
                # 直接存儲弧度值
                joint_angles_in_radians.append(
                    joint_angles[: len(self.controllable_joints)]
                )
            else:
                print("無法找到合適的解。")
                break

        return joint_angles_in_radians

    def calculate_imu_extrinsics(
        self, imu_world_quaternion, link_name, visualize=False, axis_length=0.1
    ):
        # --- 清掉舊的 markers ---
        for mid in self.marker_ids:
            try:
                p.removeUserDebugItem(mid)
            except:
                pass
        self.marker_ids.clear()

        # --- 找到 link index & 世界座標 ---
        link_idx = None
        for jid in range(self.num_joints):
            if p.getJointInfo(self.robot_id, jid)[12].decode() == link_name:
                link_idx = jid
                break
        if link_idx is None:
            print(f"找不到 link '{link_name}'")
            return None

        ls = p.getLinkState(self.robot_id, link_idx, computeForwardKinematics=True)
        link_origin_world = np.array(ls[4])
        link_orn = ls[5]  # 四元數 (x,y,z,w)

        # --- 拆 yaw from link orientation ---
        rot_link = R.from_quat(link_orn)  # scipy expects [x,y,z,w]
        roll_l, pitch_l, yaw_l = rot_link.as_euler("xyz", False)

        # --- 拆 roll/pitch from IMU orientation ---
        rot_imu = R.from_quat(imu_world_quaternion)
        roll_i, pitch_i, yaw_i = rot_imu.as_euler("xyz", False)

        # --- fuse: 用 imu 的 roll/pitch + link 的 yaw ---
        fused = R.from_euler("xyz", [roll_i, pitch_i, yaw_l], False)
        R_world_imu_fused = fused.as_matrix()  # 從 IMU→世界

        # --- 外參矩陣 world→imu_fused ---
        R_imu_world = R_world_imu_fused.T
        t_imu_world = -R_imu_world @ link_origin_world
        T = np.eye(4)
        T[:3, :3] = R_imu_world
        T[:3, 3] = t_imu_world

        # --- 視覺化 (在 link_origin_world 處) ---
        if visualize:
            # 算三軸
            x_ax = R_world_imu_fused[:, 0] * axis_length
            y_ax = R_world_imu_fused[:, 1] * axis_length
            z_ax = R_world_imu_fused[:, 2] * axis_length

            o = link_origin_world.tolist()
            # 紅 X
            self.marker_ids.append(
                p.addUserDebugLine(o, (link_origin_world + x_ax).tolist(), [1, 0, 0], 4)
            )
            # 綠 Y
            self.marker_ids.append(
                p.addUserDebugLine(o, (link_origin_world + y_ax).tolist(), [0, 1, 0], 4)
            )
            # 藍 Z
            self.marker_ids.append(
                p.addUserDebugLine(o, (link_origin_world + z_ax).tolist(), [0, 0, 1], 4)
            )

        return T

    def solveInversePositionKinematics(self, end_eff_pose):
        """
        計算逆向運動學以獲取關節角度，基於給定的末端執行器姿勢。

        Args:
            end_eff_pose (list): 末端執行器的目標位置和姿勢，
                                格式為 [x, y, z, roll, pitch, yaw] (6 個元素) 或 [x, y, z] (3 個元素)。

        Returns:
            list: 對應的關節角度。
        """
        if len(end_eff_pose) == 6:
            joint_angles = p.calculateInverseKinematics(
                self.robot_id,
                self.end_eff_index,
                targetPosition=end_eff_pose[0:3],
                targetOrientation=p.getQuaternionFromEuler(end_eff_pose[3:6]),
            )
        else:
            joint_angles = p.calculateInverseKinematics(
                self.robot_id, self.end_eff_index, targetPosition=end_eff_pose[0:3]
            )

        # 標記末端執行器的位置路徑
        # self.markEndEffectorPath()
        return joint_angles

    def set_initial_joint_positions(self):
        """從配置中讀取初始關節角度並設置"""
        print("設置初始關節角度...")

        # 轉換為弧度
        initial_positions_deg = self.arm_angle_control_node.get_arm_angles()
        print(f"初始關節角度 (角度): {initial_positions_deg}")
        initial_positions_rad = [math.radians(angle) for angle in initial_positions_deg]

        # 設置關節位置
        self.setJointPosition(initial_positions_rad)

    # function for setting joint positions of robot in pybullet
    def setJointPosition(self, position, kp=1.0, kv=1.0):
        # print('Joint position controller')
        zero_vec = [0.0] * len(self.controllable_joints)
        p.setJointMotorControlArray(
            self.robot_id,
            self.controllable_joints,
            p.POSITION_CONTROL,
            targetPositions=position,
            targetVelocities=zero_vec,
            positionGains=[kp] * len(self.controllable_joints),
            velocityGains=[kv] * len(self.controllable_joints),
        )
        for _ in range(100):  # to settle the robot to its position
            p.stepSimulation()

    def transform_object_to_world(
        self,
        T_world_to_imu,
        object_coords_imu,
        visualize=False,
        marker_color=[0, 1, 1],
        text_color=[1, 1, 1],
    ):
        """
        將相對於 IMU 坐標系的物體座標轉換為世界座標。

        Args:
            T_world_to_imu (np.ndarray): 4x4 的世界座標系到 IMU 坐標系的外參變換矩陣。
            object_coords_imu (list or np.array): 物體在 IMU 坐標系下的 [x, y, z] 座標。
                                                假設為 FLU 坐標系 (X 前, Y 左, Z 上)。
            visualize (bool): 是否在計算出的世界座標位置繪製標記。
            marker_color (list): 視覺化標記的顏色，預設為青色 [0, 1, 1]。

        Returns:
            np.ndarray or None: 物體在世界座標系下的 [x, y, z] 座標，如果輸入無效則返回 None。
        """
        # --- 輸入驗證 ---
        if not isinstance(T_world_to_imu, np.ndarray) or T_world_to_imu.shape != (4, 4):
            print("錯誤: T_world_to_imu 必須是 4x4 numpy 矩陣。")
            return None
        try:
            object_coords_imu = np.array(object_coords_imu, dtype=float)
            if object_coords_imu.shape != (3,):
                raise ValueError("Object coordinates must have 3 elements.")
        except Exception as e:
            print(f"錯誤: object_coords_imu 無效: {e}")
            return None

        # --- 坐標轉換 ---
        try:
            # 1. 計算從 IMU 到世界的逆變換矩陣
            T_imu_to_world = np.linalg.inv(T_world_to_imu)

            # 2. 將物體在 IMU 坐標系的座標轉為齊次向量
            p_imu_homogeneous = np.append(object_coords_imu, 1.0)

            # 3. 應用變換矩陣得到世界坐標（齊次）
            p_world_homogeneous = T_imu_to_world @ p_imu_homogeneous

            # 4. 提取世界坐標的 x, y, z
            object_coords_world = p_world_homogeneous[:3]

        except np.linalg.LinAlgError:
            print("錯誤: 無法計算 T_world_to_imu 的逆矩陣。")
            return None
        except Exception as e:
            print(f"坐標轉換時發生錯誤: {e}")
            return None

        # --- 視覺化 (可選) ---
        # 清除舊的標記
        for mid in self.transformed_object_marker_ids:
            try:
                p.removeUserDebugItem(mid)
            except:
                pass
        self.transformed_object_marker_ids.clear()

        if visualize:
            # 使用 markTarget 函數來繪製十字標記
            self.markTarget(object_coords_world, color=marker_color)
            # 將 markTarget 創建的 ID 保存到這個函數專用的列表中
            # 注意：markTarget 內部會管理 target_marker_ids，這裡我們需要複製一份
            # 或者修改 markTarget 讓它可以選擇性地返回 ID
            # 為了簡單起見，假設 markTarget 畫的線條 ID 存儲在 self.target_marker_ids
            # 我們將這些 ID 複製過來
            self.transformed_object_marker_ids.extend(self.target_marker_ids)
            # 2. 準備要顯示的文字
            coord_text = f"Obj: ({object_coords_world[0]:.2f}, {object_coords_world[1]:.2f}, {object_coords_world[2]:.2f})"
            # 決定文字顯示的位置 (例如，在標記上方一點)
            text_position = object_coords_world + np.array(
                [0, 0, 0.05]
            )  # Offset slightly above the marker

            # 3. 添加除錯文字
            text_id = p.addUserDebugText(
                text=coord_text,
                textPosition=text_position.tolist(),
                textColorRGB=text_color,
                textSize=1.0,  # 可以調整文字大小
                # parentObjectUniqueId=self.robot_id, # 可選：讓文字跟隨某個物體
                # parentLinkIndex=-1 # 可選：讓文字跟隨某個 link
            )
            # 將文字 ID 也加入清除列表
            self.transformed_object_marker_ids.append(text_id)

        return list(object_coords_world)

    def markTarget(self, target_position, color=[1, 0, 0]):
        """
        在給定位置畫紅色十字標記（或指定顏色），會先清除舊的標記。

        Args:
            target_position (list or np.array): 3D 目標世界座標
            color (list): 標記顏色，預設紅色 [1, 0, 0]
        """
        # 初始化 ID list
        if not hasattr(self, "target_marker_ids"):
            self.target_marker_ids = []

        # 清除上次畫的線
        for line_id in self.target_marker_ids:
            p.removeUserDebugItem(line_id)
        self.target_marker_ids.clear()

        # 畫新的十字線
        line_length = 0.1
        self.target_marker_ids.append(
            p.addUserDebugLine(
                [
                    target_position[0] - line_length,
                    target_position[1],
                    target_position[2],
                ],
                [
                    target_position[0] + line_length,
                    target_position[1],
                    target_position[2],
                ],
                color,
                lineWidth=3,
            )
        )
        self.target_marker_ids.append(
            p.addUserDebugLine(
                [
                    target_position[0],
                    target_position[1] - line_length,
                    target_position[2],
                ],
                [
                    target_position[0],
                    target_position[1] + line_length,
                    target_position[2],
                ],
                color,
                lineWidth=3,
            )
        )
        self.target_marker_ids.append(
            p.addUserDebugLine(
                [
                    target_position[0],
                    target_position[1],
                    target_position[2] - line_length,
                ],
                [
                    target_position[0],
                    target_position[1],
                    target_position[2] + line_length,
                ],
                color,
                lineWidth=3,
            )
        )

    def solveForwardPositonKinematics(self, joint_pos):
        # get end-effector link state
        eeState = p.getLinkState(self.robot_id, self.end_eff_index)
        link_trn, link_rot, com_trn, com_rot, frame_pos, frame_rot = eeState
        eePose = list(link_trn) + list(p.getEulerFromQuaternion(link_rot))
        return eePose

    # function to initiate pybullet and engine and create world
    def createWorld(self, GUI=True, view_world=False):
        # load pybullet physics engine
        if GUI:
            physicsClient = p.connect(p.GUI)
        else:
            physicsClient = p.connect(p.DIRECT)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.resetSimulation()
        GRAVITY = -9.8
        p.setGravity(0, 0, GRAVITY)
        p.setTimeStep(self.time_step)
        p.setPhysicsEngineParameter(
            fixedTimeStep=self.time_step, numSolverIterations=100, numSubSteps=10
        )
        p.setRealTimeSimulation(True)
        p.loadURDF("plane.urdf")
        rotation = R.from_euler("z", 90, degrees=True).as_quat()

        # loading robot into the environment
        # urdf_file = "urdf/" + self.robot_type + ".urdf"
        self.robot_id = p.loadURDF(
            self.urdf_path,
            useFixedBase=True,
            basePosition=[0, 0, self.initial_height],
            baseOrientation=rotation,
        )

        self.num_joints = p.getNumJoints(self.robot_id)  # Joints
        # 只保留 Revolute 和 Prismatic 关节
        self.controllable_joints = []
        # 假設你有一個要忽略的 joint name list
        mimic_joint_names = ["Revolute 6"]

        for jid in range(self.num_joints):
            info = p.getJointInfo(self.robot_id, jid)
            joint_type = info[2]
            joint_name = info[1].decode("utf-8")
            if joint_type in (p.JOINT_REVOLUTE, p.JOINT_PRISMATIC):
                if joint_name not in mimic_joint_names:
                    self.controllable_joints.append(jid)
                    link_name = info[12].decode("utf-8")
                    print(f"Joint index {jid} controls link: {link_name}")

        print("#Joints read from pybullet:", self.num_joints)
        print("#Controllable Joints:", self.controllable_joints)
        if self.end_eff_index is None:
            self.end_eff_index = self.controllable_joints[-1]
        print("#End-effector:", self.end_eff_index)

        if view_world:
            while True:
                p.stepSimulation()
                time.sleep(self.time_step)
