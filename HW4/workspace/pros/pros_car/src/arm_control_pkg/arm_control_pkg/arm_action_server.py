import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from action_interface.action import ArmGoal
import functools  # Import functools


class ArmActionServer(Node):
    def __init__(self, arm_commute_node, arm_auto_controller):
        super().__init__("arm_action_server_node")
        self._action_server = ActionServer(
            self,
            ArmGoal,
            "arm_action_server",
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
        )
        self.arm_commute_node = arm_commute_node
        self.arm_auto_controller = arm_auto_controller
        self.get_logger().info("Arm Action Server initialized")

    def goal_callback(self, goal_request):
        self.get_logger().info("Received arm auto request")
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info("Enter the cancel callback")
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        result = ArmGoal.Result()
        mode = goal_handle.request.mode

        # 建立可被下游查問的取消函式
        should_cancel = lambda: goal_handle.is_cancel_requested
        self.get_logger().info(f"Executing arm action in mode: {mode}")

        # 選擇對應的自動化方法
        arm_auto_method = self._select_arm_auto_method(mode, should_cancel)
        if arm_auto_method is None:
            self.get_logger().error(f"Unknown mode: {mode}")
            result.success = False
            result.message = f"Unknown mode: {mode}"
            return result

        rate = self.create_rate(10)
        while rclpy.ok():
            rate.sleep()

            # 處理取消請求
            if goal_handle.is_cancel_requested:
                self.get_logger().info("Navigation canceled by user")
                result = ArmGoal.Result(success=False, message="Navigation canceled")
                goal_handle.canceled()
                break

            # 執行自動化方法
            arm_auto_result = arm_auto_method()
            if isinstance(arm_auto_result, ArmGoal.Result):
                if arm_auto_result.success:
                    self.get_logger().info(
                        f"Arm action completed: {arm_auto_result.message}"
                    )
                    goal_handle.succeed()
                else:
                    self.get_logger().error(
                        f"Arm action failed: {arm_auto_result.message}"
                    )
                    goal_handle.abort()
                result = arm_auto_result
                break

            # 發佈反饋
            self._publish_feedback(goal_handle)

        return result

    def _select_arm_auto_method(self, mode: str, should_cancel):
        """
        根據模式選擇對應的 arm_auto_controller 方法或創建一個可調用對象。
        """
        print(mode)
        if mode == "wave":
            return functools.partial(self.arm_auto_controller.arm_wave, should_cancel=should_cancel)
        elif mode == "catch":
            return functools.partial(self.arm_auto_controller.catch, should_cancel=should_cancel)
        elif mode == "catch2":
            return functools.partial(self.arm_auto_controller.catch2, should_cancel=should_cancel)
        elif mode == "object_follow":
            return functools.partial(self.arm_auto_controller.object_follow, should_cancel=should_cancel)
        elif mode == "test":
            return self.arm_auto_controller.test
        elif mode == "look_up":
            return self.arm_auto_controller.look_up
        elif mode == "init_pose":
            return self.arm_auto_controller.init_pose
        elif mode in ["up", "down", "right", "left"]:
            # 使用 functools.partial 創建一個新的可調用對象
            # 這個對象在被調用時，會執行 move_end_effector_direction 並傳入固定的 direction=mode
            return functools.partial(
                self.arm_auto_controller.move_end_effector_direction, direction=mode
            )
        elif mode in ["forward", "backward"]:
            return functools.partial(
                self.arm_auto_controller.move_forward_backward, direction=mode
            )
        else:
            self.get_logger().error(f"Unknown mode requested: {mode}")  # Log error here
            return None  # Return None for unknown modes

    def _check_and_handle_cancel(self, goal_handle, result):
        """
        檢查是否收到取消請求，並處理停止動作與回報
        返回 (canceled: bool, result)
        """
        if goal_handle.is_cancel_requested:
            self.get_logger().info("Arm action canceled by client.")
            goal_handle.canceled()
            result.success = False
            result.message = "Canceled by user"
            return True, result
        return False, result

    def _publish_feedback(self, goal_handle, distance: float = 0.0):
        """
        建立並發佈 Feedback 資訊
        """
        feedback = ArmGoal.Feedback()
        feedback.distance_to_goal = float(distance)
        goal_handle.publish_feedback(feedback)
