from action_interface.action import NavGoal
from keyboard_mode_interface_pkg.base_action_client import BaseActionClient
from action_msgs.msg import GoalStatus


class CarActionClient(BaseActionClient):
    def __init__(self, node):
        """
        Initializes the CarActionClient.

        Args:
            node: The rclpy.node.Node instance to use for communication.
        """
        super().__init__(
            node=node,
            action_type=NavGoal,
            server_name="nav_action_server",
            client_name="CarActionClient",
        )

    def _create_goal_msg(self, mode):
        goal_msg = NavGoal.Goal()
        goal_msg.mode = mode
        return goal_msg

    # Public API methods with descriptive names
    def send_navigation_goal(self, mode):
        """Send a navigation goal with the specified mode"""
        return self.send_goal(mode)

    def cancel_navigation_goal(self):
        """Cancel the current navigation goal if one exists"""
        return self.cancel_goal()

    def start_next_action(self, next_action=None):
        """
        覆蓋父類的 start_next_action 方法，在導航完成後執行指定的後續動作

        Args:
            next_action: 函數或 lambda，導航完成後要執行的動作
        """
        self._logger.info("導航完成，開始執行後續動作")

        # 如果有提供後續動作，則執行它
        if next_action is not None:
            try:
                next_action()
                self._logger.info("後續動作已執行完成")
            except Exception as e:
                self._logger.error(f"執行後續動作時發生錯誤: {e}")
        else:
            self._logger.info("沒有指定後續動作")

    def is_navigation_successful(self):
        """
        Check if the last navigation goal completed successfully.

        Returns:
            bool: True if the last goal status was SUCCESS, False otherwise
        """
        return self.get_goal_status() == GoalStatus.STATUS_SUCCEEDED
