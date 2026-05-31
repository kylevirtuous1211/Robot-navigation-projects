from action_interface.action import ArmGoal
from keyboard_mode_interface_pkg.base_action_client import BaseActionClient


class ArmActionClient(BaseActionClient):
    def __init__(self, node):
        """
        Initializes the ArmActionClient.

        Args:
            node: The rclpy.node.Node instance to use for communication.
        """
        super().__init__(
            node=node,
            action_type=ArmGoal,
            server_name="arm_action_server",
            client_name="ArmActionClient",
        )

    def _create_goal_msg(self, mode):
        goal_msg = ArmGoal.Goal()
        goal_msg.mode = mode
        return goal_msg

    # Public API methods with descriptive names
    def send_arm_mode(self, mode):
        """Send an arm mode goal"""
        return self.send_goal(mode)

    def cancel_arm(self):
        """Cancel the current arm goal if one exists"""
        return self.cancel_goal()
