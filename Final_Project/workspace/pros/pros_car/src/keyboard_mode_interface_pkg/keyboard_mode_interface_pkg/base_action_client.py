import rclpy
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from keyboard_mode_interface_pkg.action_server_handler import handle_action_result


class BaseActionClient:
    def __init__(self, node, action_type, server_name, client_name):
        self._node = node
        self._logger = node.get_logger()
        self._server_name = server_name
        self._client_name = client_name

        self.action_client = ActionClient(self._node, action_type, server_name)
        self.current_goal_handle = None
        self._get_result_future = None
        self._goal_status = None  # Save latest known status

        self._logger.info(f"{client_name} initialized.")

    def send_goal(self, mode):
        goal_msg = self._create_goal_msg(mode)

        if not self.action_client.wait_for_server(timeout_sec=1.0):
            self._logger.error(f"{self._server_name} not available!")
            return False

        self._logger.info(f"Sending goal with mode: {mode}")
        future = self.action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback
        )
        future.add_done_callback(self.goal_response_callback)
        return True

    def cancel_goal(self):
        if self.current_goal_handle is None:
            self._logger.warn("No active goal to cancel.")
            return False

        self._logger.info("Requesting goal cancellation...")
        cancel_future = self.current_goal_handle.cancel_goal_async()
        cancel_future.add_done_callback(self.cancel_result_callback)
        return True

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self._logger.info("Goal rejected.")
            return

        self._logger.info("Goal accepted.")
        self.current_goal_handle = goal_handle
        self._goal_status = GoalStatus.STATUS_ACCEPTED

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def cancel_result_callback(self, future):
        self._logger.info("Cancel request completed.")
        self._goal_status = GoalStatus.STATUS_CANCELED

    def get_result_callback(self, future):
        try:
            goal_result = future.result()
            self._goal_status = goal_result.status

            handle_action_result(
                node=self._node,
                status=goal_result.status,
                result=goal_result.result,
            )
            if goal_result.status == GoalStatus.STATUS_SUCCEEDED:
                self._logger.info("Goal succeeded. Starting next action...")
                self.start_next_action()
        except Exception as e:
            self._logger.error(f"Result callback error: {e}")
        finally:
            self.current_goal_handle = None
            self._get_result_future = None

    def start_next_action(self):
        # 假設你有下一個 client：self.other_action_client
        if self.other_action_client.is_server_ready():
            self.other_action_client.send_goal(mode="some_mode")
        else:
            self._logger.error("Next action server not ready!")

    def feedback_callback(self, feedback_msg):
        pass

    def _create_goal_msg(self, mode):
        raise NotImplementedError("Derived classes must implement this method")

    # ✅ 新增功能：取得 server 狀態相關資訊
    def is_server_ready(self):
        return self.action_client.server_is_ready()

    def has_active_goal(self):
        return self.current_goal_handle is not None

    def get_goal_status(self):
        return self._goal_status
