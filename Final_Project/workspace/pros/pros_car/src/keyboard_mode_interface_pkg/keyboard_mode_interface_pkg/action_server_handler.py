# nav_result_handler.py

from action_msgs.msg import GoalStatus
from std_msgs.msg import String

status_names = {
    GoalStatus.STATUS_UNKNOWN: "UNKNOWN",
    GoalStatus.STATUS_ACCEPTED: "ACCEPTED",
    GoalStatus.STATUS_EXECUTING: "EXECUTING",
    GoalStatus.STATUS_CANCELING: "CANCELING",
    GoalStatus.STATUS_SUCCEEDED: "SUCCEEDED",
    GoalStatus.STATUS_CANCELED: "CANCELED",
    GoalStatus.STATUS_ABORTED: "ABORTED",
}


def handle_action_result(node, status, result):
    """
    Generic navigation result handler.
    - node: the ROS node instance (for logging)
    - status: GoalStatus (int)
    - result: the actual result from the action server
    - publish_abort_callback: optional function to call when aborted
    """
    status_name = status_names.get(status, f"UNKNOWN STATUS ({status})")
    node.get_logger().info(f"Navigation goal finished with status: {status_name}")

    if status == GoalStatus.STATUS_SUCCEEDED:
        node.get_logger().info(f"SUCCESS: {result.message}")

    elif status == GoalStatus.STATUS_ABORTED:
        node.get_logger().error(f"ABORTED: {result.message}")

    elif status == GoalStatus.STATUS_CANCELED:
        node.get_logger().info(f"CANCELED: {result.message}")

    else:
        node.get_logger().info(
            f"Result: success={result.success}, message='{result.message}'"
        )

    # You could also trigger emergency stop, reset system state, etc.
    # self.publish_car_signal("STOP")
