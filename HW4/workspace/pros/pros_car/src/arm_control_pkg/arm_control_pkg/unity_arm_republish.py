#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectoryPoint


class JointPointRelay(Node):
    def __init__(self):
        super().__init__("joint_point_relay")

        # 預設訂閱與發布的 topic
        self.sub = self.create_subscription(
            JointTrajectoryPoint, "/robot_arm_tmp", self.cb, 10
        )
        self.pub = self.create_publisher(JointTrajectoryPoint, "/robot_arm", 10)

        self.get_logger().info(
            "Relaying JointTrajectoryPoint: /robot_arm_tmp  ->  /robot_arm"
        )

    def cb(self, msg: JointTrajectoryPoint):
        # 直接轉發
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = JointPointRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
