import rclpy
from car_control_pkg.car_control_common import BaseCarControlNode


class ManualControlNode(BaseCarControlNode):
    def __init__(self):
        # Initialize the base class with the node name
        super().__init__("manual_control_node")

    def handle_command(self, mode, command):
        # Only handle Manual Control mode commands
        if mode == "Manual_Control":
            self.key_control(command)

    def key_control(self, key):
        # Implement key mapping logic
        if key == "w":
            self.publish_control("FORWARD")
        elif key == "s":
            self.publish_control("BACKWARD")
        elif key == "a":
            self.publish_control("LEFT_FRONT")
        elif key == "d":
            self.publish_control("RIGHT_FRONT")
        elif key == "e":
            self.publish_control("COUNTERCLOCKWISE_ROTATION")
        elif key == "r":
            self.publish_control("CLOCKWISE_ROTATION")
        elif key == "z" or key == "q":
            self.publish_control("STOP")


# def main(args=None):
#     rclpy.init(args=args)
#     node = ManualControlNode()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         node.get_logger().info("Keyboard Interrupt，節點關閉中...")
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()


# if __name__ == "__main__":
#     main()
