import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32MultiArray
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, PointStamped
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from nav_msgs.msg import Path
from ros_communication_pkg.srv import GetLatestData


class CarROSCommunicator(Node):
    def __init__(self):
        super().__init__("ros2_manager")
        self.get_logger().info("car control node starting...")

        # Store the latest received data
        self.latest_data = {}

        # --- Subscribers ---
        self.create_subscription(LaserScan, "/scan", self._lidar_callback, 10)
        self.create_subscription(
            PoseWithCovarianceStamped, "/amcl_pose", self._amcl_pose_callback, 10
        )
        self.create_subscription(
            PoseStamped, "/goal_pose", self._goal_pose_callback, 10
        )

        # --- Service Server ---
        self.create_service(
            GetLatestData, "get_latest_data", self.handle_get_latest_data
        )

    # --- Callback Functions (Store Latest Data) ---
    def _lidar_callback(self, msg):
        self.latest_data["lidar"] = str(
            msg.ranges[:10]
        )  # Convert some data to string for testing

    def _amcl_pose_callback(self, msg):
        self.latest_data["amcl_pose"] = (
            f"({msg.pose.pose.position.x}, {msg.pose.pose.position.y})"
        )

    def _goal_pose_callback(self, msg):
        self.latest_data["goal_pose"] = (
            f"({msg.pose.position.x}, {msg.pose.position.y})"
        )

    # --- Service Handler: Returns Latest Data ---
    def handle_get_latest_data(self, request, response):
        key = request.key
        if key in self.latest_data:
            response.data = self.latest_data[key]
        else:
            response.data = "No data available"
        self.get_logger().info(f"Service Request: {key} -> {response.data}")
        return response


def main():
    rclpy.init()
    node = CarROSCommunicator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
