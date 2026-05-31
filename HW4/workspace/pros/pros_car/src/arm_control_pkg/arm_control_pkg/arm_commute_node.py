import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String, Float32
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import Imu  # Import the Imu message type
import math
from rclpy.clock import Clock
from builtin_interfaces.msg import Time
import numpy as np
from scipy.spatial.transform import Rotation as R
import json  # Import the json module
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, Twist
from visualization_msgs.msg import Marker

# can change one index angle
# chnage the angle of all joints
# can increase or decrease a single joint angle


class ArmCummuteNode(Node):
    def __init__(self, arm_params, arm_angle_control):
        super().__init__("arm_commute_node")
        self.arm_angle_control = arm_angle_control
        # Load parameters first
        self.arm_params = arm_params.get_arm_params()

        # Initialize arm parameters publisher
        self.arm_pub = self.create_publisher(
            JointTrajectoryPoint, self.arm_params["global"]["arm_topic"], 10
        )

        self.rear_wheel_pub = self.create_publisher(
            Float32MultiArray,
            'car_C_rear_wheel',
            10
        )
        self.front_wheel_pub = self.create_publisher(
            Float32MultiArray,
            'car_C_front_wheel',
            10
        )

        self.arucode_pub = self.create_publisher(Float32, '/aruco/id100/depth_m', 10)

        self.goal_pose_pub = self.create_publisher(
            PoseStamped,
            'goal_pose_tmp',
            10
        )

        self.marker_pub = self.create_publisher(
            Marker,
            'goal_marker',
            10
        )

        # --- Add IMU Subscriber ---
        self.latest_imu_data = None
        self.imu_sub = self.create_subscription(
            Imu,
            self.arm_params["global"][
                "imu_receive_topic"
            ],  # Get topic name from config
            self.imu_callback,
            10,
        )
        self.get_logger().info(
            f"Subscribing to IMU topic: {self.arm_params['global']['imu_receive_topic']}"
        )
        # --------------------------

        # --- Add arucode Subscriber ---
        self.latest_arucode_depth = None
        self.arucode_sub = self.create_subscription(
            Float32,               # 訊息型別
            '/aruco/id100/depth_m', # topic 名稱
            self.arucode_sub_callback,   # callback 函式
            10                     # QoS
        )
        # --------------------------

        # --- Add yolo object offset Subscriber ---
        self.object_coordinates = {}
        self.yolo_object_offset_sub = self.create_subscription(
            String,
            self.arm_params["global"][
                "yolo_object_offset_receive_topic"
            ],  # Get topic name from config
            self.yolo_object_offset_callback,
            1,
        )
        self.get_logger().info(
            f"Subscribing to IMU topic: {self.arm_params['global']['imu_receive_topic']}"
        )
        # --------------------------
        # --- Add AMCL Pose Subscriber ---
        self.latest_amcl_pose = None
        self.amcl_sub = self.create_subscription(
            PoseWithCovarianceStamped, "/amcl_pose", self._amcl_callback, 10
        )

    def clear_arucode_topic(self):
        msg = Float32()
        msg.data = float('nan')
        self.arucode_pub.publish(msg)

    def _amcl_callback(self, msg):
        """Store latest AMCL pose"""
        self.latest_amcl_pose = msg

    def get_car_position_and_orientation(self):
        """
        Get current car position and orientation

        Returns:
            Tuple containing (position, orientation) or (None, None) if data unavailable
        """
        if self.latest_amcl_pose:
            position = self.latest_amcl_pose.pose.pose.position
            orientation = self.latest_amcl_pose.pose.pose.orientation
            return position, orientation
        return None, None

    def imu_callback(self, msg: Imu):
        """Callback function for processing incoming IMU data."""
        # Example: Log the orientation quaternion
        orientation = msg.orientation
        self.latest_imu_data = msg
        # You can add more processing here, e.g., converting quaternion to Euler angles
        # or using linear acceleration/angular velocity data.

    def get_latest_imu_data(self):
        """
        回傳收到的最新 IMU 方向四元數 [x, y, z, w]。
        如果還沒收到過就回傳 None。
        """
        orientation = self.latest_imu_data.orientation
        return [orientation.x, orientation.y, orientation.z, orientation.w]

    def publish_pos(self):
        """Publish a goal 50cm in front of the current orientation + Marker"""
        position, orientation = self.get_car_position_and_orientation()

        quat = [orientation.x, orientation.y, orientation.z, orientation.w]
        rot = R.from_quat(quat)
        forward_vec = np.array([1.0, 0.0, 0.0])  # 向前 50cm
        offset = rot.apply(forward_vec)

        goal_position = [
            position.x + offset[0],
            position.y + offset[1],
            position.z + offset[2]
        ]

        # PoseStamped 發佈
        msg = PoseStamped()
        msg.header.stamp = Clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.pose.position.x = goal_position[0]
        msg.pose.position.y = goal_position[1]
        msg.pose.position.z = goal_position[2]
        msg.pose.orientation = orientation
        self.goal_pose_pub.publish(msg)

        # Marker 發佈
        marker = Marker()
        marker.header.stamp = msg.header.stamp
        marker.header.frame_id = "map"
        marker.ns = "goal_marker"
        marker.id = 0
        marker.type = Marker.ARROW  # 你也可以用 SPHERE 或 CUBE
        marker.action = Marker.ADD
        marker.pose.position.x = goal_position[0]
        marker.pose.position.y = goal_position[1]
        marker.pose.position.z = goal_position[2]
        marker.pose.orientation = orientation  # 箭頭朝向

        marker.scale.x = 0.4  # 箭頭長度
        marker.scale.y = 0.1  # 箭頭寬度
        marker.scale.z = 0.1  # 箭頭高度

        marker.color.a = 1.0  # alpha 透明度
        marker.color.r = 1.0  # 紅色
        marker.color.g = 0.0
        marker.color.b = 0.0

        self.marker_pub.publish(marker)
        self.get_logger().info("Published goal marker at 50cm front")


    def publish_control(self, vel):
        # Both publishers are available
        rear_msg = Float32MultiArray()
        front_msg = Float32MultiArray()
        front_msg.data = vel[0:2]
        rear_msg.data = vel[2:4]
        self.rear_wheel_pub.publish(rear_msg)
        self.front_wheel_pub.publish(front_msg)

    def arucode_sub_callback(self, msg: Float32):
        self.latest_arucode_depth = msg.data

    def get_latest_arucode_depth(self):
        return self.latest_arucode_depth

    def clear_arucode_signal(self):
        self.latest_arucode_depth = None

    def yolo_object_offset_callback(self, msg: String):
        """Callback function for processing incoming YOLO object offset data."""
        try:
            # Extract the JSON string from the message data
            json_string = msg.data
            # Parse the JSON string into a Python list of dictionaries
            object_list = json.loads(json_string)

            # Create a new dictionary mapping labels to coordinates
            new_coordinates = {}
            for item in object_list:
                if isinstance(item, dict) and "label" in item and "offset_flu" in item:
                    label = item["label"]
                    coordinates = item["offset_flu"]
                    # Ensure coordinates are a list of floats
                    if isinstance(coordinates, list) and len(coordinates) == 3:
                        try:
                            float_coords = [float(c) for c in coordinates]
                            new_coordinates[label] = float_coords
                        except (ValueError, TypeError):
                            self.get_logger().warn(
                                f"Invalid coordinate format for label '{label}': {coordinates}"
                            )
                    else:
                        self.get_logger().warn(
                            f"Unexpected coordinate format for label '{label}': {coordinates}"
                        )
                else:
                    self.get_logger().warn(
                        f"Skipping invalid item in JSON list: {item}"
                    )

            # Update the stored coordinates
            self.object_coordinates = new_coordinates
            # self.get_logger().info(
            #     f"Updated object coordinates: {self.object_coordinates}"
            # )
        except json.JSONDecodeError as e:
            self.get_logger().error(f"Failed to decode JSON string: {e}")
            self.get_logger().error(f"Received string: {msg.data}")
        except Exception as e:
            self.get_logger().error(f"Error processing YOLO offset message: {e}")

    def get_latest_object_coordinates(self, label: str = None) -> dict:
        """
        回傳解析後的 YOLO 物體偏移字典，
        格式 { label: [x, y, z], … }，
        若還沒收到就回空 dict。
        """
        if label is None:
            # 全部回傳
            return self.object_coordinates
        # 單一物體回傳
        return self.object_coordinates.get(label, None)

    def degrees_to_radians(self, degree_positions):
        """Convert a list of positions from degrees to radians using NumPy

        Args:
            degree_positions (list): Joint positions in degrees

        Returns:
            list: Joint positions in radians
        """
        try:
            # Convert to numpy array, then use np.deg2rad for efficient conversion
            positions_array = np.array(degree_positions, dtype=float)
            radian_positions = np.deg2rad(positions_array).tolist()
            return radian_positions
        except (ValueError, TypeError) as e:
            # Fall back to element-by-element conversion if array conversion fails
            self.get_logger().warn(
                f"Could not convert all values at once: {e}, falling back to individual conversion"
            )
            radian_positions = []
            for pos in degree_positions:
                try:
                    radian_positions.append(float(pos) * math.pi / 180.0)
                except (ValueError, TypeError):
                    self.get_logger().error(f"Invalid angle value: {pos}")
                    radian_positions.append(0.0)
            return radian_positions

    def publish_arm_angle(self):
        """Publish the current arm joint angles"""
        joint_positions = self.arm_angle_control.get_arm_angles()
        msg = JointTrajectoryPoint()
        radian_positions = self.degrees_to_radians(joint_positions)
        msg.positions = radian_positions
        msg.velocities = []
        msg.accelerations = []
        msg.effort = []
        msg.time_from_start.sec = 0
        msg.time_from_start.nanosec = 0
        self.arm_pub.publish(msg)
        # self.get_logger().info(f"Published angles in radians: {radian_positions}")
