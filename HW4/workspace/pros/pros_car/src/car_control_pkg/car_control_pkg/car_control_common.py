import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, Twist
from nav_msgs.msg import Path
from car_control_pkg.utils import get_action_mapping, parse_control_signal
import copy
from car_control_pkg.nav2_utils import cal_distance
import json


class CarControlPublishers:
    """Class to manage common car control publishers and methods"""

    @staticmethod
    def create_publishers(node):
        """Create and return common publishers for car control"""
        rear_wheel_pub = node.create_publisher(
            Float32MultiArray, "car_C_rear_wheel", 10
        )
        front_wheel_pub = node.create_publisher(
            Float32MultiArray, "car_C_front_wheel", 10
        )

        return rear_wheel_pub, front_wheel_pub

    @staticmethod
    def create_control_subscription(node, callback):
        """Create subscription for car control signals"""
        return node.create_subscription(String, "car_control_signal", callback, 10)

    @staticmethod
    def publish_control(node, action, rear_wheel_pub, front_wheel_pub=None):
        """
        If the action is a string, it will be converted to a velocity array using the action mapping.
        If the action is a list, it will be used as the velocity array directly.
        """
        if not isinstance(action, str):
            vel = [action[0], action[1], action[0], action[1]]

        else:
            vel = get_action_mapping(action)

        if front_wheel_pub is None:
            # Only rear wheel publisher is available
            rear_msg = Float32MultiArray()
            rear_msg.data = vel  # Use entire velocity array [0:4]
            rear_wheel_pub.publish(rear_msg)
            node.get_logger().debug(f"Publishing all control data to rear wheel: {vel}")
        else:
            # Both publishers are available
            rear_msg = Float32MultiArray()
            front_msg = Float32MultiArray()
            front_msg.data = vel[0:2]
            rear_msg.data = vel[2:4]
            rear_wheel_pub.publish(rear_msg)
            front_wheel_pub.publish(front_msg)
            node.get_logger().debug(
                f"Publishing split control data: front={vel[0:2]}, rear={vel[2:4]}"
            )


class BaseCarControlNode(Node):
    """Base class for car control nodes providing common functionality"""

    def __init__(self, node_name, enable_nav_subscribers=False):
        super().__init__(node_name)

        # Create common publishers
        self.rear_wheel_pub, self.front_wheel_pub = (
            CarControlPublishers.create_publishers(self)
        )

        # Publisher to clear the plan topic
        self.plan_clear_pub = self.create_publisher(Path, '/plan', 10)
        self.goal_clear_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)

        # Create subscription to control signals
        self.subscription = CarControlPublishers.create_control_subscription(
            self, self.key_callback
        )

        # Navigation data storage
        self.latest_amcl_pose = None
        self.latest_goal_pose = None
        self.latest_global_plan = None
        self.latest_camera_depth = None
        self.latest_yolo_info = None
        self.latest_cmd_vel = None

        # Create navigation data subscribers if enabled
        if enable_nav_subscribers:
            self._create_navigation_subscribers()
            
    def clear_goal_pose(self):
        """
        Clear the /goal_pose topic by publishing an empty PoseStamped message
        and resetting internal stored goal pose.
        """
        empty = PoseStamped()
        empty.header.stamp = self.get_clock().now().to_msg()
        empty.header.frame_id = ''
        # Position and orientation default to zero
        empty.pose.position.x = 0.0
        empty.pose.position.y = 0.0
        empty.pose.position.z = 0.0
        empty.pose.orientation.x = 0.0
        empty.pose.orientation.y = 0.0
        empty.pose.orientation.z = 0.0
        empty.pose.orientation.w = 1.0
        self.goal_clear_pub.publish(empty)
        self.latest_goal_pose = None
        self.get_logger().info('Cleared /goal_pose topic')

    def clear_plan(self):
        """
        Clear the /plan topic by publishing an empty Path message
        and resetting internal stored plan.
        """
        empty = Path()
        empty.header.stamp = self.get_clock().now().to_msg()
        empty.header.frame_id = ''
        empty.poses = []
        self.plan_clear_pub.publish(empty)
        self.latest_global_plan = None
        self.get_logger().info('Cleared /plan topic')

    def _create_navigation_subscribers(self):
        """Create all subscribers needed for navigation"""
        self.amcl_sub = self.create_subscription(
            PoseWithCovarianceStamped, "/amcl_pose", self._amcl_callback, 10
        )

        self.goal_pose_sub = self.create_subscription(
            PoseStamped, "/goal_pose", self._goal_pose_callback, 10
        )

        self.plan_sub = self.create_subscription(
            Path, "/received_global_plan", self._global_plan_callback, 1
        )

        self.cmd_vel_sub = self.create_subscription(
            Twist, "/cmd_vel", self.cmd_vel_callback, 10
        )

        self.camera_depth_sub = self.create_subscription(
            Float32MultiArray,
            "/camera/x_multi_depth_values",
            self._camera_depth_callback,
            10,
        )
        self.object_coordinates = {}
        self.yolo_sub = self.create_subscription(
            String, "/yolo/object/offset", self._yolo_callback, 10
        )

        self.get_logger().info("Navigation subscribers created")

    # Callback methods for navigation data
    def _amcl_callback(self, msg):
        """Store latest AMCL pose"""
        self.latest_amcl_pose = msg

    def _goal_pose_callback(self, msg):
        """Store latest AMCL pose"""
        self.latest_goal_pose = msg

    def _global_plan_callback(self, msg):
        """Store latest global plan"""
        self.latest_global_plan = msg

    def _camera_depth_callback(self, msg):
        """Store latest camera depth data"""
        self.latest_camera_depth = list(msg.data)

    def _yolo_callback(self, msg):
        """ "Callback function for processing incoming YOLO object offset data."""
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

    def get_goal_pose(self):
        """Get goal position or None if unavailable"""
        if self.latest_goal_pose is None:
            return None

        try:
            return self.latest_goal_pose.pose.position
        except AttributeError:
            # Handle cases where the message structure is unexpected
            self.get_logger().warn("Goal pose has unexpected structure")
            return None

    # Helper methods for navigation data access
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

    def cmd_vel_callback(self, msg: Twist):
        wheel_distance = 0.5
        max_speed = 30.0
        min_speed = -30.0
        v = msg.linear.x
        omega = msg.angular.z

        v_left = v - (wheel_distance / 2.0) * omega
        v_right = v + (wheel_distance / 2.0) * omega

        v_left = max(min_speed, min(max_speed, v_left))
        v_right = max(min_speed, min(max_speed, v_right))

        speed_msg = Float32MultiArray()
        speed_msg.data = [v_left, v_right]
        self.latest_cmd_vel = [v_left, v_right]

    def get_cmd_vel_data(self):
        return self.latest_cmd_vel

    def get_path_points(self, include_orientation=True):
        path_points = []

        plan_to_use = self.latest_global_plan
        if plan_to_use and plan_to_use.poses:
            for pose in plan_to_use.poses:
                pos = pose.pose.position
                if include_orientation:
                    # Return both position and orientation data
                    orient = pose.pose.orientation
                    path_points.append(
                        {
                            "position": [pos.x, pos.y, pos.z],
                            "orientation": [orient.x, orient.y, orient.z, orient.w],
                        }
                    )
                else:
                    path_points.append([pos.x, pos.y])
        return path_points

    # Common methods for all car control nodes
    def key_callback(self, msg):
        """Parse control signal and delegate to handle_command"""
        mode, command = parse_control_signal(msg.data)  # Parse signal
        if mode is None or command is None:
            return

        # Call the handle_command method that derived classes implement
        self.handle_command(mode, command)

    def publish_control(self, action):
        """Common method to publish control actions"""
        CarControlPublishers.publish_control(
            self, action, self.rear_wheel_pub, self.front_wheel_pub
        )

    # If you inherit from this class, you must implement this method
    def handle_command(self, mode, command):
        """Handle parsed commands - to be implemented by subclasses"""
        # Default implementation does nothing
        pass
