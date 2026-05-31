"""
This class is for control robot arm angles,
all angle need to thought self.joint_positions to change
"""


class ArmAngleControl:
    def __init__(self, arm_params):
        self.arm_params = arm_params.get_arm_params()
        self.joint_positions = []
        self.arm_init()

    def get_arm_angles(self):
        return self.joint_positions

    def arm_init(self):
        """Initialize arm joints to reset positions"""
        joints_count = int(self.arm_params["global"]["joints_count"])
        self.joint_positions = []

        for i in range(joints_count):
            try:
                pos = float(self.arm_params["joints_reset"][i])
            except (KeyError, ValueError, TypeError):
                pos = 90.0
                self._node.get_logger().warn(
                    f"Joint {i} has invalid or missing reset_position. Using default 90.0."
                )
            self.joint_positions.append(pos)

        print(f"Initialized arm with positions: {self.joint_positions}")

    def arm_default_change(self):
        """Change a joint angle to its default position"""
        # Get the total number of joints from config
        joints_reset = self.arm_params["joints_reset"]
        for index in range(len(self.joint_positions)):
            self.joint_positions[index] = float(joints_reset[index])
        return self.joint_positions

    def arm_index_change(self, index, angle):
        """Just change a joint angle to a specified value"""
        self.joint_positions[index] = angle
        self.joint_positions = self.validate_joint_limits(self.joint_positions)

    def arm_all_change(self, angles):
        """Change all joint angles to specified values"""
        # Validate and set the new angles
        self.joint_positions = self.validate_joint_limits(angles)

    def arm_increase_decrease(self, index, delta):
        """Increase or decrease a joint angle by a specified amount

        Args:
            index (int): Index of the joint to adjust
            delta (float): Amount to adjust the angle by (positive to increase, negative to decrease)

        Returns:
            list: Updated joint positions after adjustment and validation
        """
        # Check if index is valid
        joints_count = self.arm_params["global"]["joints_count"]
        if index < 0 or index >= joints_count:
            print(
                f"Invalid joint index {index}. Must be between 0 and {joints_count-1}"
            )
            return self.joint_positions

        self.joint_positions[index] = self.joint_positions[index] + delta

        # Validate all joint limits and return the updated positions
        self.joint_positions = self.validate_joint_limits(self.joint_positions)

    def validate_joint_limits(self, positions):
        """Validate joint positions against limits from YAML config"""
        # Make a copy to avoid modifying the original list
        for joint_key in range(len(positions)):
            min_angle = float(
                self.arm_params["joints"][joint_key].get("min_angle", 0.0)
            )
            max_angle = float(
                self.arm_params["joints"][joint_key].get("max_angle", 180.0)
            )
            positions[joint_key] = max(min(positions[joint_key], max_angle), min_angle)

        # Return the validated positions
        # print(f"Final validated positions: {positions}")
        return positions
