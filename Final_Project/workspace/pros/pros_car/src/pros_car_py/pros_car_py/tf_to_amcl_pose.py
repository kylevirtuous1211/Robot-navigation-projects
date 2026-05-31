"""
tf_to_amcl_pose
===============

Final Project helper node.

When running **online SLAM (slam_toolbox) + Nav2** (instead of AMCL on a stored
map), the robot's pose is only available as the ``map -> base_footprint`` TF.
The existing return-to-start follower in ``nav_processing.py`` however reads the
``/amcl_pose`` *topic* (a ``PoseWithCovarianceStamped`` normally published by
AMCL, which we are not running).

This node bridges that gap: it looks up the ``map -> base_footprint`` transform
and republishes it as ``/amcl_pose`` at a fixed rate, so the existing Nav2
follower works unchanged.
"""

import rclpy
from rclpy.node import Node
import tf2_ros
from geometry_msgs.msg import PoseWithCovarianceStamped


class TfToAmclPose(Node):
    def __init__(self):
        super().__init__("tf_to_amcl_pose")

        # 可調整的座標系名稱 (與 URDF / SLAM 設定一致)
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("publish_rate", 10.0)

        self.map_frame = self.get_parameter("map_frame").value
        self.base_frame = self.get_parameter("base_frame").value
        rate = float(self.get_parameter("publish_rate").value)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, "/amcl_pose", 10
        )

        self.timer = self.create_timer(1.0 / rate, self.publish_pose)
        self._warned = False
        self.get_logger().info(
            f"tf_to_amcl_pose: republishing {self.map_frame} -> "
            f"{self.base_frame} as /amcl_pose @ {rate:.0f} Hz"
        )

    def publish_pose(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame, self.base_frame, rclpy.time.Time()
            )
        except Exception as e:
            # SLAM 尚未發布 TF 時會持續失敗，僅警告一次避免洗版
            if not self._warned:
                self.get_logger().warn(f"TF not available yet: {e}")
                self._warned = True
            return
        self._warned = False

        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.map_frame

        t = tf.transform.translation
        r = tf.transform.rotation
        msg.pose.pose.position.x = t.x
        msg.pose.pose.position.y = t.y
        msg.pose.pose.position.z = t.z
        msg.pose.pose.orientation = r
        # 小而非零的協方差，避免下游節點把它當成完全無不確定性。
        msg.pose.covariance[0] = 0.01   # x
        msg.pose.covariance[7] = 0.01   # y
        msg.pose.covariance[35] = 0.02  # yaw

        self.pose_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TfToAmclPose()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
