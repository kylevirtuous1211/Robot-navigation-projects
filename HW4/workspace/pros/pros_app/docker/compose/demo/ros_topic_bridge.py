import rclpy
from rclpy.node import Node
from roslibpy import Ros, Topic
import yaml
import importlib
from std_msgs.msg import Header  # for type hint only
import time

class RemoteTopicBridge(Node):
    def __init__(self, config_path: str):
        super().__init__("remote_topic_bridge")

        # 讀取 YAML 設定
        with open(config_path, "r") as f:
            config = yaml.safe_load(f)

        self.remote_ros = Ros(config["remote_host"], config["remote_port"])
        self.remote_ros.run()
        self.get_logger().info(
            f"Connected to remote ROSBridge at {config['remote_host']}:{config['remote_port']}"
        )

        self.topic_pairs = []  # (remote_topic_obj, local_publisher)

        for topic_cfg in config["topics"]:
            remote_topic_name = topic_cfg["remote_topic"]
            local_topic_name = topic_cfg["local_topic"]
            msg_type_full = topic_cfg["message_type"]  # e.g., "sensor_msgs/msg/Image"

            # 取得 ROS2 message 類型
            pkg, msg_type = msg_type_full.split("/msg/")
            ros2_msg_module = importlib.import_module(f"{pkg}.msg")
            ros2_msg_class = getattr(ros2_msg_module, msg_type)

            # 建立本地端 Publisher
            local_pub = self.create_publisher(ros2_msg_class, local_topic_name, 10)

            # 建立遠端訂閱者（roslibpy）
            remote_topic = Topic(
                self.remote_ros, remote_topic_name, msg_type_full.replace("/msg/", "/")
            )
            remote_topic.subscribe(
                lambda msg, lt=local_topic_name, lp=local_pub, cls=ros2_msg_class: self.republish(
                    msg, lt, lp, cls
                )
            )

            self.get_logger().info(
                f"Bridging remote topic {remote_topic_name} to local topic {local_topic_name}"
            )
            self.topic_pairs.append((remote_topic, local_pub))

    def republish(self, msg_dict, local_topic_name, publisher, msg_class):
        try:
            ros2_msg = msg_class()
            self._fill_ros_msg_fields(ros2_msg, msg_dict)
            for i in range(5):
                publisher.publish(ros2_msg)
                time.sleep(0.1)

        except Exception as e:
            self.get_logger().error(
                f"Failed to republish message on {local_topic_name}: {e}"
            )

    def _fill_ros_msg_fields(self, ros2_msg, msg_dict):
        for key, value in msg_dict.items():
            if not hasattr(ros2_msg, key):
                continue
            attr = getattr(ros2_msg, key)
            # 若是巢狀 message（如 Header），要遞迴建立
            if hasattr(attr, "__slots__") and isinstance(value, dict):
                self._fill_ros_msg_fields(attr, value)
            else:
                setattr(ros2_msg, key, value)



def main():
    import sys

    if len(sys.argv) < 2:
        print("Usage: python ros_topic_bridge.py path_to_config.yaml")
        return

    rclpy.init()
    node = RemoteTopicBridge(sys.argv[1])
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Shutting down bridge...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
