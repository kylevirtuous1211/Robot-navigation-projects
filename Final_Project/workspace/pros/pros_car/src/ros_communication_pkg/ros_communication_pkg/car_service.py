import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from custome_interfaces.srv import GetScan

# LiDAR global constants
LIDAR_RANGE = 90
LIDAR_PER_SECTOR = 20
FRONT_LIDAR_INDICES = list(range(0, 16)) + list(range(-15, 0))  # front lidar indices
LEFT_LIDAR_INDICES = list(range(16, 46))  # left lidar indices
RIGHT_LIDAR_INDICES = list(range(-45, -15))  # right lidar indices


class DataReceiverServiceNode(Node):
    def __init__(self):
        super().__init__("data_receiver_service_node")
        # 訂閱 /scan topic
        self.scan_sub = self.create_subscription(
            LaserScan, "/scan", self.scan_callback, 10
        )
        # 訂閱 /amcl_pose topic
        self.amcl_pose_sub = self.create_subscription(
            PoseWithCovarianceStamped, "/amcl_pose", self.amcl_pose_callback, 10
        )
        # 訂閱 /goal_pose topic
        self.goal_pose_sub = self.create_subscription(
            PoseStamped, "/goal_pose", self.goal_pose_callback, 10
        )
        # 建立一個 service，等待 client 呼叫時回應最新資料（全部資料摘要）
        self.srv = self.create_service(
            GetScan, "get_latest_data", self.service_callback
        )
        # 建立一個 service 用來取得 /scan 資料（依 request.processed 決定回傳處理過或原始資料）
        self.scan_srv = self.create_service(GetScan, "get_scan", self.get_scan_callback)

        # 儲存各 topic 的最新資料
        self.latest_scan = None
        self.latest_amcl_pose = None
        self.latest_goal_pose = None

        self.get_logger().info(
            "DataReceiverServiceNode 啟動完成，持續接收 /scan、/amcl_pose 與 /goal_pose 資料，並等待 service 呼叫"
        )

    def scan_callback(self, msg: LaserScan):
        self.latest_scan = msg
        self.get_logger().debug("更新 /scan 資料")

    def amcl_pose_callback(self, msg: PoseWithCovarianceStamped):
        self.latest_amcl_pose = msg
        self.get_logger().debug("更新 /amcl_pose 資料")

    def goal_pose_callback(self, msg: PoseStamped):
        self.latest_goal_pose = msg
        self.get_logger().debug("更新 /goal_pose 資料")

    def service_callback(self, request, response):
        self.get_logger().info("收到 get_latest_data service 呼叫，開始回應最新資料")
        if (
            self.latest_scan is None
            or self.latest_amcl_pose is None
            or self.latest_goal_pose is None
        ):
            response.success = False
            response.message = "尚未接收到所有 topic 的資料"
        else:
            scan_stamp = self.latest_scan.header.stamp
            amcl_pos = self.latest_amcl_pose.pose.pose.position
            goal_pos = self.latest_goal_pose.pose.position

            response.success = True
            response.message = (
                f"/scan stamp: {scan_stamp.sec}.{scan_stamp.nanosec}  |  "
                f"/amcl_pose: ({amcl_pos.x:.2f}, {amcl_pos.y:.2f})  |  "
                f"/goal_pose: ({goal_pos.x:.2f}, {goal_pos.y:.2f})"
            )
            # 這邊示範填充部分資料，可依需求調整
            dummy_array = Float32MultiArray()
            dummy_array.data = [float(scan_stamp.sec), float(scan_stamp.nanosec)]
            response.data = dummy_array

        return response

    def get_scan_callback(self, request, response):
        self.get_logger().info("收到 get_scan service 呼叫，開始回應 /scan 資料")
        if self.latest_scan is None:
            response.success = False
            response.message = "尚未接收到 /scan 資料"
            response.data = Float32MultiArray()  # 空陣列
        else:
            if request.processed:
                # 呼叫自訂的處理函式，回傳處理後的 LiDAR 資料
                processed_data = self.get_processed_lidar()
                response.success = True
                response.message = "回傳處理過的 /scan 資料"
            else:
                # 回傳原始 /scan 資料 (例如僅回傳 ranges 資料)
                processed_data = list(self.latest_scan.ranges)
                response.success = True
                response.message = "回傳原始的 /scan 資料"
            array_msg = Float32MultiArray()
            array_msg.data = processed_data
            response.data = array_msg

        return response

    def get_processed_lidar(self):
        """
        處理 /scan 資料：根據全域常數將原始 LiDAR 資料轉換，
        並回傳一個 float 陣列，內容包含前方、左側與右側區域的距離資料。
        """
        lidar_msg = self.latest_scan
        angle_min = lidar_msg.angle_min
        angle_increment = lidar_msg.angle_increment
        ranges_180 = []
        # 根據 LIDAR_PER_SECTOR 篩選取樣
        all_ranges = lidar_msg.ranges
        for i in range(len(all_ranges)):
            if i % LIDAR_PER_SECTOR == 0:
                # 可選擇計算對應角度，這裡僅保留距離數值
                ranges_180.append(all_ranges[i])
        # 根據設定好的索引範圍抽取對應區域的資料
        combined_lidar_data = (
            [ranges_180[i] for i in FRONT_LIDAR_INDICES]
            + [ranges_180[i] for i in LEFT_LIDAR_INDICES]
            + [ranges_180[i] for i in RIGHT_LIDAR_INDICES]
        )
        return combined_lidar_data


def main(args=None):
    rclpy.init(args=args)
    node = DataReceiverServiceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard Interrupt，節點即將關閉")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
