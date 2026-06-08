import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image, CameraInfo
from std_msgs.msg import Float32MultiArray
from visualization_msgs.msg import Marker
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO
import os
from ament_index_python.packages import get_package_share_directory
import torch


class YoloDetectionNode(Node):
    def __init__(self):
        super().__init__("yolo_detection_node")

        # 初始化 cv_bridge
        self.bridge = CvBridge()

        self.latest_depth_image_raw = None
        self.latest_depth_image_compressed = None

        # 使用 yolo model 位置
        model_path = os.path.join(
            get_package_share_directory("yolo_example_pkg"), "models", "detection.pt"
        )

        device = os.environ.get(
            "YOLO_DEVICE", "cuda" if torch.cuda.is_available() else "cpu"
        )
        print("Using device : ", device)
        self.model = YOLO(model_path)
        self.model.to(device)

        # 訂閱影像 Topic
        self.image_sub = self.create_subscription(
            CompressedImage, "/camera/image/compressed", self.image_callback, 1
        )

        # 訂閱 **無壓縮** 深度圖 Topic
        self.depth_sub_raw = self.create_subscription(
            Image, "/camera/depth/image_raw", self.depth_callback_raw, 1
        )

        # 訂閱 **壓縮** 深度圖 Topic
        self.depth_sub_compressed = self.create_subscription(
            CompressedImage,
            "/camera/depth/compressed",
            self.depth_callback_compressed,
            1,
        )

        # 發佈處理後的影像 Topic
        self.image_pub = self.create_publisher(
            CompressedImage, "/yolo/detection/compressed", 10
        )

        # 發布 目標檢測數據 (是否找到目標 + 距離)
        self.target_pub = self.create_publisher(
            Float32MultiArray, "/yolo/target_info", 10
        )

        self.x_multi_depth_pub = self.create_publisher(
            Float32MultiArray, "/camera/x_multi_depth_values", 10
        )

        # ===== Final Project Task 1: 自動抓取目標點 =====
        # 訂閱相機內參 (K 矩陣)，用來把像素 + 深度反投影成 3D 座標
        self.camera_info = None
        # Unity 實際發布的是 /camera/color/camera_info (RGB 內參)，
        # /camera/image/camera_info 沒有發布者。
        self.camera_info_sub = self.create_subscription(
            CameraInfo, "/camera/color/camera_info", self.camera_info_callback, 1
        )
        # 發布 bear 的 3D 位置 Marker，取代 Foxglove 手動點擊 /clicked_point。
        # arm_controller_2D 訂閱 /yolo/target_marker 後即可自動夾取。
        self.target_marker_pub = self.create_publisher(
            Marker, "/yolo/target_marker", 10
        )
        # 反投影出來的 3D 點所在的光學座標系 (Z 軸朝前)，須存在於 TF tree。
        self.optical_frame = "camera_optical_frame"

        # 設定要過濾標籤 (如果為空，那就不過濾)。
        # 目標類別由環境變數 YOLO_TARGET 決定 (預設 bear)：
        #   Task 1 / Task 2 → YOLO_TARGET=bear (預設)
        #   Task 3          → YOLO_TARGET=knob  (門把，detection.pt 已訓練的類別)
        # 同一支節點 + 同一個 /yolo/target_info 介面，任務端不需改動。
        target = os.environ.get("YOLO_TARGET", "bear")
        print(f"[yolo_node] YOLO_TARGET = {target}")
        self.allowed_labels = {target}
        # 只為這些標籤發布抓取用的 3D Marker。
        self.grasp_labels = {target}

        # 設定 YOLO 可信度閾值
        self.conf_threshold = 0.5  # 可以修改這個值來調整可信度

        # 相機畫面中央高度上切成 n 個等距水平點。
        self.x_num_splits = 20

    def depth_callback_raw(self, msg):
        """接收 **無壓縮** 深度圖"""
        try:
            self.latest_depth_image_raw = self.bridge.imgmsg_to_cv2(
                msg, desired_encoding="passthrough"
            )
        except Exception as e:
            self.get_logger().error(f"Could not convert raw depth image: {e}")

    def depth_callback_compressed(self, msg):
        """接收 **壓縮** 深度圖（當無壓縮深度圖不可用時使用）"""
        try:
            # 自行強制使用 cv2.IMREAD_UNCHANGED 解碼，避開 cv_bridge 的潛在雷區
            np_arr = np.frombuffer(msg.data, np.uint8)
            depth_img = cv2.imdecode(np_arr, cv2.IMREAD_UNCHANGED)
            if depth_img is not None:
                self.latest_depth_image_compressed = depth_img
        except Exception as e:
            self.get_logger().error(f"Could not convert compressed depth image: {e}")

    def image_callback(self, msg):
        """接收影像並進行物體檢測"""
        # 將 ROS 影像消息轉換為 OpenCV 格式
        try:
            cv_image = self.bridge.compressed_imgmsg_to_cv2(
                msg, desired_encoding="bgr8"
            )
        except Exception as e:
            self.get_logger().error(f"Could not convert image: {e}")
            return

        # 使用 YOLO 模型檢測物體
        try:
            results = self.model(cv_image, conf=self.conf_threshold, verbose=False)
        except Exception as e:
            self.get_logger().error(f"Error during YOLO detection: {e}")
            return

        # 繪製 Bounding Box
        processed_image = self.draw_bounding_boxes(cv_image, results)

        # 取得影像中心深度並發布
        self.publish_x_multi_depths(processed_image)

        # 發佈處理後的影像
        self.publish_image(processed_image)

    def draw_cross(self, image):
        # 回傳繪製十字架的影像和畫面正中間的像素座標
        height, width = image.shape[:2]
        cx_center = width // 2
        cy_center = height // 2
        # 繪製橫線
        cv2.line(image, (0, cy_center), (width, cy_center), (0, 0, 255), 2)

        # 繪製直線
        cv2.line(
            image,
            (cx_center, cy_center - 10),
            (cx_center, cy_center + 10),
            (0, 0, 255),
            2,
        )

        cv2.line(
            image,
            (cx_center, cy_center - 10),
            (cx_center, cy_center + 10),
            (0, 0, 255),
            2,
        )

        # 計算橫線上的 n 個等分點
        segment_length = width // self.x_num_splits
        points = [
            (i * segment_length, cy_center) for i in range(self.x_num_splits + 1)
        ]  # 11 個點表示 10 段區間的端點

        # 在每個等分點繪製垂直的短黑線
        for x, y in points:
            cv2.line(image, (x, y - 10), (x, y + 10), (0, 0, 0), 2)  # 黑色垂直線

        return image, points

    def draw_bounding_boxes(self, image, results):
        """在影像上繪製 YOLO 檢測到的 Bounding Box。

        場景中可能有多隻 bear，為避免目標在多隻之間跳動造成車身左右擺動，
        這裡只鎖定「面積最大 (最近) 」的那一隻作為單一目標。
        """
        image, points = self.draw_cross(image)
        center_x = points[self.x_num_splits // 2][0]

        # 先畫出所有符合標籤的框，同時挑出面積最大的目標
        best = None  # (area, cx, cy, depth, class_name)
        for result in results:
            for box in result.boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                conf = float(box.conf)
                class_id = int(box.cls[0])
                class_name = self.model.names[class_id]

                if self.allowed_labels and class_name not in self.allowed_labels:
                    continue

                cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
                area = (x2 - x1) * (y2 - y1)
                depth_value = self.get_depth_at(cx, cy)

                # 畫框 (非選中目標用灰色，選中後再以綠色覆蓋)
                cv2.rectangle(image, (x1, y1), (x2, y2), (160, 160, 160), 1)

                if best is None or area > best[0]:
                    best = (area, cx, cy, depth_value, class_name, (x1, y1, x2, y2))

        H, W = image.shape[:2]
        if best is None:
            # 沒找到任何目標
            self.publish_target_info(0, 0.0, 0.0, 0.0, 0.0)
            return image

        _, cx, cy, depth_value, class_name, (x1, y1, x2, y2) = best
        delta_x = cx - center_x
        depth_text = f"{depth_value:.2f}m" if depth_value else "N/A"
        # 目標框佔畫面比例 & 框底部的垂直位置比例 (越接近 1 代表 bear 越靠近車前)
        area_frac = float((x2 - x1) * (y2 - y1)) / float(W * H)
        bottom_frac = float(y2) / float(H)

        # 突顯選中的目標 (綠色粗框)
        cv2.rectangle(image, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.putText(
            image,
            f"{class_name} D:{depth_text} dx:{delta_x}",
            (x1, y1 - 10),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 255, 0),
            2,
        )

        # 發布抓取用 3D Marker (僅在深度有效時)
        if class_name in self.grasp_labels and depth_value > 0.0:
            self.publish_target_marker(cx, cy, depth_value)

        self.publish_target_info(1, depth_value, float(delta_x), area_frac, bottom_frac)
        return image

    def get_depth_at(self, x, y):
        """
        取得指定像素的深度值，轉換為米 (m)
        若深度出問題，回傳 -1
        """
        # **優先使用無壓縮的深度圖**
        depth_image = (
            self.latest_depth_image_raw
            if self.latest_depth_image_raw is not None
            else self.latest_depth_image_compressed
        )

        if depth_image is None:
            return -1.0

        # 如果深度影像為三通道，那只取第一個數值
        if len(depth_image.shape) == 3:
            depth_image = depth_image[:, :, 0]

        try:
            depth_value = depth_image[y, x]
            if depth_value < 0.0001 or depth_value == 0.0:  # 無效深度
                return -1.0
            return depth_value / 1000.0  # 16-bit 深度圖通常單位為 mm，轉換為 m
        except IndexError:
            return -1.0

    def camera_info_callback(self, msg):
        """快取相機內參 (K 矩陣)，供反投影使用。"""
        self.camera_info = msg

    def publish_target_marker(self, u, v, depth):
        """
        把像素 (u, v) + 深度 depth(m) 反投影成相機光學座標系下的 3D 點，
        並以 Marker 發布到 /yolo/target_marker，讓手臂自動夾取。

        光學座標系慣例 (Z 朝前)：
            X = (u - cx) * Z / fx   (右)
            Y = (v - cy) * Z / fy   (下)
            Z = depth               (前)
        """
        if self.camera_info is None:
            # 還沒收到內參，無法反投影
            return

        k = self.camera_info.k  # 3x3 row-major: [fx,0,cx, 0,fy,cy, 0,0,1]
        fx, fy = k[0], k[4]
        cx, cy = k[2], k[5]
        if fx == 0.0 or fy == 0.0:
            return

        z = float(depth)
        x = (float(u) - cx) * z / fx
        y = (float(v) - cy) * z / fy

        marker = Marker()
        marker.header.frame_id = self.optical_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "yolo_target"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = z
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.2
        marker.color.b = 0.2
        self.target_marker_pub.publish(marker)

    def publish_image(self, image):
        """將處理後的影像轉換並發佈到 ROS"""
        try:
            compressed_msg = self.bridge.cv2_to_compressed_imgmsg(image)
            self.image_pub.publish(compressed_msg)
        except Exception as e:
            self.get_logger().error(f"Could not publish image: {e}")

    def publish_target_info(self, found, distance, delta_x, area_frac=0.0, bottom_frac=0.0):
        """發佈目標資訊。

        data = [found, distance(m), delta_x(px),
                area_frac(框佔畫面比例), bottom_frac(框底部 y 比例)]
        後兩項供近距離夾取判斷 (深度 <0.45m 失效時用視覺大小/位置代替)。
        """
        msg = Float32MultiArray()
        msg.data = [
            float(found),
            float(distance),
            float(delta_x),
            float(area_frac),
            float(bottom_frac),
        ]
        self.target_pub.publish(msg)

    def publish_x_multi_depths(self, image):
        """
        取得畫面 n 個等分點的深度並發布
        """
        height, width = image.shape[:2]
        cy_center = height // 2  # 固定 Y 座標在畫面中心
        segment_length = width // self.x_num_splits

        # 計算 10 個等分點的 X 座標
        points = [(i * segment_length, cy_center) for i in range(self.x_num_splits)]

        # 取得每個等分點的深度值
        depth_values = [self.get_depth_at(x, cy_center) for x, _ in points]

        # 以 Float32MultiArray 發布
        depth_msg = Float32MultiArray()
        depth_msg.data = depth_values
        self.x_multi_depth_pub.publish(depth_msg)


def main(args=None):
    rclpy.init(args=args)
    node = YoloDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
