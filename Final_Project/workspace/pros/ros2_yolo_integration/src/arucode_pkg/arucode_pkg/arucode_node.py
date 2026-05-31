import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Float32
import cv2
import cv2.aruco as aruco
import numpy as np


class ArucoDetector(Node):
    def __init__(self):
        super().__init__("aruco_detector")

        # 訂閱彩色與深度
        self.image_sub = self.create_subscription(
            CompressedImage, "/out/compressed", self.image_callback, 10
        )
        self.depth_sub = self.create_subscription(
            Image, "/camera/depth/image_raw", self.depth_callback, 10
        )

        # 發佈：ID=100 的深度（公尺）
        self.depth_pub = self.create_publisher(Float32, "/aruco/id100/depth_m", 10)

        # ---- ArUco 相容初始化（新版/舊版都可） ----
        # 字典
        try:
            self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
        except AttributeError:
            # 舊版 OpenCV
            self.aruco_dict = aruco.Dictionary_get(aruco.DICT_ARUCO_ORIGINAL)

        # 參數
        try:
            self.aruco_params = cv2.aruco.DetectorParameters()  # 新版
        except AttributeError:
            self.aruco_params = aruco.DetectorParameters_create()  # 舊版

        # 檢測器（新版有 ArucoDetector 類；舊版用函式）
        self._aruco_detector = None
        if hasattr(cv2.aruco, "ArucoDetector"):
            self._aruco_detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)

        # 深度快取
        self._depth_img = None
        self._depth_encoding = None
        self._depth_h = None
        self._depth_w = None

        # 目標 ID
        self.target_id = 100

    # --- 深度影像回呼：只做轉換與快取 ---
    def depth_callback(self, msg: Image):
        try:
            self._depth_h = msg.height
            self._depth_w = msg.width
            self._depth_encoding = msg.encoding  # e.g. '16UC1' or '32FC1'

            if msg.encoding == "16UC1":
                depth = np.frombuffer(msg.data, dtype=np.uint16).reshape(self._depth_h, self._depth_w)
                self._depth_img = depth.astype(np.float32) / 1000.0  # 轉公尺
            elif msg.encoding == "32FC1":
                self._depth_img = np.frombuffer(msg.data, dtype=np.float32).reshape(self._depth_h, self._depth_w)
            else:
                # 其他編碼：嘗試 float32（單位不保證）
                self._depth_img = np.frombuffer(msg.data, dtype=np.float32).reshape(self._depth_h, self._depth_w)
                if self._depth_img is not None:
                    self.get_logger().warn(
                        f"[depth] 未知編碼 {msg.encoding}，已嘗試以 float32 解析（單位可能不為公尺）"
                    )
        except Exception as e:
            self._depth_img = None
            self.get_logger().error(f"[depth] 解析失敗: {e}")

    # --- 彩色影像回呼：ArUco 偵測並在遇到 ID=100 時輸出 + 發佈深度 ---
    def image_callback(self, msg: CompressedImage):
        # 轉 OpenCV BGR 影像
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        except Exception as e:
            self.get_logger().error(f"[rgb] 影像解碼失敗: {e}")
            return
        if cv_image is None:
            return

        # ArUco 偵測（新版優先）
        try:
            if self._aruco_detector is not None:
                corners, ids, _ = self._aruco_detector.detectMarkers(cv_image)
            else:
                # 舊版 API
                corners, ids, _ = aruco.detectMarkers(
                    cv_image, self.aruco_dict, parameters=self.aruco_params
                )
        except Exception as e:
            self.get_logger().error(f"[aruco] 偵測失敗: {e}")
            return

        if ids is None or len(ids) == 0:
            return

        ids_list = ids.flatten().tolist()
        if self.target_id not in ids_list:
            return

        # 找到 ID=100 的 corners，取中心深度並 publish
        for i, marker_id in enumerate(ids_list):
            if marker_id != self.target_id:
                continue
            c = corners[i][0]  # shape: (4, 2) -> (x,y)
            u = int(round(c[:, 0].mean()))
            v = int(round(c[:, 1].mean()))

            depth_m = self._get_depth_at_pixel(u, v, cv_image.shape[1], cv_image.shape[0])
            if depth_m is None:
                self.get_logger().warn(f"[aruco {self.target_id}] 深度不可用或座標越界 (u={u}, v={v})")
            else:
                # 發佈 Float32（m）
                msg_depth = Float32()
                msg_depth.data = depth_m
                self.depth_pub.publish(msg_depth)

                self.get_logger().info(
                    f"[aruco {self.target_id}] 深度: {depth_m:.3f} m at (u={u}, v={v})  -> 已發佈 /aruco/id100/depth_m"
                )

    def _get_depth_at_pixel(self, u_rgb: int, v_rgb: int, rgb_w: int, rgb_h: int):
        """
        從最新的深度影像取得 (u,v) 的深度（公尺）。
        若彩色與深度大小不同，做尺度映射；再在 3x3 視窗取有效值平均。
        """
        if self._depth_img is None:
            return None
        if rgb_w <= 0 or rgb_h <= 0 or self._depth_w is None or self._depth_h is None:
            return None

        # RGB → Depth 最近鄰尺度映射
        u_d = int(round(u_rgb * (self._depth_w / float(rgb_w))))
        v_d = int(round(v_rgb * (self._depth_h / float(rgb_h))))

        if u_d < 0 or v_d < 0 or u_d >= self._depth_w or v_d >= self._depth_h:
            return None

        # 3x3 視窗平均（忽略 0 與 NaN）
        x0 = max(0, u_d - 1)
        y0 = max(0, v_d - 1)
        x1 = min(self._depth_w - 1, u_d + 1)
        y1 = min(self._depth_h - 1, v_d + 1)

        patch = self._depth_img[y0 : y1 + 1, x0 : x1 + 1]
        if patch.size == 0:
            return None

        valid = patch[np.isfinite(patch)]
        if valid.size == 0:
            return None
        valid = valid[valid > 0.0]
        if valid.size == 0:
            return None

        return float(valid.mean())


def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
