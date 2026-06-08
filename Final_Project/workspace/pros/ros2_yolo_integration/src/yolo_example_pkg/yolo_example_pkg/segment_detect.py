import os

import cv2
import numpy as np
import rclpy
import torch
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float32MultiArray
from ultralytics import YOLO


# Per-class colors (BGR) for mask overlays.
CLASS_COLORS = {
    "bridge": (0, 165, 255),  # orange
    "road": (0, 255, 0),      # green
}
DEFAULT_COLOR = (255, 0, 255)


class YoloSegmentationNode(Node):
    def __init__(self):
        super().__init__("yolo_segmentation_node")
        self.bridge = CvBridge()

        model_path = os.path.join(
            get_package_share_directory("yolo_example_pkg"),
            "models",
            "segmentation.pt",
        )
        device = os.environ.get(
            "YOLO_DEVICE", "cuda" if torch.cuda.is_available() else "cpu"
        )
        self.get_logger().info(f"Loading segmentation model on {device}: {model_path}")
        self.model = YOLO(model_path)
        self.model.to(device)

        self.allowed_labels = {"bridge", "road"}
        self.conf_threshold = 0.5
        self.mask_alpha = 0.45

        self.image_sub = self.create_subscription(
            CompressedImage, "/camera/image/compressed", self.image_callback, 1
        )
        self.image_pub = self.create_publisher(
            CompressedImage, "/yolo/segmentation/compressed", 10
        )
        # Task 2 橋面導引訊號 (見 publish_nav_info 的 data 說明)。
        self.bridge_info_pub = self.create_publisher(
            Float32MultiArray, "/yolo/bridge_info", 10
        )
        # Task 2 道路置中訊號 (沿路前進；橋通常在路的盡頭/路上)。
        self.road_info_pub = self.create_publisher(
            Float32MultiArray, "/yolo/road_info", 10
        )

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            self.get_logger().error(f"Could not convert image: {exc}")
            return

        try:
            results = self.model(cv_image, conf=self.conf_threshold, verbose=False)
        except Exception as exc:
            self.get_logger().error(f"YOLO segmentation error: {exc}")
            return

        overlay = self.draw_masks(cv_image, results)
        self.publish_image(overlay)
        self.publish_nav_info(cv_image, results)

    def _class_union_mask(self, results, class_name, h, w):
        """把某類別的所有遮罩聯集成一張 (h,w) 0/1 圖。"""
        union = np.zeros((h, w), dtype=np.uint8)
        for result in results:
            if result.masks is None:
                continue
            mask_tensor = result.masks.data  # (n, H, W) in [0,1]
            boxes = result.boxes
            for i in range(mask_tensor.shape[0]):
                if self.model.names[int(boxes.cls[i])] != class_name:
                    continue
                mask = mask_tensor[i].detach().cpu().numpy()
                if mask.shape != (h, w):
                    mask = cv2.resize(mask, (w, h), interpolation=cv2.INTER_NEAREST)
                union[mask > 0.5] = 1
        return union

    def publish_nav_info(self, image, results):
        """從 'bridge' / 'road' 遮罩算出 Task 2 的導引訊號並發布。

        /yolo/bridge_info = [found, delta_x, area_frac, centroid_y_frac,
                             bottom_edge_dx, symmetry, aspect_ratio]
          found          : 1 有看到橋面、0 沒有
          delta_x        : 橋面質心 x - 畫面中心 (>0 橋在右、<0 橋在左) → 轉向偏置用
          area_frac      : 橋面像素 / 全畫面 (越大越近；夠大就切換到「上橋」)
          centroid_y_frac: 質心 y / 畫面高
          bottom_edge_dx : 遮罩最底 15% 列的平均 x - 畫面中心 → 橋「入口/坡腳」的水平位置
          symmetry       : (右半面積 - 左半面積)/總面積 → 正面對準時 ~0 (擺正用)
          aspect_ratio   : 遮罩邊界框 h/w → 區分「正對坡腳」(瘦高，坡延伸進畫面深處 → ~>=0.7)
                           vs「貼到橋側牆」(寬扁，側牆橫在畫面中央 → ~<=0.4)。
                           沒看到橋(found=0) 時為 0.0。
        /yolo/road_info   = [found, delta_x, area_frac]  (道路質心置中，沿路前進用)
        """
        h, w = image.shape[:2]

        # ---- bridge ----
        bridge_mask = self._class_union_mask(results, "bridge", h, w)
        bmsg = Float32MultiArray()
        bpix = int(bridge_mask.sum())
        if bpix == 0:
            bmsg.data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        else:
            ys, xs = np.nonzero(bridge_mask)
            cx = float(xs.mean())
            delta_x = cx - (w / 2.0)
            area_frac = bpix / float(h * w)
            centroid_y_frac = float(ys.mean()) / float(h)
            # 最底 15% 列 = 離車最近的橋面 (坡腳)；其平均 x 是入口方位。
            y_lo = int(ys.max() - 0.15 * (ys.max() - ys.min() + 1))
            bottom_xs = xs[ys >= y_lo]
            bottom_edge_dx = (float(bottom_xs.mean()) - w / 2.0) if bottom_xs.size else delta_x
            left_area = int((xs < w / 2.0).sum())
            right_area = bpix - left_area
            symmetry = (right_area - left_area) / float(bpix)
            # 邊界框長寬比 h/w：用以排除「貼到橋側牆」的誤上橋
            # (側牆遮罩寬扁，aspect~0.3；正對坡腳遮罩瘦高，aspect~>=0.7)。
            bbox_h = float(ys.max() - ys.min() + 1)
            bbox_w = float(xs.max() - xs.min() + 1)
            aspect_ratio = bbox_h / bbox_w if bbox_w > 0 else 0.0
            bmsg.data = [1.0, float(delta_x), float(area_frac), float(centroid_y_frac),
                         float(bottom_edge_dx), float(symmetry), float(aspect_ratio)]
        self.bridge_info_pub.publish(bmsg)

        # ---- road ----
        road_mask = self._class_union_mask(results, "road", h, w)
        rmsg = Float32MultiArray()
        rpix = int(road_mask.sum())
        if rpix == 0:
            rmsg.data = [0.0, 0.0, 0.0]
        else:
            _, rxs = np.nonzero(road_mask)
            rmsg.data = [1.0, float(rxs.mean() - w / 2.0), rpix / float(h * w)]
        self.road_info_pub.publish(rmsg)

    def draw_masks(self, image, results):
        h, w = image.shape[:2]
        overlay = image.copy()

        for result in results:
            if result.masks is None:
                continue
            mask_tensor = result.masks.data  # (n, H, W) in [0,1]
            boxes = result.boxes
            for i in range(mask_tensor.shape[0]):
                class_id = int(boxes.cls[i])
                class_name = self.model.names[class_id]
                if self.allowed_labels and class_name not in self.allowed_labels:
                    continue
                conf = float(boxes.conf[i])
                color = CLASS_COLORS.get(class_name, DEFAULT_COLOR)

                mask = mask_tensor[i].detach().cpu().numpy()
                if mask.shape != (h, w):
                    mask = cv2.resize(mask, (w, h), interpolation=cv2.INTER_NEAREST)
                binary = (mask > 0.5).astype(np.uint8)

                tint = np.zeros_like(image)
                tint[binary == 1] = color
                overlay = cv2.addWeighted(overlay, 1.0, tint, self.mask_alpha, 0)

                contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                cv2.drawContours(overlay, contours, -1, color, 2)

                x1, y1, _, _ = map(int, boxes.xyxy[i])
                cv2.putText(
                    overlay,
                    f"{class_name} {conf:.2f}",
                    (x1, max(y1 - 5, 15)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    color,
                    2,
                )

        return overlay

    def publish_image(self, image):
        try:
            self.image_pub.publish(self.bridge.cv2_to_compressed_imgmsg(image))
        except Exception as exc:
            self.get_logger().error(f"Could not publish image: {exc}")


def main(args=None):
    rclpy.init(args=args)
    node = YoloSegmentationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
