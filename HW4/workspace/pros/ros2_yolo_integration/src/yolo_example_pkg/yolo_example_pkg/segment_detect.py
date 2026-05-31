import os

import cv2
import numpy as np
import rclpy
import torch
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
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
