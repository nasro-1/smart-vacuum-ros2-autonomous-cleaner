#!/usr/bin/env python3
"""YOLO-based dirt/trash detector node.

Subscribes to a camera topic, runs inference, and publishes the
normalized horizontal offset, confidence, and area ratio of the
best detection to /dirt_target (geometry_msgs/Point).
"""

import os
from typing import Optional

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from sensor_msgs.msg import Image

try:
    from ultralytics import YOLO
except ImportError:
    YOLO = None


class DirtDetectorNode(Node):
    def __init__(self):
        super().__init__("dirt_detector")

        # ROS parameters — override via launch file or command line
        self.declare_parameter("weights",           "")          # required: path to .pt file
        self.declare_parameter("camera_topic",      "/camera/image_raw")
        self.declare_parameter("conf_threshold",    0.4)
        self.declare_parameter("target_class",      "trash")
        self.declare_parameter("debug_image_topic", "/camera/dirt_debug")

        self.weights         = self.get_parameter("weights").value
        self.camera_topic    = self.get_parameter("camera_topic").value
        self.conf_threshold  = float(self.get_parameter("conf_threshold").value)
        self.target_class    = str(self.get_parameter("target_class").value)
        debug_topic          = self.get_parameter("debug_image_topic").value

        # If no weights path given, try common relative locations
        if not self.weights:
            self.weights = self._find_weights()

        self.target_pub = self.create_publisher(Point, "/dirt_target", 10)
        self.debug_pub  = self.create_publisher(Image, debug_topic, 10)
        self.create_subscription(Image, self.camera_topic, self._image_cb, 10)

        self.model: Optional[object] = None
        self.target_class_id: Optional[int] = None
        self.model_ready = self._load_model()

    # ------------------------------------------------------------------
    # Model loading
    # ------------------------------------------------------------------
    def _find_weights(self) -> str:
        """Search common relative paths for the weights file."""
        candidates = [
            os.path.join(os.environ.get("HOME", ""), "models", "dirt_yolo.pt"),
            os.path.join(os.path.dirname(__file__), "..", "models", "dirt_yolo.pt"),
            "models/dirt_yolo.pt",
        ]
        for path in candidates:
            if os.path.exists(path):
                return path
        return candidates[0]   # will be reported as missing in _load_model

    def _load_model(self) -> bool:
        if YOLO is None:
            self.get_logger().error(
                "ultralytics is not installed. Run: pip install ultralytics"
            )
            return False

        if not os.path.exists(self.weights):
            self.get_logger().warn(
                f"YOLO weights not found at '{self.weights}'. "
                "Pass the correct path via the 'weights' ROS parameter."
            )
            return False

        self.model = YOLO(self.weights)
        names = self.model.names
        for class_id, class_name in names.items():
            if str(class_name).lower() == self.target_class.lower():
                self.target_class_id = int(class_id)
                break

        if self.target_class_id is None:
            self.get_logger().warn(
                f"Class '{self.target_class}' not in model classes: {list(names.values())}"
            )
            return False

        self.get_logger().info(
            f"YOLO ready — weights={self.weights}, "
            f"class='{self.target_class}' (id={self.target_class_id})"
        )
        return True

    # ------------------------------------------------------------------
    # Image conversion helpers
    # ------------------------------------------------------------------
    @staticmethod
    def _ros_to_bgr(msg: Image) -> Optional[np.ndarray]:
        if msg.encoding not in ("rgb8", "bgr8"):
            return None
        arr = np.frombuffer(msg.data, dtype=np.uint8)
        try:
            arr = arr.reshape((msg.height, msg.width, 3))
        except ValueError:
            return None
        return cv2.cvtColor(arr, cv2.COLOR_RGB2BGR) if msg.encoding == "rgb8" else arr

    @staticmethod
    def _bgr_to_ros(frame: np.ndarray, stamp) -> Image:
        msg = Image()
        msg.header.stamp    = stamp
        msg.header.frame_id = "camera"
        msg.height, msg.width = frame.shape[:2]
        msg.encoding    = "bgr8"
        msg.is_bigendian = 0
        msg.step         = msg.width * 3
        msg.data         = frame.tobytes()
        return msg

    # ------------------------------------------------------------------
    # Main callback
    # ------------------------------------------------------------------
    def _image_cb(self, msg: Image):
        # Retry model load if weights appeared after node start
        if not self.model_ready:
            if os.path.exists(self.weights):
                self.model_ready = self._load_model()
            if not self.model_ready:
                return

        frame = self._ros_to_bgr(msg)
        if frame is None:
            self.get_logger().warn(f"Unsupported encoding: {msg.encoding}")
            return

        results = self.model(frame, verbose=False)
        if not results or results[0].boxes is None:
            return

        img_h, img_w = frame.shape[:2]
        image_area   = float(img_h * img_w)
        best, best_area = None, -1.0

        for box in results[0].boxes:
            if int(box.cls.item()) != self.target_class_id:
                continue
            conf = float(box.conf.item())
            if conf < self.conf_threshold:
                continue
            x1, y1, x2, y2 = box.xyxy[0].tolist()
            area = max(0.0, (x2 - x1) * (y2 - y1))
            if area > best_area:
                best_area, best = area, (x1, y1, x2, y2, conf)

        if best is None:
            return

        x1, y1, x2, y2, conf = best
        cx         = (x1 + x2) / 2.0
        offset     = ((cx / img_w) - 0.5) * 2.0
        area_ratio = best_area / image_area if image_area > 0 else 0.0

        pt = Point()
        pt.x, pt.y, pt.z = float(offset), float(conf), float(area_ratio)
        self.target_pub.publish(pt)

        # Debug overlay
        cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (30, 220, 30), 2)
        cv2.putText(
            frame,
            f"{self.target_class} {conf:.2f}  off={offset:+.2f}  area={area_ratio:.3f}",
            (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (30, 220, 30), 2, cv2.LINE_AA,
        )
        self.debug_pub.publish(self._bgr_to_ros(frame, msg.header.stamp))


def main():
    rclpy.init()
    node = DirtDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()