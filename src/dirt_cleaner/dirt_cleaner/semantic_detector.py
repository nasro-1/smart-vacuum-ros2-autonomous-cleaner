#!/usr/bin/env python3
"""Semantic surface detector node.

Uses HSV colour segmentation on the bottom 40% of the camera frame
(floor region closest to the robot) to classify the dominant surface
type: water, dirt, carpet, or clear.

Publishes:
  /semantic_class  (std_msgs/String)  — dominant class name
  /semantic_area   (std_msgs/Float32) — pixel count of dominant class
  /semantic_debug  (sensor_msgs/Image) — annotated debug frame
"""

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, String

# ---------------------------------------------------------------------------
# HSV class definitions  (tune these if scene lighting changes)
# ---------------------------------------------------------------------------
SURFACE_CLASSES = {
    "water": {
        "lower": np.array([85,  40,  40]),
        "upper": np.array([140, 255, 255]),
        "color": (255, 80,  0),    # BGR debug overlay color
        "min_area": 500,
    },
    "dirt": {
        "lower": np.array([20,  100, 100]),
        "upper": np.array([35,  255, 255]),
        "color": (0,   200, 255),
        "min_area": 400,
    },
    "carpet": {
        "lower": np.array([8,   60,  40]),
        "upper": np.array([22,  200, 160]),
        "color": (0,   165, 255),
        "min_area": 400,
    },
}

# Detection priority: first match wins
CLASS_PRIORITY = ["water", "dirt", "carpet"]

# Fraction of frame height used as floor ROI (bottom portion)
ROI_START_FRAC = 0.60


class SemanticDetector(Node):
    def __init__(self):
        super().__init__("semantic_detector")
        self.bridge = CvBridge()
        self.last_detected = "clear"

        self.create_subscription(Image, "/camera/image_raw", self._image_cb, 10)
        self.class_pub = self.create_publisher(String,  "/semantic_class", 10)
        self.area_pub  = self.create_publisher(Float32, "/semantic_area",  10)
        self.debug_pub = self.create_publisher(Image,   "/semantic_debug", 10)

        self.get_logger().info(
            f"SemanticDetector ready — classes: {CLASS_PRIORITY}"
        )

    def _image_cb(self, msg: Image):
        frame   = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        hsv     = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        roi_y   = int(frame.shape[0] * ROI_START_FRAC)
        roi_hsv = hsv[roi_y:, :]
        roi_dbg = frame[roi_y:, :].copy()

        detected      = "clear"
        detected_area = 0.0

        for cls in CLASS_PRIORITY:
            cfg   = SURFACE_CLASSES[cls]
            mask  = cv2.inRange(roi_hsv, cfg["lower"], cfg["upper"])
            count = cv2.countNonZero(mask)

            if count >= cfg["min_area"]:
                detected      = cls
                detected_area = float(count)

                contours, _ = cv2.findContours(
                    mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
                )
                cv2.drawContours(roi_dbg, contours, -1, cfg["color"], 2)
                cv2.putText(
                    roi_dbg,
                    f"{cls.upper()}  px={count}",
                    (10, 28), cv2.FONT_HERSHEY_SIMPLEX,
                    0.7, cfg["color"], 2,
                )
                break   # highest-priority class wins

        # Publish results
        class_msg      = String()
        class_msg.data = detected
        self.class_pub.publish(class_msg)

        area_msg      = Float32()
        area_msg.data = detected_area
        self.area_pub.publish(area_msg)

        if detected != self.last_detected:
            self.get_logger().info(
                f"Surface: {self.last_detected} → {detected}  (area={detected_area:.0f} px)"
            )
            self.last_detected = detected

        # Publish debug frame
        frame[roi_y:, :] = roi_dbg
        self.debug_pub.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))


def main(args=None):
    rclpy.init(args=args)
    node = SemanticDetector()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()