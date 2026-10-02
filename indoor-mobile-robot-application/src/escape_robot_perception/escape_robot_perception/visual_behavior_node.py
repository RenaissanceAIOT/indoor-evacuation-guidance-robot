"""OpenCV color target and floor-line extraction for low-speed demos."""

from __future__ import annotations

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32


class VisualBehaviorNode(Node):
    def __init__(self) -> None:
        super().__init__("visual_behavior")
        self.declare_parameter("image_topic", "/camera/color/image_raw")
        self.declare_parameter("hsv_lower", [0, 120, 80])
        self.declare_parameter("hsv_upper", [12, 255, 255])
        self.declare_parameter("line_threshold", 70)
        self._bridge = CvBridge()
        self._target_pub = self.create_publisher(PointStamped, "perception/color_target", 10)
        self._line_pub = self.create_publisher(Float32, "perception/line_offset", 10)
        self.create_subscription(
            Image, str(self.get_parameter("image_topic").value), self._on_image, 10
        )

    def _on_image(self, msg: Image) -> None:
        image = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        height, width = image.shape[:2]
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        lower = np.array(self.get_parameter("hsv_lower").value, dtype=np.uint8)
        upper = np.array(self.get_parameter("hsv_upper").value, dtype=np.uint8)
        mask = cv2.inRange(hsv, lower, upper)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            contour = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(contour)
            if area > 150.0:
                moments = cv2.moments(contour)
                point = PointStamped()
                point.header = msg.header
                point.point.x = moments["m10"] / moments["m00"]
                point.point.y = moments["m01"] / moments["m00"]
                point.point.z = area / float(width * height)
                self._target_pub.publish(point)

        roi = image[int(height * 0.65) :, :]
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        threshold = int(self.get_parameter("line_threshold").value)
        _, binary = cv2.threshold(gray, threshold, 255, cv2.THRESH_BINARY_INV)
        line_moments = cv2.moments(binary)
        if line_moments["m00"] > 0.0:
            line_x = line_moments["m10"] / line_moments["m00"]
            normalized_offset = (line_x - width * 0.5) / (width * 0.5)
            self._line_pub.publish(Float32(data=float(normalized_offset)))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VisualBehaviorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

