"""Optional MediaPipe hand-gesture adapter.

Publishes a deliberately small command vocabulary: ``OPEN_PALM``, ``LEFT``,
``RIGHT`` and ``NONE``. This is a human-machine interaction demo, not a safety
input for the evacuation planner.
"""

from __future__ import annotations

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String


class GestureNode(Node):
    def __init__(self) -> None:
        super().__init__("gesture")
        self.declare_parameter("image_topic", "/camera/color/image_raw")
        try:
            import mediapipe as mp
        except ImportError as exc:
            raise RuntimeError(
                "mediapipe is optional; install it in the perception environment to run gesture_node"
            ) from exc
        self._bridge = CvBridge()
        self._hands = mp.solutions.hands.Hands(
            static_image_mode=False,
            max_num_hands=1,
            min_detection_confidence=0.6,
            min_tracking_confidence=0.6,
        )
        self._pub = self.create_publisher(String, "perception/gesture", 10)
        self.create_subscription(
            Image, str(self.get_parameter("image_topic").value), self._on_image, 10
        )

    def _on_image(self, msg: Image) -> None:
        bgr = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        result = self._hands.process(cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB))
        gesture = "NONE"
        if result.multi_hand_landmarks:
            landmarks = result.multi_hand_landmarks[0].landmark
            raised = sum(landmarks[tip].y < landmarks[pip].y for tip, pip in [(8, 6), (12, 10), (16, 14), (20, 18)])
            if raised >= 4:
                gesture = "OPEN_PALM"
            elif landmarks[8].x < landmarks[0].x - 0.15:
                gesture = "LEFT"
            elif landmarks[8].x > landmarks[0].x + 0.15:
                gesture = "RIGHT"
        self._pub.publish(String(data=gesture))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = GestureNode()
        rclpy.spin(node)
    except (RuntimeError, KeyboardInterrupt) as exc:
        print(exc)
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
