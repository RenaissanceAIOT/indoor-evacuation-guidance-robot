"""Fuse YOLO detections with aligned depth into camera-frame 3D points."""

from __future__ import annotations

import json
import time

import cv2
import message_filters
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Point, Pose, PoseArray, Quaternion
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String

from escape_robot_perception.depth_geometry import project_pixel


class ObjectDepthNode(Node):
    def __init__(self) -> None:
        super().__init__("object_depth")
        self.declare_parameter("model", "yolov8n.pt")
        self.declare_parameter("confidence", 0.45)
        self.declare_parameter("target_classes", ["person", "bottle", "chair"])
        self.declare_parameter("max_rate_hz", 10.0)
        self.declare_parameter("depth_scale", 0.001)
        self.declare_parameter("rgb_topic", "/camera/color/image_raw")
        self.declare_parameter("depth_topic", "/camera/aligned_depth_to_color/image_raw")
        self.declare_parameter("camera_info_topic", "/camera/color/camera_info")

        try:
            from ultralytics import YOLO
        except ImportError as exc:
            raise RuntimeError(
                "ultralytics is optional and not installed; run "
                "'python3 -m pip install ultralytics' in the perception environment"
            ) from exc

        self._model = YOLO(str(self.get_parameter("model").value))
        self._targets = set(self.get_parameter("target_classes").value)
        self._confidence = float(self.get_parameter("confidence").value)
        self._period = 1.0 / float(self.get_parameter("max_rate_hz").value)
        self._depth_scale = float(self.get_parameter("depth_scale").value)
        self._last_run = 0.0
        self._camera_matrix: tuple[float, float, float, float] | None = None
        self._bridge = CvBridge()

        self._poses_pub = self.create_publisher(PoseArray, "perception/detections_3d", 10)
        self._json_pub = self.create_publisher(String, "perception/detections", 10)
        self._debug_pub = self.create_publisher(Image, "perception/debug_image", 5)
        self.create_subscription(
            CameraInfo,
            str(self.get_parameter("camera_info_topic").value),
            self._on_camera_info,
            qos_profile_sensor_data,
        )
        rgb = message_filters.Subscriber(
            self, Image, str(self.get_parameter("rgb_topic").value),
            qos_profile=qos_profile_sensor_data,
        )
        depth = message_filters.Subscriber(
            self, Image, str(self.get_parameter("depth_topic").value),
            qos_profile=qos_profile_sensor_data,
        )
        self._sync = message_filters.ApproximateTimeSynchronizer(
            [rgb, depth], queue_size=8, slop=0.08
        )
        self._sync.registerCallback(self._on_images)

    def _on_camera_info(self, msg: CameraInfo) -> None:
        if not np.isfinite(msg.k).all() or msg.k[0] <= 0 or msg.k[4] <= 0:
            self._camera_matrix = None
            return
        self._camera_matrix = (msg.k[0], msg.k[4], msg.k[2], msg.k[5])

    def _on_images(self, rgb_msg: Image, depth_msg: Image) -> None:
        if self._camera_matrix is None or time.monotonic() - self._last_run < self._period:
            return
        self._last_run = time.monotonic()
        image = self._bridge.imgmsg_to_cv2(rgb_msg, desired_encoding="bgr8")
        depth = self._bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
        if depth.shape[:2] != image.shape[:2]:
            self.get_logger().error("RGB and aligned depth dimensions differ")
            return
        if depth_msg.encoding not in ("16UC1", "32FC1"):
            self.get_logger().error(f"unsupported depth encoding: {depth_msg.encoding}")
            return
        scale = 1.0 if depth_msg.encoding == "32FC1" else self._depth_scale
        result = self._model.predict(image, conf=self._confidence, verbose=False)[0]
        fx, fy, cx, cy = self._camera_matrix
        pose_array = PoseArray()
        pose_array.header = rgb_msg.header
        records = []

        for box in result.boxes:
            class_id = int(box.cls[0])
            label = str(result.names[class_id])
            if label not in self._targets:
                continue
            x1, y1, x2, y2 = (int(value) for value in box.xyxy[0].tolist())
            u = max(0, min(depth.shape[1] - 1, (x1 + x2) // 2))
            v = max(0, min(depth.shape[0] - 1, (y1 + y2) // 2))
            radius = 3
            patch = depth[
                max(0, v - radius) : min(depth.shape[0], v + radius + 1),
                max(0, u - radius) : min(depth.shape[1], u + radius + 1),
            ].astype(np.float32)
            valid = patch[np.isfinite(patch) & (patch > 0)]
            if valid.size == 0:
                continue
            z = float(np.median(valid)) * scale
            if not 0.15 <= z <= 10.0:
                continue
            x, y, z = project_pixel(u, v, z, fx, fy, cx, cy)
            confidence = float(box.conf[0])
            pose_array.poses.append(
                Pose(position=Point(x=x, y=y, z=z), orientation=Quaternion(w=1.0))
            )
            records.append(
                {
                    "class": label,
                    "confidence": round(confidence, 4),
                    "camera_xyz_m": [round(x, 3), round(y, 3), round(z, 3)],
                    "bbox_xyxy": [x1, y1, x2, y2],
                }
            )
            cv2.rectangle(image, (x1, y1), (x2, y2), (50, 220, 80), 2)
            cv2.putText(
                image,
                f"{label} {confidence:.2f} {z:.2f}m",
                (x1, max(20, y1 - 6)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (50, 220, 80),
                2,
            )

        self._poses_pub.publish(pose_array)
        self._json_pub.publish(String(data=json.dumps(records, ensure_ascii=False)))
        debug = self._bridge.cv2_to_imgmsg(image, encoding="bgr8")
        debug.header = rgb_msg.header
        self._debug_pub.publish(debug)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = ObjectDepthNode()
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
