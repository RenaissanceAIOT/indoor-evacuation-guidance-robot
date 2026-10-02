"""Kinematic mock of the omnidirectional base for laptop-only demonstrations."""

from __future__ import annotations

import math
import time

import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import BatteryState, Imu
from std_msgs.msg import Bool
from tf2_ros import TransformBroadcaster


class MockBase(Node):
    def __init__(self) -> None:
        super().__init__("mock_mobile_base")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("cmd_timeout_sec", 0.35)
        self.declare_parameter("battery_voltage", 12.1)
        self._base_frame = str(self.get_parameter("base_frame").value)
        self._odom_frame = str(self.get_parameter("odom_frame").value)
        self._timeout = float(self.get_parameter("cmd_timeout_sec").value)
        self._command = Twist()
        self._stop = False
        self._last_interlock = None
        self._last_command = time.monotonic()
        self._last_update = time.monotonic()
        self._x = 0.0
        self._y = 0.0
        self._yaw = 0.0
        self._odom_pub = self.create_publisher(Odometry, "odom", 20)
        self._imu_pub = self.create_publisher(Imu, "imu/data_raw", 20)
        self._battery_pub = self.create_publisher(BatteryState, "battery_state", 5)
        self._tf = TransformBroadcaster(self)
        self.create_subscription(Twist, "cmd_vel", self._on_cmd, 20)
        self.create_subscription(Bool, "base/stop", self._on_stop, 10)
        self.create_timer(0.02, self._step)

    def _on_cmd(self, msg: Twist) -> None:
        if not all(math.isfinite(v) for v in (msg.linear.x, msg.linear.y, msg.angular.z)):
            self._command = Twist()
            return
        self._command = msg
        self._last_command = time.monotonic()

    def _on_stop(self, msg: Bool) -> None:
        self._stop = msg.data
        self._last_interlock = time.monotonic()

    def _step(self) -> None:
        now_mono = time.monotonic()
        dt = min(0.1, now_mono - self._last_update)
        self._last_update = now_mono
        timed_out = (now_mono - self._last_command > self._timeout or self._stop
                     or (self._last_interlock is not None
                         and now_mono - self._last_interlock > 1.5))
        vx = 0.0 if timed_out else max(-0.45, min(0.45, self._command.linear.x))
        vy = 0.0 if timed_out else max(-0.35, min(0.35, self._command.linear.y))
        wz = 0.0 if timed_out else max(-1.2, min(1.2, self._command.angular.z))
        self._x += (vx * math.cos(self._yaw) - vy * math.sin(self._yaw)) * dt
        self._y += (vx * math.sin(self._yaw) + vy * math.cos(self._yaw)) * dt
        self._yaw += wz * dt
        half_yaw = self._yaw * 0.5
        stamp = self.get_clock().now().to_msg()

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self._odom_frame
        odom.child_frame_id = self._base_frame
        odom.pose.pose.position.x = self._x
        odom.pose.pose.position.y = self._y
        odom.pose.pose.orientation.z = math.sin(half_yaw)
        odom.pose.pose.orientation.w = math.cos(half_yaw)
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = wz
        self._odom_pub.publish(odom)

        imu = Imu()
        imu.header.stamp = stamp
        imu.header.frame_id = "imu_link"
        imu.orientation_covariance[0] = -1.0
        imu.angular_velocity.z = wz
        imu.linear_acceleration.z = 9.80665
        self._imu_pub.publish(imu)

        battery = BatteryState()
        battery.header.stamp = stamp
        battery.voltage = float(self.get_parameter("battery_voltage").value)
        battery.present = True
        self._battery_pub.publish(battery)

        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = self._odom_frame
        transform.child_frame_id = self._base_frame
        transform.transform.translation.x = self._x
        transform.transform.translation.y = self._y
        transform.transform.rotation.z = math.sin(half_yaw)
        transform.transform.rotation.w = math.cos(half_yaw)
        self._tf.sendTransform(transform)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MockBase()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
