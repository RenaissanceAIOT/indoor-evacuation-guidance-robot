"""ROS 2 node that owns the mobile-base serial port."""

from __future__ import annotations

import math
import threading
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import BatteryState, Imu
from std_msgs.msg import Bool, ColorRGBA, Empty
from tf2_ros import TransformBroadcaster
from trajectory_msgs.msg import JointTrajectory

from .protocol import (
    FrameParser,
    ProtocolError,
    TELEMETRY_CODE,
    build_arm_command,
    build_beep_command,
    build_imu_calibration_command,
    build_light_command,
    build_model_command,
    build_velocity_command,
    decode_telemetry,
)

try:
    import serial
except ImportError:  # pragma: no cover - reported cleanly at runtime
    serial = None


def _quaternion_from_yaw(yaw: float) -> tuple[float, float, float, float]:
    half = yaw * 0.5
    return 0.0, 0.0, math.sin(half), math.cos(half)


class BaseDriver(Node):
    def __init__(self) -> None:
        super().__init__("mobile_base_driver")
        self.declare_parameter("port", "/dev/escape_base")
        self.declare_parameter("baudrate", 230400)
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("imu_frame", "imu_link")
        self.declare_parameter("publish_tf", True)
        self.declare_parameter("mecanum", True)
        self.declare_parameter("cmd_timeout_sec", 0.35)
        self.declare_parameter("max_vx_mps", 0.45)
        self.declare_parameter("max_vy_mps", 0.35)
        self.declare_parameter("max_wz_rps", 1.2)
        self.declare_parameter("battery_capacity_ah", 10.0)
        self.declare_parameter("arm_enabled", False)

        self._base_frame = str(self.get_parameter("base_frame").value)
        self._odom_frame = str(self.get_parameter("odom_frame").value)
        self._imu_frame = str(self.get_parameter("imu_frame").value)
        self._publish_tf = bool(self.get_parameter("publish_tf").value)
        self._cmd_timeout = float(self.get_parameter("cmd_timeout_sec").value)
        self._limits = (
            float(self.get_parameter("max_vx_mps").value),
            float(self.get_parameter("max_vy_mps").value),
            float(self.get_parameter("max_wz_rps").value),
        )

        self._parser = FrameParser()
        self._serial = self._open_serial()
        self._write_lock = threading.Lock()
        self._last_command_time = time.monotonic()
        self._watchdog_stopped = False
        self._system_stop = False
        self._last_interlock_time = None
        self._last_telemetry_time: float | None = None
        self._x = 0.0
        self._y = 0.0
        self._yaw = 0.0
        self._rx_frames = 0
        self._tx_frames = 0

        self._odom_pub = self.create_publisher(Odometry, "odom", 20)
        self._imu_pub = self.create_publisher(Imu, "imu/data_raw", 20)
        self._battery_pub = self.create_publisher(BatteryState, "battery_state", 10)
        self._diagnostics_pub = self.create_publisher(DiagnosticArray, "diagnostics", 10)
        self._tf = TransformBroadcaster(self)

        self.create_subscription(Twist, "cmd_vel", self._on_cmd_vel, 20)
        self.create_subscription(Bool, "base/stop", self._on_stop, 10)
        self.create_subscription(JointTrajectory, "arm_controller/joint_trajectory", self._on_arm, 10)
        self.create_subscription(Bool, "base/beep", self._on_beep, 10)
        self.create_subscription(ColorRGBA, "base/light", self._on_light, 10)
        self.create_subscription(Empty, "base/calibrate_imu", self._on_calibrate_imu, 10)

        self.create_timer(0.01, self._poll_serial)
        self.create_timer(0.05, self._enforce_watchdog)
        self.create_timer(1.0, self._publish_diagnostics)

        self._write(build_model_command(bool(self.get_parameter("mecanum").value)))
        self.get_logger().info(
            f"Mobile-base controller connected on {self.get_parameter('port').value} at "
            f"{self.get_parameter('baudrate').value} bps"
        )

    def _open_serial(self):
        if serial is None:
            raise RuntimeError("pyserial is not installed; install python3-serial")
        port = str(self.get_parameter("port").value)
        baudrate = int(self.get_parameter("baudrate").value)
        try:
            return serial.Serial(port=port, baudrate=baudrate, timeout=0, write_timeout=0.1)
        except serial.SerialException as exc:
            raise RuntimeError(f"cannot open mobile-base serial port {port}: {exc}") from exc

    def _write(self, frame: bytes) -> None:
        with self._write_lock:
            try:
                self._serial.write(frame)
                self._tx_frames += 1
            except Exception as exc:  # serial errors are device/environment specific
                self.get_logger().error(f"serial write failed: {exc}", throttle_duration_sec=2.0)

    def _on_cmd_vel(self, msg: Twist) -> None:
        if self._motion_inhibited():
            self._write(build_velocity_command(0.0, 0.0, 0.0))
            return
        if not all(math.isfinite(v) for v in (msg.linear.x, msg.linear.y, msg.angular.z)):
            self._write(build_velocity_command(0.0, 0.0, 0.0))
            self.get_logger().error("non-finite velocity command rejected")
            return
        self._write(
            build_velocity_command(
                msg.linear.x,
                msg.linear.y,
                msg.angular.z,
                max_vx_mps=self._limits[0],
                max_vy_mps=self._limits[1],
                max_wz_rps=self._limits[2],
            )
        )
        self._last_command_time = time.monotonic()
        self._watchdog_stopped = False

    def _on_stop(self, msg: Bool) -> None:
        self._system_stop = msg.data
        self._last_interlock_time = time.monotonic()
        if msg.data:
            self._write(build_velocity_command(0.0, 0.0, 0.0))

    def _motion_inhibited(self) -> bool:
        now = time.monotonic()
        telemetry_stale = self._last_telemetry_time is None or now - self._last_telemetry_time > 0.3
        interlock_stale = (self._last_interlock_time is not None
                           and now - self._last_interlock_time > 1.5)
        return telemetry_stale or self._system_stop or interlock_stale

    def _on_arm(self, msg: JointTrajectory) -> None:
        if not self.get_parameter("arm_enabled").value:
            self.get_logger().warning("arm bridge disabled; verify firmware before enabling")
            return
        if not msg.points:
            self.get_logger().warning("ignored arm trajectory without points")
            return
        try:
            self._write(build_arm_command(msg.points[-1].positions))
        except ProtocolError as exc:
            self.get_logger().error(f"invalid arm command: {exc}")

    def _on_beep(self, msg: Bool) -> None:
        self._write(build_beep_command(msg.data))

    def _on_light(self, msg: ColorRGBA) -> None:
        red = int(max(0.0, min(1.0, msg.r)) * 255)
        green = int(max(0.0, min(1.0, msg.g)) * 255)
        blue = int(max(0.0, min(1.0, msg.b)) * 255)
        self._write(build_light_command(mode=1, red=red, green=green, blue=blue))

    def _on_calibrate_imu(self, _msg: Empty) -> None:
        self._write(build_imu_calibration_command())

    def _enforce_watchdog(self) -> None:
        if (time.monotonic() - self._last_command_time <= self._cmd_timeout
                and not self._motion_inhibited()):
            return
        self._write(build_velocity_command(0.0, 0.0, 0.0))
        if not self._watchdog_stopped:
            self._watchdog_stopped = True
            self.get_logger().warning("watchdog/interlock: commanded zero velocity")

    def _poll_serial(self) -> None:
        try:
            waiting = self._serial.in_waiting
            if waiting <= 0:
                return
            for frame in self._parser.feed(self._serial.read(waiting)):
                self._rx_frames += 1
                if frame.code == TELEMETRY_CODE:
                    self._publish_telemetry(decode_telemetry(frame))
        except Exception as exc:
            self.get_logger().error(f"serial read/parse failed: {exc}", throttle_duration_sec=2.0)

    def _publish_telemetry(self, data) -> None:
        monotonic_now = time.monotonic()
        dt = 0.0 if self._last_telemetry_time is None else monotonic_now - self._last_telemetry_time
        self._last_telemetry_time = monotonic_now
        if not 0.0 < dt < 0.2:
            dt = 0.02

        vx, vy, wz = data.velocity_mps
        self._x += (vx * math.cos(self._yaw) - vy * math.sin(self._yaw)) * dt
        self._y += (vx * math.sin(self._yaw) + vy * math.cos(self._yaw)) * dt
        self._yaw = math.atan2(math.sin(self._yaw + wz * dt), math.cos(self._yaw + wz * dt))
        qx, qy, qz, qw = _quaternion_from_yaw(self._yaw)
        stamp = self.get_clock().now().to_msg()

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self._odom_frame
        odom.child_frame_id = self._base_frame
        odom.pose.pose.position.x = self._x
        odom.pose.pose.position.y = self._y
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = wz
        odom.pose.covariance[0] = 0.03
        odom.pose.covariance[7] = 0.03
        odom.pose.covariance[35] = 0.08
        odom.twist.covariance[0] = 0.02
        odom.twist.covariance[7] = 0.02
        odom.twist.covariance[35] = 0.04
        self._odom_pub.publish(odom)

        imu = Imu()
        imu.header.stamp = stamp
        imu.header.frame_id = self._imu_frame
        imu.orientation_covariance[0] = -1.0
        imu.linear_acceleration.x, imu.linear_acceleration.y, imu.linear_acceleration.z = (
            data.acceleration_mps2
        )
        imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z = (
            data.angular_velocity_rps
        )
        imu.linear_acceleration_covariance[0] = 0.08
        imu.linear_acceleration_covariance[4] = 0.08
        imu.linear_acceleration_covariance[8] = 0.12
        imu.angular_velocity_covariance[0] = 0.02
        imu.angular_velocity_covariance[4] = 0.02
        imu.angular_velocity_covariance[8] = 0.015
        self._imu_pub.publish(imu)

        battery = BatteryState()
        battery.header.stamp = stamp
        battery.voltage = data.battery_voltage
        battery.capacity = float(self.get_parameter("battery_capacity_ah").value)
        battery.present = True
        battery.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LIPO
        battery.percentage = max(0.0, min(1.0, (data.battery_voltage - 9.84) / (12.6 - 9.84)))
        if data.battery_voltage < 9.84:
            battery.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_DEAD
        elif data.battery_voltage < 10.12:
            battery.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_UNSPEC_FAILURE
        else:
            battery.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_GOOD
        self._battery_pub.publish(battery)

        if self._publish_tf:
            transform = TransformStamped()
            transform.header.stamp = stamp
            transform.header.frame_id = self._odom_frame
            transform.child_frame_id = self._base_frame
            transform.transform.translation.x = self._x
            transform.transform.translation.y = self._y
            transform.transform.rotation.x = qx
            transform.transform.rotation.y = qy
            transform.transform.rotation.z = qz
            transform.transform.rotation.w = qw
            self._tf.sendTransform(transform)

    def _publish_diagnostics(self) -> None:
        now = time.monotonic()
        telemetry_age = -1.0 if self._last_telemetry_time is None else now - self._last_telemetry_time
        healthy = 0.0 <= telemetry_age < 0.3
        status = DiagnosticStatus()
        status.name = "Laboratory mobile-base serial link"
        status.hardware_id = str(self.get_parameter("port").value)
        status.level = DiagnosticStatus.OK if healthy else DiagnosticStatus.ERROR
        status.message = "telemetry active" if healthy else "telemetry stale or absent"
        status.values = [
            KeyValue(key="telemetry_age_sec", value=f"{telemetry_age:.3f}"),
            KeyValue(key="rx_frames", value=str(self._rx_frames)),
            KeyValue(key="tx_frames", value=str(self._tx_frames)),
            KeyValue(key="bad_checksums", value=str(self._parser.bad_checksums)),
            KeyValue(key="discarded_bytes", value=str(self._parser.discarded_bytes)),
        ]
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        array.status = [status]
        self._diagnostics_pub.publish(array)

    def destroy_node(self) -> bool:
        try:
            self._write(build_velocity_command(0.0, 0.0, 0.0))
            self._serial.close()
        finally:
            return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = BaseDriver()
        rclpy.spin(node)
    except (RuntimeError, KeyboardInterrupt) as exc:
        if node is not None:
            node.get_logger().error(str(exc))
        else:
            print(f"base driver failed: {exc}")
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
