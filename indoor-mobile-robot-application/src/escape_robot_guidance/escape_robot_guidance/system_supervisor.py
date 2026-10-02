"""Runtime readiness monitor and mission-event recorder."""

from __future__ import annotations

import json
import os
import time
from datetime import datetime, timezone
from pathlib import Path

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import BatteryState, LaserScan
from std_msgs.msg import Bool, String


class SystemSupervisor(Node):
    def __init__(self) -> None:
        super().__init__("system_supervisor")
        self.declare_parameter("base_timeout_sec", 2.0)
        self.declare_parameter("scan_timeout_sec", 1.0)
        self.declare_parameter("minimum_voltage", 10.12)
        self.declare_parameter("log_directory", "~/.local/share/escape_robot/logs")
        self._last_base = 0.0
        self._last_scan = 0.0
        self._voltage = 0.0
        self._last_battery = 0.0
        self._base_ok = False
        self._ready_pub = self.create_publisher(Bool, "system/ready", 10)
        self._stop_pub = self.create_publisher(Bool, "base/stop", 10)
        self._health_pub = self.create_publisher(String, "system/health", 10)
        self.create_subscription(DiagnosticArray, "diagnostics", self._on_diagnostics, 10)
        self.create_subscription(BatteryState, "battery_state", self._on_battery, 10)
        self.create_subscription(LaserScan, "scan", self._on_scan, qos_profile_sensor_data)
        self.create_subscription(String, "guidance/status", self._record_guidance, 10)
        self.create_timer(0.5, self._evaluate)

        log_dir = Path(os.path.expanduser(str(self.get_parameter("log_directory").value)))
        try:
            log_dir.mkdir(parents=True, exist_ok=True)
            session = datetime.now(timezone.utc).strftime("mission-%Y%m%dT%H%M%SZ.jsonl")
            self._log_path: Path | None = log_dir / session
        except OSError as exc:
            self.get_logger().warning(f"mission logging disabled: {exc}")
            self._log_path = None

    def _on_diagnostics(self, msg: DiagnosticArray) -> None:
        relevant = [status for status in msg.status if "mobile-base" in status.name.lower()]
        if not relevant:
            return
        self._last_base = time.monotonic()
        self._base_ok = all(status.level == status.OK for status in relevant)

    def _on_battery(self, msg: BatteryState) -> None:
        self._voltage = float(msg.voltage)
        self._last_battery = time.monotonic()

    def _on_scan(self, _msg: LaserScan) -> None:
        self._last_scan = time.monotonic()

    def _record_guidance(self, msg: String) -> None:
        self._append_event("guidance_status", msg.data)

    def _append_event(self, event_type: str, payload: str) -> None:
        if self._log_path is None:
            return
        event = {
            "timestamp": datetime.now(timezone.utc).isoformat(),
            "event": event_type,
            "payload": payload,
        }
        try:
            with self._log_path.open("a", encoding="utf-8") as stream:
                stream.write(json.dumps(event, ensure_ascii=False) + "\n")
        except OSError as exc:
            self.get_logger().warning(f"mission log write failed: {exc}", throttle_duration_sec=5.0)

    def _evaluate(self) -> None:
        now = time.monotonic()
        base_fresh = now - self._last_base <= float(self.get_parameter("base_timeout_sec").value)
        scan_fresh = now - self._last_scan <= float(self.get_parameter("scan_timeout_sec").value)
        power_ok = (now - self._last_battery <= 2.0
                    and self._voltage >= float(self.get_parameter("minimum_voltage").value))
        ready = base_fresh and self._base_ok and scan_fresh and power_ok
        health = {
            "ready": ready,
            "base_link": "ok" if base_fresh and self._base_ok else "stale_or_error",
            "laser_scan": "ok" if scan_fresh else "stale",
            "power": "ok" if power_ok else "low",
            "voltage": round(self._voltage, 2),
        }
        self._ready_pub.publish(Bool(data=ready))
        self._stop_pub.publish(Bool(data=not ready))
        self._health_pub.publish(String(data=json.dumps(health, ensure_ascii=False)))
        self._append_event("system_health", json.dumps(health, ensure_ascii=False))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SystemSupervisor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
