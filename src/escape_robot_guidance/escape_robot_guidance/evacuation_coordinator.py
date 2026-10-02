"""ROS 2 evacuation mission coordinator.

This node intentionally consumes a generic hazard Boolean. A real building
alarm must be connected through a separately validated gateway; camera output
alone must never be treated as a certified fire alarm.
"""

from __future__ import annotations

import json
import math
import time
from dataclasses import dataclass
from enum import Enum

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import Bool, ColorRGBA, String

from .goal_lifecycle import GoalLifecycle


class MissionState(str, Enum):
    IDLE = "IDLE"
    SELECTING_EXIT = "SELECTING_EXIT"
    GUIDING = "GUIDING"
    ARRIVED = "ARRIVED"
    BLOCKED = "BLOCKED"
    FAULT = "FAULT"


@dataclass(slots=True)
class Exit:
    exit_id: str
    x: float
    y: float
    yaw: float
    hazard_risk: float
    crowd_density: float
    enabled: bool


class EvacuationCoordinator(Node):
    def __init__(self) -> None:
        super().__init__("evacuation_coordinator")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("hazard_weight", 8.0)
        self.declare_parameter("crowd_weight", 3.0)
        self.declare_parameter("distance_weight", 1.0)
        self.declare_parameter("exit_ids", ["east", "west"])
        self.declare_parameter("exit_east", [6.8, 1.2, 0.0, 0.05, 0.2, 1.0])
        self.declare_parameter("exit_west", [-5.4, -0.8, 3.14, 0.15, 0.1, 1.0])

        self._state = MissionState.IDLE
        self._pose_xy: tuple[float, float] | None = None
        self._blocked: set[str] = set()
        self._active_exit: str | None = None
        self._hazard_active = False
        self._goal_handle = None
        self._cancel_requested = False
        self._goals = GoalLifecycle()
        self._ready = False
        self._last_ready = 0.0
        self._exits = self._load_exits()

        self._nav = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self._status_pub = self.create_publisher(String, "guidance/status", 10)
        self._selected_exit_pub = self.create_publisher(String, "guidance/selected_exit", 10)
        self._beep_pub = self.create_publisher(Bool, "base/beep", 10)
        self._light_pub = self.create_publisher(ColorRGBA, "base/light", 10)
        self.create_subscription(Bool, "hazard/active", self._on_hazard, 10)
        self.create_subscription(Bool, "system/ready", self._on_ready, 10)
        self.create_subscription(String, "hazard/blocked_exits", self._on_blocked_exits, 10)
        self.create_subscription(PoseWithCovarianceStamped, "amcl_pose", self._on_pose, 10)
        self.create_timer(1.0, self._publish_status)
        self.create_timer(0.2, self._check_health)

    def _healthy(self) -> bool:
        return self._ready and time.monotonic() - self._last_ready <= 1.5

    def _on_ready(self, msg: Bool) -> None:
        self._ready = msg.data
        self._last_ready = time.monotonic()
        self._check_health()

    def _check_health(self) -> None:
        if self._hazard_active and not self._healthy():
            self._state = MissionState.FAULT
            self._cancel_goal("fault")

    def _cancel_goal(self, reason: str) -> None:
        self._goals.cancel(reason)
        if self._goal_handle is not None and not self._cancel_requested:
            self._cancel_requested = True
            self._goal_handle.cancel_goal_async()

    def _load_exits(self) -> list[Exit]:
        exits = []
        for exit_id in self.get_parameter("exit_ids").value:
            values = list(self.get_parameter(f"exit_{exit_id}").value)
            if len(values) != 6:
                raise ValueError(f"exit_{exit_id} must be [x,y,yaw,risk,crowd,enabled]")
            exits.append(Exit(str(exit_id), *map(float, values[:-1]), bool(values[-1])))
        return exits

    def _on_pose(self, msg: PoseWithCovarianceStamped) -> None:
        xy = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        self._pose_xy = xy if all(math.isfinite(v) for v in xy) else None

    def _on_blocked_exits(self, msg: String) -> None:
        self._blocked = {value.strip() for value in msg.data.split(",") if value.strip()}
        if (self._hazard_active and self._active_exit in self._blocked
                and self._state not in (MissionState.FAULT, MissionState.ARRIVED)):
            self.get_logger().warning(f"active exit {self._active_exit} became blocked; replanning")
            self._cancel_and_reselect()

    def _on_hazard(self, msg: Bool) -> None:
        if msg.data == self._hazard_active:
            return
        self._hazard_active = msg.data
        if msg.data:
            self._set_guidance_outputs(True)
            self._select_and_send_goal()
        else:
            self._set_guidance_outputs(False)
            self._cancel_goal("clear")
            self._active_exit = None
            self._state = MissionState.IDLE

    def _score_exit(self, candidate: Exit) -> float:
        if not candidate.enabled or candidate.exit_id in self._blocked:
            return math.inf
        distance = 0.0
        if self._pose_xy is not None:
            distance = math.hypot(candidate.x - self._pose_xy[0], candidate.y - self._pose_xy[1])
        return (
            float(self.get_parameter("distance_weight").value) * distance
            + float(self.get_parameter("hazard_weight").value) * candidate.hazard_risk
            + float(self.get_parameter("crowd_weight").value) * candidate.crowd_density
        )

    def _select_and_send_goal(self) -> None:
        if not self._hazard_active:
            return
        if self._goals.token is not None:
            self._cancel_goal("reselect")
            return
        if not self._healthy() or self._pose_xy is None:
            self._state = MissionState.FAULT
            self.get_logger().error("health/localization not ready; re-arm after inspection")
            return
        self._state = MissionState.SELECTING_EXIT
        candidates = sorted(
            ((self._score_exit(item), item) for item in self._exits),
            key=lambda item: (item[0], item[1].exit_id),
        )
        if not candidates or not math.isfinite(candidates[0][0]):
            self._state = MissionState.BLOCKED
            self.get_logger().error("no enabled and unblocked exit is available")
            return
        _, selected = candidates[0]
        self._active_exit = selected.exit_id
        self._selected_exit_pub.publish(String(data=selected.exit_id))

        if not self._nav.server_is_ready():
            self._state = MissionState.FAULT
            self.get_logger().error("Nav2 navigate_to_pose action server is unavailable")
            return

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = str(self.get_parameter("map_frame").value)
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = selected.x
        goal.pose.pose.position.y = selected.y
        goal.pose.pose.orientation.z = math.sin(selected.yaw * 0.5)
        goal.pose.pose.orientation.w = math.cos(selected.yaw * 0.5)
        token = self._goals.begin()
        try:
            future = self._nav.send_goal_async(goal, feedback_callback=self._on_feedback)
            future.add_done_callback(lambda done: self._on_goal_response(done, token))
        except Exception as exc:
            self._goals.finish(token)
            self._state = MissionState.FAULT
            self.get_logger().error(f"goal dispatch failed: {exc}")

    def _on_goal_response(self, future, token) -> None:
        try:
            handle = future.result()
        except Exception as exc:
            if self._goals.current(token):
                self._goals.finish(token)
                self._state = MissionState.FAULT if self._hazard_active else MissionState.IDLE
            self.get_logger().error(f"goal response failed: {exc}")
            return
        if not self._goals.current(token):
            if handle.accepted:
                handle.cancel_goal_async()
            return
        self._goal_handle = handle
        self._cancel_requested = False
        if not self._goal_handle.accepted:
            reason = self._goals.finish(token)
            self._goal_handle = None
            if reason == "reselect":
                self._select_and_send_goal()
                return
            if reason is None:
                self._state = MissionState.FAULT
            self.get_logger().error("Nav2 rejected the evacuation goal")
            return
        result_future = self._goal_handle.get_result_async()
        result_future.add_done_callback(lambda done: self._on_result(done, token))
        if self._goals.cancel_reason is not None:
            self._cancel_goal(self._goals.cancel_reason)
        else:
            self._state = MissionState.GUIDING

    def _on_feedback(self, feedback_msg) -> None:
        distance = feedback_msg.feedback.distance_remaining
        self.get_logger().debug(f"evacuation distance remaining: {distance:.2f} m")

    def _on_result(self, future, token) -> None:
        if not self._goals.current(token):
            return
        reason = self._goals.finish(token)
        self._goal_handle = None
        if reason is not None:
            if reason == "reselect":
                self._select_and_send_goal()
            return
        try:
            status = future.result().status
        except Exception as exc:
            self._state = MissionState.FAULT
            self.get_logger().error(f"goal result failed: {exc}")
            return
        if status == GoalStatus.STATUS_SUCCEEDED:
            self._state = MissionState.ARRIVED
            self._beep_pub.publish(Bool(data=False))
            self._light_pub.publish(ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0))
            return
        if self._hazard_active:
            if self._active_exit:
                self._blocked.add(self._active_exit)
            self.get_logger().warning("navigation failed; selecting an alternate exit")
            self._select_and_send_goal()
        else:
            self._state = MissionState.IDLE

    def _cancel_and_reselect(self) -> None:
        if self._goals.token is None:
            self._select_and_send_goal()
            return
        self._cancel_goal("reselect")

    def _set_guidance_outputs(self, active: bool) -> None:
        self._beep_pub.publish(Bool(data=active))
        color = ColorRGBA()
        color.r = 1.0 if active else 0.0
        color.g = 0.15 if active else 0.0
        color.b = 0.0
        color.a = 1.0
        self._light_pub.publish(color)

    def _publish_status(self) -> None:
        payload = {
            "state": self._state.value,
            "hazard_active": self._hazard_active,
            "system_ready": self._healthy(),
            "selected_exit": self._active_exit,
            "blocked_exits": sorted(self._blocked),
        }
        self._status_pub.publish(String(data=json.dumps(payload, ensure_ascii=False)))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = EvacuationCoordinator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
