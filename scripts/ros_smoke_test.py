"""Run actual ROS message checks against the kinematic mock, never hardware.

Requires a built and sourced ROS 2 workspace. No map, sensor or model required.
Run in an isolated ROS_DOMAIN_ID; the script publishes relative cmd_vel/stop
topics within a private namespace and destroys both nodes on exit.
"""

import math
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Bool

from escape_robot_base.mock_base_node import MockBase


def main() -> None:
    namespace = "/portfolio_mock_check"
    rclpy.init(args=["--ros-args", "-r", f"__ns:={namespace}"])
    base = MockBase()
    probe = Node("mock_probe")
    executor = SingleThreadedExecutor()
    latest = []
    samples = []

    def receive(message: Odometry) -> None:
        latest[:] = [message]
        samples.append(time.monotonic())

    probe.create_subscription(Odometry, "odom", receive, 20)
    command_pub = probe.create_publisher(Twist, "cmd_vel", 20)
    stop_pub = probe.create_publisher(Bool, "base/stop", 10)
    executor.add_node(base)
    executor.add_node(probe)

    def spin_for(duration, command=None, stop=None):
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            if command is not None:
                command_pub.publish(command)
            if stop is not None:
                stop_pub.publish(Bool(data=stop))
            executor.spin_once(timeout_sec=0.02)

    def velocity():
        assert latest and time.monotonic() - samples[-1] < 0.3, "no fresh odometry"
        twist = latest[0].twist.twist
        return twist.linear.x, twist.linear.y, twist.angular.z

    def assert_velocity(expected):
        assert all(math.isclose(a, b, abs_tol=1e-6) for a, b in zip(velocity(), expected)), (
            velocity(), expected
        )

    try:
        deadline = time.monotonic() + 5.0
        while not latest or command_pub.get_subscription_count() == 0 or stop_pub.get_subscription_count() == 0:
            assert time.monotonic() < deadline, "ROS discovery timed out"
            executor.spin_once(timeout_sec=0.05)

        command = Twist()
        command.linear.x, command.linear.y, command.angular.z = 0.2, -0.1, 0.3
        initial_x = latest[0].pose.pose.position.x
        spin_for(0.5, command=command)
        assert_velocity((0.2, -0.1, 0.3))
        assert latest[0].pose.pose.position.x > initial_x, "pose did not integrate"

        spin_for(0.65)
        assert_velocity((0.0, 0.0, 0.0))

        spin_for(0.25, command=command, stop=True)
        assert_velocity((0.0, 0.0, 0.0))
        spin_for(0.25, command=command, stop=False)
        assert_velocity((0.2, -0.1, 0.3))
        spin_for(1.8, command=command)
        assert_velocity((0.0, 0.0, 0.0))

        invalid = Twist()
        invalid.linear.x = float("nan")
        spin_for(0.25, command=invalid, stop=False)
        assert_velocity((0.0, 0.0, 0.0))
        print("ROS mock smoke passed: motion, pose integration, watchdog, stop/release, stale interlock, nonfinite rejection")
    finally:
        executor.shutdown()
        probe.destroy_node()
        base.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
