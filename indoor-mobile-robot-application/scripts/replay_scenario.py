"""Synthetic routing demonstration. No ROS, hardware or building data required."""

import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src/escape_robot_guidance"))
from escape_robot_guidance.risk_router import Edge, ExitScore, RiskGraph


def evaluate(blocked):
    graph = RiskGraph()
    graph.add_edge("lobby", Edge("east", 8.0, 0.05, 0.2, "east" in blocked))
    graph.add_edge("lobby", Edge("west", 11.0, 0.15, 0.1, "west" in blocked))
    return graph.safest_exit("lobby", [ExitScore("east"), ExitScore("west")])


def main():
    for name, blocked, expected in (
        ("normal", set(), "east"),
        ("east_blocked", {"east"}, "west"),
        ("all_blocked", {"east", "west"}, None),
    ):
        route = evaluate(blocked)
        actual = route.exit_id if route else None
        assert actual == expected, (name, actual, expected)
        print(json.dumps({
            "source": "synthetic_fixture_not_field_result",
            "scenario": name,
            "exit": actual,
            "path": route.path if route else [],
            "cost": round(route.total_cost, 3) if route else None,
            "decision": "GUIDE" if route else "BLOCKED",
        }, ensure_ascii=False))


if __name__ == "__main__":
    main()
