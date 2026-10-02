import unittest

from escape_robot_guidance.risk_router import Edge, ExitScore, RiskGraph


def make_graph() -> RiskGraph:
    graph = RiskGraph()
    graph.add_edge("lobby", Edge("corridor_a", 4.0, hazard_risk=0.1))
    graph.add_edge("corridor_a", Edge("east", 3.0, crowd_density=0.5))
    graph.add_edge("lobby", Edge("corridor_b", 5.0))
    graph.add_edge("corridor_b", Edge("west", 4.0, crowd_density=0.1))
    return graph


class RiskRouterTest(unittest.TestCase):
    def test_safest_exit_balances_distance_and_risk(self) -> None:
        result = make_graph().safest_exit(
            "lobby",
            [ExitScore("east", terminal_risk=0.6), ExitScore("west", terminal_risk=0.0)],
            hazard_weight=8.0,
            crowd_weight=3.0,
        )
        self.assertIsNotNone(result)
        self.assertEqual(result.exit_id, "west")
        self.assertEqual(result.path, ("lobby", "corridor_b", "west"))

    def test_disabled_exit_is_never_selected(self) -> None:
        result = make_graph().safest_exit(
            "lobby",
            [ExitScore("east", enabled=False), ExitScore("west")],
        )
        self.assertIsNotNone(result)
        self.assertEqual(result.exit_id, "west")

    def test_returns_none_when_all_exits_unreachable(self) -> None:
        graph = RiskGraph()
        graph.add_edge("start", Edge("exit", 1.0, blocked=True))
        self.assertIsNone(graph.safest_exit("start", [ExitScore("exit")]))


if __name__ == "__main__":
    unittest.main()
