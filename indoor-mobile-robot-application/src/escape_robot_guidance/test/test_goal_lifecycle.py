import unittest

from escape_robot_guidance.goal_lifecycle import GoalLifecycle


class GoalLifecycleTests(unittest.TestCase):
    def test_single_goal_ownership(self):
        owner = GoalLifecycle()
        token = owner.begin()
        with self.assertRaises(RuntimeError):
            owner.begin()
        owner.finish(token)
        self.assertGreater(owner.begin(), token)

    def test_reselect_waits_for_terminal_result(self):
        owner = GoalLifecycle()
        token = owner.begin()
        owner.cancel("reselect")
        self.assertTrue(owner.current(token))
        self.assertEqual(owner.finish(token), "reselect")
        self.assertIsNone(owner.token)

    def test_safety_cancellation_cannot_be_overwritten_by_replan(self):
        owner = GoalLifecycle()
        token = owner.begin()
        owner.cancel("reselect")
        owner.cancel("fault")
        owner.cancel("reselect")
        self.assertEqual(owner.finish(token), "fault")

    def test_late_callback_cannot_finish_new_goal(self):
        owner = GoalLifecycle()
        old = owner.begin()
        owner.finish(old)
        new = owner.begin()
        self.assertFalse(owner.current(old))
        with self.assertRaises(RuntimeError):
            owner.finish(old)
        self.assertTrue(owner.current(new))

    def test_clear_during_acceptance_preserves_cancel_intent(self):
        owner = GoalLifecycle()
        token = owner.begin()
        owner.cancel("clear")
        owner.cancel("reselect")
        self.assertEqual(owner.finish(token), "clear")


if __name__ == "__main__":
    unittest.main()
