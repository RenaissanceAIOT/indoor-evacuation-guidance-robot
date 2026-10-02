"""Single-goal ownership, independent of ROS for deterministic testing."""


class GoalLifecycle:
    def __init__(self):
        self.sequence = 0
        self.token = None
        self.cancel_reason = None

    def begin(self):
        if self.token is not None:
            raise RuntimeError("previous goal has not terminated")
        self.sequence += 1
        self.token = self.sequence
        self.cancel_reason = None
        return self.token

    def current(self, token):
        return self.token == token

    def cancel(self, reason):
        if self.token is not None:
            # Safety/clear events take precedence over a replan already queued.
            if self.cancel_reason != "reselect" and reason == "reselect" and self.cancel_reason:
                return
            self.cancel_reason = reason

    def finish(self, token):
        if not self.current(token):
            raise RuntimeError("stale goal callback")
        reason = self.cancel_reason
        self.token = None
        self.cancel_reason = None
        return reason
