class RobotInterface:
    """Mock robot interface.

    Future extensions can connect ROS2 topics/services/actions, MoveIt2,
    Isaac Sim, UR5 controllers, or a lab-specific robot controller.
    """

    def __init__(self, mode="mock"):
        self.mode = mode
        self.connected = False

    def connect(self):
        if self.mode != "mock":
            return {
                "status": "not_implemented",
                "message": f"Robot mode '{self.mode}' is reserved for future integration.",
            }
        self.connected = True
        return {"status": "mock_connected", "mode": self.mode}

    def execute_action_plan(self, action_plan):
        if not self.connected:
            self.connect()
        return {
            "status": "mock_executed",
            "mode": self.mode,
            "action_plan": action_plan,
            "message": "Mock robot action plan accepted. No real robot command was sent.",
        }

