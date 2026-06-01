from src.articulation_state_observer import (
    OBSERVATION_STATUS_NOT_OBSERVABLE,
    OBSERVATION_STATUS_OK,
    build_articulation_state_report,
    observe_articulation_state,
)


STANDARD_INSPECTION = {
    "requested": True,
    "robot_prim_path": "/World/TETO_Robot",
    "robot_prim_exists": True,
    "joint_metadata_table": [
        {
            "joint_name": "shoulder_pan_joint",
            "joint_prim_path": "/World/TETO_Robot/shoulder_pan_joint",
            "category": "arm",
            "metadata_only": True,
            "control_target_generated": False,
        },
        {
            "joint_name": "shoulder_lift_joint",
            "joint_prim_path": "/World/TETO_Robot/shoulder_lift_joint",
            "category": "arm",
            "metadata_only": True,
            "control_target_generated": False,
        },
        {
            "joint_name": "elbow_joint",
            "joint_prim_path": "/World/TETO_Robot/elbow_joint",
            "category": "arm",
            "metadata_only": True,
            "control_target_generated": False,
        },
        {
            "joint_name": "wrist_1_joint",
            "joint_prim_path": "/World/TETO_Robot/wrist_1_joint",
            "category": "arm",
            "metadata_only": True,
            "control_target_generated": False,
        },
        {
            "joint_name": "wrist_2_joint",
            "joint_prim_path": "/World/TETO_Robot/wrist_2_joint",
            "category": "arm",
            "metadata_only": True,
            "control_target_generated": False,
        },
        {
            "joint_name": "wrist_3_joint",
            "joint_prim_path": "/World/TETO_Robot/wrist_3_joint",
            "category": "arm",
            "metadata_only": True,
            "control_target_generated": False,
        },
        {
            "joint_name": "robot_gripper_joint",
            "joint_prim_path": "/World/TETO_Robot/robot_gripper_joint",
            "category": "gripper_or_tool",
            "metadata_only": True,
            "control_target_generated": False,
        },
    ],
}

READY = {"articulation_ready": True, "robot_prim_path": "/World/TETO_Robot"}


class FakeAttribute:
    def __init__(self, value):
        self.value = value

    def Get(self):
        return self.value


class FakePrim:
    def __init__(self, path, attributes=None, valid=True):
        self.path = path
        self.attributes = attributes or {}
        self.valid = valid

    def IsValid(self):
        return self.valid

    def GetAttribute(self, name):
        if name not in self.attributes:
            return None
        return FakeAttribute(self.attributes[name])


class FakeStage:
    def __init__(self, prims):
        self.prims = prims

    def GetPrimAtPath(self, path):
        return self.prims.get(path, FakePrim(path, valid=False))


class FakeWorld:
    def __init__(self, stage):
        self.stage = stage


def _state_rows(joint_names):
    return [
        {
            "joint_name": joint_name,
            "category": "arm",
            "position": 0.0,
            "velocity": 0.0,
            "lower_limit": -3.14,
            "upper_limit": 3.14,
        }
        for joint_name in joint_names
    ]


def test_dry_run_missing_articulation_is_not_observable_without_crashing():
    report = build_articulation_state_report(
        requested=True,
        robot_prim_path="/World/TETO_Robot",
        robot_prim_inspection={"robot_prim_exists": False},
    )

    assert report["status"] == OBSERVATION_STATUS_NOT_OBSERVABLE
    assert report["articulation_state_observable"] is False
    assert report["control_enabled"] is False
    assert report["motion_generated"] is False
    assert report["command_generated"] is False
    assert report["joint_targets_generated"] is False


def test_true_like_articulation_state_generates_ok_report():
    report = build_articulation_state_report(
        requested=True,
        robot_prim_path="/World/TETO_Robot",
        robot_prim_inspection=STANDARD_INSPECTION,
        articulation_readiness=READY,
        joint_state_table=_state_rows(
            [
                "shoulder_pan_joint",
                "shoulder_lift_joint",
                "elbow_joint",
                "wrist_1_joint",
                "wrist_2_joint",
                "wrist_3_joint",
            ]
        ),
    )

    assert report["status"] == OBSERVATION_STATUS_OK
    assert report["articulation_state_observable"] is True
    assert report["arm_joint_count"] == 6
    assert report["missing_arm_joint_names"] == []
    assert report["joint_positions_available"] is True
    assert report["joint_velocities_available"] is True
    assert report["joint_limits_available"] is True
    assert report["joint_state_table"][0]["metadata_only"] is True
    assert report["joint_state_table"][0]["control_target_generated"] is False


def test_missing_joints_are_recorded_without_control_targets():
    report = build_articulation_state_report(
        requested=True,
        robot_prim_path="/World/TETO_Robot",
        robot_prim_inspection=STANDARD_INSPECTION,
        articulation_readiness=READY,
        joint_state_table=_state_rows(["shoulder_pan_joint", "shoulder_lift_joint"]),
    )

    assert report["status"] == OBSERVATION_STATUS_OK
    assert report["arm_joint_count"] == 2
    assert "elbow_joint" in report["missing_arm_joint_names"]
    assert report["joint_targets_generated"] is False


def test_missing_joint_limits_warn_without_crashing():
    report = build_articulation_state_report(
        requested=True,
        robot_prim_path="/World/TETO_Robot",
        robot_prim_inspection=STANDARD_INSPECTION,
        articulation_readiness=READY,
        joint_state_table=[
            {
                "joint_name": "shoulder_pan_joint",
                "category": "arm",
                "position": 0.0,
                "velocity": 0.0,
            }
        ],
    )

    assert report["status"] == OBSERVATION_STATUS_OK
    assert report["joint_limits_available"] is False
    assert any("joint limits unavailable" in warning for warning in report["warnings"])
    assert report["command_generated"] is False


def test_joint_position_outside_limit_records_error_without_command():
    report = build_articulation_state_report(
        requested=True,
        robot_prim_path="/World/TETO_Robot",
        robot_prim_inspection=STANDARD_INSPECTION,
        articulation_readiness=READY,
        joint_state_table=[
            {
                "joint_name": "shoulder_pan_joint",
                "category": "arm",
                "position": 4.0,
                "velocity": 0.0,
                "lower_limit": -1.0,
                "upper_limit": 1.0,
            }
        ],
    )

    assert report["joint_state_table"][0]["within_limit"] is False
    assert any("E_JOINT_LIMIT_VIOLATION" in error for error in report["errors"])
    assert report["control_enabled"] is False
    assert report["motion_generated"] is False
    assert report["command_generated"] is False
    assert report["joint_targets_generated"] is False


def test_observe_articulation_state_reads_stage_attributes():
    stage = FakeStage(
        {
            "/World/TETO_Robot/shoulder_pan_joint": FakePrim(
                "/World/TETO_Robot/shoulder_pan_joint",
                attributes={
                    "state:angular:physics:position": 0.1,
                    "state:angular:physics:velocity": 0.2,
                    "physics:lowerLimit": -3.14,
                    "physics:upperLimit": 3.14,
                },
            )
        }
    )
    report = observe_articulation_state(
        FakeWorld(stage),
        robot_prim_path="/World/TETO_Robot",
        robot_prim_inspection={
            "robot_prim_exists": True,
            "joint_metadata_table": [
                {
                    "joint_name": "shoulder_pan_joint",
                    "joint_prim_path": "/World/TETO_Robot/shoulder_pan_joint",
                    "category": "arm",
                }
            ],
        },
        articulation_readiness=READY,
    )

    assert report["status"] == OBSERVATION_STATUS_OK
    assert report["joint_state_table"][0]["position"] == 0.1
    assert report["joint_state_table"][0]["velocity"] == 0.2
    assert report["joint_state_table"][0]["limit_available"] is True
