from src.simulation_motion_precheck import build_simulation_motion_precheck_report


ARM_JOINTS = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]


def ready_readiness(**overrides):
    report = {
        "readiness_status": "READY",
        "articulation_ready": True,
        "robot_prim_path": "/World/TETO_Robot",
        "articulation_root_found": True,
        "arm_joint_count": 6,
        "arm_joint_names": list(ARM_JOINTS),
        "missing_arm_joint_names": [],
        "extra_joint_like_names": ["robot_gripper_joint", "root_joint"],
    }
    report.update(overrides)
    return report


def ready_state(**overrides):
    rows = [
        {
            "joint_name": joint_name,
            "category": "arm",
            "position": 0.0,
            "velocity": 0.0,
            "lower_limit": -360.0,
            "upper_limit": 360.0,
            "limit_available": True,
            "within_limit": True,
            "metadata_only": True,
            "control_target_generated": False,
        }
        for joint_name in ARM_JOINTS
    ]
    rows.extend(
        [
            {
                "joint_name": "robot_gripper_joint",
                "category": "gripper_or_tool",
                "position": None,
                "velocity": None,
                "lower_limit": None,
                "upper_limit": None,
                "limit_available": False,
                "within_limit": None,
                "metadata_only": True,
                "control_target_generated": False,
            },
            {
                "joint_name": "root_joint",
                "category": "structural",
                "position": None,
                "velocity": None,
                "lower_limit": None,
                "upper_limit": None,
                "limit_available": False,
                "within_limit": None,
                "metadata_only": True,
                "control_target_generated": False,
            },
        ]
    )
    report = {
        "status": "OK",
        "metadata_only": True,
        "control_enabled": False,
        "motion_generated": False,
        "command_generated": False,
        "joint_targets_generated": False,
        "robot_prim_path": "/World/TETO_Robot",
        "articulation_state_observable": True,
        "arm_joint_count": 6,
        "observed_joint_count": 8,
        "expected_arm_joint_names": list(ARM_JOINTS),
        "observed_arm_joint_names": list(ARM_JOINTS),
        "missing_arm_joint_names": [],
        "extra_joint_names": ["robot_gripper_joint", "root_joint"],
        "joint_positions_available": True,
        "joint_velocities_available": True,
        "joint_limits_available": True,
        "joint_state_table": rows,
        "warnings": [],
        "errors": [],
    }
    report.update(overrides)
    return report


def ready_precheck(**overrides):
    params = {
        "requested": True,
        "robot_asset_loaded": True,
        "robot_prim_exists": True,
        "robot_prim_path": "/World/TETO_Robot",
        "robot_prim_inspection": {"articulation_root_found": True},
        "articulation_readiness": ready_readiness(),
        "articulation_state": ready_state(),
    }
    params.update(overrides)
    return build_simulation_motion_precheck_report(**params)


def assert_no_control(report):
    assert report["control_enabled"] is False
    assert report["motion_generated"] is False
    assert report["command_generated"] is False
    assert report["joint_targets_generated"] is False
    assert report["trajectory_generated"] is False
    assert report["tcp_pose_world_generated"] is False
    assert report["robot_motion_executed"] is False
    assert report["real_robot_allowed"] is False


def test_dry_run_missing_articulation_is_not_ready_without_crash():
    report = build_simulation_motion_precheck_report(requested=True)

    assert report["status"] == "NOT_READY"
    assert report["ready"] is False
    assert "robot_prim_exists" in report["missing_requirements"]
    assert "articulation_readiness_ready" in report["missing_requirements"]
    assert "articulation_state_ok" in report["missing_requirements"]
    assert_no_control(report)


def test_ready_input_is_ready_for_simulation_motion():
    report = ready_precheck()

    assert report["status"] == "READY_FOR_SIMULATION_MOTION"
    assert report["ready"] is True
    assert report["blocking_reasons"] == []
    assert report["arm_joint_count"] == 6
    assert report["observed_joint_count"] == 8
    assert report["non_arm_extra_joints"] == ["robot_gripper_joint", "root_joint"]
    assert report["joint_positions_within_limits"] is True
    assert_no_control(report)


def test_missing_robot_asset_is_not_ready():
    report = ready_precheck(robot_asset_loaded=False)

    assert report["status"] == "NOT_READY"
    assert "robot_asset_loaded" in report["missing_requirements"]
    assert "E_ROBOT_ASSET_NOT_LOADED" in report["blocking_reasons"]


def test_missing_robot_prim_is_not_ready():
    report = ready_precheck(robot_prim_exists=False)

    assert report["status"] == "NOT_READY"
    assert "robot_prim_exists" in report["missing_requirements"]


def test_articulation_readiness_not_ready_blocks_precheck():
    report = ready_precheck(articulation_readiness=ready_readiness(readiness_status="NOT_READY", articulation_ready=False))

    assert report["status"] == "NOT_READY"
    assert "articulation_readiness_ready" in report["missing_requirements"]
    assert "articulation_ready" in report["missing_requirements"]


def test_articulation_state_not_observable_blocks_precheck():
    report = ready_precheck(
        articulation_state=ready_state(status="NOT_OBSERVABLE", articulation_state_observable=False)
    )

    assert report["status"] == "NOT_READY"
    assert "articulation_state_ok" in report["missing_requirements"]
    assert "articulation_state_observable" in report["missing_requirements"]


def test_missing_arm_joint_blocks_precheck():
    state = ready_state(
        arm_joint_count=5,
        observed_arm_joint_names=ARM_JOINTS[:-1],
        missing_arm_joint_names=["wrist_3_joint"],
    )
    report = ready_precheck(articulation_state=state)

    assert report["status"] == "NOT_READY"
    assert "arm_joint_count_6" in report["missing_requirements"]
    assert "no_missing_arm_joints" in report["missing_requirements"]


def test_joint_limit_missing_warns_and_blocks_precheck():
    state = ready_state()
    state["joint_state_table"][0]["limit_available"] = False
    state["joint_state_table"][0]["lower_limit"] = None
    state["joint_state_table"][0]["upper_limit"] = None
    state["joint_limits_available"] = False
    report = ready_precheck(articulation_state=state)

    assert report["status"] == "NOT_READY"
    assert "joint_limits_available" in report["missing_requirements"]
    assert any("joint limits unavailable" in warning for warning in report["warnings"])


def test_joint_position_out_of_limit_errors_and_blocks_precheck():
    state = ready_state()
    state["joint_state_table"][0]["position"] = 999.0
    state["joint_state_table"][0]["within_limit"] = False
    report = ready_precheck(articulation_state=state)

    assert report["status"] == "NOT_READY"
    assert "arm_joint_positions_within_limits" in report["missing_requirements"]
    assert "E_JOINT_LIMIT_VIOLATION" in report["blocking_reasons"]
    assert report["joint_precheck_table"][0]["control_target_generated"] is False
    assert_no_control(report)


def test_extra_non_arm_joints_do_not_fail():
    report = ready_precheck()

    assert report["status"] == "READY_FOR_SIMULATION_MOTION"
    assert report["extra_joint_names"] == ["robot_gripper_joint", "root_joint"]
    assert report["non_arm_extra_joints"] == ["robot_gripper_joint", "root_joint"]
