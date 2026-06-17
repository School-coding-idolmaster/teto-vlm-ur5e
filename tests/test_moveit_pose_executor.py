from src.moveit_pose_executor import (
    E_CURRENT_TCP_POSE_MISSING,
    E_EXCESSIVE_CARTESIAN_MOTION,
    E_MANUAL_CONFIRMATION_REQUIRED,
    E_MOVEIT_PLAN_FAILED,
    MoveItPoseExecutorRequest,
    evaluate_moveit_pose_execute,
    evaluate_moveit_pose_plan,
)


def test_plan_blocks_without_current_tcp_pose_before_calling_moveit(monkeypatch):
    called = False

    def fake_plan(_target_pose, _config):
        nonlocal called
        called = True
        return {"success": True}

    monkeypatch.setattr("src.moveit_pose_executor._plan_with_move_group_action", fake_plan)

    result = evaluate_moveit_pose_plan(
        MoveItPoseExecutorRequest(
            requested=True,
            target_pose=_pose([0.40, 0.0, 0.32]),
            config=_base_config(),
        )
    )

    assert result["moveit_pose_executor_status"] == "BLOCKED"
    assert E_CURRENT_TCP_POSE_MISSING in result["blocking_reasons"]
    assert result["moveit_plan_called"] is False
    assert called is False


def test_plan_success_comes_from_move_group_action_result(monkeypatch):
    def fake_plan(target_pose, config):
        assert target_pose["position_m"] == [0.4, 0.0, 0.32]
        assert config["planning_group"] == "ur_manipulator"
        return {
            "action_call_attempted": True,
            "action_server_available": True,
            "goal_accepted": True,
            "success": True,
            "error_code": 1,
            "error_code_name": "SUCCESS",
            "planning_time_s": 0.42,
            "trajectory_point_count": 8,
        }

    monkeypatch.setattr("src.moveit_pose_executor._plan_with_move_group_action", fake_plan)

    result = evaluate_moveit_pose_plan(
        MoveItPoseExecutorRequest(
            requested=True,
            target_pose=_pose([0.40, 0.0, 0.32]),
            current_tcp_pose=_pose([0.40, 0.0, 0.30]),
            config=_base_config(),
        )
    )

    assert result["moveit_pose_executor_status"] == "PASS"
    assert result["plan_success"] is True
    assert result["moveit_plan_called"] is True
    assert result["plan_success_source"] == "actual_moveit_action_result"
    assert result["trajectory_point_count"] == 8


def test_plan_records_planner_start_state_and_joint_delta_audit(monkeypatch):
    def fake_plan(_target_pose, _config):
        return {
            "action_call_attempted": True,
            "action_server_available": True,
            "goal_accepted": True,
            "success": True,
            "error_code": 1,
            "error_code_name": "SUCCESS",
            "planning_time_s": 0.42,
            "trajectory_point_count": 2,
            "planned_joint_names": [
                "shoulder_pan_joint",
                "shoulder_lift_joint",
                "elbow_joint",
                "wrist_1_joint",
                "wrist_2_joint",
                "wrist_3_joint",
            ],
            "joint_trajectory_points": [
                {"positions": [0.0, -1.0, 1.2, 0.1, 0.2, 0.3]},
                {"positions": [0.01, -0.98, 1.23, 0.35, -1.05, 0.28]},
            ],
        }

    monkeypatch.setattr("src.moveit_pose_executor._plan_with_move_group_action", fake_plan)

    result = evaluate_moveit_pose_plan(
        MoveItPoseExecutorRequest(
            requested=True,
            target_pose=_pose([0.40, 0.0, 0.32]),
            current_tcp_pose=_pose([0.40, 0.0, 0.30]),
            config={**_base_config(), "pipeline_id": "move_group", "planner_id": "ur_manipulator[RRTConnectkConfigDefault]"},
        )
    )

    assert result["moveit_pose_executor_status"] == "PASS"
    assert result["planner_mode"] == "joint_space_pose_goal"
    assert result["planning_pipeline_id"] == "move_group"
    assert result["planner_id"] == "ur_manipulator[RRTConnectkConfigDefault]"
    assert result["moveit_goal_type"] == "move_group_pose_goal_constraints"
    assert result["joint_space_pose_goal_used"] is True
    assert result["cartesian_path_used"] is False
    assert result["cartesian_path_fraction"] is None
    assert result["joint_space_fallback_used"] is False
    assert result["start_state_source"] == "implicit_planning_scene"
    assert result["start_state_is_diff"] is True
    assert result["explicit_start_state_provided"] is False
    assert result["current_joint_state_available"] is False
    assert result["target_orientation_source"] == "copied_from_current_tcp_pose"
    assert result["orientation_mode"] == "keep_current_orientation"
    assert result["orientation_locked"] is True
    assert result["planned_joint_names"] == [
        "shoulder_pan_joint",
        "shoulder_lift_joint",
        "elbow_joint",
        "wrist_1_joint",
        "wrist_2_joint",
        "wrist_3_joint",
    ]
    assert result["planned_start_joint_positions"] == [0.0, -1.0, 1.2, 0.1, 0.2, 0.3]
    assert result["planned_final_joint_positions"] == [0.01, -0.98, 1.23, 0.35, -1.05, 0.28]
    assert result["per_joint_delta_rad"]["wrist_1_joint"] == 0.25
    assert result["per_joint_delta_rad"]["wrist_2_joint"] == -1.25
    assert result["max_joint_delta_rad"] == 1.25
    assert result["wrist_joint_names"] == ["wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
    assert result["wrist_joint_delta_rad"] == {
        "wrist_1_joint": 0.25,
        "wrist_2_joint": -1.25,
        "wrist_3_joint": -0.02,
    }
    assert result["max_wrist_joint_delta_rad"] == 1.25
    assert result["joint_delta_audit_status"] == "AVAILABLE"
    assert result["joint_wrap_suspected"] is False
    assert result["planned_waypoint_count"] == 2
    assert result["planned_joint_path_length_rad"] == 1.58
    assert result["path_metric_source"] == "joint_trajectory"
    assert "W_SUSPICIOUS_WRIST_JOINT_DELTA_FOR_CARTESIAN_STEP" in result["planner_audit_warnings"]
    assert "W_SUSPICIOUS_WRIST_JOINT_DELTA_FOR_CARTESIAN_STEP" not in result["warnings"]


def test_plan_allows_exact_relative_max_translation_from_nonzero_tcp_pose(monkeypatch):
    current_position = [-0.153217, 0.315916, 1.046994]
    target_position = [-0.153217, 0.315916, 1.051994]

    monkeypatch.setattr(
        "src.moveit_pose_executor._plan_with_move_group_action",
        lambda _target_pose, _config: {
            "action_call_attempted": True,
            "action_server_available": True,
            "goal_accepted": True,
            "success": True,
            "error_code": 1,
            "error_code_name": "SUCCESS",
            "planning_time_s": 0.1,
            "trajectory_point_count": 3,
        },
    )

    result = evaluate_moveit_pose_plan(
        MoveItPoseExecutorRequest(
            requested=True,
            target_pose=_pose(target_position),
            current_tcp_pose=_pose(current_position),
            config={**_base_config(), "max_translation_m": 0.005, "hard_safety_limit_m": 0.01},
        )
    )

    assert result["moveit_pose_executor_status"] == "PASS"
    assert E_EXCESSIVE_CARTESIAN_MOTION not in result["blocking_reasons"]
    assert result["translation_distance_m"] == 0.005
    assert result["motion_check_source"] == "moveit_pose_executor"
    assert result["motion_check_current_position_m"] == current_position
    assert result["motion_check_target_position_m"] == target_position
    assert result["motion_check_distance_m"] == 0.005
    assert result["motion_check_max_distance_m"] == 0.005
    assert result["motion_check_hard_limit_m"] == 0.01
    assert result["motion_check_eps"] == 1e-9


def test_real_motion_policy_tightens_two_mm_tolerance(monkeypatch):
    monkeypatch.setattr(
        "src.moveit_pose_executor._plan_with_move_group_action",
        lambda _target_pose, _config: {
            "action_call_attempted": True,
            "action_server_available": True,
            "goal_accepted": True,
            "success": True,
            "error_code": 1,
            "error_code_name": "SUCCESS",
            "planning_time_s": 0.1,
            "trajectory_point_count": 2,
        },
    )

    result = evaluate_moveit_pose_plan(
        MoveItPoseExecutorRequest(
            requested=True,
            target_pose=_pose([0.40, 0.0, 0.302]),
            current_tcp_pose=_pose([0.40, 0.0, 0.30]),
            config={
                **_base_config(),
                "requested_distance_m": 0.002,
                "configured_max_distance_m": 0.05,
                "safety_policy_source": "cli_defaults",
                "safety_policy_name": "lab_directional_step_motion_v1",
                "position_tolerance_m": 0.002,
                "orientation_tolerance_rad": 0.01,
                "small_motion_tolerance_policy": "real_motion_safety_policy_v1",
            },
        )
    )

    assert result["moveit_pose_executor_status"] == "PASS"
    assert result["requested_distance_m"] == 0.002
    assert result["moveit_position_tolerance_m"] <= 0.0005
    assert result["moveit_position_tolerance_m"] == 0.0002
    assert result["moveit_position_tolerance_m"] < result["requested_distance_m"]
    assert result["moveit_orientation_tolerance_rad"] == 0.01
    assert result["tolerance_to_requested_distance_ratio"] == 0.1
    assert "real_motion_safety_policy_v1" in result["small_motion_tolerance_policy"]
    assert result["configured_max_distance_m"] == 0.05
    assert result["requested_distance_within_configured_limit"] is True
    assert result["safety_policy_source"] == "cli_defaults"
    assert result["safety_policy_name"] == "lab_directional_step_motion_v1"
    assert result["target_frame"] == "base_link"
    assert result["current_tcp_frame"] == "base_link"
    assert result["moveit_end_effector_link"] == "tool0"
    assert result["moveit_planning_frame"] == "base_link"
    assert result["moveit_group_name"] == "ur_manipulator"


def test_real_motion_policy_scales_five_cm_tolerance(monkeypatch):
    monkeypatch.setattr(
        "src.moveit_pose_executor._plan_with_move_group_action",
        lambda _target_pose, _config: {
            "action_call_attempted": True,
            "action_server_available": True,
            "goal_accepted": True,
            "success": True,
            "error_code": 1,
            "error_code_name": "SUCCESS",
            "planning_time_s": 0.1,
            "trajectory_point_count": 2,
        },
    )

    result = evaluate_moveit_pose_plan(
        MoveItPoseExecutorRequest(
            requested=True,
            target_pose=_pose([0.40, 0.0, 0.25]),
            current_tcp_pose=_pose([0.40, 0.0, 0.30]),
            config={
                **_base_config(),
                "max_translation_m": 0.05,
                "hard_safety_limit_m": 0.05,
                "requested_distance_m": 0.05,
                "configured_max_distance_m": 0.05,
                "position_tolerance_m": 0.002,
                "orientation_tolerance_rad": 0.01,
                "small_motion_tolerance_policy": "real_motion_safety_policy_v1",
            },
        )
    )

    assert result["moveit_pose_executor_status"] == "PASS"
    assert result["requested_distance_m"] == 0.05
    assert result["configured_max_distance_m"] == 0.05
    assert result["requested_distance_within_configured_limit"] is True
    assert result["moveit_position_tolerance_m"] == 0.002
    assert result["moveit_position_tolerance_m"] <= 0.005
    assert result["moveit_orientation_tolerance_rad"] == 0.01
    assert result["tolerance_to_requested_distance_ratio"] == 0.04


def test_plan_blocks_relative_motion_above_hard_safety_limit_before_moveit(monkeypatch):
    current_position = [-0.153217, 0.315916, 1.046994]
    target_position = [-0.153217, 0.315916, 1.066994]
    called = False

    def fake_plan(_target_pose, _config):
        nonlocal called
        called = True
        return {"success": True}

    monkeypatch.setattr("src.moveit_pose_executor._plan_with_move_group_action", fake_plan)

    result = evaluate_moveit_pose_plan(
        MoveItPoseExecutorRequest(
            requested=True,
            target_pose=_pose(target_position),
            current_tcp_pose=_pose(current_position),
            config={**_base_config(), "max_translation_m": 0.005, "hard_safety_limit_m": 0.01},
        )
    )

    assert result["moveit_pose_executor_status"] == "BLOCKED"
    assert E_EXCESSIVE_CARTESIAN_MOTION in result["blocking_reasons"]
    assert result["moveit_plan_called"] is False
    assert called is False
    assert result["motion_check_distance_m"] == 0.02


def test_plan_allows_exact_hard_limit_when_max_allows_it(monkeypatch):
    monkeypatch.setattr(
        "src.moveit_pose_executor._plan_with_move_group_action",
        lambda _target_pose, _config: {
            "action_call_attempted": True,
            "action_server_available": True,
            "goal_accepted": True,
            "success": True,
            "error_code": 1,
            "error_code_name": "SUCCESS",
            "planning_time_s": 0.1,
            "trajectory_point_count": 3,
        },
    )

    result = evaluate_moveit_pose_plan(
        MoveItPoseExecutorRequest(
            requested=True,
            target_pose=_pose([-0.153217, 0.315916, 1.056994]),
            current_tcp_pose=_pose([-0.153217, 0.315916, 1.046994]),
            config={**_base_config(), "max_translation_m": 0.01, "hard_safety_limit_m": 0.01},
        )
    )

    assert result["moveit_pose_executor_status"] == "PASS"
    assert E_EXCESSIVE_CARTESIAN_MOTION not in result["blocking_reasons"]
    assert result["motion_check_distance_m"] == 0.01


def test_plan_failure_blocks_on_moveit_error(monkeypatch):
    monkeypatch.setattr(
        "src.moveit_pose_executor._plan_with_move_group_action",
        lambda _target_pose, _config: {
            "action_call_attempted": True,
            "action_server_available": True,
            "goal_accepted": True,
            "success": False,
            "error_code": -1,
            "error_code_name": "PLANNING_FAILED",
            "trajectory_point_count": 0,
        },
    )

    result = evaluate_moveit_pose_plan(
        MoveItPoseExecutorRequest(
            requested=True,
            target_pose=_pose([0.40, 0.0, 0.32]),
            current_tcp_pose=_pose([0.40, 0.0, 0.30]),
            config=_base_config(),
        )
    )

    assert result["moveit_pose_executor_status"] == "BLOCKED"
    assert result["plan_success"] is False
    assert E_MOVEIT_PLAN_FAILED in result["blocking_reasons"]


def test_execute_requires_manual_confirmation_before_action_calls(monkeypatch):
    plan_called = False

    def fake_plan(_target_pose, _config):
        nonlocal plan_called
        plan_called = True
        return {"success": True}

    monkeypatch.setattr("src.moveit_pose_executor._plan_with_move_group_action", fake_plan)

    result = evaluate_moveit_pose_execute(
        MoveItPoseExecutorRequest(
            requested=True,
            target_pose=_pose([0.40, 0.0, 0.32]),
            current_tcp_pose=_pose([0.40, 0.0, 0.30]),
            config={**_base_config(), **_robot_state_config()},
            manual_confirmation_result={"manual_confirmation_accepted": False},
            robot_state_result={"read_only_state_contract_ready": True},
        )
    )

    assert result["moveit_pose_executor_status"] == "BLOCKED"
    assert E_MANUAL_CONFIRMATION_REQUIRED in result["blocking_reasons"]
    assert result["moveit_plan_called"] is False
    assert result["moveit_execute_called"] is False
    assert plan_called is False


def test_execute_success_requires_execute_trajectory_success(monkeypatch):
    monkeypatch.setattr(
        "src.moveit_pose_executor._plan_with_move_group_action",
        lambda _target_pose, _config: {
            "action_call_attempted": True,
            "action_server_available": True,
            "goal_accepted": True,
            "success": True,
            "error_code": 1,
            "error_code_name": "SUCCESS",
            "planning_time_s": 0.2,
            "trajectory_point_count": 4,
            "planned_trajectory": object(),
        },
    )
    monkeypatch.setattr(
        "src.moveit_pose_executor._execute_trajectory_action",
        lambda _trajectory, _config: {
            "action_call_attempted": True,
            "action_server_available": True,
            "goal_accepted": True,
            "success": True,
            "error_code": 1,
            "error_code_name": "SUCCESS",
        },
    )

    result = evaluate_moveit_pose_execute(
        MoveItPoseExecutorRequest(
            requested=True,
            target_pose=_pose([0.40, 0.0, 0.32]),
            current_tcp_pose=_pose([0.40, 0.0, 0.30]),
            config={**_base_config(), **_robot_state_config()},
            manual_confirmation_result={"manual_confirmation_accepted": True},
            robot_state_result={"read_only_state_contract_ready": True},
        )
    )

    assert result["moveit_pose_executor_status"] == "PASS"
    assert result["plan_success"] is True
    assert result["execute_success"] is True
    assert result["real_robot_motion_executed"] is True
    assert result["execute_success_source"] == "actual_execute_trajectory_action_result"


def _pose(position):
    return {
        "frame": "base_link",
        "position_m": list(position),
        "orientation_xyzw": [0.0, 0.0, 0.0, 1.0],
    }


def _base_config():
    return {
        "planning_group": "ur_manipulator",
        "end_effector_link": "tool0",
        "allowed_frames": ["base_link"],
        "max_translation_m": 0.20,
        "workspace_bounds": {"x": [-1.0, 1.0], "y": [-1.0, 1.0], "z": [0.0, 2.0]},
    }


def _robot_state_config():
    return {
        "manual_confirmation_required": True,
        "robot_state_ok": True,
        "safety_status_ok": True,
        "protective_stop": False,
        "emergency_stop": False,
        "speed_scaling": 0.05,
        "max_speed_scale": 0.10,
        "max_acc_scale": 0.10,
    }
