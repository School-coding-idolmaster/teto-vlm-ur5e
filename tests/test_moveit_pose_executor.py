from src.moveit_pose_executor import (
    E_CURRENT_TCP_POSE_MISSING,
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
