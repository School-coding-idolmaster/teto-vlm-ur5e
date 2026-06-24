import pytest

from src.real_ur5_hover_executor import (
    E_EMERGENCY_STOP_ACTIVE,
    E_LOW_CONFIDENCE,
    E_MANUAL_CONFIRMATION_REQUIRED,
    E_MOVEIT_EXECUTE_NOT_ALLOWED,
    E_PROTECTIVE_STOP_ACTIVE,
    E_REAL_MOTION_NOT_ENABLED,
    E_SCENE_STALE,
    E_TARGET_DEPTH_INVALID,
    E_TF_UNAVAILABLE,
    E_WORKSPACE_VIOLATION,
    RealUR5HoverExecutionRequest,
    evaluate_real_ur5_hover_execution,
)


pytestmark = [pytest.mark.safety, pytest.mark.real_path]


def test_real_motion_disabled_by_default_blocks():
    result = evaluate_real_ur5_hover_execution(RealUR5HoverExecutionRequest())

    assert result["real_robot_motion_executed"] is False
    assert E_REAL_MOTION_NOT_ENABLED in result["blocking_reasons"]


def test_enable_real_robot_motion_without_moveit_execute_blocks():
    config = _enabled_config()
    config["enable_moveit_execute"] = False

    result = _evaluate(config)

    assert result["real_robot_motion_executed"] is False
    assert E_MOVEIT_EXECUTE_NOT_ALLOWED in result["blocking_reasons"]


def test_enable_real_robot_motion_without_manual_confirmation_blocks():
    result = evaluate_real_ur5_hover_execution(
        RealUR5HoverExecutionRequest(
            config=_enabled_config(),
            planner_gateway_result=_planner(),
            moveit_plan_result={"plan_only_ready": True},
            ur5_state_result=_ur5_state(),
            manual_confirmation_result={"manual_confirmation_accepted": False},
        )
    )

    assert result["real_robot_motion_executed"] is False
    assert E_MANUAL_CONFIRMATION_REQUIRED in result["blocking_reasons"]


def test_protective_stop_and_emergency_stop_block():
    for flag, reason in (("protective_stop", E_PROTECTIVE_STOP_ACTIVE), ("emergency_stop", E_EMERGENCY_STOP_ACTIVE)):
        config = _enabled_config()
        config[flag] = True

        result = _evaluate(config)

        assert result["real_robot_motion_executed"] is False
        assert reason in result["blocking_reasons"]


def test_low_confidence_scene_tf_depth_and_workspace_block():
    cases = [
        ({"confidence_overall": 0.1}, E_LOW_CONFIDENCE),
        ({"scene_ttl_valid": False}, E_SCENE_STALE),
        ({"tf_available": False}, E_TF_UNAVAILABLE),
        ({"target_depth_valid": False}, E_TARGET_DEPTH_INVALID),
        ({"bounded_target_point_m": [3.0, 0.0, 0.1]}, E_WORKSPACE_VIOLATION),
    ]
    for override, reason in cases:
        config = _enabled_config()
        config.update(override)

        result = _evaluate(config)

        assert result["real_robot_motion_executed"] is False
        assert reason in result["blocking_reasons"]


def test_positive_confirmed_request_executes_low_speed_hover():
    result = _evaluate(_enabled_config())

    assert result["real_ur5_hover_executor_status"] == "PASS"
    assert result["trajectory_send_allowed"] is True
    assert result["controller_command_sent"] is True
    assert result["real_robot_motion_executed"] is True
    assert result["urscript_generated"] is False
    assert result["rtde_write_attempted"] is False
    assert result["dashboard_command_attempted"] is False


def _evaluate(config):
    return evaluate_real_ur5_hover_execution(
        RealUR5HoverExecutionRequest(
            config=config,
            planner_gateway_result=_planner(),
            moveit_plan_result={"plan_only_ready": True},
            ur5_state_result=_ur5_state(),
            manual_confirmation_result={"manual_confirmation_accepted": True},
        )
    )


def _enabled_config():
    return {
        "enable_ros2_runtime": True,
        "enable_live_camera": True,
        "enable_live_vlm": True,
        "enable_moveit_plan": True,
        "enable_moveit_execute": True,
        "enable_real_robot_motion": True,
        "manual_confirmation_required": True,
        "ros2_runtime_available": True,
        "moveit_runtime_available": True,
        "moveit_plan_success": True,
        "moveit_execute_allowed": True,
        "robot_state_ok": True,
        "safety_status_ok": True,
        "protective_stop": False,
        "emergency_stop": False,
        "speed_scaling": 0.05,
        "workspace_check_passed": True,
        "target_depth_valid": True,
        "tf_available": True,
        "scene_ttl_valid": True,
        "confidence_overall": 0.9,
        "confidence_threshold": 0.7,
        "bounded_target_point_m": [0.1, 0.1, 0.2],
        "workspace_bounds": {"x": [-1.0, 1.0], "y": [-1.0, 1.0], "z": [0.0, 1.0]},
    }


def _planner():
    return {"workspace_check_passed": True, "ttl_check_passed": True, "bounded_target_point_m": [0.1, 0.1, 0.2]}


def _ur5_state():
    return {"read_only_state_contract_ready": True}
