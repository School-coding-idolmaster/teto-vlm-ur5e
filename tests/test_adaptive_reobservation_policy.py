import pytest

from src.adaptive_reobservation_policy import (
    ABORTED,
    INITIAL_GROUNDING,
    STABLE_SUBGOAL_EXECUTION,
    AdaptiveReobservationPolicyRequest,
    E_MEMORY_TARGET_INCONSISTENT,
    E_SNAPSHOT_EXPIRED,
    E_TARGET_MOVED,
    evaluate_adaptive_reobservation_policy,
)
from src.memory_guided_execution import (
    E_CAMERA_MONITOR_UNAVAILABLE,
    E_POSITION_ERROR_TOO_LARGE,
    E_REPEATED_SUBGOAL_FAILURE,
    E_SCENE_STALE,
    E_TARGET_LOST,
    build_working_memory,
)


def _memory(goal_type="relative_motion"):
    return build_working_memory(
        task_goal="move up 0.05 meters",
        goal_type=goal_type,
        target_delta_m=[0.0, 0.0, 0.05],
        latest_verified_tcp_m=[0.4, 0.0, 0.4],
        motion_mode="single_axis_relative",
    )


def _monitor(**overrides):
    result = {
        "monitor_type": "mock",
        "camera_check_status": "PASS",
        "frequency_mode": "normal",
        "target_visible": True,
        "target_moved": False,
        "depth_valid": True,
        "tf_valid": True,
        "unexpected_obstacle": False,
        "scene_freshness_status": "fresh",
    }
    result.update(overrides)
    return result


def _request(**overrides):
    values = {
        "working_memory": _memory(),
        "current_execution_phase": STABLE_SUBGOAL_EXECUTION,
        "subgoal_index": 1,
        "position_error_m": 0.001,
        "position_error_limit_m": 0.008,
        "direction_check_passed": True,
        "subgoal_failure_count": 0,
        "scene_monitor_result": _monitor(),
        "target_task_type": "relative_motion",
        "camera_monitor_available": True,
        "stable_substep_count": 2,
    }
    values.update(overrides)
    return AdaptiveReobservationPolicyRequest(**values)


def test_initial_grounding_allows_semantic_models():
    result = evaluate_adaptive_reobservation_policy(
        _request(
            current_execution_phase=INITIAL_GROUNDING,
            working_memory=_memory("move_to_object"),
            target_task_type="move_to_object",
        )
    )

    assert result["execution_load_mode"] == "full_observation"
    assert result["llm_call_policy"] == "allowed_on_event"
    assert result["vlm_call_policy"] == "required_now"
    assert result["load_reduction_active"] is False


def test_stable_execution_suppresses_models_and_uses_reduced_monitoring():
    result = evaluate_adaptive_reobservation_policy(_request())

    assert result["policy_status"] == "PASS"
    assert result["execution_load_mode"] == "lightweight_monitor"
    assert result["llm_call_policy"] == "suppressed"
    assert result["vlm_call_policy"] == "monitor_only"
    assert result["camera_monitor_frequency_mode"] == "reduced"
    assert result["reobserve_required"] is False
    assert result["load_reduction_active"] is True
    assert result["llm_reobserve_called"] is False
    assert result["vlm_reobserve_called"] is False


def test_first_stable_substep_keeps_normal_monitor_frequency_before_reduction():
    result = evaluate_adaptive_reobservation_policy(
        _request(stable_substep_count=0)
    )

    assert result["execution_load_mode"] == "lightweight_monitor"
    assert result["camera_monitor_frequency_mode"] == "normal"
    assert result["llm_call_policy"] == "suppressed"
    assert result["vlm_call_policy"] == "monitor_only"


def test_stable_relative_motion_without_camera_is_warn_only():
    result = evaluate_adaptive_reobservation_policy(
        _request(
            scene_monitor_result={
                "monitor_type": "none",
                "camera_check_status": "NOT_AVAILABLE",
            },
            camera_monitor_available=False,
        )
    )

    assert result["policy_status"] == "WARN"
    assert result["execution_load_mode"] == "lightweight_monitor"
    assert result["camera_monitor_frequency_mode"] == "unavailable"
    assert result["reobserve_required"] is False
    assert E_CAMERA_MONITOR_UNAVAILABLE in result["warnings"]


def test_object_target_without_camera_requires_reobservation():
    result = evaluate_adaptive_reobservation_policy(
        _request(
            working_memory=_memory("move_to_object"),
            target_task_type="move_to_object",
            scene_monitor_result={
                "monitor_type": "none",
                "camera_check_status": "NOT_AVAILABLE",
            },
            camera_monitor_available=False,
        )
    )

    assert result["policy_status"] == "BLOCKED"
    assert result["execution_load_mode"] == "recovery_reobserve"
    assert result["reobserve_reason"] == E_CAMERA_MONITOR_UNAVAILABLE
    assert result["llm_call_policy"] == "required_now"
    assert result["vlm_call_policy"] == "required_now"


@pytest.mark.parametrize(
    ("overrides", "reason"),
    [
        ({"position_error_m": 0.02}, E_POSITION_ERROR_TOO_LARGE),
        (
            {
                "scene_monitor_result": _monitor(
                    camera_check_status="WARN",
                    scene_freshness_status="stale",
                )
            },
            E_SCENE_STALE,
        ),
        (
            {
                "working_memory": _memory("move_to_object"),
                "target_task_type": "move_to_object",
                "scene_monitor_result": _monitor(
                    camera_check_status="FAIL",
                    target_visible=False,
                ),
            },
            E_TARGET_LOST,
        ),
        ({"scene_monitor_result": _monitor(target_moved=True)}, E_TARGET_MOVED),
        ({"memory_target_consistent": False}, E_MEMORY_TARGET_INCONSISTENT),
        ({"snapshot_expired": True}, E_SNAPSHOT_EXPIRED),
    ],
)
def test_anomalies_restore_full_reobservation(overrides, reason):
    result = evaluate_adaptive_reobservation_policy(_request(**overrides))

    assert result["policy_status"] == "BLOCKED"
    assert result["execution_load_mode"] == "recovery_reobserve"
    assert result["camera_monitor_frequency_mode"] == "elevated"
    assert result["reobserve_required"] is True
    assert reason in result["trigger_reasons"]
    assert result["llm_call_policy"] == "required_now"
    assert result["vlm_call_policy"] == "required_now"
    assert result["load_reduction_active"] is False
    assert result["llm_reobserve_called"] is False
    assert result["vlm_reobserve_called"] is False


def test_repeated_failure_aborts_fail_closed():
    result = evaluate_adaptive_reobservation_policy(
        _request(subgoal_failure_count=2, subgoal_failed=True)
    )

    assert result["execution_phase"] == ABORTED
    assert result["policy_status"] == "BLOCKED"
    assert result["execution_load_mode"] == "abort"
    assert result["abort_required"] is True
    assert E_REPEATED_SUBGOAL_FAILURE in result["trigger_reasons"]
