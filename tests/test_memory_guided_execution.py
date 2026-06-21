import pytest

from src.memory_guided_execution import (
    E_CAMERA_MONITOR_UNAVAILABLE,
    E_DEPTH_INVALID,
    E_DIRECTION_CHECK_FAILED,
    E_POSITION_ERROR_TOO_LARGE,
    E_REPEATED_SUBGOAL_FAILURE,
    E_SCENE_STALE,
    E_TARGET_LOST,
    E_TF_STALE,
    E_UNEXPECTED_OBSTACLE,
    ReobservationPolicyRequest,
    build_working_memory,
    evaluate_event_triggered_reobservation,
    make_scene_monitor_result,
    update_working_memory,
)


def _memory(goal_type="relative_motion"):
    return build_working_memory(
        task_goal="move up 0.05 meters",
        goal_type=goal_type,
        target_delta_m=[0.0, 0.0, 0.05],
        latest_verified_tcp_m=[0.4, 0.0, 0.4],
    )


def _request(**overrides):
    values = {
        "working_memory": _memory(),
        "latest_measured_tcp_m": [0.4, 0.0, 0.42],
        "subgoal_target_tcp_m": [0.4, 0.0, 0.42],
        "position_error_m": 0.0,
        "position_error_limit_m": 0.008,
        "direction_check_passed": True,
        "scene_monitor_result": make_scene_monitor_result(),
    }
    values.update(overrides)
    return ReobservationPolicyRequest(**values)


def test_working_memory_tracks_verified_progress_and_remaining_delta():
    before = _memory()
    policy = evaluate_event_triggered_reobservation(_request(working_memory=before))
    after = update_working_memory(
        before,
        latest_verified_tcp_m=[0.4, 0.0, 0.42],
        measured_total_delta_m=[0.0, 0.0, 0.02],
        completed_substeps=1,
        last_error_m=0.001,
        scene_monitor_result={
            "monitor_type": "mock",
            "camera_check_status": "PASS",
            "scene_snapshot_id": "scene-1",
            "scene_freshness_status": "fresh",
        },
        reobservation_policy_result=policy,
    )

    assert before["working_memory_version"] == "teto_memory_guided_execution.v1"
    assert after["latest_verified_tcp_m"] == [0.4, 0.0, 0.42]
    assert after["remaining_delta_m"] == [0.0, 0.0, 0.03]
    assert after["completed_substeps"] == 1
    assert after["last_error_m"] == 0.001
    assert after["scene_snapshot_id"] == "scene-1"
    assert after["scene_freshness_status"] == "fresh"
    assert after["reobserve_required"] is False


def test_relative_motion_without_camera_can_continue_in_warn_only_mode():
    result = evaluate_event_triggered_reobservation(_request())

    assert result["policy_status"] == "WARN"
    assert result["continue_allowed"] is True
    assert result["reobserve_required"] is False
    assert E_CAMERA_MONITOR_UNAVAILABLE in result["warnings"]
    assert result["vlm_reobserve_called"] is False
    assert result["llm_reobserve_called"] is False


def test_object_target_without_monitor_does_not_pretend_pass():
    result = evaluate_event_triggered_reobservation(
        _request(working_memory=_memory("move_to_object"))
    )

    assert result["policy_status"] == "REOBSERVE_REQUIRED"
    assert result["continue_allowed"] is False
    assert result["reobserve_required"] is True
    assert result["reobserve_reason"] == E_CAMERA_MONITOR_UNAVAILABLE


def test_relative_motion_can_fail_closed_when_camera_policy_is_block():
    result = evaluate_event_triggered_reobservation(
        _request(camera_unavailable_policy="block")
    )

    assert result["continue_allowed"] is False
    assert result["reobserve_required"] is True
    assert result["reobserve_reason"] == E_CAMERA_MONITOR_UNAVAILABLE


@pytest.mark.parametrize(
    ("overrides", "reason"),
    [
        ({"position_error_m": 0.02}, E_POSITION_ERROR_TOO_LARGE),
        ({"direction_check_passed": False}, E_DIRECTION_CHECK_FAILED),
        (
            {
                "scene_monitor_result": {
                    "monitor_type": "mock",
                    "camera_check_status": "FAIL",
                    "target_visible": False,
                }
            },
            E_TARGET_LOST,
        ),
        (
            {
                "scene_monitor_result": {
                    "monitor_type": "mock",
                    "camera_check_status": "WARN",
                    "scene_freshness_status": "stale",
                }
            },
            E_SCENE_STALE,
        ),
        (
            {
                "scene_monitor_result": {
                    "monitor_type": "mock",
                    "camera_check_status": "FAIL",
                    "depth_valid": False,
                }
            },
            E_DEPTH_INVALID,
        ),
        (
            {
                "scene_monitor_result": {
                    "monitor_type": "mock",
                    "camera_check_status": "FAIL",
                    "tf_fresh": False,
                }
            },
            E_TF_STALE,
        ),
    ],
)
def test_anomalies_trigger_reobservation_without_calling_models(overrides, reason):
    result = evaluate_event_triggered_reobservation(_request(**overrides))

    assert result["continue_allowed"] is False
    assert result["reobserve_required"] is True
    assert reason in result["trigger_reasons"]
    assert result["vlm_reobserve_called"] is False
    assert result["llm_reobserve_called"] is False


@pytest.mark.parametrize(
    ("overrides", "reason"),
    [
        (
            {
                "scene_monitor_result": {
                    "monitor_type": "mock",
                    "camera_check_status": "FAIL",
                    "unexpected_obstacle": True,
                }
            },
            E_UNEXPECTED_OBSTACLE,
        ),
        ({"subgoal_failure_count": 2}, E_REPEATED_SUBGOAL_FAILURE),
    ],
)
def test_hard_anomalies_require_abort(overrides, reason):
    result = evaluate_event_triggered_reobservation(_request(**overrides))

    assert result["policy_status"] == "ABORT_REQUIRED"
    assert result["abort_required"] is True
    assert reason in result["trigger_reasons"]
