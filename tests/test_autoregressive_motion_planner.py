from src.autoregressive_motion_planner import (
    AutoregressiveMotionPlannerRequest,
    PLANNER_VERSION,
    plan_offline_autoregressive_motion,
)
from src.motion_command_normalizer import normalize_motion_command
from scripts import run_long_motion_autoregressive_preview as preview_cli


def test_ten_cm_preview_generates_five_sequential_targets():
    result = _plan(distance=0.10)

    assert result["autoregressive_motion_planner_version"] == PLANNER_VERSION
    assert result["final_plan_status"] == "PASS"
    assert result["substep_count"] == 5
    assert [step["current_tcp_pose_m"][0] for step in result["substeps"]] == [
        0.40,
        0.42,
        0.44,
        0.46,
        0.48,
    ]
    assert [step["target_tcp_pose_m"][0] for step in result["substeps"]] == [
        0.42,
        0.44,
        0.46,
        0.48,
        0.50,
    ]
    assert result["substeps"][0]["target_generated_from"] == "current_tcp_pose"
    assert all(
        step["target_generated_from"] == "simulated_latest_verified_tcp_pose"
        for step in result["substeps"][1:]
    )
    _assert_no_execution(result)


def test_twenty_cm_preview_generates_ten_substeps_at_expanded_boundary():
    result = _plan(distance=0.20)

    assert result["final_plan_status"] == "PASS"
    assert result["substep_count"] == 10
    assert result["decomposed_total_distance_m"] == 0.20
    assert result["substeps"][-1]["target_tcp_pose_m"][0] == 0.60
    _assert_no_execution(result)


def test_above_total_envelope_blocks_before_target_sequence():
    result = _plan(distance=0.25)

    assert result["final_plan_status"] == "BLOCKED"
    assert result["final_blocking_reason"] == "E_LONG_MOTION_TOTAL_EXCEEDS_LIMIT"
    assert result["substep_count"] == 0
    assert result["substeps"] == []
    _assert_no_execution(result)


def test_missing_current_tcp_pose_requires_readiness_without_target_generation():
    result = _plan(distance=0.10, current_tcp_pose=None)

    assert result["final_plan_status"] == "NEEDS_CURRENT_TCP"
    assert result["final_blocking_reason"] == "E_CURRENT_TCP_POSE_MISSING"
    assert result["substeps"][0]["current_tcp_pose_available"] is False
    assert result["substeps"][0]["target_pose_generation_status"] == "BLOCKED"
    assert result["substeps"][0]["target_tcp_pose_m"] is None
    _assert_no_execution(result)


def test_simulated_verification_failure_at_third_substep_aborts_remaining_plan():
    result = _plan(
        distance=0.10,
        config={"simulate_verification_failure_at_substep": 3},
    )

    assert result["final_plan_status"] == "ABORTED"
    assert result["final_abort_reason"] == "E_SIMULATED_POST_STEP_VERIFICATION_FAILED"
    assert len(result["substeps"]) == 3
    assert result["substeps"][2]["post_step_verification_status"] == "FAILED"
    assert result["substeps"][2]["continue_allowed"] is False
    _assert_no_execution(result)


def test_simulated_opposite_direction_aborts_and_records_direction_check():
    result = _plan(
        distance=0.10,
        config={"simulate_direction_mismatch_at_substep": 2},
    )

    assert result["final_plan_status"] == "ABORTED"
    assert result["final_abort_reason"] == "E_SIMULATED_POST_STEP_DIRECTION_MISMATCH"
    assert len(result["substeps"]) == 2
    assert result["substeps"][1]["direction_check_passed"] is False
    _assert_no_execution(result)


def test_planner_risk_warning_is_soft_evidence_by_default():
    result = _plan(
        distance=0.10,
        config={
            "planner_risk_status": "WARN",
            "planner_risk_warnings": ["W_PATH_LENGTH_RATIO_HIGH"],
        },
    )

    assert result["final_plan_status"] == "PASS"
    assert result["substeps"][0]["planner_risk_status"] == "WARN"
    assert result["substeps"][0]["planner_risk_warnings"] == ["W_PATH_LENGTH_RATIO_HIGH"]


def test_planner_risk_warning_aborts_only_when_blocking_is_explicitly_enabled():
    result = _plan(
        distance=0.10,
        config={
            "planner_risk_status": "WARN",
            "planner_risk_warnings": ["W_PATH_LENGTH_RATIO_HIGH"],
            "planner_risk_blocking_enabled": True,
        },
    )

    assert result["final_plan_status"] == "ABORTED"
    assert result["final_abort_reason"] == "E_PLANNER_RISK_BLOCKING_ENABLED"
    assert len(result["substeps"]) == 1
    _assert_no_execution(result)


def test_workspace_violation_aborts_at_the_first_violating_substep():
    result = _plan(
        distance=0.10,
        config={"workspace_bounds": {"x": [0.0, 0.45], "y": [-1.0, 1.0], "z": [0.0, 2.0]}},
    )

    assert result["final_plan_status"] == "ABORTED"
    assert result["final_abort_reason"] == "E_DECOMPOSED_WORKSPACE_ENVELOPE_EXCEEDED"
    assert len(result["substeps"]) == 3
    assert result["substeps"][2]["workspace_envelope_within_limit"] is False


def test_session_envelope_violation_aborts_sequential_preview():
    result = _plan(distance=0.10, config={"session_radius_limit_m": 0.05})

    assert result["final_plan_status"] == "ABORTED"
    assert result["final_abort_reason"] == "E_SESSION_RADIUS_EXCEEDS_LIMIT"
    assert len(result["substeps"]) == 3
    assert result["substeps"][2]["session_envelope_within_limit"] is False


def test_direction_axis_sign_conflict_is_invalid_request():
    result = plan_offline_autoregressive_motion(
        AutoregressiveMotionPlannerRequest(
            canonical_motion_intent={
                "parse_status": "PASS",
                "intent": "relative_cartesian_motion",
                "motion_frame": "base_link",
                "direction_axis": "x",
                "direction_sign": "-",
                "requested_distance_m": 0.10,
                "delta_m": [0.10, 0.0, 0.0],
            },
            current_tcp_pose=_pose(),
        )
    )

    assert result["final_plan_status"] == "INVALID_REQUEST"
    assert result["final_blocking_reason"] == "E_DIRECTION_SIGN_CONFLICTS_WITH_DELTA"


def test_v3_0_11_language_normalizes_before_planner_and_parser_never_permits_execution():
    canonical = normalize_motion_command("move forward 20 cm")
    result = plan_offline_autoregressive_motion(
        AutoregressiveMotionPlannerRequest(
            canonical_motion_intent=canonical,
            current_tcp_pose=_pose(),
        )
    )

    assert canonical["parse_status"] == "PASS"
    assert canonical["execution_permission_decided_by_parser"] is False
    assert result["substep_count"] == 10
    assert result["execution_permission_decided_by_parser"] is False
    assert result["safety_gate_still_required"] is True


def test_compound_motion_stays_clarification_and_is_not_planned_as_single_axis():
    canonical = normalize_motion_command("go up 5 cm and right 2 cm")
    result = plan_offline_autoregressive_motion(
        AutoregressiveMotionPlannerRequest(
            canonical_motion_intent=canonical,
            current_tcp_pose=_pose(),
        )
    )

    assert canonical["parse_status"] == "NEEDS_CLARIFICATION"
    assert result["final_plan_status"] == "INVALID_REQUEST"
    assert result["substeps"] == []


def test_contract_only_semantics_are_explicit_for_every_substep():
    result = _plan(distance=0.10, config={"substep_execution_mode": "contract_only"})

    assert result["substep_execution_mode"] == "contract_only"
    assert result["one_shot_target_pose_created"] is False
    assert result["moveit_plan_request_created"] is False
    assert result["real_substep_execution_enabled"] is False
    assert all(step["execution_status"] == "SKIPPED_CONTRACT_ONLY" for step in result["substeps"])
    _assert_no_execution(result)


def test_offline_preview_cli_writes_json_and_markdown_without_execution(tmp_path, capsys):
    exit_code = preview_cli.main(
        [
            "--parser",
            "rule",
            "--mock-current-tcp-pose",
            "--cmd",
            "move forward 10 cm",
            "--output-dir",
            str(tmp_path),
        ]
    )

    output = capsys.readouterr().out
    assert exit_code == 0
    assert '"final_plan_status": "PASS"' in output
    assert '"any_execution_attempted": false' in output
    assert len(list(tmp_path.glob("*.json"))) == 1
    assert len(list(tmp_path.glob("*.md"))) == 1


def _plan(
    *,
    distance,
    current_tcp_pose="default",
    config=None,
):
    merged_config = {
        "enable_long_step_decomposition": True,
        "max_one_shot_distance_m": 0.05,
        "max_decomposed_substep_distance_m": 0.02,
        "max_decomposed_total_distance_m": 0.20,
        "substep_execution_mode": "offline_preview",
        **(config or {}),
    }
    return plan_offline_autoregressive_motion(
        AutoregressiveMotionPlannerRequest(
            canonical_motion_intent={
                "parse_status": "PASS",
                "intent": "relative_cartesian_motion",
                "motion_frame": "base_link",
                "direction_axis": "x",
                "direction_sign": "+",
                "requested_distance_m": distance,
                "delta_m": [distance, 0.0, 0.0],
                "execution_permission_decided_by_parser": False,
                "safety_gate_still_required": True,
            },
            current_tcp_pose=_pose() if current_tcp_pose == "default" else current_tcp_pose,
            config=merged_config,
        )
    )


def _pose():
    return {
        "frame": "base_link",
        "position_m": [0.40, 0.0, 0.30],
        "orientation_xyzw": [0.0, 0.0, 0.0, 1.0],
    }


def _assert_no_execution(result):
    assert result["real_substep_execution_enabled"] is False
    assert result["execute_trajectory_called"] is False
    assert result["trajectory_sent"] is False
    assert result["real_robot_motion_executed"] is False
    assert all(step["execute_trajectory_called"] is False for step in result["substeps"])
    assert all(step["trajectory_sent"] is False for step in result["substeps"])
    assert all(step["real_robot_motion_executed"] is False for step in result["substeps"])
