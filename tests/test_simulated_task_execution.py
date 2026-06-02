import json
from pathlib import Path

from src.semantic_simulation_bridge import load_semantic_task_contract
from src.simulated_task_execution import (
    FAILURE_REASON_DELTA_OUT_OF_TOLERANCE,
    FAILURE_REASON_NONE,
    FAILURE_REASON_POST_STATE_MISSING,
    FAILURE_REASON_PRECHECK_NOT_READY,
    FAILURE_REASON_SEMANTIC_GATE_BLOCKED,
    FALLBACK_TYPE_RECHECK_SIMULATION_PRECHECK,
    FALLBACK_TYPE_REOBSERVE,
    SimulatedTaskExecutionRequest,
    analyze_execution_failure,
    execute_safe_simulated_task,
    recommend_retry_or_fallback,
)
from src.simulation_runtime import run_first_simulation_execution


FIXTURE_DIR = Path(__file__).parent / "fixtures" / "semantic_contracts"

VALID_TASK = {
    "task_type": "hover_to_object",
    "target_label": "camera",
    "target_world_point": [0.2, 0.1, 0.4],
    "scene_version": "run_simulation_item_001",
    "ttl_ms": 500,
}


def _true_like_context() -> dict:
    return {
        "mode": "isaac",
        "started_at": "2026-06-02 12:00:00",
        "semantic_bridge_status": "OK",
        "semantic_gate_passed": True,
        "semantic_task_id": "fixture_eligible_hover_to_object",
        "simulation_motion_precheck_status": "READY_FOR_SIMULATION_MOTION",
        "ready_for_simulation_motion": True,
        "simulation_motion_precheck": {"blocking_reasons": [], "warnings": [], "errors": []},
        "simulation_micro_motion_status": "OK",
        "simulation_micro_motion_blocking_reasons": [],
        "motion": {
            "actual_delta_rad": 0.01,
            "delta_within_tolerance": True,
        },
        "actual_delta_rad": 0.01,
        "delta_within_tolerance": True,
        "before_articulation_state": {"status": "OK"},
        "after_articulation_state": {"status": "OK"},
        "robot_motion_executed": True,
        "real_robot_motion_executed": False,
    }


def _execution_request() -> SimulatedTaskExecutionRequest:
    return SimulatedTaskExecutionRequest(
        requested=True,
        execution_attempt_id="attempt_fixture_001",
        retry_recommendation_enabled=True,
        fallback_recommendation_enabled=True,
    )


def test_eligible_semantic_contract_produces_succeeded_in_true_like_result():
    result = execute_safe_simulated_task(_execution_request(), _true_like_context())

    assert result["simulated_task_status"] == "SUCCEEDED"
    assert result["execution_feedback_status"] == "OK"
    assert result["failure_reason"] == FAILURE_REASON_NONE
    assert result["retry_recommended"] is False
    assert result["fallback_recommended"] is False
    assert result["post_motion_state_check"]["post_motion_state_check_status"] == "OK"


def test_dry_run_produces_dry_run_only_without_claiming_robot_motion(tmp_path):
    contract = load_semantic_task_contract(FIXTURE_DIR / "eligible_hover_to_object.json")

    result = run_first_simulation_execution(
        VALID_TASK,
        dry_run=True,
        steps=3,
        semantic_simulation_bridge=True,
        semantic_task_contract=contract,
        safe_simulated_task_execution=True,
        execution_enable_retry_recommendation=True,
        execution_enable_fallback_recommendation=True,
        output_dir=tmp_path,
        write_report=True,
    )

    assert result["simulated_task_status"] == "DRY_RUN_ONLY"
    assert result["execution_feedback_status"] == "WARNING"
    assert result["failure_reason"] == "E_DRY_RUN_ONLY"
    assert result["robot_motion_executed"] is False
    assert result["real_robot_motion_executed"] is False
    assert result["post_motion_state_check"]["post_motion_state_check_status"] == "DRY_RUN_ONLY"


def test_semantic_gate_blocked_contracts_produce_blocked_status(tmp_path):
    for fixture in ("no_target_rejected.json", "low_confidence.json", "unsafe_target.json"):
        result = run_first_simulation_execution(
            VALID_TASK,
            dry_run=True,
            steps=3,
            semantic_simulation_bridge=True,
            semantic_task_contract=load_semantic_task_contract(FIXTURE_DIR / fixture),
            safe_simulated_task_execution=True,
            execution_enable_retry_recommendation=True,
            execution_enable_fallback_recommendation=True,
            output_dir=tmp_path / fixture,
            write_report=True,
        )

        assert result["semantic_bridge_status"] == "BLOCKED_BY_SEMANTIC_GATE"
        assert result["semantic_gate_passed"] is False
        assert result["triggered_simulation_micro_motion"] is False
        assert result["simulation_micro_motion_status"] == "BLOCKED_BY_SEMANTIC_GATE"
        assert result["simulated_task_status"] == "BLOCKED_BY_SEMANTIC_GATE"
        assert result["execution_feedback_status"] == "BLOCKED"
        assert result["failure_reason"] == FAILURE_REASON_SEMANTIC_GATE_BLOCKED
    assert result["fallback_recommended"] is True

    blocked_report = tmp_path / "no_target_rejected.json" / "simulated_task_execution_report.md"
    blocked_manifest = tmp_path / "no_target_rejected.json" / "evidence_manifest.json"
    assert blocked_report.exists()
    assert blocked_manifest.exists()
    assert "BLOCKED_BY_SEMANTIC_GATE" in blocked_report.read_text(encoding="utf-8")


def test_precheck_not_ready_produces_blocked_by_precheck():
    context = _true_like_context()
    context.update(
        {
            "simulation_motion_precheck_status": "NOT_READY",
            "ready_for_simulation_motion": False,
            "simulation_micro_motion_status": "BLOCKED_BY_PRECHECK",
            "simulation_motion_precheck": {"blocking_reasons": ["E_ROBOT_PRIM_NOT_FOUND"]},
        }
    )

    result = execute_safe_simulated_task(_execution_request(), context)

    assert result["simulated_task_status"] == "BLOCKED_BY_PRECHECK"
    assert result["failure_reason"] == FAILURE_REASON_PRECHECK_NOT_READY
    assert result["fallback_type"] == FALLBACK_TYPE_RECHECK_SIMULATION_PRECHECK


def test_missing_before_after_state_produces_post_check_failed():
    context = _true_like_context()
    context["before_articulation_state"] = {}
    context["after_articulation_state"] = {}

    result = execute_safe_simulated_task(_execution_request(), context)

    assert result["simulated_task_status"] == "POST_CHECK_FAILED"
    assert result["failure_reason"] == FAILURE_REASON_POST_STATE_MISSING
    assert result["post_motion_state_check"]["post_motion_state_check_status"] == "FAILED"


def test_delta_outside_tolerance_produces_motion_failed():
    context = _true_like_context()
    context["delta_within_tolerance"] = False
    context["motion"]["delta_within_tolerance"] = False

    result = execute_safe_simulated_task(_execution_request(), context)

    assert result["simulated_task_status"] == "MOTION_FAILED"
    assert result["failure_reason"] == FAILURE_REASON_DELTA_OUT_OF_TOLERANCE


def test_failure_analysis_maps_no_target_to_reobserve_fallback():
    failure = analyze_execution_failure(
        {
            "semantic_bridge_blocking_reasons": ["E_NO_TARGET"],
            "simulation_motion_precheck": {},
        },
        "BLOCKED_BY_SEMANTIC_GATE",
        post_check={},
    )
    recommendation = recommend_retry_or_fallback(
        failure,
        retry_enabled=True,
        fallback_enabled=True,
    )

    assert failure["failure_reason"] == FAILURE_REASON_SEMANTIC_GATE_BLOCKED
    assert recommendation["fallback_recommended"] is True
    assert recommendation["fallback_type"] == FALLBACK_TYPE_REOBSERVE
    assert recommendation["automatic_retry_executed"] is False


def test_evidence_files_are_written_for_safe_execution(tmp_path):
    contract = load_semantic_task_contract(FIXTURE_DIR / "eligible_hover_to_object.json")

    run_first_simulation_execution(
        VALID_TASK,
        dry_run=True,
        steps=3,
        semantic_simulation_bridge=True,
        semantic_task_contract=contract,
        safe_simulated_task_execution=True,
        execution_enable_retry_recommendation=True,
        execution_enable_fallback_recommendation=True,
        output_dir=tmp_path,
        write_report=True,
    )

    manifest = json.loads((tmp_path / "evidence_manifest.json").read_text(encoding="utf-8"))
    summary = (tmp_path / "summary.md").read_text(encoding="utf-8")
    report = (tmp_path / "simulated_task_execution_report.md").read_text(encoding="utf-8")
    failure = json.loads((tmp_path / "failure_analysis.json").read_text(encoding="utf-8"))
    recommendation = json.loads((tmp_path / "retry_fallback_recommendation.json").read_text(encoding="utf-8"))

    assert manifest["simulated_task_execution_requested"] is True
    assert manifest["execution_evidence_available"] is True
    assert manifest["replay_ready"] is True
    assert manifest["safety_boundary_confirmed"] is True
    assert manifest["no_automatic_retry_executed"] is True
    assert manifest["latest_execution_summary"]["simulated_task_status"] == "DRY_RUN_ONLY"
    assert "simulated_task_execution_report.md" in [
        item["name"] for item in manifest["execution_evidence_files"]
    ]
    assert "simulated_task_execution_result.json" in [
        item["name"] for item in manifest["simulated_task_execution_files"]
    ]
    assert "## Safe Simulated Task Execution Summary" in summary
    assert "### Human-readable conclusion" in summary
    assert "DRY_RUN_ONLY: this run generated replay-ready dry-run evidence only" in summary
    assert "### Evidence Files" in summary
    assert "# TETO V2.8.1 Safe Simulated Task Execution Evidence Report" in report
    assert "## Execution Lifecycle Table" in report
    assert "## Gate Decision Table" in report
    assert "## Motion Verification Table" in report
    assert "## Failure / Retry / Fallback Table" in report
    assert "## Replay Readiness Summary" in report
    assert "This is a safe simulated task execution evidence report." in report
    assert "It does not call a live camera or live VLM." in report
    assert "It does not execute target poses, tcp_pose_world, trajectories, MoveIt goals, URScript, Dashboard commands, or real robot commands." in report
    assert "Retry and fallback are recommendations only; no automatic repeated motion is executed." in report
    assert failure["next_safe_action"]
    assert failure["failure_category"] == "DRY_RUN"
    assert failure["blocking_stage"] == "DRY_RUN"
    assert failure["human_readable_message"] == "Dry-run only; no real Isaac state change is claimed."
    assert failure["retry_recommended"] is True
    assert failure["fallback_recommended"] is True
    assert failure["fallback_type"] == "RECHECK_SIMULATION_PRECHECK"
    assert recommendation["automatic_retry_executed"] is False
    assert recommendation["recommendation_reason"]
    assert recommendation["next_safe_action"]
    assert "real_robot_motion_executed: True" not in report
    assert "robot_motion_executed: True" not in summary
