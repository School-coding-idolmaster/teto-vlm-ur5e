import json
from pathlib import Path

from src.semantic_simulation_bridge import (
    MICRO_MOTION_COMMAND_TYPE,
    build_semantic_simulation_bridge_result,
    build_simulation_micro_motion_request_from_semantic_contract,
    evaluate_semantic_contract_for_simulation_bridge,
    format_semantic_simulation_bridge_report,
    load_semantic_task_contract,
    SemanticSimulationBridgeRequest,
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


def _contract(name: str) -> dict:
    return load_semantic_task_contract(FIXTURE_DIR / name)


def test_eligible_semantic_contract_passes_semantic_gate():
    gate = evaluate_semantic_contract_for_simulation_bridge(_contract("eligible_hover_to_object.json"))

    assert gate["passed"] is True
    assert gate["blocking_reasons"] == []
    assert gate["summary"]["target_label"] == "red_mug"


def test_eligible_semantic_contract_builds_simulation_only_micro_motion_request():
    request = build_simulation_micro_motion_request_from_semantic_contract(
        _contract("eligible_hover_to_object.json")
    )

    assert request.joint_name == "wrist_3_joint"
    assert request.requested_delta_rad == 0.01
    assert request.tolerance_rad == 0.005


def test_blocking_contracts_do_not_pass_semantic_gate():
    cases = {
        "no_target_rejected.json": "E_NO_TARGET",
        "low_confidence.json": "E_LOW_CONFIDENCE",
        "unsafe_target.json": "E_UNSAFE_TARGET",
        "stale_scene.json": "E_STATE_STALE",
        "invalid_contract.json": "E_MISSING_SCENE_VERSION",
    }

    for fixture, reason in cases.items():
        gate = evaluate_semantic_contract_for_simulation_bridge(_contract(fixture))
        assert gate["passed"] is False
        assert reason in gate["blocking_reasons"]


def test_invalid_bbox_and_missing_pixel_center_are_blocked():
    gate = evaluate_semantic_contract_for_simulation_bridge(_contract("invalid_contract.json"))

    assert gate["passed"] is False
    assert "E_MISSING_GROUNDING" in gate["blocking_reasons"]


def test_tcp_pose_world_is_audited_but_not_used_for_request():
    contract = _contract("eligible_hover_to_object.json")
    contract["tcp_pose_world"] = [0.1, 0.2, 0.3, 0, 0, 0]
    contract["pose_candidates"] = [{"tcp_pose_world": [0, 0, 0, 0, 0, 0]}]

    result = build_semantic_simulation_bridge_result(
        SemanticSimulationBridgeRequest(semantic_task_contract=contract)
    )

    assert result["gate_passed"] is True
    assert "tcp_pose_world" in result["audited_non_executable_fields"]
    assert result["simulation_micro_motion_request"] == {
        "joint_name": "wrist_3_joint",
        "requested_delta_rad": 0.01,
        "tolerance_rad": 0.005,
        "command_type": MICRO_MOTION_COMMAND_TYPE,
        "triggered_by_semantic_bridge": True,
    }


def test_report_contains_safety_boundary_statement():
    result = build_semantic_simulation_bridge_result(
        SemanticSimulationBridgeRequest(semantic_task_contract=_contract("eligible_hover_to_object.json"))
    )

    report = format_semantic_simulation_bridge_report(result)

    assert "# TETO V2.6.0 Semantic-to-Simulation Motion Bridge Report" in report
    assert "It does not call a live camera or live VLM." in report
    assert "It does not execute target poses, tcp_pose_world, trajectories, MoveIt goals, URScript, or real robot commands." in report


def test_dry_run_semantic_bridge_triggers_micro_motion_without_real_robot(tmp_path):
    result = run_first_simulation_execution(
        VALID_TASK,
        dry_run=True,
        steps=3,
        semantic_simulation_bridge=True,
        semantic_task_contract=_contract("eligible_hover_to_object.json"),
        semantic_task_contract_path=str(FIXTURE_DIR / "eligible_hover_to_object.json"),
        output_dir=tmp_path,
        write_report=True,
    )

    assert result["status"] == "PASS"
    assert result["mode"] == "dry_run"
    assert result["semantic_bridge_status"] == "OK"
    assert result["semantic_gate_passed"] is True
    assert result["triggered_simulation_micro_motion"] is True
    assert result["simulation_motion_precheck_requested"] is True
    assert result["simulation_micro_motion_status"] == "DRY_RUN_ONLY"
    assert result["robot_motion_executed"] is False
    assert result["real_robot_motion_executed"] is False
    assert result["safety"]["no_live_camera_used"] is True
    assert result["safety"]["no_tcp_pose_world_executed"] is True
    assert (tmp_path / "semantic_simulation_bridge_result.json").exists()
    assert (tmp_path / "semantic_simulation_bridge_report.md").exists()
    assert (tmp_path / "semantic_task_contract_copy.json").exists()


def test_blocked_semantic_bridge_generates_evidence_without_motion(tmp_path):
    result = run_first_simulation_execution(
        VALID_TASK,
        dry_run=True,
        steps=3,
        semantic_simulation_bridge=True,
        semantic_task_contract=_contract("no_target_rejected.json"),
        semantic_task_contract_path=str(FIXTURE_DIR / "no_target_rejected.json"),
        output_dir=tmp_path,
        write_report=True,
    )

    assert result["status"] == "PASS"
    assert result["semantic_bridge_status"] == "BLOCKED_BY_SEMANTIC_GATE"
    assert result["semantic_gate_passed"] is False
    assert result["triggered_simulation_micro_motion"] is False
    assert result["simulation_micro_motion_requested"] is False
    assert result["simulation_micro_motion_status"] == "BLOCKED_BY_SEMANTIC_GATE"
    assert result["robot_motion_executed"] is False
    assert result["real_robot_motion_executed"] is False
    assert "E_NO_TARGET" in result["semantic_bridge_blocking_reasons"]
    assert (tmp_path / "semantic_simulation_bridge_result.json").exists()
    assert (tmp_path / "semantic_simulation_bridge_report.md").exists()
    saved = json.loads((tmp_path / "evidence_manifest.json").read_text(encoding="utf-8"))
    assert saved["semantic_bridge_evidence_available"] is True
    assert saved["triggered_simulation_micro_motion"] is False
