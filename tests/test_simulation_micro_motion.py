import copy
import json

import pytest

from src.simulation_micro_motion import (
    DEFAULT_MICRO_MOTION_TOLERANCE_RAD,
    SimulationMicroMotionRequest,
    build_joint_diff_summary,
    compute_joint_delta,
    execute_simulation_micro_motion,
    format_joint_diff_table,
    format_simulation_micro_motion_report,
    validate_micro_motion_request,
)
from src.simulation_runtime import run_first_simulation_execution


ARM_JOINTS = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]

VALID_TASK = {
    "task_type": "hover_to_object",
    "target_label": "camera",
    "target_world_point": [0.2, 0.1, 0.4],
    "scene_version": "run_simulation_item_001",
    "ttl_ms": 500,
}


def ready_precheck():
    return {
        "status": "READY_FOR_SIMULATION_MOTION",
        "ready": True,
        "blocking_reasons": [],
        "warnings": [],
        "errors": [],
        "observed_arm_joint_names": ARM_JOINTS,
    }


def ready_readiness():
    return {
        "readiness_status": "READY",
        "articulation_ready": True,
    }


def articulation_state(wrist_position=0.2):
    rows = []
    for name in ARM_JOINTS:
        rows.append(
            {
                "joint_name": name,
                "category": "arm",
                "position": wrist_position if name == "wrist_3_joint" else 0.0,
                "velocity": 0.0,
                "lower_limit": -3.14,
                "upper_limit": 3.14,
                "limit_available": True,
                "within_limit": True,
                "metadata_only": True,
                "control_target_generated": False,
            }
        )
    return {
        "requested": True,
        "status": "OK",
        "articulation_state_observable": True,
        "observed_arm_joint_names": ARM_JOINTS,
        "joint_state_table": rows,
    }


def fake_executor(request, before_state):
    after_state = copy.deepcopy(before_state)
    for row in after_state["joint_state_table"]:
        if row["joint_name"] == request.joint_name:
            row["position"] = row["position"] + request.requested_delta_rad
    return {"after_articulation_state": after_state}


def test_valid_wrist_micro_motion_request_executes_small_delta():
    request = SimulationMicroMotionRequest(joint_name="wrist_3_joint", requested_delta_rad=0.01)

    result = execute_simulation_micro_motion(
        request,
        simulation_motion_precheck=ready_precheck(),
        articulation_readiness=ready_readiness(),
        before_articulation_state=articulation_state(),
        motion_executor=fake_executor,
    )

    assert result["simulation_micro_motion_status"] == "OK"
    assert result["simulation_only"] is True
    assert result["real_robot_allowed"] is False
    assert result["real_robot_motion_executed"] is False
    assert result["robot_motion_executed"] is True
    assert result["motion"]["joint_name"] == "wrist_3_joint"
    assert result["motion"]["requested_delta_rad"] == 0.01
    assert result["motion"]["actual_delta_rad"] == pytest.approx(0.01)
    assert result["motion"]["delta_within_tolerance"] is True


def test_delta_larger_than_micro_limit_is_rejected():
    request = SimulationMicroMotionRequest(joint_name="wrist_3_joint", requested_delta_rad=0.02)

    errors = validate_micro_motion_request(request, available_joint_names=ARM_JOINTS)

    assert "E_MICRO_MOTION_DELTA_TOO_LARGE" in errors


def test_non_arm_or_unknown_joint_is_rejected():
    root_errors = validate_micro_motion_request(
        SimulationMicroMotionRequest(joint_name="root_joint", requested_delta_rad=0.01),
        available_joint_names=ARM_JOINTS,
    )
    unknown_errors = validate_micro_motion_request(
        SimulationMicroMotionRequest(joint_name="missing_joint", requested_delta_rad=0.01),
        available_joint_names=ARM_JOINTS,
    )

    assert "E_MICRO_MOTION_JOINT_NOT_ARM" in root_errors
    assert "E_MICRO_MOTION_JOINT_UNKNOWN" in unknown_errors


def test_precheck_not_ready_blocks_motion():
    result = execute_simulation_micro_motion(
        SimulationMicroMotionRequest(),
        simulation_motion_precheck={"status": "NOT_READY", "ready": False, "blocking_reasons": ["E_NOT_READY"]},
        articulation_readiness=ready_readiness(),
        before_articulation_state=articulation_state(),
        motion_executor=fake_executor,
    )

    assert result["simulation_micro_motion_status"] == "BLOCKED_BY_PRECHECK"
    assert result["robot_motion_executed"] is False
    assert result["errors"] == ["E_SIMULATION_MOTION_PRECHECK_NOT_READY"]
    assert "E_NOT_READY" in result["blocking_reasons"]


def test_before_after_delta_calculation_is_correct():
    assert compute_joint_delta(0.2, 0.21) == 0.009999999999999981


def test_report_markdown_contains_simulation_only_safety_statement():
    result = execute_simulation_micro_motion(
        SimulationMicroMotionRequest(),
        simulation_motion_precheck=ready_precheck(),
        articulation_readiness=ready_readiness(),
        before_articulation_state=articulation_state(),
        motion_executor=fake_executor,
    )

    report = format_simulation_micro_motion_report(result)

    assert "This is simulation-only micro-motion." in report
    assert "No real robot command was generated." in report
    assert "No ROS2 / MoveIt / RTDE / URScript / real UR5 control chain was used." in report
    assert "The motion was executed only through the local Isaac Sim simulation API." in report


def test_joint_diff_summary_and_table_are_formatted():
    result = execute_simulation_micro_motion(
        SimulationMicroMotionRequest(),
        simulation_motion_precheck=ready_precheck(),
        articulation_readiness=ready_readiness(),
        before_articulation_state=articulation_state(),
        motion_executor=fake_executor,
    )

    summary = build_joint_diff_summary(result)
    table = format_joint_diff_table(result)

    assert summary["joint_name"] == "wrist_3_joint"
    assert summary["before_joint_position_rad"] == 0.2
    assert summary["after_joint_position_rad"] == pytest.approx(0.21)
    assert summary["requested_delta_rad"] == 0.01
    assert summary["actual_delta_rad"] == pytest.approx(0.01)
    assert summary["delta_within_tolerance"] is True
    assert "| Joint name | Before rad | After rad | Requested delta rad | Actual delta rad |" in table
    assert "| wrist_3_joint | 0.2 |" in table


def test_report_markdown_contains_diff_and_evidence_sections():
    result = execute_simulation_micro_motion(
        SimulationMicroMotionRequest(),
        simulation_motion_precheck=ready_precheck(),
        articulation_readiness=ready_readiness(),
        before_articulation_state=articulation_state(),
        motion_executor=fake_executor,
    )

    report = format_simulation_micro_motion_report(result)

    assert "# TETO V2.8.0 Simulation Micro-Motion Evidence Report" in report
    assert "## Status" in report
    assert "## Precheck Summary" in report
    assert "## Joint Diff Summary" in report
    assert "## Evidence Files" in report
    assert "## Safety Boundary" in report
    assert "before_joint_position_rad: 0.2" in report
    assert "after_joint_position_rad: 0.21000000000000002" in report
    assert "requested_delta_rad: 0.01" in report
    assert "actual_delta_rad: 0.010000000000000009" in report
    assert "tolerance_rad: 0.005" in report
    assert "delta_within_tolerance: True" in report


def test_blocked_by_precheck_report_is_auditable():
    result = execute_simulation_micro_motion(
        SimulationMicroMotionRequest(),
        simulation_motion_precheck={"status": "NOT_READY", "ready": False, "blocking_reasons": ["E_NOT_READY"]},
        articulation_readiness=ready_readiness(),
        before_articulation_state=articulation_state(),
        motion_executor=fake_executor,
    )

    report = format_simulation_micro_motion_report(result)

    assert "simulation_micro_motion_status: BLOCKED_BY_PRECHECK" in report
    assert "simulation_motion_precheck_status: NOT_READY" in report
    assert "blocking_reasons: [\"E_NOT_READY\"]" in report
    assert "robot_motion_executed: False" in report


def test_result_json_does_not_add_real_robot_control_chain_fields():
    result = execute_simulation_micro_motion(
        SimulationMicroMotionRequest(),
        simulation_motion_precheck=ready_precheck(),
        articulation_readiness=ready_readiness(),
        before_articulation_state=articulation_state(),
        motion_executor=fake_executor,
    )
    serialized = json.dumps(result)

    assert "ROS2" not in serialized
    assert "MoveIt" not in serialized
    assert "RTDE" not in serialized
    assert "URScript" not in serialized
    assert result["real_robot_allowed"] is False
    assert result["real_robot_motion_executed"] is False
    assert result["motion"]["command_type"] == "ISAAC_SIMULATION_API_LOCAL_ONLY"


def test_dry_run_micro_motion_does_not_claim_robot_motion_executed(tmp_path):
    result = run_first_simulation_execution(
        VALID_TASK,
        dry_run=True,
        steps=3,
        execute_simulation_micro_motion=True,
        micro_motion_joint="wrist_3_joint",
        micro_motion_delta_rad=0.01,
        micro_motion_tolerance_rad=DEFAULT_MICRO_MOTION_TOLERANCE_RAD,
        output_dir=tmp_path,
        write_report=True,
    )

    assert result["status"] == "PASS"
    assert result["simulation_micro_motion_requested"] is True
    assert result["simulation_micro_motion_status"] == "DRY_RUN_ONLY"
    assert result["robot_motion_executed"] is False
    assert result["real_robot_motion_executed"] is False
    assert (tmp_path / "simulation_motion_result.json").exists()
    assert (tmp_path / "simulation_motion_report.md").exists()
    assert (tmp_path / "before_articulation_state.json").exists()
    assert (tmp_path / "after_articulation_state.json").exists()
