import json

from src.evidence_exporter import EVIDENCE_MANIFEST_SCHEMA_VERSION, export_simulation_evidence
from src.simulation_runtime import run_first_simulation_execution


VALID_TASK = {
    "task_type": "hover_to_object",
    "target_label": "camera",
    "target_world_point": [0.2, 0.1, 0.4],
    "scene_version": "run_simulation_item_001",
    "ttl_ms": 500,
}


def test_dry_run_execution_writes_evidence_artifacts(tmp_path):
    result = run_first_simulation_execution(
        VALID_TASK,
        dry_run=True,
        steps=3,
        move_object=True,
        output_dir=tmp_path,
        write_report=True,
        demo_command="python3 scripts/run_first_simulation_execution.py --dry-run --steps 3 --move-object",
    )

    report_path = tmp_path / "simulation_execution_result.json"
    summary_path = tmp_path / "summary.md"
    demo_command_path = tmp_path / "demo_command.txt"
    pose_delta_path = tmp_path / "pose_delta.md"
    manifest_path = tmp_path / "evidence_manifest.json"
    structure_report_path = tmp_path / "robot_structure_report.md"

    assert report_path.exists()
    assert summary_path.exists()
    assert demo_command_path.exists()
    assert pose_delta_path.exists()
    assert manifest_path.exists()

    summary = summary_path.read_text(encoding="utf-8")
    assert "TETO version: TETO V2.9.1" in summary
    assert f"run_id: {tmp_path.name}" in summary
    assert "mode: dry_run" in summary
    assert "status: PASS" in summary
    assert "error.code: OK" in summary
    assert "world_reset: True" in summary
    assert "steps: 3/3" in summary
    assert "allow_robot_motion: False" in summary
    assert "object_type: cube" in summary
    assert "object prim path: /World/TETO_Cube" in summary
    assert "## Robot Asset" in summary
    assert "robot asset available: False" in summary
    assert "## Robot Prim Inspection" in summary
    assert "inspection status: NOT_REQUESTED" in summary
    assert "## Articulation Readiness" in summary
    assert "readiness_status: NOT_REQUESTED" in summary
    assert "## Articulation State Observation" in summary
    assert "status: NOT_REQUESTED" in summary
    assert f"report path: {report_path}" in summary

    demo_command = demo_command_path.read_text(encoding="utf-8")
    assert "--move-object" in demo_command
    assert "mode=dry_run" in demo_command
    assert "steps_requested=3" in demo_command
    assert "move_object=True" in demo_command
    assert "check_robot_asset=False" in demo_command
    assert "inspect_robot_prim=False" in demo_command
    assert "check_articulation_readiness=False" in demo_command

    pose_delta = pose_delta_path.read_text(encoding="utf-8")
    assert "initial_position: [0.0, 0.0, 0.5]" in pose_delta
    assert "target_position: [0.3, 0.0, 0.5]" in pose_delta
    assert "final_position: [0.3, 0.0, 0.5]" in pose_delta
    assert "displacement: [0.3, 0.0, 0.0]" in pose_delta
    assert "moved: True" in pose_delta

    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    assert manifest["schema_version"] == EVIDENCE_MANIFEST_SCHEMA_VERSION
    assert manifest["run_id"] == tmp_path.name
    assert manifest["teto_version"] == "TETO V2.9.1"
    assert manifest["mode"] == "dry_run"
    assert manifest["status"] == "PASS"
    assert manifest["report_path"] == str(report_path)
    assert manifest["summary_path"] == str(summary_path)
    assert manifest["demo_command_path"] == str(demo_command_path)
    assert manifest["pose_delta_path"] == str(pose_delta_path)
    assert manifest["robot_asset"]["robot_asset_available"] is False
    assert manifest["robot_asset"]["robot_asset_loaded"] is False
    assert manifest["robot_prim_inspection"]["requested"] is False
    assert manifest["robot_prim_inspection"]["inspection_status"] == "NOT_REQUESTED"
    assert manifest["articulation_readiness"]["requested"] is False
    assert manifest["articulation_readiness"]["readiness_status"] == "NOT_REQUESTED"
    assert manifest["articulation_readiness_path"] is None
    assert manifest["articulation_state"]["requested"] is False
    assert manifest["articulation_state"]["status"] == "NOT_REQUESTED"
    assert manifest["articulation_state_path"] is None
    assert manifest["articulation_state_report_path"] is None
    assert manifest["simulation_motion_precheck"]["requested"] is False
    assert manifest["simulation_motion_precheck"]["status"] == "NOT_REQUESTED"
    assert manifest["simulation_motion_precheck_path"] is None
    assert manifest["simulation_motion_precheck_report_path"] is None
    assert manifest["robot_prim_inspection_path"] is None
    assert manifest["robot_structure_report_path"] is None
    assert manifest["screenshot_before_path"] is None
    assert manifest["screenshot_after_path"] is None
    assert manifest["video_path"] is None
    assert result["report_path"] == str(report_path)
    assert result["robot_structure_report_generated"] is False
    assert result["robot_structure_report_path"] is None
    assert not structure_report_path.exists()


def test_evidence_exporter_writes_micro_motion_manifest_and_summary(tmp_path):
    result = run_first_simulation_execution(
        VALID_TASK,
        dry_run=True,
        steps=3,
        execute_simulation_micro_motion=True,
        micro_motion_joint="wrist_3_joint",
        micro_motion_delta_rad=0.01,
        output_dir=tmp_path,
        write_report=True,
        demo_command=(
            "python3 scripts/run_first_simulation_execution.py --dry-run --steps 3 "
            "--execute-simulation-micro-motion --micro-motion-joint wrist_3_joint --micro-motion-delta-rad 0.01"
        ),
    )

    summary = (tmp_path / "summary.md").read_text(encoding="utf-8")
    manifest = json.loads((tmp_path / "evidence_manifest.json").read_text(encoding="utf-8"))
    report = (tmp_path / "simulation_motion_report.md").read_text(encoding="utf-8")

    assert "## Micro-Motion Evidence Summary" in summary
    assert "actual_delta_rad: null" in summary
    assert "delta_within_tolerance: False" in summary
    assert f"simulation_motion_report_path: {tmp_path / 'simulation_motion_report.md'}" in summary
    assert f"before_joint_state_path: {tmp_path / 'before_articulation_state.json'}" in summary
    assert f"after_joint_state_path: {tmp_path / 'after_articulation_state.json'}" in summary

    assert manifest["motion_evidence_available"] is True
    assert manifest["simulation_only"] is True
    assert manifest["real_robot_motion_executed"] is False
    assert manifest["motion_diff_summary"]["joint_name"] == "wrist_3_joint"
    assert manifest["motion_diff_summary"]["requested_delta_rad"] == 0.01
    assert manifest["motion_diff_summary"]["actual_delta_rad"] is None
    assert manifest["motion_diff_summary"]["delta_within_tolerance"] is False
    assert manifest["motion_evidence_files"] == [
        {"name": "simulation_motion_result.json", "path": str(tmp_path / "simulation_motion_result.json")},
        {"name": "simulation_motion_report.md", "path": str(tmp_path / "simulation_motion_report.md")},
        {"name": "before_articulation_state.json", "path": str(tmp_path / "before_articulation_state.json")},
        {"name": "after_articulation_state.json", "path": str(tmp_path / "after_articulation_state.json")},
    ]
    assert manifest["simulation_micro_motion"]["motion_evidence_files"] == manifest["motion_evidence_files"]

    assert "# TETO V2.9.0 Simulation Micro-Motion Evidence Report" in report
    assert "## Joint Diff Summary" in report
    assert "## Evidence Files" in report
    assert "simulation_motion_result.json" in report
    assert "before_articulation_state.json" in report
    assert result["robot_motion_executed"] is False
    assert result["real_robot_motion_executed"] is False


def test_evidence_exporter_writes_semantic_bridge_manifest_and_summary(tmp_path):
    contract_path = "tests/fixtures/semantic_contracts/eligible_hover_to_object.json"
    with open(contract_path, encoding="utf-8") as contract_file:
        contract = json.load(contract_file)
    result = run_first_simulation_execution(
        VALID_TASK,
        dry_run=True,
        steps=3,
        semantic_simulation_bridge=True,
        semantic_task_contract=contract,
        semantic_task_contract_path=contract_path,
        output_dir=tmp_path,
        write_report=True,
        demo_command=(
            "python3 scripts/run_first_simulation_execution.py --dry-run --steps 3 "
            "--semantic-simulation-bridge --semantic-task-json tests/fixtures/semantic_contracts/eligible_hover_to_object.json"
        ),
    )

    summary = (tmp_path / "summary.md").read_text(encoding="utf-8")
    manifest = json.loads((tmp_path / "evidence_manifest.json").read_text(encoding="utf-8"))
    bridge_report = (tmp_path / "semantic_simulation_bridge_report.md").read_text(encoding="utf-8")

    assert "## Semantic-to-Simulation Bridge Summary" in summary
    assert "semantic_bridge_status: OK" in summary
    assert "semantic_task_id: fixture_eligible_hover_to_object" in summary
    assert "triggered_simulation_micro_motion: True" in summary
    assert manifest["semantic_bridge_requested"] is True
    assert manifest["semantic_bridge_status"] == "OK"
    assert manifest["semantic_bridge_evidence_available"] is True
    assert manifest["semantic_task_id"] == "fixture_eligible_hover_to_object"
    assert manifest["semantic_intent"] == "hover_to_object"
    assert manifest["semantic_target_label"] == "red_mug"
    assert manifest["semantic_gate_passed"] is True
    assert manifest["triggered_simulation_micro_motion"] is True
    assert {
        "name": "semantic_simulation_bridge_report.md",
        "path": str(tmp_path / "semantic_simulation_bridge_report.md"),
    } in manifest["semantic_bridge_files"]
    assert "# TETO V2.6.0 Semantic-to-Simulation Motion Bridge Report" in bridge_report
    assert "It does not call a live camera or live VLM." in bridge_report
    assert (tmp_path / "semantic_task_contract_copy.json").exists()
    assert result["real_robot_motion_executed"] is False


def test_evidence_exporter_writes_safe_execution_polish_fields(tmp_path):
    contract_path = "tests/fixtures/semantic_contracts/eligible_hover_to_object.json"
    with open(contract_path, encoding="utf-8") as contract_file:
        contract = json.load(contract_file)

    result = run_first_simulation_execution(
        VALID_TASK,
        dry_run=True,
        steps=3,
        semantic_simulation_bridge=True,
        semantic_task_contract=contract,
        semantic_task_contract_path=contract_path,
        safe_simulated_task_execution=True,
        execution_enable_retry_recommendation=True,
        execution_enable_fallback_recommendation=True,
        output_dir=tmp_path,
        write_report=True,
    )

    summary = (tmp_path / "summary.md").read_text(encoding="utf-8")
    manifest = json.loads((tmp_path / "evidence_manifest.json").read_text(encoding="utf-8"))
    failure = json.loads((tmp_path / "failure_analysis.json").read_text(encoding="utf-8"))
    recommendation = json.loads((tmp_path / "retry_fallback_recommendation.json").read_text(encoding="utf-8"))

    assert "### Human-readable conclusion" in summary
    assert "DRY_RUN_ONLY" in summary
    assert "### Evidence Files" in summary
    assert manifest["execution_evidence_available"] is True
    assert manifest["replay_ready"] is True
    assert manifest["safety_boundary_confirmed"] is True
    assert manifest["no_automatic_retry_executed"] is True
    assert manifest["latest_execution_summary"]["execution_feedback_status"] == "WARNING"
    assert manifest["latest_execution_summary"]["fallback_type"] == "RECHECK_SIMULATION_PRECHECK"
    assert "simulated_task_execution_report.md" in [
        item["name"] for item in manifest["replay_bundle_files"]
    ]
    assert failure["failure_reason"] == "E_DRY_RUN_ONLY"
    assert failure["failure_category"] == "DRY_RUN"
    assert failure["blocking_stage"] == "DRY_RUN"
    assert failure["next_safe_action"]
    assert recommendation["automatic_retry_executed"] is False
    assert recommendation["recommendation_reason"]
    assert recommendation["next_safe_action"]
    assert result["robot_motion_executed"] is False
    assert result["real_robot_motion_executed"] is False


def test_evidence_exporter_writes_robot_asset_metadata(tmp_path):
    result = run_first_simulation_execution(
        VALID_TASK,
        dry_run=True,
        steps=1,
        check_robot_asset=True,
        output_dir=tmp_path,
        write_report=True,
        demo_command="python3 scripts/run_first_simulation_execution.py --dry-run --steps 1 --check-robot-asset",
    )

    summary = (tmp_path / "summary.md").read_text(encoding="utf-8")
    assert "## Robot Asset" in summary
    assert "check requested: True" in summary
    assert "load requested: False" in summary
    assert "robot_type: ur5" in summary
    assert "robot prim path: /World/TETO_Robot" in summary
    assert "robot asset available: False" in summary
    assert "robot asset loaded: False" in summary
    assert "robot asset status: UNAVAILABLE" in summary
    assert "robot asset blocking reason: E_ROBOT_ASSET_UNAVAILABLE" in summary

    manifest = json.loads((tmp_path / "evidence_manifest.json").read_text(encoding="utf-8"))
    assert manifest["robot_asset"] == {
        "check_requested": True,
        "load_requested": False,
        "robot_type": "ur5",
        "robot_prim_path": "/World/TETO_Robot",
        "robot_asset_path": None,
        "robot_asset_source": "dry_run",
        "robot_asset_available": False,
        "robot_asset_loaded": False,
        "robot_prim_exists": False,
        "robot_asset_status": "UNAVAILABLE",
        "robot_asset_blocking_reason": "E_ROBOT_ASSET_UNAVAILABLE",
    }
    assert result["robot_asset_status"] == "UNAVAILABLE"


def test_evidence_exporter_writes_robot_prim_inspection_metadata(tmp_path):
    result = run_first_simulation_execution(
        VALID_TASK,
        dry_run=True,
        steps=1,
        inspect_robot_prim=True,
        output_dir=tmp_path,
        write_report=True,
        demo_command="python3 scripts/run_first_simulation_execution.py --dry-run --steps 1 --inspect-robot-prim",
    )

    summary = (tmp_path / "summary.md").read_text(encoding="utf-8")
    assert "## Robot Prim Inspection" in summary
    assert "requested: True" in summary
    assert "robot prim path: /World/TETO_Robot" in summary
    assert "robot prim exists: False" in summary
    assert "inspection status: E_ROBOT_PRIM_NOT_FOUND" in summary
    assert "### Joint Metadata Classification" in summary
    assert "| Arm joints | 0 | - |" in summary
    assert "Robot structure report:" in summary
    assert "read-only USD metadata records" in summary

    inspection_path = tmp_path / "robot_prim_inspection.json"
    structure_report_path = tmp_path / "robot_structure_report.md"
    assert inspection_path.exists()
    assert structure_report_path.exists()
    inspection = json.loads(inspection_path.read_text(encoding="utf-8"))
    assert inspection["requested"] is True
    assert inspection["robot_prim_path"] == "/World/TETO_Robot"
    assert inspection["robot_prim_exists"] is False
    assert inspection["inspection_status"] == "E_ROBOT_PRIM_NOT_FOUND"
    assert inspection["joint_metadata_summary"]["metadata_only"] is True
    assert inspection["joint_metadata_summary"]["control_ready"] is False
    assert inspection["joint_metadata_summary"]["control_targets_generated"] is False
    assert inspection["joint_metadata_table"] == []

    manifest = json.loads((tmp_path / "evidence_manifest.json").read_text(encoding="utf-8"))
    assert manifest["robot_prim_inspection"] == inspection
    assert manifest["robot_prim_inspection_path"] == str(inspection_path)
    assert manifest["robot_structure_report_path"] == str(structure_report_path)
    assert result["robot_prim_inspection_requested"] is True
    assert result["robot_structure_report_generated"] is True
    assert result["robot_structure_report_path"] == str(structure_report_path)

    structure_report = structure_report_path.read_text(encoding="utf-8")
    assert "# TETO UR5e Structure Report" in structure_report
    assert "## Asset Load Summary" in structure_report
    assert "## Prim Structure Summary" in structure_report
    assert "## Joint Metadata Classification" in structure_report
    assert "## Joint Metadata Table" in structure_report
    assert "## Safety Boundary" in structure_report
    assert "E_ROBOT_PRIM_NOT_FOUND" in structure_report
    assert "This report is generated from read-only USD metadata inspection" in structure_report


def test_evidence_exporter_writes_articulation_readiness_metadata(tmp_path):
    result = run_first_simulation_execution(
        VALID_TASK,
        dry_run=True,
        steps=1,
        inspect_robot_prim=True,
        check_articulation_readiness=True,
        output_dir=tmp_path,
        write_report=True,
        demo_command=(
            "python3 scripts/run_first_simulation_execution.py --dry-run --steps 1 "
            "--inspect-robot-prim --check-articulation-readiness"
        ),
    )

    summary = (tmp_path / "summary.md").read_text(encoding="utf-8")
    assert "## Articulation Readiness" in summary
    assert "readiness_status: NOT_READY" in summary
    assert "articulation_ready: False" in summary
    assert "control_enabled: False" in summary
    assert "motion_generated: False" in summary
    assert "command_generated: False" in summary

    readiness_path = tmp_path / "articulation_readiness.json"
    assert readiness_path.exists()
    readiness = json.loads(readiness_path.read_text(encoding="utf-8"))
    assert readiness["requested"] is True
    assert readiness["readiness_status"] == "NOT_READY"
    assert readiness["articulation_ready"] is False
    assert readiness["control_enabled"] is False
    assert readiness["motion_generated"] is False
    assert readiness["command_generated"] is False
    assert readiness["safety_boundary"]["read_only"] is True

    manifest = json.loads((tmp_path / "evidence_manifest.json").read_text(encoding="utf-8"))
    assert manifest["articulation_readiness"] == readiness
    assert manifest["articulation_readiness_path"] == str(readiness_path)
    assert result["articulation_readiness_path"] == str(readiness_path)

    structure_report = (tmp_path / "robot_structure_report.md").read_text(encoding="utf-8")
    assert "## Articulation Readiness" in structure_report
    assert "readiness_status: NOT_READY" in structure_report


def test_evidence_exporter_writes_articulation_state_metadata(tmp_path):
    result = run_first_simulation_execution(
        VALID_TASK,
        dry_run=True,
        steps=1,
        observe_articulation_state=True,
        output_dir=tmp_path,
        write_report=True,
        demo_command="python3 scripts/run_first_simulation_execution.py --dry-run --steps 1 --observe-articulation-state",
    )

    summary = (tmp_path / "summary.md").read_text(encoding="utf-8")
    assert "## Articulation State Observation" in summary
    assert "status: NOT_OBSERVABLE" in summary
    assert "control_enabled: False" in summary
    assert "motion_generated: False" in summary
    assert "command_generated: False" in summary
    assert "joint_targets_generated: False" in summary

    state_path = tmp_path / "articulation_state.json"
    state_report_path = tmp_path / "articulation_state_report.md"
    assert state_path.exists()
    assert state_report_path.exists()
    state = json.loads(state_path.read_text(encoding="utf-8"))
    assert state["requested"] is True
    assert state["status"] == "NOT_OBSERVABLE"
    assert state["metadata_only"] is True
    assert state["control_enabled"] is False
    assert state["motion_generated"] is False
    assert state["command_generated"] is False
    assert state["joint_targets_generated"] is False

    manifest = json.loads((tmp_path / "evidence_manifest.json").read_text(encoding="utf-8"))
    assert manifest["articulation_state"] == state
    assert manifest["articulation_state_path"] == str(state_path)
    assert manifest["articulation_state_report_path"] == str(state_report_path)
    assert result["articulation_state_path"] == str(state_path)
    assert result["articulation_state_report_path"] == str(state_report_path)

    state_report = state_report_path.read_text(encoding="utf-8")
    assert "metadata/state observation only" in state_report
    assert "control_enabled: False" in state_report
    assert "joint_targets_generated: False" in state_report
    assert "## Safety Boundary" in state_report


def test_evidence_exporter_writes_simulation_motion_precheck_metadata(tmp_path):
    result = run_first_simulation_execution(
        VALID_TASK,
        dry_run=True,
        steps=1,
        check_simulation_motion_precheck=True,
        output_dir=tmp_path,
        write_report=True,
        demo_command=(
            "python3 scripts/run_first_simulation_execution.py --dry-run --steps 1 "
            "--check-simulation-motion-precheck"
        ),
    )

    summary = (tmp_path / "summary.md").read_text(encoding="utf-8")
    assert "## Simulation Motion Precheck Summary" in summary
    assert "simulation_motion_precheck_status: NOT_READY" in summary
    assert "ready_for_simulation_motion: False" in summary
    assert "trajectory_generated: False" in summary
    assert "tcp_pose_world_generated: False" in summary
    assert "robot_motion_executed: False" in summary

    precheck_path = tmp_path / "simulation_motion_precheck.json"
    precheck_report_path = tmp_path / "simulation_motion_precheck_report.md"
    assert precheck_path.exists()
    assert precheck_report_path.exists()
    precheck = json.loads(precheck_path.read_text(encoding="utf-8"))
    assert precheck["requested"] is True
    assert precheck["status"] == "NOT_READY"
    assert precheck["ready"] is False
    assert precheck["control_enabled"] is False
    assert precheck["motion_generated"] is False
    assert precheck["command_generated"] is False
    assert precheck["joint_targets_generated"] is False
    assert precheck["trajectory_generated"] is False
    assert precheck["tcp_pose_world_generated"] is False
    assert precheck["robot_motion_executed"] is False

    manifest = json.loads((tmp_path / "evidence_manifest.json").read_text(encoding="utf-8"))
    assert manifest["simulation_motion_precheck"] == precheck
    assert manifest["simulation_motion_precheck_path"] == str(precheck_path)
    assert manifest["simulation_motion_precheck_report_path"] == str(precheck_report_path)
    assert result["simulation_motion_precheck_path"] == str(precheck_path)
    assert result["simulation_motion_precheck_report_path"] == str(precheck_report_path)

    precheck_report = precheck_report_path.read_text(encoding="utf-8")
    assert "simulation-only precheck" in precheck_report
    assert "metadata/state/readiness only" in precheck_report
    assert "control_enabled: False" in precheck_report
    assert "trajectory_generated: False" in precheck_report
    assert "tcp_pose_world_generated: False" in precheck_report
    assert "robot_motion_executed: False" in precheck_report
    assert "## Safety Boundary" in precheck_report


def test_evidence_exporter_displays_joint_metadata_classification(tmp_path):
    result = {
        "teto_version": "TETO V2.2.0",
        "status": "PASS",
        "mode": "isaac",
        "error": {"code": "OK", "message": ""},
        "world_reset": True,
        "steps_completed": 1,
        "steps_requested": 1,
        "allow_robot_motion": False,
        "finished_at": "2026-05-31 12:00:00",
        "report_path": str(tmp_path / "simulation_execution_result.json"),
        "robot_prim_inspection_requested": True,
        "robot_prim_inspection": {
            "requested": True,
            "robot_prim_path": "/World/TETO_Robot",
            "robot_prim_exists": True,
            "robot_root_type_name": "Xform",
            "total_descendant_prim_count": 45,
            "link_like_prim_count": 26,
            "joint_like_prim_count": 8,
            "visual_like_prim_count": 7,
            "collision_like_prim_count": 7,
            "articulation_root_found": True,
            "physics_schema_summary": [],
            "joint_names": [],
            "joint_prim_paths": [],
            "possible_dof_names": [],
            "possible_dof_count": 8,
            "joint_metadata_summary": {
                "possible_dof_count": 8,
                "possible_dof_names": [
                    "robot_gripper_joint",
                    "shoulder_pan_joint",
                    "shoulder_lift_joint",
                    "elbow_joint",
                    "wrist_1_joint",
                    "wrist_2_joint",
                    "wrist_3_joint",
                    "root_joint",
                ],
                "arm_joint_count": 6,
                "arm_joint_names": [
                    "shoulder_pan_joint",
                    "shoulder_lift_joint",
                    "elbow_joint",
                    "wrist_1_joint",
                    "wrist_2_joint",
                    "wrist_3_joint",
                ],
                "structural_joint_count": 1,
                "structural_joint_names": ["root_joint"],
                "gripper_or_tool_joint_count": 1,
                "gripper_or_tool_joint_names": ["robot_gripper_joint"],
                "unknown_joint_count": 0,
                "unknown_joint_names": [],
                "metadata_only": True,
                "control_ready": False,
                "control_targets_generated": False,
            },
            "joint_metadata_table": [
                {
                    "joint_name": "shoulder_pan_joint",
                    "joint_prim_path": "/World/TETO_Robot/joints/shoulder_pan_joint",
                    "joint_type_name": "PhysicsRevoluteJoint",
                    "category": "arm",
                    "is_ur5e_arm_joint": True,
                    "metadata_only": True,
                    "control_target_generated": False,
                    "applied_schemas": ["PhysicsJointAPI"],
                    "parent_path": None,
                    "child_path": None,
                }
            ],
            "inspection_status": "OK",
            "inspection_warnings": [],
        },
    }

    paths = export_simulation_evidence(result, tmp_path)

    summary = paths["summary_path"].read_text(encoding="utf-8")
    assert "### Joint Metadata Classification" in summary
    assert "| Arm joints | 6 | shoulder_pan_joint, shoulder_lift_joint, elbow_joint, wrist_1_joint, wrist_2_joint, wrist_3_joint |" in summary
    assert "| Structural joints | 1 | root_joint |" in summary
    assert "| Gripper/tool joints | 1 | robot_gripper_joint |" in summary
    assert "| Unknown joints | 0 | - |" in summary
    assert "not joint targets, not joint commands" in summary

    manifest = json.loads(paths["evidence_manifest_path"].read_text(encoding="utf-8"))
    assert manifest["robot_prim_inspection"]["joint_metadata_summary"]["arm_joint_count"] == 6
    assert manifest["robot_prim_inspection"]["joint_metadata_table"][0]["category"] == "arm"
    assert manifest["robot_structure_report_path"] == str(tmp_path / "robot_structure_report.md")

    structure_report = (tmp_path / "robot_structure_report.md").read_text(encoding="utf-8")
    assert "# TETO UR5e Structure Report" in structure_report
    assert "## Asset Load Summary" in structure_report
    assert "## Prim Structure Summary" in structure_report
    assert "## Joint Metadata Classification" in structure_report
    assert "## Joint Metadata Table" in structure_report
    assert "## Safety Boundary" in structure_report
    assert "| shoulder_pan_joint | arm | /World/TETO_Robot/joints/shoulder_pan_joint | PhysicsRevoluteJoint | True | True | False |" in structure_report
    assert "metadata_only" in structure_report
    assert "control_target_generated" in structure_report
    assert "This report is generated from read-only USD metadata inspection" in structure_report
    assert "robot asset understanding and evidence export, not robot control" in structure_report


def test_evidence_exporter_uses_cube_fields_as_compatibility_fallback(tmp_path):
    result = {
        "teto_version": "TETO V2.0.3",
        "status": "PASS",
        "mode": "dry_run",
        "error": {"code": "OK", "message": ""},
        "world_reset": True,
        "steps_completed": 1,
        "steps_requested": 1,
        "allow_robot_motion": False,
        "finished_at": "2026-05-31 04:00:00",
        "report_path": str(tmp_path / "simulation_execution_result.json"),
        "object_type": "cube",
        "cube_prim_path": "/World/TETO_Cube",
        "cube_initial_position": [0.0, 0.0, 0.5],
        "cube_target_position": [0.3, 0.0, 0.5],
        "cube_final_position": [0.3, 0.0, 0.5],
        "cube_displacement": [0.3, 0.0, 0.0],
        "cube_moved": True,
    }

    paths = export_simulation_evidence(result, tmp_path)

    summary = paths["summary_path"].read_text(encoding="utf-8")
    pose_delta = paths["pose_delta_path"].read_text(encoding="utf-8")
    assert "object_type: cube" in summary
    assert "object prim path: /World/TETO_Cube" in summary
    assert "initial position: [0.0, 0.0, 0.5]" in summary
    assert "target_position: [0.3, 0.0, 0.5]" in pose_delta
    assert "moved: True" in pose_delta
