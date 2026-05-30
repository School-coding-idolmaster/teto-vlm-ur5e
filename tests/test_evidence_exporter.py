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

    assert report_path.exists()
    assert summary_path.exists()
    assert demo_command_path.exists()
    assert pose_delta_path.exists()
    assert manifest_path.exists()

    summary = summary_path.read_text(encoding="utf-8")
    assert "TETO version: TETO V2.0.3" in summary
    assert f"run_id: {tmp_path.name}" in summary
    assert "mode: dry_run" in summary
    assert "status: PASS" in summary
    assert "error.code: OK" in summary
    assert "world_reset: True" in summary
    assert "steps: 3/3" in summary
    assert "allow_robot_motion: False" in summary
    assert "object_type: cube" in summary
    assert "object prim path: /World/TETO_Cube" in summary
    assert f"report path: {report_path}" in summary

    demo_command = demo_command_path.read_text(encoding="utf-8")
    assert "--move-object" in demo_command
    assert "mode=dry_run" in demo_command
    assert "steps_requested=3" in demo_command
    assert "move_object=True" in demo_command

    pose_delta = pose_delta_path.read_text(encoding="utf-8")
    assert "initial_position: [0.0, 0.0, 0.5]" in pose_delta
    assert "target_position: [0.3, 0.0, 0.5]" in pose_delta
    assert "final_position: [0.3, 0.0, 0.5]" in pose_delta
    assert "displacement: [0.3, 0.0, 0.0]" in pose_delta
    assert "moved: True" in pose_delta

    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    assert manifest["schema_version"] == EVIDENCE_MANIFEST_SCHEMA_VERSION
    assert manifest["run_id"] == tmp_path.name
    assert manifest["teto_version"] == "TETO V2.0.3"
    assert manifest["mode"] == "dry_run"
    assert manifest["status"] == "PASS"
    assert manifest["report_path"] == str(report_path)
    assert manifest["summary_path"] == str(summary_path)
    assert manifest["demo_command_path"] == str(demo_command_path)
    assert manifest["pose_delta_path"] == str(pose_delta_path)
    assert manifest["screenshot_before_path"] is None
    assert manifest["screenshot_after_path"] is None
    assert manifest["video_path"] is None
    assert result["report_path"] == str(report_path)


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
