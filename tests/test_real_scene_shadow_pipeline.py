import json
from datetime import datetime, timezone

import yaml

from src.camera_snapshot import CameraSnapshotRequest, evaluate_camera_snapshot_contract
from src.grounding.result import GroundingResultRequest, evaluate_grounding_result_contract
from src.real_scene_shadow_pipeline import (
    E_INVALID_BBOX,
    E_INVALID_PIXEL_CENTER,
    E_LIVE_CAMERA_DISABLED,
    E_LIVE_VLM_DISABLED,
    E_LOW_CONFIDENCE,
    E_NO_TARGET,
    E_ROBOT_COMMAND_NOT_ALLOWED,
    E_SCENE_VERSION_MISMATCH,
    E_SNAPSHOT_MISMATCH,
    STATUS_BLOCKED,
    STATUS_SHADOW_ACCEPTED,
    build_real_scene_shadow_request,
    evaluate_real_scene_shadow_from_contracts,
    evaluate_real_scene_shadow_pipeline,
    format_real_scene_shadow_report,
)
from src.simulation_runtime import run_first_simulation_execution


VALID_TASK = {
    "task_type": "hover_to_object",
    "target_label": "camera",
    "target_world_point": [0.2, 0.1, 0.4],
    "scene_version": "run_simulation_item_001",
    "ttl_ms": 500,
}


def test_valid_real_scene_shadow_pipeline_passes_from_example_config():
    request = build_real_scene_shadow_request(
        requested=True,
        config_path="configs/real_scene_shadow.example.yaml",
    )

    result = evaluate_real_scene_shadow_pipeline(request)

    assert result["shadow_pipeline_status"] == STATUS_SHADOW_ACCEPTED
    assert result["semantic_gate_passed"] is True
    assert result["no_motion_shadow_passed"] is True
    assert result["snapshot_id"] == "example_offline_snapshot_001"
    assert result["grounding_id"] == "example_grounding_001"
    assert result["live_camera_used"] is False
    assert result["live_vlm_called"] is False
    assert result["real_robot_motion_executed"] is False
    assert result["real_robot_command_enabled"] is False
    assert result["robot_command_generated"] is False
    assert result["trajectory_generated"] is False
    assert result["joint_targets_generated"] is False
    assert result["tcp_pose_world_generated"] is False


def test_no_target_grounding_blocks_shadow_pipeline():
    result = _evaluate_shadow(grounding_updates={"grounded": False, "rejected": True, "error_code": E_NO_TARGET})

    assert result["shadow_pipeline_status"] == STATUS_BLOCKED
    assert E_NO_TARGET in result["blocking_reasons"]


def test_low_confidence_blocks_shadow_pipeline():
    result = _evaluate_shadow(grounding_updates={"overall_confidence": 0.42})

    assert result["shadow_pipeline_status"] == STATUS_BLOCKED
    assert E_LOW_CONFIDENCE in result["blocking_reasons"]


def test_invalid_bbox_blocks_shadow_pipeline():
    result = _evaluate_shadow(grounding_updates={"bbox_xyxy": [10, 10, 900, 40]})

    assert result["shadow_pipeline_status"] == STATUS_BLOCKED
    assert E_INVALID_BBOX in result["blocking_reasons"]


def test_invalid_pixel_center_blocks_shadow_pipeline():
    result = _evaluate_shadow(grounding_updates={"pixel_center": [900, 40]})

    assert result["shadow_pipeline_status"] == STATUS_BLOCKED
    assert E_INVALID_PIXEL_CENTER in result["blocking_reasons"]


def test_snapshot_id_mismatch_blocks_shadow_pipeline():
    result = _evaluate_shadow(grounding_updates={"snapshot_id": "different_snapshot"})

    assert result["shadow_pipeline_status"] == STATUS_BLOCKED
    assert E_SNAPSHOT_MISMATCH in result["blocking_reasons"]


def test_scene_version_mismatch_blocks_shadow_pipeline():
    result = _evaluate_shadow(grounding_updates={"scene_version": "different_scene"})

    assert result["shadow_pipeline_status"] == STATUS_BLOCKED
    assert E_SCENE_VERSION_MISMATCH in result["blocking_reasons"]


def test_live_vlm_called_blocks_shadow_pipeline():
    result = _evaluate_shadow(grounding_updates={"live_vlm_called": True})

    assert result["shadow_pipeline_status"] == STATUS_BLOCKED
    assert E_LIVE_VLM_DISABLED in result["blocking_reasons"]


def test_live_camera_used_blocks_shadow_pipeline():
    result = _evaluate_shadow(grounding_updates={"live_camera_used": True})

    assert result["shadow_pipeline_status"] == STATUS_BLOCKED
    assert E_LIVE_CAMERA_DISABLED in result["blocking_reasons"]


def test_robot_control_field_blocks_shadow_pipeline_without_generating_controls():
    result = _evaluate_shadow(grounding_updates={"robot_command": {"joint_targets": [0.0]}})

    assert result["shadow_pipeline_status"] == STATUS_BLOCKED
    assert E_ROBOT_COMMAND_NOT_ALLOWED in result["blocking_reasons"]
    assert result["real_robot_motion_executed"] is False
    assert result["real_robot_command_enabled"] is False
    assert result["robot_command_generated"] is False
    assert result["trajectory_generated"] is False
    assert result["joint_targets_generated"] is False
    assert result["tcp_pose_world_generated"] is False


def test_real_scene_shadow_report_contains_no_motion_safety_statement():
    report = format_real_scene_shadow_report(_evaluate_shadow())

    assert "TETO V2.9.0 Real-Scene No-Motion Shadow Pipeline Report" in report
    assert "does not capture from a live camera" in report
    assert "does not call live Qwen or any live VLM" in report
    assert "does not connect to a real UR5" in report
    assert "does not generate joint targets, trajectories, robot commands, or real execution requests" in report


def test_runtime_manifest_contains_real_scene_shadow_evidence_fields(tmp_path):
    snapshot_path = _write_snapshot(tmp_path, _valid_snapshot())
    grounding_path = _write_grounding(tmp_path, _valid_grounding())
    config_path = _write_shadow_config(tmp_path, snapshot_path, grounding_path)

    run_first_simulation_execution(
        VALID_TASK,
        steps=1,
        run_real_scene_shadow=True,
        real_scene_shadow_config=config_path,
        real_scene_shadow_report=True,
        output_dir=tmp_path,
        write_report=True,
    )

    manifest = json.loads((tmp_path / "evidence_manifest.json").read_text(encoding="utf-8"))
    summary = (tmp_path / "summary.md").read_text(encoding="utf-8")
    report = (tmp_path / "real_scene_shadow_report.md").read_text(encoding="utf-8")

    assert manifest["real_scene_shadow_evidence_available"] is True
    assert manifest["snapshot_id"] == "snapshot_fixture_001"
    assert manifest["grounding_id"] == "grounding_fixture_001"
    assert manifest["shadow_pipeline_status"] == STATUS_SHADOW_ACCEPTED
    assert manifest["semantic_gate_passed"] is True
    assert manifest["real_scene_shadow_semantic_gate_passed"] is True
    assert manifest["no_motion_shadow_passed"] is True
    assert manifest["real_scene_shadow_blocking_reasons"] == []
    assert manifest["live_camera_used"] is False
    assert manifest["live_vlm_called"] is False
    assert manifest["real_robot_motion_executed"] is False
    assert manifest["real_robot_command_enabled"] is False
    assert manifest["robot_command_generated"] is False
    assert manifest["trajectory_generated"] is False
    assert manifest["joint_targets_generated"] is False
    assert manifest["tcp_pose_world_generated"] is False
    assert "real_scene_shadow_report.md" in [item["name"] for item in manifest["real_scene_shadow_evidence_files"]]
    assert "## Real-Scene Shadow Pipeline Summary" in summary
    assert "shadow_pipeline_status: SHADOW_ACCEPTED" in summary
    assert "does not capture live camera frames" in summary
    assert "No-Motion Safety Boundary" in report


def test_cli_real_scene_shadow_arguments_parse():
    from scripts.harnesses.run_shadow_simulation_contract import build_parser

    parser = build_parser()
    args = parser.parse_args(
        [
            "--run-real-scene-shadow",
            "--real-scene-shadow-config",
            "configs/real_scene_shadow.example.yaml",
            "--grounding-result",
            "examples/grounding_result_no_target_example.json",
            "--real-scene-shadow-report",
        ]
    )

    assert args.run_real_scene_shadow is True
    assert args.real_scene_shadow_config == "configs/real_scene_shadow.example.yaml"
    assert args.grounding_result == "examples/grounding_result_no_target_example.json"
    assert args.real_scene_shadow_report is True


def _evaluate_shadow(grounding_updates=None):
    snapshot = evaluate_camera_snapshot_contract(
        CameraSnapshotRequest(requested=True, snapshot=_valid_snapshot()),
        now=datetime(2026, 6, 2, tzinfo=timezone.utc),
    )
    grounding_payload = _valid_grounding()
    grounding_payload.update(grounding_updates or {})
    grounding = evaluate_grounding_result_contract(
        GroundingResultRequest(requested=True, result=grounding_payload)
    )
    return evaluate_real_scene_shadow_from_contracts(snapshot, grounding)


def _write_snapshot(tmp_path, snapshot):
    path = tmp_path / "camera_snapshot.yaml"
    path.write_text(yaml.safe_dump({"camera_snapshot": snapshot}), encoding="utf-8")
    return path


def _write_grounding(tmp_path, grounding):
    path = tmp_path / "grounding_result.json"
    path.write_text(json.dumps({"grounding_result": grounding}, indent=2), encoding="utf-8")
    return path


def _write_shadow_config(tmp_path, snapshot_path, grounding_path):
    path = tmp_path / "real_scene_shadow.yaml"
    path.write_text(
        yaml.safe_dump(
            {
                "camera_snapshot_config": str(snapshot_path),
                "grounding_result": str(grounding_path),
                "overall_confidence_threshold": 0.6,
            }
        ),
        encoding="utf-8",
    )
    return path


def _valid_snapshot():
    return {
        "snapshot_id": "snapshot_fixture_001",
        "scene_version": "fixture_scene_v1",
        "capture_timestamp": "2026-06-02T00:00:00Z",
        "ttl_ms": 315360000000,
        "source": "offline_file",
        "frame_id": "fixture_frame_001",
        "image_ref": "data/processed/examples/offline_rgb_placeholder.jpg",
        "depth_ref": "data/processed/examples/offline_depth_placeholder.png",
        "camera_info_ref": "data/processed/examples/camera_info_placeholder.json",
        "metadata_ref": "data/processed/examples/camera_metadata_placeholder.json",
        "extrinsics_ref": "data/processed/examples/camera_extrinsics_placeholder.json",
        "width": 640,
        "height": 480,
        "color_encoding": "rgb8",
        "depth_encoding": "uint16_mm",
        "camera_frame": "camera_color_optical_frame",
        "alignment_status": "aligned_rgb_depth",
        "sync_status": "offline_manifest",
        "depth_available": True,
        "camera_info_available": True,
        "metadata_available": True,
        "extrinsics_available": True,
        "depth_required": True,
        "live_camera_enabled": False,
    }


def _valid_grounding():
    return {
        "grounding_id": "grounding_fixture_001",
        "snapshot_id": "snapshot_fixture_001",
        "scene_version": "fixture_scene_v1",
        "source": "mock_grounding",
        "user_command": "hover to the camera",
        "target_label": "camera",
        "target_object_id": "fixture_camera_001",
        "bbox_xyxy": [120, 90, 260, 260],
        "pixel_center": [190, 175],
        "mask_ref": "data/processed/examples/offline_mask_placeholder.png",
        "semantic_confidence": 0.91,
        "grounding_confidence": 0.89,
        "overall_confidence": 0.88,
        "grounded": True,
        "rejected": False,
        "warnings": [],
        "live_vlm_called": False,
        "live_camera_used": False,
    }
