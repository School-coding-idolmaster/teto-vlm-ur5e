import json

import yaml

from src.camera_snapshot import CameraSnapshotRequest, evaluate_camera_snapshot_contract
from src.geometry_validity import (
    E_BBOX_MISSING,
    E_INVALID_BBOX,
    E_INVALID_BBOX_AREA,
    E_INVALID_BBOX_FORMAT,
    E_INVALID_PIXEL_CENTER,
    E_LIVE_CAMERA_DISABLED,
    E_LIVE_VLM_DISABLED,
    E_LOW_CONFIDENCE,
    E_NO_DEPTH,
    E_NO_TARGET,
    E_PIXEL_CENTER_MISSING,
    E_ROBOT_COMMAND_NOT_ALLOWED,
    E_SCENE_VERSION_MISMATCH,
    E_SNAPSHOT_MISMATCH,
    STATUS_BLOCKED,
    STATUS_PASS,
    GeometryValidityRequest,
    build_geometry_validity_request,
    evaluate_geometry_validity,
    evaluate_geometry_validity_from_contracts,
    format_geometry_validity_report,
)
from src.grounding_result import GroundingResultRequest, evaluate_grounding_result_contract
from src.simulation_runtime import run_first_simulation_execution


VALID_TASK = {
    "task_type": "hover_to_object",
    "target_label": "camera",
    "target_world_point": [0.2, 0.1, 0.4],
    "scene_version": "run_simulation_item_001",
    "ttl_ms": 500,
}


def test_valid_snapshot_and_grounding_pass_geometry_validity():
    result = _evaluate_geometry()

    assert result["geometry_validity_status"] == STATUS_PASS
    assert result["no_motion_geometry_passed"] is True
    assert result["bbox_valid"] is True
    assert result["pixel_center_valid"] is True
    assert result["bbox_inside_image"] is True
    assert result["pixel_center_inside_image"] is True
    assert result["confidence_check_passed"] is True
    assert result["ttl_check_passed"] is True
    assert result["depth_required"] is True
    assert result["depth_available"] is True
    assert result["live_camera_used"] is False
    assert result["live_vlm_called"] is False
    assert result["real_robot_motion_executed"] is False
    assert result["real_robot_command_enabled"] is False
    assert result["robot_command_generated"] is False
    assert result["trajectory_generated"] is False
    assert result["joint_targets_generated"] is False
    assert result["tcp_pose_world_generated"] is False


def test_bbox_missing_blocks_geometry_validity():
    result = _evaluate_geometry(grounding_updates={"bbox_xyxy": None})

    assert result["geometry_validity_status"] == STATUS_BLOCKED
    assert E_BBOX_MISSING in result["blocking_reasons"]


def test_bbox_invalid_format_blocks_geometry_validity():
    result = _evaluate_geometry(grounding_updates={"bbox_xyxy": [10, 20, 30]})

    assert result["geometry_validity_status"] == STATUS_BLOCKED
    assert E_INVALID_BBOX_FORMAT in result["blocking_reasons"]


def test_bbox_out_of_image_blocks_geometry_validity():
    result = _evaluate_geometry(grounding_updates={"bbox_xyxy": [10, 20, 900, 120]})

    assert result["geometry_validity_status"] == STATUS_BLOCKED
    assert E_INVALID_BBOX in result["blocking_reasons"]


def test_bbox_zero_area_blocks_geometry_validity():
    result = _evaluate_geometry(grounding_updates={"bbox_xyxy": [10, 20, 10, 120]})

    assert result["geometry_validity_status"] == STATUS_BLOCKED
    assert E_INVALID_BBOX_AREA in result["blocking_reasons"]


def test_pixel_center_missing_blocks_geometry_validity():
    result = _evaluate_geometry(grounding_updates={"pixel_center": None})

    assert result["geometry_validity_status"] == STATUS_BLOCKED
    assert E_PIXEL_CENTER_MISSING in result["blocking_reasons"]


def test_pixel_center_out_of_image_blocks_geometry_validity():
    result = _evaluate_geometry(grounding_updates={"pixel_center": [900, 120]})

    assert result["geometry_validity_status"] == STATUS_BLOCKED
    assert E_INVALID_PIXEL_CENTER in result["blocking_reasons"]


def test_low_confidence_blocks_geometry_validity():
    result = _evaluate_geometry(grounding_updates={"overall_confidence": 0.42})

    assert result["geometry_validity_status"] == STATUS_BLOCKED
    assert E_LOW_CONFIDENCE in result["blocking_reasons"]


def test_no_target_blocks_geometry_validity():
    result = _evaluate_geometry(grounding_updates={"grounded": False, "rejected": True, "error_code": E_NO_TARGET})

    assert result["geometry_validity_status"] == STATUS_BLOCKED
    assert E_NO_TARGET in result["blocking_reasons"]


def test_depth_required_but_depth_unavailable_blocks_geometry_validity():
    result = _evaluate_geometry(snapshot_updates={"depth_available": False})

    assert result["geometry_validity_status"] == STATUS_BLOCKED
    assert E_NO_DEPTH in result["blocking_reasons"]


def test_scene_version_mismatch_blocks_geometry_validity():
    result = _evaluate_geometry(grounding_updates={"scene_version": "different_scene"})

    assert result["geometry_validity_status"] == STATUS_BLOCKED
    assert E_SCENE_VERSION_MISMATCH in result["blocking_reasons"]


def test_snapshot_id_mismatch_blocks_geometry_validity():
    result = _evaluate_geometry(grounding_updates={"snapshot_id": "different_snapshot"})

    assert result["geometry_validity_status"] == STATUS_BLOCKED
    assert E_SNAPSHOT_MISMATCH in result["blocking_reasons"]


def test_live_camera_used_blocks_geometry_validity():
    result = _evaluate_geometry(grounding_updates={"live_camera_used": True})

    assert result["geometry_validity_status"] == STATUS_BLOCKED
    assert E_LIVE_CAMERA_DISABLED in result["blocking_reasons"]


def test_live_vlm_called_blocks_geometry_validity():
    result = _evaluate_geometry(grounding_updates={"live_vlm_called": True})

    assert result["geometry_validity_status"] == STATUS_BLOCKED
    assert E_LIVE_VLM_DISABLED in result["blocking_reasons"]


def test_robot_control_fields_block_geometry_validity_without_generating_controls():
    result = _evaluate_geometry(grounding_updates={"trajectory": {"joint_targets": [0.0]}})

    assert result["geometry_validity_status"] == STATUS_BLOCKED
    assert E_ROBOT_COMMAND_NOT_ALLOWED in result["blocking_reasons"]
    assert result["robot_command_generated"] is False
    assert result["trajectory_generated"] is False
    assert result["joint_targets_generated"] is False
    assert result["tcp_pose_world_generated"] is False


def test_example_config_smoke_passes():
    result = evaluate_geometry_validity(
        build_geometry_validity_request(
            requested=True,
            config_path="configs/geometry_validity.example.yaml",
        )
    )

    assert result["geometry_validity_status"] == STATUS_PASS
    assert result["snapshot_id"] == "example_offline_snapshot_001"
    assert result["grounding_id"] == "example_grounding_001"


def test_report_contains_no_motion_no_live_camera_no_live_vlm_no_real_robot_statement():
    report = format_geometry_validity_report(_evaluate_geometry())

    assert "TETO V2.9.1 Geometry Validity Contract Report" in report
    assert "no-motion" in report
    assert "no-live-camera" in report
    assert "no-live-VLM" in report
    assert "no-real-robot" in report
    assert "does not generate joint targets, trajectories, robot commands, or real execution requests" in report


def test_runtime_manifest_contains_geometry_validity_evidence_fields(tmp_path):
    snapshot_path = _write_snapshot(tmp_path, _valid_snapshot())
    grounding_path = _write_grounding(tmp_path, _valid_grounding())
    config_path = _write_geometry_config(tmp_path, snapshot_path, grounding_path)

    run_first_simulation_execution(
        VALID_TASK,
        steps=1,
        check_geometry_validity=True,
        geometry_validity_config=config_path,
        geometry_validity_report=True,
        output_dir=tmp_path,
        write_report=True,
    )

    manifest = json.loads((tmp_path / "evidence_manifest.json").read_text(encoding="utf-8"))
    summary = (tmp_path / "summary.md").read_text(encoding="utf-8")
    report = (tmp_path / "geometry_validity_report.md").read_text(encoding="utf-8")

    assert manifest["geometry_validity_evidence_available"] is True
    assert manifest["geometry_validity_requested"] is True
    assert manifest["geometry_validity_status"] == STATUS_PASS
    assert manifest["snapshot_id"] == "snapshot_fixture_001"
    assert manifest["grounding_id"] == "grounding_fixture_001"
    assert manifest["scene_version"] == "fixture_scene_v1"
    assert manifest["bbox_valid"] is True
    assert manifest["pixel_center_valid"] is True
    assert manifest["bbox_inside_image"] is True
    assert manifest["pixel_center_inside_image"] is True
    assert manifest["confidence_check_passed"] is True
    assert manifest["ttl_check_passed"] is True
    assert manifest["depth_required"] is True
    assert manifest["depth_available"] is True
    assert manifest["camera_frame_available"] is True
    assert manifest["blocking_reasons"] == []
    assert manifest["warnings"] == []
    assert manifest["no_motion_geometry_passed"] is True
    assert manifest["live_camera_used"] is False
    assert manifest["live_vlm_called"] is False
    assert manifest["real_robot_motion_executed"] is False
    assert manifest["real_robot_command_enabled"] is False
    assert manifest["robot_command_generated"] is False
    assert manifest["trajectory_generated"] is False
    assert manifest["joint_targets_generated"] is False
    assert manifest["tcp_pose_world_generated"] is False
    assert "geometry_validity_report.md" in [item["name"] for item in manifest["geometry_validity_evidence_files"]]
    assert "## Geometry Validity Evidence Summary" in summary
    assert "geometry_validity_status: PASS" in summary
    assert "no-motion" in report
    assert "no-live-camera" in report
    assert "no-live-VLM" in report


def test_cli_geometry_validity_arguments_parse():
    from scripts.harnesses.run_shadow_simulation_contract import build_parser

    parser = build_parser()
    args = parser.parse_args(
        [
            "--check-geometry-validity",
            "--geometry-validity-config",
            "configs/geometry_validity.example.yaml",
            "--geometry-validity-report",
            "--grounding-result",
            "examples/grounding_result_invalid_bbox_example.json",
            "--camera-snapshot-config",
            "configs/camera_snapshot.example.yaml",
        ]
    )

    assert args.check_geometry_validity is True
    assert args.geometry_validity_config == "configs/geometry_validity.example.yaml"
    assert args.geometry_validity_report is True
    assert args.grounding_result == "examples/grounding_result_invalid_bbox_example.json"
    assert args.camera_snapshot_config == "configs/camera_snapshot.example.yaml"


def _evaluate_geometry(snapshot_updates=None, grounding_updates=None):
    snapshot_payload = _valid_snapshot()
    snapshot_payload.update(snapshot_updates or {})
    snapshot = evaluate_camera_snapshot_contract(
        CameraSnapshotRequest(requested=True, snapshot=snapshot_payload)
    )
    grounding_payload = _valid_grounding()
    grounding_payload.update(grounding_updates or {})
    grounding = evaluate_grounding_result_contract(
        GroundingResultRequest(requested=True, result=grounding_payload)
    )
    return evaluate_geometry_validity_from_contracts(snapshot, grounding)


def _write_snapshot(tmp_path, snapshot):
    path = tmp_path / "camera_snapshot.yaml"
    path.write_text(yaml.safe_dump({"camera_snapshot": snapshot}), encoding="utf-8")
    return path


def _write_grounding(tmp_path, grounding):
    path = tmp_path / "grounding_result.json"
    path.write_text(json.dumps({"grounding_result": grounding}, indent=2), encoding="utf-8")
    return path


def _write_geometry_config(tmp_path, snapshot_path, grounding_path):
    path = tmp_path / "geometry_validity.yaml"
    path.write_text(
        yaml.safe_dump(
            {
                "geometry_validity": {
                    "camera_snapshot_config": str(snapshot_path),
                    "grounding_result": str(grounding_path),
                    "depth_required": True,
                    "thresholds": {"confidence_threshold": 0.6, "min_bbox_area_px": 1.0},
                }
            }
        ),
        encoding="utf-8",
    )
    return path


def _valid_snapshot():
    return {
        "snapshot_id": "snapshot_fixture_001",
        "scene_version": "fixture_scene_v1",
        "capture_timestamp": "2099-01-01T00:00:00Z",
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
