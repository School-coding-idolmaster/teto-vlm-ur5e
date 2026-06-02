import json
from datetime import datetime, timezone

import yaml

from src.camera_snapshot import (
    E_CAMERA_SNAPSHOT_STALE,
    E_DEPTH_REF_MISSING,
    E_IMAGE_REF_MISSING,
    E_LIVE_CAMERA_DISABLED,
    E_ROBOT_COMMAND_NOT_ALLOWED,
    CameraSnapshotRequest,
    build_camera_snapshot_request,
    evaluate_camera_snapshot_contract,
    format_camera_snapshot_report,
)
from src.simulation_runtime import run_first_simulation_execution


VALID_TASK = {
    "task_type": "hover_to_object",
    "target_label": "camera",
    "target_world_point": [0.2, 0.1, 0.4],
    "scene_version": "run_simulation_item_001",
    "ttl_ms": 500,
}


def test_valid_offline_camera_snapshot_passes():
    result = evaluate_camera_snapshot_contract(
        CameraSnapshotRequest(requested=True, snapshot=_valid_snapshot()),
        now=datetime(2026, 6, 2, tzinfo=timezone.utc),
    )

    assert result["validity_status"] == "PASS"
    assert result["no_motion_snapshot_passed"] is True
    assert result["live_capture_used"] is False
    assert result["live_camera_enabled"] is False
    assert result["live_vlm_called"] is False
    assert result["real_robot_motion_executed"] is False
    assert result["real_robot_command_enabled"] is False


def test_missing_image_ref_blocks_snapshot():
    snapshot = _valid_snapshot()
    snapshot["image_ref"] = None

    result = evaluate_camera_snapshot_contract(CameraSnapshotRequest(requested=True, snapshot=snapshot))

    assert result["validity_status"] == "BLOCKED"
    assert E_IMAGE_REF_MISSING in result["blocking_reasons"]


def test_missing_depth_ref_when_required_blocks_snapshot():
    snapshot = _valid_snapshot()
    snapshot["depth_required"] = True
    snapshot["depth_ref"] = None

    result = evaluate_camera_snapshot_contract(CameraSnapshotRequest(requested=True, snapshot=snapshot))

    assert result["validity_status"] == "BLOCKED"
    assert E_DEPTH_REF_MISSING in result["blocking_reasons"]


def test_expired_ttl_blocks_snapshot():
    snapshot = _valid_snapshot()
    snapshot["capture_timestamp"] = "2026-01-01T00:00:00Z"
    snapshot["ttl_ms"] = 1

    result = evaluate_camera_snapshot_contract(
        CameraSnapshotRequest(requested=True, snapshot=snapshot),
        now=datetime(2026, 1, 1, 0, 0, 1, tzinfo=timezone.utc),
    )

    assert result["validity_status"] == "BLOCKED"
    assert E_CAMERA_SNAPSHOT_STALE in result["blocking_reasons"]


def test_live_camera_source_is_blocked():
    snapshot = _valid_snapshot()
    snapshot["source"] = "live_camera"

    result = evaluate_camera_snapshot_contract(CameraSnapshotRequest(requested=True, snapshot=snapshot))

    assert result["validity_status"] == "BLOCKED"
    assert E_LIVE_CAMERA_DISABLED in result["blocking_reasons"]


def test_live_camera_enabled_true_is_blocked():
    snapshot = _valid_snapshot()
    snapshot["live_camera_enabled"] = True

    result = evaluate_camera_snapshot_contract(CameraSnapshotRequest(requested=True, snapshot=snapshot))

    assert result["validity_status"] == "BLOCKED"
    assert E_LIVE_CAMERA_DISABLED in result["blocking_reasons"]


def test_robot_command_field_is_blocked():
    snapshot = _valid_snapshot()
    snapshot["trajectory_plan"] = {"joint_targets": [0.0]}

    result = evaluate_camera_snapshot_contract(CameraSnapshotRequest(requested=True, snapshot=snapshot))

    assert result["validity_status"] == "BLOCKED"
    assert E_ROBOT_COMMAND_NOT_ALLOWED in result["blocking_reasons"]
    assert "trajectory_plan" in result["forbidden_robot_control_fields"]
    assert "trajectory_plan.joint_targets" in result["forbidden_robot_control_fields"]


def test_example_config_smoke_passes():
    request = build_camera_snapshot_request(
        requested=True,
        config_path="configs/camera_snapshot.example.yaml",
    )

    result = evaluate_camera_snapshot_contract(
        request,
        now=datetime(2026, 6, 2, tzinfo=timezone.utc),
    )

    assert result["validity_status"] == "PASS"
    assert result["snapshot_id"] == "example_offline_snapshot_001"
    assert result["source"] == "offline_file"
    assert result["no_motion_snapshot_passed"] is True


def test_report_contains_no_motion_no_live_camera_no_real_robot_statement():
    result = evaluate_camera_snapshot_contract(
        CameraSnapshotRequest(requested=True, snapshot=_valid_snapshot()),
        now=datetime(2026, 6, 2, tzinfo=timezone.utc),
    )

    report = format_camera_snapshot_report(result)

    assert "TETO V2.8.2 Camera Snapshot Contract Report" in report
    assert "does not capture from a live camera" in report
    assert "does not call live Qwen or any live VLM" in report
    assert "does not connect to a real UR5" in report
    assert "does not generate joint targets or robot commands" in report


def test_runtime_manifest_contains_camera_snapshot_evidence_fields(tmp_path):
    config_path = _write_snapshot(tmp_path, _valid_snapshot())

    run_first_simulation_execution(
        VALID_TASK,
        steps=1,
        check_camera_snapshot=True,
        camera_snapshot_config=config_path,
        camera_snapshot_report=True,
        output_dir=tmp_path,
        write_report=True,
    )

    manifest = json.loads((tmp_path / "evidence_manifest.json").read_text(encoding="utf-8"))
    summary = (tmp_path / "summary.md").read_text(encoding="utf-8")
    report = (tmp_path / "camera_snapshot_report.md").read_text(encoding="utf-8")

    assert manifest["camera_snapshot_evidence_available"] is True
    assert manifest["camera_snapshot_id"] == "snapshot_fixture_001"
    assert manifest["scene_version"] == "fixture_scene_v1"
    assert manifest["camera_snapshot_validity_status"] == "PASS"
    assert manifest["camera_snapshot_blocking_reasons"] == []
    assert manifest["camera_snapshot_warnings"] == []
    assert manifest["no_motion_snapshot_passed"] is True
    assert manifest["live_capture_used"] is False
    assert manifest["live_camera_enabled"] is False
    assert manifest["live_vlm_called"] is False
    assert manifest["real_robot_motion_executed"] is False
    assert manifest["real_robot_command_enabled"] is False
    assert "camera_snapshot_report.md" in [item["name"] for item in manifest["camera_snapshot_evidence_files"]]
    assert "## Camera Snapshot Evidence Summary" in summary
    assert "camera_snapshot_validity_status: PASS" in summary
    assert "does not capture a live camera frame" in summary
    assert "No-Motion Safety Boundary" in report


def test_camera_snapshot_evidence_does_not_emit_real_robot_control_payload_fields(tmp_path):
    config_path = _write_snapshot(tmp_path, _valid_snapshot())

    run_first_simulation_execution(
        VALID_TASK,
        steps=1,
        check_camera_snapshot=True,
        camera_snapshot_config=config_path,
        camera_snapshot_report=True,
        output_dir=tmp_path,
        write_report=True,
    )

    combined = "\n".join(
        [
            (tmp_path / "camera_snapshot_result.json").read_text(encoding="utf-8"),
            (tmp_path / "camera_snapshot_report.md").read_text(encoding="utf-8"),
            (tmp_path / "summary.md").read_text(encoding="utf-8"),
            (tmp_path / "evidence_manifest.json").read_text(encoding="utf-8"),
        ]
    )

    for field in (
        "trajectory_command",
        "trajectory_plan",
        "tcp_pose_world_command",
        "urscript_program",
        "dashboard_command",
        "rtde_control_command",
        "moveit_plan",
        "ros2_action_goal",
        "live_camera_frame",
        "live_vlm_response",
        "automatic_retry_motion_command",
    ):
        assert field not in combined


def _write_snapshot(tmp_path, snapshot):
    path = tmp_path / "camera_snapshot.yaml"
    path.write_text(yaml.safe_dump({"camera_snapshot": snapshot}), encoding="utf-8")
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
