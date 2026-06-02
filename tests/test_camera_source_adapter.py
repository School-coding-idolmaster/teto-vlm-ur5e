import json

import yaml

from src.camera_source_adapter import (
    E_CAMERA_FRAME_MISSING,
    E_CAMERA_INFO_MISSING,
    E_CONTINUOUS_CAPTURE_DISABLED,
    E_IMAGE_REF_MISSING,
    E_LIVE_CAMERA_CAPTURE_NOT_ALLOWED,
    E_LIVE_VLM_DISABLED,
    E_ROBOT_COMMAND_NOT_ALLOWED,
    MODE_LIVE_DISABLED,
    MODE_MANUAL_SNAPSHOT,
    MODE_OFFLINE_FILE,
    MODE_REALSENSE_ONE_SHOT,
    STATUS_BLOCKED,
    STATUS_PASS,
    STATUS_SAFE_DISABLED,
    CameraSourceAdapterRequest,
    build_camera_source_adapter_request,
    evaluate_camera_source_adapter,
    format_camera_source_report,
)
from src.simulation_runtime import run_first_simulation_execution


VALID_TASK = {
    "task_type": "hover_to_object",
    "target_label": "camera",
    "target_world_point": [0.2, 0.1, 0.4],
    "scene_version": "run_simulation_item_001",
    "ttl_ms": 500,
}


def test_offline_file_source_passes():
    result = _evaluate_camera_source(source_mode=MODE_OFFLINE_FILE)

    assert result["camera_source_status"] == STATUS_PASS
    assert result["source_mode"] == MODE_OFFLINE_FILE
    assert result["no_motion_camera_adapter_passed"] is True
    assert result["one_shot_capture_used"] is False
    assert result["continuous_capture_used"] is False
    assert result["live_camera_capture_allowed"] is False
    assert result["live_camera_capture_used"] is False
    assert result["live_vlm_called"] is False
    assert result["real_robot_motion_executed"] is False
    assert result["robot_command_generated"] is False
    assert result["trajectory_generated"] is False
    assert result["joint_targets_generated"] is False
    assert result["tcp_pose_world_generated"] is False


def test_manual_snapshot_source_passes():
    result = _evaluate_camera_source(source_mode=MODE_MANUAL_SNAPSHOT)

    assert result["camera_source_status"] == STATUS_PASS
    assert result["source_mode"] == MODE_MANUAL_SNAPSHOT
    assert result["capture_method"] == "declared_snapshot_manifest"


def test_live_disabled_source_is_safe_disabled():
    result = _evaluate_camera_source(source_mode=MODE_LIVE_DISABLED)

    assert result["camera_source_status"] == STATUS_SAFE_DISABLED
    assert result["source_mode"] == MODE_LIVE_DISABLED
    assert result["no_motion_camera_adapter_passed"] is True
    assert result["live_camera_capture_used"] is False


def test_realsense_one_shot_without_allow_flag_blocks():
    result = _evaluate_camera_source(source_mode=MODE_REALSENSE_ONE_SHOT)

    assert result["camera_source_status"] == STATUS_BLOCKED
    assert E_LIVE_CAMERA_CAPTURE_NOT_ALLOWED in result["blocking_reasons"]
    assert result["one_shot_capture_used"] is False


def test_continuous_capture_requested_blocks():
    result = _evaluate_camera_source(config_updates={"continuous_capture_enabled": True})

    assert result["camera_source_status"] == STATUS_BLOCKED
    assert E_CONTINUOUS_CAPTURE_DISABLED in result["blocking_reasons"]
    assert result["continuous_capture_used"] is False


def test_missing_image_ref_blocks():
    result = _evaluate_camera_source(config_updates={"image_ref": None})

    assert result["camera_source_status"] == STATUS_BLOCKED
    assert E_IMAGE_REF_MISSING in result["blocking_reasons"]


def test_missing_camera_info_ref_when_required_blocks():
    result = _evaluate_camera_source(config_updates={"camera_info_ref": None})

    assert result["camera_source_status"] == STATUS_BLOCKED
    assert E_CAMERA_INFO_MISSING in result["blocking_reasons"]


def test_missing_frame_id_or_camera_frame_blocks():
    result = _evaluate_camera_source(config_updates={"frame_id": None})

    assert result["camera_source_status"] == STATUS_BLOCKED
    assert E_CAMERA_FRAME_MISSING in result["blocking_reasons"]


def test_live_vlm_called_blocks():
    result = _evaluate_camera_source(config_updates={"live_vlm_called": True})

    assert result["camera_source_status"] == STATUS_BLOCKED
    assert E_LIVE_VLM_DISABLED in result["blocking_reasons"]


def test_robot_control_field_blocks_without_generating_controls():
    result = _evaluate_camera_source(config_updates={"trajectory": {"joint_targets": [0.0]}})

    assert result["camera_source_status"] == STATUS_BLOCKED
    assert E_ROBOT_COMMAND_NOT_ALLOWED in result["blocking_reasons"]
    assert result["robot_command_generated"] is False
    assert result["trajectory_generated"] is False
    assert result["joint_targets_generated"] is False
    assert result["tcp_pose_world_generated"] is False


def test_example_configs_smoke():
    offline = evaluate_camera_source_adapter(
        build_camera_source_adapter_request(
            requested=True,
            config_path="configs/camera_source_offline.example.yaml",
        )
    )
    manual = evaluate_camera_source_adapter(
        build_camera_source_adapter_request(
            requested=True,
            config_path="configs/camera_source_manual.example.yaml",
        )
    )
    disabled = evaluate_camera_source_adapter(
        build_camera_source_adapter_request(
            requested=True,
            config_path="configs/camera_source_live_disabled.example.yaml",
        )
    )

    assert offline["camera_source_status"] == STATUS_PASS
    assert manual["camera_source_status"] == STATUS_PASS
    assert disabled["camera_source_status"] == STATUS_SAFE_DISABLED


def test_report_contains_no_motion_no_live_vlm_no_real_robot_no_ros2_no_moveit_statement():
    report = format_camera_source_report(_evaluate_camera_source())

    assert "TETO V2.9.3 Camera Source Adapter Report" in report
    assert "no-motion" in report
    assert "no-live-VLM" in report
    assert "no-real-robot" in report
    assert "no-ROS2" in report
    assert "no-MoveIt" in report
    assert "not a continuous live camera loop" in report
    assert "does not generate joint targets, trajectories, robot commands, or real execution requests" in report


def test_runtime_manifest_contains_camera_source_evidence_fields(tmp_path):
    config_path = _write_camera_source_config(tmp_path, _valid_camera_source_config())

    run_first_simulation_execution(
        VALID_TASK,
        steps=1,
        check_camera_source_adapter=True,
        camera_source_config=config_path,
        camera_source_report=True,
        output_dir=tmp_path,
        write_report=True,
    )

    manifest = json.loads((tmp_path / "evidence_manifest.json").read_text(encoding="utf-8"))
    summary = (tmp_path / "summary.md").read_text(encoding="utf-8")
    report = (tmp_path / "camera_source_report.md").read_text(encoding="utf-8")

    assert manifest["camera_source_evidence_available"] is True
    assert manifest["camera_source_status"] == STATUS_PASS
    assert manifest["source_mode"] == MODE_OFFLINE_FILE
    assert manifest["snapshot_id"] == "camera_source_fixture_snapshot_001"
    assert manifest["scene_version"] == "camera_source_fixture_scene_v1"
    assert manifest["capture_timestamp"] == "2099-01-01T00:00:00Z"
    assert manifest["frame_id"] == "camera_source_fixture_frame_001"
    assert manifest["camera_frame"] == "camera_color_optical_frame"
    assert manifest["image_ref"] == "examples/camera_source/sample_image_ref.json"
    assert manifest["depth_ref"] == "examples/camera_source/sample_depth_ref.json"
    assert manifest["camera_info_ref"] == "examples/camera_source/sample_camera_info.json"
    assert manifest["metadata_ref"] == "examples/camera_source/sample_metadata.json"
    assert manifest["extrinsics_ref"] == "examples/camera_source/sample_extrinsics.json"
    assert manifest["depth_available"] is True
    assert manifest["camera_info_available"] is True
    assert manifest["one_shot_capture_used"] is False
    assert manifest["continuous_capture_used"] is False
    assert manifest["live_camera_capture_allowed"] is False
    assert manifest["live_camera_capture_used"] is False
    assert manifest["live_vlm_called"] is False
    assert manifest["real_robot_motion_executed"] is False
    assert manifest["real_robot_command_enabled"] is False
    assert manifest["robot_command_generated"] is False
    assert manifest["trajectory_generated"] is False
    assert manifest["joint_targets_generated"] is False
    assert manifest["tcp_pose_world_generated"] is False
    assert manifest["no_motion_camera_adapter_passed"] is True
    assert manifest["blocking_reasons"] == []
    assert manifest["warnings"] == []
    assert "camera_source_report.md" in [item["name"] for item in manifest["camera_source_evidence_files"]]
    assert "## Camera Source Evidence Summary" in summary
    assert "camera_source_status: PASS" in summary
    assert "no-live-VLM" in report
    assert "no-real-robot" in report


def test_cli_camera_source_arguments_parse():
    from scripts.run_first_simulation_execution import build_parser

    parser = build_parser()
    args = parser.parse_args(
        [
            "--check-camera-source-adapter",
            "--camera-source-config",
            "configs/camera_source_offline.example.yaml",
            "--camera-source-report",
            "--allow-live-camera-capture",
            "--camera-source-mode",
            "manual_snapshot",
        ]
    )

    assert args.check_camera_source_adapter is True
    assert args.camera_source_config == "configs/camera_source_offline.example.yaml"
    assert args.camera_source_report is True
    assert args.allow_live_camera_capture is True
    assert args.camera_source_mode == "manual_snapshot"


def _evaluate_camera_source(source_mode=MODE_OFFLINE_FILE, config_updates=None):
    config = _valid_camera_source_config()
    config["source_mode"] = source_mode
    config.update(config_updates or {})
    return evaluate_camera_source_adapter(CameraSourceAdapterRequest(requested=True, config=config))


def _write_camera_source_config(tmp_path, config):
    path = tmp_path / "camera_source.yaml"
    path.write_text(yaml.safe_dump({"camera_source_adapter": config}), encoding="utf-8")
    return path


def _valid_camera_source_config():
    return {
        "source_mode": MODE_OFFLINE_FILE,
        "snapshot_id": "camera_source_fixture_snapshot_001",
        "scene_version": "camera_source_fixture_scene_v1",
        "capture_timestamp": "2099-01-01T00:00:00Z",
        "frame_id": "camera_source_fixture_frame_001",
        "camera_frame": "camera_color_optical_frame",
        "image_ref": "examples/camera_source/sample_image_ref.json",
        "depth_ref": "examples/camera_source/sample_depth_ref.json",
        "camera_info_ref": "examples/camera_source/sample_camera_info.json",
        "metadata_ref": "examples/camera_source/sample_metadata.json",
        "extrinsics_ref": "examples/camera_source/sample_extrinsics.json",
        "width": 640,
        "height": 480,
        "color_encoding": "rgb8",
        "depth_encoding": "uint16_mm",
        "depth_available": True,
        "camera_info_available": True,
        "metadata_available": True,
        "extrinsics_available": True,
        "depth_required": True,
        "alignment_status": "aligned_rgb_depth",
        "sync_status": "offline_manifest",
        "capture_backend": "offline_file",
        "capture_method": "declared_snapshot_manifest",
        "allow_live_camera_capture": False,
        "continuous_capture_enabled": False,
        "live_vlm_called": False,
    }
