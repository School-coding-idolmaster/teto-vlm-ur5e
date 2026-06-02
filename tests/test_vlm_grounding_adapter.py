import json

import yaml

from src.simulation_runtime import run_first_simulation_execution
from src.vlm_grounding_adapter import (
    E_BBOX_MISSING,
    E_LIVE_VLM_DISABLED,
    E_LOW_CONFIDENCE,
    E_NO_TARGET,
    E_PIXEL_CENTER_MISSING,
    E_ROBOT_COMMAND_NOT_ALLOWED,
    E_SCENE_VERSION_MISMATCH,
    E_SNAPSHOT_MISMATCH,
    E_UNSUPPORTED_COMMAND,
    MODE_FUTURE_LOCAL_QWEN_ADAPTER,
    MODE_LOCAL_VLM_DISABLED,
    MODE_MANUAL_ANNOTATION,
    MODE_MOCK_VLM,
    MODE_OFFLINE_GROUNDING_JSON,
    STATUS_BLOCKED,
    STATUS_PASS,
    STATUS_SAFE_DISABLED,
    VLMGroundingAdapterRequest,
    build_vlm_grounding_adapter_request,
    evaluate_vlm_grounding_adapter,
    format_vlm_grounding_report,
)


VALID_TASK = {
    "task_type": "hover_to_object",
    "target_label": "camera",
    "target_world_point": [0.2, 0.1, 0.4],
    "scene_version": "run_simulation_item_001",
    "ttl_ms": 500,
}


def test_mock_command_hover_over_red_mug_passes():
    result = _evaluate_vlm_grounding()

    assert result["vlm_grounding_status"] == STATUS_PASS
    assert result["adapter_mode"] == MODE_MOCK_VLM
    assert result["user_command"] == "hover over the red mug"
    assert result["normalized_command"] == "hover over the red mug"
    assert result["target_label"] == "red_mug"
    assert result["bbox_xyxy"] == [120, 80, 260, 220]
    assert result["pixel_center"] == [190, 150]
    assert result["semantic_confidence"] == 0.90
    assert result["grounding_confidence"] == 0.88
    assert result["overall_confidence"] == 0.89
    assert result["grounded"] is True
    assert result["rejected"] is False
    assert result["no_motion_grounding_passed"] is True
    assert result["live_vlm_called"] is False
    assert result["live_camera_used"] is False
    assert result["real_robot_motion_executed"] is False
    assert result["real_robot_command_enabled"] is False
    assert result["robot_command_generated"] is False
    assert result["trajectory_generated"] is False
    assert result["joint_targets_generated"] is False
    assert result["tcp_pose_world_generated"] is False


def test_offline_grounding_json_passes_from_example_config():
    result = evaluate_vlm_grounding_adapter(
        build_vlm_grounding_adapter_request(
            requested=True,
            config_path="configs/vlm_grounding_offline.example.yaml",
        )
    )

    assert result["vlm_grounding_status"] == STATUS_PASS
    assert result["adapter_mode"] == MODE_OFFLINE_GROUNDING_JSON
    assert result["grounding_id"] == "offline_grounding_red_mug_001"
    assert result["snapshot_id"] == "vlm_grounding_fixture_snapshot_001"
    assert result["scene_version"] == "vlm_grounding_fixture_scene_v1"
    assert result["grounded"] is True
    assert result["live_vlm_called"] is False


def test_manual_annotation_passes():
    result = _evaluate_vlm_grounding(
        config_updates={
            "adapter_mode": MODE_MANUAL_ANNOTATION,
            "manual_annotation": _valid_grounding_payload(grounding_id="manual_grounding_red_mug_001"),
        }
    )

    assert result["vlm_grounding_status"] == STATUS_PASS
    assert result["adapter_mode"] == MODE_MANUAL_ANNOTATION
    assert result["grounding_id"] == "manual_grounding_red_mug_001"
    assert result["grounded"] is True


def test_local_vlm_disabled_is_safe_disabled_without_live_call():
    result = _evaluate_vlm_grounding(config_updates={"adapter_mode": MODE_LOCAL_VLM_DISABLED})

    assert result["vlm_grounding_status"] in {STATUS_SAFE_DISABLED, STATUS_BLOCKED}
    assert result["live_vlm_called"] is False
    assert result["no_motion_grounding_passed"] is True


def test_unsupported_mock_command_blocks():
    result = _evaluate_vlm_grounding(config_updates={"user_command": "brew coffee in the red mug"})

    assert result["vlm_grounding_status"] == STATUS_BLOCKED
    assert E_UNSUPPORTED_COMMAND in result["blocking_reasons"]
    assert result["error_code"] == E_UNSUPPORTED_COMMAND


def test_no_target_blocks():
    result = _evaluate_vlm_grounding(
        config_updates={
            "adapter_mode": MODE_MANUAL_ANNOTATION,
            "manual_annotation": {
                **_valid_grounding_payload(),
                "grounded": False,
                "rejected": True,
                "bbox_xyxy": None,
                "pixel_center": None,
                "target_label": None,
                "target_object_id": None,
                "error_code": E_NO_TARGET,
            },
        }
    )

    assert result["vlm_grounding_status"] == STATUS_BLOCKED
    assert E_NO_TARGET in result["blocking_reasons"]


def test_low_confidence_blocks():
    result = _evaluate_vlm_grounding(config_updates={"overall_confidence": 0.42})

    assert result["vlm_grounding_status"] == STATUS_BLOCKED
    assert E_LOW_CONFIDENCE in result["blocking_reasons"]


def test_missing_bbox_blocks():
    result = _evaluate_vlm_grounding(
        config_updates={
            "adapter_mode": MODE_MANUAL_ANNOTATION,
            "manual_annotation": {**_valid_grounding_payload(), "bbox_xyxy": None},
        }
    )

    assert result["vlm_grounding_status"] == STATUS_BLOCKED
    assert E_BBOX_MISSING in result["blocking_reasons"]


def test_missing_pixel_center_blocks():
    result = _evaluate_vlm_grounding(
        config_updates={
            "adapter_mode": MODE_MANUAL_ANNOTATION,
            "manual_annotation": {**_valid_grounding_payload(), "pixel_center": None},
        }
    )

    assert result["vlm_grounding_status"] == STATUS_BLOCKED
    assert E_PIXEL_CENTER_MISSING in result["blocking_reasons"]


def test_snapshot_id_mismatch_blocks():
    result = _evaluate_vlm_grounding(config_updates={"expected_snapshot_id": "different_snapshot"})

    assert result["vlm_grounding_status"] == STATUS_BLOCKED
    assert E_SNAPSHOT_MISMATCH in result["blocking_reasons"]


def test_scene_version_mismatch_blocks():
    result = _evaluate_vlm_grounding(config_updates={"expected_scene_version": "different_scene"})

    assert result["vlm_grounding_status"] == STATUS_BLOCKED
    assert E_SCENE_VERSION_MISMATCH in result["blocking_reasons"]


def test_live_vlm_called_blocks_but_result_never_marks_live_call_executed():
    result = _evaluate_vlm_grounding(config_updates={"live_vlm_called": True})

    assert result["vlm_grounding_status"] == STATUS_BLOCKED
    assert E_LIVE_VLM_DISABLED in result["blocking_reasons"]
    assert result["error_code"] == E_LIVE_VLM_DISABLED
    assert result["live_vlm_called"] is False


def test_future_local_qwen_adapter_is_declaration_only_without_model_call():
    result = _evaluate_vlm_grounding(config_updates={"adapter_mode": MODE_FUTURE_LOCAL_QWEN_ADAPTER})

    assert result["vlm_grounding_status"] == STATUS_SAFE_DISABLED
    assert result["source_declaration"]["declaration_only"] is True
    assert result["source_declaration"]["live_model_invocation_supported"] is False
    assert result["live_vlm_called"] is False


def test_robot_control_fields_block_without_generating_controls():
    result = _evaluate_vlm_grounding(config_updates={"trajectory": {"joint_targets": [0.0]}})

    assert result["vlm_grounding_status"] == STATUS_BLOCKED
    assert E_ROBOT_COMMAND_NOT_ALLOWED in result["blocking_reasons"]
    assert result["robot_command_generated"] is False
    assert result["trajectory_generated"] is False
    assert result["joint_targets_generated"] is False
    assert result["tcp_pose_world_generated"] is False


def test_robot_command_tcp_pose_world_and_joint_target_fields_block():
    for field_name in ["robot_command", "tcp_pose_world", "joint_target"]:
        result = _evaluate_vlm_grounding(config_updates={field_name: [0.0]})

        assert result["vlm_grounding_status"] == STATUS_BLOCKED
        assert E_ROBOT_COMMAND_NOT_ALLOWED in result["blocking_reasons"]


def test_example_configs_smoke():
    mock = evaluate_vlm_grounding_adapter(
        build_vlm_grounding_adapter_request(
            requested=True,
            config_path="configs/vlm_grounding_mock.example.yaml",
        )
    )
    offline = evaluate_vlm_grounding_adapter(
        build_vlm_grounding_adapter_request(
            requested=True,
            config_path="configs/vlm_grounding_offline.example.yaml",
        )
    )
    disabled = evaluate_vlm_grounding_adapter(
        build_vlm_grounding_adapter_request(
            requested=True,
            config_path="configs/vlm_grounding_disabled.example.yaml",
        )
    )

    assert mock["vlm_grounding_status"] == STATUS_PASS
    assert offline["vlm_grounding_status"] == STATUS_PASS
    assert disabled["vlm_grounding_status"] == STATUS_SAFE_DISABLED


def test_report_contains_no_motion_no_live_vlm_no_real_robot_no_ros2_no_moveit_statement():
    report = format_vlm_grounding_report(_evaluate_vlm_grounding())

    assert "TETO V2.9.4 VLM Grounding Adapter Report" in report
    assert "no-motion" in report
    assert "no-live-VLM" in report
    assert "no-real-robot" in report
    assert "no-ROS2" in report
    assert "no-MoveIt" in report
    assert "does not call live Qwen or any live VLM" in report
    assert "does not generate joint targets, trajectories, robot commands, or real execution requests" in report


def test_runtime_manifest_contains_vlm_grounding_evidence_fields(tmp_path):
    config_path = _write_vlm_grounding_config(tmp_path, _valid_vlm_grounding_config())

    run_first_simulation_execution(
        VALID_TASK,
        steps=1,
        check_vlm_grounding_adapter=True,
        vlm_grounding_config=config_path,
        vlm_grounding_report=True,
        output_dir=tmp_path,
        write_report=True,
    )

    manifest = json.loads((tmp_path / "evidence_manifest.json").read_text(encoding="utf-8"))
    summary = (tmp_path / "summary.md").read_text(encoding="utf-8")
    report = (tmp_path / "vlm_grounding_report.md").read_text(encoding="utf-8")

    assert (tmp_path / "vlm_grounding_result.json").exists()
    assert manifest["vlm_grounding_evidence_available"] is True
    assert manifest["vlm_grounding_status"] == STATUS_PASS
    assert manifest["grounding_id"] == "mock_grounding_red_mug_001"
    assert manifest["snapshot_id"] == "vlm_grounding_fixture_snapshot_001"
    assert manifest["scene_version"] == "vlm_grounding_fixture_scene_v1"
    assert manifest["user_command"] == "hover over the red mug"
    assert manifest["normalized_command"] == "hover over the red mug"
    assert manifest["adapter_mode"] == MODE_MOCK_VLM
    assert manifest["target_label"] == "red_mug"
    assert manifest["target_object_id"] == "mock_red_mug_001"
    assert manifest["bbox_xyxy"] == [120, 80, 260, 220]
    assert manifest["pixel_center"] == [190, 150]
    assert manifest["semantic_confidence"] == 0.90
    assert manifest["grounding_confidence"] == 0.88
    assert manifest["overall_confidence"] == 0.89
    assert manifest["grounded"] is True
    assert manifest["rejected"] is False
    assert manifest["rejection_reason"] is None
    assert manifest["error_code"] is None
    assert manifest["no_motion_grounding_passed"] is True
    assert manifest["blocking_reasons"] == []
    assert manifest["warnings"] == []
    assert manifest["live_camera_used"] is False
    assert manifest["live_vlm_called"] is False
    assert manifest["real_robot_motion_executed"] is False
    assert manifest["real_robot_command_enabled"] is False
    assert manifest["robot_command_generated"] is False
    assert manifest["trajectory_generated"] is False
    assert manifest["joint_targets_generated"] is False
    assert manifest["tcp_pose_world_generated"] is False
    assert "vlm_grounding_report.md" in [item["name"] for item in manifest["vlm_grounding_evidence_files"]]
    assert "## VLM Grounding Evidence Summary" in summary
    assert "vlm_grounding_status: PASS" in summary
    assert "no-live-VLM" in report
    assert "no-real-robot" in report


def test_cli_vlm_grounding_arguments_parse():
    from scripts.run_first_simulation_execution import build_parser

    parser = build_parser()
    args = parser.parse_args(
        [
            "--check-vlm-grounding-adapter",
            "--vlm-grounding-config",
            "configs/vlm_grounding_mock.example.yaml",
            "--vlm-grounding-report",
            "--user-command",
            "hover over the red mug",
            "--allow-live-vlm",
        ]
    )

    assert args.check_vlm_grounding_adapter is True
    assert args.vlm_grounding_config == "configs/vlm_grounding_mock.example.yaml"
    assert args.vlm_grounding_report is True
    assert args.user_command == "hover over the red mug"
    assert args.allow_live_vlm is True


def _evaluate_vlm_grounding(config_updates=None):
    config = _valid_vlm_grounding_config()
    config.update(config_updates or {})
    return evaluate_vlm_grounding_adapter(VLMGroundingAdapterRequest(requested=True, config=config))


def _write_vlm_grounding_config(tmp_path, config):
    path = tmp_path / "vlm_grounding.yaml"
    path.write_text(yaml.safe_dump({"vlm_grounding_adapter": config}), encoding="utf-8")
    return path


def _valid_vlm_grounding_config():
    return {
        "adapter_mode": MODE_MOCK_VLM,
        "snapshot_id": "vlm_grounding_fixture_snapshot_001",
        "scene_version": "vlm_grounding_fixture_scene_v1",
        "user_command": "hover over the red mug",
        "expected_snapshot_id": "vlm_grounding_fixture_snapshot_001",
        "expected_scene_version": "vlm_grounding_fixture_scene_v1",
        "overall_confidence_threshold": 0.60,
        "allow_live_vlm": False,
        "live_vlm_called": False,
    }


def _valid_grounding_payload(grounding_id="manual_grounding_001"):
    return {
        "grounding_id": grounding_id,
        "snapshot_id": "vlm_grounding_fixture_snapshot_001",
        "scene_version": "vlm_grounding_fixture_scene_v1",
        "user_command": "hover over the red mug",
        "target_label": "red_mug",
        "target_object_id": "manual_red_mug_001",
        "bbox_xyxy": [120, 80, 260, 220],
        "pixel_center": [190, 150],
        "mask_ref": None,
        "semantic_confidence": 0.9,
        "grounding_confidence": 0.88,
        "overall_confidence": 0.89,
        "grounded": True,
        "rejected": False,
        "live_vlm_called": False,
        "live_camera_used": False,
    }
