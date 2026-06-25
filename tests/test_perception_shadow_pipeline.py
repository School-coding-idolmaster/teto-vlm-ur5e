import json

import yaml

from src.perception_shadow_pipeline import (
    E_LIVE_CAMERA_DISABLED,
    E_LIVE_VLM_DISABLED,
    E_PIPELINE_ID_MISMATCH,
    E_ROBOT_COMMAND_NOT_ALLOWED,
    E_SCENE_VERSION_MISMATCH,
    STATUS_BLOCKED,
    STATUS_PASS,
    PerceptionShadowPipelineRequest,
    build_perception_shadow_request,
    evaluate_perception_shadow_pipeline,
    format_perception_shadow_report,
)
from src.projector_shadow import E_CAMERA_INFO_MISSING, E_INVALID_DEPTH, E_OUT_OF_WORKSPACE
from src.simulation_runtime import run_first_simulation_execution
from src.grounding.vlm_adapter import E_LOW_CONFIDENCE, E_NO_TARGET


VALID_TASK = {
    "task_type": "hover_to_object",
    "target_label": "camera",
    "target_world_point": [0.2, 0.1, 0.4],
    "scene_version": "run_simulation_item_001",
    "ttl_ms": 500,
}


def test_full_mock_positive_pipeline_passes():
    result = _evaluate_config_path("configs/perception_shadow_mock_positive.example.yaml")

    assert result["perception_shadow_status"] == STATUS_PASS
    assert result["camera_source_status"] == STATUS_PASS
    assert result["camera_snapshot_validity_status"] == STATUS_PASS
    assert result["vlm_grounding_status"] == STATUS_PASS
    assert result["real_scene_shadow_status"] == "SHADOW_ACCEPTED"
    assert result["semantic_gate_passed"] is True
    assert result["geometry_validity_status"] == STATUS_PASS
    assert result["projector_status"] == STATUS_PASS
    assert result["target_label"] == "red_mug"
    assert result["bbox_xyxy"] == [120, 80, 260, 220]
    assert result["pixel_center"] == [190, 150]
    assert result["world_point_m"] == [-0.073333, -0.12, 1.0]
    assert result["workspace_check_passed"] is True
    assert result["replay_ready"] is True
    assert result["no_motion_perception_passed"] is True
    assert result["live_camera_used"] is False
    assert result["live_vlm_called"] is False
    assert result["real_robot_motion_executed"] is False
    assert result["robot_command_generated"] is False
    assert result["trajectory_generated"] is False
    assert result["joint_targets_generated"] is False
    assert result["tcp_pose_world_generated"] is False


def test_offline_grounding_positive_pipeline_passes():
    config = _valid_perception_config()
    config["vlm_grounding_adapter"] = {
        "adapter_mode": "offline_grounding_json",
        "grounding_result_path": "examples/vlm_grounding/grounding_red_mug_example.json",
        "user_command": "hover over the red mug",
        "expected_snapshot_id": "vlm_grounding_fixture_snapshot_001",
        "expected_scene_version": "vlm_grounding_fixture_scene_v1",
    }
    config["camera_source_adapter"]["snapshot_id"] = "vlm_grounding_fixture_snapshot_001"
    config["camera_source_adapter"]["scene_version"] = "vlm_grounding_fixture_scene_v1"

    result = _evaluate_config(config)

    assert result["perception_shadow_status"] == STATUS_PASS
    assert result["vlm_grounding_status"] == STATUS_PASS
    assert result["grounding_id"] == "offline_grounding_red_mug_001"


def test_no_target_blocks():
    result = _evaluate_config_path("configs/perception_shadow_no_target.example.yaml")

    assert result["perception_shadow_status"] == STATUS_BLOCKED
    assert E_NO_TARGET in result["blocking_reasons"]


def test_low_confidence_blocks():
    result = _evaluate_config_path("configs/perception_shadow_low_confidence.example.yaml")

    assert result["perception_shadow_status"] == STATUS_BLOCKED
    assert E_LOW_CONFIDENCE in result["blocking_reasons"]


def test_invalid_bbox_blocks():
    result = _evaluate_config_path("configs/perception_shadow_invalid_geometry.example.yaml")

    assert result["perception_shadow_status"] == STATUS_BLOCKED
    assert "E_INVALID_BBOX" in result["blocking_reasons"]


def test_invalid_pixel_center_blocks():
    config = _valid_perception_config()
    config["vlm_grounding_adapter"] = _manual_grounding_config(pixel_center=[900, 150])

    result = _evaluate_config(config)

    assert result["perception_shadow_status"] == STATUS_BLOCKED
    assert "E_INVALID_PIXEL_CENTER" in result["blocking_reasons"]


def test_invalid_depth_blocks():
    result = _evaluate_config_path("configs/perception_shadow_invalid_depth.example.yaml")

    assert result["perception_shadow_status"] == STATUS_BLOCKED
    assert E_INVALID_DEPTH in result["blocking_reasons"]


def test_missing_camera_info_blocks():
    config = _valid_perception_config()
    config["projector_shadow"].pop("camera_info_ref")

    result = _evaluate_config(config)

    assert result["perception_shadow_status"] == STATUS_BLOCKED
    assert E_CAMERA_INFO_MISSING in result["blocking_reasons"]


def test_projector_out_of_workspace_blocks():
    result = _evaluate_config_path("configs/perception_shadow_out_of_workspace.example.yaml")

    assert result["perception_shadow_status"] == STATUS_BLOCKED
    assert E_OUT_OF_WORKSPACE in result["blocking_reasons"]


def test_scene_version_mismatch_between_stages_blocks():
    config = _valid_perception_config()
    config["vlm_grounding_adapter"]["scene_version"] = "different_scene"

    result = _evaluate_config(config)

    assert result["perception_shadow_status"] == STATUS_BLOCKED
    assert E_SCENE_VERSION_MISMATCH in result["blocking_reasons"]


def test_snapshot_id_mismatch_between_stages_blocks():
    config = _valid_perception_config()
    config["vlm_grounding_adapter"]["snapshot_id"] = "different_snapshot"

    result = _evaluate_config(config)

    assert result["perception_shadow_status"] == STATUS_BLOCKED
    assert E_PIPELINE_ID_MISMATCH in result["blocking_reasons"]


def test_live_camera_used_blocks():
    config = _valid_perception_config()
    config["camera_snapshot"] = {
        **_snapshot_from_camera_source(config["camera_source_adapter"]),
        "live_camera_enabled": True,
    }

    result = _evaluate_config(config)

    assert result["perception_shadow_status"] == STATUS_BLOCKED
    assert E_LIVE_CAMERA_DISABLED in result["blocking_reasons"]
    assert result["live_camera_used"] is False


def test_live_vlm_called_blocks():
    config = _valid_perception_config()
    config["vlm_grounding_adapter"]["live_vlm_called"] = True

    result = _evaluate_config(config)

    assert result["perception_shadow_status"] == STATUS_BLOCKED
    assert E_LIVE_VLM_DISABLED in result["blocking_reasons"]
    assert result["live_vlm_called"] is False


def test_robot_control_fields_block_without_generating_controls():
    config = _valid_perception_config()
    config["projector_shadow"]["trajectory"] = {"joint_targets": [0.0]}

    result = _evaluate_config(config)

    assert result["perception_shadow_status"] == STATUS_BLOCKED
    assert E_ROBOT_COMMAND_NOT_ALLOWED in result["blocking_reasons"]
    assert result["robot_command_generated"] is False
    assert result["trajectory_generated"] is False
    assert result["joint_targets_generated"] is False
    assert result["tcp_pose_world_generated"] is False


def test_robot_command_tcp_pose_world_and_joint_target_fields_block():
    for field_name in ["robot_command", "tcp_pose_world", "joint_target"]:
        config = _valid_perception_config()
        config[field_name] = [0.0]

        result = _evaluate_config(config)

        assert result["perception_shadow_status"] == STATUS_BLOCKED
        assert E_ROBOT_COMMAND_NOT_ALLOWED in result["blocking_reasons"]


def test_runtime_manifest_contains_perception_shadow_evidence_fields(tmp_path):
    run_first_simulation_execution(
        VALID_TASK,
        steps=1,
        run_perception_shadow_pipeline=True,
        perception_shadow_config="configs/perception_shadow_mock_positive.example.yaml",
        perception_shadow_report=True,
        output_dir=tmp_path,
        write_report=True,
    )

    manifest = json.loads((tmp_path / "evidence_manifest.json").read_text(encoding="utf-8"))
    summary = (tmp_path / "summary.md").read_text(encoding="utf-8")
    report = (tmp_path / "perception_shadow_report.md").read_text(encoding="utf-8")

    assert (tmp_path / "perception_shadow_result.json").exists()
    assert manifest["perception_shadow_evidence_available"] is True
    assert manifest["perception_shadow_status"] == STATUS_PASS
    assert manifest["user_command"] == "hover over the red mug"
    assert manifest["normalized_command"] == "hover over the red mug"
    assert manifest["snapshot_id"] == "perception_fixture_snapshot_001"
    assert manifest["grounding_id"] == "mock_grounding_red_mug_001"
    assert manifest["scene_version"] == "perception_fixture_scene_v1"
    assert manifest["camera_source_status"] == STATUS_PASS
    assert manifest["vlm_grounding_status"] == STATUS_PASS
    assert manifest["real_scene_shadow_status"] == "SHADOW_ACCEPTED"
    assert manifest["geometry_validity_status"] == STATUS_PASS
    assert manifest["projector_status"] == STATUS_PASS
    assert manifest["semantic_gate_passed"] is True
    assert manifest["no_motion_perception_passed"] is True
    assert manifest["target_label"] == "red_mug"
    assert manifest["target_object_id"] == "mock_red_mug_001"
    assert manifest["bbox_xyxy"] == [120, 80, 260, 220]
    assert manifest["pixel_center"] == [190, 150]
    assert manifest["overall_confidence"] == 0.89
    assert manifest["depth_value_m"] == 0.8
    assert manifest["camera_point_m"] == [-0.173333, -0.12, 0.8]
    assert manifest["world_point_m"] == [-0.073333, -0.12, 1.0]
    assert manifest["workspace_check_passed"] is True
    assert manifest["replay_ready"] is True
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
    assert "perception_shadow_report.md" in [
        item["name"] for item in manifest["perception_shadow_evidence_files"]
    ]
    assert "## Full Perception Shadow Pipeline Summary" in summary
    assert "perception_shadow_status: PASS" in summary
    assert "world_point_m: [-0.073333, -0.12, 1.0]" in summary
    assert "no-live-camera" in report
    assert "no-live-VLM" in report
    assert "no-real-robot" in report
    assert "no-ROS2" in report
    assert "no-MoveIt" in report


def test_report_contains_no_motion_safety_statement():
    report = format_perception_shadow_report(
        _evaluate_config_path("configs/perception_shadow_mock_positive.example.yaml")
    )

    assert "TETO V2.9.5 Full Perception Shadow Pipeline Report" in report
    assert "no-motion" in report
    assert "no-live-camera" in report
    assert "no-live-VLM" in report
    assert "no-real-robot" in report
    assert "no-ROS2" in report
    assert "no-MoveIt" in report
    assert "does not generate joint targets, trajectories, robot commands" in report


def test_cli_perception_shadow_arguments_parse():
    from scripts.harnesses.run_shadow_simulation_contract import build_parser

    parser = build_parser()
    args = parser.parse_args(
        [
            "--run-perception-shadow-pipeline",
            "--perception-shadow-config",
            "configs/perception_shadow_mock_positive.example.yaml",
            "--perception-shadow-report",
            "--user-command",
            "hover over the red mug",
            "--camera-source-config",
            "configs/perception_shadow_camera_source_fixture.yaml",
            "--vlm-grounding-config",
            "configs/perception_shadow_vlm_grounding_fixture.yaml",
            "--geometry-validity-config",
            "configs/geometry_validity.example.yaml",
            "--projector-shadow-config",
            "configs/projector_shadow.example.yaml",
        ]
    )

    assert args.run_perception_shadow_pipeline is True
    assert args.perception_shadow_config == "configs/perception_shadow_mock_positive.example.yaml"
    assert args.perception_shadow_report is True
    assert args.user_command == "hover over the red mug"
    assert args.camera_source_config == "configs/perception_shadow_camera_source_fixture.yaml"
    assert args.vlm_grounding_config == "configs/perception_shadow_vlm_grounding_fixture.yaml"
    assert args.geometry_validity_config == "configs/geometry_validity.example.yaml"
    assert args.projector_shadow_config == "configs/projector_shadow.example.yaml"


def _evaluate_config_path(path):
    return evaluate_perception_shadow_pipeline(
        build_perception_shadow_request(requested=True, config_path=path)
    )


def _evaluate_config(config):
    return evaluate_perception_shadow_pipeline(
        PerceptionShadowPipelineRequest(requested=True, config=config)
    )


def _valid_perception_config():
    return {
        "user_command": "hover over the red mug",
        "camera_source_adapter": {
            "source_mode": "offline_file",
            "snapshot_id": "perception_fixture_snapshot_001",
            "scene_version": "perception_fixture_scene_v1",
            "capture_timestamp": "2099-01-01T00:00:00Z",
            "frame_id": "perception_fixture_frame_001",
            "camera_frame": "camera_color_optical_frame",
            "image_ref": "examples/camera_source/sample_image_ref.json",
            "depth_ref": "examples/camera_source/sample_depth_ref.json",
            "camera_info_ref": "examples/camera_source/sample_camera_info.json",
            "width": 640,
            "height": 480,
            "depth_available": True,
            "camera_info_available": True,
            "depth_required": True,
            "capture_backend": "offline_file",
            "capture_method": "declared_snapshot_manifest",
            "allow_live_camera_capture": False,
            "continuous_capture_enabled": False,
            "live_vlm_called": False,
        },
        "vlm_grounding_adapter": {
            "adapter_mode": "mock_vlm",
            "snapshot_id": "perception_fixture_snapshot_001",
            "scene_version": "perception_fixture_scene_v1",
            "user_command": "hover over the red mug",
            "expected_snapshot_id": "perception_fixture_snapshot_001",
            "expected_scene_version": "perception_fixture_scene_v1",
            "overall_confidence_threshold": 0.60,
            "allow_live_vlm": False,
            "live_vlm_called": False,
        },
        "geometry_validity": {
            "depth_required": True,
            "thresholds": {"confidence_threshold": 0.60, "min_bbox_area_px": 1.0},
        },
        "projector_shadow": {
            "camera_info_ref": "examples/projector_camera_info_example.json",
            "depth_sample": {"depth_value_m": 0.8},
            "mock_tf_ref": "examples/projector_mock_tf_example.json",
            "camera_frame": "camera_color_optical_frame",
            "world_frame": "mock_world",
            "projection_method": "pinhole_mock_tf",
            "projection_confidence": 0.86,
            "require_world_projection": True,
            "workspace_m": {
                "x": [-1.0, 1.0],
                "y": [-1.0, 1.0],
                "z": [0.0, 2.5],
            },
        },
    }


def _manual_grounding_config(pixel_center):
    return {
        "adapter_mode": "manual_annotation",
        "user_command": "hover over the red mug",
        "expected_snapshot_id": "perception_fixture_snapshot_001",
        "expected_scene_version": "perception_fixture_scene_v1",
        "manual_annotation": {
            "grounding_id": "manual_perception_grounding_001",
            "snapshot_id": "perception_fixture_snapshot_001",
            "scene_version": "perception_fixture_scene_v1",
            "target_label": "red_mug",
            "target_object_id": "perception_red_mug_001",
            "bbox_xyxy": [120, 80, 260, 220],
            "pixel_center": pixel_center,
            "semantic_confidence": 0.9,
            "grounding_confidence": 0.88,
            "overall_confidence": 0.89,
            "grounded": True,
            "rejected": False,
        },
    }


def _snapshot_from_camera_source(config):
    return {
        key: config.get(key)
        for key in [
            "snapshot_id",
            "scene_version",
            "capture_timestamp",
            "frame_id",
            "camera_frame",
            "image_ref",
            "depth_ref",
            "camera_info_ref",
            "width",
            "height",
            "depth_available",
            "camera_info_available",
            "depth_required",
        ]
    }
