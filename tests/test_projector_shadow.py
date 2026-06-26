import json

import yaml

from src.vision.snapshot.camera_snapshot import CameraSnapshotRequest, evaluate_camera_snapshot_contract
from src.geometry_validity import evaluate_geometry_validity_from_contracts
from src.grounding.result import GroundingResultRequest, evaluate_grounding_result_contract
from src.projector.shadow import (
    E_CAMERA_FRAME_MISSING,
    E_CAMERA_INFO_MISSING,
    E_DEPTH_OUT_OF_RANGE,
    E_GEOMETRY_NOT_VALID,
    E_INVALID_CAMERA_INTRINSICS,
    E_INVALID_DEPTH,
    E_LIVE_CAMERA_DISABLED,
    E_LIVE_VLM_DISABLED,
    E_NO_DEPTH,
    E_OUT_OF_WORKSPACE,
    E_PIXEL_CENTER_MISSING,
    E_ROBOT_COMMAND_NOT_ALLOWED,
    E_TF_UNAVAILABLE,
    E_WORLD_FRAME_MISSING,
    STATUS_BLOCKED,
    STATUS_PASS,
    build_projector_shadow_request,
    evaluate_projector_shadow,
    evaluate_projector_shadow_from_contracts,
    format_projector_shadow_report,
)
from src.simulation_runtime import run_first_simulation_execution


VALID_TASK = {
    "task_type": "hover_to_object",
    "target_label": "camera",
    "target_world_point": [0.2, 0.1, 0.4],
    "scene_version": "run_simulation_item_001",
    "ttl_ms": 500,
}


def test_valid_projection_passes():
    result = _evaluate_projector()

    assert result["projector_status"] == STATUS_PASS
    assert result["no_motion_projector_passed"] is True
    assert result["camera_intrinsics_available"] is True
    assert result["depth_valid"] is True
    assert result["tf_available"] is True
    assert result["tf_source"] == "mock_or_config"
    assert result["real_tf_used"] is False
    assert result["ros2_tf_used"] is False
    assert result["workspace_check_passed"] is True
    assert result["camera_point_m"] == [-0.173333, -0.086667, 0.8]
    assert result["world_point_m"] == [-0.073333, -0.086667, 1.0]
    assert result["robot_command_generated"] is False
    assert result["trajectory_generated"] is False
    assert result["joint_targets_generated"] is False
    assert result["tcp_pose_world_generated"] is False


def test_geometry_not_pass_blocks_projector_shadow():
    result = _evaluate_projector(grounding_updates={"bbox_xyxy": None})

    assert result["projector_status"] == STATUS_BLOCKED
    assert E_GEOMETRY_NOT_VALID in result["blocking_reasons"]


def test_missing_pixel_center_blocks_projector_shadow():
    geometry = _valid_geometry()
    geometry["pixel_center"] = None

    result = evaluate_projector_shadow_from_contracts(geometry, projector_config=_valid_projector_config())

    assert result["projector_status"] == STATUS_BLOCKED
    assert E_PIXEL_CENTER_MISSING in result["blocking_reasons"]


def test_missing_camera_info_blocks_projector_shadow():
    config = _valid_projector_config()
    config.pop("camera_info")

    result = _evaluate_projector(projector_config=config)

    assert result["projector_status"] == STATUS_BLOCKED
    assert E_CAMERA_INFO_MISSING in result["blocking_reasons"]


def test_invalid_intrinsics_blocks_projector_shadow():
    config = _valid_projector_config()
    config["camera_info"]["intrinsics"]["fx"] = 0.0

    result = _evaluate_projector(projector_config=config)

    assert result["projector_status"] == STATUS_BLOCKED
    assert E_INVALID_CAMERA_INTRINSICS in result["blocking_reasons"]


def test_missing_depth_blocks_projector_shadow():
    config = _valid_projector_config()
    config.pop("depth_sample")

    result = _evaluate_projector(projector_config=config)

    assert result["projector_status"] == STATUS_BLOCKED
    assert E_NO_DEPTH in result["blocking_reasons"]


def test_invalid_depth_blocks_projector_shadow():
    config = _valid_projector_config()
    config["depth_sample"]["depth_value_m"] = 0.0

    result = _evaluate_projector(projector_config=config)

    assert result["projector_status"] == STATUS_BLOCKED
    assert E_INVALID_DEPTH in result["blocking_reasons"]


def test_depth_out_of_range_blocks_projector_shadow():
    config = _valid_projector_config()
    config["depth_sample"]["depth_value_m"] = 10.0

    result = _evaluate_projector(projector_config=config)

    assert result["projector_status"] == STATUS_BLOCKED
    assert E_DEPTH_OUT_OF_RANGE in result["blocking_reasons"]


def test_missing_camera_frame_blocks_projector_shadow():
    geometry = _valid_geometry()
    geometry["camera_snapshot"]["camera_frame"] = None
    config = _valid_projector_config()
    config.pop("camera_frame")

    result = evaluate_projector_shadow_from_contracts(geometry, projector_config=config)

    assert result["projector_status"] == STATUS_BLOCKED
    assert E_CAMERA_FRAME_MISSING in result["blocking_reasons"]


def test_missing_world_frame_blocks_projector_shadow():
    config = _valid_projector_config()
    config.pop("world_frame")

    result = _evaluate_projector(projector_config=config)

    assert result["projector_status"] == STATUS_BLOCKED
    assert E_WORLD_FRAME_MISSING in result["blocking_reasons"]


def test_missing_transform_blocks_projector_shadow():
    config = _valid_projector_config()
    config.pop("mock_tf")

    result = _evaluate_projector(projector_config=config)

    assert result["projector_status"] == STATUS_BLOCKED
    assert E_TF_UNAVAILABLE in result["blocking_reasons"]


def test_out_of_workspace_blocks_projector_shadow():
    config = _valid_projector_config()
    config["workspace_m"] = {"x": [0.5, 1.0], "y": [0.5, 1.0], "z": [0.0, 2.0]}

    result = _evaluate_projector(projector_config=config)

    assert result["projector_status"] == STATUS_BLOCKED
    assert E_OUT_OF_WORKSPACE in result["blocking_reasons"]


def test_live_camera_used_blocks_projector_shadow():
    geometry = _valid_geometry()
    geometry["live_camera_used"] = True

    result = evaluate_projector_shadow_from_contracts(geometry, projector_config=_valid_projector_config())

    assert result["projector_status"] == STATUS_BLOCKED
    assert E_LIVE_CAMERA_DISABLED in result["blocking_reasons"]


def test_live_vlm_called_blocks_projector_shadow():
    geometry = _valid_geometry()
    geometry["live_vlm_called"] = True

    result = evaluate_projector_shadow_from_contracts(geometry, projector_config=_valid_projector_config())

    assert result["projector_status"] == STATUS_BLOCKED
    assert E_LIVE_VLM_DISABLED in result["blocking_reasons"]


def test_robot_control_field_blocks_projector_shadow_without_generating_controls():
    config = _valid_projector_config()
    config["robot_command"] = {"joint_targets": [0.0]}

    result = _evaluate_projector(projector_config=config)

    assert result["projector_status"] == STATUS_BLOCKED
    assert E_ROBOT_COMMAND_NOT_ALLOWED in result["blocking_reasons"]
    assert result["robot_command_generated"] is False
    assert result["trajectory_generated"] is False
    assert result["joint_targets_generated"] is False
    assert result["tcp_pose_world_generated"] is False


def test_example_config_smoke_passes():
    result = evaluate_projector_shadow(
        build_projector_shadow_request(
            requested=True,
            config_path="configs/projector_shadow.example.yaml",
        )
    )

    assert result["projector_status"] == STATUS_PASS
    assert result["snapshot_id"] == "example_offline_snapshot_001"
    assert result["grounding_id"] == "example_grounding_001"
    assert result["camera_point_m"] == [-0.173333, -0.086667, 0.8]
    assert result["world_point_m"] == [-0.073333, -0.086667, 1.0]


def test_report_contains_no_motion_no_live_no_real_robot_no_ros2_tf_statement():
    report = format_projector_shadow_report(_evaluate_projector())

    assert "TETO V2.9.2 2D-to-3D Projector Shadow Contract Report" in report
    assert "no-motion" in report
    assert "no-live-camera" in report
    assert "no-live-VLM" in report
    assert "no-real-robot" in report
    assert "no-ROS2-TF" in report
    assert "does not generate joint targets, trajectories, robot commands, or real execution requests" in report


def test_runtime_manifest_contains_projector_shadow_evidence_fields(tmp_path):
    snapshot_path = _write_snapshot(tmp_path, _valid_snapshot())
    grounding_path = _write_grounding(tmp_path, _valid_grounding())
    geometry_config_path = _write_geometry_config(tmp_path, snapshot_path, grounding_path)
    projector_config_path = _write_projector_config(tmp_path, geometry_config_path)

    run_first_simulation_execution(
        VALID_TASK,
        steps=1,
        check_projector_shadow=True,
        projector_shadow_config=projector_config_path,
        projector_shadow_report=True,
        output_dir=tmp_path,
        write_report=True,
    )

    manifest = json.loads((tmp_path / "evidence_manifest.json").read_text(encoding="utf-8"))
    summary = (tmp_path / "summary.md").read_text(encoding="utf-8")
    report = (tmp_path / "projector_shadow_report.md").read_text(encoding="utf-8")

    assert manifest["projector_shadow_evidence_available"] is True
    assert manifest["projector_requested"] is True
    assert manifest["projector_status"] == STATUS_PASS
    assert manifest["snapshot_id"] == "snapshot_fixture_001"
    assert manifest["grounding_id"] == "grounding_fixture_001"
    assert manifest["scene_version"] == "fixture_scene_v1"
    assert manifest["pixel_center"] == [190, 175]
    assert manifest["depth_value_m"] == 0.8
    assert manifest["depth_valid"] is True
    assert manifest["camera_intrinsics_available"] is True
    assert manifest["camera_frame"] == "camera_color_optical_frame"
    assert manifest["world_frame"] == "mock_world"
    assert manifest["camera_point_m"] == [-0.173333, -0.086667, 0.8]
    assert manifest["world_point_m"] == [-0.073333, -0.086667, 1.0]
    assert manifest["projection_confidence"] == 0.86
    assert manifest["projection_method"] == "pinhole_mock_tf"
    assert manifest["tf_available"] is True
    assert manifest["tf_source"] == "mock_or_config"
    assert manifest["real_tf_used"] is False
    assert manifest["ros2_tf_used"] is False
    assert manifest["workspace_check_passed"] is True
    assert manifest["blocking_reasons"] == []
    assert manifest["warnings"] == []
    assert manifest["no_motion_projector_passed"] is True
    assert manifest["live_camera_used"] is False
    assert manifest["live_vlm_called"] is False
    assert manifest["real_robot_motion_executed"] is False
    assert manifest["real_robot_command_enabled"] is False
    assert manifest["robot_command_generated"] is False
    assert manifest["trajectory_generated"] is False
    assert manifest["joint_targets_generated"] is False
    assert manifest["tcp_pose_world_generated"] is False
    assert "projector_shadow_report.md" in [item["name"] for item in manifest["projector_shadow_evidence_files"]]
    assert "## Projector Shadow Evidence Summary" in summary
    assert "projector_status: PASS" in summary
    assert "no-ROS2-TF" in report


def test_cli_projector_shadow_arguments_parse():
    from scripts.harnesses.run_shadow_simulation_contract import build_parser

    parser = build_parser()
    args = parser.parse_args(
        [
            "--check-projector-shadow",
            "--projector-shadow-config",
            "configs/projector_shadow.example.yaml",
            "--projector-shadow-report",
            "--camera-snapshot-config",
            "configs/camera_snapshot.example.yaml",
            "--grounding-result",
            "examples/grounding_result_example.json",
            "--geometry-validity-config",
            "configs/geometry_validity.example.yaml",
        ]
    )

    assert args.check_projector_shadow is True
    assert args.projector_shadow_config == "configs/projector_shadow.example.yaml"
    assert args.projector_shadow_report is True
    assert args.camera_snapshot_config == "configs/camera_snapshot.example.yaml"
    assert args.grounding_result == "examples/grounding_result_example.json"
    assert args.geometry_validity_config == "configs/geometry_validity.example.yaml"


def _evaluate_projector(snapshot_updates=None, grounding_updates=None, projector_config=None):
    return evaluate_projector_shadow_from_contracts(
        _valid_geometry(snapshot_updates=snapshot_updates, grounding_updates=grounding_updates),
        projector_config=projector_config or _valid_projector_config(),
    )


def _valid_geometry(snapshot_updates=None, grounding_updates=None):
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


def _valid_projector_config():
    return {
        "camera_info": {
            "intrinsics": {
                "fx": 600.0,
                "fy": 600.0,
                "cx": 320.0,
                "cy": 240.0,
            }
        },
        "depth_sample": {"depth_value_m": 0.8},
        "mock_tf": {
            "tf_source": "mock_or_config",
            "real_tf_used": False,
            "ros2_tf_used": False,
            "translation_m": [0.1, 0.0, 0.2],
            "rotation_matrix": [
                [1.0, 0.0, 0.0],
                [0.0, 1.0, 0.0],
                [0.0, 0.0, 1.0],
            ],
        },
        "camera_frame": "camera_color_optical_frame",
        "world_frame": "mock_world",
        "projection_method": "pinhole_mock_tf",
        "projection_confidence": 0.86,
        "require_world_projection": True,
        "depth_range_m": {"min": 0.05, "max": 2.0},
        "workspace_m": {"x": [-1.0, 1.0], "y": [-1.0, 1.0], "z": [0.0, 2.5]},
    }


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


def _write_projector_config(tmp_path, geometry_config_path):
    path = tmp_path / "projector_shadow.yaml"
    config = _valid_projector_config()
    config["geometry_validity_config"] = str(geometry_config_path)
    path.write_text(yaml.safe_dump({"projector_shadow": config}), encoding="utf-8")
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
