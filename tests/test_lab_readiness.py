import json

import yaml

from src.lab_readiness import (
    E_LIVE_CAMERA_DISABLED,
    E_LIVE_VLM_DISABLED,
    E_MISSING_CAMERA_EXTRINSICS,
    E_MISSING_SAFETY_STATUS_CHECK,
    E_MISSING_TCP_CALIBRATION,
    E_REAL_ROBOT_MOTION_NOT_ALLOWED,
    E_SHADOW_MODE_NOT_ENABLED,
    LabBackendReadinessRequest,
    build_lab_readiness_request,
    evaluate_camera_readiness,
    evaluate_lab_backend_readiness,
    evaluate_lab_readiness,
    evaluate_live_vlm_readiness,
    evaluate_shadow_mode_readiness,
    format_lab_readiness_report,
)
from src.simulation_runtime import run_first_simulation_execution


VALID_TASK = {
    "task_type": "hover_to_object",
    "target_label": "camera",
    "target_world_point": [0.2, 0.1, 0.4],
    "scene_version": "run_simulation_item_001",
    "ttl_ms": 500,
}


def test_missing_config_produces_config_only_not_crash():
    request = build_lab_readiness_request(
        check_lab_backend=True,
        check_camera=True,
        check_live_vlm=True,
        check_shadow_mode=True,
        config_path=None,
    )

    result = evaluate_lab_readiness(request)

    assert result["status"] == "CONFIG_ONLY"
    assert result["lab_backend_readiness_status"] == "CONFIG_ONLY"
    assert result["no_motion_readiness_passed"] is False
    assert E_MISSING_TCP_CALIBRATION not in result["blocking_reasons"]


def test_example_config_with_no_motion_flags_is_ready_for_shadow_mode(tmp_path):
    config_path = _write_config(tmp_path, _ready_config())
    request = build_lab_readiness_request(
        check_lab_backend=True,
        check_camera=True,
        check_live_vlm=True,
        check_shadow_mode=True,
        config_path=config_path,
    )

    result = evaluate_lab_readiness(request)

    assert result["status"] == "READY_FOR_SHADOW_MODE"
    assert result["no_motion_readiness_passed"] is True
    assert result["allow_robot_motion"] is False
    assert result["allow_live_camera"] is False
    assert result["allow_live_vlm"] is False
    assert result["real_robot_command_enabled"] is False


def test_allow_robot_motion_true_is_rejected_in_v280():
    config = _ready_config()
    config["lab_backend"]["allow_robot_motion"] = True

    result = evaluate_lab_backend_readiness(
        LabBackendReadinessRequest(config=config, check_lab_backend=True)
    )

    assert result["status"] == "BLOCKED"
    assert E_REAL_ROBOT_MOTION_NOT_ALLOWED in result["blocking_reasons"]


def test_allow_live_camera_true_is_rejected_by_default_but_fixture_can_permit():
    config = _ready_config()
    config["camera"]["allow_live_camera"] = True

    blocked = evaluate_camera_readiness(LabBackendReadinessRequest(config=config, check_camera=True))
    permitted = evaluate_camera_readiness(
        LabBackendReadinessRequest(config=config, check_camera=True, permit_live_camera=True)
    )

    assert blocked["status"] == "BLOCKED"
    assert E_LIVE_CAMERA_DISABLED in blocked["blocking_reasons"]
    assert permitted["status"] == "READY_FOR_SHADOW_MODE"


def test_allow_live_vlm_true_is_rejected_by_default_but_fixture_can_permit():
    config = _ready_config()
    config["live_vlm"]["allow_live_vlm"] = True

    blocked = evaluate_live_vlm_readiness(LabBackendReadinessRequest(config=config, check_live_vlm=True))
    permitted = evaluate_live_vlm_readiness(
        LabBackendReadinessRequest(config=config, check_live_vlm=True, permit_live_vlm=True)
    )

    assert blocked["status"] == "BLOCKED"
    assert E_LIVE_VLM_DISABLED in blocked["blocking_reasons"]
    assert permitted["status"] == "READY_FOR_SHADOW_MODE"


def test_missing_camera_extrinsics_blocks_camera_readiness():
    config = _ready_config()
    config["camera"]["extrinsics_configured"] = False

    result = evaluate_camera_readiness(LabBackendReadinessRequest(config=config, check_camera=True))

    assert E_MISSING_CAMERA_EXTRINSICS in result["blocking_reasons"]


def test_missing_tcp_calibration_blocks_lab_backend_readiness():
    config = _ready_config()
    config["lab_backend"]["tcp_calibration_configured"] = False

    result = evaluate_lab_backend_readiness(
        LabBackendReadinessRequest(config=config, check_lab_backend=True)
    )

    assert E_MISSING_TCP_CALIBRATION in result["blocking_reasons"]


def test_missing_safety_status_check_blocks_lab_backend_readiness():
    config = _ready_config()
    config["lab_backend"]["safety_status_check_available"] = False

    result = evaluate_lab_backend_readiness(
        LabBackendReadinessRequest(config=config, check_lab_backend=True)
    )

    assert E_MISSING_SAFETY_STATUS_CHECK in result["blocking_reasons"]


def test_shadow_mode_disabled_is_not_ready():
    config = _ready_config()
    config["shadow_mode"]["shadow_mode_enabled"] = False

    result = evaluate_shadow_mode_readiness(
        LabBackendReadinessRequest(config=config, check_shadow_mode=True)
    )

    assert result["status"] == "NOT_READY"
    assert E_SHADOW_MODE_NOT_ENABLED in result["blocking_reasons"]


def test_readiness_report_contains_no_motion_safety_boundary():
    result = evaluate_lab_readiness(
        LabBackendReadinessRequest(
            config=_ready_config(),
            check_lab_backend=True,
            check_camera=True,
            check_live_vlm=True,
            check_shadow_mode=True,
        )
    )

    report = format_lab_readiness_report(result)

    assert "no-motion" in report.lower()
    assert "does not connect to a real UR5" in report
    assert "does not generate trajectories" in report
    assert "does not execute tcp_pose_world" in report


def test_runtime_evidence_manifest_and_summary_contain_readiness_files(tmp_path):
    config_path = _write_config(tmp_path, _ready_config())

    run_first_simulation_execution(
        VALID_TASK,
        steps=1,
        check_lab_readiness=True,
        check_camera_readiness=True,
        check_live_vlm_readiness=True,
        check_shadow_mode_readiness=True,
        lab_readiness_config=config_path,
        output_dir=tmp_path,
        write_report=True,
    )

    manifest = json.loads((tmp_path / "evidence_manifest.json").read_text(encoding="utf-8"))
    summary = (tmp_path / "summary.md").read_text(encoding="utf-8")

    assert manifest["lab_readiness_requested"] is True
    assert manifest["no_motion_readiness_passed"] is True
    assert "lab_readiness_result.json" in [item["name"] for item in manifest["readiness_evidence_files"]]
    assert "## Lab / Camera / VLM No-Motion Readiness Summary" in summary
    assert (tmp_path / "lab_readiness_result.json").exists()
    assert (tmp_path / "lab_readiness_report.md").exists()
    assert (tmp_path / "camera_readiness_result.json").exists()
    assert (tmp_path / "live_vlm_readiness_result.json").exists()
    assert (tmp_path / "shadow_mode_readiness_result.json").exists()


def test_no_real_execution_fields_are_introduced_misleadingly():
    result = evaluate_lab_readiness(
        LabBackendReadinessRequest(
            config=_ready_config(),
            check_lab_backend=True,
            check_camera=True,
            check_live_vlm=True,
            check_shadow_mode=True,
        )
    )

    serialized = json.dumps(result, sort_keys=True)

    assert "real_robot_command_enabled" in serialized
    assert "allow_robot_motion" in serialized
    assert '"real_robot_command_enabled": false' in serialized
    assert '"allow_robot_motion": false' in serialized
    assert "tcp_pose_world_command" not in serialized
    assert "trajectory_command" not in serialized
    assert "urscript_program" not in serialized


def _write_config(tmp_path, config):
    path = tmp_path / "local.lab_readiness.yaml"
    path.write_text(yaml.safe_dump(config), encoding="utf-8")
    return path


def _ready_config():
    return {
        "lab_backend": {
            "backend_name": "ur5_lab_shadow_backend",
            "backend_type": "ur5",
            "robot_model": "UR5",
            "robot_ip_configured": True,
            "tcp_calibration_configured": True,
            "robot_calibration_configured": True,
            "camera_extrinsics_configured": True,
            "robot_mode_check_available": True,
            "safety_status_check_available": True,
            "program_state_check_available": True,
            "speed_scaling_check_available": True,
            "allow_real_robot_backend": False,
            "allow_robot_motion": False,
            "real_robot_command_enabled": False,
        },
        "camera": {
            "camera_backend": "realsense_shadow",
            "camera_configured": True,
            "rgb_stream_configured": True,
            "depth_stream_configured": True,
            "camera_info_configured": True,
            "metadata_configured": True,
            "extrinsics_configured": True,
            "allow_live_camera": False,
        },
        "live_vlm": {
            "vlm_backend": "qwen_shadow",
            "model_name": "qwen2.5vl:3b",
            "endpoint_configured": True,
            "schema_output_supported": True,
            "allow_live_vlm": False,
        },
        "shadow_mode": {
            "shadow_mode_enabled": True,
            "fake_execution_enabled": True,
            "no_motion_enforced": True,
            "evidence_export_enabled": True,
            "manual_confirmation_required": True,
        },
    }
