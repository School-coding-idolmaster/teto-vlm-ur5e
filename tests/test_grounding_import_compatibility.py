from src import grounding_result as legacy_result
from src import vlm_grounding_adapter as legacy_vlm
from src.grounding.command_normalization import normalize_command as split_normalize_command
from src.grounding.forbidden_fields import (
    FORBIDDEN_ROBOT_CONTROL_FIELDS as split_forbidden_fields,
    find_forbidden_robot_control_fields,
)
from src.grounding.reporting import format_vlm_grounding_report as split_format_vlm_grounding_report
from src.grounding.scene_binding import (
    SCENE_BINDING_SCENE_VERSION,
    SCENE_BINDING_SNAPSHOT_ID,
    find_scene_binding_mismatches,
)
from src.grounding import result as new_result
from src.grounding import vlm_adapter as new_vlm


def test_grounding_result_old_and_new_imports_share_public_api():
    names = [
        "CONTRACT_VERSION",
        "CURRENT_GROUNDING_VERSION",
        "FORBIDDEN_ROBOT_CONTROL_FIELDS",
        "GroundingResultRequest",
        "load_grounding_result",
        "build_grounding_result_request",
        "evaluate_grounding_result_contract",
    ]

    for name in names:
        assert getattr(legacy_result, name) is getattr(new_result, name)


def test_vlm_grounding_adapter_old_and_new_imports_share_public_api():
    names = [
        "CONTRACT_VERSION",
        "CURRENT_VLM_GROUNDING_VERSION",
        "STATUS_PASS",
        "STATUS_BLOCKED",
        "STATUS_SAFE_DISABLED",
        "STATUS_NOT_REQUESTED",
        "MODE_OFFLINE_GROUNDING_JSON",
        "MODE_MOCK_VLM",
        "MODE_MANUAL_ANNOTATION",
        "MODE_LOCAL_VLM_DISABLED",
        "MODE_FUTURE_LOCAL_QWEN_ADAPTER",
        "SUPPORTED_MODES",
        "DEFAULT_CONFIDENCE_THRESHOLD",
        "E_UNSUPPORTED_ADAPTER_MODE",
        "E_LIVE_VLM_DISABLED",
        "E_UNSUPPORTED_COMMAND",
        "E_NO_TARGET",
        "E_LOW_CONFIDENCE",
        "E_BBOX_MISSING",
        "E_PIXEL_CENTER_MISSING",
        "E_SNAPSHOT_MISMATCH",
        "E_SCENE_VERSION_MISMATCH",
        "E_ROBOT_COMMAND_NOT_ALLOWED",
        "FORBIDDEN_ROBOT_CONTROL_FIELDS",
        "MOCK_COMMANDS",
        "VLM_GROUNDING_FIELDS",
        "VLMGroundingAdapterRequest",
        "load_vlm_grounding_config",
        "build_vlm_grounding_adapter_request",
        "evaluate_vlm_grounding_adapter",
        "normalize_command",
        "format_vlm_grounding_report",
    ]

    for name in names:
        assert getattr(legacy_vlm, name) is getattr(new_vlm, name)


def test_vlm_grounding_split_helpers_preserve_import_identity():
    assert legacy_vlm.normalize_command is new_vlm.normalize_command
    assert new_vlm.normalize_command is split_normalize_command
    assert legacy_vlm.format_vlm_grounding_report is new_vlm.format_vlm_grounding_report
    assert new_vlm.format_vlm_grounding_report is split_format_vlm_grounding_report


def test_forbidden_fields_helper_preserves_old_imports_and_detection():
    assert legacy_result.FORBIDDEN_ROBOT_CONTROL_FIELDS is new_result.FORBIDDEN_ROBOT_CONTROL_FIELDS
    assert legacy_vlm.FORBIDDEN_ROBOT_CONTROL_FIELDS is new_vlm.FORBIDDEN_ROBOT_CONTROL_FIELDS
    assert new_result.FORBIDDEN_ROBOT_CONTROL_FIELDS is split_forbidden_fields
    assert new_vlm.FORBIDDEN_ROBOT_CONTROL_FIELDS is split_forbidden_fields
    assert find_forbidden_robot_control_fields(
        {"trajectory": {"joint_targets": [0.0]}, "items": [{"tcp_pose_world": [0, 0, 0]}]}
    ) == ["trajectory", "trajectory.joint_targets", "items[0].tcp_pose_world"]


def test_scene_binding_helper_import_path_and_order():
    result = {"snapshot_id": "snapshot_a", "scene_version": "scene_a"}

    assert find_scene_binding_mismatches(
        result,
        expected_snapshot_id="snapshot_b",
        expected_scene_version="scene_b",
    ) == [SCENE_BINDING_SNAPSHOT_ID, SCENE_BINDING_SCENE_VERSION]
    assert (
        find_scene_binding_mismatches(
            result,
            expected_snapshot_id="snapshot_a",
            expected_scene_version="scene_a",
        )
        == []
    )
