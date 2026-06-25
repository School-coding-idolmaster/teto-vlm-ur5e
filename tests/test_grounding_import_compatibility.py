from src import grounding_result as legacy_result
from src import vlm_grounding_adapter as legacy_vlm
from src.grounding.command_normalization import normalize_command as split_normalize_command
from src.grounding.reporting import format_vlm_grounding_report as split_format_vlm_grounding_report
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
