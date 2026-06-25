from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict

import yaml

from src.grounding.command_normalization import normalize_command
from src.grounding.forbidden_fields import (
    FORBIDDEN_ROBOT_CONTROL_FIELDS,
    find_forbidden_robot_control_fields as _forbidden_robot_control_fields,
)
from src.grounding.reporting import format_vlm_grounding_report


CONTRACT_VERSION = "teto_vlm_grounding_adapter.v1"
CURRENT_VLM_GROUNDING_VERSION = "TETO V2.9.4"

STATUS_PASS = "PASS"
STATUS_BLOCKED = "BLOCKED"
STATUS_SAFE_DISABLED = "SAFE_DISABLED"
STATUS_NOT_REQUESTED = "NOT_REQUESTED"

MODE_OFFLINE_GROUNDING_JSON = "offline_grounding_json"
MODE_MOCK_VLM = "mock_vlm"
MODE_MANUAL_ANNOTATION = "manual_annotation"
MODE_LOCAL_VLM_DISABLED = "local_vlm_disabled"
MODE_FUTURE_LOCAL_QWEN_ADAPTER = "future_local_qwen_adapter"

SUPPORTED_MODES = {
    MODE_OFFLINE_GROUNDING_JSON,
    MODE_MOCK_VLM,
    MODE_MANUAL_ANNOTATION,
    MODE_LOCAL_VLM_DISABLED,
    MODE_FUTURE_LOCAL_QWEN_ADAPTER,
}

DEFAULT_CONFIDENCE_THRESHOLD = 0.6

E_UNSUPPORTED_ADAPTER_MODE = "E_UNSUPPORTED_ADAPTER_MODE"
E_LIVE_VLM_DISABLED = "E_LIVE_VLM_DISABLED"
E_UNSUPPORTED_COMMAND = "E_UNSUPPORTED_COMMAND"
E_NO_TARGET = "E_NO_TARGET"
E_LOW_CONFIDENCE = "E_LOW_CONFIDENCE"
E_BBOX_MISSING = "E_BBOX_MISSING"
E_PIXEL_CENTER_MISSING = "E_PIXEL_CENTER_MISSING"
E_SNAPSHOT_MISMATCH = "E_SNAPSHOT_MISMATCH"
E_SCENE_VERSION_MISMATCH = "E_SCENE_VERSION_MISMATCH"
E_ROBOT_COMMAND_NOT_ALLOWED = "E_ROBOT_COMMAND_NOT_ALLOWED"

MOCK_COMMANDS = {
    "hover over the red mug",
    "move above the red mug",
    "point to the red mug",
}

VLM_GROUNDING_FIELDS = (
    "vlm_grounding_requested",
    "vlm_grounding_status",
    "grounding_id",
    "snapshot_id",
    "scene_version",
    "user_command",
    "normalized_command",
    "adapter_mode",
    "target_label",
    "target_object_id",
    "bbox_xyxy",
    "pixel_center",
    "mask_ref",
    "semantic_confidence",
    "grounding_confidence",
    "overall_confidence",
    "grounded",
    "rejected",
    "rejection_reason",
    "error_code",
    "blocking_reasons",
    "warnings",
    "next_safe_action",
    "no_motion_grounding_passed",
    "live_vlm_called",
    "live_camera_used",
    "real_robot_motion_executed",
    "real_robot_command_enabled",
    "robot_command_generated",
    "trajectory_generated",
    "joint_targets_generated",
    "tcp_pose_world_generated",
)


@dataclass(frozen=True)
class VLMGroundingAdapterRequest:
    requested: bool = False
    config_path: str | None = None
    adapter_mode: str | None = None
    user_command: str | None = None
    allow_live_vlm: bool = False
    config: Dict[str, Any] | None = None


def load_vlm_grounding_config(path: str | Path | None) -> Dict[str, Any]:
    if not path:
        return {}
    resolved_path = Path(path).expanduser()
    if not resolved_path.is_file():
        return {}
    with resolved_path.open("r", encoding="utf-8") as config_file:
        if resolved_path.suffix.lower() == ".json":
            data = json.load(config_file)
        else:
            data = yaml.safe_load(config_file)
    if not isinstance(data, dict):
        return {}
    config = data.get("vlm_grounding_adapter") or data.get("vlm_grounding")
    return config if isinstance(config, dict) else data


def build_vlm_grounding_adapter_request(
    *,
    requested: bool = False,
    config_path: str | Path | None = None,
    adapter_mode: str | None = None,
    user_command: str | None = None,
    allow_live_vlm: bool = False,
) -> VLMGroundingAdapterRequest:
    config = load_vlm_grounding_config(config_path)
    return VLMGroundingAdapterRequest(
        requested=requested,
        config_path=str(Path(config_path).expanduser()) if config_path else None,
        adapter_mode=adapter_mode,
        user_command=user_command,
        allow_live_vlm=allow_live_vlm,
        config=config,
    )


def evaluate_vlm_grounding_adapter(request: VLMGroundingAdapterRequest | None = None) -> Dict[str, Any]:
    request = request or VLMGroundingAdapterRequest()
    if not request.requested:
        return _not_requested_result()

    config = request.config if isinstance(request.config, dict) else {}
    adapter_mode = request.adapter_mode or _string(config.get("adapter_mode")) or MODE_LOCAL_VLM_DISABLED
    user_command = request.user_command or _string(config.get("user_command"))
    normalized_command = normalize_command(user_command)
    threshold = _optional_float(config.get("overall_confidence_threshold")) or DEFAULT_CONFIDENCE_THRESHOLD
    warnings = list(config.get("warnings") or []) if isinstance(config.get("warnings"), list) else []
    blocking_reasons: list[str] = []

    raw_result: Dict[str, Any]
    if adapter_mode == MODE_MOCK_VLM:
        raw_result = _mock_result(config, user_command, normalized_command)
    elif adapter_mode == MODE_OFFLINE_GROUNDING_JSON:
        raw_result = _offline_result(config)
    elif adapter_mode == MODE_MANUAL_ANNOTATION:
        raw_result = _manual_result(config, user_command, normalized_command)
    elif adapter_mode in {MODE_LOCAL_VLM_DISABLED, MODE_FUTURE_LOCAL_QWEN_ADAPTER}:
        raw_result = _disabled_result(config, user_command, normalized_command, adapter_mode)
        if adapter_mode == MODE_FUTURE_LOCAL_QWEN_ADAPTER:
            warnings.append("future_local_qwen_adapter_declaration_only")
    else:
        raw_result = _base_result(config, user_command, normalized_command)
        blocking_reasons.append(E_UNSUPPORTED_ADAPTER_MODE)

    forbidden_fields = _forbidden_robot_control_fields(config) + _forbidden_robot_control_fields(raw_result)
    if forbidden_fields:
        blocking_reasons.append(E_ROBOT_COMMAND_NOT_ALLOWED)
        warnings.append(f"forbidden_robot_control_fields={_unique(forbidden_fields)}")

    if config.get("live_vlm_called") is True or raw_result.get("live_vlm_called") is True:
        blocking_reasons.append(E_LIVE_VLM_DISABLED)
    if adapter_mode == MODE_MOCK_VLM and normalized_command not in MOCK_COMMANDS:
        blocking_reasons.append(E_UNSUPPORTED_COMMAND)
    declaration_only = adapter_mode in {MODE_LOCAL_VLM_DISABLED, MODE_FUTURE_LOCAL_QWEN_ADAPTER}
    if not declaration_only:
        if raw_result.get("grounded") is not True:
            blocking_reasons.append(raw_result.get("error_code") or raw_result.get("rejection_reason") or E_NO_TARGET)
        if raw_result.get("rejected") is True:
            blocking_reasons.append(raw_result.get("error_code") or raw_result.get("rejection_reason") or E_NO_TARGET)
        if raw_result.get("grounded") is True and not raw_result.get("bbox_xyxy"):
            blocking_reasons.append(E_BBOX_MISSING)
        if raw_result.get("grounded") is True and not raw_result.get("pixel_center"):
            blocking_reasons.append(E_PIXEL_CENTER_MISSING)
    overall_confidence = _optional_float(raw_result.get("overall_confidence"))
    if not declaration_only and raw_result.get("grounded") is True and (
        overall_confidence is None or overall_confidence < float(threshold)
    ):
        blocking_reasons.append(E_LOW_CONFIDENCE)

    expected_snapshot_id = _string(config.get("expected_snapshot_id"))
    expected_scene_version = _string(config.get("expected_scene_version"))
    if expected_snapshot_id and raw_result.get("snapshot_id") != expected_snapshot_id:
        blocking_reasons.append(E_SNAPSHOT_MISMATCH)
    if expected_scene_version and raw_result.get("scene_version") != expected_scene_version:
        blocking_reasons.append(E_SCENE_VERSION_MISMATCH)

    blocking_reasons = _unique([str(reason) for reason in blocking_reasons if reason])
    warnings = _unique([str(warning) for warning in warnings if warning])
    status = _status_for(adapter_mode, blocking_reasons)
    no_motion_passed = status in {STATUS_PASS, STATUS_SAFE_DISABLED}
    error_code = _error_code(raw_result, blocking_reasons)
    rejected = bool(blocking_reasons) or raw_result.get("rejected") is True
    grounded = raw_result.get("grounded") is True and not blocking_reasons

    return {
        **_contract_fields(raw_result),
        "contract_version": CONTRACT_VERSION,
        "teto_version": CURRENT_VLM_GROUNDING_VERSION,
        "vlm_grounding_requested": True,
        "requested": True,
        "config_path": request.config_path,
        "vlm_grounding_status": status,
        "adapter_mode": adapter_mode,
        "user_command": user_command,
        "normalized_command": normalized_command,
        "grounded": grounded,
        "rejected": rejected,
        "rejection_reason": _rejection_reason(raw_result, blocking_reasons),
        "error_code": error_code,
        "blocking_reasons": blocking_reasons,
        "warnings": warnings,
        "next_safe_action": _next_safe_action(status, adapter_mode),
        "no_motion_grounding_passed": no_motion_passed,
        "allow_live_vlm": request.allow_live_vlm or config.get("allow_live_vlm") is True,
        "live_vlm_called": False,
        "live_camera_used": False,
        "real_robot_motion_executed": False,
        "real_robot_command_enabled": False,
        "robot_command_generated": False,
        "trajectory_generated": False,
        "joint_targets_generated": False,
        "tcp_pose_world_generated": False,
        "forbidden_robot_control_fields": _unique(forbidden_fields),
        "overall_confidence_threshold": threshold,
        "source_declaration": _source_declaration(adapter_mode),
        "safety_boundary": _safety_boundary(),
    }


def _not_requested_result() -> Dict[str, Any]:
    return {
        "contract_version": CONTRACT_VERSION,
        "teto_version": CURRENT_VLM_GROUNDING_VERSION,
        "vlm_grounding_requested": False,
        "requested": False,
        "vlm_grounding_status": STATUS_NOT_REQUESTED,
        "grounding_id": None,
        "snapshot_id": None,
        "scene_version": None,
        "adapter_mode": None,
        "user_command": None,
        "normalized_command": None,
        "target_label": None,
        "target_object_id": None,
        "bbox_xyxy": None,
        "pixel_center": None,
        "mask_ref": None,
        "semantic_confidence": None,
        "grounding_confidence": None,
        "overall_confidence": None,
        "grounded": False,
        "rejected": False,
        "rejection_reason": None,
        "error_code": None,
        "blocking_reasons": [],
        "warnings": [],
        "next_safe_action": None,
        "no_motion_grounding_passed": False,
        "live_vlm_called": False,
        "live_camera_used": False,
        "real_robot_motion_executed": False,
        "real_robot_command_enabled": False,
        "robot_command_generated": False,
        "trajectory_generated": False,
        "joint_targets_generated": False,
        "tcp_pose_world_generated": False,
        "safety_boundary": _safety_boundary(),
    }


def _mock_result(config: Dict[str, Any], user_command: str | None, normalized_command: str | None) -> Dict[str, Any]:
    result = _base_result(config, user_command, normalized_command)
    if normalized_command not in MOCK_COMMANDS:
        return {
            **result,
            "grounding_id": _string(config.get("grounding_id")) or "mock_grounding_blocked_unsupported_command",
            "grounded": False,
            "rejected": True,
            "rejection_reason": E_UNSUPPORTED_COMMAND,
            "error_code": E_UNSUPPORTED_COMMAND,
        }
    return {
        **result,
        "grounding_id": _string(config.get("grounding_id")) or "mock_grounding_red_mug_001",
        "target_label": _string(config.get("target_label")) or "red_mug",
        "target_object_id": _string(config.get("target_object_id")) or "mock_red_mug_001",
        "bbox_xyxy": config.get("bbox_xyxy") or [120, 80, 260, 220],
        "pixel_center": config.get("pixel_center") or [190, 150],
        "mask_ref": _string(config.get("mask_ref")),
        "semantic_confidence": _optional_float(config.get("semantic_confidence")) or 0.90,
        "grounding_confidence": _optional_float(config.get("grounding_confidence")) or 0.88,
        "overall_confidence": _optional_float(config.get("overall_confidence")) or 0.89,
        "grounded": True,
        "rejected": False,
    }


def _offline_result(config: Dict[str, Any]) -> Dict[str, Any]:
    raw = _load_declared_result(config.get("grounding_result_path")) or _dict(config.get("grounding_result")) or {}
    return {**_base_result(config, _string(config.get("user_command")), normalize_command(_string(config.get("user_command")))), **raw}


def _manual_result(config: Dict[str, Any], user_command: str | None, normalized_command: str | None) -> Dict[str, Any]:
    annotation = _dict(config.get("manual_annotation")) or _dict(config.get("grounding_result")) or config
    return {**_base_result(config, user_command, normalized_command), **annotation, "source": "manual_annotation"}


def _disabled_result(
    config: Dict[str, Any],
    user_command: str | None,
    normalized_command: str | None,
    adapter_mode: str,
) -> Dict[str, Any]:
    return {
        **_base_result(config, user_command, normalized_command),
        "grounding_id": _string(config.get("grounding_id")) or f"{adapter_mode}_declaration",
        "grounded": False,
        "rejected": False,
    }


def _base_result(config: Dict[str, Any], user_command: str | None, normalized_command: str | None) -> Dict[str, Any]:
    return {
        "grounding_id": _string(config.get("grounding_id")),
        "snapshot_id": _string(config.get("snapshot_id")),
        "scene_version": _string(config.get("scene_version")),
        "user_command": user_command,
        "normalized_command": normalized_command,
        "target_label": _string(config.get("target_label")),
        "target_object_id": _string(config.get("target_object_id")),
        "bbox_xyxy": config.get("bbox_xyxy"),
        "pixel_center": config.get("pixel_center"),
        "mask_ref": _string(config.get("mask_ref")),
        "semantic_confidence": _optional_float(config.get("semantic_confidence")),
        "grounding_confidence": _optional_float(config.get("grounding_confidence")),
        "overall_confidence": _optional_float(config.get("overall_confidence")),
        "grounded": config.get("grounded") is True,
        "rejected": config.get("rejected") is True,
        "rejection_reason": _string(config.get("rejection_reason")),
        "error_code": _string(config.get("error_code")),
        "live_vlm_called": config.get("live_vlm_called") is True,
    }


def _contract_fields(raw: Dict[str, Any]) -> Dict[str, Any]:
    return {
        "grounding_id": _string(raw.get("grounding_id")),
        "snapshot_id": _string(raw.get("snapshot_id")),
        "scene_version": _string(raw.get("scene_version")),
        "target_label": _string(raw.get("target_label")),
        "target_object_id": _string(raw.get("target_object_id")),
        "bbox_xyxy": raw.get("bbox_xyxy"),
        "pixel_center": raw.get("pixel_center"),
        "mask_ref": _string(raw.get("mask_ref")),
        "semantic_confidence": _optional_float(raw.get("semantic_confidence")),
        "grounding_confidence": _optional_float(raw.get("grounding_confidence")),
        "overall_confidence": _optional_float(raw.get("overall_confidence")),
    }


def _load_declared_result(path: Any) -> Dict[str, Any] | None:
    if not isinstance(path, str) or not path:
        return None
    resolved_path = Path(path).expanduser()
    if not resolved_path.is_file():
        return None
    with resolved_path.open("r", encoding="utf-8") as result_file:
        if resolved_path.suffix.lower() == ".json":
            data = json.load(result_file)
        else:
            data = yaml.safe_load(result_file)
    if not isinstance(data, dict):
        return {}
    result = data.get("grounding_result") or data.get("vlm_grounding_result")
    return result if isinstance(result, dict) else data


def _status_for(adapter_mode: str, blocking_reasons: list[str]) -> str:
    if blocking_reasons:
        return STATUS_BLOCKED
    if adapter_mode in {MODE_LOCAL_VLM_DISABLED, MODE_FUTURE_LOCAL_QWEN_ADAPTER}:
        return STATUS_SAFE_DISABLED
    return STATUS_PASS


def _error_code(raw_result: Dict[str, Any], blocking_reasons: list[str]) -> str | None:
    if blocking_reasons:
        return blocking_reasons[0]
    return _string(raw_result.get("error_code"))


def _rejection_reason(raw_result: Dict[str, Any], blocking_reasons: list[str]) -> str | None:
    if blocking_reasons:
        return blocking_reasons[0]
    return _string(raw_result.get("rejection_reason"))


def _next_safe_action(status: str, adapter_mode: str) -> str:
    if status == STATUS_PASS:
        return "Use this grounding result only as no-motion evidence for downstream validation."
    if status == STATUS_SAFE_DISABLED:
        return f"Adapter mode {adapter_mode} is declaration-only; use mock, offline, or manual grounding evidence."
    return "Fix the grounding evidence and rerun without live VLM, live camera, or robot control."


def _source_declaration(adapter_mode: str) -> Dict[str, Any]:
    return {
        "adapter_mode": adapter_mode,
        "offline_only": adapter_mode in {MODE_OFFLINE_GROUNDING_JSON, MODE_MANUAL_ANNOTATION, MODE_MOCK_VLM},
        "declaration_only": adapter_mode in {MODE_LOCAL_VLM_DISABLED, MODE_FUTURE_LOCAL_QWEN_ADAPTER},
        "live_model_invocation_supported": False,
    }


def _safety_boundary() -> Dict[str, bool]:
    return {
        "allow_live_vlm_default": False,
        "no_live_camera_capture": True,
        "no_live_vlm_call": True,
        "no_real_ur5_connection": True,
        "no_ros2": True,
        "no_moveit": True,
        "no_rtde": True,
        "no_urscript": True,
        "no_dashboard": True,
        "no_trajectory": True,
        "no_tcp_pose_world": True,
        "no_joint_targets": True,
        "no_robot_command": True,
        "no_real_execution_request": True,
    }


def _dict(value: Any) -> Dict[str, Any] | None:
    return value if isinstance(value, dict) else None


def _string(value: Any) -> str | None:
    return value if isinstance(value, str) and value else None


def _optional_float(value: Any) -> float | None:
    if isinstance(value, bool):
        return None
    if isinstance(value, (int, float)):
        return float(value)
    return None


def _unique(values: list[str]) -> list[str]:
    seen: set[str] = set()
    result: list[str] = []
    for value in values:
        if value not in seen:
            seen.add(value)
            result.append(value)
    return result
