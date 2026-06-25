from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict

import yaml

from src.grounding.forbidden_fields import (
    FORBIDDEN_ROBOT_CONTROL_FIELDS,
    find_forbidden_robot_control_fields as _forbidden_robot_control_fields,
)


CONTRACT_VERSION = "teto_grounding_result.v1"
CURRENT_GROUNDING_VERSION = "TETO V2.9.0"


@dataclass(frozen=True)
class GroundingResultRequest:
    requested: bool = False
    result_path: str | None = None
    result: Dict[str, Any] | None = None


def load_grounding_result(path: str | Path | None) -> Dict[str, Any] | None:
    if not path:
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
    value = data.get("grounding_result")
    return value if isinstance(value, dict) else data


def build_grounding_result_request(
    *,
    requested: bool = False,
    result_path: str | Path | None = None,
) -> GroundingResultRequest:
    return GroundingResultRequest(
        requested=requested,
        result_path=str(Path(result_path).expanduser()) if result_path else None,
        result=load_grounding_result(result_path),
    )


def evaluate_grounding_result_contract(request: GroundingResultRequest | None = None) -> Dict[str, Any]:
    request = request or GroundingResultRequest()
    if not request.requested:
        return _not_requested_result()
    raw = request.result if isinstance(request.result, dict) else {}
    forbidden_fields = _forbidden_robot_control_fields(raw)
    warnings = list(raw.get("warnings") or []) if isinstance(raw.get("warnings"), list) else []
    return {
        "contract_version": CONTRACT_VERSION,
        "teto_version": CURRENT_GROUNDING_VERSION,
        "requested": True,
        "result_path": request.result_path,
        "grounding_id": _string(raw.get("grounding_id")),
        "snapshot_id": _string(raw.get("snapshot_id")),
        "scene_version": _string(raw.get("scene_version")),
        "source": _string(raw.get("source")) or "mock_grounding",
        "user_command": _string(raw.get("user_command")),
        "target_label": _string(raw.get("target_label")),
        "target_object_id": _string(raw.get("target_object_id")),
        "bbox_xyxy": raw.get("bbox_xyxy"),
        "pixel_center": raw.get("pixel_center"),
        "mask_ref": _string(raw.get("mask_ref")),
        "semantic_confidence": _optional_float(raw.get("semantic_confidence")),
        "grounding_confidence": _optional_float(raw.get("grounding_confidence")),
        "overall_confidence": _optional_float(raw.get("overall_confidence")),
        "grounded": raw.get("grounded") is True,
        "rejected": raw.get("rejected") is True,
        "rejection_reason": _string(raw.get("rejection_reason")),
        "error_code": _string(raw.get("error_code")),
        "warnings": warnings,
        "live_vlm_called": raw.get("live_vlm_called") is True,
        "live_camera_used": raw.get("live_camera_used") is True,
        "real_robot_motion_executed": False,
        "real_robot_command_enabled": False,
        "forbidden_robot_control_fields": forbidden_fields,
        "raw_keys": sorted(str(key) for key in raw.keys()),
    }


def _not_requested_result() -> Dict[str, Any]:
    return {
        "contract_version": CONTRACT_VERSION,
        "teto_version": CURRENT_GROUNDING_VERSION,
        "requested": False,
        "grounding_id": None,
        "snapshot_id": None,
        "scene_version": None,
        "warnings": [],
        "live_vlm_called": False,
        "live_camera_used": False,
        "real_robot_motion_executed": False,
        "real_robot_command_enabled": False,
        "forbidden_robot_control_fields": [],
    }


def _string(value: Any) -> str | None:
    return value if isinstance(value, str) and value else None


def _optional_float(value: Any) -> float | None:
    if isinstance(value, bool):
        return None
    if isinstance(value, (int, float)):
        return float(value)
    return None
