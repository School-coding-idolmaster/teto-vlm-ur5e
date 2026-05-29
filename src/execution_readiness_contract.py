from typing import Any, Dict, List

from src.planner_gateway_contract import (
    CONTRACT_VERSION as PLANNER_CONTRACT_VERSION,
    DEFAULT_MISSING_RUNTIME_INPUTS as PLANNER_MISSING_RUNTIME_INPUTS,
    evaluate_planner_gateway_eligibility,
)
from src.projector_contract import (
    CONTRACT_VERSION as PROJECTOR_CONTRACT_VERSION,
    DEFAULT_MISSING_RUNTIME_INPUTS as PROJECTOR_MISSING_RUNTIME_INPUTS,
    evaluate_projector_eligibility,
)


CONTRACT_VERSION = "teto_execution_readiness.v1"


def evaluate_execution_readiness(normalized_json: Dict[str, Any] | None) -> Dict[str, Any]:
    planner = evaluate_planner_gateway_eligibility(normalized_json)
    projector = evaluate_projector_eligibility(normalized_json)
    planner_eligible = planner.get("eligible") is True
    projector_eligible = projector.get("eligible") is True

    if not planner_eligible:
        status = "planner_rejected"
        ready = False
    elif not projector_eligible:
        status = "projector_rejected"
        ready = False
    else:
        status = "dry_run_ready"
        ready = True

    return {
        "contract_version": CONTRACT_VERSION,
        "ready": ready,
        "status": status,
        "planner_eligible": planner_eligible,
        "projector_eligible": projector_eligible,
        "allow_robot_motion": False,
        "blocking_reasons": _unique(list(planner.get("reasons", [])) + list(projector.get("errors", []))),
        "warnings": _unique(list(planner.get("warnings", [])) + list(projector.get("warnings", []))),
    }


def build_execution_readiness_input(normalized_json: Dict[str, Any] | None) -> Dict[str, Any]:
    normalized = _extract_normalized_json(normalized_json)
    readiness = evaluate_execution_readiness(normalized)
    scene = _dict_value(normalized.get("scene") if normalized else {})
    target = _dict_value(normalized.get("target") if normalized else {})
    geometry = _dict_value(normalized.get("geometry_2d") if normalized else {})
    return {
        "contract_version": CONTRACT_VERSION,
        "readiness": readiness,
        "scene_version": scene.get("scene_version", "unknown"),
        "target": {
            "target_id": target.get("target_id", "unknown"),
            "label": target.get("label", "unknown"),
            "bbox_xyxy": target.get("bbox_xyxy"),
            "pixel_center": geometry.get("pixel_center"),
        },
        "grounding_2d": {
            "image_width": geometry.get("image_width"),
            "image_height": geometry.get("image_height"),
            "confidence": geometry.get("confidence"),
        },
        "contracts": {
            "planner": PLANNER_CONTRACT_VERSION,
            "projector": PROJECTOR_CONTRACT_VERSION,
        },
        "missing_runtime_inputs": _unique(
            list(PLANNER_MISSING_RUNTIME_INPUTS) + list(PROJECTOR_MISSING_RUNTIME_INPUTS)
        ),
        "execution_policy": {
            "dry_run_only": True,
            "allow_robot_motion": False,
        },
    }


def _extract_normalized_json(value: Dict[str, Any] | None) -> Dict[str, Any] | None:
    if not isinstance(value, dict):
        return None
    normalized = value.get("normalized_json")
    if isinstance(normalized, dict):
        return normalized
    if any(key in value for key in ("scene", "target", "geometry_2d", "error", "manipulation_assessment")):
        return value
    return None


def _dict_value(value: Any) -> Dict[str, Any]:
    return value if isinstance(value, dict) else {}


def _unique(values: List[str]) -> List[str]:
    seen = set()
    result = []
    for value in values:
        if value not in seen:
            seen.add(value)
            result.append(value)
    return result
