from typing import Any, Dict, List


CONTRACT_VERSION = "teto_projector.v1"
MIN_GEOMETRY_CONFIDENCE = 0.5
DEFAULT_MISSING_RUNTIME_INPUTS = [
    "depth_sample",
    "camera_info",
    "camera_frame",
    "camera_extrinsics",
    "tf_tree",
]
PROJECTOR_ERROR_CODES = [
    "OK",
    "E_NOT_CANDIDATE",
    "E_NOT_GROUNDED",
    "E_MISSING_PIXEL_CENTER",
    "E_MISSING_BBOX",
    "E_LOW_GEOMETRY_CONFIDENCE",
    "E_NO_DEPTH",
    "E_CAMERA_INFO_MISSING",
    "E_CAMERA_FRAME_MISSING",
    "E_TF_UNAVAILABLE",
    "E_WORLD_TRANSFORM_FAILED",
    "E_LOW_PROJECTOR_CONFIDENCE",
]


def evaluate_projector_eligibility(normalized_json: Dict[str, Any] | None) -> Dict[str, Any]:
    normalized = _extract_normalized_json(normalized_json)
    errors: List[str] = []
    warnings: List[str] = []
    target = _dict_value(normalized.get("target") if normalized else {})
    geometry = _dict_value(normalized.get("geometry_2d") if normalized else {})
    manipulation = _dict_value(normalized.get("manipulation_assessment") if normalized else {})

    candidate = target.get("candidate")
    if candidate is None:
        candidate = manipulation.get("candidate")
    bbox_xyxy = target.get("bbox_xyxy")
    pixel_center = geometry.get("pixel_center")
    geometry_confidence = _number_or_none(geometry.get("confidence"))
    grounded = _grounded_value(normalized, bbox_xyxy, pixel_center) if normalized else False

    if candidate is not True:
        errors.append("E_NOT_CANDIDATE")
    if grounded is not True:
        errors.append("E_NOT_GROUNDED")
    if pixel_center is None:
        errors.append("E_MISSING_PIXEL_CENTER")
    if bbox_xyxy is None:
        errors.append("E_MISSING_BBOX")
    if geometry_confidence is None or geometry_confidence < MIN_GEOMETRY_CONFIDENCE:
        errors.append("E_LOW_GEOMETRY_CONFIDENCE")

    eligible = not errors
    return {
        "contract_version": CONTRACT_VERSION,
        "projector_status": "ready" if eligible else "rejected",
        "eligible": eligible,
        "camera_point_m": None,
        "world_point_m": None,
        "projector_confidence": 0.0,
        "missing_runtime_inputs": list(DEFAULT_MISSING_RUNTIME_INPUTS),
        "errors": [] if eligible else _unique(errors),
        "warnings": warnings,
        "allow_robot_motion": False,
    }


def build_projector_input(normalized_json: Dict[str, Any]) -> Dict[str, Any]:
    normalized = _extract_normalized_json(normalized_json)
    if normalized is None:
        raise ValueError("projector input requires normalized_json")

    target = _dict_value(normalized.get("target"))
    geometry = _dict_value(normalized.get("geometry_2d"))
    return {
        "contract_version": CONTRACT_VERSION,
        "pixel_center": geometry.get("pixel_center"),
        "bbox_xyxy": target.get("bbox_xyxy"),
        "image_width": geometry.get("image_width"),
        "image_height": geometry.get("image_height"),
        "camera_frame": None,
        "depth_sample": None,
        "allow_robot_motion": False,
    }


def _extract_normalized_json(value: Dict[str, Any] | None) -> Dict[str, Any] | None:
    if not isinstance(value, dict):
        return None
    normalized = value.get("normalized_json")
    if isinstance(normalized, dict):
        return normalized
    if any(key in value for key in ("target", "geometry_2d", "manipulation_assessment")):
        return value
    return None


def _dict_value(value: Any) -> Dict[str, Any]:
    return value if isinstance(value, dict) else {}


def _grounded_value(normalized: Dict[str, Any], bbox_xyxy: Any, pixel_center: Any) -> bool:
    if "grounded" in normalized:
        return normalized.get("grounded") is True
    return bbox_xyxy is not None and pixel_center is not None


def _number_or_none(value: Any) -> float | None:
    if isinstance(value, bool):
        return None
    if isinstance(value, (int, float)):
        return float(value)
    return None


def _unique(values: List[str]) -> List[str]:
    seen = set()
    result = []
    for value in values:
        if value not in seen:
            seen.add(value)
            result.append(value)
    return result
