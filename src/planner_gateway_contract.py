from typing import Any, Dict, List


CONTRACT_VERSION = "teto_planner_gateway_input.v1"
SUPPORTED_SCHEMA_VERSION = "teto_robot_task.v1"
DEFAULT_INTENT_NAME = "hover_to_object"
DEFAULT_MISSING_RUNTIME_INPUTS = [
    "camera_frame",
    "depth_aligned_to_color",
    "camera_point_m",
    "world_frame",
    "world_point_m",
    "tf_timestamp",
    "scene_ttl_ms",
    "robot_safety_state",
]
DEFAULT_PLANNER_REQUIREMENTS = {
    "requires_depth": True,
    "requires_tf": True,
    "requires_fresh_scene": True,
    "requires_workspace_check": True,
    "requires_collision_check": True,
    "requires_human_confirmation": True,
}


def evaluate_planner_gateway_eligibility(
    record_or_normalized_json: Dict[str, Any] | None,
    *,
    require_grounding: bool = True,
    grounded: bool | None = None,
) -> Dict[str, Any]:
    normalized = _extract_normalized_json(record_or_normalized_json)
    reasons: List[str] = []
    warnings: List[str] = []
    required_missing_fields: List[str] = []

    if normalized is None:
        return _eligibility_result(False, reasons=["E_MISSING_NORMALIZED_JSON"], required_missing_fields=["normalized_json"])

    schema_version = normalized.get("schema_version")
    scene = _dict_value(normalized.get("scene"))
    target = _dict_value(normalized.get("target"))
    manipulation = _dict_value(normalized.get("manipulation_assessment"))
    geometry = _dict_value(normalized.get("geometry_2d"))
    error = _dict_value(normalized.get("error"))

    scene_status = scene.get("status")
    candidate = target.get("candidate")
    if candidate is None:
        candidate = manipulation.get("candidate")
    error_code = error.get("code")
    target_id = target.get("target_id")
    target_label = target.get("label")
    bbox_xyxy = target.get("bbox_xyxy")
    pixel_center = geometry.get("pixel_center")
    geometry_confidence = _number_or_none(geometry.get("confidence"))
    normalized_grounded = bbox_xyxy is not None and pixel_center is not None
    effective_grounded = normalized_grounded if grounded is None else bool(grounded) and normalized_grounded

    if schema_version != SUPPORTED_SCHEMA_VERSION:
        reasons.append("E_SCHEMA_UNSUPPORTED")
        required_missing_fields.append("schema_version")
    if scene_status != "valid":
        reasons.append("E_SCENE_INVALID")
    if candidate is not True:
        reasons.append("E_NOT_CANDIDATE")
    if error_code != "OK":
        reasons.append(str(error_code) if error_code else "E_ERROR_CODE_NOT_OK")
    if _is_unknown(target_id) or _is_unknown(target_label):
        reasons.append("E_UNKNOWN_TARGET")
        if _is_unknown(target_id):
            required_missing_fields.append("target.target_id")
        if _is_unknown(target_label):
            required_missing_fields.append("target.label")
    if bbox_xyxy is None:
        reasons.append("E_MISSING_BBOX")
        required_missing_fields.append("target.bbox_xyxy")
    if pixel_center is None:
        reasons.append("E_MISSING_PIXEL_CENTER")
        required_missing_fields.append("geometry_2d.pixel_center")
    if require_grounding and not effective_grounded:
        reasons.append("E_NOT_GROUNDED")
    if geometry_confidence is None or geometry_confidence <= 0:
        reasons.append("E_LOW_GEOMETRY_CONFIDENCE")
        required_missing_fields.append("geometry_2d.confidence")

    unique_reasons = _unique(reasons)
    return _eligibility_result(
        not unique_reasons,
        reasons=unique_reasons,
        warnings=warnings,
        required_missing_fields=_unique(required_missing_fields),
    )


def build_planner_gateway_input(
    normalized_json: Dict[str, Any],
    *,
    task_id: str | None = None,
    intent_name: str = DEFAULT_INTENT_NAME,
) -> Dict[str, Any]:
    eligibility = evaluate_planner_gateway_eligibility(normalized_json)
    if not eligibility["eligible"]:
        raise ValueError(f"planner gateway input is not eligible: {', '.join(eligibility['reasons'])}")

    scene = _dict_value(normalized_json.get("scene"))
    target = _dict_value(normalized_json.get("target"))
    geometry = _dict_value(normalized_json.get("geometry_2d"))
    confidence = _dict_value(normalized_json.get("confidence"))
    return {
        "contract_version": CONTRACT_VERSION,
        "task_id": task_id or "task_unknown",
        "scene_version": scene.get("scene_version", "unknown"),
        "intent": {
            "name": intent_name,
            "source": "teto_semantic_ir",
            "user_instruction": normalized_json.get("user_instruction", "unknown"),
        },
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
            "grounded": True,
        },
        "confidence": {
            "semantic": confidence.get("semantic"),
            "geometry": geometry.get("confidence"),
            "overall": confidence.get("overall"),
        },
        "planner_requirements": dict(DEFAULT_PLANNER_REQUIREMENTS),
        "missing_runtime_inputs": list(DEFAULT_MISSING_RUNTIME_INPUTS),
        "execution_policy": {
            "dry_run_only": True,
            "allow_robot_motion": False,
            "max_speed_scale": None,
            "max_acc_scale": None,
        },
    }


def evaluate_replay_record_for_planner(
    replay_record: Dict[str, Any],
    result_record: Dict[str, Any],
    *,
    task_id: str | None = None,
    intent_name: str = DEFAULT_INTENT_NAME,
) -> Dict[str, Any]:
    normalized = _extract_normalized_json(result_record)
    eligibility = evaluate_planner_gateway_eligibility(
        normalized,
        grounded=replay_record.get("grounded") if isinstance(replay_record, dict) else None,
    )
    planner_input = None
    if eligibility["eligible"] and normalized is not None:
        planner_input = build_planner_gateway_input(normalized, task_id=task_id, intent_name=intent_name)
    return {
        "scene_version": replay_record.get("scene_version", "unknown") if isinstance(replay_record, dict) else "unknown",
        "eligible": eligibility["eligible"],
        "status": eligibility["status"],
        "reasons": eligibility["reasons"],
        "warnings": eligibility["warnings"],
        "required_missing_fields": eligibility["required_missing_fields"],
        "contract_version": CONTRACT_VERSION,
        "planner_input": planner_input,
    }


def _eligibility_result(
    eligible: bool,
    *,
    reasons: List[str],
    warnings: List[str] | None = None,
    required_missing_fields: List[str] | None = None,
) -> Dict[str, Any]:
    return {
        "eligible": eligible,
        "status": "eligible" if eligible else "rejected",
        "reasons": reasons,
        "warnings": warnings or [],
        "required_missing_fields": required_missing_fields or [],
        "contract_version": CONTRACT_VERSION,
    }


def _extract_normalized_json(record_or_normalized_json: Dict[str, Any] | None) -> Dict[str, Any] | None:
    if not isinstance(record_or_normalized_json, dict):
        return None
    normalized = record_or_normalized_json.get("normalized_json")
    if isinstance(normalized, dict):
        return normalized
    if "schema_version" in record_or_normalized_json:
        return record_or_normalized_json
    return None


def _dict_value(value: Any) -> Dict[str, Any]:
    return value if isinstance(value, dict) else {}


def _is_unknown(value: Any) -> bool:
    return value in (None, "", "unknown")


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
