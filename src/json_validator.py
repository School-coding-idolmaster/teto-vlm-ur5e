import json
from copy import deepcopy
from datetime import datetime
import hashlib
from pathlib import Path
from typing import Any, Dict, List, Tuple

from PIL import Image


ROBOT_TASK_PROMPT_TYPE = "robot_task_json"

APPROX_POSITIONS = {"left", "right", "center", "top", "bottom", "front", "back", "edge", "unknown"}
VISIBILITIES = {"clear", "partially_occluded", "heavily_occluded", "unknown"}
SURFACES = {"table", "floor", "shelf", "unknown"}
RELATIONS = {
    "left_of",
    "right_of",
    "in_front_of",
    "behind",
    "on",
    "under",
    "near",
    "far",
    "overlapping",
    "unknown",
}
DIFFICULTIES = {"easy", "medium", "hard", "unsafe", "unknown"}
CONFIDENCE_LEVELS = {"high", "medium", "low", "unknown"}
ERROR_CODES = {
    "OK",
    "E_NO_TARGET",
    "E_UNCLEAR_IMAGE",
    "E_AMBIGUOUS_TARGET",
    "E_UNSAFE",
    "E_PARSE",
    "E_PARSE_FAILED",
}
NO_TARGET_OR_UNSAFE_CODES = {"E_NO_TARGET", "E_UNCLEAR_IMAGE", "E_AMBIGUOUS_TARGET", "E_UNSAFE"}

LIVING_TARGET_TERMS = {
    "human",
    "person",
    "people",
    "man",
    "woman",
    "child",
    "boy",
    "girl",
    "baby",
    "animal",
    "bird",
    "cat",
    "dog",
    "horse",
    "fish",
    "rabbit",
    "pet",
    "living_being",
}
UNSAFE_TARGET_TERMS = {
    "knife",
    "blade",
    "scissors",
    "needle",
    "glass",
    "mirror",
    "vase",
    "fire",
    "flame",
    "hot",
    "liquid",
    "water",
    "transparent",
    "reflective",
    "sharp",
    "dangerous",
    "fragile",
}


DEFAULT_ROBOT_TASK_JSON = {
    "schema_version": "teto_robot_task.v1",
    "task_type": "target_analysis",
    "user_instruction": "unknown",
    "scene": {
        "scene_version": "unknown",
        "capture_timestamp": "unknown",
        "image_path": "unknown",
        "image_width": None,
        "image_height": None,
        "record_type": "legacy_rgb_only_record",
        "source": "legacy_semantic_image",
        "is_realsense_scene_snapshot": False,
        "status": "unknown",
    },
    "target": {
        "target_id": "unknown",
        "label": "unknown",
        "candidate": False,
        "bbox_xyxy": None,
        "approx_position": "unknown",
        "visibility": "unknown",
    },
    "geometry_2d": {
        "pixel_center": None,
        "image_width": None,
        "image_height": None,
        "confidence": None,
    },
    "spatial_context": {
        "surface": "unknown",
        "nearby_objects": [],
        "relations": [],
        "obstacles": [],
    },
    "manipulation_assessment": {
        "candidate": False,
        "difficulty": "unknown",
        "reason": "unknown",
    },
    "confidence": {
        "semantic": "unknown",
        "spatial": "unknown",
        "overall": "unknown",
    },
    "error": {
        "code": "OK",
        "message": "",
    },
}


def extract_json_from_response(raw_response: str) -> Dict[str, Any]:
    text = raw_response.strip() if isinstance(raw_response, str) else ""
    if not text:
        return {
            "data": None,
            "parse_status": "failed",
            "validation_errors": ["response is empty"],
        }

    try:
        data = json.loads(text)
        if isinstance(data, dict):
            return {"data": data, "parse_status": "success", "validation_errors": []}
        return {
            "data": None,
            "parse_status": "failed",
            "validation_errors": ["top-level JSON value must be an object"],
        }
    except json.JSONDecodeError as first_error:
        return {
            "data": None,
            "parse_status": "failed",
            "validation_errors": [f"invalid JSON: {first_error.msg}"],
        }


def validate_robot_task_json(data: Dict[str, Any]) -> List[str]:
    if not isinstance(data, dict):
        return ["top-level JSON value must be an object"]

    errors: List[str] = []
    _require_object(data, "target", errors)
    _require_object(data, "spatial_context", errors)
    _require_object(data, "manipulation_assessment", errors)
    _require_object(data, "confidence", errors)
    _require_object(data, "error", errors)

    target = data.get("target", {}) if isinstance(data.get("target"), dict) else {}
    spatial_context = data.get("spatial_context", {}) if isinstance(data.get("spatial_context"), dict) else {}
    assessment = (
        data.get("manipulation_assessment", {})
        if isinstance(data.get("manipulation_assessment"), dict)
        else {}
    )
    confidence = data.get("confidence", {}) if isinstance(data.get("confidence"), dict) else {}
    error = data.get("error", {}) if isinstance(data.get("error"), dict) else {}

    _validate_vocab(target, "target.approx_position", APPROX_POSITIONS, errors)
    _validate_vocab(target, "target.visibility", VISIBILITIES, errors)
    _validate_vocab(spatial_context, "spatial_context.surface", SURFACES, errors)
    _validate_vocab(assessment, "manipulation_assessment.difficulty", DIFFICULTIES, errors)
    _validate_vocab(confidence, "confidence.semantic", CONFIDENCE_LEVELS, errors)
    _validate_vocab(confidence, "confidence.spatial", CONFIDENCE_LEVELS, errors)
    _validate_vocab(confidence, "confidence.overall", CONFIDENCE_LEVELS, errors)
    _validate_vocab(error, "error.code", ERROR_CODES, errors)

    relations = spatial_context.get("relations", [])
    if not isinstance(relations, list):
        errors.append("spatial_context.relations must be a list")
    else:
        for index, relation in enumerate(relations):
            if not isinstance(relation, dict):
                errors.append(f"spatial_context.relations[{index}] must be an object")
                continue
            _validate_vocab(relation, f"spatial_context.relations[{index}].relation", RELATIONS, errors)
            _validate_vocab(
                relation,
                f"spatial_context.relations[{index}].confidence",
                CONFIDENCE_LEVELS,
                errors,
            )

    for field in ("nearby_objects", "obstacles"):
        value = spatial_context.get(field, [])
        if not isinstance(value, list):
            errors.append(f"spatial_context.{field} must be a list")

    if "candidate" not in assessment:
        errors.append("missing field: manipulation_assessment.candidate")
    elif not isinstance(assessment["candidate"], bool):
        errors.append("manipulation_assessment.candidate must be a boolean")

    return errors


def validate_robot_task_safety(data: Dict[str, Any]) -> Tuple[List[str], List[str]]:
    errors: List[str] = []
    warnings: List[str] = []
    if not isinstance(data, dict):
        return errors, warnings

    target = data.get("target", {}) if isinstance(data.get("target"), dict) else {}
    assessment = (
        data.get("manipulation_assessment", {})
        if isinstance(data.get("manipulation_assessment"), dict)
        else {}
    )
    error = data.get("error", {}) if isinstance(data.get("error"), dict) else {}

    living_label = _find_matching_label(data, LIVING_TARGET_TERMS)
    unsafe_label = _find_matching_label(data, UNSAFE_TARGET_TERMS)
    label = _string_value(target.get("label"), "unknown")
    candidate = assessment.get("candidate")
    difficulty = assessment.get("difficulty")
    error_code = error.get("code")

    if living_label:
        if candidate is not False:
            errors.append("living target must have manipulation_assessment.candidate=false")
        if difficulty != "unsafe":
            errors.append("living target must have manipulation_assessment.difficulty=unsafe")
        if error_code != "E_UNSAFE":
            errors.append("living target must have error.code=E_UNSAFE")
        warnings.append(f"living object label detected: {living_label}")
    elif unsafe_label:
        if candidate is not False:
            errors.append("unsafe target must have manipulation_assessment.candidate=false")
        if difficulty not in {"unsafe", "hard"}:
            errors.append("unsafe target must have manipulation_assessment.difficulty=unsafe or hard")
        if error_code not in {"E_UNSAFE", "OK"}:
            errors.append("unsafe target should use error.code=E_UNSAFE when not a candidate")
        warnings.append(f"unsafe or difficult object label detected: {unsafe_label}")

    if label == "unknown":
        if candidate is True:
            errors.append("unknown target must not have manipulation_assessment.candidate=true")
            warnings.append("unknown target was marked as a manipulation candidate")
        if error_code not in {"E_NO_TARGET", "E_UNCLEAR_IMAGE", "E_AMBIGUOUS_TARGET", "E_UNSAFE"}:
            errors.append("unknown target should use a no-target, unclear, ambiguous, or unsafe error code")

    return errors, warnings


def normalize_robot_task_json(
    data: Dict[str, Any],
    image_size: Tuple[int, int] | None = None,
    scene_context: Dict[str, Any] | None = None,
) -> Dict[str, Any]:
    if not isinstance(data, dict):
        normalized = deepcopy(DEFAULT_ROBOT_TASK_JSON)
        _apply_legacy_semantic_image_record(normalized, scene_context, image_size, "invalid")
        return {
            "parsed_json": deepcopy(DEFAULT_ROBOT_TASK_JSON),
            "normalized_json": normalized,
            "parse_status": "failed",
            "validation_status": "failed",
            "raw_validation_errors": ["top-level JSON value must be an object"],
            "pre_normalization_errors": ["top-level JSON value must be an object"],
            "normalized_validation_errors": ["top-level JSON value must be an object"],
            "post_normalization_errors": ["top-level JSON value must be an object"],
            "validation_errors": ["top-level JSON value must be an object"],
            "validation_warnings": [],
        }

    pre_normalization_errors = validate_robot_task_json(data)
    safety_errors, validation_warnings = validate_robot_task_safety(data)
    pre_normalization_errors.extend(safety_errors)
    geometry_errors: List[str] = []
    normalized = deepcopy(DEFAULT_ROBOT_TASK_JSON)

    normalized["schema_version"] = _string_value(data.get("schema_version"), "teto_robot_task.v1")
    normalized["task_type"] = _string_value(data.get("task_type"), "target_analysis")
    normalized["user_instruction"] = _string_value(data.get("user_instruction"), "unknown")

    target = data.get("target", {}) if isinstance(data.get("target"), dict) else {}
    normalized["target"]["label"] = _string_value(target.get("label"), "unknown")
    normalized["target"]["approx_position"] = _controlled_value(
        target.get("approx_position"), APPROX_POSITIONS
    )
    normalized["target"]["visibility"] = _controlled_value(target.get("visibility"), VISIBILITIES)

    spatial_context = data.get("spatial_context", {}) if isinstance(data.get("spatial_context"), dict) else {}
    normalized["spatial_context"]["surface"] = _controlled_value(
        spatial_context.get("surface"), SURFACES
    )
    normalized["spatial_context"]["nearby_objects"] = _string_list(
        spatial_context.get("nearby_objects")
    )
    normalized["spatial_context"]["relations"] = _normalize_relations(
        spatial_context.get("relations")
    )
    normalized["spatial_context"]["obstacles"] = _string_list(spatial_context.get("obstacles"))

    assessment = (
        data.get("manipulation_assessment", {})
        if isinstance(data.get("manipulation_assessment"), dict)
        else {}
    )
    normalized["manipulation_assessment"]["candidate"] = (
        assessment.get("candidate") if isinstance(assessment.get("candidate"), bool) else False
    )
    normalized["manipulation_assessment"]["difficulty"] = _controlled_value(
        assessment.get("difficulty"), DIFFICULTIES
    )
    normalized["manipulation_assessment"]["reason"] = _string_value(
        assessment.get("reason"), "unknown"
    )
    normalized["target"]["candidate"] = normalized["manipulation_assessment"]["candidate"]

    confidence = data.get("confidence", {}) if isinstance(data.get("confidence"), dict) else {}
    normalized["confidence"]["semantic"] = _controlled_value(
        confidence.get("semantic"), CONFIDENCE_LEVELS
    )
    normalized["confidence"]["spatial"] = _controlled_value(
        confidence.get("spatial"), CONFIDENCE_LEVELS
    )
    normalized["confidence"]["overall"] = _controlled_value(
        confidence.get("overall"), CONFIDENCE_LEVELS
    )

    error = data.get("error", {}) if isinstance(data.get("error"), dict) else {}
    normalized["error"]["code"] = _controlled_value(error.get("code"), ERROR_CODES, "OK")
    normalized["error"]["message"] = _string_value(error.get("message"), "")

    _normalize_geometry_2d(data, normalized, geometry_errors, validation_warnings, image_size)
    pre_normalization_errors.extend(geometry_errors)
    _apply_safety_overrides(normalized, data)
    normalized["target"]["candidate"] = normalized["manipulation_assessment"]["candidate"]
    _clear_grounding_for_no_target(normalized)

    normalized_validation_errors = validate_robot_task_json(normalized)
    normalized_safety_errors, normalized_safety_warnings = validate_robot_task_safety(normalized)
    normalized_validation_errors.extend(normalized_safety_errors)
    validation_warnings.extend(_new_warnings_only(normalized_safety_warnings, validation_warnings))
    status = "invalid" if normalized_validation_errors else "valid"
    _apply_target_id(normalized)
    _apply_legacy_semantic_image_record(normalized, scene_context, image_size, status)

    return {
        "normalized_json": normalized,
        "parse_status": "success",
        "validation_status": _validation_status(normalized_validation_errors, validation_warnings),
        "raw_validation_errors": pre_normalization_errors,
        "pre_normalization_errors": pre_normalization_errors,
        "normalized_validation_errors": normalized_validation_errors,
        "post_normalization_errors": normalized_validation_errors,
        "validation_errors": normalized_validation_errors,
        "validation_warnings": validation_warnings,
    }


def parse_robot_task_response(
    raw_response: str,
    image_size: Tuple[int, int] | None = None,
    scene_context: Dict[str, Any] | None = None,
) -> Dict[str, Any]:
    extracted = extract_json_from_response(raw_response)
    if extracted["parse_status"] == "failed":
        normalized = deepcopy(DEFAULT_ROBOT_TASK_JSON)
        if image_size:
            normalized["geometry_2d"]["image_width"] = image_size[0]
            normalized["geometry_2d"]["image_height"] = image_size[1]
        _apply_legacy_semantic_image_record(normalized, scene_context, image_size, "invalid")
        normalized["error"]["code"] = "E_PARSE"
        normalized["error"]["message"] = "; ".join(extracted["validation_errors"])
        return {
            "raw_response": raw_response,
            "parsed_json": None,
            "normalized_json": normalized,
            "parse_status": "failed",
            "validation_status": "failed",
            "raw_validation_errors": extracted["validation_errors"],
            "pre_normalization_errors": extracted["validation_errors"],
            "normalized_validation_errors": extracted["validation_errors"],
            "post_normalization_errors": extracted["validation_errors"],
            "validation_errors": extracted["validation_errors"],
            "validation_warnings": [],
        }

    normalized = normalize_robot_task_json(
        extracted["data"],
        image_size=image_size,
        scene_context=scene_context,
    )
    return {
        "raw_response": raw_response,
        "parsed_json": deepcopy(extracted["data"]),
        **normalized,
    }


def attach_robot_task_json_fields(prompt_type: str, item: Dict[str, Any]) -> Dict[str, Any]:
    if prompt_type != ROBOT_TASK_PROMPT_TYPE:
        return item

    image_size, image_size_warning = _read_image_size(item.get("image_path"))
    scene_context = {
        "image_path": item.get("image_path"),
        "run_id": item.get("run_name") or item.get("run_id"),
        "item_index": item.get("item_index") or item.get("index"),
        "capture_timestamp": item.get("created_at"),
    }
    parsed = parse_robot_task_response(
        str(item.get("response", "")),
        image_size=image_size,
        scene_context=scene_context,
    )
    item.update(parsed)
    if image_size_warning:
        item.setdefault("validation_warnings", []).append(image_size_warning)
        normalized = item.get("normalized_json")
        if isinstance(normalized, dict):
            geometry = normalized.setdefault("geometry_2d", {})
            geometry.setdefault("image_width", None)
            geometry.setdefault("image_height", None)
        if item.get("validation_status") == "passed":
            item["validation_status"] = "warning"
    return item


def _require_object(data: Dict[str, Any], field: str, errors: List[str]) -> None:
    if field not in data:
        errors.append(f"missing field: {field}")
    elif not isinstance(data[field], dict):
        errors.append(f"{field} must be an object")


def _validate_vocab(data: Dict[str, Any], field: str, allowed_values: set[str], errors: List[str]) -> None:
    key = field.split(".")[-1]
    if key not in data:
        errors.append(f"missing field: {field}")
        return
    if data[key] not in allowed_values:
        allowed = ", ".join(sorted(allowed_values))
        errors.append(f"{field} must be one of: {allowed}")


def _string_value(value: Any, default: str) -> str:
    return value if isinstance(value, str) and value else default


def _controlled_value(value: Any, allowed_values: set[str], default: str = "unknown") -> str:
    return value if isinstance(value, str) and value in allowed_values else default


def _string_list(value: Any) -> List[str]:
    if not isinstance(value, list):
        return []
    return [item for item in value if isinstance(item, str)]


def _normalize_relations(value: Any) -> List[Dict[str, str]]:
    if not isinstance(value, list):
        return []

    relations = []
    for item in value:
        if not isinstance(item, dict):
            continue
        relations.append(
            {
                "object": _string_value(item.get("object"), "unknown"),
                "relation": _controlled_value(item.get("relation"), RELATIONS),
                "target": _string_value(item.get("target"), "unknown"),
                "confidence": _controlled_value(item.get("confidence"), CONFIDENCE_LEVELS),
            }
        )
    return relations


def _normalize_geometry_2d(
    source: Dict[str, Any],
    normalized: Dict[str, Any],
    validation_errors: List[str],
    validation_warnings: List[str],
    image_size: Tuple[int, int] | None,
) -> None:
    bbox = _first_nested_value(
        source,
        ("target", "bbox_xyxy"),
        ("target", "bbox"),
        ("geometry_2d", "bbox_xyxy"),
        ("objects", 0, "bbox_xyxy"),
        ("objects", 0, "bbox"),
        ("candidates", 0, "bbox_xyxy"),
        ("candidates", 0, "bbox"),
    )
    geometry = source.get("geometry_2d", {}) if isinstance(source.get("geometry_2d"), dict) else {}
    pixel_center = _first_nested_value(
        source,
        ("geometry_2d", "pixel_center"),
        ("target", "pixel_center"),
        ("objects", 0, "pixel_center"),
        ("candidates", 0, "pixel_center"),
    )
    confidence = _normalize_optional_float(geometry.get("confidence"))
    if confidence is None:
        confidence = _normalize_optional_float(_first_nested_value(source, ("target", "grounding_confidence")))

    model_width = _normalize_optional_int(geometry.get("image_width"))
    model_height = _normalize_optional_int(geometry.get("image_height"))
    image_width = image_size[0] if image_size else model_width
    image_height = image_size[1] if image_size else model_height

    normalized_bbox = _normalize_bbox(bbox, image_width, image_height, validation_errors, validation_warnings)
    normalized_center = _normalize_pixel_center(
        pixel_center,
        image_width,
        image_height,
        validation_errors,
        validation_warnings,
    )
    if normalized_bbox is not None and normalized_center is None:
        normalized_center = [
            (normalized_bbox[0] + normalized_bbox[2]) / 2,
            (normalized_bbox[1] + normalized_bbox[3]) / 2,
        ]
        validation_warnings.append("pixel_center was inferred from bbox_xyxy")

    if normalized_bbox is None and normalized_center is None:
        validation_warnings.append("2D grounding is missing")

    normalized["target"]["bbox_xyxy"] = normalized_bbox
    normalized["geometry_2d"] = {
        "pixel_center": normalized_center,
        "image_width": image_width,
        "image_height": image_height,
        "confidence": confidence,
    }


def _normalize_bbox(
    value: Any,
    image_width: int | None,
    image_height: int | None,
    validation_errors: List[str],
    validation_warnings: List[str],
) -> List[float] | None:
    if value is None:
        return None
    if not isinstance(value, list) or len(value) != 4:
        validation_errors.append("target.bbox_xyxy must be null or a list of 4 numbers")
        return None
    if not all(_is_number(item) for item in value):
        validation_errors.append("target.bbox_xyxy values must be numbers")
        return None

    bbox = [float(item) for item in value]
    x_min, y_min, x_max, y_max = bbox
    if x_min >= x_max:
        validation_errors.append("target.bbox_xyxy must satisfy x_min < x_max")
        return None
    if y_min >= y_max:
        validation_errors.append("target.bbox_xyxy must satisfy y_min < y_max")
        return None

    if image_width is not None and (x_min < 0 or x_max > image_width):
        validation_warnings.append("target.bbox_xyxy x coordinates are outside image bounds")
    if image_height is not None and (y_min < 0 or y_max > image_height):
        validation_warnings.append("target.bbox_xyxy y coordinates are outside image bounds")
    return bbox


def _normalize_pixel_center(
    value: Any,
    image_width: int | None,
    image_height: int | None,
    validation_errors: List[str],
    validation_warnings: List[str],
) -> List[float] | None:
    if value is None:
        return None
    if not isinstance(value, list) or len(value) != 2:
        validation_errors.append("geometry_2d.pixel_center must be null or a list of 2 numbers")
        return None
    if not all(_is_number(item) for item in value):
        validation_errors.append("geometry_2d.pixel_center values must be numbers")
        return None

    center = [float(item) for item in value]
    cx, cy = center
    if image_width is not None and not 0 <= cx <= image_width:
        validation_warnings.append("geometry_2d.pixel_center x coordinate is outside image bounds")
    if image_height is not None and not 0 <= cy <= image_height:
        validation_warnings.append("geometry_2d.pixel_center y coordinate is outside image bounds")
    return center


def _read_image_size(image_path: Any) -> Tuple[Tuple[int, int] | None, str]:
    if not image_path:
        return None, "image size could not be read: image_path is missing"
    try:
        with Image.open(Path(str(image_path)).expanduser()) as image:
            return image.size, ""
    except Exception as exc:
        return None, f"image size could not be read: {exc}"


def _first_nested_value(data: Dict[str, Any], *paths: Tuple[Any, ...]) -> Any:
    for path in paths:
        current: Any = data
        for key in path:
            if isinstance(key, int):
                if not isinstance(current, list) or key >= len(current):
                    current = None
                    break
                current = current[key]
            else:
                if not isinstance(current, dict) or key not in current:
                    current = None
                    break
                current = current[key]
        if current is not None:
            return current
    return None


def _normalize_optional_float(value: Any) -> float | None:
    if not _is_number(value):
        return None
    number = float(value)
    if 0.0 <= number <= 1.0:
        return number
    return None


def _normalize_optional_int(value: Any) -> int | None:
    if isinstance(value, bool):
        return None
    if isinstance(value, int) and value > 0:
        return value
    return None


def _is_number(value: Any) -> bool:
    return isinstance(value, (int, float)) and not isinstance(value, bool)


def _matches_terms(label: str, terms: set[str]) -> bool:
    lower_label = label.lower()
    normalized = lower_label.replace("_", " ").replace("-", " ")
    words = set(normalized.split())
    for term in terms:
        normalized_term = term.lower().replace("_", " ").replace("-", " ")
        if term in lower_label or normalized_term in words or normalized_term in normalized:
            return True
    return False


def _apply_safety_overrides(data: Dict[str, Any], source: Dict[str, Any] | None = None) -> None:
    label = data["target"]["label"]
    assessment = data["manipulation_assessment"]
    error = data["error"]
    label_source = source if source is not None else data
    living_label = _find_matching_label(label_source, LIVING_TARGET_TERMS)
    unsafe_label = _find_matching_label(label_source, UNSAFE_TARGET_TERMS)

    if living_label:
        assessment["candidate"] = False
        assessment["difficulty"] = "unsafe"
        if assessment["reason"] == "unknown":
            assessment["reason"] = "Living humans or animals are not robot manipulation candidates."
        error["code"] = "E_UNSAFE"
        if not error["message"]:
            if label == "unknown":
                error["message"] = (
                    "Only living beings or unsafe targets were detected; "
                    "no manipulation candidate is allowed."
                )
            else:
                error["message"] = "Living target is unsafe for robot manipulation."
        return

    if unsafe_label:
        assessment["candidate"] = False
        if assessment["difficulty"] not in {"unsafe", "hard"}:
            assessment["difficulty"] = "unsafe"
        if assessment["reason"] == "unknown":
            assessment["reason"] = "Target appears unsafe or difficult for robot manipulation."
        if assessment["difficulty"] == "unsafe":
            error["code"] = "E_UNSAFE"
            if not error["message"]:
                error["message"] = "Target is unsafe for robot manipulation."
        return

    if label == "unknown":
        assessment["candidate"] = False
        if error["code"] == "OK":
            error["code"] = "E_NO_TARGET"
        if not error["message"]:
            error["message"] = "No suitable manipulation target was identified."


def _clear_grounding_for_no_target(data: Dict[str, Any]) -> None:
    target = data.get("target", {}) if isinstance(data.get("target"), dict) else {}
    geometry = data.get("geometry_2d", {}) if isinstance(data.get("geometry_2d"), dict) else {}
    assessment = (
        data.get("manipulation_assessment", {})
        if isinstance(data.get("manipulation_assessment"), dict)
        else {}
    )
    error = data.get("error", {}) if isinstance(data.get("error"), dict) else {}
    no_clear_target = (
        assessment.get("candidate") is False
        and (
            target.get("label") == "unknown"
            or error.get("code") in NO_TARGET_OR_UNSAFE_CODES
        )
    )
    if not no_clear_target:
        return

    target["bbox_xyxy"] = None
    geometry["pixel_center"] = None
    if geometry.get("confidence") is not None:
        geometry["confidence"] = 0.0


def _apply_target_id(data: Dict[str, Any]) -> None:
    target = data.get("target", {}) if isinstance(data.get("target"), dict) else {}
    assessment = (
        data.get("manipulation_assessment", {})
        if isinstance(data.get("manipulation_assessment"), dict)
        else {}
    )
    label = target.get("label")
    target["target_id"] = (
        "obj_001"
        if assessment.get("candidate") is True and isinstance(label, str) and label != "unknown"
        else "unknown"
    )


def _apply_legacy_semantic_image_record(
    data: Dict[str, Any],
    scene_context: Dict[str, Any] | None,
    image_size: Tuple[int, int] | None,
    status: str,
) -> None:
    context = scene_context or {}
    geometry = data.get("geometry_2d", {}) if isinstance(data.get("geometry_2d"), dict) else {}
    image_width = image_size[0] if image_size else geometry.get("image_width")
    image_height = image_size[1] if image_size else geometry.get("image_height")
    if isinstance(geometry, dict):
        geometry["image_width"] = image_width
        geometry["image_height"] = image_height

    data["scene"] = {
        "scene_version": _scene_version(context),
        "capture_timestamp": _scene_timestamp(context.get("capture_timestamp")),
        "image_path": _string_value(context.get("image_path"), "unknown"),
        "image_width": image_width,
        "image_height": image_height,
        "record_type": "legacy_rgb_only_record",
        "source": "legacy_semantic_image",
        "is_realsense_scene_snapshot": False,
        "missing_realsense_fields": [
            "snapshot_id",
            "depth_ref",
            "camera_info_ref",
            "metadata_ref",
            "tf_snapshot_ref",
        ],
        "status": status if status in {"valid", "invalid", "unknown"} else "unknown",
    }


def _scene_version(context: Dict[str, Any]) -> str:
    run_id = _string_value(context.get("run_id"), "")
    item_index = context.get("item_index")
    if run_id and isinstance(item_index, int):
        return f"{run_id}_item_{item_index:03d}"
    if run_id and isinstance(item_index, str) and item_index.isdigit():
        return f"{run_id}_item_{int(item_index):03d}"

    image_path = _string_value(context.get("image_path"), "unknown")
    digest_source = f"{run_id}|{item_index}|{image_path}".encode("utf-8")
    digest = hashlib.sha1(digest_source).hexdigest()[:12]
    return f"scene_{digest}"


def _scene_timestamp(value: Any) -> str:
    if isinstance(value, str) and value:
        return value.replace(" ", "T")
    return datetime.now().isoformat(timespec="seconds")


def _new_warnings_only(new_warnings: List[str], existing_warnings: List[str]) -> List[str]:
    return [warning for warning in new_warnings if warning not in existing_warnings]


def _find_matching_label(data: Dict[str, Any], terms: set[str]) -> str:
    for label in _iter_object_labels(data):
        if _matches_terms(label, terms):
            return label
    return ""


def _iter_object_labels(data: Dict[str, Any]):
    target = data.get("target", {}) if isinstance(data.get("target"), dict) else {}
    for value in (
        target.get("label"),
        data.get("candidate_label"),
        data.get("object_label"),
    ):
        if isinstance(value, str) and value:
            yield value

    assessment = (
        data.get("manipulation_assessment", {})
        if isinstance(data.get("manipulation_assessment"), dict)
        else {}
    )
    candidate_label = assessment.get("candidate_label")
    if isinstance(candidate_label, str) and candidate_label:
        yield candidate_label

    spatial_context = data.get("spatial_context", {}) if isinstance(data.get("spatial_context"), dict) else {}
    for field in ("nearby_objects", "obstacles"):
        values = spatial_context.get(field, [])
        if isinstance(values, list):
            for value in values:
                if isinstance(value, str) and value:
                    yield value

    relations = spatial_context.get("relations", [])
    if isinstance(relations, list):
        for relation in relations:
            if not isinstance(relation, dict):
                continue
            for key in ("object", "target", "label", "candidate_label"):
                value = relation.get(key)
                if isinstance(value, str) and value:
                    yield value

    for field in ("objects", "detected_objects", "visible_objects", "scene_objects"):
        values = data.get(field, [])
        if isinstance(values, list):
            for value in values:
                if isinstance(value, str) and value:
                    yield value
                elif isinstance(value, dict):
                    for key in ("label", "name", "object", "target", "candidate_label"):
                        label = value.get(key)
                        if isinstance(label, str) and label:
                            yield label


def _validation_status(errors: List[str], warnings: List[str]) -> str:
    if errors:
        return "failed"
    if warnings:
        return "warning"
    return "passed"
