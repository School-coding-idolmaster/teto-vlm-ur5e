import json
from copy import deepcopy
from typing import Any, Dict, List, Tuple


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
    "target": {
        "label": "unknown",
        "approx_position": "unknown",
        "visibility": "unknown",
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


def normalize_robot_task_json(data: Dict[str, Any]) -> Dict[str, Any]:
    if not isinstance(data, dict):
        return {
            "parsed_json": deepcopy(DEFAULT_ROBOT_TASK_JSON),
            "normalized_json": deepcopy(DEFAULT_ROBOT_TASK_JSON),
            "parse_status": "failed",
            "validation_status": "failed",
            "validation_errors": ["top-level JSON value must be an object"],
            "validation_warnings": [],
        }

    validation_errors = validate_robot_task_json(data)
    safety_errors, validation_warnings = validate_robot_task_safety(data)
    validation_errors.extend(safety_errors)
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

    _apply_safety_overrides(normalized, data)

    return {
        "normalized_json": normalized,
        "parse_status": "success",
        "validation_status": _validation_status(validation_errors, validation_warnings),
        "validation_errors": validation_errors,
        "validation_warnings": validation_warnings,
    }


def parse_robot_task_response(raw_response: str) -> Dict[str, Any]:
    extracted = extract_json_from_response(raw_response)
    if extracted["parse_status"] == "failed":
        normalized = deepcopy(DEFAULT_ROBOT_TASK_JSON)
        normalized["error"]["code"] = "E_PARSE"
        normalized["error"]["message"] = "; ".join(extracted["validation_errors"])
        return {
            "raw_response": raw_response,
            "parsed_json": None,
            "normalized_json": normalized,
            "parse_status": "failed",
            "validation_status": "failed",
            "validation_errors": extracted["validation_errors"],
            "validation_warnings": [],
        }

    normalized = normalize_robot_task_json(extracted["data"])
    return {
        "raw_response": raw_response,
        "parsed_json": deepcopy(extracted["data"]),
        **normalized,
    }


def attach_robot_task_json_fields(prompt_type: str, item: Dict[str, Any]) -> Dict[str, Any]:
    if prompt_type != ROBOT_TASK_PROMPT_TYPE:
        return item

    parsed = parse_robot_task_response(str(item.get("response", "")))
    item.update(parsed)
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
