from __future__ import annotations

import math
import re
from typing import Any


POLICY_VERSION = "teto_v3_0_11_llm_first_motion_semantics_v1"
SEMANTIC_ALIGNMENT_VERSION = "teto_v3_0_11_llm_first_motion_semantics_v1"
VECTOR_MOTION_CONTRACT_VERSION = "teto_v3_0_13_vector_relative_motion_v1"
QWEN_SEMANTIC_SCHEMA_VERSION = "teto_motion_semantics.v1"
DEFAULT_FRAME = "base_link"
DEFAULT_SMALL_STEP_M = 0.01
DEFAULT_QWEN_CONFIDENCE_THRESHOLD = 0.80
EPS = 1e-9

STATUS_PASS = "PASS"
STATUS_NEEDS_CLARIFICATION = "NEEDS_CLARIFICATION"
STATUS_UNSUPPORTED_INTENT = "UNSUPPORTED_INTENT"
STATUS_BLOCKED = "BLOCKED"

INTENT_RELATIVE_CARTESIAN_MOTION = "relative_cartesian_motion"

_SEMANTIC_DIRECTION_TO_AXIS_SIGN = {
    "up": ("z", "+"),
    "down": ("z", "-"),
    "left": ("y", "+"),
    "right": ("y", "-"),
    "forward": ("x", "+"),
    "backward": ("x", "-"),
}
_AXIS_SIGN_TO_SEMANTIC_DIRECTION = {value: key for key, value in _SEMANTIC_DIRECTION_TO_AXIS_SIGN.items()}

_NUMBER_WORDS = {
    "one": 1.0,
    "two": 2.0,
    "three": 3.0,
    "four": 4.0,
    "five": 5.0,
    "six": 6.0,
    "seven": 7.0,
    "eight": 8.0,
    "nine": 9.0,
    "ten": 10.0,
}

_DIRECTION_PATTERNS = [
    ("up", "z", "+", "explicit_direction_word", [r"\bup\b", r"\braise\b", r"\braised\b", r"\blift\b", r"\blifted\b", r"\bhigher\b", r"\bupward\b", r"\bupwards\b", r"抬高", r"向上", r"往上", r"上移"]),
    ("down", "z", "-", "explicit_direction_word", [r"\bdown\b", r"\blower\b", r"\blowered\b", r"\bdrop\b", r"\bdropped\b", r"\bdescend\b", r"\bdescending\b", r"\bdownward\b", r"\bdownwards\b", r"往下", r"向下", r"下降", r"下移"]),
    ("left", "y", "+", "explicit_direction_word", [r"\bleft\b", r"向左", r"往左", r"左移"]),
    ("right", "y", "-", "explicit_direction_word", [r"\bright\b", r"向右", r"往右", r"右移"]),
    ("forward", "x", "+", "explicit_direction_word", [r"\bforward\b", r"\bforwards\b", r"\bahead\b", r"往前", r"向前", r"前移"]),
    ("backward", "x", "-", "explicit_direction_word", [r"\bbackward\b", r"\bbackwards\b", r"\bback\b", r"往后", r"向后", r"後ろ", r"后移"]),
    ("+x", "x", "+", "explicit_direction_word", [r"(?:^|\s)\+\s*x\b", r"\bx\s*\+"]),
    ("-x", "x", "-", "explicit_direction_word", [r"(?:^|\s)-\s*x\b", r"\bx\s*-"]),
    ("+y", "y", "+", "explicit_direction_word", [r"(?:^|\s)\+\s*y\b", r"\by\s*\+"]),
    ("-y", "y", "-", "explicit_direction_word", [r"(?:^|\s)-\s*y\b", r"\by\s*-"]),
    ("+z", "z", "+", "explicit_direction_word", [r"(?:^|\s)\+\s*z\b", r"\bz\s*\+"]),
    ("-z", "z", "-", "explicit_direction_word", [r"(?:^|\s)-\s*z\b", r"\bz\s*-"]),
]

_VISION_OR_OBJECT_PATTERNS = [
    r"\bmug\b",
    r"\bcup\b",
    r"\bobject\b",
    r"\btarget\b",
    r"\babove\b",
    r"\bover\b",
    r"\bhover\b",
    r"杯",
    r"物体",
]

_MANIPULATION_PATTERNS = [
    r"\bgrab\b",
    r"\bgrasp\b",
    r"\bpick\b",
    r"\bplace\b",
    r"\bpush\b",
    r"\bpress\b",
    r"\bopen\b",
    r"\bclose\b",
    r"\bforce\b",
    r"抓",
    r"拿",
    r"夹",
]

_FORBIDDEN_CONTROL_PATTERNS = [
    r"\burscript\b",
    r"\brtde\b",
    r"\bdashboard\b",
    r"\bmovej\b",
    r"\bmovel\b",
    r"\bservoj\b",
    r"\bjoint\b",
    r"\btrajectory\b",
    r"\bvelocity\b",
]

_FUZZY_PATTERNS = [
    r"\ba little\b",
    r"\ba bit\b",
    r"\bslightly\b",
    r"\bsmall step\b",
    r"\bnudge\b",
    r"一点",
    r"一点点",
    r"稍微",
]


def normalize_motion_command(
    command: str,
    *,
    default_small_step_m: float = DEFAULT_SMALL_STEP_M,
    frame: str = DEFAULT_FRAME,
    parser_source: str = "normalizer",
    qwen_semantic: dict[str, Any] | None = None,
    qwen_confidence_threshold: float = DEFAULT_QWEN_CONFIDENCE_THRESHOLD,
) -> dict[str, Any]:
    raw_command = command if isinstance(command, str) else ""
    fallback = _normalize_rule_based(
        raw_command,
        default_small_step_m=default_small_step_m,
        frame=frame,
        parser_source=parser_source,
    )
    if not isinstance(qwen_semantic, dict):
        return {
            **fallback,
            "fallback_parse_used": True,
            "canonicalization_source": "fallback_rule" if fallback.get("parse_status") == STATUS_PASS else fallback.get("canonicalization_source"),
        }

    semantic = _normalize_qwen_semantic(
        raw_command,
        qwen_semantic,
        fallback=fallback,
        default_small_step_m=default_small_step_m,
        frame=frame,
        confidence_threshold=qwen_confidence_threshold,
    )
    if semantic["decision"] == "use_qwen":
        result = semantic["result"]
        conflict, reason = _qwen_fallback_conflict(result, fallback)
        return {
            **result,
            "qwen_fallback_conflict": conflict,
            "qwen_fallback_conflict_reason": reason,
        }

    result = {
        **fallback,
        **semantic["qwen_fields"],
        "parser_source": "rule_based" if parser_source in {"normalizer", "fallback_rule"} else parser_source,
        "qwen_semantic_parse_used": False,
        "fallback_parse_used": True,
        "canonicalization_source": "fallback_rule" if fallback.get("parse_status") == STATUS_PASS else fallback.get("canonicalization_source"),
    }
    if semantic["decision"] == "low_confidence":
        result["qwen_fallback_conflict"] = fallback.get("parse_status") != STATUS_PASS
        result["qwen_fallback_conflict_reason"] = "qwen_low_confidence" if result["qwen_fallback_conflict"] else None
    elif semantic["decision"] == "invalid_schema":
        result["qwen_fallback_conflict"] = fallback.get("parse_status") != STATUS_PASS
        result["qwen_fallback_conflict_reason"] = semantic.get("invalid_reason")
    return result


def _normalize_rule_based(
    command: str,
    *,
    default_small_step_m: float,
    frame: str,
    parser_source: str,
) -> dict[str, Any]:
    raw_command = command if isinstance(command, str) else ""
    normalized = _normalize(raw_command)
    base = _base_evidence(raw_command, normalized, frame=frame, parser_source=parser_source)
    if not normalized:
        return _blocked(base, STATUS_NEEDS_CLARIFICATION, clarification="E_EMPTY_COMMAND")
    if _matches_any(normalized, _FORBIDDEN_CONTROL_PATTERNS):
        return _blocked(base, STATUS_UNSUPPORTED_INTENT, unsupported="UNSUPPORTED_DIRECT_ROBOT_CONTROL")
    if _matches_any(normalized, _MANIPULATION_PATTERNS):
        return _blocked(base, STATUS_UNSUPPORTED_INTENT, unsupported="UNSUPPORTED_VISION_OR_MANIPULATION_INTENT")
    if re.search(r"\bfaster\b|\bspeed up\b|\bslower\b", normalized):
        return _blocked(base, STATUS_UNSUPPORTED_INTENT, unsupported="UNSUPPORTED_SPEED_INTENT")
    if re.search(r"\bcloser\b|\baway\b|\bover there\b|\bthere\b", normalized):
        return _blocked(base, STATUS_NEEDS_CLARIFICATION, clarification="E_TARGET_LOCATION_UNSPECIFIED")
    if re.search(r"\bmove\s+(?:to|toward|towards)\b", normalized) and _matches_any(normalized, _VISION_OR_OBJECT_PATTERNS):
        return _blocked(base, STATUS_UNSUPPORTED_INTENT, unsupported="NEEDS_VISION")
    if _matches_any(normalized, _VISION_OR_OBJECT_PATTERNS):
        return _blocked(base, STATUS_UNSUPPORTED_INTENT, unsupported="NEEDS_VISION")

    vector_delta = _extract_explicit_vector_delta(normalized)
    if vector_delta is not None:
        return _vector_motion_result(
            base,
            vector_delta,
            parser_source=parser_source,
            vector_source="fallback_rule",
            confidence=0.90,
        )

    direction = _extract_direction(normalized)
    distance = _extract_distance(normalized)
    fuzzy = _matches_any(normalized, _FUZZY_PATTERNS)

    if direction["status"] == "conflict":
        return _blocked(base, STATUS_NEEDS_CLARIFICATION, clarification="E_CONFLICTING_DIRECTIONS")
    if direction["axis"] is None:
        return _blocked(base, STATUS_NEEDS_CLARIFICATION, clarification="E_DIRECTION_MISSING")
    if distance["status"] == "invalid":
        return _blocked(base, STATUS_NEEDS_CLARIFICATION, clarification="E_INVALID_DISTANCE")
    if distance["distance_m"] is None:
        if not fuzzy:
            return _blocked(base, STATUS_NEEDS_CLARIFICATION, clarification="E_DISTANCE_MISSING")
        distance = {
            "distance_m": round(float(default_small_step_m), 6),
            "source": "inferred_default",
            "unit": "default_small_step",
            "inferred_default_distance_m": round(float(default_small_step_m), 6),
        }

    distance_m = float(distance["distance_m"])
    if distance_m <= 0.0 or not math.isfinite(distance_m):
        return _blocked(base, STATUS_NEEDS_CLARIFICATION, clarification="E_INVALID_DISTANCE")
    delta = _delta_from_axis_direction(direction["axis"], direction["sign"], distance_m)
    confidence = 0.94 if distance["source"] == "explicit" else 0.82
    return {
        **base,
        "parse_status": STATUS_PASS,
        "intent": INTENT_RELATIVE_CARTESIAN_MOTION,
        "parser_confidence": confidence,
        "motion_parse_confidence": confidence,
        "distance_source": distance["source"],
        "direction_source": direction["source"],
        "inferred_default_distance_m": distance.get("inferred_default_distance_m"),
        "requested_distance_m": round(distance_m, 6),
        "direction_axis": direction["axis"],
        "direction_sign": direction["sign"],
        "delta_m": delta,
        **_vector_contract_fields(delta, vector_source="fallback_rule"),
        "unit": distance["unit"],
        "fallback_parse_used": True,
        "canonicalization_source": "fallback_rule",
    }


def _normalize_qwen_semantic(
    command: str,
    payload: dict[str, Any],
    *,
    fallback: dict[str, Any],
    default_small_step_m: float,
    frame: str,
    confidence_threshold: float,
) -> dict[str, Any]:
    semantic, invalid_reason = _coerce_qwen_semantic(payload)
    qwen_fields = _qwen_evidence_fields(semantic, invalid_reason=invalid_reason)
    if invalid_reason:
        return {"decision": "invalid_schema", "qwen_fields": qwen_fields, "invalid_reason": invalid_reason}

    confidence_overall = qwen_fields.get("qwen_confidence_overall")
    if confidence_overall is not None and confidence_overall < float(confidence_threshold):
        return {"decision": "low_confidence", "qwen_fields": qwen_fields}

    raw_command = command if isinstance(command, str) else ""
    normalized = _normalize(raw_command)
    base = {
        **_base_evidence(raw_command, normalized, frame=frame, parser_source="qwen_llm"),
        **qwen_fields,
        "qwen_semantic_parse_used": True,
        "fallback_parse_used": False,
    }
    intent_status = str(semantic.get("intent_status") or "").strip().lower()
    intent_type = str(semantic.get("intent_type") or "").strip().lower()
    motion = semantic.get("motion") if isinstance(semantic.get("motion"), dict) else {}
    motion_mode = str(motion.get("mode") or "single_axis").strip().lower()
    direction_semantic = str(motion.get("direction_semantic") or "").strip().lower()
    distance = motion.get("distance") if isinstance(motion.get("distance"), dict) else {}
    distance_quality = str(distance.get("quality") or "").strip().lower()

    if intent_status == "unsupported" or intent_type in {"vision_target_motion", "manipulation", "speed_control"}:
        return {
            "decision": "use_qwen",
            "result": _blocked(
                {**base, "canonicalization_source": "unsupported"},
                STATUS_UNSUPPORTED_INTENT,
                unsupported=_unsupported_reason(intent_type, semantic),
            ),
        }
    if intent_status == "needs_clarification":
        return {
            "decision": "use_qwen",
            "result": _blocked(
                {**base, "canonicalization_source": "clarification"},
                STATUS_NEEDS_CLARIFICATION,
                clarification=_clarification_reason(direction_semantic, distance_quality, semantic),
            ),
        }
    if intent_status != "ok" or intent_type != INTENT_RELATIVE_CARTESIAN_MOTION:
        return {
            "decision": "use_qwen",
            "result": _blocked(
                {**base, "canonicalization_source": "clarification"},
                STATUS_NEEDS_CLARIFICATION,
                clarification="E_QWEN_SEMANTIC_INTENT_UNCLEAR",
            ),
        }
    if motion_mode == "unsupported_compound":
        return {
            "decision": "use_qwen",
            "result": _blocked(
                {**base, "canonicalization_source": "clarification"},
                STATUS_NEEDS_CLARIFICATION,
                clarification="E_VECTOR_MOTION_NEEDS_CLARIFICATION",
            ),
        }
    if motion_mode == "vector_delta":
        vector_delta = _semantic_vector_delta(motion.get("delta"))
        if vector_delta is None:
            return {
                "decision": "use_qwen",
                "result": _blocked(
                    {**base, "canonicalization_source": "clarification"},
                    STATUS_NEEDS_CLARIFICATION,
                    clarification="E_INVALID_VECTOR_DELTA",
                ),
            }
        return {
            "decision": "use_qwen",
            "result": _vector_motion_result(
                base,
                vector_delta,
                parser_source="qwen_llm",
                vector_source="qwen_semantic",
                confidence=confidence_overall if confidence_overall is not None else 0.90,
                canonicalization_source="qwen_semantic",
            ),
        }
    if direction_semantic == "conflicting":
        return {
            "decision": "use_qwen",
            "result": _blocked(
                {**base, "canonicalization_source": "clarification"},
                STATUS_NEEDS_CLARIFICATION,
                clarification="E_CONFLICTING_DIRECTIONS",
            ),
        }
    if direction_semantic in {"", "missing", "unknown"} or direction_semantic not in _SEMANTIC_DIRECTION_TO_AXIS_SIGN:
        return {
            "decision": "use_qwen",
            "result": _blocked(
                {**base, "canonicalization_source": "clarification"},
                STATUS_NEEDS_CLARIFICATION,
                clarification="E_DIRECTION_MISSING",
            ),
        }

    distance_m, distance_source, unit = _semantic_distance_m(distance, default_small_step_m=default_small_step_m)
    if distance_m is None:
        clarification = "E_DISTANCE_MISSING" if distance_quality in {"", "missing"} else "E_INVALID_DISTANCE"
        return {
            "decision": "use_qwen",
            "result": _blocked(
                {**base, "canonicalization_source": "clarification"},
                STATUS_NEEDS_CLARIFICATION,
                clarification=clarification,
            ),
        }
    if distance_m <= 0.0 or not math.isfinite(distance_m):
        return {
            "decision": "use_qwen",
            "result": _blocked(
                {**base, "canonicalization_source": "clarification"},
                STATUS_NEEDS_CLARIFICATION,
                clarification="E_INVALID_DISTANCE",
            ),
        }

    axis, sign = _SEMANTIC_DIRECTION_TO_AXIS_SIGN[direction_semantic]
    delta = _delta_from_axis_direction(axis, sign, distance_m)
    confidence = confidence_overall if confidence_overall is not None else 0.90
    return {
        "decision": "use_qwen",
        "result": {
            **base,
            "parse_status": STATUS_PASS,
            "intent": INTENT_RELATIVE_CARTESIAN_MOTION,
            "parser_confidence": round(float(confidence), 6),
            "motion_parse_confidence": round(float(confidence), 6),
            "distance_source": distance_source,
            "direction_source": "qwen_semantic_direction",
            "inferred_default_distance_m": round(float(default_small_step_m), 6) if distance_source == "inferred_default" else None,
            "requested_distance_m": round(float(distance_m), 6),
            "direction_axis": axis,
            "direction_sign": sign,
            "delta_m": delta,
            **_vector_contract_fields(delta, vector_source="qwen_semantic"),
            "unit": unit,
            "canonicalization_source": "qwen_semantic",
        },
    }


def _base_evidence(raw_command: str, normalized: str, *, frame: str, parser_source: str) -> dict[str, Any]:
    return {
        "natural_language_coverage_version": POLICY_VERSION,
        "motion_language_policy_version": POLICY_VERSION,
        "semantic_alignment_version": SEMANTIC_ALIGNMENT_VERSION,
        "vector_motion_contract_version": VECTOR_MOTION_CONTRACT_VERSION,
        "raw_command": raw_command.strip(),
        "normalized_command": normalized,
        "parser_source": parser_source,
        "parser_confidence": None,
        "motion_parse_confidence": None,
        "parse_status": STATUS_BLOCKED,
        "clarification_required": False,
        "clarification_reason": None,
        "unsupported_intent_reason": None,
        "distance_source": "missing",
        "direction_source": "missing",
        "inferred_default_distance_m": None,
        "requested_distance_m": None,
        "direction_axis": None,
        "direction_sign": None,
        "motion_frame": frame,
        "intent": None,
        "delta_m": None,
        "vector_motion_supported": True,
        "motion_contract_type": None,
        "vector_delta_m": None,
        "requested_distance_norm_m": None,
        "vector_components_m": None,
        "vector_component_count_nonzero": 0,
        "vector_motion_frame": frame,
        "legacy_axis_compatible": False,
        "vector_source": None,
        "one_shot_vector_motion_allowed": False,
        "unit": None,
        "requires_confirmation": True,
        "safety_gate_still_required": True,
        "execution_permission_decided_by_parser": False,
        "qwen_semantic_schema_version": None,
        "qwen_intent_status": None,
        "qwen_intent_type": None,
        "qwen_direction_semantic": None,
        "qwen_distance_quality": None,
        "qwen_distance_m": None,
        "qwen_language": None,
        "qwen_confidence_intent": None,
        "qwen_confidence_direction": None,
        "qwen_confidence_distance": None,
        "qwen_confidence_overall": None,
        "qwen_semantic_parse_used": False,
        "fallback_parse_used": False,
        "qwen_fallback_conflict": False,
        "qwen_fallback_conflict_reason": None,
        "canonicalization_source": None,
    }


def _blocked(base: dict[str, Any], status: str, *, clarification: str | None = None, unsupported: str | None = None) -> dict[str, Any]:
    return {
        **base,
        "parse_status": status,
        "clarification_required": status == STATUS_NEEDS_CLARIFICATION,
        "clarification_reason": clarification,
        "unsupported_intent_reason": unsupported,
        "parser_confidence": 0.0,
        "motion_parse_confidence": 0.0,
    }


def _coerce_qwen_semantic(payload: dict[str, Any]) -> tuple[dict[str, Any], str | None]:
    if not isinstance(payload, dict):
        return {}, "E_QWEN_SCHEMA_NOT_OBJECT"
    if payload.get("schema_version") == QWEN_SEMANTIC_SCHEMA_VERSION or "intent_status" in payload or "motion" in payload:
        semantic = dict(payload)
        if not isinstance(semantic.get("motion"), dict):
            semantic["motion"] = {}
        if not isinstance(semantic.get("confidence"), dict):
            confidence = _optional_float(semantic.get("confidence"))
            semantic["confidence"] = {"overall": confidence} if confidence is not None else {}
        return semantic, None
    if "intent" in payload:
        return _legacy_payload_to_semantic(payload), None
    return {}, "E_QWEN_SCHEMA_UNRECOGNIZED"


def _legacy_payload_to_semantic(payload: dict[str, Any]) -> dict[str, Any]:
    intent = str(payload.get("intent") or "").strip().lower()
    axis = str(payload.get("axis") or "").strip().lower()
    direction = str(payload.get("direction") or "").strip()
    distance_m = _optional_float(payload.get("distance_m"))
    confidence = _optional_float(payload.get("confidence"))
    if intent == INTENT_RELATIVE_CARTESIAN_MOTION:
        direction_semantic = _AXIS_SIGN_TO_SEMANTIC_DIRECTION.get((axis, direction), "unknown")
        intent_status = "ok"
        intent_type = INTENT_RELATIVE_CARTESIAN_MOTION
        distance_quality = "explicit" if distance_m is not None else "missing"
    elif intent == "needs_clarification":
        direction_semantic = "missing"
        intent_status = "needs_clarification"
        intent_type = "unknown"
        distance_quality = "missing"
    else:
        direction_semantic = "unknown"
        intent_status = "unsupported"
        intent_type = "unknown"
        distance_quality = "missing"
    return {
        "schema_version": "legacy_axis_direction.v1",
        "intent_status": intent_status,
        "intent_type": intent_type,
        "motion": {
            "reference": "tcp",
            "direction_semantic": direction_semantic,
            "distance": {
                "value": distance_m,
                "unit": "m",
                "meters": distance_m,
                "quality": distance_quality,
            },
            "fuzzy_magnitude": "unspecified",
            "frame_hint": DEFAULT_FRAME,
        },
        "clarification": {"required": intent_status == "needs_clarification", "reason": payload.get("reason")},
        "unsupported": {"reason": payload.get("reason")},
        "confidence": {
            "intent": confidence,
            "direction": confidence,
            "distance": confidence,
            "overall": confidence,
        },
        "language": "unknown",
        "notes": "legacy_qwen_axis_direction_schema",
    }


def _qwen_evidence_fields(semantic: dict[str, Any], *, invalid_reason: str | None = None) -> dict[str, Any]:
    motion = semantic.get("motion") if isinstance(semantic.get("motion"), dict) else {}
    distance = motion.get("distance") if isinstance(motion.get("distance"), dict) else {}
    confidence = semantic.get("confidence") if isinstance(semantic.get("confidence"), dict) else {}
    return {
        "qwen_semantic_schema_version": semantic.get("schema_version") if semantic else None,
        "qwen_intent_status": semantic.get("intent_status") if semantic else None,
        "qwen_intent_type": semantic.get("intent_type") if semantic else None,
        "qwen_direction_semantic": motion.get("direction_semantic"),
        "qwen_motion_mode": motion.get("mode"),
        "qwen_vector_delta_m": _semantic_vector_delta(motion.get("delta")),
        "qwen_distance_quality": distance.get("quality"),
        "qwen_distance_m": _semantic_distance_value(distance),
        "qwen_language": semantic.get("language") if semantic else None,
        "qwen_confidence_intent": _optional_float(confidence.get("intent")),
        "qwen_confidence_direction": _optional_float(confidence.get("direction")),
        "qwen_confidence_distance": _optional_float(confidence.get("distance")),
        "qwen_confidence_overall": _optional_float(confidence.get("overall")),
        "qwen_semantic_parse_used": False,
        "fallback_parse_used": False,
        "qwen_fallback_conflict": False,
        "qwen_fallback_conflict_reason": invalid_reason,
    }


def _semantic_distance_m(distance: dict[str, Any], *, default_small_step_m: float) -> tuple[float | None, str, str | None]:
    quality = str(distance.get("quality") or "").strip().lower()
    if quality == "fuzzy_small":
        return round(float(default_small_step_m), 6), "inferred_default", "default_small_step"
    if quality == "explicit":
        distance_m = _semantic_distance_value(distance)
        return (round(float(distance_m), 6), "explicit", _string(distance.get("unit")) or "m") if distance_m is not None else (None, "missing", None)
    if quality in {"missing", "ambiguous", ""}:
        return None, "missing", None
    distance_m = _semantic_distance_value(distance)
    if distance_m is not None:
        return round(float(distance_m), 6), "explicit", _string(distance.get("unit")) or "m"
    return None, "missing", None


def _semantic_distance_value(distance: dict[str, Any]) -> float | None:
    meters = _optional_float(distance.get("meters"))
    if meters is not None:
        return meters
    value = _optional_float(distance.get("value"))
    unit = str(distance.get("unit") or "").strip().lower()
    if value is None:
        return None
    if unit in {"m", "meter", "meters"}:
        return value
    if unit in {"cm", "centimeter", "centimeters", "厘米", "公分"}:
        return value / 100.0
    if unit in {"mm", "millimeter", "millimeters", "毫米"}:
        return value / 1000.0
    return None


def _clarification_reason(direction_semantic: str, distance_quality: str, semantic: dict[str, Any]) -> str:
    clarification = semantic.get("clarification") if isinstance(semantic.get("clarification"), dict) else {}
    raw_reason = _string(clarification.get("reason"))
    if raw_reason:
        return raw_reason
    if direction_semantic == "conflicting":
        return "E_CONFLICTING_DIRECTIONS"
    if direction_semantic in {"", "missing", "unknown"}:
        return "E_DIRECTION_MISSING"
    if distance_quality in {"", "missing"}:
        return "E_DISTANCE_MISSING"
    return "E_COMMAND_NEEDS_CLARIFICATION"


def _unsupported_reason(intent_type: str, semantic: dict[str, Any]) -> str:
    unsupported = semantic.get("unsupported") if isinstance(semantic.get("unsupported"), dict) else {}
    raw_reason = _string(unsupported.get("reason"))
    if raw_reason:
        return raw_reason
    if intent_type == "vision_target_motion":
        return "NEEDS_VISION"
    if intent_type == "manipulation":
        return "UNSUPPORTED_VISION_OR_MANIPULATION_INTENT"
    if intent_type == "speed_control":
        return "UNSUPPORTED_SPEED_INTENT"
    return "E_UNSUPPORTED_INTENT"


def _qwen_fallback_conflict(qwen_result: dict[str, Any], fallback: dict[str, Any]) -> tuple[bool, str | None]:
    if fallback.get("parse_status") != STATUS_PASS:
        reason = fallback.get("clarification_reason") or fallback.get("unsupported_intent_reason") or fallback.get("parse_status")
        return True, f"fallback_rejected:{reason}"
    if qwen_result.get("parse_status") != STATUS_PASS:
        return False, None
    mismatches = []
    for key in ("direction_axis", "direction_sign"):
        if qwen_result.get(key) != fallback.get(key):
            mismatches.append(key)
    if qwen_result.get("delta_m") != fallback.get("delta_m"):
        mismatches.append("delta_m")
    qwen_distance = _optional_float(qwen_result.get("requested_distance_m"))
    fallback_distance = _optional_float(fallback.get("requested_distance_m"))
    if qwen_distance is not None and fallback_distance is not None and abs(qwen_distance - fallback_distance) > EPS:
        mismatches.append("requested_distance_m")
    return (bool(mismatches), ",".join(mismatches) if mismatches else None)


def _normalize(command: str) -> str:
    lowered = command.lower().strip()
    replacements = {
        "end-effector": "end effector",
        "tcp": "tcp",
        "+": " +",
        "-": " -",
    }
    for left, right in replacements.items():
        lowered = lowered.replace(left, right)
    return re.sub(r"\s+", " ", lowered)


def _matches_any(text: str, patterns: list[str]) -> bool:
    return any(re.search(pattern, text) for pattern in patterns)


def _extract_direction(normalized: str) -> dict[str, Any]:
    matches = []
    seen = set()
    for word, axis, sign, source, patterns in _DIRECTION_PATTERNS:
        if any(re.search(pattern, normalized) for pattern in patterns):
            key = (axis, sign)
            if key not in seen:
                matches.append({"word": word, "axis": axis, "sign": sign, "source": source})
                seen.add(key)
    if not matches:
        return {"status": "missing", "axis": None, "sign": None, "source": "missing"}
    axes = {(item["axis"], item["sign"]) for item in matches}
    if len(axes) > 1:
        return {"status": "conflict", "axis": None, "sign": None, "source": "ambiguous"}
    match = matches[0]
    return {"status": "pass", "axis": match["axis"], "sign": match["sign"], "source": match["source"]}


def _extract_distance(normalized: str) -> dict[str, Any]:
    number = r"(\d+(?:\.\d+)?|one|two|three|four|five|six|seven|eight|nine|ten)"
    match = re.search(
        rf"\b{number}\s*(mm|millimeter|millimeters|cm|centimeter|centimeters|m|meter|meters)\b",
        normalized,
    )
    if not match:
        match = re.search(rf"\b{number}\s*(毫米|厘米|公分|米)\b", normalized)
    if not match:
        return {"status": "missing", "distance_m": None, "source": "missing", "unit": None}
    value = _number_value(match.group(1))
    unit = match.group(2)
    if value is None or value <= 0.0 or not math.isfinite(value):
        return {"status": "invalid", "distance_m": None, "source": "ambiguous", "unit": unit}
    if unit in {"mm", "millimeter", "millimeters", "毫米"}:
        distance_m = value / 1000.0
    elif unit in {"cm", "centimeter", "centimeters", "厘米", "公分"}:
        distance_m = value / 100.0
    elif unit in {"m", "meter", "meters", "米"}:
        distance_m = value
    else:
        return {"status": "invalid", "distance_m": None, "source": "ambiguous", "unit": unit}
    return {
        "status": "pass",
        "distance_m": round(float(distance_m), 6),
        "source": "explicit",
        "unit": unit,
    }


def canonicalize_delta_motion(
    delta: dict[str, Any] | list[float] | tuple[float, ...],
    *,
    frame: str = DEFAULT_FRAME,
    vector_source: str = "delta_json",
    parser_source: str = "explicit_delta",
) -> dict[str, Any]:
    vector = _coerce_delta_vector(delta)
    base = _base_evidence("", "", frame=frame, parser_source=parser_source)
    if vector is None or _vector_norm(vector) <= EPS:
        return _blocked(base, STATUS_NEEDS_CLARIFICATION, clarification="E_INVALID_VECTOR_DELTA")
    return _vector_motion_result(
        base,
        vector,
        parser_source=parser_source,
        vector_source=vector_source,
        confidence=1.0,
        canonicalization_source=vector_source,
    )


def _vector_motion_result(
    base: dict[str, Any],
    delta: list[float],
    *,
    parser_source: str,
    vector_source: str,
    confidence: float,
    canonicalization_source: str | None = None,
) -> dict[str, Any]:
    vector = [round(float(value), 6) for value in delta]
    norm = round(_vector_norm(vector), 6)
    nonzero = [index for index, value in enumerate(vector) if abs(value) > EPS]
    axis = ("x", "y", "z")[nonzero[0]] if len(nonzero) == 1 else None
    sign = ("+" if vector[nonzero[0]] > 0.0 else "-") if len(nonzero) == 1 else None
    return {
        **base,
        "parse_status": STATUS_PASS,
        "intent": INTENT_RELATIVE_CARTESIAN_MOTION,
        "parser_source": parser_source,
        "parser_confidence": round(float(confidence), 6),
        "motion_parse_confidence": round(float(confidence), 6),
        "distance_source": "vector_components",
        "direction_source": "vector_delta",
        "requested_distance_m": norm,
        "direction_axis": axis,
        "direction_sign": sign,
        "delta_m": vector,
        **_vector_contract_fields(vector, vector_source=vector_source),
        "unit": "m",
        "fallback_parse_used": vector_source == "fallback_rule",
        "canonicalization_source": canonicalization_source or vector_source,
    }


def _vector_contract_fields(delta: list[float], *, vector_source: str) -> dict[str, Any]:
    vector = [round(float(value), 6) for value in delta]
    nonzero = sum(abs(value) > EPS for value in vector)
    norm = round(_vector_norm(vector), 6)
    return {
        "vector_motion_supported": True,
        "motion_contract_type": "single_axis_relative" if nonzero == 1 else "vector_relative",
        "vector_delta_m": {"x": vector[0], "y": vector[1], "z": vector[2]},
        "requested_distance_norm_m": norm,
        "vector_components_m": {"x": vector[0], "y": vector[1], "z": vector[2]},
        "vector_component_count_nonzero": nonzero,
        "vector_motion_frame": DEFAULT_FRAME,
        "legacy_axis_compatible": nonzero == 1,
        "vector_source": vector_source,
        "one_shot_vector_motion_allowed": False if nonzero > 1 and norm > 0.05 + EPS else None,
    }


def _extract_explicit_vector_delta(normalized: str) -> list[float] | None:
    axis_matches = re.findall(
        r"\b([xyz])\s*[:=]?\s*([-+]?\d+(?:\.\d+)?)\s*(mm|cm|m)?\b",
        normalized,
    )
    if len({match[0] for match in axis_matches}) >= 2:
        components = {"x": 0.0, "y": 0.0, "z": 0.0}
        for axis, raw_value, unit in axis_matches:
            value = float(raw_value)
            components[axis] = value / 1000.0 if unit == "mm" else value / 100.0 if unit == "cm" else value
        return [components["x"], components["y"], components["z"]]

    direction_map = {
        "forward": ("x", 1.0),
        "backward": ("x", -1.0),
        "left": ("y", 1.0),
        "right": ("y", -1.0),
        "up": ("z", 1.0),
        "down": ("z", -1.0),
    }
    number = r"(\d+(?:\.\d+)?|one|two|three|four|five|six|seven|eight|nine|ten)"
    matches = re.findall(
        rf"\b(forward|backward|left|right|up|down)\b[^,;]*?\b{number}\s*"
        r"(mm|millimeter|millimeters|cm|centimeter|centimeters|m|meter|meters)\b",
        normalized,
    )
    if len(matches) < 2:
        return None
    components = {"x": 0.0, "y": 0.0, "z": 0.0}
    used_axes = set()
    for direction, raw_value, unit in matches:
        axis, sign = direction_map[direction]
        if axis in used_axes:
            return None
        value = _number_value(raw_value)
        if value is None:
            return None
        meters = value / 1000.0 if unit.startswith("mm") or unit.startswith("millimeter") else value / 100.0 if unit.startswith("cm") or unit.startswith("centimeter") else value
        components[axis] = sign * meters
        used_axes.add(axis)
    return [components["x"], components["y"], components["z"]]


def _semantic_vector_delta(value: Any) -> list[float] | None:
    if not isinstance(value, dict):
        return None
    components = []
    for axis in ("x", "y", "z"):
        component = value.get(axis)
        if isinstance(component, dict):
            meters = _semantic_distance_value(component)
        else:
            meters = _optional_float(component)
        if meters is None:
            return None
        components.append(float(meters))
    return components if _vector_norm(components) > EPS else None


def _coerce_delta_vector(value: Any) -> list[float] | None:
    if isinstance(value, dict):
        raw = [value.get("x"), value.get("y"), value.get("z")]
    elif isinstance(value, (list, tuple)) and len(value) == 3:
        raw = list(value)
    else:
        return None
    try:
        vector = [float(item) for item in raw]
    except (TypeError, ValueError):
        return None
    return vector if all(math.isfinite(item) for item in vector) else None


def _vector_norm(delta: list[float]) -> float:
    return math.sqrt(sum(float(value) ** 2 for value in delta))


def _number_value(raw: str) -> float | None:
    if raw in _NUMBER_WORDS:
        return _NUMBER_WORDS[raw]
    try:
        return float(raw)
    except (TypeError, ValueError):
        return None


def _delta_from_axis_direction(axis: str, sign: str, distance_m: float) -> list[float]:
    signed = float(distance_m) if sign == "+" else -float(distance_m)
    if axis == "x":
        return [round(signed, 6), 0.0, 0.0]
    if axis == "y":
        return [0.0, round(signed, 6), 0.0]
    return [0.0, 0.0, round(signed, 6)]


def _optional_float(value: Any) -> float | None:
    if isinstance(value, bool):
        return None
    if isinstance(value, (int, float)):
        number = float(value)
        return number if math.isfinite(number) else None
    if isinstance(value, str) and value.strip():
        try:
            number = float(value)
        except ValueError:
            return None
        return number if math.isfinite(number) else None
    return None


def _string(value: Any) -> str | None:
    return value.strip() if isinstance(value, str) and value.strip() else None
