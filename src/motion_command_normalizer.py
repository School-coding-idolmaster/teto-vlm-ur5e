from __future__ import annotations

import math
import re
from typing import Any


POLICY_VERSION = "teto_v3_0_10_nl_motion_coverage_v1"
DEFAULT_FRAME = "base_link"
DEFAULT_SMALL_STEP_M = 0.01
EPS = 1e-9

STATUS_PASS = "PASS"
STATUS_NEEDS_CLARIFICATION = "NEEDS_CLARIFICATION"
STATUS_UNSUPPORTED_INTENT = "UNSUPPORTED_INTENT"
STATUS_BLOCKED = "BLOCKED"

INTENT_RELATIVE_CARTESIAN_MOTION = "relative_cartesian_motion"

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
) -> dict[str, Any]:
    raw_command = command if isinstance(command, str) else ""
    normalized = _normalize(raw_command)
    base = {
        "natural_language_coverage_version": POLICY_VERSION,
        "motion_language_policy_version": POLICY_VERSION,
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
        "unit": None,
        "requires_confirmation": True,
        "safety_gate_still_required": True,
        "execution_permission_decided_by_parser": False,
    }
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
        "unit": distance["unit"],
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
