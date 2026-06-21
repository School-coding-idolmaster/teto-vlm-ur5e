from __future__ import annotations

import json
import math
import os
import re
import time
import urllib.error
import urllib.request
from dataclasses import dataclass
from typing import Any, Callable, Dict

from src.motion_command_normalizer import (
    DEFAULT_SMALL_STEP_M,
    QWEN_SEMANTIC_SCHEMA_VERSION,
    STATUS_NEEDS_CLARIFICATION,
    STATUS_PASS as NORMALIZER_STATUS_PASS,
    STATUS_UNSUPPORTED_INTENT,
    normalize_motion_command,
)


CONTRACT_VERSION = "teto_qwen_motion_parser.v2"

STATUS_PASS = "PASS"
STATUS_BLOCKED = "BLOCKED"

DEFAULT_MODEL_NAME = "Qwen/Qwen2.5-VL-3B-Instruct"
DEFAULT_ENDPOINT = "http://127.0.0.1:18080/api/generate"
DEFAULT_TIMEOUT_S = 60.0
DEFAULT_CONFIDENCE_THRESHOLD = 0.80
DEFAULT_FRAME = "base_link"
EPS = 1e-9

INTENT_RELATIVE_CARTESIAN_MOTION = "relative_cartesian_motion"
ALLOWED_AXES = {"x", "y", "z"}
ALLOWED_DIRECTIONS = {"+", "-"}

E_LLM_CALL_FAILED = "E_LLM_CALL_FAILED"
E_INVALID_JSON = "E_INVALID_JSON"
E_LOW_CONFIDENCE = "E_LOW_CONFIDENCE"
E_UNSUPPORTED_INTENT = "E_UNSUPPORTED_INTENT"
E_INVALID_AXIS = "E_INVALID_AXIS"
E_INVALID_DIRECTION = "E_INVALID_DIRECTION"
E_INVALID_DISTANCE = "E_INVALID_DISTANCE"
E_EXCESSIVE_CARTESIAN_MOTION = "E_EXCESSIVE_CARTESIAN_MOTION"
E_UNSUPPORTED_OR_FORBIDDEN_COMMAND = "E_UNSUPPORTED_OR_FORBIDDEN_COMMAND"

FORBIDDEN_PATTERNS = [
    r"\bhover\b",
    r"\bmug\b",
    r"\bobject\b",
    r"\bvision\b",
    r"\bcamera\b",
    r"\bgrasp\b",
    r"\bpick\b",
    r"\brotate\b",
    r"\burscript\b",
    r"\bscript\b",
    r"\brtde\b",
    r"\bdashboard\b",
    r"\bmovej\b",
    r"\bmovel\b",
    r"\bservoj\b",
    r"\bjoint\b",
    r"\bvelocity\b",
    r"\bforce\b",
    r"\btrajectory\b",
]

FORBIDDEN_FIELDS = {
    "urscript",
    "rtde",
    "dashboard_command",
    "joint",
    "joint_command",
    "joint_target",
    "joint_targets",
    "velocity",
    "force",
    "trajectory",
    "robot_command",
}


@dataclass(frozen=True)
class QwenMotionParserRequest:
    user_text: str
    max_distance_m: float
    hard_safety_limit_m: float
    model_name: str | None = None
    endpoint: str | None = None
    timeout_s: float | None = None
    confidence_threshold: float = DEFAULT_CONFIDENCE_THRESHOLD
    llm_callable: Callable[[str], str] | None = None


def evaluate_qwen_motion_parser(request: QwenMotionParserRequest) -> Dict[str, Any]:
    user_text = _string(request.user_text) or ""
    model_name = _string(request.model_name) or os.environ.get("TETO_QWEN_MODEL") or DEFAULT_MODEL_NAME
    endpoint = _string(request.endpoint) or os.environ.get("TETO_QWEN_ENDPOINT") or DEFAULT_ENDPOINT
    timeout_s = _optional_float(request.timeout_s) or _optional_float(os.environ.get("TETO_QWEN_TIMEOUT_S")) or DEFAULT_TIMEOUT_S
    prompt = build_qwen_motion_prompt(user_text)
    start = time.monotonic()
    raw_output = None
    blocking_reasons: list[str] = []
    payload: Dict[str, Any] | None = None

    try:
        raw_output = request.llm_callable(prompt) if request.llm_callable else _call_qwen(
            prompt=prompt,
            model_name=model_name,
            endpoint=endpoint,
            timeout_s=timeout_s,
        )
    except Exception as exc:
        blocking_reasons.append(E_LLM_CALL_FAILED)
        raw_output = None
        warnings = [f"llm_error={exc}"]
    else:
        warnings = []
        try:
            payload = _strict_json_object(raw_output)
        except Exception as exc:
            blocking_reasons.append(E_INVALID_JSON)
            warnings.append(f"json_error={exc}")

    latency_ms = round((time.monotonic() - start) * 1000.0, 3)
    normalized_contract = None
    normalized_language: dict[str, Any] | None = None
    delta_m = None
    axis = None
    direction = None
    distance_m = None
    confidence = None

    if _contains_forbidden_text(user_text) or _contains_forbidden_text(raw_output):
        blocking_reasons.append(E_UNSUPPORTED_OR_FORBIDDEN_COMMAND)

    if payload is not None:
        blocking_reasons.extend(_forbidden_payload_reasons(payload))
        normalized_language = normalize_motion_command(
            user_text,
            qwen_semantic=payload,
            default_small_step_m=DEFAULT_SMALL_STEP_M,
            parser_source="qwen_llm",
            qwen_confidence_threshold=float(request.confidence_threshold),
        )
        warnings.extend(normalized_language.get("canonicalization_warnings") or [])
        legacy_reasons = _legacy_payload_blocking_reasons(payload, confidence_threshold=float(request.confidence_threshold))
        blocking_reasons.extend(legacy_reasons)

        if not normalized_language.get("qwen_semantic_parse_used"):
            if normalized_language.get("qwen_confidence_overall") is not None:
                blocking_reasons.append(E_LOW_CONFIDENCE)
            else:
                blocking_reasons.append(E_UNSUPPORTED_INTENT)
        elif normalized_language.get("parse_status") == STATUS_UNSUPPORTED_INTENT:
            blocking_reasons.append(E_UNSUPPORTED_INTENT)
        elif normalized_language.get("parse_status") == STATUS_NEEDS_CLARIFICATION:
            reason = str(normalized_language.get("clarification_reason") or "")
            if "DISTANCE" in reason:
                blocking_reasons.append(E_INVALID_DISTANCE)
            elif "DIRECTION" in reason or "CONFLICT" in reason:
                blocking_reasons.append(E_INVALID_DIRECTION)
            else:
                blocking_reasons.append(E_UNSUPPORTED_INTENT)

        axis = _string(normalized_language.get("direction_axis"))
        direction = _string(normalized_language.get("direction_sign"))
        distance_m = _optional_float(normalized_language.get("requested_distance_m"))
        confidence = _optional_float(normalized_language.get("motion_parse_confidence"))
        if distance_m is not None and distance_m > float(request.hard_safety_limit_m) + EPS:
            blocking_reasons.append(E_EXCESSIVE_CARTESIAN_MOTION)
        elif distance_m is not None and distance_m > float(request.max_distance_m) + EPS:
            blocking_reasons.append(E_EXCESSIVE_CARTESIAN_MOTION)

        if not blocking_reasons and normalized_language.get("parse_status") == NORMALIZER_STATUS_PASS and distance_m is not None:
            delta_m = normalized_language.get("delta_m")
            if not _vector3(delta_m) and axis and direction:
                delta_m = _delta_from_axis_direction(axis, direction, distance_m)
            if _vector3(delta_m):
                normalized_contract = {
                    "intent": INTENT_RELATIVE_CARTESIAN_MOTION,
                    "frame": DEFAULT_FRAME,
                    "delta_m": [round(float(value), 6) for value in delta_m],
                    "direction_axis": normalized_language.get("direction_axis"),
                    "direction_sign": normalized_language.get("direction_sign"),
                    "distance_m": distance_m,
                    "requested_distance_m": normalized_language.get("requested_distance_m"),
                    "motion_contract_type": normalized_language.get("motion_contract_type"),
                    "vector_delta_m": normalized_language.get("vector_delta_m"),
                    "vector_components_m": normalized_language.get("vector_components_m"),
                    "requested_distance_norm_m": normalized_language.get("requested_distance_norm_m"),
                    "legacy_axis_compatible": normalized_language.get("legacy_axis_compatible"),
                    "max_distance_m": float(request.max_distance_m),
                    "hard_safety_limit_m": float(request.hard_safety_limit_m),
                    "must_confirm": True,
                }

    blocking_reasons = _unique(blocking_reasons)
    status = STATUS_PASS if not blocking_reasons else STATUS_BLOCKED
    result = {
        "contract_version": CONTRACT_VERSION,
        "qwen_motion_parser_status": status,
        "parser_source": "qwen_llm",
        "llm_called": True,
        "model_name": model_name,
        "qwen_endpoint": endpoint,
        "llm_latency_ms": latency_ms,
        "raw_llm_output": raw_output,
        "qwen_payload": payload,
        "normalized_contract": normalized_contract,
        "axis": axis,
        "direction": direction,
        "distance_m": round(float(distance_m), 6) if distance_m is not None else None,
        "confidence": confidence,
        "delta_m": [round(float(value), 6) for value in delta_m] if delta_m else None,
        "parser_blocking_reasons": blocking_reasons,
        "blocking_reasons": blocking_reasons,
        "warnings": _unique(warnings),
    }
    if isinstance(normalized_language, dict):
        result.update(_language_result_fields(normalized_language))
    return result


def build_qwen_motion_prompt(user_text: str) -> str:
    return (
        "You are a semantic motion-command parser for TETO, a UR5e safety wrapper.\n"
        "Return STRICT JSON ONLY. No markdown, no prose, no code fences.\n"
        "You perform semantic parsing only; do not approve execution or decide robot permission.\n"
        f"Use this schema_version exactly: {QWEN_SEMANTIC_SCHEMA_VERSION}.\n"
        "Valid output schema:\n"
        "{\n"
        "  \"schema_version\": \"teto_motion_semantics.v1\",\n"
        "  \"intent_status\": \"ok | needs_clarification | unsupported\",\n"
        "  \"intent_type\": \"relative_cartesian_motion | vision_target_motion | manipulation | speed_control | unknown\",\n"
        "  \"motion\": {\n"
        "    \"reference\": \"tcp | tool | end_effector | robot_hand | arm_tip | unspecified\",\n"
        "    \"mode\": \"single_axis | vector_delta | unsupported_compound\",\n"
        "    \"direction_semantic\": \"up | down | left | right | forward | backward | conflicting | missing | unknown\",\n"
        "    \"delta\": {\n"
        "      \"x\": {\"value\": 0, \"unit\": \"m | cm | mm\", \"meters\": 0.0, \"quality\": \"explicit\"},\n"
        "      \"y\": {\"value\": 0, \"unit\": \"m | cm | mm\", \"meters\": 0.0, \"quality\": \"explicit\"},\n"
        "      \"z\": {\"value\": 0, \"unit\": \"m | cm | mm\", \"meters\": 0.0, \"quality\": \"explicit\"}\n"
        "    },\n"
        "    \"distance\": {\"value\": number|null, \"unit\": \"mm|cm|m|unspecified\", \"meters\": number|null, \"quality\": \"explicit | fuzzy_small | missing | ambiguous\"},\n"
        "    \"fuzzy_magnitude\": \"tiny | small | medium | large | unspecified\",\n"
        "    \"frame_hint\": \"base_link | camera | user_relative | unspecified\"\n"
        "  },\n"
        "  \"clarification\": {\"required\": boolean, \"reason\": string},\n"
        "  \"unsupported\": {\"reason\": string},\n"
        "  \"confidence\": {\"intent\": number, \"direction\": number, \"distance\": number, \"overall\": number},\n"
        "  \"language\": \"en | zh | mixed | unknown\",\n"
        "  \"notes\": string\n"
        "}\n"
        "Instructions:\n"
        "- Relative motion of TCP/tool/end-effector/robot hand/arm tip is relative_cartesian_motion.\n"
        "- If direction is clear and distance is fuzzy small, set distance.quality=fuzzy_small, not needs_clarification.\n"
        "- If distance is explicit, convert meters accurately.\n"
        "- If direction is missing, return needs_clarification with direction_semantic=missing; never infer a direction from distance alone.\n"
        "- If directions conflict, return needs_clarification with direction_semantic=conflicting.\n"
        "- If a command asks for compatible Cartesian components (for example forward 30 cm and left 10 cm), use mode=vector_delta and populate each explicit axis component.\n"
        "- If a command asks for opposing directions on the same axis or gives ambiguous component distances, return needs_clarification with direction_semantic=conflicting unless a clear correction cancels the first direction.\n"
        "- Object/location targets such as mug/cup/bottle/object/there are vision_target_motion or needs_clarification, not relative Cartesian motion.\n"
        "- Grasp/pick/push/touch/manipulation commands are manipulation unsupported for this stage.\n"
        "- Speed-only changes are speed_control unsupported for this stage.\n"
        "- Do not invent a target, direction, distance, or execution permission.\n"
        "Examples:\n"
        "drop the tool a tiny bit => ok, relative_cartesian_motion, direction_semantic=down, distance.quality=fuzzy_small.\n"
        "把末端降低 2 厘米 => ok, relative_cartesian_motion, direction_semantic=down, distance.meters=0.02, quality=explicit.\n"
        "末端往下移动一点 => ok, relative_cartesian_motion, direction_semantic=down, distance.quality=fuzzy_small.\n"
        "move to the mug => unsupported, vision_target_motion.\n"
        "grab the cup => unsupported, manipulation.\n"
        "move 5 cm => needs_clarification, direction_semantic=missing.\n"
        "移动 5 厘米 => needs_clarification, direction_semantic=missing.\n"
        "move up and down 5 cm => needs_clarification, direction_semantic=conflicting.\n"
        "go up 5 cm and right 2 cm => ok, relative_cartesian_motion, mode=vector_delta, z.meters=0.05, y.meters=-0.02.\n"
        "先上再下 5 厘米 => needs_clarification, direction_semantic=conflicting.\n"
        "move forward, no, actually move backward 5 cm => ok, relative_cartesian_motion, direction_semantic=backward, distance.meters=0.05.\n"
        f"User command: {user_text}"
    )


def _call_qwen(*, prompt: str, model_name: str, endpoint: str | None, timeout_s: float) -> str:
    if endpoint:
        return _call_http_endpoint(prompt=prompt, model_name=model_name, endpoint=endpoint, timeout_s=timeout_s)
    try:
        from ollama import chat
    except ImportError as exc:
        raise RuntimeError("ollama package is required when TETO_QWEN_ENDPOINT is not configured") from exc
    response = chat(model=model_name, messages=[{"role": "user", "content": prompt}])
    message = getattr(response, "message", None)
    content = getattr(message, "content", None)
    if isinstance(content, str):
        return content
    if isinstance(response, dict):
        message = response.get("message")
        if isinstance(message, dict) and isinstance(message.get("content"), str):
            return message["content"]
    raise RuntimeError("Qwen response did not contain message.content")


def _call_http_endpoint(*, prompt: str, model_name: str, endpoint: str, timeout_s: float) -> str:
    url = endpoint.rstrip("/")
    generate_style = url.endswith("/api/generate")
    if not (url.endswith("/api/generate") or url.endswith("/api/chat")):
        url = f"{url}/api/generate"
        generate_style = True
    body = {
        "model": model_name,
        "stream": False,
        "options": {"temperature": 0.0, "num_predict": 512},
    }
    if generate_style:
        body["prompt"] = prompt
    else:
        body["messages"] = [{"role": "user", "content": prompt}]
    payload = json.dumps(
        body
    ).encode("utf-8")
    request = urllib.request.Request(
        url,
        data=payload,
        headers={"Content-Type": "application/json"},
        method="POST",
    )
    try:
        with urllib.request.urlopen(request, timeout=float(timeout_s)) as response:
            data = json.loads(response.read().decode("utf-8"))
    except urllib.error.URLError as exc:
        raise RuntimeError(f"Qwen endpoint request failed: {exc}") from exc
    if isinstance(data, dict) and isinstance(data.get("response"), str):
        return data["response"]
    message = data.get("message") if isinstance(data, dict) else None
    if isinstance(message, dict) and isinstance(message.get("content"), str):
        return message["content"]
    raise RuntimeError("Qwen endpoint response did not contain message.content")


def _strict_json_object(text: str) -> Dict[str, Any]:
    raw = text.strip() if isinstance(text, str) else ""
    if not raw:
        raise ValueError("empty Qwen response")
    payload = json.loads(_extract_json_object_text(raw))
    if not isinstance(payload, dict):
        raise ValueError("Qwen JSON response must be an object")
    return payload


def _extract_json_object_text(raw: str) -> str:
    try:
        payload = json.loads(raw)
    except json.JSONDecodeError:
        pass
    else:
        if not isinstance(payload, dict):
            raise ValueError("Qwen JSON response must be an object")
        return raw

    fenced_payloads = _fenced_json_payloads(raw)
    if fenced_payloads:
        if len(fenced_payloads) > 1:
            raise ValueError("ambiguous Qwen JSON response: multiple fenced JSON blocks")
        return _single_json_object_text(fenced_payloads[0])

    return _single_json_object_text(raw)


def _fenced_json_payloads(raw: str) -> list[str]:
    pattern = re.compile(r"```[ \t]*(?:json)?[ \t]*\r?\n(?P<body>.*?)```", re.IGNORECASE | re.DOTALL)
    return [match.group("body").strip() for match in pattern.finditer(raw)]


def _single_json_object_text(raw: str) -> str:
    candidates = _balanced_json_object_candidates(raw)
    if not candidates:
        raise ValueError("Qwen response did not contain a JSON object")
    if len(candidates) > 1:
        raise ValueError("ambiguous Qwen JSON response: multiple JSON objects")
    candidate = candidates[0]
    try:
        payload = json.loads(candidate)
    except json.JSONDecodeError as exc:
        raise ValueError(f"Qwen JSON parse failed: {exc}") from exc
    if not isinstance(payload, dict):
        raise ValueError("Qwen JSON response must be an object")
    return candidate


def _balanced_json_object_candidates(raw: str) -> list[str]:
    candidates: list[str] = []
    start: int | None = None
    depth = 0
    in_string = False
    escaped = False

    for index, char in enumerate(raw):
        if in_string:
            if escaped:
                escaped = False
            elif char == "\\":
                escaped = True
            elif char == '"':
                in_string = False
            continue

        if char == '"':
            in_string = True
            continue
        if char == "{":
            if depth == 0:
                start = index
            depth += 1
            continue
        if char == "}" and depth > 0:
            depth -= 1
            if depth == 0 and start is not None:
                candidates.append(raw[start : index + 1])
                start = None

    return candidates


def _delta_from_axis_direction(axis: str, direction: str, distance_m: float) -> list[float]:
    signed = float(distance_m) if direction == "+" else -float(distance_m)
    if axis == "x":
        return [signed, 0.0, 0.0]
    if axis == "y":
        return [0.0, signed, 0.0]
    return [0.0, 0.0, signed]


def _legacy_payload_blocking_reasons(payload: Dict[str, Any], *, confidence_threshold: float) -> list[str]:
    if payload.get("schema_version") == QWEN_SEMANTIC_SCHEMA_VERSION or "intent_status" in payload or "motion" in payload:
        return []
    reasons = []
    intent = _string(payload.get("intent"))
    axis = _string(payload.get("axis"))
    direction = _string(payload.get("direction"))
    distance_m = _optional_float(payload.get("distance_m"))
    confidence = _optional_float(payload.get("confidence"))
    if intent != INTENT_RELATIVE_CARTESIAN_MOTION:
        reasons.append(E_UNSUPPORTED_INTENT)
    if axis not in ALLOWED_AXES:
        reasons.append(E_INVALID_AXIS)
    if direction not in ALLOWED_DIRECTIONS:
        reasons.append(E_INVALID_DIRECTION)
    if confidence is None or confidence < float(confidence_threshold):
        reasons.append(E_LOW_CONFIDENCE)
    if distance_m is None or distance_m <= 0.0 or not math.isfinite(distance_m):
        reasons.append(E_INVALID_DISTANCE)
    return reasons


def _language_result_fields(language: dict[str, Any]) -> dict[str, Any]:
    keys = [
        "natural_language_coverage_version",
        "motion_language_policy_version",
        "semantic_alignment_version",
        "raw_command",
        "normalized_command",
        "parse_status",
        "clarification_required",
        "clarification_reason",
        "unsupported_intent_reason",
        "distance_source",
        "direction_source",
        "inferred_default_distance_m",
        "requested_distance_m",
        "requested_distance_norm_m",
        "direction_axis",
        "direction_sign",
        "delta_m",
        "vector_motion_supported",
        "motion_contract_type",
        "vector_delta_m",
        "vector_components_m",
        "vector_component_count_nonzero",
        "vector_motion_frame",
        "legacy_axis_compatible",
        "vector_source",
        "one_shot_vector_motion_allowed",
        "motion_frame",
        "parser_confidence",
        "motion_parse_confidence",
        "requires_confirmation",
        "safety_gate_still_required",
        "execution_permission_decided_by_parser",
        "qwen_semantic_schema_version",
        "qwen_intent_status",
        "qwen_intent_type",
        "qwen_direction_semantic",
        "qwen_motion_mode",
        "qwen_vector_delta_m",
        "qwen_distance_quality",
        "qwen_distance_m",
        "qwen_language",
        "qwen_confidence_intent",
        "qwen_confidence_direction",
        "qwen_confidence_distance",
        "qwen_confidence_overall",
        "qwen_semantic_parse_used",
        "fallback_parse_used",
        "qwen_fallback_conflict",
        "qwen_fallback_conflict_reason",
        "canonicalization_source",
        "canonicalization_warnings",
    ]
    return {key: language.get(key) for key in keys}


def _vector3(value: Any) -> bool:
    if not isinstance(value, (list, tuple)) or len(value) != 3:
        return False
    return all(_optional_float(item) is not None for item in value)



def _forbidden_payload_reasons(payload: Dict[str, Any]) -> list[str]:
    found = []
    for key, value in payload.items():
        key_text = _string(key)
        if key_text in FORBIDDEN_FIELDS:
            found.append(key_text)
        if isinstance(value, str) and _contains_forbidden_text(value):
            found.append(key_text)
    return [E_UNSUPPORTED_OR_FORBIDDEN_COMMAND] if found else []


def _contains_forbidden_text(value: Any) -> bool:
    if not isinstance(value, str) or not value:
        return False
    lowered = value.lower()
    return any(re.search(pattern, lowered) for pattern in FORBIDDEN_PATTERNS)


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


def _unique(values: list[str]) -> list[str]:
    result = []
    for value in values:
        if value and value not in result:
            result.append(value)
    return result
