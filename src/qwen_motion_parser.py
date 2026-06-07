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


CONTRACT_VERSION = "teto_qwen_motion_parser.v1"

STATUS_PASS = "PASS"
STATUS_BLOCKED = "BLOCKED"

DEFAULT_MODEL_NAME = "qwen2.5vl:3b"
DEFAULT_TIMEOUT_S = 15.0
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
    user_text = _string(request.user_text)
    model_name = _string(request.model_name) or os.environ.get("TETO_QWEN_MODEL") or DEFAULT_MODEL_NAME
    endpoint = _string(request.endpoint) or os.environ.get("TETO_QWEN_ENDPOINT")
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
    delta_m = None
    axis = None
    direction = None
    distance_m = None
    confidence = None

    if _contains_forbidden_text(user_text) or _contains_forbidden_text(raw_output):
        blocking_reasons.append(E_UNSUPPORTED_OR_FORBIDDEN_COMMAND)

    if payload is not None:
        blocking_reasons.extend(_forbidden_payload_reasons(payload))
        intent = _string(payload.get("intent"))
        axis = _string(payload.get("axis"))
        direction = _string(payload.get("direction"))
        distance_m = _optional_float(payload.get("distance_m"))
        confidence = _optional_float(payload.get("confidence"))
        if intent != INTENT_RELATIVE_CARTESIAN_MOTION:
            blocking_reasons.append(E_UNSUPPORTED_INTENT)
        if axis not in ALLOWED_AXES:
            blocking_reasons.append(E_INVALID_AXIS)
        if direction not in ALLOWED_DIRECTIONS:
            blocking_reasons.append(E_INVALID_DIRECTION)
        if confidence is None or confidence < float(request.confidence_threshold):
            blocking_reasons.append(E_LOW_CONFIDENCE)
        if distance_m is None or distance_m <= 0.0 or not math.isfinite(distance_m):
            blocking_reasons.append(E_INVALID_DISTANCE)
        elif distance_m > float(request.hard_safety_limit_m) + EPS:
            blocking_reasons.append(E_EXCESSIVE_CARTESIAN_MOTION)
        elif distance_m > float(request.max_distance_m) + EPS:
            blocking_reasons.append(E_EXCESSIVE_CARTESIAN_MOTION)

        if not blocking_reasons and axis and direction and distance_m is not None:
            delta_m = _delta_from_axis_direction(axis, direction, distance_m)
            normalized_contract = {
                "intent": INTENT_RELATIVE_CARTESIAN_MOTION,
                "frame": DEFAULT_FRAME,
                "delta_m": [round(value, 6) for value in delta_m],
                "max_distance_m": float(request.max_distance_m),
                "hard_safety_limit_m": float(request.hard_safety_limit_m),
                "must_confirm": True,
            }

    blocking_reasons = _unique(blocking_reasons)
    status = STATUS_PASS if not blocking_reasons else STATUS_BLOCKED
    return {
        "contract_version": CONTRACT_VERSION,
        "qwen_motion_parser_status": status,
        "parser_source": "qwen_llm",
        "llm_called": True,
        "model_name": model_name,
        "qwen_endpoint": endpoint,
        "llm_latency_ms": latency_ms,
        "raw_llm_output": raw_output,
        "normalized_contract": normalized_contract,
        "axis": axis,
        "direction": direction,
        "distance_m": round(float(distance_m), 6) if distance_m is not None else None,
        "confidence": confidence,
        "delta_m": [round(value, 6) for value in delta_m] if delta_m else None,
        "parser_blocking_reasons": blocking_reasons,
        "blocking_reasons": blocking_reasons,
        "warnings": _unique(warnings),
    }


def build_qwen_motion_prompt(user_text: str) -> str:
    return (
        "You are a safety-bounded parser for TETO UR5e relative Cartesian motion.\n"
        "Return STRICT JSON ONLY. No markdown, no prose, no code fences.\n"
        "Supported action: ONLY one small relative TCP translation along base_link x, y, or z.\n"
        "Allowed directions: up/down/left/right/forward/backward, or explicit +/- x/y/z axis moves.\n"
        "Allowed units: millimeters, mm, meters, m. Convert distance_m to meters.\n"
        "Reject ambiguous commands and any command involving objects, vision, hover, grasp, pick, rotate,\n"
        "joint motion, velocity, force, URScript, RTDE, dashboard, trajectories, or large motion.\n"
        "If unsupported, still return JSON with intent \"unsupported\", confidence below 0.80, and a reason.\n"
        "Required JSON schema:\n"
        "{\n"
        "  \"intent\": \"relative_cartesian_motion\",\n"
        "  \"axis\": \"x\" | \"y\" | \"z\",\n"
        "  \"direction\": \"+\" | \"-\",\n"
        "  \"distance_m\": number,\n"
        "  \"confidence\": number,\n"
        "  \"reason\": string\n"
        "}\n"
        f"User command: {user_text}"
    )


def _call_qwen(*, prompt: str, model_name: str, endpoint: str | None, timeout_s: float) -> str:
    if endpoint:
        return _call_ollama_http(prompt=prompt, model_name=model_name, endpoint=endpoint, timeout_s=timeout_s)
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


def _call_ollama_http(*, prompt: str, model_name: str, endpoint: str, timeout_s: float) -> str:
    url = endpoint.rstrip("/")
    if not url.endswith("/api/chat"):
        url = f"{url}/api/chat"
    payload = json.dumps(
        {
            "model": model_name,
            "messages": [{"role": "user", "content": prompt}],
            "stream": False,
            "options": {"temperature": 0.0},
        }
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
    message = data.get("message") if isinstance(data, dict) else None
    if isinstance(message, dict) and isinstance(message.get("content"), str):
        return message["content"]
    raise RuntimeError("Qwen endpoint response did not contain message.content")


def _strict_json_object(text: str) -> Dict[str, Any]:
    raw = text.strip() if isinstance(text, str) else ""
    if not raw:
        raise ValueError("empty Qwen response")
    payload = json.loads(raw)
    if not isinstance(payload, dict):
        raise ValueError("Qwen JSON response must be an object")
    return payload


def _delta_from_axis_direction(axis: str, direction: str, distance_m: float) -> list[float]:
    signed = float(distance_m) if direction == "+" else -float(distance_m)
    if axis == "x":
        return [signed, 0.0, 0.0]
    if axis == "y":
        return [0.0, signed, 0.0]
    return [0.0, 0.0, signed]


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
