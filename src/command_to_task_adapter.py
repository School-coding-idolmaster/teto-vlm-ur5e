from __future__ import annotations

import json
import os
import re
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Callable, Dict

import yaml


CONTRACT_VERSION = "teto_command_to_task.v1"
CURRENT_COMMAND_TO_TASK_VERSION = "TETO V3.1.0"

STATUS_PASS = "PASS"
STATUS_BLOCKED = "BLOCKED"
STATUS_SAFE_DISABLED = "SAFE_DISABLED"
STATUS_NOT_REQUESTED = "NOT_REQUESTED"

MODE_RULE_BASED = "rule_based"
MODE_LLM_QWEN = "qwen_llm"
MODE_OFFLINE_LLM_JSON = "offline_llm_json"
MODE_LLM_DISABLED = "llm_disabled"

SUPPORTED_MODES = {
    MODE_RULE_BASED,
    MODE_LLM_QWEN,
    MODE_OFFLINE_LLM_JSON,
    MODE_LLM_DISABLED,
}

INTENT_HOVER_TO_OBJECT = "hover_to_object"
INTENT_GO_HOME = "go_home"
INTENT_MOVE_TO_WAYPOINT = "move_to_waypoint"
INTENT_CARTESIAN_OFFSET = "cartesian_offset"
INTENT_STOP = "stop"

SUPPORTED_INTENTS = {
    INTENT_HOVER_TO_OBJECT,
    INTENT_GO_HOME,
    INTENT_MOVE_TO_WAYPOINT,
    INTENT_CARTESIAN_OFFSET,
    INTENT_STOP,
}

DEFAULT_CONFIDENCE_THRESHOLD = 0.60
DEFAULT_MODEL_NAME = "qwen2.5vl:3b"
DEFAULT_CARTESIAN_FRAME = "base_link"
DEFAULT_MAX_CARTESIAN_TRANSLATION_M = 0.20
ALLOWED_CARTESIAN_FRAMES = {"base_link"}

E_UNSUPPORTED_ADAPTER_MODE = "E_UNSUPPORTED_ADAPTER_MODE"
E_LLM_DISABLED = "E_LLM_DISABLED"
E_LLM_CALL_FAILED = "E_LLM_CALL_FAILED"
E_PARSE_FAILED = "E_PARSE_FAILED"
E_UNSUPPORTED_INTENT = "E_UNSUPPORTED_INTENT"
E_TARGET_LABEL_REQUIRED = "E_TARGET_LABEL_REQUIRED"
E_WAYPOINT_REQUIRED = "E_WAYPOINT_REQUIRED"
E_CARTESIAN_OFFSET_REQUIRED = "E_CARTESIAN_OFFSET_REQUIRED"
E_INVALID_CARTESIAN_OFFSET = "E_INVALID_CARTESIAN_OFFSET"
E_EXCESSIVE_CARTESIAN_MOTION = "E_EXCESSIVE_CARTESIAN_MOTION"
E_INVALID_FRAME = "E_INVALID_FRAME"
E_LOW_CONFIDENCE = "E_LOW_CONFIDENCE"
E_ROBOT_COMMAND_NOT_ALLOWED = "E_ROBOT_COMMAND_NOT_ALLOWED"

FORBIDDEN_ROBOT_CONTROL_FIELDS = {
    "robot_command",
    "real_robot_command",
    "real_robot_backend",
    "trajectory",
    "trajectory_plan",
    "trajectory_command",
    "tcp_pose_world",
    "tcp_pose_world_command",
    "joint_target",
    "joint_targets",
    "joint_command",
    "urscript",
    "urscript_program",
    "dashboard_command",
    "rtde_control_command",
    "moveit_plan",
    "ros2_action_goal",
    "automatic_retry_motion",
    "automatic_retry_motion_request",
    "automatic_retry_motion_command",
}


@dataclass(frozen=True)
class CommandToTaskAdapterRequest:
    requested: bool = False
    config_path: str | None = None
    user_command: str | None = None
    adapter_mode: str | None = None
    config: Dict[str, Any] | None = None
    llm_callable: Callable[[str], str] | None = None


def load_command_to_task_config(path: str | Path | None) -> Dict[str, Any]:
    if not path:
        return {}
    resolved = Path(path).expanduser()
    if not resolved.is_file():
        return {}
    with resolved.open("r", encoding="utf-8") as config_file:
        data = json.load(config_file) if resolved.suffix.lower() == ".json" else yaml.safe_load(config_file)
    if not isinstance(data, dict):
        return {}
    config = data.get("command_to_task_adapter") or data.get("command_to_task")
    return config if isinstance(config, dict) else data


def build_command_to_task_request(
    *,
    requested: bool = False,
    config_path: str | Path | None = None,
    user_command: str | None = None,
    adapter_mode: str | None = None,
) -> CommandToTaskAdapterRequest:
    config = load_command_to_task_config(config_path)
    return CommandToTaskAdapterRequest(
        requested=requested,
        config_path=str(Path(config_path).expanduser()) if config_path else None,
        user_command=user_command,
        adapter_mode=adapter_mode,
        config=config,
    )


def evaluate_command_to_task_adapter(
    request: CommandToTaskAdapterRequest | None = None,
) -> Dict[str, Any]:
    request = request or CommandToTaskAdapterRequest()
    if not request.requested:
        return _not_requested_result()

    config = request.config if isinstance(request.config, dict) else {}
    adapter_mode = request.adapter_mode or _string(config.get("adapter_mode")) or MODE_LLM_DISABLED
    user_command = request.user_command or _string(config.get("user_command"))
    normalized_command = _normalize_text(user_command)
    confidence_threshold = _optional_float(config.get("confidence_threshold")) or DEFAULT_CONFIDENCE_THRESHOLD
    max_cartesian_translation_m = (
        _optional_float(config.get("max_cartesian_translation_m")) or DEFAULT_MAX_CARTESIAN_TRANSLATION_M
    )
    allowed_cartesian_frames = _allowed_cartesian_frames(config)
    warnings = _string_list(config.get("warnings"))
    blocking_reasons: list[str] = []

    if adapter_mode == MODE_RULE_BASED:
        raw_task = _rule_based_task(user_command)
    elif adapter_mode == MODE_OFFLINE_LLM_JSON:
        raw_task = _offline_llm_task(config)
    elif adapter_mode == MODE_LLM_QWEN:
        raw_task = _qwen_llm_task(config, user_command, request.llm_callable)
    elif adapter_mode == MODE_LLM_DISABLED:
        raw_task = _base_task(user_command)
        blocking_reasons.append(E_LLM_DISABLED)
    else:
        raw_task = _base_task(user_command)
        blocking_reasons.append(E_UNSUPPORTED_ADAPTER_MODE)

    raw_error = _string(raw_task.get("error_code"))
    if raw_error and raw_error != "OK":
        blocking_reasons.append(raw_error)
    warnings.extend(_string_list(raw_task.get("warnings")))

    forbidden_fields = _unique(_forbidden_robot_control_fields(config) + _forbidden_robot_control_fields(raw_task))
    if forbidden_fields:
        blocking_reasons.append(E_ROBOT_COMMAND_NOT_ALLOWED)
        warnings.append(f"forbidden_robot_control_fields={forbidden_fields}")

    blocking_reasons.extend(
        validate_task_contract(
            raw_task,
            confidence_threshold=confidence_threshold,
            max_cartesian_translation_m=max_cartesian_translation_m,
            allowed_cartesian_frames=allowed_cartesian_frames,
        )
    )
    blocking_reasons = _unique([str(reason) for reason in blocking_reasons if reason])
    warnings = _unique([str(warning) for warning in warnings if warning])
    status = STATUS_PASS if not blocking_reasons else STATUS_SAFE_DISABLED if adapter_mode == MODE_LLM_DISABLED else STATUS_BLOCKED
    accepted = status == STATUS_PASS
    rejected = status == STATUS_BLOCKED

    return {
        **_contract_fields(raw_task),
        "contract_version": CONTRACT_VERSION,
        "schema_version": CONTRACT_VERSION,
        "teto_version": CURRENT_COMMAND_TO_TASK_VERSION,
        "command_to_task_requested": True,
        "requested": True,
        "config_path": request.config_path,
        "adapter_mode": adapter_mode,
        "command_to_task_status": status,
        "user_command": user_command,
        "normalized_command": normalized_command,
        "accepted": accepted,
        "rejected": rejected,
        "error_code": blocking_reasons[0] if blocking_reasons else _string(raw_task.get("error_code")),
        "blocking_reasons": blocking_reasons,
        "warnings": warnings,
        "confidence_threshold": confidence_threshold,
        "max_cartesian_translation_m": max_cartesian_translation_m,
        "llm_called": adapter_mode == MODE_LLM_QWEN and "E_LLM_CALL_FAILED" not in blocking_reasons,
        "live_camera_used": False,
        "ros2_publish_attempted": False,
        "moveit_called": False,
        "real_robot_motion_executed": False,
        "robot_command_generated": False,
        "trajectory_generated": False,
        "joint_targets_generated": False,
        "tcp_pose_world_generated": False,
        "forbidden_robot_control_fields": forbidden_fields,
        "next_safe_action": _next_safe_action(status),
        "task_contract": _task_contract(raw_task, user_command, normalized_command),
        "safety_boundary": _safety_boundary(),
    }


def validate_task_contract(
    task: Dict[str, Any],
    *,
    confidence_threshold: float = DEFAULT_CONFIDENCE_THRESHOLD,
    max_cartesian_translation_m: float = DEFAULT_MAX_CARTESIAN_TRANSLATION_M,
    allowed_cartesian_frames: set[str] | None = None,
) -> list[str]:
    reasons: list[str] = []
    intent = _normalize_intent(task.get("intent") or task.get("intent_name"))
    confidence = _confidence(task)
    allowed_cartesian_frames = allowed_cartesian_frames or set(ALLOWED_CARTESIAN_FRAMES)

    if intent not in SUPPORTED_INTENTS:
        reasons.append(E_UNSUPPORTED_INTENT)
    if intent == INTENT_HOVER_TO_OBJECT and not _target_label(task):
        reasons.append(E_TARGET_LABEL_REQUIRED)
    if intent == INTENT_MOVE_TO_WAYPOINT and not _string(task.get("waypoint")):
        reasons.append(E_WAYPOINT_REQUIRED)
    if intent == INTENT_CARTESIAN_OFFSET:
        frame = _string(task.get("frame")) or DEFAULT_CARTESIAN_FRAME
        offset = _cartesian_offset(task)
        if frame not in allowed_cartesian_frames:
            reasons.append(E_INVALID_FRAME)
        if offset is None:
            reasons.append(E_CARTESIAN_OFFSET_REQUIRED)
        elif not _valid_cartesian_offset(offset):
            reasons.append(E_INVALID_CARTESIAN_OFFSET)
        elif _translation_distance(offset) > float(max_cartesian_translation_m):
            reasons.append(E_EXCESSIVE_CARTESIAN_MOTION)
    if confidence is None or confidence < float(confidence_threshold):
        reasons.append(E_LOW_CONFIDENCE)
    if _forbidden_robot_control_fields(task):
        reasons.append(E_ROBOT_COMMAND_NOT_ALLOWED)
    return _unique(reasons)


def build_task_prompt(user_command: str | None) -> str:
    command = user_command.strip() if isinstance(user_command, str) else ""
    return (
        "Convert the user command into strict JSON only. Do not output Markdown or explanation.\n"
        "You may only choose a task intent. Do not output robot commands, ROS topics, MoveIt goals, "
        "URScript, RTDE, Dashboard commands, poses, trajectories, joint targets, or coordinates.\n\n"
        "Allowed JSON schema:\n"
        "{\n"
        '  "intent": "hover_to_object | go_home | move_to_waypoint | cartesian_offset | stop",\n'
        '  "target_label": "snake_case object label or null",\n'
        '  "waypoint": "snake_case waypoint name or null",\n'
        '  "frame": "base_link or null",\n'
        '  "dx": 0.0,\n'
        '  "dy": 0.0,\n'
        '  "dz": 0.0,\n'
        '  "confidence": 0.0,\n'
        '  "error_code": "OK or E_UNSUPPORTED_COMMAND"\n'
        "}\n\n"
        "Rules:\n"
        "- For commands like move above/hover over/go above an object, use intent hover_to_object.\n"
        "- For go home/return home, use intent go_home.\n"
        "- For move to inspection pose or named pose, use intent move_to_waypoint.\n"
        "- For Cartesian nudges like move higher/lower/left/right/forward/back, use cartesian_offset.\n"
        "- Cartesian offsets are meters in base_link: forward +dx, back -dx, left +dy, right -dy, up/higher +dz, down/lower -dz.\n"
        "- Convert cm, millimeters, meters, slightly, and a little into numeric meter offsets; slightly/a little means 0.03 m.\n"
        "- Never output a target pose, tcp_pose_world, trajectory, joint target, MoveIt goal, ROS topic, URScript, RTDE, or Dashboard command.\n"
        "- Use snake_case labels, e.g. red mug -> red_mug.\n"
        "- If unsupported, choose stop only for explicit stop/halt; otherwise use confidence 0.0 and error_code E_UNSUPPORTED_COMMAND.\n\n"
        f"User command: {command}"
    )


def format_command_to_task_report(result: Dict[str, Any]) -> str:
    return "\n".join(
        [
            "# TETO V3.0.1 Command-to-Task Adapter Report",
            "",
            f"- command_to_task_status: {_format_value(result.get('command_to_task_status'))}",
            f"- adapter_mode: {_format_value(result.get('adapter_mode'))}",
            f"- user_command: {_format_value(result.get('user_command'))}",
            f"- intent: {_format_value(result.get('intent'))}",
            f"- target_label: {_format_value(result.get('target_label'))}",
            f"- waypoint: {_format_value(result.get('waypoint'))}",
            f"- frame: {_format_value(result.get('frame'))}",
            f"- cartesian_offset_m: {_format_value(result.get('cartesian_offset_m'))}",
            f"- confidence: {_format_value(result.get('confidence'))}",
            f"- blocking_reasons: {_format_value(result.get('blocking_reasons'))}",
            "",
            "Safety: this adapter outputs task intent only. It does not publish ROS, call MoveIt, send robot commands, or bypass downstream validation gates.",
        ]
    )


def _not_requested_result() -> Dict[str, Any]:
    return {
        "contract_version": CONTRACT_VERSION,
        "schema_version": CONTRACT_VERSION,
        "teto_version": CURRENT_COMMAND_TO_TASK_VERSION,
        "command_to_task_requested": False,
        "requested": False,
        "command_to_task_status": STATUS_NOT_REQUESTED,
        "intent": None,
        "target_label": None,
        "waypoint": None,
        "frame": None,
        "dx": None,
        "dy": None,
        "dz": None,
        "cartesian_offset_m": None,
        "translation_distance_m": None,
        "confidence": None,
        "accepted": False,
        "rejected": False,
        "blocking_reasons": [],
        "warnings": [],
        "llm_called": False,
        "live_camera_used": False,
        "ros2_publish_attempted": False,
        "moveit_called": False,
        "real_robot_motion_executed": False,
        "robot_command_generated": False,
        "trajectory_generated": False,
        "joint_targets_generated": False,
        "tcp_pose_world_generated": False,
        "safety_boundary": _safety_boundary(),
    }


def _rule_based_task(user_command: str | None) -> Dict[str, Any]:
    command = _normalize_text(user_command)
    task = _base_task(user_command)
    if not command:
        return {**task, "confidence": 0.0, "error_code": "E_UNSUPPORTED_COMMAND"}
    if re.search(r"\b(?:stop|halt)\b", command):
        return {**task, "intent": INTENT_STOP, "confidence": 0.95, "error_code": "OK"}
    if re.search(r"\b(?:go|return|move)\s+home\b", command):
        return {**task, "intent": INTENT_GO_HOME, "confidence": 0.95, "error_code": "OK"}
    waypoint_match = re.search(r"\b(?:move|go)\s+to\s+(?:the\s+)?(.+?)\s+(?:pose|waypoint)\b", command)
    if waypoint_match:
        return {
            **task,
            "intent": INTENT_MOVE_TO_WAYPOINT,
            "waypoint": _snake_case(waypoint_match.group(1) + "_pose"),
            "confidence": 0.90,
            "error_code": "OK",
        }
    hover_match = re.search(r"\b(?:move\s+above|hover\s+over|go\s+above)\s+(?:the\s+|a\s+|an\s+)?(.+)$", command)
    if hover_match:
        return {
            **task,
            "intent": INTENT_HOVER_TO_OBJECT,
            "target_label": _snake_case(hover_match.group(1)),
            "confidence": 0.90,
            "error_code": "OK",
        }
    return {**task, "confidence": 0.0, "error_code": "E_UNSUPPORTED_COMMAND"}


def _offline_llm_task(config: Dict[str, Any]) -> Dict[str, Any]:
    raw = _dict(config.get("llm_response")) or _load_declared_json(config.get("llm_response_path"))
    if raw is None:
        return {**_base_task(_string(config.get("user_command"))), "confidence": 0.0, "error_code": E_PARSE_FAILED}
    return _task_from_llm_payload(raw, _string(config.get("user_command")))


def _qwen_llm_task(
    config: Dict[str, Any],
    user_command: str | None,
    llm_callable: Callable[[str], str] | None,
) -> Dict[str, Any]:
    prompt = build_task_prompt(user_command)
    try:
        response_text = llm_callable(prompt) if llm_callable else _call_configured_llm(config, prompt)
        payload = _extract_json(response_text)
    except Exception as exc:
        return {
            **_base_task(user_command),
            "confidence": 0.0,
            "error_code": E_LLM_CALL_FAILED,
            "warnings": [f"llm_error={exc}"],
        }
    return _task_from_llm_payload(payload, user_command)


def _call_configured_llm(config: Dict[str, Any], prompt: str) -> str:
    backend = _string(config.get("llm_backend")) or "ollama"
    if backend == "ollama":
        return _call_ollama(config, prompt)
    if backend == "transformers":
        return _call_transformers_qwen(config, prompt)
    raise ValueError(f"unsupported llm_backend: {backend}")


def _call_ollama(config: Dict[str, Any], prompt: str) -> str:
    try:
        from ollama import chat
    except ImportError as exc:
        raise RuntimeError("ollama package is required for llm_backend=ollama") from exc
    model = _string(config.get("model_name")) or os.environ.get("TETO_QWEN_MODEL") or DEFAULT_MODEL_NAME
    response = chat(model=model, messages=[{"role": "user", "content": prompt}])
    message = getattr(response, "message", None)
    content = getattr(message, "content", None)
    if content is not None:
        return content
    if isinstance(response, dict):
        message = response.get("message")
        if isinstance(message, dict) and isinstance(message.get("content"), str):
            return message["content"]
    raise RuntimeError("LLM response did not contain message.content")


def _call_transformers_qwen(config: Dict[str, Any], prompt: str) -> str:
    try:
        import torch
        from transformers import AutoProcessor, Qwen2_5_VLForConditionalGeneration
    except ImportError as exc:
        raise RuntimeError("torch and transformers are required for llm_backend=transformers") from exc

    model_path = _string(config.get("model_path")) or os.environ.get("TETO_QWEN_MODEL_PATH")
    if not model_path:
        raise RuntimeError("model_path or TETO_QWEN_MODEL_PATH is required for llm_backend=transformers")
    model = Qwen2_5_VLForConditionalGeneration.from_pretrained(
        model_path,
        torch_dtype="auto",
        device_map="auto",
    )
    processor = AutoProcessor.from_pretrained(model_path)
    messages = [{"role": "user", "content": [{"type": "text", "text": prompt}]}]
    text = processor.apply_chat_template(messages, tokenize=False, add_generation_prompt=True)
    inputs = processor(text=[text], return_tensors="pt").to(model.device)
    generated = model.generate(**inputs, max_new_tokens=int(config.get("max_new_tokens", 256)))
    generated = generated[:, inputs.input_ids.shape[-1] :]
    decoded = processor.batch_decode(generated, skip_special_tokens=True, clean_up_tokenization_spaces=False)
    return decoded[0] if decoded else ""


def _task_from_llm_payload(payload: Dict[str, Any], user_command: str | None) -> Dict[str, Any]:
    task = _base_task(user_command)
    error_code = _string(payload.get("error_code")) or "OK"
    return {
        **task,
        "_llm_payload": payload,
        "intent": _normalize_intent(payload.get("intent") or payload.get("intent_name")),
        "target_label": _snake_case(_string(payload.get("target_label"))),
        "waypoint": _snake_case(_string(payload.get("waypoint"))),
        "frame": _string(payload.get("frame")) or DEFAULT_CARTESIAN_FRAME,
        "dx": _optional_float(payload.get("dx")),
        "dy": _optional_float(payload.get("dy")),
        "dz": _optional_float(payload.get("dz")),
        "confidence": _confidence(payload),
        "error_code": error_code,
    }


def _base_task(user_command: str | None) -> Dict[str, Any]:
    return {
        "intent": None,
        "target_label": None,
        "waypoint": None,
        "frame": None,
        "dx": None,
        "dy": None,
        "dz": None,
        "confidence": None,
        "error_code": None,
        "user_command": user_command,
    }


def _contract_fields(task: Dict[str, Any]) -> Dict[str, Any]:
    intent = _normalize_intent(task.get("intent") or task.get("intent_name"))
    offset = _cartesian_offset(task) if intent == INTENT_CARTESIAN_OFFSET else None
    return {
        "intent": intent,
        "intent_name": intent,
        "target_label": _target_label(task),
        "waypoint": _string(task.get("waypoint")),
        "frame": _string(task.get("frame")) if intent == INTENT_CARTESIAN_OFFSET else None,
        "dx": offset[0] if offset else None,
        "dy": offset[1] if offset else None,
        "dz": offset[2] if offset else None,
        "cartesian_offset_m": offset,
        "translation_distance_m": round(_translation_distance(offset), 6) if offset else None,
        "confidence": _confidence(task),
    }


def _task_contract(task: Dict[str, Any], user_command: str | None, normalized_command: str) -> Dict[str, Any]:
    fields = _contract_fields(task)
    return {
        "schema_version": CONTRACT_VERSION,
        "intent": fields["intent"],
        "target_label": fields["target_label"],
        "waypoint": fields["waypoint"],
        "frame": fields["frame"],
        "dx": fields["dx"],
        "dy": fields["dy"],
        "dz": fields["dz"],
        "cartesian_offset_m": fields["cartesian_offset_m"],
        "translation_distance_m": fields["translation_distance_m"],
        "confidence": fields["confidence"],
        "user_command": user_command,
        "normalized_command": normalized_command,
        "execution_policy": {
            "intent_only": True,
            "allow_ros_publish": False,
            "allow_moveit_execute": False,
            "allow_robot_motion": False,
            "requires_planner_gateway": True,
            "requires_validation_gates": True,
        },
    }


def _extract_json(text: str) -> Dict[str, Any]:
    raw = text.strip() if isinstance(text, str) else ""
    if not raw:
        raise ValueError("empty LLM response")
    raw = re.sub(r"^```(?:json)?\s*", "", raw)
    raw = re.sub(r"\s*```$", "", raw)
    try:
        payload = json.loads(raw)
    except json.JSONDecodeError:
        match = re.search(r"\{.*\}", raw, flags=re.DOTALL)
        if not match:
            raise
        payload = json.loads(match.group(0))
    if not isinstance(payload, dict):
        raise ValueError("LLM JSON response must be an object")
    return payload


def _load_declared_json(path: Any) -> Dict[str, Any] | None:
    if not isinstance(path, str) or not path:
        return None
    resolved = Path(path).expanduser()
    if not resolved.is_file():
        return None
    with resolved.open("r", encoding="utf-8") as result_file:
        data = json.load(result_file) if resolved.suffix.lower() == ".json" else yaml.safe_load(result_file)
    return data if isinstance(data, dict) else None


def _normalize_intent(value: Any) -> str | None:
    intent = _string(value)
    if intent == "return_home":
        return INTENT_GO_HOME
    return intent


def _target_label(task: Dict[str, Any]) -> str | None:
    return _snake_case(_string(task.get("target_label") or task.get("target_query")))


def _normalize_text(value: str | None) -> str:
    if not isinstance(value, str):
        return ""
    return re.sub(r"\s+", " ", value.strip()).casefold()


def _snake_case(value: str | None) -> str | None:
    if not value:
        return None
    cleaned = re.sub(r"\b(?:the|a|an|please)\b", " ", value.casefold())
    cleaned = re.sub(r"[^0-9a-zA-Z\u3040-\u30ff\u4e00-\u9fff]+", "_", cleaned)
    cleaned = re.sub(r"_+", "_", cleaned).strip("_")
    return cleaned or None


def _dict(value: Any) -> Dict[str, Any] | None:
    return value if isinstance(value, dict) else None


def _string(value: Any) -> str | None:
    return value.strip() if isinstance(value, str) and value.strip() else None


def _string_list(value: Any) -> list[str]:
    return [str(item) for item in value] if isinstance(value, list) else []


def _optional_float(value: Any) -> float | None:
    if isinstance(value, bool):
        return None
    if isinstance(value, (int, float)):
        return float(value)
    return None


def _confidence(value: Dict[str, Any]) -> float | None:
    confidence = _optional_float(value.get("confidence"))
    if confidence is not None:
        return confidence
    return _optional_float(value.get("overall_confidence"))


def _cartesian_offset(task: Dict[str, Any]) -> list[float] | None:
    values = [_optional_float(task.get(axis)) for axis in ("dx", "dy", "dz")]
    if any(value is None for value in values):
        return None
    return [float(value) for value in values if value is not None]


def _valid_cartesian_offset(offset: list[float]) -> bool:
    if len(offset) != 3:
        return False
    if not all(_finite_number(value) for value in offset):
        return False
    return any(abs(value) > 0.0 for value in offset)


def _translation_distance(offset: list[float] | None) -> float:
    if not offset or len(offset) != 3:
        return 0.0
    return sum(float(value) ** 2 for value in offset) ** 0.5


def _finite_number(value: Any) -> bool:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return False
    return value == value and value not in {float("inf"), float("-inf")}


def _allowed_cartesian_frames(config: Dict[str, Any]) -> set[str]:
    frames = config.get("allowed_cartesian_frames")
    if isinstance(frames, list):
        values = {_string(frame) for frame in frames}
        return {frame for frame in values if frame}
    return set(ALLOWED_CARTESIAN_FRAMES)


def _forbidden_robot_control_fields(value: Any, prefix: str = "") -> list[str]:
    found: list[str] = []
    if isinstance(value, dict):
        for key, child in value.items():
            key_name = str(key)
            path = f"{prefix}.{key_name}" if prefix else key_name
            if key_name in FORBIDDEN_ROBOT_CONTROL_FIELDS:
                found.append(path)
            found.extend(_forbidden_robot_control_fields(child, path))
    elif isinstance(value, list):
        for index, item in enumerate(value):
            found.extend(_forbidden_robot_control_fields(item, f"{prefix}[{index}]"))
    return _unique(found)


def _unique(values: list[str]) -> list[str]:
    seen: set[str] = set()
    result: list[str] = []
    for value in values:
        if value and value not in seen:
            seen.add(value)
            result.append(value)
    return result


def _next_safe_action(status: str) -> str:
    if status == STATUS_PASS:
        return "Pass this task intent to validation and planner gateway; do not execute from LLM output."
    if status == STATUS_SAFE_DISABLED:
        return "Enable an explicit adapter mode or use offline/rule-based evidence for tests."
    return "Fix command-to-task output before planner gateway."


def _safety_boundary() -> Dict[str, bool]:
    return {
        "intent_only": True,
        "no_live_camera": True,
        "no_ros2_publish": True,
        "no_moveit": True,
        "no_rtde": True,
        "no_urscript": True,
        "no_dashboard": True,
        "no_real_robot_motion": True,
        "no_trajectory": True,
        "no_tcp_pose_world": True,
        "no_joint_targets": True,
        "requires_planner_gateway": True,
        "requires_validation_gates": True,
    }


def _format_value(value: Any) -> str:
    if value is None:
        return "null"
    if isinstance(value, (dict, list)):
        return json.dumps(value, ensure_ascii=False, sort_keys=True)
    return str(value)
