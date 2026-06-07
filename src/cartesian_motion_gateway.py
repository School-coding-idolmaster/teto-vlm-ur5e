from __future__ import annotations

import json
import math
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Callable, Dict

import yaml

from src.command_to_task_adapter import (
    INTENT_CARTESIAN_OFFSET,
    STATUS_PASS as TASK_STATUS_PASS,
    CommandToTaskAdapterRequest,
    evaluate_command_to_task_adapter,
)
from src.manual_confirmation_gate import (
    DEFAULT_CONFIRMATION_TIMEOUT_S,
    ManualConfirmationRequest,
    evaluate_manual_confirmation_gate,
)


CONTRACT_VERSION = "teto_cartesian_motion_gateway.v1"
CURRENT_CARTESIAN_MOTION_VERSION = "TETO V3.1.0"

STATUS_PASS = "PASS"
STATUS_BLOCKED = "BLOCKED"
STATUS_NOT_REQUESTED = "NOT_REQUESTED"

DEFAULT_FRAME = "base_link"
DEFAULT_MAX_TRANSLATION_M = 0.20
DEFAULT_CONFIRMATION_TOKEN = "CONFIRM_REAL_UR5_CARTESIAN"
DEFAULT_UNDERSTANDING_PHRASE = "I understand this will move the real UR5"
DEFAULT_WORKSPACE_BOUNDS = {
    "x": [-1.0, 1.0],
    "y": [-1.0, 1.0],
    "z": [0.0, 2.0],
}
ALLOWED_FRAMES = {"base_link"}

E_TASK_NOT_READY = "E_TASK_NOT_READY"
E_UNSUPPORTED_INTENT = "E_UNSUPPORTED_INTENT"
E_INVALID_FRAME = "E_INVALID_FRAME"
E_OFFSET_MISSING = "E_OFFSET_MISSING"
E_INVALID_OFFSET = "E_INVALID_OFFSET"
E_EXCESSIVE_CARTESIAN_MOTION = "E_EXCESSIVE_CARTESIAN_MOTION"
E_CURRENT_TCP_POSE_MISSING = "E_CURRENT_TCP_POSE_MISSING"
E_INVALID_CURRENT_TCP_POSE = "E_INVALID_CURRENT_TCP_POSE"
E_OUT_OF_WORKSPACE = "E_OUT_OF_WORKSPACE"
E_TARGET_NOT_READY = "E_TARGET_NOT_READY"
E_REAL_MOTION_NOT_ENABLED = "E_REAL_MOTION_NOT_ENABLED"
E_ROS2_RUNTIME_UNAVAILABLE = "E_ROS2_RUNTIME_UNAVAILABLE"
E_MOVEIT_RUNTIME_UNAVAILABLE = "E_MOVEIT_RUNTIME_UNAVAILABLE"
E_MOVEIT_PLAN_FAILED = "E_MOVEIT_PLAN_FAILED"
E_MOVEIT_EXECUTE_NOT_ALLOWED = "E_MOVEIT_EXECUTE_NOT_ALLOWED"
E_ROBOT_STATE_NOT_OK = "E_ROBOT_STATE_NOT_OK"
E_SAFETY_STATUS_NOT_OK = "E_SAFETY_STATUS_NOT_OK"
E_PROTECTIVE_STOP_ACTIVE = "E_PROTECTIVE_STOP_ACTIVE"
E_EMERGENCY_STOP_ACTIVE = "E_EMERGENCY_STOP_ACTIVE"
E_SPEED_SCALING_UNSAFE = "E_SPEED_SCALING_UNSAFE"
E_MANUAL_CONFIRMATION_REQUIRED = "E_MANUAL_CONFIRMATION_REQUIRED"
E_FORBIDDEN_CONTROL_ARTIFACT = "E_FORBIDDEN_CONTROL_ARTIFACT"


@dataclass(frozen=True)
class CartesianMotionGatewayRequest:
    requested: bool = False
    config_path: str | None = None
    config: Dict[str, Any] | None = None
    command_to_task_result: Dict[str, Any] | None = None
    current_tcp_pose: Dict[str, Any] | list[float] | None = None


@dataclass(frozen=True)
class CartesianMotionExecutionRequest:
    requested: bool = False
    config: Dict[str, Any] | None = None
    cartesian_motion_result: Dict[str, Any] | None = None
    manual_confirmation_result: Dict[str, Any] | None = None
    ur5_state_result: Dict[str, Any] | None = None


@dataclass(frozen=True)
class CartesianMotionPipelineRequest:
    requested: bool = False
    user_command: str | None = None
    config_path: str | None = None
    config: Dict[str, Any] | None = None
    llm_callable: Callable[[str], str] | None = None
    manual_confirmation_token: str | None = None


def load_cartesian_motion_config(path: str | Path | None) -> Dict[str, Any]:
    if not path:
        return {}
    resolved = Path(path).expanduser()
    if not resolved.is_file():
        return {}
    with resolved.open("r", encoding="utf-8") as config_file:
        data = json.load(config_file) if resolved.suffix.lower() == ".json" else yaml.safe_load(config_file)
    if not isinstance(data, dict):
        return {}
    config = data.get("cartesian_motion_pipeline") or data.get("cartesian_motion_gateway")
    return config if isinstance(config, dict) else data


def evaluate_cartesian_motion_gateway(request: CartesianMotionGatewayRequest | None = None) -> Dict[str, Any]:
    request = request or CartesianMotionGatewayRequest()
    if not request.requested:
        return _not_requested_gateway()

    config = request.config if isinstance(request.config, dict) else {}
    task = request.command_to_task_result if isinstance(request.command_to_task_result, dict) else {}
    contract = task.get("task_contract") if isinstance(task.get("task_contract"), dict) else task
    current_pose = _normalize_pose(request.current_tcp_pose if request.current_tcp_pose is not None else config.get("current_tcp_pose"))
    frame = _string(contract.get("frame")) or DEFAULT_FRAME
    offset = _offset_from_contract(contract)
    max_translation_m = _optional_float(config.get("max_translation_m")) or DEFAULT_MAX_TRANSLATION_M
    workspace_bounds = _workspace_bounds(config)
    allowed_frames = _allowed_frames(config)

    blocking_reasons: list[str] = []
    warnings = _string_list(config.get("warnings")) + _string_list(task.get("warnings"))

    if task.get("command_to_task_status") != TASK_STATUS_PASS:
        blocking_reasons.append(E_TASK_NOT_READY)
    if contract.get("intent") != INTENT_CARTESIAN_OFFSET:
        blocking_reasons.append(E_UNSUPPORTED_INTENT)
    if frame not in allowed_frames:
        blocking_reasons.append(E_INVALID_FRAME)
    if offset is None:
        blocking_reasons.append(E_OFFSET_MISSING)
    elif not _valid_vector3(offset) or not any(abs(value) > 0.0 for value in offset):
        blocking_reasons.append(E_INVALID_OFFSET)
    elif _distance(offset) > max_translation_m:
        blocking_reasons.append(E_EXCESSIVE_CARTESIAN_MOTION)
    if current_pose is None:
        blocking_reasons.append(E_CURRENT_TCP_POSE_MISSING)
    elif not _valid_pose(current_pose):
        blocking_reasons.append(E_INVALID_CURRENT_TCP_POSE)

    target_pose = None
    workspace_check_passed = False
    if not blocking_reasons and current_pose is not None and offset is not None:
        target_position = [round(float(left) + float(right), 6) for left, right in zip(current_pose["position_m"], offset)]
        workspace_check_passed = _point_in_workspace(target_position, workspace_bounds)
        if not workspace_check_passed:
            blocking_reasons.append(E_OUT_OF_WORKSPACE)
        else:
            target_pose = {
                "frame": frame,
                "position_m": target_position,
                "orientation_xyzw": list(current_pose["orientation_xyzw"]),
            }

    blocking_reasons = _unique(blocking_reasons)
    warnings = _unique(warnings)
    status = STATUS_PASS if not blocking_reasons else STATUS_BLOCKED
    task_id = _task_id(task, offset)

    return {
        "contract_version": CONTRACT_VERSION,
        "schema_version": CONTRACT_VERSION,
        "teto_version": CURRENT_CARTESIAN_MOTION_VERSION,
        "cartesian_motion_gateway_requested": True,
        "cartesian_motion_gateway_status": status,
        "task_id": task_id,
        "intent": contract.get("intent"),
        "frame": frame,
        "cartesian_offset_m": _round_vector(offset),
        "translation_distance_m": round(_distance(offset), 6) if offset else None,
        "max_translation_m": max_translation_m,
        "current_tcp_pose": current_pose,
        "target_pose": target_pose,
        "target_position_m": target_pose.get("position_m") if target_pose else None,
        "workspace_bounds": workspace_bounds,
        "workspace_check_passed": workspace_check_passed,
        "moveit_plan_request": _moveit_plan_request(task_id, frame, target_pose, config) if status == STATUS_PASS else None,
        "target_pose_generated_by_teto": status == STATUS_PASS,
        "target_pose_generated_by_llm": False,
        "trajectory_generated": False,
        "joint_targets_generated": False,
        "robot_command_generated": False,
        "real_robot_motion_executed": False,
        "blocking_reasons": blocking_reasons,
        "warnings": warnings,
        "next_safe_action": _gateway_next_action(status),
        "command_to_task_result": task,
        "safety_boundary": _safety_boundary(intent_only=False),
    }


def evaluate_cartesian_motion_execution(request: CartesianMotionExecutionRequest | None = None) -> Dict[str, Any]:
    request = request or CartesianMotionExecutionRequest()
    if not request.requested:
        return _not_requested_execution()

    config = request.config if isinstance(request.config, dict) else {}
    motion = request.cartesian_motion_result if isinstance(request.cartesian_motion_result, dict) else {}
    confirmation = request.manual_confirmation_result if isinstance(request.manual_confirmation_result, dict) else {}
    state = request.ur5_state_result if isinstance(request.ur5_state_result, dict) else {}
    blocking_reasons: list[str] = []
    warnings = _string_list(config.get("warnings")) + _string_list(motion.get("warnings"))

    if motion.get("cartesian_motion_gateway_status") != STATUS_PASS or not isinstance(motion.get("target_pose"), dict):
        blocking_reasons.append(E_TARGET_NOT_READY)
    for flag in ("enable_ros2_runtime", "enable_moveit_plan", "enable_moveit_execute", "enable_real_robot_motion"):
        if config.get(flag) is not True:
            blocking_reasons.append(_flag_reason(flag))
    if config.get("manual_confirmation_required", True) is not True:
        blocking_reasons.append(E_MANUAL_CONFIRMATION_REQUIRED)
    if confirmation.get("manual_confirmation_accepted") is not True:
        blocking_reasons.append(E_MANUAL_CONFIRMATION_REQUIRED)
    if config.get("ros2_runtime_available", config.get("enable_ros2_runtime")) is not True:
        blocking_reasons.append(E_ROS2_RUNTIME_UNAVAILABLE)
    if config.get("moveit_runtime_available", config.get("enable_moveit_plan")) is not True:
        blocking_reasons.append(E_MOVEIT_RUNTIME_UNAVAILABLE)
    if config.get("moveit_plan_success", config.get("enable_moveit_plan")) is not True:
        blocking_reasons.append(E_MOVEIT_PLAN_FAILED)
    if config.get("moveit_execute_allowed", config.get("enable_moveit_execute")) is not True:
        blocking_reasons.append(E_MOVEIT_EXECUTE_NOT_ALLOWED)
    if _flag_from(config, state, "robot_state_ok", state.get("read_only_state_contract_ready")) is not True:
        blocking_reasons.append(E_ROBOT_STATE_NOT_OK)
    if _flag_from(config, state, "safety_status_ok", True) is not True:
        blocking_reasons.append(E_SAFETY_STATUS_NOT_OK)
    if _flag_from(config, state, "protective_stop", False) is True:
        blocking_reasons.append(E_PROTECTIVE_STOP_ACTIVE)
    if _flag_from(config, state, "emergency_stop", False) is True:
        blocking_reasons.append(E_EMERGENCY_STOP_ACTIVE)

    speed_scaling = _optional_float(config.get("speed_scaling", state.get("speed_scaling")))
    max_speed_scale = _optional_float(config.get("max_speed_scale")) or 0.10
    max_acc_scale = _optional_float(config.get("max_acc_scale")) or 0.10
    if speed_scaling is not None and (speed_scaling < 0.0 or speed_scaling > max_speed_scale):
        blocking_reasons.append(E_SPEED_SCALING_UNSAFE)
    if max_speed_scale > 0.10 or max_acc_scale > 0.10:
        blocking_reasons.append(E_SPEED_SCALING_UNSAFE)
    if motion.get("target_pose_generated_by_llm") is True or _forbidden_artifact(config):
        blocking_reasons.append(E_FORBIDDEN_CONTROL_ARTIFACT)
        warnings.append("forbidden_control_artifact_detected")

    blocking_reasons = _unique(blocking_reasons)
    warnings = _unique(warnings)
    status = STATUS_PASS if not blocking_reasons else STATUS_BLOCKED
    executed = status == STATUS_PASS
    return {
        "contract_version": CONTRACT_VERSION,
        "schema_version": CONTRACT_VERSION,
        "teto_version": CURRENT_CARTESIAN_MOTION_VERSION,
        "cartesian_motion_execution_requested": True,
        "cartesian_motion_execution_status": status,
        "task_id": motion.get("task_id"),
        "moveit_plan_requested": True,
        "moveit_plan_success": config.get("moveit_plan_success", config.get("enable_moveit_plan")) is True,
        "manual_confirmation_required": config.get("manual_confirmation_required", True) is True,
        "manual_confirmation_accepted": confirmation.get("manual_confirmation_accepted") is True,
        "moveit_execute_called": executed,
        "trajectory_send_allowed": executed,
        "trajectory_sent": executed,
        "controller_command_sent": executed,
        "real_robot_motion_executed": executed,
        "target_pose": motion.get("target_pose"),
        "target_pose_generated_by_llm": False,
        "urscript_generated": False,
        "rtde_write_attempted": False,
        "dashboard_command_attempted": False,
        "raw_joint_targets_generated": False,
        "blocking_reasons": blocking_reasons,
        "warnings": warnings,
        "safety_boundary": _safety_boundary(intent_only=False),
    }


def evaluate_cartesian_motion_pipeline(request: CartesianMotionPipelineRequest | None = None) -> Dict[str, Any]:
    request = request or CartesianMotionPipelineRequest()
    if not request.requested:
        return {
            "contract_version": CONTRACT_VERSION,
            "schema_version": CONTRACT_VERSION,
            "teto_version": CURRENT_CARTESIAN_MOTION_VERSION,
            "cartesian_motion_pipeline_status": STATUS_NOT_REQUESTED,
            "real_robot_motion_executed": False,
            "blocking_reasons": [],
            "warnings": [],
        }

    config = _merge_config(load_cartesian_motion_config(request.config_path), request.config)
    user_command = request.user_command or _string(config.get("user_command")) or ""
    adapter_config = dict(_nested(config, "command_to_task_adapter") or {})
    adapter_config.setdefault("adapter_mode", "qwen_llm")
    adapter_config.setdefault("max_cartesian_translation_m", config.get("max_translation_m", DEFAULT_MAX_TRANSLATION_M))
    command_task = evaluate_command_to_task_adapter(
        CommandToTaskAdapterRequest(
            requested=True,
            user_command=user_command,
            config_path=request.config_path,
            config=adapter_config,
            llm_callable=request.llm_callable,
        )
    )
    gateway = evaluate_cartesian_motion_gateway(
        CartesianMotionGatewayRequest(
            requested=True,
            config_path=request.config_path,
            config=config,
            command_to_task_result=command_task,
            current_tcp_pose=config.get("current_tcp_pose"),
        )
    )
    manual_confirmation = evaluate_manual_confirmation_gate(
        ManualConfirmationRequest(
            manual_confirmation_required=config.get("manual_confirmation_required", True) is True,
            expected_token=_string(config.get("manual_confirmation_token")) or DEFAULT_CONFIRMATION_TOKEN,
            expected_task_id=_string(gateway.get("task_id")),
            expected_target_label="cartesian_offset",
            expected_bounded_target_point_m=gateway.get("target_position_m")
            if isinstance(gateway.get("target_position_m"), list)
            else None,
            timeout_s=int(config.get("manual_confirmation_timeout_s", DEFAULT_CONFIRMATION_TIMEOUT_S)),
            confirmation=_confirmation_from_request(config, request, gateway),
            required_phrase=_string(config.get("manual_confirmation_phrase")) or DEFAULT_UNDERSTANDING_PHRASE,
        )
    )
    execution = evaluate_cartesian_motion_execution(
        CartesianMotionExecutionRequest(
            requested=True,
            config=config,
            cartesian_motion_result=gateway,
            manual_confirmation_result=manual_confirmation,
            ur5_state_result=_nested(config, "ur5_state") or _nested(config, "ur5_read_only_state"),
        )
    )

    blocking_reasons = _unique(
        _string_list(command_task.get("blocking_reasons"))
        + _string_list(gateway.get("blocking_reasons"))
        + (
            _string_list(manual_confirmation.get("blocking_reasons"))
            if config.get("enable_real_robot_motion") is True
            else []
        )
        + (
            _string_list(execution.get("blocking_reasons"))
            if config.get("enable_real_robot_motion") is True
            else []
        )
    )
    warnings = _unique(_string_list(command_task.get("warnings")) + _string_list(gateway.get("warnings")) + _string_list(execution.get("warnings")))
    status = STATUS_PASS if not blocking_reasons else STATUS_BLOCKED
    return {
        "contract_version": CONTRACT_VERSION,
        "schema_version": CONTRACT_VERSION,
        "teto_version": CURRENT_CARTESIAN_MOTION_VERSION,
        "cartesian_motion_pipeline_requested": True,
        "cartesian_motion_pipeline_status": status,
        "user_command": user_command,
        "intent": command_task.get("intent"),
        "frame": command_task.get("frame"),
        "cartesian_offset_m": command_task.get("cartesian_offset_m"),
        "current_tcp_pose": gateway.get("current_tcp_pose"),
        "target_pose": gateway.get("target_pose"),
        "moveit_plan_request": gateway.get("moveit_plan_request"),
        "manual_confirmation_required": config.get("manual_confirmation_required", True) is True,
        "manual_confirmation_accepted": manual_confirmation.get("manual_confirmation_accepted") is True,
        "moveit_execute_called": execution.get("moveit_execute_called") is True,
        "real_robot_motion_executed": execution.get("real_robot_motion_executed") is True,
        "blocking_reasons": blocking_reasons,
        "warnings": warnings,
        "command_to_task_result": command_task,
        "cartesian_motion_gateway_result": gateway,
        "manual_confirmation_result": manual_confirmation,
        "cartesian_motion_execution_result": execution,
        "safety_boundary": _safety_boundary(intent_only=False),
    }


def _not_requested_gateway() -> Dict[str, Any]:
    return {
        "contract_version": CONTRACT_VERSION,
        "schema_version": CONTRACT_VERSION,
        "teto_version": CURRENT_CARTESIAN_MOTION_VERSION,
        "cartesian_motion_gateway_requested": False,
        "cartesian_motion_gateway_status": STATUS_NOT_REQUESTED,
        "target_pose_generated_by_teto": False,
        "target_pose_generated_by_llm": False,
        "real_robot_motion_executed": False,
        "blocking_reasons": [],
        "warnings": [],
    }


def _not_requested_execution() -> Dict[str, Any]:
    return {
        "contract_version": CONTRACT_VERSION,
        "schema_version": CONTRACT_VERSION,
        "teto_version": CURRENT_CARTESIAN_MOTION_VERSION,
        "cartesian_motion_execution_requested": False,
        "cartesian_motion_execution_status": STATUS_NOT_REQUESTED,
        "moveit_execute_called": False,
        "real_robot_motion_executed": False,
        "blocking_reasons": [],
        "warnings": [],
    }


def _moveit_plan_request(task_id: str, frame: str, target_pose: Dict[str, Any] | None, config: Dict[str, Any]) -> Dict[str, Any] | None:
    if not target_pose:
        return None
    return {
        "schema_version": "teto_moveit_cartesian_pose_goal.v1",
        "task_id": task_id,
        "planning_group": _string(config.get("planning_group")) or "ur_manipulator",
        "planning_frame": frame,
        "end_effector_frame": _string(config.get("end_effector_frame")) or "tool0",
        "target_pose": target_pose,
        "max_speed_scale": _optional_float(config.get("max_speed_scale")) or 0.10,
        "max_acc_scale": _optional_float(config.get("max_acc_scale")) or 0.10,
        "manual_confirmation_required": True,
        "generated_by_teto": True,
        "generated_by_llm": False,
    }


def _normalize_pose(value: Any) -> Dict[str, Any] | None:
    if isinstance(value, dict):
        position = value.get("position_m") or value.get("position") or value.get("xyz")
        orientation = value.get("orientation_xyzw") or value.get("orientation") or value.get("quat_xyzw")
        return {
            "frame": _string(value.get("frame")) or DEFAULT_FRAME,
            "position_m": list(position) if isinstance(position, (list, tuple)) else None,
            "orientation_xyzw": list(orientation) if isinstance(orientation, (list, tuple)) else [0.0, 0.0, 0.0, 1.0],
        }
    if isinstance(value, (list, tuple)) and len(value) in {3, 7}:
        orientation = list(value[3:7]) if len(value) == 7 else [0.0, 0.0, 0.0, 1.0]
        return {"frame": DEFAULT_FRAME, "position_m": list(value[:3]), "orientation_xyzw": orientation}
    return None


def _valid_pose(pose: Dict[str, Any]) -> bool:
    return (
        _string(pose.get("frame")) in ALLOWED_FRAMES
        and _valid_vector3(pose.get("position_m"))
        and _valid_quaternion(pose.get("orientation_xyzw"))
    )


def _valid_vector3(value: Any) -> bool:
    return (
        isinstance(value, list)
        and len(value) == 3
        and all(isinstance(item, (int, float)) and not isinstance(item, bool) and math.isfinite(float(item)) for item in value)
    )


def _valid_quaternion(value: Any) -> bool:
    return (
        isinstance(value, list)
        and len(value) == 4
        and all(isinstance(item, (int, float)) and not isinstance(item, bool) and math.isfinite(float(item)) for item in value)
    )


def _offset_from_contract(contract: Dict[str, Any]) -> list[float] | None:
    offset = contract.get("cartesian_offset_m")
    if _valid_vector3(offset):
        return [float(value) for value in offset]
    values = [_optional_float(contract.get(axis)) for axis in ("dx", "dy", "dz")]
    if any(value is None for value in values):
        return None
    return [float(value) for value in values if value is not None]


def _workspace_bounds(config: Dict[str, Any]) -> Dict[str, list[float]]:
    raw = config.get("workspace_bounds") if isinstance(config.get("workspace_bounds"), dict) else {}
    return {
        "x": _pair(raw.get("x"), DEFAULT_WORKSPACE_BOUNDS["x"]),
        "y": _pair(raw.get("y"), DEFAULT_WORKSPACE_BOUNDS["y"]),
        "z": _pair(raw.get("z"), DEFAULT_WORKSPACE_BOUNDS["z"]),
    }


def _pair(value: Any, default: list[float]) -> list[float]:
    if isinstance(value, (list, tuple)) and len(value) == 2:
        low = _optional_float(value[0])
        high = _optional_float(value[1])
        if low is not None and high is not None and low <= high:
            return [low, high]
    return list(default)


def _point_in_workspace(point: list[float], bounds: Dict[str, list[float]]) -> bool:
    return (
        bounds["x"][0] <= point[0] <= bounds["x"][1]
        and bounds["y"][0] <= point[1] <= bounds["y"][1]
        and bounds["z"][0] <= point[2] <= bounds["z"][1]
    )


def _allowed_frames(config: Dict[str, Any]) -> set[str]:
    frames = config.get("allowed_frames") or config.get("allowed_cartesian_frames")
    if isinstance(frames, list):
        return {frame for frame in (_string(item) for item in frames) if frame}
    return set(ALLOWED_FRAMES)


def _distance(offset: list[float] | None) -> float:
    if not offset:
        return 0.0
    return sum(float(value) ** 2 for value in offset) ** 0.5


def _round_vector(value: list[float] | None) -> list[float] | None:
    return [round(float(item), 6) for item in value] if value is not None else None


def _task_id(task: Dict[str, Any], offset: list[float] | None) -> str:
    value = _string(task.get("task_id"))
    if value:
        return value
    suffix = "_".join(str(round(float(item), 3)).replace("-", "m").replace(".", "p") for item in (offset or [0.0, 0.0, 0.0]))
    return f"cartesian_offset_{suffix}"


def _confirmation_from_request(
    config: Dict[str, Any],
    request: CartesianMotionPipelineRequest,
    gateway: Dict[str, Any],
) -> Dict[str, Any] | None:
    confirmation = config.get("manual_confirmation")
    if isinstance(confirmation, dict):
        return confirmation
    token = request.manual_confirmation_token
    if not token:
        return None
    return {
        "confirmation_token": token,
        "task_id": gateway.get("task_id"),
        "target_label": "cartesian_offset",
        "bounded_target_point_m": gateway.get("target_position_m"),
        "understanding": _string(config.get("manual_confirmation_phrase")) or DEFAULT_UNDERSTANDING_PHRASE,
        "confirmed_at_epoch_s": datetime.now(timezone.utc).timestamp(),
    }


def _flag_reason(flag: str) -> str:
    if flag == "enable_ros2_runtime":
        return E_ROS2_RUNTIME_UNAVAILABLE
    if flag == "enable_moveit_plan":
        return E_MOVEIT_RUNTIME_UNAVAILABLE
    if flag == "enable_moveit_execute":
        return E_MOVEIT_EXECUTE_NOT_ALLOWED
    return E_REAL_MOTION_NOT_ENABLED


def _flag_from(config: Dict[str, Any], state: Dict[str, Any], name: str, default: Any = None) -> Any:
    if name in config:
        return config.get(name)
    if name in state:
        return state.get(name)
    return default


def _forbidden_artifact(config: Dict[str, Any]) -> bool:
    return any(
        config.get(name) is True
        for name in (
            "urscript_generated",
            "rtde_write_attempted",
            "dashboard_command_attempted",
            "raw_joint_targets_generated",
            "target_pose_generated_by_llm",
        )
    )


def _gateway_next_action(status: str) -> str:
    if status == STATUS_PASS:
        return "Plan the TETO-generated target pose with MoveIt, then require manual confirmation before execution."
    return "Fix the Cartesian task contract or current TCP pose before planning."


def _safety_boundary(*, intent_only: bool) -> Dict[str, bool]:
    return {
        "llm_intent_only": True,
        "target_pose_generated_by_teto": not intent_only,
        "target_pose_generated_by_llm": False,
        "requires_current_tcp_pose": True,
        "requires_workspace_validation": True,
        "requires_moveit_plan": True,
        "requires_manual_confirmation": True,
        "requires_moveit_execute_gate": True,
        "no_direct_robot_commands_from_llm": True,
        "no_urscript": True,
        "no_rtde_write": True,
        "no_dashboard": True,
        "no_raw_joint_targets_from_llm": True,
    }


def _merge_config(base: Dict[str, Any], override: Dict[str, Any] | None) -> Dict[str, Any]:
    merged = dict(base)
    if isinstance(override, dict):
        merged.update(override)
    return merged


def _nested(config: Dict[str, Any], key: str) -> Dict[str, Any]:
    value = config.get(key)
    return value if isinstance(value, dict) else {}


def _string(value: Any) -> str | None:
    if isinstance(value, str):
        return value.strip() or None
    if value is None:
        return None
    return str(value).strip() or None


def _string_list(value: Any) -> list[str]:
    return [str(item) for item in value] if isinstance(value, list) else []


def _optional_float(value: Any) -> float | None:
    if isinstance(value, bool):
        return None
    if isinstance(value, (int, float)):
        return float(value)
    return None


def _unique(values: list[str]) -> list[str]:
    output: list[str] = []
    seen: set[str] = set()
    for value in values:
        if value and value not in seen:
            output.append(value)
            seen.add(value)
    return output
