from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Dict


CURRENT_REAL_UR5_HOVER_EXECUTOR_VERSION = "TETO V3.0.0"

STATUS_PASS = "PASS"
STATUS_BLOCKED = "BLOCKED"

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
E_WORKSPACE_VIOLATION = "E_WORKSPACE_VIOLATION"
E_TARGET_DEPTH_INVALID = "E_TARGET_DEPTH_INVALID"
E_TF_UNAVAILABLE = "E_TF_UNAVAILABLE"
E_SCENE_STALE = "E_SCENE_STALE"
E_LOW_CONFIDENCE = "E_LOW_CONFIDENCE"
E_MANUAL_CONFIRMATION_REQUIRED = "E_MANUAL_CONFIRMATION_REQUIRED"
E_EXECUTION_FAILED = "E_EXECUTION_FAILED"
E_POST_MOTION_STATE_INVALID = "E_POST_MOTION_STATE_INVALID"
E_HOME_TARGET_UNAVAILABLE = "E_HOME_TARGET_UNAVAILABLE"

DEFAULT_MAX_SPEED_SCALE = 0.10
DEFAULT_MAX_ACC_SCALE = 0.10


@dataclass(frozen=True)
class RealUR5HoverExecutionRequest:
    config: Dict[str, Any] | None = None
    planner_gateway_result: Dict[str, Any] | None = None
    ros2_interface_result: Dict[str, Any] | None = None
    moveit_plan_result: Dict[str, Any] | None = None
    ur5_state_result: Dict[str, Any] | None = None
    manual_confirmation_result: Dict[str, Any] | None = None


def evaluate_real_ur5_hover_execution(request: RealUR5HoverExecutionRequest | None = None) -> Dict[str, Any]:
    request = request or RealUR5HoverExecutionRequest()
    config = request.config if isinstance(request.config, dict) else {}
    planner = request.planner_gateway_result if isinstance(request.planner_gateway_result, dict) else {}
    ros2 = request.ros2_interface_result if isinstance(request.ros2_interface_result, dict) else {}
    moveit = request.moveit_plan_result if isinstance(request.moveit_plan_result, dict) else {}
    state = request.ur5_state_result if isinstance(request.ur5_state_result, dict) else {}
    confirmation = request.manual_confirmation_result if isinstance(request.manual_confirmation_result, dict) else {}

    blocking_reasons: list[str] = []
    warnings: list[str] = []

    for flag in (
        "enable_live_camera",
        "enable_ros2_runtime",
        "enable_moveit_plan",
        "enable_moveit_execute",
        "enable_real_robot_motion",
    ):
        if config.get(flag) is not True:
            blocking_reasons.append(E_REAL_MOTION_NOT_ENABLED if flag == "enable_real_robot_motion" else _flag_reason(flag))
    if config.get("enable_live_vlm") is not True and config.get("approved_local_mock_vlm") is not True:
        blocking_reasons.append(E_REAL_MOTION_NOT_ENABLED)
    if config.get("manual_confirmation_required", True) is not True:
        blocking_reasons.append(E_MANUAL_CONFIRMATION_REQUIRED)
    if confirmation.get("manual_confirmation_accepted") is not True:
        blocking_reasons.append(E_MANUAL_CONFIRMATION_REQUIRED)

    if config.get("ros2_runtime_available", ros2.get("ros2_runtime_available")) is not True:
        blocking_reasons.append(E_ROS2_RUNTIME_UNAVAILABLE)
    if config.get("moveit_runtime_available", config.get("enable_moveit_plan")) is not True:
        blocking_reasons.append(E_MOVEIT_RUNTIME_UNAVAILABLE)
    if config.get("moveit_plan_success", moveit.get("plan_only_ready")) is not True:
        blocking_reasons.append(E_MOVEIT_PLAN_FAILED)
    if config.get("moveit_execute_allowed", config.get("enable_moveit_execute")) is not True:
        blocking_reasons.append(E_MOVEIT_EXECUTE_NOT_ALLOWED)

    robot_state_ok = _flag_from(config, state, "robot_state_ok", default=state.get("read_only_state_contract_ready"))
    safety_status_ok = _flag_from(config, state, "safety_status_ok", default=True)
    protective_stop = _flag_from(config, state, "protective_stop", default=False)
    emergency_stop = _flag_from(config, state, "emergency_stop", default=False)
    speed_scaling = _float(config.get("speed_scaling", state.get("speed_scaling")))
    max_speed_scale = _float(config.get("max_speed_scale")) or DEFAULT_MAX_SPEED_SCALE
    max_acc_scale = _float(config.get("max_acc_scale")) or DEFAULT_MAX_ACC_SCALE

    if robot_state_ok is not True:
        blocking_reasons.append(E_ROBOT_STATE_NOT_OK)
    if safety_status_ok is not True:
        blocking_reasons.append(E_SAFETY_STATUS_NOT_OK)
    if protective_stop is True:
        blocking_reasons.append(E_PROTECTIVE_STOP_ACTIVE)
    if emergency_stop is True:
        blocking_reasons.append(E_EMERGENCY_STOP_ACTIVE)
    if speed_scaling is not None and (speed_scaling < 0.0 or speed_scaling > max_speed_scale):
        blocking_reasons.append(E_SPEED_SCALING_UNSAFE)
    if max_speed_scale > DEFAULT_MAX_SPEED_SCALE or max_acc_scale > DEFAULT_MAX_ACC_SCALE:
        blocking_reasons.append(E_SPEED_SCALING_UNSAFE)

    if config.get("workspace_check_passed", planner.get("workspace_check_passed")) is not True:
        blocking_reasons.append(E_WORKSPACE_VIOLATION)
    if config.get("target_depth_valid", True) is not True:
        blocking_reasons.append(E_TARGET_DEPTH_INVALID)
    if config.get("tf_available", True) is not True:
        blocking_reasons.append(E_TF_UNAVAILABLE)
    if config.get("scene_ttl_valid", planner.get("ttl_check_passed", True)) is not True:
        blocking_reasons.append(E_SCENE_STALE)
    if _float(config.get("confidence_overall", planner.get("overall_confidence"))) is not None:
        confidence = _float(config.get("confidence_overall", planner.get("overall_confidence")))
        threshold = _float(config.get("confidence_threshold")) or 0.70
        if confidence is not None and confidence < threshold:
            blocking_reasons.append(E_LOW_CONFIDENCE)

    bounded_target_point_m = config.get("bounded_target_point_m", planner.get("bounded_target_point_m"))
    workspace_bounds = _workspace_bounds(config)
    if not _valid_point3(bounded_target_point_m) or not _point_in_workspace(bounded_target_point_m, workspace_bounds):
        blocking_reasons.append(E_WORKSPACE_VIOLATION)

    if _flag_any(config, ("urscript_generated", "rtde_write_attempted", "dashboard_command_attempted", "raw_joint_targets_generated", "tcp_pose_world_generated_by_llm")):
        blocking_reasons.append(E_EXECUTION_FAILED)
        warnings.append("forbidden_robot_control_artifact_detected")

    blocking_reasons = _unique(blocking_reasons)
    status = STATUS_PASS if not blocking_reasons else STATUS_BLOCKED
    trajectory_send_allowed = status == STATUS_PASS
    executed = status == STATUS_PASS

    return_home_requested = config.get("return_home_requested", False) is True
    return_home_executed = False
    return_home_skipped = False
    if executed and return_home_requested:
        if config.get("allow_return_home", True) is True and _string(config.get("return_home_named_target")):
            return_home_executed = True
        else:
            return_home_skipped = True
            warnings.append(E_HOME_TARGET_UNAVAILABLE)

    return {
        "teto_version": CURRENT_REAL_UR5_HOVER_EXECUTOR_VERSION,
        "real_ur5_hover_executor_status": status,
        "moveit_execute_called": executed,
        "trajectory_send_allowed": trajectory_send_allowed,
        "trajectory_sent": executed,
        "controller_command_sent": executed,
        "real_robot_motion_executed": executed,
        "return_home_requested": return_home_requested,
        "return_home_executed": return_home_executed,
        "return_home_skipped": return_home_skipped,
        "urscript_generated": False,
        "rtde_write_attempted": False,
        "dashboard_command_attempted": False,
        "raw_joint_targets_generated": False,
        "tcp_pose_world_generated_by_llm": False,
        "bounded_target_point_m": bounded_target_point_m,
        "max_speed_scale": max_speed_scale,
        "max_acc_scale": max_acc_scale,
        "blocking_reasons": blocking_reasons,
        "warnings": _unique(warnings),
        "safety_boundary": {
            "uses_ros2_moveit_only": True,
            "forbid_urscript": True,
            "forbid_rtde_write": True,
            "forbid_dashboard_command": True,
            "forbid_raw_joint_targets": True,
            "forbid_tcp_pose_world_from_llm": True,
        },
    }


def _flag_reason(flag: str) -> str:
    if flag == "enable_live_camera":
        return E_REAL_MOTION_NOT_ENABLED
    if flag == "enable_ros2_runtime":
        return E_ROS2_RUNTIME_UNAVAILABLE
    if flag == "enable_moveit_plan":
        return E_MOVEIT_RUNTIME_UNAVAILABLE
    return E_MOVEIT_EXECUTE_NOT_ALLOWED


def _flag_from(config: Dict[str, Any], state: Dict[str, Any], name: str, default: Any = None) -> Any:
    if name in config:
        return config.get(name)
    if name in state:
        return state.get(name)
    return default


def _flag_any(config: Dict[str, Any], names: tuple[str, ...]) -> bool:
    return any(config.get(name) is True for name in names)


def _workspace_bounds(config: Dict[str, Any]) -> Dict[str, list[float]]:
    raw = config.get("workspace_bounds") if isinstance(config.get("workspace_bounds"), dict) else {}
    if {"x_min", "x_max", "y_min", "y_max", "z_min", "z_max"}.issubset(raw):
        return {
            "x": [float(raw["x_min"]), float(raw["x_max"])],
            "y": [float(raw["y_min"]), float(raw["y_max"])],
            "z": [float(raw["z_min"]), float(raw["z_max"])],
        }
    return {
        "x": _pair(raw.get("x"), [-1.0, 1.0]),
        "y": _pair(raw.get("y"), [-1.0, 1.0]),
        "z": _pair(raw.get("z"), [0.0, 2.0]),
    }


def _pair(value: Any, default: list[float]) -> list[float]:
    if isinstance(value, (list, tuple)) and len(value) == 2:
        return [float(value[0]), float(value[1])]
    return list(default)


def _point_in_workspace(point: Any, bounds: Dict[str, list[float]]) -> bool:
    if not _valid_point3(point):
        return False
    x, y, z = [float(item) for item in point]
    return bounds["x"][0] <= x <= bounds["x"][1] and bounds["y"][0] <= y <= bounds["y"][1] and bounds["z"][0] <= z <= bounds["z"][1]


def _valid_point3(value: Any) -> bool:
    return (
        isinstance(value, (list, tuple))
        and len(value) == 3
        and all(isinstance(item, (int, float)) and not isinstance(item, bool) and math.isfinite(item) for item in value)
    )


def _float(value: Any) -> float | None:
    if isinstance(value, bool):
        return None
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def _string(value: Any) -> str | None:
    if isinstance(value, str):
        return value.strip() or None
    if value is None:
        return None
    return str(value).strip() or None


def _unique(values: list[str]) -> list[str]:
    output: list[str] = []
    seen: set[str] = set()
    for value in values:
        if value and value not in seen:
            seen.add(value)
            output.append(value)
    return output
