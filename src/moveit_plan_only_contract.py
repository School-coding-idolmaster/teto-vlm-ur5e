from __future__ import annotations

import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict

import yaml


CONTRACT_VERSION = "teto_moveit_plan_only.v1"
CURRENT_MOVEIT_PLAN_ONLY_VERSION = "TETO V2.11.0"

STATUS_NOT_REQUESTED = "NOT_REQUESTED"
STATUS_PASS = "PASS"
STATUS_BLOCKED = "BLOCKED"
PLAN_ONLY_READY = "PLAN_ONLY_READY"
MESSAGE_EXPORTED = "MESSAGE_EXPORTED"

E_ROS2_MESSAGE_EXPORT_MISSING = "E_ROS2_MESSAGE_EXPORT_MISSING"
E_ROS2_MESSAGE_EXPORT_NOT_READY = "E_ROS2_MESSAGE_EXPORT_NOT_READY"
E_MESSAGE_SCHEMA_MISSING = "E_MESSAGE_SCHEMA_MISSING"
E_BOUNDED_TARGET_POINT_MISSING = "E_BOUNDED_TARGET_POINT_MISSING"
E_INVALID_BOUNDED_TARGET_POINT = "E_INVALID_BOUNDED_TARGET_POINT"
E_OUT_OF_WORKSPACE = "E_OUT_OF_WORKSPACE"
E_PLANNING_GROUP_MISSING = "E_PLANNING_GROUP_MISSING"
E_PLANNING_FRAME_MISSING = "E_PLANNING_FRAME_MISSING"
E_END_EFFECTOR_FRAME_MISSING = "E_END_EFFECTOR_FRAME_MISSING"
E_PLAN_ONLY_REQUIRED = "E_PLAN_ONLY_REQUIRED"
E_MOVEIT_EXECUTION_NOT_ALLOWED = "E_MOVEIT_EXECUTION_NOT_ALLOWED"
E_TRAJECTORY_SEND_NOT_ALLOWED = "E_TRAJECTORY_SEND_NOT_ALLOWED"
E_EXECUTION_NOT_ALLOWED_IN_PLAN_ONLY = "E_EXECUTION_NOT_ALLOWED_IN_PLAN_ONLY"
E_REAL_ROBOT_NOT_ALLOWED = "E_REAL_ROBOT_NOT_ALLOWED"

DEFAULT_WORKSPACE_BOUNDS = {
    "x": [-1.0, 1.0],
    "y": [-1.0, 1.0],
    "z": [0.0, 2.0],
}

MOVEIT_PLAN_ONLY_FIELDS = (
    "moveit_plan_only_requested",
    "moveit_plan_only_status",
    "plan_only_status",
    "plan_only_ready",
    "planning_group",
    "planning_frame",
    "end_effector_frame",
    "bounded_target_point_m",
    "moveit_plan_requested",
    "moveit_plan_only",
    "moveit_execute_allowed",
    "moveit_execute_called",
    "trajectory_generated",
    "trajectory_send_allowed",
    "trajectory_sent",
    "controller_command_sent",
    "execution_allowed",
    "real_robot_enabled",
    "blocking_reasons",
    "warnings",
)


@dataclass(frozen=True)
class MoveItPlanOnlyRequest:
    requested: bool = False
    config_path: str | None = None
    config: Dict[str, Any] | None = None
    ros2_message_export_result: Dict[str, Any] | None = None


def load_moveit_plan_only_config(path: str | Path | None) -> Dict[str, Any]:
    return _load_config(path, "moveit_plan_only")


def build_moveit_plan_only_request(
    *,
    requested: bool = False,
    config_path: str | Path | None = None,
    config: Dict[str, Any] | None = None,
    ros2_message_export_result: Dict[str, Any] | None = None,
) -> MoveItPlanOnlyRequest:
    loaded_config = config if isinstance(config, dict) else load_moveit_plan_only_config(config_path)
    return MoveItPlanOnlyRequest(
        requested=requested,
        config_path=str(Path(config_path).expanduser()) if config_path else None,
        config=loaded_config,
        ros2_message_export_result=ros2_message_export_result
        if isinstance(ros2_message_export_result, dict)
        else None,
    )


def evaluate_moveit_plan_only(request: MoveItPlanOnlyRequest | None = None) -> Dict[str, Any]:
    request = request or MoveItPlanOnlyRequest()
    if not request.requested:
        return _not_requested_result()

    config = request.config if isinstance(request.config, dict) else {}
    export = _message_export_result(request, config)
    exported_message = export.get("exported_message") if isinstance(export.get("exported_message"), dict) else {}
    message_ready = (
        export.get("ros2_message_export_status") == STATUS_PASS
        and export.get("message_export_status") == MESSAGE_EXPORTED
    )
    message_schema = _string(export.get("message_schema") or exported_message.get("schema"))
    bounded_target_point_m = export.get("bounded_target_point_m", exported_message.get("bounded_target_point_m"))
    planning_group = _string(config.get("planning_group"))
    planning_frame = _string(config.get("planning_frame"))
    end_effector_frame = _string(config.get("end_effector_frame"))
    workspace_bounds = _workspace_bounds(config)
    plan_only_requested = config.get("plan_only", True) is True
    moveit_execute_allowed_requested = config.get("moveit_execute_allowed", False) is True
    trajectory_send_allowed_requested = config.get("trajectory_send_allowed", False) is True
    execution_allowed_requested = config.get("execution_allowed", False) is True
    real_robot_enabled_requested = config.get("real_robot_enabled", False) is True

    blocking_reasons: list[str] = []
    warnings = _string_list(config.get("warnings")) + _string_list(export.get("warnings"))

    if not export:
        blocking_reasons.append(E_ROS2_MESSAGE_EXPORT_MISSING)
    elif not message_ready:
        blocking_reasons.append(E_ROS2_MESSAGE_EXPORT_NOT_READY)
    if not message_schema:
        blocking_reasons.append(E_MESSAGE_SCHEMA_MISSING)
    if bounded_target_point_m is None:
        blocking_reasons.append(E_BOUNDED_TARGET_POINT_MISSING)
    elif not _valid_point3(bounded_target_point_m):
        blocking_reasons.append(E_INVALID_BOUNDED_TARGET_POINT)
    elif not _point_in_workspace(bounded_target_point_m, workspace_bounds):
        blocking_reasons.append(E_OUT_OF_WORKSPACE)
    if not planning_group:
        blocking_reasons.append(E_PLANNING_GROUP_MISSING)
    if not planning_frame:
        blocking_reasons.append(E_PLANNING_FRAME_MISSING)
    if not end_effector_frame:
        blocking_reasons.append(E_END_EFFECTOR_FRAME_MISSING)
    if not plan_only_requested:
        blocking_reasons.append(E_PLAN_ONLY_REQUIRED)
    if moveit_execute_allowed_requested or config.get("moveit_execute_called", False) is True:
        blocking_reasons.append(E_MOVEIT_EXECUTION_NOT_ALLOWED)
    if trajectory_send_allowed_requested or config.get("trajectory_sent", False) is True:
        blocking_reasons.append(E_TRAJECTORY_SEND_NOT_ALLOWED)
    if execution_allowed_requested:
        blocking_reasons.append(E_EXECUTION_NOT_ALLOWED_IN_PLAN_ONLY)
    if real_robot_enabled_requested:
        blocking_reasons.append(E_REAL_ROBOT_NOT_ALLOWED)

    blocking_reasons = _unique(blocking_reasons)
    warnings = _unique(warnings)
    status = STATUS_PASS if not blocking_reasons else STATUS_BLOCKED
    plan_only_status = PLAN_ONLY_READY if status == STATUS_PASS else STATUS_BLOCKED
    bounded_plan_goal = _build_plan_goal(
        export=export,
        planning_group=planning_group,
        planning_frame=planning_frame,
        end_effector_frame=end_effector_frame,
        bounded_target_point_m=bounded_target_point_m,
    ) if status == STATUS_PASS else None

    return {
        "contract_version": CONTRACT_VERSION,
        "schema_version": CONTRACT_VERSION,
        "teto_version": CURRENT_MOVEIT_PLAN_ONLY_VERSION,
        "moveit_plan_only_requested": True,
        "requested": True,
        "config_path": request.config_path,
        "moveit_plan_only_status": status,
        "plan_only_status": plan_only_status,
        "plan_only_ready": status == STATUS_PASS,
        "message_id": export.get("message_id"),
        "message_schema": message_schema,
        "intent_name": export.get("intent_name") or exported_message.get("intent_name"),
        "planning_group": planning_group,
        "planning_frame": planning_frame,
        "end_effector_frame": end_effector_frame,
        "workspace_bounds": workspace_bounds,
        "bounded_target_point_m": _round_point(bounded_target_point_m) if _valid_point3(bounded_target_point_m) else bounded_target_point_m,
        "world_frame": export.get("world_frame") or exported_message.get("world_frame"),
        "robot_base_frame": export.get("robot_base_frame") or exported_message.get("robot_base_frame"),
        "camera_frame": export.get("camera_frame") or exported_message.get("camera_frame"),
        "moveit_plan_requested": True,
        "moveit_plan_only": True,
        "moveit_execute_allowed": False,
        "moveit_execute_called": False,
        "trajectory_generated": False,
        "trajectory_send_allowed": False,
        "trajectory_sent": False,
        "controller_command_sent": False,
        "execution_allowed": False,
        "real_robot_enabled": False,
        "real_robot_motion_executed": False,
        "tcp_pose_world_generated": False,
        "joint_targets_generated": False,
        "robot_command_generated": False,
        "blocking_reasons": blocking_reasons,
        "warnings": warnings,
        "next_safe_action": _next_safe_action(status),
        "plan_only_goal_constraint": bounded_plan_goal,
        "bounded_plan_goal": bounded_plan_goal,
        "hover_goal_constraint": bounded_plan_goal,
        "ros2_message_export_result": export,
        "safety_boundary": _safety_boundary(),
    }


def format_moveit_plan_only_report(result: Dict[str, Any]) -> str:
    return "\n".join(
        [
            "# TETO V2.11.0 MoveIt Plan-Only Contract Report",
            "",
            "## Overall Status",
            "",
            f"- moveit_plan_only_status: {_format_value(result.get('moveit_plan_only_status'))}",
            f"- plan_only_status: {_format_value(result.get('plan_only_status'))}",
            f"- planning_group: {_format_value(result.get('planning_group'))}",
            f"- planning_frame: {_format_value(result.get('planning_frame'))}",
            f"- end_effector_frame: {_format_value(result.get('end_effector_frame'))}",
            f"- bounded_target_point_m: {_format_value(result.get('bounded_target_point_m'))}",
            f"- blocking_reasons: {_format_value(result.get('blocking_reasons'))}",
            f"- warnings: {_format_value(result.get('warnings'))}",
            "",
            "## Contract Fields",
            "",
            "| Field | Value |",
            "| --- | --- |",
            *[f"| {field} | {_format_value(result.get(field))} |" for field in MOVEIT_PLAN_ONLY_FIELDS],
            "",
            "## Non-Executable Goal Constraint",
            "",
            "The goal constraint is plan-only evidence and is not an executable joint target or tcp_pose_world command.",
            "",
            "```json",
            json.dumps(result.get("bounded_plan_goal"), ensure_ascii=False, indent=2, sort_keys=True),
            "```",
            "",
            "## No-Motion Safety Boundary",
            "",
            "V2.11.0 creates MoveIt plan-only evidence only. It does not execute a MoveIt plan, send a trajectory, create executable joint targets, generate tcp_pose_world, or command a real UR5.",
            "",
        ]
    )


def _build_plan_goal(
    *,
    export: Dict[str, Any],
    planning_group: str | None,
    planning_frame: str | None,
    end_effector_frame: str | None,
    bounded_target_point_m: Any,
) -> Dict[str, Any]:
    return {
        "constraint_type": "bounded_hover_goal",
        "non_executable": True,
        "message_id": export.get("message_id"),
        "planning_group": planning_group,
        "planning_frame": planning_frame,
        "end_effector_frame": end_effector_frame,
        "bounded_target_point_m": _round_point(bounded_target_point_m),
        "execution_allowed": False,
        "trajectory_send_allowed": False,
    }


def _not_requested_result() -> Dict[str, Any]:
    return {
        "contract_version": CONTRACT_VERSION,
        "schema_version": CONTRACT_VERSION,
        "teto_version": CURRENT_MOVEIT_PLAN_ONLY_VERSION,
        "moveit_plan_only_requested": False,
        "requested": False,
        "moveit_plan_only_status": STATUS_NOT_REQUESTED,
        "plan_only_status": STATUS_NOT_REQUESTED,
        "plan_only_ready": False,
        "moveit_plan_requested": False,
        "moveit_plan_only": True,
        "moveit_execute_allowed": False,
        "moveit_execute_called": False,
        "trajectory_generated": False,
        "trajectory_send_allowed": False,
        "trajectory_sent": False,
        "controller_command_sent": False,
        "execution_allowed": False,
        "real_robot_enabled": False,
        "real_robot_motion_executed": False,
        "tcp_pose_world_generated": False,
        "joint_targets_generated": False,
        "robot_command_generated": False,
        "blocking_reasons": [],
        "warnings": [],
        "next_safe_action": _next_safe_action(STATUS_NOT_REQUESTED),
        "safety_boundary": _safety_boundary(),
    }


def _message_export_result(request: MoveItPlanOnlyRequest, config: Dict[str, Any]) -> Dict[str, Any]:
    if isinstance(request.ros2_message_export_result, dict) and request.ros2_message_export_result:
        return _unwrap(request.ros2_message_export_result, "ros2_message_export")
    if isinstance(config.get("ros2_message_export_result"), dict):
        return _unwrap(config["ros2_message_export_result"], "ros2_message_export")
    return _load_result(config.get("ros2_message_export_result_path"), "ros2_message_export")


def _workspace_bounds(config: Dict[str, Any]) -> Dict[str, list[float]]:
    source = config.get("workspace_bounds") if isinstance(config.get("workspace_bounds"), dict) else DEFAULT_WORKSPACE_BOUNDS
    bounds: Dict[str, list[float]] = {}
    for axis in ("x", "y", "z"):
        values = source.get(axis)
        if isinstance(values, (list, tuple)) and len(values) == 2:
            bounds[axis] = [float(values[0]), float(values[1])]
        else:
            bounds[axis] = list(DEFAULT_WORKSPACE_BOUNDS[axis])
    return bounds


def _point_in_workspace(point: Any, bounds: Dict[str, list[float]]) -> bool:
    if not _valid_point3(point):
        return False
    for value, axis in zip(point, ("x", "y", "z")):
        lower, upper = bounds[axis]
        if float(value) < lower or float(value) > upper:
            return False
    return True


def _next_safe_action(status: str) -> str:
    if status == STATUS_PASS:
        return "Use as plan-only shadow evidence; do not execute or send trajectories."
    if status == STATUS_BLOCKED:
        return "Fix message export or plan-only declarations while keeping execution disabled."
    return "Request MoveIt plan-only check with message export evidence."


def _safety_boundary() -> Dict[str, Any]:
    return {
        "execution_allowed": False,
        "moveit_plan_requested": True,
        "moveit_plan_only": True,
        "moveit_execute_allowed": False,
        "moveit_execute_called": False,
        "trajectory_generated": False,
        "trajectory_send_allowed": False,
        "trajectory_sent": False,
        "controller_command_sent": False,
        "robot_command_generated": False,
        "tcp_pose_world_generated": False,
        "joint_targets_generated": False,
        "real_robot_enabled": False,
        "real_robot_motion_executed": False,
    }


def _load_config(path: Any, root_key: str) -> Dict[str, Any]:
    data = _load_yaml_or_json(path)
    if not isinstance(data, dict):
        return {}
    config = data.get(root_key)
    return config if isinstance(config, dict) else data


def _load_result(path: Any, root_key: str) -> Dict[str, Any]:
    return _unwrap(_load_yaml_or_json(path), root_key)


def _load_yaml_or_json(path: Any) -> Dict[str, Any]:
    if not path:
        return {}
    resolved_path = Path(str(path)).expanduser()
    if not resolved_path.is_file():
        return {}
    with resolved_path.open("r", encoding="utf-8") as result_file:
        data = json.load(result_file) if resolved_path.suffix.lower() == ".json" else yaml.safe_load(result_file)
    return data if isinstance(data, dict) else {}


def _unwrap(data: Any, root_key: str) -> Dict[str, Any]:
    if not isinstance(data, dict):
        return {}
    nested = data.get(root_key)
    return nested if isinstance(nested, dict) else data


def _valid_point3(value: Any) -> bool:
    if not isinstance(value, (list, tuple)) or len(value) != 3:
        return False
    return all(isinstance(item, (int, float)) and not isinstance(item, bool) and math.isfinite(item) for item in value)


def _round_point(value: Any) -> list[float] | Any:
    if not _valid_point3(value):
        return value
    return [round(float(item), 6) for item in value]


def _string(value: Any) -> str | None:
    if isinstance(value, str):
        stripped = value.strip()
        return stripped or None
    if value is None:
        return None
    return str(value).strip() or None


def _string_list(value: Any) -> list[str]:
    if isinstance(value, (list, tuple)):
        return [str(item) for item in value if item]
    if value:
        return [str(value)]
    return []


def _unique(values: list[str]) -> list[str]:
    output: list[str] = []
    seen: set[str] = set()
    for value in values:
        if value and value not in seen:
            seen.add(value)
            output.append(value)
    return output


def _format_value(value: Any) -> str:
    if value is None:
        return "null"
    if isinstance(value, bool):
        return "true" if value else "false"
    if isinstance(value, (list, dict)):
        return json.dumps(value, ensure_ascii=False, sort_keys=True)
    return str(value)
