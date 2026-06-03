from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict

import yaml


CONTRACT_VERSION = "teto_robot_system_shadow_bridge.v1"
CURRENT_ROBOT_SYSTEM_SHADOW_BRIDGE_VERSION = "TETO V2.11.0"

STATUS_NOT_REQUESTED = "NOT_REQUESTED"
STATUS_PASS = "PASS"
STATUS_BLOCKED = "BLOCKED"
ROBOT_SYSTEM_SHADOW_READY = "ROBOT_SYSTEM_SHADOW_READY"
MESSAGE_EXPORTED = "MESSAGE_EXPORTED"
PLAN_ONLY_READY = "PLAN_ONLY_READY"
READ_ONLY_STATE_CONTRACT_READY = "READ_ONLY_STATE_CONTRACT_READY"

E_ROS2_MESSAGE_EXPORT_NOT_READY = "E_ROS2_MESSAGE_EXPORT_NOT_READY"
E_MOVEIT_PLAN_ONLY_NOT_READY = "E_MOVEIT_PLAN_ONLY_NOT_READY"
E_UR5_READ_ONLY_STATE_NOT_READY = "E_UR5_READ_ONLY_STATE_NOT_READY"
E_EXECUTION_NOT_ALLOWED_VIOLATED = "E_EXECUTION_NOT_ALLOWED_VIOLATED"
E_ROBOT_COMMAND_GENERATED = "E_ROBOT_COMMAND_GENERATED"
E_TRAJECTORY_SEND_ATTEMPTED = "E_TRAJECTORY_SEND_ATTEMPTED"
E_REAL_ROBOT_MOTION_ATTEMPTED = "E_REAL_ROBOT_MOTION_ATTEMPTED"
E_ROS2_PUBLISH_ATTEMPTED = "E_ROS2_PUBLISH_ATTEMPTED"
E_RTDE_WRITE_ATTEMPTED = "E_RTDE_WRITE_ATTEMPTED"
E_DASHBOARD_COMMAND_ATTEMPTED = "E_DASHBOARD_COMMAND_ATTEMPTED"

ROBOT_SYSTEM_SHADOW_BRIDGE_FIELDS = (
    "robot_system_shadow_bridge_requested",
    "robot_system_shadow_bridge_status",
    "robot_system_shadow_status",
    "robot_system_shadow_ready",
    "ros2_message_export_ready",
    "moveit_plan_only_ready",
    "ur5_read_only_state_ready",
    "execution_allowed",
    "ros2_publish_enabled",
    "ros2_publish_attempted",
    "moveit_execute_allowed",
    "moveit_execute_called",
    "trajectory_generated",
    "trajectory_send_allowed",
    "trajectory_sent",
    "controller_command_sent",
    "tcp_pose_world_generated",
    "joint_targets_generated",
    "robot_command_generated",
    "real_robot_enabled",
    "real_robot_motion_executed",
    "automatic_retry_motion",
    "blocking_reasons",
    "warnings",
)


@dataclass(frozen=True)
class RobotSystemShadowBridgeRequest:
    requested: bool = False
    config_path: str | None = None
    config: Dict[str, Any] | None = None
    ros2_message_export_result: Dict[str, Any] | None = None
    moveit_plan_only_result: Dict[str, Any] | None = None
    ur5_read_only_state_result: Dict[str, Any] | None = None


def load_robot_system_shadow_bridge_config(path: str | Path | None) -> Dict[str, Any]:
    return _load_config(path, "robot_system_shadow_bridge")


def build_robot_system_shadow_bridge_request(
    *,
    requested: bool = False,
    config_path: str | Path | None = None,
    config: Dict[str, Any] | None = None,
    ros2_message_export_result: Dict[str, Any] | None = None,
    moveit_plan_only_result: Dict[str, Any] | None = None,
    ur5_read_only_state_result: Dict[str, Any] | None = None,
) -> RobotSystemShadowBridgeRequest:
    loaded_config = config if isinstance(config, dict) else load_robot_system_shadow_bridge_config(config_path)
    return RobotSystemShadowBridgeRequest(
        requested=requested,
        config_path=str(Path(config_path).expanduser()) if config_path else None,
        config=loaded_config,
        ros2_message_export_result=ros2_message_export_result if isinstance(ros2_message_export_result, dict) else None,
        moveit_plan_only_result=moveit_plan_only_result if isinstance(moveit_plan_only_result, dict) else None,
        ur5_read_only_state_result=ur5_read_only_state_result if isinstance(ur5_read_only_state_result, dict) else None,
    )


def evaluate_robot_system_shadow_bridge(
    request: RobotSystemShadowBridgeRequest | None = None,
) -> Dict[str, Any]:
    request = request or RobotSystemShadowBridgeRequest()
    if not request.requested:
        return _not_requested_result()

    config = request.config if isinstance(request.config, dict) else {}
    message_export = _source_result(request.ros2_message_export_result, config, "ros2_message_export")
    moveit = _source_result(request.moveit_plan_only_result, config, "moveit_plan_only")
    ur5_state = _source_result(request.ur5_read_only_state_result, config, "ur5_read_only_state")

    ros2_message_export_ready = (
        message_export.get("ros2_message_export_status") == STATUS_PASS
        and message_export.get("message_export_status") == MESSAGE_EXPORTED
    )
    moveit_plan_only_ready = (
        moveit.get("moveit_plan_only_status") == STATUS_PASS
        and moveit.get("plan_only_status") == PLAN_ONLY_READY
        and moveit.get("plan_only_ready") is True
    )
    ur5_read_only_state_ready = (
        ur5_state.get("ur5_read_only_state_status") == STATUS_PASS
        and ur5_state.get("read_only_state_status") == READ_ONLY_STATE_CONTRACT_READY
        and ur5_state.get("read_only_state_contract_ready") is True
    )

    blocking_reasons: list[str] = []
    warnings: list[str] = _string_list(config.get("warnings"))
    if not ros2_message_export_ready:
        blocking_reasons.append(E_ROS2_MESSAGE_EXPORT_NOT_READY)
    if config.get("require_moveit_plan_only_ready", True) is True and not moveit_plan_only_ready:
        blocking_reasons.append(E_MOVEIT_PLAN_ONLY_NOT_READY)
    if config.get("require_ur5_read_only_contract_ready", True) is True and not ur5_read_only_state_ready:
        blocking_reasons.append(E_UR5_READ_ONLY_STATE_NOT_READY)

    if _flag(config, "execution_allowed") or _flag(message_export, "execution_allowed") or _flag(moveit, "execution_allowed"):
        blocking_reasons.append(E_EXECUTION_NOT_ALLOWED_VIOLATED)
    if _flag(config, "moveit_execute_called") or _flag(moveit, "moveit_execute_called"):
        blocking_reasons.append(E_EXECUTION_NOT_ALLOWED_VIOLATED)
    if _flag(config, "trajectory_sent") or _flag(moveit, "trajectory_sent"):
        blocking_reasons.append(E_TRAJECTORY_SEND_ATTEMPTED)
    if _flag(config, "robot_command_generated") or _flag(message_export, "robot_command_generated") or _flag(moveit, "robot_command_generated"):
        blocking_reasons.append(E_ROBOT_COMMAND_GENERATED)
    if _flag(config, "real_robot_motion_executed") or _flag(message_export, "real_robot_motion_executed") or _flag(ur5_state, "real_robot_motion_executed"):
        blocking_reasons.append(E_REAL_ROBOT_MOTION_ATTEMPTED)
    if _flag(config, "ros2_publish_attempted") or _flag(message_export, "ros2_publish_attempted"):
        blocking_reasons.append(E_ROS2_PUBLISH_ATTEMPTED)
    if _flag(config, "rtde_write_attempted") or _flag(ur5_state, "rtde_write_attempted"):
        blocking_reasons.append(E_RTDE_WRITE_ATTEMPTED)
    if _flag(config, "dashboard_command_attempted") or _flag(ur5_state, "dashboard_command_attempted"):
        blocking_reasons.append(E_DASHBOARD_COMMAND_ATTEMPTED)

    warnings = _unique(
        warnings
        + _string_list(message_export.get("warnings"))
        + _string_list(moveit.get("warnings"))
        + _string_list(ur5_state.get("warnings"))
    )
    blocking_reasons = _unique(blocking_reasons)
    status = STATUS_PASS if not blocking_reasons else STATUS_BLOCKED
    shadow_status = ROBOT_SYSTEM_SHADOW_READY if status == STATUS_PASS else STATUS_BLOCKED

    return {
        "contract_version": CONTRACT_VERSION,
        "schema_version": CONTRACT_VERSION,
        "teto_version": CURRENT_ROBOT_SYSTEM_SHADOW_BRIDGE_VERSION,
        "robot_system_shadow_bridge_requested": True,
        "requested": True,
        "config_path": request.config_path,
        "robot_system_shadow_bridge_status": status,
        "robot_system_shadow_status": shadow_status,
        "robot_system_shadow_ready": status == STATUS_PASS,
        "shadow_bridge_only": config.get("shadow_bridge_only", True) is True,
        "ros2_message_export_ready": ros2_message_export_ready,
        "moveit_plan_only_ready": moveit_plan_only_ready,
        "ur5_read_only_state_ready": ur5_read_only_state_ready,
        "message_id": message_export.get("message_id"),
        "planning_group": moveit.get("planning_group"),
        "robot_model": ur5_state.get("robot_model"),
        "manual_confirmation_required": True,
        "execution_allowed": False,
        "ros2_publish_enabled": False,
        "ros2_publish_attempted": False,
        "moveit_execute_allowed": False,
        "moveit_execute_called": False,
        "trajectory_generated": False,
        "trajectory_send_allowed": False,
        "trajectory_sent": False,
        "controller_command_sent": False,
        "tcp_pose_world_generated": False,
        "joint_targets_generated": False,
        "robot_command_generated": False,
        "real_robot_enabled": False,
        "real_robot_motion_executed": False,
        "rtde_write_enabled": False,
        "rtde_write_attempted": False,
        "dashboard_command_enabled": False,
        "dashboard_command_attempted": False,
        "urscript_generated": False,
        "automatic_retry_motion": False,
        "blocking_reasons": blocking_reasons,
        "warnings": warnings,
        "next_safe_action": _next_safe_action(status),
        "ros2_message_export_result": message_export,
        "moveit_plan_only_result": moveit,
        "ur5_read_only_state_result": ur5_state,
        "safety_boundary": _safety_boundary(),
    }


def format_robot_system_shadow_bridge_report(result: Dict[str, Any]) -> str:
    return "\n".join(
        [
            "# TETO V2.11.0 Full Robot-System Shadow Bridge Report",
            "",
            "## Overall Status",
            "",
            f"- robot_system_shadow_bridge_status: {_format_value(result.get('robot_system_shadow_bridge_status'))}",
            f"- robot_system_shadow_status: {_format_value(result.get('robot_system_shadow_status'))}",
            f"- robot_system_shadow_ready: {_format_value(result.get('robot_system_shadow_ready'))}",
            f"- ros2_message_export_ready: {_format_value(result.get('ros2_message_export_ready'))}",
            f"- moveit_plan_only_ready: {_format_value(result.get('moveit_plan_only_ready'))}",
            f"- ur5_read_only_state_ready: {_format_value(result.get('ur5_read_only_state_ready'))}",
            f"- blocking_reasons: {_format_value(result.get('blocking_reasons'))}",
            f"- warnings: {_format_value(result.get('warnings'))}",
            "",
            "## Bridge Fields",
            "",
            "| Field | Value |",
            "| --- | --- |",
            *[f"| {field} | {_format_value(result.get(field))} |" for field in ROBOT_SYSTEM_SHADOW_BRIDGE_FIELDS],
            "",
            "## No-Motion Safety Boundary",
            "",
            "V2.11.0 carries the ROS2-compatible planner request to the robot-system shadow boundary only. It does not publish ROS2 commands, execute MoveIt plans, send controller trajectories, issue UR driver/RTDE/Dashboard/URScript commands, or move a real UR5.",
            "",
            "| Safety Flag | Value |",
            "| --- | --- |",
            *[
                f"| {key} | {_format_value(value)} |"
                for key, value in sorted((result.get("safety_boundary") or {}).items())
            ],
            "",
        ]
    )


def _not_requested_result() -> Dict[str, Any]:
    return {
        "contract_version": CONTRACT_VERSION,
        "schema_version": CONTRACT_VERSION,
        "teto_version": CURRENT_ROBOT_SYSTEM_SHADOW_BRIDGE_VERSION,
        "robot_system_shadow_bridge_requested": False,
        "requested": False,
        "robot_system_shadow_bridge_status": STATUS_NOT_REQUESTED,
        "robot_system_shadow_status": STATUS_NOT_REQUESTED,
        "robot_system_shadow_ready": False,
        "ros2_message_export_ready": False,
        "moveit_plan_only_ready": False,
        "ur5_read_only_state_ready": False,
        "execution_allowed": False,
        "ros2_publish_enabled": False,
        "ros2_publish_attempted": False,
        "moveit_execute_allowed": False,
        "moveit_execute_called": False,
        "trajectory_generated": False,
        "trajectory_send_allowed": False,
        "trajectory_sent": False,
        "controller_command_sent": False,
        "tcp_pose_world_generated": False,
        "joint_targets_generated": False,
        "robot_command_generated": False,
        "real_robot_enabled": False,
        "real_robot_motion_executed": False,
        "automatic_retry_motion": False,
        "blocking_reasons": [],
        "warnings": [],
        "next_safe_action": _next_safe_action(STATUS_NOT_REQUESTED),
        "safety_boundary": _safety_boundary(),
    }


def _source_result(source: Dict[str, Any] | None, config: Dict[str, Any], root_key: str) -> Dict[str, Any]:
    if isinstance(source, dict) and source:
        return _unwrap(source, root_key)
    embedded = config.get(f"{root_key}_result")
    if isinstance(embedded, dict):
        return _unwrap(embedded, root_key)
    return _load_result(config.get(f"{root_key}_result_path"), root_key)


def _flag(source: Dict[str, Any], key: str) -> bool:
    return source.get(key, False) is True


def _next_safe_action(status: str) -> str:
    if status == STATUS_PASS:
        return "Robot-system shadow bridge evidence is ready for review; do not execute."
    if status == STATUS_BLOCKED:
        return "Fix shadow bridge inputs or closed-boundary declarations before rehearsal evidence is accepted."
    return "Request robot-system shadow bridge check with message export, plan-only, and read-only state evidence."


def _safety_boundary() -> Dict[str, Any]:
    return {
        "execution_allowed": False,
        "ros2_publish_enabled": False,
        "ros2_publish_attempted": False,
        "moveit_execute_allowed": False,
        "moveit_execute_called": False,
        "trajectory_generated": False,
        "trajectory_send_allowed": False,
        "trajectory_sent": False,
        "controller_command_sent": False,
        "tcp_pose_world_generated": False,
        "joint_targets_generated": False,
        "robot_command_generated": False,
        "real_robot_enabled": False,
        "real_robot_motion_executed": False,
        "rtde_write_enabled": False,
        "rtde_write_attempted": False,
        "dashboard_command_enabled": False,
        "dashboard_command_attempted": False,
        "urscript_generated": False,
        "automatic_retry_motion": False,
    }


def _load_config(path: Any, root_key: str) -> Dict[str, Any]:
    if not path:
        return {}
    resolved_path = Path(str(path)).expanduser()
    if not resolved_path.is_file():
        return {}
    with resolved_path.open("r", encoding="utf-8") as config_file:
        data = json.load(config_file) if resolved_path.suffix.lower() == ".json" else yaml.safe_load(config_file)
    if not isinstance(data, dict):
        return {}
    config = data.get(root_key)
    return config if isinstance(config, dict) else data


def _load_result(path: Any, root_key: str) -> Dict[str, Any]:
    return _unwrap(_load_config(path, root_key), root_key)


def _unwrap(data: Any, root_key: str) -> Dict[str, Any]:
    if not isinstance(data, dict):
        return {}
    nested = data.get(root_key)
    return nested if isinstance(nested, dict) else data


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
