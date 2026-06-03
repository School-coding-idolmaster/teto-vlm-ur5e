from __future__ import annotations

import json
import os
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict

import yaml


CONTRACT_VERSION = "teto_ros2_interface_readiness.v1"
CURRENT_ROS2_INTERFACE_READINESS_VERSION = "TETO V2.10.1"

STATUS_NOT_REQUESTED = "NOT_REQUESTED"
STATUS_BLOCKED = "BLOCKED"
STATUS_READY_FOR_SHADOW_BRIDGE = "READY_FOR_SHADOW_BRIDGE"

W_ROS2_RUNTIME_UNAVAILABLE = "W_ROS2_RUNTIME_UNAVAILABLE"
E_ROS2_RUNTIME_UNAVAILABLE = "E_ROS2_RUNTIME_UNAVAILABLE"
E_ROS2_ENVIRONMENT_UNDECLARED = "E_ROS2_ENVIRONMENT_UNDECLARED"
E_ROS_DISTRO_UNDECLARED = "E_ROS_DISTRO_UNDECLARED"
E_ROS_DOMAIN_ID_UNDECLARED = "E_ROS_DOMAIN_ID_UNDECLARED"
E_PLANNER_GATEWAY_INTERFACE_MODE_UNDECLARED = "E_PLANNER_GATEWAY_INTERFACE_MODE_UNDECLARED"
E_PLANNER_GATEWAY_INTERFACE_MODE_UNSUPPORTED = "E_PLANNER_GATEWAY_INTERFACE_MODE_UNSUPPORTED"
E_PLANNER_GATEWAY_ENDPOINT_MISSING = "E_PLANNER_GATEWAY_ENDPOINT_MISSING"
E_MESSAGE_SCHEMA_UNDECLARED = "E_MESSAGE_SCHEMA_UNDECLARED"
E_FRAME_DECLARATION_MISSING = "E_FRAME_DECLARATION_MISSING"
E_SHADOW_ONLY_REQUIRED = "E_SHADOW_ONLY_REQUIRED"
E_ROS2_PUBLISH_NOT_ALLOWED = "E_ROS2_PUBLISH_NOT_ALLOWED"
E_MOVEIT_NOT_ALLOWED_IN_READINESS = "E_MOVEIT_NOT_ALLOWED_IN_READINESS"
E_REAL_ROBOT_NOT_ALLOWED = "E_REAL_ROBOT_NOT_ALLOWED"
E_EXECUTION_NOT_ALLOWED_IN_READINESS = "E_EXECUTION_NOT_ALLOWED_IN_READINESS"

SUPPORTED_ROS_DISTROS = {"humble", "iron", "jazzy", "rolling", "unavailable"}
SUPPORTED_INTERFACE_MODES = {"topic", "service", "action", "json_export_only"}
REQUIRED_FRAMES = ("world_frame", "robot_base_frame", "camera_frame")

ROS2_INTERFACE_READINESS_FIELDS = (
    "ros2_interface_readiness_requested",
    "ros2_interface_readiness_status",
    "ros2_environment_declared",
    "ros_distro",
    "ros_domain_id",
    "planner_gateway_interface_mode",
    "planner_gateway_endpoint",
    "message_schema",
    "world_frame",
    "robot_base_frame",
    "camera_frame",
    "target_frame",
    "shadow_only",
    "ros2_publish_enabled",
    "ros2_publish_attempted",
    "moveit_enabled",
    "moveit_called",
    "real_robot_enabled",
    "execution_allowed",
    "trajectory_generated",
    "tcp_pose_world_generated",
    "joint_targets_generated",
    "robot_command_generated",
    "real_robot_motion_executed",
    "ros2_runtime_available",
    "allow_missing_ros2_runtime",
    "blocking_reasons",
    "warnings",
)


@dataclass(frozen=True)
class ROS2InterfaceReadinessRequest:
    requested: bool = False
    config_path: str | None = None
    config: Dict[str, Any] | None = None
    environ: Dict[str, str] | None = None


def load_ros2_interface_config(path: str | Path | None) -> Dict[str, Any]:
    if not path:
        return {}
    resolved_path = Path(path).expanduser()
    if not resolved_path.is_file():
        return {}
    with resolved_path.open("r", encoding="utf-8") as config_file:
        if resolved_path.suffix.lower() == ".json":
            data = json.load(config_file)
        else:
            data = yaml.safe_load(config_file)
    if not isinstance(data, dict):
        return {}
    config = data.get("ros2_interface_readiness") or data.get("ros2_interface")
    return config if isinstance(config, dict) else data


def build_ros2_interface_readiness_request(
    *,
    requested: bool = False,
    config_path: str | Path | None = None,
    config: Dict[str, Any] | None = None,
    environ: Dict[str, str] | None = None,
) -> ROS2InterfaceReadinessRequest:
    loaded_config = config if isinstance(config, dict) else load_ros2_interface_config(config_path)
    return ROS2InterfaceReadinessRequest(
        requested=requested,
        config_path=str(Path(config_path).expanduser()) if config_path else None,
        config=loaded_config,
        environ=environ,
    )


def evaluate_ros2_interface_readiness(
    request: ROS2InterfaceReadinessRequest | None = None,
) -> Dict[str, Any]:
    request = request or ROS2InterfaceReadinessRequest()
    if not request.requested:
        return _not_requested_result()

    config = request.config if isinstance(request.config, dict) else {}
    env = request.environ if isinstance(request.environ, dict) else os.environ
    ros_distro = _string(config.get("ros_distro"))
    ros_domain_id = _declared_value(config.get("ros_domain_id", config.get("ROS_DOMAIN_ID")))
    interface = config.get("planner_gateway_interface") if isinstance(config.get("planner_gateway_interface"), dict) else {}
    interface_mode = _string(
        config.get("planner_gateway_interface_mode")
        or config.get("interface_mode")
        or interface.get("mode")
    )
    endpoint = _planner_gateway_endpoint(config, interface_mode)
    message_schema = _string(config.get("message_schema") or interface.get("message_schema"))
    frames = config.get("frames") if isinstance(config.get("frames"), dict) else {}

    world_frame = _string(config.get("world_frame") or frames.get("world_frame"))
    robot_base_frame = _string(config.get("robot_base_frame") or frames.get("robot_base_frame"))
    camera_frame = _string(config.get("camera_frame") or frames.get("camera_frame"))
    target_frame = _string(config.get("target_frame") or frames.get("target_frame"))
    missing_frames = [
        frame_name
        for frame_name, frame_value in (
            ("world_frame", world_frame),
            ("robot_base_frame", robot_base_frame),
            ("camera_frame", camera_frame),
        )
        if not frame_value
    ]

    ros2_environment_declared = config.get("ros2_environment_declared") is True
    shadow_only = config.get("shadow_only") is True
    ros2_publish_requested = config.get("ros2_publish_enabled", False) is True
    ros2_publish_attempted_requested = config.get("ros2_publish_attempted", False) is True
    moveit_enabled_requested = config.get("moveit_enabled", False) is True
    moveit_called_requested = config.get("moveit_called", False) is True
    real_robot_enabled_requested = config.get("real_robot_enabled", False) is True
    execution_allowed_requested = config.get("execution_allowed", False) is True
    allow_missing_runtime = config.get("allow_missing_ros2_runtime", True) is True
    require_runtime = config.get("require_ros2_runtime", False) is True
    ros2_runtime_available = _ros2_runtime_available(env)

    blocking_reasons: list[str] = []
    warnings: list[str] = []

    if not ros2_environment_declared:
        blocking_reasons.append(E_ROS2_ENVIRONMENT_UNDECLARED)
    if not ros_distro or ros_distro not in SUPPORTED_ROS_DISTROS:
        blocking_reasons.append(E_ROS_DISTRO_UNDECLARED)
    if ros_domain_id is None:
        blocking_reasons.append(E_ROS_DOMAIN_ID_UNDECLARED)
    if not interface_mode:
        blocking_reasons.append(E_PLANNER_GATEWAY_INTERFACE_MODE_UNDECLARED)
    elif interface_mode not in SUPPORTED_INTERFACE_MODES:
        blocking_reasons.append(E_PLANNER_GATEWAY_INTERFACE_MODE_UNSUPPORTED)
    if not endpoint:
        blocking_reasons.append(E_PLANNER_GATEWAY_ENDPOINT_MISSING)
    if not message_schema:
        blocking_reasons.append(E_MESSAGE_SCHEMA_UNDECLARED)
    if missing_frames:
        blocking_reasons.append(E_FRAME_DECLARATION_MISSING)
    if not shadow_only:
        blocking_reasons.append(E_SHADOW_ONLY_REQUIRED)
    if ros2_publish_requested or ros2_publish_attempted_requested:
        blocking_reasons.append(E_ROS2_PUBLISH_NOT_ALLOWED)
    if moveit_enabled_requested or moveit_called_requested:
        blocking_reasons.append(E_MOVEIT_NOT_ALLOWED_IN_READINESS)
    if real_robot_enabled_requested:
        blocking_reasons.append(E_REAL_ROBOT_NOT_ALLOWED)
    if execution_allowed_requested:
        blocking_reasons.append(E_EXECUTION_NOT_ALLOWED_IN_READINESS)
    if not ros2_runtime_available:
        if require_runtime or not allow_missing_runtime:
            blocking_reasons.append(E_ROS2_RUNTIME_UNAVAILABLE)
        else:
            warnings.append(W_ROS2_RUNTIME_UNAVAILABLE)

    status = STATUS_BLOCKED if blocking_reasons else STATUS_READY_FOR_SHADOW_BRIDGE
    return {
        "contract_version": CONTRACT_VERSION,
        "schema_version": CONTRACT_VERSION,
        "teto_version": CURRENT_ROS2_INTERFACE_READINESS_VERSION,
        "ros2_interface_readiness_requested": True,
        "requested": True,
        "config_path": request.config_path,
        "ros2_interface_readiness_status": status,
        "ros2_environment_declared": ros2_environment_declared,
        "ros_distro": ros_distro,
        "ros_domain_id": ros_domain_id,
        "planner_gateway_interface_mode": interface_mode,
        "planner_gateway_endpoint": endpoint,
        "planner_gateway_endpoint_field": _planner_gateway_endpoint_field(interface_mode),
        "message_schema": message_schema,
        "world_frame": world_frame,
        "robot_base_frame": robot_base_frame,
        "camera_frame": camera_frame,
        "target_frame": target_frame,
        "missing_frames": missing_frames,
        "shadow_only": shadow_only,
        "ros2_publish_enabled": False,
        "ros2_publish_attempted": False,
        "moveit_enabled": False,
        "moveit_called": False,
        "real_robot_enabled": False,
        "execution_allowed": False,
        "trajectory_generated": False,
        "tcp_pose_world_generated": False,
        "joint_targets_generated": False,
        "robot_command_generated": False,
        "real_robot_motion_executed": False,
        "ros2_runtime_available": ros2_runtime_available,
        "allow_missing_ros2_runtime": allow_missing_runtime,
        "require_ros2_runtime": require_runtime,
        "blocking_reasons": blocking_reasons,
        "warnings": warnings,
        "next_safe_action": _next_safe_action(status),
        "safety_boundary": _safety_boundary(),
    }


def format_ros2_interface_readiness_report(result: Dict[str, Any]) -> str:
    return "\n".join(
        [
            "# TETO V2.10.1 ROS2 Interface Readiness Report",
            "",
            "## Overall Status",
            "",
            f"- ros2_interface_readiness_status: {_format_value(result.get('ros2_interface_readiness_status'))}",
            f"- ros2_environment_declared: {_format_value(result.get('ros2_environment_declared'))}",
            f"- ros_distro: {_format_value(result.get('ros_distro'))}",
            f"- ros_domain_id: {_format_value(result.get('ros_domain_id'))}",
            f"- planner_gateway_interface_mode: {_format_value(result.get('planner_gateway_interface_mode'))}",
            f"- planner_gateway_endpoint: {_format_value(result.get('planner_gateway_endpoint'))}",
            f"- message_schema: {_format_value(result.get('message_schema'))}",
            f"- blocking_reasons: {_format_value(result.get('blocking_reasons'))}",
            f"- warnings: {_format_value(result.get('warnings'))}",
            f"- next_safe_action: {_format_value(result.get('next_safe_action'))}",
            "",
            "## Interface Fields",
            "",
            "| Field | Value |",
            "| --- | --- |",
            *[f"| {field} | {_format_value(result.get(field))} |" for field in ROS2_INTERFACE_READINESS_FIELDS],
            "",
            "## No-Motion Safety Boundary",
            "",
            "V2.10.1 checks ROS2 environment and Planner Gateway interface declarations only. It does not publish ROS2 messages and does not publish ROS2 topics, services, or actions; does not call MoveIt; does not connect to a real UR5; does not use RTDE, URScript, Dashboard, or a real robot backend; and does not generate trajectory, tcp_pose_world, joint targets, robot commands, automatic retry motion, or real execution requests.",
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


def _planner_gateway_endpoint(config: Dict[str, Any], interface_mode: str | None) -> str | None:
    interface = config.get("planner_gateway_interface") if isinstance(config.get("planner_gateway_interface"), dict) else {}
    field = _planner_gateway_endpoint_field(interface_mode)
    if field:
        return _string(config.get(field) or interface.get(field))
    for fallback in ("topic_name", "service_name", "action_name", "json_export_path"):
        value = _string(config.get(fallback) or interface.get(fallback))
        if value:
            return value
    return None


def _planner_gateway_endpoint_field(interface_mode: str | None) -> str | None:
    return {
        "topic": "topic_name",
        "service": "service_name",
        "action": "action_name",
        "json_export_only": "json_export_path",
    }.get(interface_mode or "")


def _ros2_runtime_available(env: Dict[str, str]) -> bool:
    return bool(_string(env.get("ROS_DISTRO")) or _string(env.get("AMENT_PREFIX_PATH")))


def _next_safe_action(status: str) -> str:
    if status == STATUS_READY_FOR_SHADOW_BRIDGE:
        return "Ready to use as shadow-only bridge declaration evidence; do not publish or execute."
    if status == STATUS_BLOCKED:
        return "Fix ROS2 interface declarations while keeping shadow_only=true and all publish/motion flags disabled."
    return "Request ROS2 interface readiness check with a config file."


def _not_requested_result() -> Dict[str, Any]:
    return {
        "contract_version": CONTRACT_VERSION,
        "schema_version": CONTRACT_VERSION,
        "teto_version": CURRENT_ROS2_INTERFACE_READINESS_VERSION,
        "ros2_interface_readiness_requested": False,
        "requested": False,
        "ros2_interface_readiness_status": STATUS_NOT_REQUESTED,
        "ros2_environment_declared": False,
        "ros_distro": None,
        "ros_domain_id": None,
        "planner_gateway_interface_mode": None,
        "planner_gateway_endpoint": None,
        "message_schema": None,
        "world_frame": None,
        "robot_base_frame": None,
        "camera_frame": None,
        "target_frame": None,
        "shadow_only": True,
        "ros2_publish_enabled": False,
        "ros2_publish_attempted": False,
        "moveit_enabled": False,
        "moveit_called": False,
        "real_robot_enabled": False,
        "execution_allowed": False,
        "trajectory_generated": False,
        "tcp_pose_world_generated": False,
        "joint_targets_generated": False,
        "robot_command_generated": False,
        "real_robot_motion_executed": False,
        "ros2_runtime_available": False,
        "allow_missing_ros2_runtime": True,
        "blocking_reasons": [],
        "warnings": [],
        "next_safe_action": _next_safe_action(STATUS_NOT_REQUESTED),
        "safety_boundary": _safety_boundary(),
    }


def _safety_boundary() -> Dict[str, bool]:
    return {
        "ros2_publish_enabled": False,
        "ros2_publish_attempted": False,
        "moveit_called": False,
        "trajectory_generated": False,
        "tcp_pose_world_generated": False,
        "joint_targets_generated": False,
        "robot_command_generated": False,
        "real_robot_motion_executed": False,
        "execution_allowed": False,
    }


def _string(value: Any) -> str | None:
    if isinstance(value, str):
        stripped = value.strip()
        return stripped or None
    if value is None:
        return None
    return str(value).strip() or None


def _declared_value(value: Any) -> str | None:
    if value is None:
        return None
    if isinstance(value, str):
        stripped = value.strip()
        return stripped or None
    return str(value)


def _format_value(value: Any) -> str:
    if value is None:
        return "null"
    if isinstance(value, bool):
        return "true" if value else "false"
    if isinstance(value, (list, dict)):
        return json.dumps(value, ensure_ascii=False, sort_keys=True)
    return str(value)
