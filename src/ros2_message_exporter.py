from __future__ import annotations

import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict

import yaml


CONTRACT_VERSION = "teto_ros2_message_export.v1"
CURRENT_ROS2_MESSAGE_EXPORT_VERSION = "TETO V2.10.2"

STATUS_NOT_REQUESTED = "NOT_REQUESTED"
STATUS_PASS = "PASS"
STATUS_BLOCKED = "BLOCKED"
MESSAGE_EXPORTED = "MESSAGE_EXPORTED"
PLANNER_INPUT_READY = "PLANNER_INPUT_READY"
ROS2_INTERFACE_READY = "READY_FOR_SHADOW_BRIDGE"

DEFAULT_MESSAGE_SCHEMA = "teto_planner_gateway/PlannerRequest.v1"
DEFAULT_TTL_MS = 500

E_PLANNER_INPUT_MISSING = "E_PLANNER_INPUT_MISSING"
E_PLANNER_INPUT_NOT_READY = "E_PLANNER_INPUT_NOT_READY"
E_BOUNDED_TARGET_POINT_MISSING = "E_BOUNDED_TARGET_POINT_MISSING"
E_INVALID_BOUNDED_TARGET_POINT = "E_INVALID_BOUNDED_TARGET_POINT"
E_ROS2_INTERFACE_NOT_READY = "E_ROS2_INTERFACE_NOT_READY"
E_MESSAGE_SCHEMA_MISSING = "E_MESSAGE_SCHEMA_MISSING"
E_FRAME_DECLARATION_MISSING = "E_FRAME_DECLARATION_MISSING"
E_ROS2_PUBLISH_NOT_ALLOWED = "E_ROS2_PUBLISH_NOT_ALLOWED"
E_FAKE_PUBLISH_ONLY_REQUIRED = "E_FAKE_PUBLISH_ONLY_REQUIRED"
E_EXECUTION_NOT_ALLOWED_IN_FAKE_PUBLISH = "E_EXECUTION_NOT_ALLOWED_IN_FAKE_PUBLISH"
E_MOVEIT_NOT_ALLOWED_IN_FAKE_PUBLISH = "E_MOVEIT_NOT_ALLOWED_IN_FAKE_PUBLISH"
E_REAL_ROBOT_NOT_ALLOWED = "E_REAL_ROBOT_NOT_ALLOWED"
E_ROBOT_COMMAND_NOT_ALLOWED = "E_ROBOT_COMMAND_NOT_ALLOWED"

FORBIDDEN_GENERATED_FLAGS = (
    "trajectory_generated",
    "tcp_pose_world_generated",
    "joint_targets_generated",
    "robot_command_generated",
    "real_robot_motion_executed",
)

ROS2_MESSAGE_EXPORT_FIELDS = (
    "ros2_message_export_requested",
    "ros2_message_export_status",
    "message_export_status",
    "message_id",
    "message_schema",
    "fake_publish_only",
    "ros2_publish_enabled",
    "ros2_publish_attempted",
    "planner_gateway_interface_mode",
    "planner_gateway_endpoint",
    "bounded_target_point_m",
    "world_frame",
    "robot_base_frame",
    "camera_frame",
    "execution_allowed",
    "moveit_called",
    "trajectory_generated",
    "tcp_pose_world_generated",
    "joint_targets_generated",
    "robot_command_generated",
    "real_robot_motion_executed",
    "blocking_reasons",
    "warnings",
)


@dataclass(frozen=True)
class ROS2MessageExportRequest:
    requested: bool = False
    config_path: str | None = None
    config: Dict[str, Any] | None = None
    planner_gateway_shadow_result: Dict[str, Any] | None = None
    ros2_interface_readiness_result: Dict[str, Any] | None = None


def load_ros2_message_export_config(path: str | Path | None) -> Dict[str, Any]:
    return _load_config(path, "ros2_message_export")


def build_ros2_message_export_request(
    *,
    requested: bool = False,
    config_path: str | Path | None = None,
    config: Dict[str, Any] | None = None,
    planner_gateway_shadow_result: Dict[str, Any] | None = None,
    ros2_interface_readiness_result: Dict[str, Any] | None = None,
) -> ROS2MessageExportRequest:
    loaded_config = config if isinstance(config, dict) else load_ros2_message_export_config(config_path)
    return ROS2MessageExportRequest(
        requested=requested,
        config_path=str(Path(config_path).expanduser()) if config_path else None,
        config=loaded_config,
        planner_gateway_shadow_result=planner_gateway_shadow_result
        if isinstance(planner_gateway_shadow_result, dict)
        else None,
        ros2_interface_readiness_result=ros2_interface_readiness_result
        if isinstance(ros2_interface_readiness_result, dict)
        else None,
    )


def evaluate_ros2_message_export(request: ROS2MessageExportRequest | None = None) -> Dict[str, Any]:
    request = request or ROS2MessageExportRequest()
    if not request.requested:
        return _not_requested_result()

    config = request.config if isinstance(request.config, dict) else {}
    planner = _planner_result(request, config)
    readiness = _readiness_result(request, config)

    fake_publish_only_requested = config.get("fake_publish_only", True) is True
    ros2_publish_requested = config.get("ros2_publish_enabled", False) is True
    ros2_publish_attempted_requested = config.get("ros2_publish_attempted", False) is True
    execution_allowed_requested = config.get("execution_allowed", False) is True
    moveit_enabled_requested = config.get("moveit_enabled", False) is True
    real_robot_enabled_requested = config.get("real_robot_enabled", False) is True

    message_schema = _string(config.get("message_schema")) or _string(readiness.get("message_schema"))
    planner_status = _string(planner.get("planner_input_status"))
    planner_input_ready = planner_status == PLANNER_INPUT_READY and planner.get("planner_input_ready") is True
    readiness_status = _string(readiness.get("ros2_interface_readiness_status"))
    bounded_target_point_m = planner.get("bounded_target_point_m")
    world_frame = _string(planner.get("world_frame") or readiness.get("world_frame"))
    robot_base_frame = _string(readiness.get("robot_base_frame") or config.get("robot_base_frame"))
    camera_frame = _string(readiness.get("camera_frame") or planner.get("camera_frame") or config.get("camera_frame"))

    blocking_reasons: list[str] = []
    warnings: list[str] = _string_list(config.get("warnings"))

    if not planner:
        blocking_reasons.append(E_PLANNER_INPUT_MISSING)
    elif not planner_input_ready:
        blocking_reasons.append(E_PLANNER_INPUT_NOT_READY)
    if bounded_target_point_m is None:
        blocking_reasons.append(E_BOUNDED_TARGET_POINT_MISSING)
    elif not _valid_point3(bounded_target_point_m):
        blocking_reasons.append(E_INVALID_BOUNDED_TARGET_POINT)
    if readiness_status != ROS2_INTERFACE_READY:
        blocking_reasons.append(E_ROS2_INTERFACE_NOT_READY)
    if not message_schema:
        blocking_reasons.append(E_MESSAGE_SCHEMA_MISSING)
    if not (world_frame and robot_base_frame and camera_frame):
        blocking_reasons.append(E_FRAME_DECLARATION_MISSING)
    if ros2_publish_requested or ros2_publish_attempted_requested:
        blocking_reasons.append(E_ROS2_PUBLISH_NOT_ALLOWED)
    if not fake_publish_only_requested:
        blocking_reasons.append(E_FAKE_PUBLISH_ONLY_REQUIRED)
    if execution_allowed_requested:
        blocking_reasons.append(E_EXECUTION_NOT_ALLOWED_IN_FAKE_PUBLISH)
    if moveit_enabled_requested:
        blocking_reasons.append(E_MOVEIT_NOT_ALLOWED_IN_FAKE_PUBLISH)
    if real_robot_enabled_requested:
        blocking_reasons.append(E_REAL_ROBOT_NOT_ALLOWED)

    forbidden_generated_flags = [
        flag_name
        for flag_name in FORBIDDEN_GENERATED_FLAGS
        if planner.get(flag_name) is True or config.get(flag_name) is True or readiness.get(flag_name) is True
    ]
    if forbidden_generated_flags:
        blocking_reasons.append(E_ROBOT_COMMAND_NOT_ALLOWED)
        warnings.append(f"forbidden_generated_flags={forbidden_generated_flags}")

    blocking_reasons = _unique(blocking_reasons)
    warnings = _unique(warnings + _string_list(planner.get("warnings")) + _string_list(readiness.get("warnings")))
    status = STATUS_PASS if not blocking_reasons else STATUS_BLOCKED
    message_export_status = MESSAGE_EXPORTED if status == STATUS_PASS else STATUS_BLOCKED

    message = None
    message_id = _message_id(planner, config)
    if status == STATUS_PASS:
        message = _build_message(
            planner=planner,
            readiness=readiness,
            config=config,
            message_id=message_id,
            message_schema=message_schema,
            world_frame=world_frame,
            robot_base_frame=robot_base_frame,
            camera_frame=camera_frame,
            bounded_target_point_m=bounded_target_point_m,
        )

    return {
        "contract_version": CONTRACT_VERSION,
        "schema_version": CONTRACT_VERSION,
        "teto_version": CURRENT_ROS2_MESSAGE_EXPORT_VERSION,
        "ros2_message_export_requested": True,
        "requested": True,
        "config_path": request.config_path,
        "ros2_message_export_status": status,
        "message_export_status": message_export_status,
        "message_id": message_id if status == STATUS_PASS else None,
        "message_schema": message_schema,
        "fake_publish_only": True,
        "ros2_publish_enabled": False,
        "ros2_publish_attempted": False,
        "planner_gateway_interface_mode": readiness.get("planner_gateway_interface_mode"),
        "planner_gateway_endpoint": readiness.get("planner_gateway_endpoint"),
        "bounded_target_point_m": _round_point(bounded_target_point_m) if _valid_point3(bounded_target_point_m) else bounded_target_point_m,
        "world_frame": world_frame,
        "robot_base_frame": robot_base_frame,
        "camera_frame": camera_frame,
        "execution_allowed": False,
        "moveit_enabled": False,
        "moveit_called": False,
        "trajectory_generated": False,
        "tcp_pose_world_generated": False,
        "joint_targets_generated": False,
        "robot_command_generated": False,
        "real_robot_motion_executed": False,
        "blocking_reasons": blocking_reasons,
        "warnings": warnings,
        "next_safe_action": _next_safe_action(status),
        "planner_gateway_shadow_result": planner,
        "ros2_interface_readiness_result": readiness,
        "exported_message": message,
        "safety_boundary": _safety_boundary(),
    }


def format_ros2_message_export_report(result: Dict[str, Any]) -> str:
    return "\n".join(
        [
            "# TETO V2.10.2 ROS2 Message Export / Fake Publish Report",
            "",
            "## Overall Status",
            "",
            f"- ros2_message_export_status: {_format_value(result.get('ros2_message_export_status'))}",
            f"- message_export_status: {_format_value(result.get('message_export_status'))}",
            f"- message_id: {_format_value(result.get('message_id'))}",
            f"- message_schema: {_format_value(result.get('message_schema'))}",
            f"- fake_publish_only: {_format_value(result.get('fake_publish_only'))}",
            f"- planner_gateway_interface_mode: {_format_value(result.get('planner_gateway_interface_mode'))}",
            f"- planner_gateway_endpoint: {_format_value(result.get('planner_gateway_endpoint'))}",
            f"- blocking_reasons: {_format_value(result.get('blocking_reasons'))}",
            f"- warnings: {_format_value(result.get('warnings'))}",
            f"- next_safe_action: {_format_value(result.get('next_safe_action'))}",
            "",
            "## Message Export Fields",
            "",
            "| Field | Value |",
            "| --- | --- |",
            *[f"| {field} | {_format_value(result.get(field))} |" for field in ROS2_MESSAGE_EXPORT_FIELDS],
            "",
            "## Exported Message",
            "",
            "This deterministic JSON artifact represents what would be sent to a future Planner Gateway. It is not published.",
            "",
            "```json",
            json.dumps(result.get("exported_message"), ensure_ascii=False, indent=2, sort_keys=True),
            "```",
            "",
            "## No-Motion Safety Boundary",
            "",
            "V2.10.2 exports a ROS2-compatible PlannerRequest artifact for future ROS2 Planner Gateway integration only. It does not publish ROS2 messages, does not call rclpy publish, does not call MoveIt, does not generate trajectory or tcp_pose_world, does not create joint targets or robot commands, and does not control a real UR5.",
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


def _planner_result(request: ROS2MessageExportRequest, config: Dict[str, Any]) -> Dict[str, Any]:
    if isinstance(request.planner_gateway_shadow_result, dict) and request.planner_gateway_shadow_result:
        return _unwrap(request.planner_gateway_shadow_result, "planner_gateway_shadow")
    if isinstance(config.get("planner_gateway_shadow_result"), dict):
        return _unwrap(config["planner_gateway_shadow_result"], "planner_gateway_shadow")
    return _load_result(config.get("planner_gateway_shadow_result_path"), "planner_gateway_shadow")


def _readiness_result(request: ROS2MessageExportRequest, config: Dict[str, Any]) -> Dict[str, Any]:
    if isinstance(request.ros2_interface_readiness_result, dict) and request.ros2_interface_readiness_result:
        return _unwrap(request.ros2_interface_readiness_result, "ros2_interface_readiness")
    if isinstance(config.get("ros2_interface_readiness_result"), dict):
        return _unwrap(config["ros2_interface_readiness_result"], "ros2_interface_readiness")
    return _load_result(config.get("ros2_interface_readiness_result_path"), "ros2_interface_readiness")


def _build_message(
    *,
    planner: Dict[str, Any],
    readiness: Dict[str, Any],
    config: Dict[str, Any],
    message_id: str,
    message_schema: str,
    world_frame: str,
    robot_base_frame: str,
    camera_frame: str,
    bounded_target_point_m: Any,
) -> Dict[str, Any]:
    perception = planner.get("perception_shadow_result") if isinstance(planner.get("perception_shadow_result"), dict) else {}
    return {
        "schema": message_schema,
        "message_id": message_id,
        "task_id": planner.get("task_id"),
        "scene_version": planner.get("scene_version"),
        "intent_name": planner.get("intent_name"),
        "target_label": planner.get("target_label"),
        "target_object_id": planner.get("target_object_id"),
        "world_frame": world_frame,
        "robot_base_frame": robot_base_frame,
        "camera_frame": camera_frame,
        "world_point_m": _round_point(planner.get("world_point_m")),
        "bounded_target_point_m": _round_point(bounded_target_point_m),
        "hover_offset_m": planner.get("hover_offset_m"),
        "confidence_overall": planner.get("overall_confidence") or perception.get("overall_confidence"),
        "ttl_ms": config.get("ttl_ms", perception.get("ttl_ms", DEFAULT_TTL_MS)),
        "manual_confirmation_required": planner.get("manual_confirmation_required", True) is True,
        "execution_allowed": False,
        "fake_publish_only": True,
        "ros2_publish_enabled": False,
        "ros2_publish_attempted": False,
        "planner_gateway_interface_mode": readiness.get("planner_gateway_interface_mode"),
        "planner_gateway_endpoint": readiness.get("planner_gateway_endpoint"),
        "created_from_version": CURRENT_ROS2_MESSAGE_EXPORT_VERSION,
    }


def _not_requested_result() -> Dict[str, Any]:
    return {
        "contract_version": CONTRACT_VERSION,
        "schema_version": CONTRACT_VERSION,
        "teto_version": CURRENT_ROS2_MESSAGE_EXPORT_VERSION,
        "ros2_message_export_requested": False,
        "requested": False,
        "ros2_message_export_status": STATUS_NOT_REQUESTED,
        "message_export_status": STATUS_NOT_REQUESTED,
        "message_id": None,
        "message_schema": None,
        "fake_publish_only": True,
        "ros2_publish_enabled": False,
        "ros2_publish_attempted": False,
        "execution_allowed": False,
        "moveit_enabled": False,
        "moveit_called": False,
        "trajectory_generated": False,
        "tcp_pose_world_generated": False,
        "joint_targets_generated": False,
        "robot_command_generated": False,
        "real_robot_motion_executed": False,
        "blocking_reasons": [],
        "warnings": [],
        "next_safe_action": _next_safe_action(STATUS_NOT_REQUESTED),
        "exported_message": None,
        "safety_boundary": _safety_boundary(),
    }


def _next_safe_action(status: str) -> str:
    if status == STATUS_PASS:
        return "Review the fake-publish JSON artifact; do not publish or execute."
    if status == STATUS_BLOCKED:
        return "Fix planner input, ROS2 readiness, or fake-publish safety declarations before exporting."
    return "Request ROS2 message export with planner shadow and ROS2 readiness evidence."


def _safety_boundary() -> Dict[str, Any]:
    return {
        "fake_publish_only": True,
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


def _message_id(planner: Dict[str, Any], config: Dict[str, Any]) -> str:
    configured = _string(config.get("message_id"))
    if configured:
        return configured
    task_id = _string(planner.get("task_id")) or "planner_request"
    return f"ros2_fake_publish_{task_id}"


def _load_config(path: Any, root_key: str) -> Dict[str, Any]:
    data = _load_yaml_or_json(path)
    if not isinstance(data, dict):
        return {}
    config = data.get(root_key)
    return config if isinstance(config, dict) else data


def _load_result(path: Any, root_key: str) -> Dict[str, Any]:
    data = _load_yaml_or_json(path)
    return _unwrap(data, root_key)


def _load_yaml_or_json(path: Any) -> Dict[str, Any]:
    if not path:
        return {}
    resolved_path = Path(str(path)).expanduser()
    if not resolved_path.is_file():
        return {}
    with resolved_path.open("r", encoding="utf-8") as result_file:
        if resolved_path.suffix.lower() == ".json":
            data = json.load(result_file)
        else:
            data = yaml.safe_load(result_file)
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
    if isinstance(value, list):
        return [str(item) for item in value if item]
    if isinstance(value, tuple):
        return [str(item) for item in value if item]
    if value:
        return [str(value)]
    return []


def _unique(values: list[str]) -> list[str]:
    seen: set[str] = set()
    output: list[str] = []
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
