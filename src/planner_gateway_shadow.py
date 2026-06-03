from __future__ import annotations

import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict

import yaml


CONTRACT_VERSION = "teto_planner_gateway_shadow.v1"
CURRENT_PLANNER_GATEWAY_SHADOW_VERSION = "TETO V2.10.0"

STATUS_PASS = "PASS"
STATUS_BLOCKED = "BLOCKED"
STATUS_NOT_REQUESTED = "NOT_REQUESTED"
PLANNER_INPUT_READY = "PLANNER_INPUT_READY"

DEFAULT_INTENT_NAME = "hover_to_object"
DEFAULT_WORLD_FRAME = "base_link"
DEFAULT_HOVER_OFFSET_M = 0.08
DEFAULT_MAX_SPEED_SCALE = 0.05
DEFAULT_MAX_ACC_SCALE = 0.05
DEFAULT_CONFIDENCE_THRESHOLD = 0.5
DEFAULT_WORKSPACE_BOUNDS = {
    "x": [-1.0, 1.0],
    "y": [-1.0, 1.0],
    "z": [0.0, 2.0],
}

E_PERCEPTION_NOT_READY = "E_PERCEPTION_NOT_READY"
E_WORLD_POINT_MISSING = "E_WORLD_POINT_MISSING"
E_INVALID_WORLD_POINT = "E_INVALID_WORLD_POINT"
E_WORLD_FRAME_MISSING = "E_WORLD_FRAME_MISSING"
E_UNSUPPORTED_INTENT = "E_UNSUPPORTED_INTENT"
E_OUT_OF_WORKSPACE = "E_OUT_OF_WORKSPACE"
E_LOW_CONFIDENCE = "E_LOW_CONFIDENCE"
E_SCENE_VERSION_MISSING = "E_SCENE_VERSION_MISSING"
E_SNAPSHOT_ID_MISSING = "E_SNAPSHOT_ID_MISSING"
E_MANUAL_CONFIRMATION_REQUIRED = "E_MANUAL_CONFIRMATION_REQUIRED"
E_EXECUTION_NOT_ALLOWED_IN_SHADOW = "E_EXECUTION_NOT_ALLOWED_IN_SHADOW"
E_ROS2_PUBLISH_DISABLED = "E_ROS2_PUBLISH_DISABLED"
E_MOVEIT_DISABLED = "E_MOVEIT_DISABLED"
E_ROBOT_COMMAND_NOT_ALLOWED = "E_ROBOT_COMMAND_NOT_ALLOWED"
E_LIVE_CAMERA_DISABLED = "E_LIVE_CAMERA_DISABLED"
E_LIVE_VLM_DISABLED = "E_LIVE_VLM_DISABLED"
E_REAL_ROBOT_MOTION_NOT_ALLOWED = "E_REAL_ROBOT_MOTION_NOT_ALLOWED"

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

PLANNER_GATEWAY_SHADOW_FIELDS = (
    "planner_gateway_shadow_requested",
    "planner_gateway_shadow_status",
    "gateway_request_id",
    "schema_version",
    "task_id",
    "user_command",
    "normalized_command",
    "intent_name",
    "target_label",
    "target_object_id",
    "snapshot_id",
    "grounding_id",
    "scene_version",
    "world_frame",
    "camera_frame",
    "world_point_m",
    "target_point_valid",
    "hover_offset_m",
    "bounded_target_point_m",
    "workspace_check_passed",
    "ttl_check_passed",
    "confidence_check_passed",
    "semantic_gate_passed",
    "geometry_validity_status",
    "projector_status",
    "planner_input_ready",
    "manual_confirmation_required",
    "execution_allowed",
    "ros2_publish_enabled",
    "ros2_publish_attempted",
    "moveit_called",
    "trajectory_generated",
    "tcp_pose_world_generated",
    "joint_targets_generated",
    "robot_command_generated",
    "real_robot_motion_executed",
    "blocking_reasons",
    "warnings",
    "next_safe_action",
    "replay_ready",
)


@dataclass(frozen=True)
class PlannerGatewayShadowRequest:
    requested: bool = False
    config_path: str | None = None
    perception_shadow_result_path: str | None = None
    perception_shadow_result: Dict[str, Any] | None = None
    config: Dict[str, Any] | None = None


def load_planner_gateway_shadow_config(path: str | Path | None) -> Dict[str, Any]:
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
    config = data.get("planner_gateway_shadow") or data.get("planner_gateway_shadow_contract")
    return config if isinstance(config, dict) else data


def load_perception_shadow_result(path: str | Path | None) -> Dict[str, Any]:
    if not path:
        return {}
    resolved_path = Path(path).expanduser()
    if not resolved_path.is_file():
        return {}
    with resolved_path.open("r", encoding="utf-8") as result_file:
        if resolved_path.suffix.lower() == ".json":
            data = json.load(result_file)
        else:
            data = yaml.safe_load(result_file)
    if not isinstance(data, dict):
        return {}
    perception = data.get("perception_shadow")
    return perception if isinstance(perception, dict) else data


def build_planner_gateway_shadow_request(
    *,
    requested: bool = False,
    config_path: str | Path | None = None,
    perception_shadow_result_path: str | Path | None = None,
    perception_shadow_result: Dict[str, Any] | None = None,
) -> PlannerGatewayShadowRequest:
    config = load_planner_gateway_shadow_config(config_path)
    return PlannerGatewayShadowRequest(
        requested=requested,
        config_path=str(Path(config_path).expanduser()) if config_path else None,
        perception_shadow_result_path=str(Path(perception_shadow_result_path).expanduser())
        if perception_shadow_result_path
        else None,
        perception_shadow_result=perception_shadow_result if isinstance(perception_shadow_result, dict) else None,
        config=config,
    )


def evaluate_planner_gateway_shadow(
    request: PlannerGatewayShadowRequest | None = None,
) -> Dict[str, Any]:
    request = request or PlannerGatewayShadowRequest()
    if not request.requested:
        return _not_requested_result()

    config = request.config if isinstance(request.config, dict) else {}
    perception = (
        request.perception_shadow_result
        if isinstance(request.perception_shadow_result, dict)
        else load_perception_shadow_result(request.perception_shadow_result_path)
    )
    if isinstance(config.get("perception_shadow_result"), dict) and not perception:
        perception = config["perception_shadow_result"]
    if isinstance(config.get("perception_result"), dict) and not perception:
        perception = config["perception_result"]

    intent_name = _string(config.get("intent_name")) or DEFAULT_INTENT_NAME
    allowed_intents = _string_list(config.get("allowed_intents")) or [DEFAULT_INTENT_NAME]
    world_frame = _string(config.get("world_frame")) or _string(perception.get("world_frame")) or DEFAULT_WORLD_FRAME
    if config.get("world_frame") is None and not perception.get("world_frame"):
        world_frame = None
    camera_frame = _string(perception.get("camera_frame"))
    confidence_threshold = _optional_float(config.get("confidence_threshold")) or DEFAULT_CONFIDENCE_THRESHOLD
    hover_offset_m = _optional_float(config.get("hover_offset_m"))
    if hover_offset_m is None:
        hover_offset_m = DEFAULT_HOVER_OFFSET_M
    max_speed_scale = _optional_float(config.get("max_speed_scale"))
    if max_speed_scale is None:
        max_speed_scale = DEFAULT_MAX_SPEED_SCALE
    max_acc_scale = _optional_float(config.get("max_acc_scale"))
    if max_acc_scale is None:
        max_acc_scale = DEFAULT_MAX_ACC_SCALE
    manual_confirmation_required = config.get("manual_confirmation_required", True) is True
    execution_allowed = config.get("execution_allowed", False) is True
    ros2_publish_enabled = config.get("ros2_publish_enabled", False) is True
    ros2_publish_attempted = config.get("ros2_publish_attempted", False) is True
    moveit_called = config.get("moveit_called", False) is True or perception.get("moveit_called", False) is True

    blocking_reasons: list[str] = []
    warnings = _list(perception.get("warnings")) + _list(config.get("warnings"))
    perception_status = perception.get("perception_shadow_status")
    if perception_status != STATUS_PASS:
        blocking_reasons.append(E_PERCEPTION_NOT_READY)

    world_point_m = perception.get("world_point_m")
    target_point_valid = True
    if world_point_m is None:
        blocking_reasons.append(E_WORLD_POINT_MISSING)
        target_point_valid = False
    elif not _valid_point3(world_point_m):
        blocking_reasons.append(E_INVALID_WORLD_POINT)
        target_point_valid = False

    if not world_frame:
        blocking_reasons.append(E_WORLD_FRAME_MISSING)
    if intent_name not in allowed_intents:
        blocking_reasons.append(E_UNSUPPORTED_INTENT)
    if not perception.get("scene_version"):
        blocking_reasons.append(E_SCENE_VERSION_MISSING)
    if not perception.get("snapshot_id"):
        blocking_reasons.append(E_SNAPSHOT_ID_MISSING)
    if not manual_confirmation_required:
        blocking_reasons.append(E_MANUAL_CONFIRMATION_REQUIRED)
    if execution_allowed:
        blocking_reasons.append(E_EXECUTION_NOT_ALLOWED_IN_SHADOW)
    if ros2_publish_enabled or ros2_publish_attempted:
        blocking_reasons.append(E_ROS2_PUBLISH_DISABLED)
    if moveit_called:
        blocking_reasons.append(E_MOVEIT_DISABLED)
    if perception.get("live_camera_used") is True:
        blocking_reasons.append(E_LIVE_CAMERA_DISABLED)
    if perception.get("live_vlm_called") is True:
        blocking_reasons.append(E_LIVE_VLM_DISABLED)
    if perception.get("real_robot_motion_executed") is True or config.get("real_robot_motion_executed") is True:
        blocking_reasons.append(E_REAL_ROBOT_MOTION_NOT_ALLOWED)

    overall_confidence = _optional_float(perception.get("overall_confidence"))
    confidence_check_passed = overall_confidence is not None and overall_confidence >= confidence_threshold
    if not confidence_check_passed:
        blocking_reasons.append(E_LOW_CONFIDENCE)

    bounded_target_point_m = _bounded_target_point(world_point_m, hover_offset_m) if target_point_valid else None
    workspace_check_passed = _point_in_workspace(bounded_target_point_m, _workspace_bounds(config))
    if bounded_target_point_m is not None and not workspace_check_passed:
        blocking_reasons.append(E_OUT_OF_WORKSPACE)

    forbidden_fields = _unique(_forbidden_robot_control_fields(config) + _forbidden_robot_control_fields(perception))
    if forbidden_fields:
        blocking_reasons.append(E_ROBOT_COMMAND_NOT_ALLOWED)
        warnings.append(f"forbidden_robot_control_fields={forbidden_fields}")

    blocking_reasons = _unique([str(reason) for reason in blocking_reasons if reason])
    warnings = _unique([str(warning) for warning in warnings if warning])
    status = STATUS_PASS if not blocking_reasons else STATUS_BLOCKED
    planner_input_ready = status == STATUS_PASS

    shadow_request = None
    if planner_input_ready:
        shadow_request = {
            "schema_version": CONTRACT_VERSION,
            "gateway_request_id": _gateway_request_id(perception, config),
            "task_id": _task_id(perception, config),
            "mode": "shadow_only",
            "intent_name": intent_name,
            "user_command": perception.get("user_command"),
            "normalized_command": perception.get("normalized_command"),
            "target_label": perception.get("target_label"),
            "target_object_id": perception.get("target_object_id"),
            "snapshot_id": perception.get("snapshot_id"),
            "grounding_id": perception.get("grounding_id"),
            "scene_version": perception.get("scene_version"),
            "world_frame": world_frame,
            "camera_frame": camera_frame,
            "target_world_point_m": _round_point(world_point_m),
            "hover_offset_m": hover_offset_m,
            "bounded_target_point_m": bounded_target_point_m,
            "max_speed_scale": max_speed_scale,
            "max_acc_scale": max_acc_scale,
            "must_reconfirm": True,
            "manual_confirmation_required": True,
            "execution_allowed": False,
            "ros2_publish_enabled": False,
            "planner_input_ready": True,
        }

    return {
        "contract_version": CONTRACT_VERSION,
        "schema_version": CONTRACT_VERSION,
        "teto_version": CURRENT_PLANNER_GATEWAY_SHADOW_VERSION,
        "planner_gateway_shadow_requested": True,
        "requested": True,
        "config_path": request.config_path,
        "perception_shadow_result_path": request.perception_shadow_result_path,
        "planner_gateway_shadow_status": status,
        "planner_input_status": PLANNER_INPUT_READY if planner_input_ready else STATUS_BLOCKED,
        "gateway_request_id": _gateway_request_id(perception, config),
        "task_id": _task_id(perception, config),
        "user_command": perception.get("user_command"),
        "normalized_command": perception.get("normalized_command"),
        "intent_name": intent_name,
        "target_label": perception.get("target_label"),
        "target_object_id": perception.get("target_object_id"),
        "snapshot_id": perception.get("snapshot_id"),
        "grounding_id": perception.get("grounding_id"),
        "scene_version": perception.get("scene_version"),
        "world_frame": world_frame,
        "camera_frame": camera_frame,
        "world_point_m": _round_point(world_point_m) if target_point_valid else world_point_m,
        "target_world_point_m": _round_point(world_point_m) if target_point_valid else world_point_m,
        "target_point_valid": target_point_valid,
        "hover_offset_m": hover_offset_m,
        "bounded_target_point_m": bounded_target_point_m,
        "max_speed_scale": max_speed_scale,
        "max_acc_scale": max_acc_scale,
        "must_reconfirm": True,
        "ttl_check_passed": _ttl_check_passed(perception),
        "workspace_check_passed": workspace_check_passed,
        "confidence_check_passed": confidence_check_passed,
        "semantic_gate_passed": perception.get("semantic_gate_passed") is True,
        "geometry_validity_status": perception.get("geometry_validity_status"),
        "projector_status": perception.get("projector_status"),
        "planner_input_ready": planner_input_ready,
        "manual_confirmation_required": manual_confirmation_required,
        "execution_allowed": False,
        "ros2_publish_enabled": False,
        "ros2_publish_attempted": False,
        "moveit_called": False,
        "live_camera_used": False,
        "live_vlm_called": False,
        "trajectory_generated": False,
        "tcp_pose_world_generated": False,
        "joint_targets_generated": False,
        "robot_command_generated": False,
        "real_robot_motion_executed": False,
        "blocking_reasons": blocking_reasons,
        "warnings": warnings,
        "next_safe_action": _next_safe_action(status),
        "replay_ready": bool(planner_input_ready and perception.get("replay_ready") is True),
        "shadow_request": shadow_request,
        "perception_shadow_result": perception,
        "forbidden_robot_control_fields": forbidden_fields,
        "safety_boundary": _safety_boundary(),
    }


def format_planner_gateway_shadow_report(result: Dict[str, Any]) -> str:
    return "\n".join(
        [
            "# TETO V2.10.0 Planner Gateway Shadow Contract Report",
            "",
            "## Overall Status",
            "",
            f"- planner_gateway_shadow_status: {_format_value(result.get('planner_gateway_shadow_status'))}",
            f"- planner_input_status: {_format_value(result.get('planner_input_status'))}",
            f"- gateway_request_id: {_format_value(result.get('gateway_request_id'))}",
            f"- task_id: {_format_value(result.get('task_id'))}",
            f"- intent_name: {_format_value(result.get('intent_name'))}",
            f"- user_command: {_format_value(result.get('user_command'))}",
            f"- world_frame: {_format_value(result.get('world_frame'))}",
            f"- world_point_m: {_format_value(result.get('world_point_m'))}",
            f"- bounded_target_point_m: {_format_value(result.get('bounded_target_point_m'))}",
            f"- planner_input_ready: {_format_value(result.get('planner_input_ready'))}",
            f"- blocking_reasons: {_format_value(result.get('blocking_reasons'))}",
            f"- warnings: {_format_value(result.get('warnings'))}",
            f"- next_safe_action: {_format_value(result.get('next_safe_action'))}",
            "",
            "## Planner Gateway Shadow Fields",
            "",
            "| Field | Value |",
            "| --- | --- |",
            *[f"| {field} | {_format_value(result.get(field))} |" for field in PLANNER_GATEWAY_SHADOW_FIELDS],
            "",
            "## Shadow Request",
            "",
            "The generated request is bounded planner input evidence only. It is not an execution command.",
            "",
            "```json",
            json.dumps(result.get("shadow_request"), ensure_ascii=False, indent=2, sort_keys=True),
            "```",
            "",
            "## No-Motion Safety Boundary",
            "",
            "V2.10.0 converts V2.9.5 perception shadow world_point_m evidence into Planner Gateway shadow input. It is no-ROS2-publish, no-MoveIt, no-real-robot, no-trajectory evidence only. It does not publish ROS2 topics, services, or actions; does not call MoveIt; does not connect to a real UR5; does not use RTDE, URScript, Dashboard, or a real robot backend; and does not generate trajectory, tcp_pose_world, joint targets, robot commands, automatic retry motion, or real execution requests.",
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
        "teto_version": CURRENT_PLANNER_GATEWAY_SHADOW_VERSION,
        "planner_gateway_shadow_requested": False,
        "requested": False,
        "planner_gateway_shadow_status": STATUS_NOT_REQUESTED,
        "planner_input_status": STATUS_NOT_REQUESTED,
        "planner_input_ready": False,
        "manual_confirmation_required": True,
        "execution_allowed": False,
        "ros2_publish_enabled": False,
        "ros2_publish_attempted": False,
        "moveit_called": False,
        "live_camera_used": False,
        "live_vlm_called": False,
        "trajectory_generated": False,
        "tcp_pose_world_generated": False,
        "joint_targets_generated": False,
        "robot_command_generated": False,
        "real_robot_motion_executed": False,
        "blocking_reasons": [],
        "warnings": [],
        "next_safe_action": None,
        "replay_ready": False,
        "shadow_request": None,
        "safety_boundary": _safety_boundary(),
    }


def _gateway_request_id(perception: Dict[str, Any], config: Dict[str, Any]) -> str:
    value = _string(config.get("gateway_request_id"))
    if value:
        return value
    snapshot_id = perception.get("snapshot_id") or "snapshot_unknown"
    grounding_id = perception.get("grounding_id") or "grounding_unknown"
    return f"planner_shadow_{snapshot_id}_{grounding_id}"


def _task_id(perception: Dict[str, Any], config: Dict[str, Any]) -> str:
    value = _string(config.get("task_id"))
    if value:
        return value
    target = perception.get("target_object_id") or perception.get("target_label") or "target_unknown"
    return f"shadow_task_{target}"


def _ttl_check_passed(perception: Dict[str, Any]) -> bool:
    geometry = perception.get("geometry_validity")
    if isinstance(geometry, dict) and isinstance(geometry.get("ttl_check_passed"), bool):
        return geometry["ttl_check_passed"]
    if isinstance(perception.get("ttl_check_passed"), bool):
        return perception["ttl_check_passed"]
    return perception.get("perception_shadow_status") == STATUS_PASS


def _bounded_target_point(point: Any, hover_offset_m: float) -> list[float] | None:
    if not _valid_point3(point):
        return None
    return _round_point([float(point[0]), float(point[1]), float(point[2]) + hover_offset_m])


def _point_in_workspace(point: Any, bounds: Dict[str, list[float]]) -> bool:
    if not _valid_point3(point):
        return False
    x, y, z = [float(value) for value in point]
    return (
        bounds["x"][0] <= x <= bounds["x"][1]
        and bounds["y"][0] <= y <= bounds["y"][1]
        and bounds["z"][0] <= z <= bounds["z"][1]
    )


def _workspace_bounds(config: Dict[str, Any]) -> Dict[str, list[float]]:
    raw = config.get("workspace_bounds") if isinstance(config.get("workspace_bounds"), dict) else {}
    bounds: Dict[str, list[float]] = {}
    for axis, default in DEFAULT_WORKSPACE_BOUNDS.items():
        value = raw.get(axis)
        bounds[axis] = _bounds_pair(value) or list(default)
    return bounds


def _bounds_pair(value: Any) -> list[float] | None:
    if not isinstance(value, list) or len(value) != 2:
        return None
    low = _optional_float(value[0])
    high = _optional_float(value[1])
    if low is None or high is None or low > high:
        return None
    return [low, high]


def _valid_point3(value: Any) -> bool:
    if not isinstance(value, list) or len(value) != 3:
        return False
    for item in value:
        number = _optional_float(item)
        if number is None or not math.isfinite(number):
            return False
    return True


def _round_point(value: Any) -> list[float] | None:
    if not _valid_point3(value):
        return None
    return [round(float(item), 6) for item in value]


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


def _safety_boundary() -> Dict[str, bool]:
    return {
        "no_ros2_publish": True,
        "no_ros2_service_call": True,
        "no_ros2_action_goal": True,
        "no_moveit": True,
        "no_real_ur5_connection": True,
        "no_rtde": True,
        "no_urscript": True,
        "no_dashboard": True,
        "no_trajectory": True,
        "no_tcp_pose_world": True,
        "no_joint_targets": True,
        "no_robot_command": True,
        "no_real_robot_backend": True,
        "no_real_execution_request": True,
        "no_automatic_retry_motion": True,
    }


def _next_safe_action(status: str) -> str:
    if status == STATUS_PASS:
        return "Use this bounded planner gateway shadow input only for offline review and manual reconfirmation."
    return "Fix the blocked perception or planner shadow contract field and rerun without ROS2, MoveIt, or robot control."


def _string(value: Any) -> str | None:
    return value if isinstance(value, str) and value else None


def _string_list(value: Any) -> list[str]:
    return [str(item) for item in value] if isinstance(value, list) else []


def _list(value: Any) -> list[str]:
    return [str(item) for item in value] if isinstance(value, list) else []


def _optional_float(value: Any) -> float | None:
    if isinstance(value, bool):
        return None
    if isinstance(value, (int, float)):
        return float(value)
    return None


def _unique(values: list[str]) -> list[str]:
    seen: set[str] = set()
    result: list[str] = []
    for value in values:
        if value not in seen:
            seen.add(value)
            result.append(value)
    return result


def _format_value(value: Any) -> str:
    if value is None:
        return "null"
    if isinstance(value, (dict, list)):
        return json.dumps(value, ensure_ascii=False, sort_keys=True)
    return str(value)
