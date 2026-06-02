from __future__ import annotations

import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict

import yaml

from src.geometry_validity import build_geometry_validity_request, evaluate_geometry_validity


CONTRACT_VERSION = "teto_projector_shadow.v1"
CURRENT_PROJECTOR_SHADOW_VERSION = "TETO V2.9.2"

STATUS_PASS = "PASS"
STATUS_BLOCKED = "BLOCKED"
STATUS_NOT_REQUESTED = "NOT_REQUESTED"

E_GEOMETRY_NOT_VALID = "E_GEOMETRY_NOT_VALID"
E_PIXEL_CENTER_MISSING = "E_PIXEL_CENTER_MISSING"
E_CAMERA_INFO_MISSING = "E_CAMERA_INFO_MISSING"
E_INVALID_CAMERA_INTRINSICS = "E_INVALID_CAMERA_INTRINSICS"
E_NO_DEPTH = "E_NO_DEPTH"
E_INVALID_DEPTH = "E_INVALID_DEPTH"
E_DEPTH_OUT_OF_RANGE = "E_DEPTH_OUT_OF_RANGE"
E_CAMERA_FRAME_MISSING = "E_CAMERA_FRAME_MISSING"
E_WORLD_FRAME_MISSING = "E_WORLD_FRAME_MISSING"
E_TF_UNAVAILABLE = "E_TF_UNAVAILABLE"
E_INVALID_PROJECTION = "E_INVALID_PROJECTION"
E_OUT_OF_WORKSPACE = "E_OUT_OF_WORKSPACE"
E_LIVE_CAMERA_DISABLED = "E_LIVE_CAMERA_DISABLED"
E_LIVE_VLM_DISABLED = "E_LIVE_VLM_DISABLED"
E_ROBOT_COMMAND_NOT_ALLOWED = "E_ROBOT_COMMAND_NOT_ALLOWED"

DEFAULT_MIN_DEPTH_M = 0.05
DEFAULT_MAX_DEPTH_M = 5.0
DEFAULT_PROJECTION_METHOD = "pinhole_mock_tf"

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

PROJECTOR_FIELDS = (
    "projector_requested",
    "projector_status",
    "snapshot_id",
    "grounding_id",
    "scene_version",
    "pixel_center",
    "depth_value_m",
    "depth_valid",
    "camera_intrinsics_available",
    "camera_frame",
    "world_frame",
    "camera_point_m",
    "world_point_m",
    "projection_confidence",
    "projection_method",
    "tf_available",
    "tf_source",
    "real_tf_used",
    "ros2_tf_used",
    "workspace_check_passed",
    "blocking_reasons",
    "warnings",
    "next_safe_action",
    "no_motion_projector_passed",
    "live_camera_used",
    "live_vlm_called",
    "real_robot_motion_executed",
    "real_robot_command_enabled",
    "robot_command_generated",
    "trajectory_generated",
    "joint_targets_generated",
    "tcp_pose_world_generated",
)


@dataclass(frozen=True)
class ProjectorShadowRequest:
    requested: bool = False
    config_path: str | None = None
    projector_config: Dict[str, Any] | None = None
    camera_snapshot_config: str | None = None
    grounding_result_path: str | None = None
    geometry_validity_config: str | None = None


def load_projector_shadow_config(path: str | Path | None) -> Dict[str, Any]:
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
    config = data.get("projector_shadow")
    return config if isinstance(config, dict) else data


def build_projector_shadow_request(
    *,
    requested: bool = False,
    config_path: str | Path | None = None,
    camera_snapshot_config: str | Path | None = None,
    grounding_result_path: str | Path | None = None,
    geometry_validity_config: str | Path | None = None,
) -> ProjectorShadowRequest:
    config = load_projector_shadow_config(config_path)
    configured_snapshot = camera_snapshot_config or config.get("camera_snapshot_config")
    configured_grounding = grounding_result_path or config.get("grounding_result")
    configured_geometry = geometry_validity_config or config.get("geometry_validity_config")
    return ProjectorShadowRequest(
        requested=requested,
        config_path=str(Path(config_path).expanduser()) if config_path else None,
        projector_config=config,
        camera_snapshot_config=str(Path(configured_snapshot).expanduser())
        if isinstance(configured_snapshot, (str, Path))
        else None,
        grounding_result_path=str(Path(configured_grounding).expanduser())
        if isinstance(configured_grounding, (str, Path))
        else None,
        geometry_validity_config=str(Path(configured_geometry).expanduser())
        if isinstance(configured_geometry, (str, Path))
        else None,
    )


def evaluate_projector_shadow(request: ProjectorShadowRequest | None = None) -> Dict[str, Any]:
    request = request or ProjectorShadowRequest()
    if not request.requested:
        return _not_requested_result()

    geometry = evaluate_geometry_validity(
        build_geometry_validity_request(
            requested=True,
            config_path=request.geometry_validity_config,
            camera_snapshot_config=request.camera_snapshot_config,
            grounding_result_path=request.grounding_result_path,
        )
    )
    return evaluate_projector_shadow_from_contracts(
        geometry,
        projector_config=request.projector_config or {},
        config_path=request.config_path,
        camera_snapshot_config=request.camera_snapshot_config,
        grounding_result_path=request.grounding_result_path,
        geometry_validity_config=request.geometry_validity_config,
    )


def evaluate_projector_shadow_from_contracts(
    geometry_validity: Dict[str, Any],
    *,
    projector_config: Dict[str, Any] | None = None,
    config_path: str | None = None,
    camera_snapshot_config: str | None = None,
    grounding_result_path: str | None = None,
    geometry_validity_config: str | None = None,
) -> Dict[str, Any]:
    projector_config = projector_config if isinstance(projector_config, dict) else {}
    blocking_reasons: list[str] = []
    warnings = list(geometry_validity.get("warnings") or [])

    camera_snapshot = (
        geometry_validity.get("camera_snapshot")
        if isinstance(geometry_validity.get("camera_snapshot"), dict)
        else {}
    )
    grounding_result = (
        geometry_validity.get("grounding_result")
        if isinstance(geometry_validity.get("grounding_result"), dict)
        else {}
    )
    pixel_center = geometry_validity.get("pixel_center")
    camera_frame = _string(projector_config.get("camera_frame")) or _string(camera_snapshot.get("camera_frame"))
    world_frame = _string(projector_config.get("world_frame"))
    camera_info = _load_payload(projector_config, "camera_info", "camera_info_ref", "camera_info")
    depth_sample = _load_payload(projector_config, "depth_sample", "depth_sample_ref", "depth_sample")
    mock_tf = _load_payload(projector_config, "mock_tf", "mock_tf_ref", "mock_tf")
    intrinsics = _extract_intrinsics(camera_info)
    depth_value_m = _extract_depth_value(depth_sample, projector_config)
    min_depth_m = _optional_float(projector_config.get("min_depth_m"))
    max_depth_m = _optional_float(projector_config.get("max_depth_m"))
    depth_range = projector_config.get("depth_range_m") if isinstance(projector_config.get("depth_range_m"), dict) else {}
    min_depth_m = _optional_float(depth_range.get("min", min_depth_m)) or DEFAULT_MIN_DEPTH_M
    max_depth_m = _optional_float(depth_range.get("max", max_depth_m)) or DEFAULT_MAX_DEPTH_M
    projection_method = _string(projector_config.get("projection_method")) or DEFAULT_PROJECTION_METHOD
    projection_confidence = _projection_confidence(geometry_validity, projector_config)
    require_world_projection = projector_config.get("require_world_projection", True) is not False
    tf_available = isinstance(mock_tf, dict) and bool(mock_tf)
    tf_payload = mock_tf if isinstance(mock_tf, dict) else {}
    tf_source = _string(tf_payload.get("tf_source"))
    tf_source = tf_source or "mock_or_config"
    real_tf_used = projector_config.get("real_tf_used") is True or tf_payload.get("real_tf_used") is True
    ros2_tf_used = projector_config.get("ros2_tf_used") is True or tf_payload.get("ros2_tf_used") is True

    camera_intrinsics_available = camera_info is not None
    intrinsics_valid = _intrinsics_valid(intrinsics)
    depth_valid = isinstance(depth_value_m, (int, float)) and not isinstance(depth_value_m, bool) and depth_value_m > 0
    depth_in_range = (
        depth_valid and float(min_depth_m) <= float(depth_value_m) <= float(max_depth_m)
    )

    camera_point_m: list[float] | None = None
    world_point_m: list[float] | None = None
    if _pixel_center_valid(pixel_center) and intrinsics_valid and depth_in_range:
        u, v = [float(value) for value in pixel_center]
        fx = float(intrinsics["fx"])
        fy = float(intrinsics["fy"])
        cx = float(intrinsics["cx"])
        cy = float(intrinsics["cy"])
        z = float(depth_value_m)
        camera_point_m = [
            (u - cx) * z / fx,
            (v - cy) * z / fy,
            z,
        ]
        if tf_available:
            world_point_m = _apply_mock_transform(camera_point_m, mock_tf)

    projection_valid = _point_valid(camera_point_m) and (
        (not require_world_projection) or _point_valid(world_point_m)
    )
    workspace_check_passed = _workspace_check(world_point_m or camera_point_m, projector_config)

    if geometry_validity.get("geometry_validity_status") != STATUS_PASS:
        blocking_reasons.append(E_GEOMETRY_NOT_VALID)
    if not _pixel_center_valid(pixel_center):
        blocking_reasons.append(E_PIXEL_CENTER_MISSING)
    if camera_info is None:
        blocking_reasons.append(E_CAMERA_INFO_MISSING)
    elif not intrinsics_valid:
        blocking_reasons.append(E_INVALID_CAMERA_INTRINSICS)
    if depth_value_m is None:
        blocking_reasons.append(E_NO_DEPTH)
    elif not depth_valid:
        blocking_reasons.append(E_INVALID_DEPTH)
    elif not depth_in_range:
        blocking_reasons.append(E_DEPTH_OUT_OF_RANGE)
    if not camera_frame:
        blocking_reasons.append(E_CAMERA_FRAME_MISSING)
    if not world_frame:
        blocking_reasons.append(E_WORLD_FRAME_MISSING)
    if require_world_projection and not tf_available:
        blocking_reasons.append(E_TF_UNAVAILABLE)
    if camera_point_m is not None and not projection_valid:
        blocking_reasons.append(E_INVALID_PROJECTION)
    if projection_valid and not workspace_check_passed:
        blocking_reasons.append(E_OUT_OF_WORKSPACE)
    if geometry_validity.get("live_camera_used") is True or camera_snapshot.get("live_capture_used") is True:
        blocking_reasons.append(E_LIVE_CAMERA_DISABLED)
    if geometry_validity.get("live_vlm_called") is True:
        blocking_reasons.append(E_LIVE_VLM_DISABLED)
    if real_tf_used or ros2_tf_used:
        blocking_reasons.append(E_TF_UNAVAILABLE)
        warnings.append("real_or_ros2_tf_requested_but_disabled")

    forbidden_fields = _unique(
        list(geometry_validity.get("forbidden_robot_control_fields") or [])
        + _forbidden_robot_control_fields(projector_config)
    )
    if forbidden_fields:
        blocking_reasons.append(E_ROBOT_COMMAND_NOT_ALLOWED)
        warnings.append(f"forbidden_robot_control_fields={forbidden_fields}")

    blocking_reasons = _unique(blocking_reasons)
    warnings = _unique(warnings)
    status = STATUS_PASS if not blocking_reasons else STATUS_BLOCKED
    no_motion_projector_passed = status == STATUS_PASS
    return {
        "contract_version": CONTRACT_VERSION,
        "teto_version": CURRENT_PROJECTOR_SHADOW_VERSION,
        "projector_requested": True,
        "requested": True,
        "config_path": config_path,
        "camera_snapshot_config": camera_snapshot_config,
        "grounding_result_path": grounding_result_path,
        "geometry_validity_config": geometry_validity_config,
        "geometry_validity": geometry_validity,
        "projector_status": status,
        "snapshot_id": geometry_validity.get("snapshot_id"),
        "grounding_id": geometry_validity.get("grounding_id"),
        "scene_version": geometry_validity.get("scene_version"),
        "pixel_center": pixel_center,
        "depth_value_m": depth_value_m,
        "depth_valid": depth_valid and depth_in_range,
        "camera_intrinsics_available": camera_intrinsics_available and intrinsics_valid,
        "camera_frame": camera_frame,
        "world_frame": world_frame,
        "camera_point_m": _round_point(camera_point_m),
        "world_point_m": _round_point(world_point_m),
        "projection_confidence": projection_confidence,
        "projection_method": projection_method,
        "tf_available": tf_available,
        "tf_source": tf_source,
        "real_tf_used": False,
        "ros2_tf_used": False,
        "workspace_check_passed": workspace_check_passed,
        "blocking_reasons": blocking_reasons,
        "warnings": warnings,
        "next_safe_action": _next_safe_action(status),
        "no_motion_projector_passed": no_motion_projector_passed,
        "live_camera_used": False,
        "live_vlm_called": False,
        "real_robot_motion_executed": False,
        "real_robot_command_enabled": False,
        "robot_command_generated": False,
        "trajectory_generated": False,
        "joint_targets_generated": False,
        "tcp_pose_world_generated": False,
        "forbidden_robot_control_fields": forbidden_fields,
        "safety_boundary": _safety_boundary(),
    }


def format_projector_shadow_report(result: Dict[str, Any]) -> str:
    return "\n".join(
        [
            "# TETO V2.9.2 2D-to-3D Projector Shadow Contract Report",
            "",
            "## Overall Status",
            "",
            f"- projector_status: {_format_value(result.get('projector_status'))}",
            f"- snapshot_id: {_format_value(result.get('snapshot_id'))}",
            f"- grounding_id: {_format_value(result.get('grounding_id'))}",
            f"- scene_version: {_format_value(result.get('scene_version'))}",
            f"- camera_point_m: {_format_value(result.get('camera_point_m'))}",
            f"- world_point_m: {_format_value(result.get('world_point_m'))}",
            f"- no_motion_projector_passed: {_format_value(result.get('no_motion_projector_passed'))}",
            f"- blocking_reasons: {_format_value(result.get('blocking_reasons'))}",
            f"- warnings: {_format_value(result.get('warnings'))}",
            f"- next_safe_action: {_format_value(result.get('next_safe_action'))}",
            "",
            "## Projection Checks",
            "",
            "| Field | Value |",
            "| --- | --- |",
            *[f"| {field} | {_format_value(result.get(field))} |" for field in PROJECTOR_FIELDS],
            "",
            "## No-Motion Safety Boundary",
            "",
            "V2.9.2 converts declared pixel_center, depth, camera intrinsics, and mock/config transform into shadow camera_point_m and world_point_m evidence only. It is no-motion, no-live-camera, no-live-VLM, no-real-robot, and no-ROS2-TF evidence. It does not capture from a live camera, does not call live Qwen or any live VLM, does not use real ROS2 tf2, does not connect to a real UR5, does not use MoveIt, RTDE, URScript, Dashboard, a trajectory planner, or tcp_pose_world execution, and does not generate joint targets, trajectories, robot commands, or real execution requests.",
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
        "teto_version": CURRENT_PROJECTOR_SHADOW_VERSION,
        "projector_requested": False,
        "requested": False,
        "projector_status": STATUS_NOT_REQUESTED,
        "blocking_reasons": [],
        "warnings": [],
        "next_safe_action": None,
        "no_motion_projector_passed": False,
        "real_tf_used": False,
        "ros2_tf_used": False,
        "live_camera_used": False,
        "live_vlm_called": False,
        "real_robot_motion_executed": False,
        "real_robot_command_enabled": False,
        "robot_command_generated": False,
        "trajectory_generated": False,
        "joint_targets_generated": False,
        "tcp_pose_world_generated": False,
        "safety_boundary": _safety_boundary(),
    }


def _load_payload(config: Dict[str, Any], inline_key: str, ref_key: str, wrapped_key: str) -> Any:
    inline = config.get(inline_key)
    if inline is not None:
        if isinstance(inline, dict) and wrapped_key in inline and isinstance(inline.get(wrapped_key), dict):
            return inline[wrapped_key]
        return inline
    ref = config.get(ref_key)
    if not isinstance(ref, str) or not ref:
        return None
    path = Path(ref).expanduser()
    if not path.is_file():
        return None
    with path.open("r", encoding="utf-8") as payload_file:
        if path.suffix.lower() == ".json":
            data = json.load(payload_file)
        else:
            data = yaml.safe_load(payload_file)
    if isinstance(data, dict) and wrapped_key in data:
        return data.get(wrapped_key)
    return data


def _extract_intrinsics(camera_info: Any) -> Dict[str, float | None]:
    if not isinstance(camera_info, dict):
        return {"fx": None, "fy": None, "cx": None, "cy": None}
    intrinsics = camera_info.get("intrinsics") if isinstance(camera_info.get("intrinsics"), dict) else camera_info
    return {
        "fx": _optional_float(intrinsics.get("fx")),
        "fy": _optional_float(intrinsics.get("fy")),
        "cx": _optional_float(intrinsics.get("cx")),
        "cy": _optional_float(intrinsics.get("cy")),
    }


def _intrinsics_valid(intrinsics: Dict[str, Any]) -> bool:
    return (
        _positive_number(intrinsics.get("fx"))
        and _positive_number(intrinsics.get("fy"))
        and _finite_number(intrinsics.get("cx"))
        and _finite_number(intrinsics.get("cy"))
    )


def _extract_depth_value(depth_sample: Any, config: Dict[str, Any]) -> float | None:
    inline_depth = _optional_float(config.get("depth_value_m"))
    if inline_depth is not None:
        return inline_depth
    if isinstance(depth_sample, dict):
        return _optional_float(depth_sample.get("depth_value_m"))
    return _optional_float(depth_sample)


def _projection_confidence(geometry: Dict[str, Any], config: Dict[str, Any]) -> float | None:
    configured = _optional_float(config.get("projection_confidence"))
    if configured is not None:
        return configured
    grounding = geometry.get("grounding_result") if isinstance(geometry.get("grounding_result"), dict) else {}
    confidence = _optional_float(grounding.get("overall_confidence"))
    if confidence is not None:
        return confidence
    return 1.0 if geometry.get("geometry_validity_status") == STATUS_PASS else 0.0


def _apply_mock_transform(camera_point: list[float], mock_tf: Dict[str, Any]) -> list[float] | None:
    translation = mock_tf.get("translation_m", [0.0, 0.0, 0.0])
    rotation = mock_tf.get("rotation_matrix")
    if not _vector_valid(translation, 3):
        return None
    if rotation is None:
        rotated = list(camera_point)
    elif _matrix_valid(rotation):
        rotated = [
            sum(float(rotation[row][col]) * float(camera_point[col]) for col in range(3))
            for row in range(3)
        ]
    else:
        return None
    return [rotated[index] + float(translation[index]) for index in range(3)]


def _workspace_check(point: list[float] | None, config: Dict[str, Any]) -> bool:
    workspace = config.get("workspace_m")
    if not isinstance(workspace, dict):
        return _point_valid(point)
    if not _point_valid(point):
        return False
    axes = ("x", "y", "z")
    for index, axis in enumerate(axes):
        limits = workspace.get(axis)
        if not isinstance(limits, list) or len(limits) != 2:
            return False
        lower = _optional_float(limits[0])
        upper = _optional_float(limits[1])
        if lower is None or upper is None or lower > upper:
            return False
        if not (lower <= float(point[index]) <= upper):
            return False
    return True


def _pixel_center_valid(value: Any) -> bool:
    return (
        isinstance(value, list)
        and len(value) == 2
        and all(isinstance(item, (int, float)) and not isinstance(item, bool) for item in value)
    )


def _point_valid(value: Any) -> bool:
    return _vector_valid(value, 3)


def _vector_valid(value: Any, length: int) -> bool:
    return (
        isinstance(value, list)
        and len(value) == length
        and all(_finite_number(item) for item in value)
    )


def _matrix_valid(value: Any) -> bool:
    return isinstance(value, list) and len(value) == 3 and all(_vector_valid(row, 3) for row in value)


def _positive_number(value: Any) -> bool:
    return _finite_number(value) and float(value) > 0


def _finite_number(value: Any) -> bool:
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(float(value))


def _optional_float(value: Any) -> float | None:
    if isinstance(value, bool):
        return None
    if isinstance(value, (int, float)) and math.isfinite(float(value)):
        return float(value)
    return None


def _round_point(point: list[float] | None) -> list[float] | None:
    if not _point_valid(point):
        return None
    return [round(float(value), 6) for value in point]


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
        "no_live_camera_capture": True,
        "no_live_vlm_call": True,
        "no_real_ur5_connection": True,
        "no_ros2": True,
        "no_ros2_tf": True,
        "no_moveit": True,
        "no_rtde": True,
        "no_urscript": True,
        "no_dashboard": True,
        "no_trajectory": True,
        "no_tcp_pose_world": True,
        "no_joint_targets": True,
        "no_robot_command": True,
        "no_real_execution_request": True,
        "no_automatic_retry_motion": True,
    }


def _next_safe_action(status: str) -> str:
    if status == STATUS_PASS:
        return "Use this projector result only as no-motion shadow geometry evidence for V3.0 preparation."
    return "Fix projector shadow evidence and rerun without live capture, live VLM, ROS2 TF, or robot control."


def _string(value: Any) -> str | None:
    return value if isinstance(value, str) and value else None


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
        return json.dumps(value, ensure_ascii=False)
    return str(value)
