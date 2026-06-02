from __future__ import annotations

import json
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict

import yaml

from src.camera_snapshot import build_camera_snapshot_request, evaluate_camera_snapshot_contract
from src.grounding_result import build_grounding_result_request, evaluate_grounding_result_contract


CONTRACT_VERSION = "teto_geometry_validity.v1"
CURRENT_GEOMETRY_VALIDITY_VERSION = "TETO V2.9.1"

STATUS_PASS = "PASS"
STATUS_BLOCKED = "BLOCKED"
STATUS_NOT_REQUESTED = "NOT_REQUESTED"

E_SNAPSHOT_MISMATCH = "E_SNAPSHOT_MISMATCH"
E_SCENE_VERSION_MISMATCH = "E_SCENE_VERSION_MISMATCH"
E_IMAGE_SIZE_MISSING = "E_IMAGE_SIZE_MISSING"
E_INVALID_IMAGE_SIZE = "E_INVALID_IMAGE_SIZE"
E_BBOX_MISSING = "E_BBOX_MISSING"
E_INVALID_BBOX_FORMAT = "E_INVALID_BBOX_FORMAT"
E_INVALID_BBOX = "E_INVALID_BBOX"
E_INVALID_BBOX_AREA = "E_INVALID_BBOX_AREA"
E_PIXEL_CENTER_MISSING = "E_PIXEL_CENTER_MISSING"
E_INVALID_PIXEL_CENTER = "E_INVALID_PIXEL_CENTER"
E_CAMERA_FRAME_MISSING = "E_CAMERA_FRAME_MISSING"
E_NO_DEPTH = "E_NO_DEPTH"
E_LOW_CONFIDENCE = "E_LOW_CONFIDENCE"
E_STATE_STALE = "E_STATE_STALE"
E_NO_TARGET = "E_NO_TARGET"
E_LIVE_CAMERA_DISABLED = "E_LIVE_CAMERA_DISABLED"
E_LIVE_VLM_DISABLED = "E_LIVE_VLM_DISABLED"
E_ROBOT_COMMAND_NOT_ALLOWED = "E_ROBOT_COMMAND_NOT_ALLOWED"

DEFAULT_CONFIDENCE_THRESHOLD = 0.6
DEFAULT_MIN_BBOX_AREA_PX = 1.0

GEOMETRY_FIELDS = (
    "geometry_validity_requested",
    "geometry_validity_status",
    "snapshot_id",
    "grounding_id",
    "scene_version",
    "image_width",
    "image_height",
    "bbox_xyxy",
    "pixel_center",
    "bbox_valid",
    "pixel_center_valid",
    "bbox_area_valid",
    "bbox_inside_image",
    "pixel_center_inside_image",
    "camera_frame_available",
    "frame_id_available",
    "depth_required",
    "depth_available",
    "confidence_check_passed",
    "ttl_check_passed",
    "scene_version_match",
    "snapshot_id_match",
    "blocking_reasons",
    "warnings",
    "next_safe_action",
    "no_motion_geometry_passed",
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
class GeometryValidityRequest:
    requested: bool = False
    config_path: str | None = None
    camera_snapshot_config: str | None = None
    grounding_result_path: str | None = None
    confidence_threshold: float = DEFAULT_CONFIDENCE_THRESHOLD
    min_bbox_area_px: float = DEFAULT_MIN_BBOX_AREA_PX
    depth_required: bool | None = None


def load_geometry_validity_config(path: str | Path | None) -> Dict[str, Any]:
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
    config = data.get("geometry_validity")
    return config if isinstance(config, dict) else data


def build_geometry_validity_request(
    *,
    requested: bool = False,
    config_path: str | Path | None = None,
    camera_snapshot_config: str | Path | None = None,
    grounding_result_path: str | Path | None = None,
) -> GeometryValidityRequest:
    config = load_geometry_validity_config(config_path)
    thresholds = config.get("thresholds") if isinstance(config.get("thresholds"), dict) else {}
    configured_snapshot = camera_snapshot_config or config.get("camera_snapshot_config")
    configured_grounding = grounding_result_path or config.get("grounding_result")
    confidence_threshold = _optional_float(
        thresholds.get("confidence_threshold", config.get("confidence_threshold"))
    )
    min_bbox_area = _optional_float(thresholds.get("min_bbox_area_px", config.get("min_bbox_area_px")))
    depth_required = config.get("depth_required")
    return GeometryValidityRequest(
        requested=requested,
        config_path=str(Path(config_path).expanduser()) if config_path else None,
        camera_snapshot_config=str(Path(configured_snapshot).expanduser())
        if isinstance(configured_snapshot, (str, Path))
        else None,
        grounding_result_path=str(Path(configured_grounding).expanduser())
        if isinstance(configured_grounding, (str, Path))
        else None,
        confidence_threshold=confidence_threshold
        if confidence_threshold is not None
        else DEFAULT_CONFIDENCE_THRESHOLD,
        min_bbox_area_px=min_bbox_area if min_bbox_area is not None else DEFAULT_MIN_BBOX_AREA_PX,
        depth_required=depth_required if isinstance(depth_required, bool) else None,
    )


def evaluate_geometry_validity(request: GeometryValidityRequest | None = None) -> Dict[str, Any]:
    request = request or GeometryValidityRequest()
    if not request.requested:
        return _not_requested_result()

    camera_snapshot = evaluate_camera_snapshot_contract(
        build_camera_snapshot_request(requested=True, config_path=request.camera_snapshot_config)
    )
    grounding_result = evaluate_grounding_result_contract(
        build_grounding_result_request(requested=True, result_path=request.grounding_result_path)
    )
    return evaluate_geometry_validity_from_contracts(
        camera_snapshot,
        grounding_result,
        config_path=request.config_path,
        camera_snapshot_config=request.camera_snapshot_config,
        grounding_result_path=request.grounding_result_path,
        confidence_threshold=request.confidence_threshold,
        min_bbox_area_px=request.min_bbox_area_px,
        depth_required=request.depth_required,
    )


def evaluate_geometry_validity_from_contracts(
    camera_snapshot: Dict[str, Any],
    grounding_result: Dict[str, Any],
    *,
    config_path: str | None = None,
    camera_snapshot_config: str | None = None,
    grounding_result_path: str | None = None,
    confidence_threshold: float = DEFAULT_CONFIDENCE_THRESHOLD,
    min_bbox_area_px: float = DEFAULT_MIN_BBOX_AREA_PX,
    depth_required: bool | None = None,
) -> Dict[str, Any]:
    blocking_reasons: list[str] = []
    warnings = list(grounding_result.get("warnings") or [])

    image_width = camera_snapshot.get("width")
    image_height = camera_snapshot.get("height")
    image_size_missing = image_width is None or image_height is None
    image_size_valid = _valid_image_dimension(image_width) and _valid_image_dimension(image_height)
    bbox = grounding_result.get("bbox_xyxy")
    pixel_center = grounding_result.get("pixel_center")
    bbox_format_valid = _bbox_format_valid(bbox)
    bbox_area_valid = _bbox_area_valid(bbox, min_bbox_area_px) if bbox_format_valid else False
    bbox_inside_image = _bbox_inside_image(bbox, image_width, image_height) if bbox_format_valid else False
    bbox_valid = bbox_format_valid and bbox_area_valid and bbox_inside_image
    pixel_center_format_valid = _pixel_center_format_valid(pixel_center)
    pixel_center_inside_image = (
        _pixel_center_inside_image(pixel_center, image_width, image_height)
        if pixel_center_format_valid
        else False
    )
    pixel_center_valid = pixel_center_format_valid and pixel_center_inside_image
    snapshot_id_match = camera_snapshot.get("snapshot_id") == grounding_result.get("snapshot_id")
    scene_version_match = camera_snapshot.get("scene_version") == grounding_result.get("scene_version")
    camera_frame_available = bool(camera_snapshot.get("camera_frame"))
    frame_id_available = bool(camera_snapshot.get("frame_id"))
    effective_depth_required = (
        depth_required if depth_required is not None else camera_snapshot.get("depth_required") is True
    )
    depth_available = camera_snapshot.get("depth_available") is True
    overall_confidence = grounding_result.get("overall_confidence")
    confidence_check_passed = (
        isinstance(overall_confidence, (int, float))
        and not isinstance(overall_confidence, bool)
        and float(overall_confidence) >= float(confidence_threshold)
    )
    ttl_check_passed = not _is_stale(camera_snapshot.get("capture_timestamp"), camera_snapshot.get("ttl_ms"))

    if not snapshot_id_match:
        blocking_reasons.append(E_SNAPSHOT_MISMATCH)
    if not scene_version_match:
        blocking_reasons.append(E_SCENE_VERSION_MISMATCH)
    if image_size_missing:
        blocking_reasons.append(E_IMAGE_SIZE_MISSING)
    elif not image_size_valid:
        blocking_reasons.append(E_INVALID_IMAGE_SIZE)
    if bbox is None:
        blocking_reasons.append(E_BBOX_MISSING)
    elif not bbox_format_valid:
        blocking_reasons.append(E_INVALID_BBOX_FORMAT)
    else:
        if not bbox_inside_image:
            blocking_reasons.append(E_INVALID_BBOX)
        if not bbox_area_valid:
            blocking_reasons.append(E_INVALID_BBOX_AREA)
    if pixel_center is None:
        blocking_reasons.append(E_PIXEL_CENTER_MISSING)
    elif not pixel_center_valid:
        blocking_reasons.append(E_INVALID_PIXEL_CENTER)
    if not camera_frame_available or not frame_id_available:
        blocking_reasons.append(E_CAMERA_FRAME_MISSING)
    if effective_depth_required and not depth_available:
        blocking_reasons.append(E_NO_DEPTH)
    if not confidence_check_passed:
        blocking_reasons.append(E_LOW_CONFIDENCE)
    if not ttl_check_passed:
        blocking_reasons.append(E_STATE_STALE)
    if grounding_result.get("rejected") is True:
        blocking_reasons.append(
            grounding_result.get("rejection_reason")
            or grounding_result.get("error_code")
            or E_NO_TARGET
        )
    if grounding_result.get("grounded") is not True:
        blocking_reasons.append(E_NO_TARGET)
    if (
        grounding_result.get("live_camera_used") is True
        or camera_snapshot.get("live_capture_used") is True
        or camera_snapshot.get("live_camera_enabled") is True
    ):
        blocking_reasons.append(E_LIVE_CAMERA_DISABLED)
    if grounding_result.get("live_vlm_called") is True or camera_snapshot.get("live_vlm_called") is True:
        blocking_reasons.append(E_LIVE_VLM_DISABLED)

    forbidden_fields = _unique(
        list(camera_snapshot.get("forbidden_robot_control_fields") or [])
        + list(grounding_result.get("forbidden_robot_control_fields") or [])
    )
    if forbidden_fields:
        blocking_reasons.append(E_ROBOT_COMMAND_NOT_ALLOWED)
        warnings.append(f"forbidden_robot_control_fields={forbidden_fields}")

    blocking_reasons = _unique(blocking_reasons)
    warnings = _unique(warnings)
    status = STATUS_PASS if not blocking_reasons else STATUS_BLOCKED
    no_motion_geometry_passed = status == STATUS_PASS
    return {
        "contract_version": CONTRACT_VERSION,
        "teto_version": CURRENT_GEOMETRY_VALIDITY_VERSION,
        "geometry_validity_requested": True,
        "requested": True,
        "config_path": config_path,
        "camera_snapshot_config": camera_snapshot_config,
        "grounding_result_path": grounding_result_path,
        "camera_snapshot": camera_snapshot,
        "grounding_result": grounding_result,
        "geometry_validity_status": status,
        "snapshot_id": camera_snapshot.get("snapshot_id"),
        "grounding_id": grounding_result.get("grounding_id"),
        "scene_version": camera_snapshot.get("scene_version"),
        "image_width": image_width,
        "image_height": image_height,
        "bbox_xyxy": bbox,
        "pixel_center": pixel_center,
        "bbox_valid": bbox_valid,
        "pixel_center_valid": pixel_center_valid,
        "bbox_area_valid": bbox_area_valid,
        "bbox_inside_image": bbox_inside_image,
        "pixel_center_inside_image": pixel_center_inside_image,
        "camera_frame_available": camera_frame_available,
        "frame_id_available": frame_id_available,
        "depth_required": effective_depth_required,
        "depth_available": depth_available,
        "confidence_threshold": confidence_threshold,
        "confidence_check_passed": confidence_check_passed,
        "ttl_check_passed": ttl_check_passed,
        "scene_version_match": scene_version_match,
        "snapshot_id_match": snapshot_id_match,
        "blocking_reasons": blocking_reasons,
        "warnings": warnings,
        "next_safe_action": _next_safe_action(status),
        "no_motion_geometry_passed": no_motion_geometry_passed,
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


def format_geometry_validity_report(result: Dict[str, Any]) -> str:
    return "\n".join(
        [
            "# TETO V2.9.1 Geometry Validity Contract Report",
            "",
            "## Overall Status",
            "",
            f"- geometry_validity_status: {_format_value(result.get('geometry_validity_status'))}",
            f"- snapshot_id: {_format_value(result.get('snapshot_id'))}",
            f"- grounding_id: {_format_value(result.get('grounding_id'))}",
            f"- scene_version: {_format_value(result.get('scene_version'))}",
            f"- no_motion_geometry_passed: {_format_value(result.get('no_motion_geometry_passed'))}",
            f"- blocking_reasons: {_format_value(result.get('blocking_reasons'))}",
            f"- warnings: {_format_value(result.get('warnings'))}",
            f"- next_safe_action: {_format_value(result.get('next_safe_action'))}",
            "",
            "## Geometry Checks",
            "",
            "| Field | Value |",
            "| --- | --- |",
            *[f"| {field} | {_format_value(result.get(field))} |" for field in GEOMETRY_FIELDS],
            "",
            "## No-Motion Safety Boundary",
            "",
            "V2.9.1 validates declared snapshot and offline/mock grounding geometry before any 2D-to-3D projector handoff. It is no-motion, no-live-camera, no-live-VLM, and no-real-robot evidence only. It does not capture from a live camera, does not call live Qwen or any live VLM, does not connect to a real UR5, does not use ROS2, MoveIt, RTDE, URScript, Dashboard, a trajectory planner, or tcp_pose_world execution, and does not generate joint targets, trajectories, robot commands, or real execution requests.",
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
        "teto_version": CURRENT_GEOMETRY_VALIDITY_VERSION,
        "geometry_validity_requested": False,
        "requested": False,
        "geometry_validity_status": STATUS_NOT_REQUESTED,
        "blocking_reasons": [],
        "warnings": [],
        "next_safe_action": None,
        "no_motion_geometry_passed": False,
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


def _valid_image_dimension(value: Any) -> bool:
    return isinstance(value, int) and not isinstance(value, bool) and value > 0


def _bbox_format_valid(value: Any) -> bool:
    return (
        isinstance(value, list)
        and len(value) == 4
        and all(isinstance(item, (int, float)) and not isinstance(item, bool) for item in value)
    )


def _bbox_area_valid(value: Any, min_bbox_area_px: float) -> bool:
    x1, y1, x2, y2 = [float(item) for item in value]
    return (x2 - x1) * (y2 - y1) >= float(min_bbox_area_px)


def _bbox_inside_image(value: Any, width: Any, height: Any) -> bool:
    if not _valid_image_dimension(width) or not _valid_image_dimension(height):
        return False
    x1, y1, x2, y2 = [float(item) for item in value]
    return 0 <= x1 < x2 <= float(width) and 0 <= y1 < y2 <= float(height)


def _pixel_center_format_valid(value: Any) -> bool:
    return (
        isinstance(value, list)
        and len(value) == 2
        and all(isinstance(item, (int, float)) and not isinstance(item, bool) for item in value)
    )


def _pixel_center_inside_image(value: Any, width: Any, height: Any) -> bool:
    if not _valid_image_dimension(width) or not _valid_image_dimension(height):
        return False
    x, y = [float(item) for item in value]
    return 0 <= x <= float(width) and 0 <= y <= float(height)


def _is_stale(capture_timestamp: Any, ttl_ms: Any) -> bool:
    if not isinstance(capture_timestamp, str) or not isinstance(ttl_ms, int) or isinstance(ttl_ms, bool):
        return False
    captured_at = _parse_datetime(capture_timestamp)
    if captured_at is None:
        return True
    age_ms = (datetime.now(timezone.utc) - captured_at).total_seconds() * 1000
    return age_ms > ttl_ms


def _parse_datetime(value: str) -> datetime | None:
    normalized = value.strip()
    if normalized.endswith("Z"):
        normalized = f"{normalized[:-1]}+00:00"
    try:
        parsed = datetime.fromisoformat(normalized)
    except ValueError:
        return None
    if parsed.tzinfo is None:
        parsed = parsed.replace(tzinfo=timezone.utc)
    return parsed.astimezone(timezone.utc)


def _safety_boundary() -> Dict[str, bool]:
    return {
        "no_live_camera_capture": True,
        "no_live_vlm_call": True,
        "no_real_ur5_connection": True,
        "no_ros2": True,
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
        return "Allow only a no-motion handoff to the next offline 2D-to-3D projector validation step."
    return "Fix snapshot/grounding geometry evidence and rerun without live capture, live VLM, or robot control."


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
        return json.dumps(value, ensure_ascii=False)
    return str(value)
