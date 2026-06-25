from __future__ import annotations

import json
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict

import yaml


CONTRACT_VERSION = "teto_camera_snapshot.v1"
CURRENT_CAMERA_SNAPSHOT_VERSION = "TETO V2.8.2"

STATUS_PASS = "PASS"
STATUS_BLOCKED = "BLOCKED"
STATUS_NOT_REQUESTED = "NOT_REQUESTED"

FORMAL_REALSENSE_SOURCES = frozenset({"realsense_d455", "realsense_replay"})

E_CAMERA_SNAPSHOT_INVALID = "E_CAMERA_SNAPSHOT_INVALID"
E_SCENE_VERSION_MISSING = "E_SCENE_VERSION_MISSING"
E_CAPTURE_TIMESTAMP_MISSING = "E_CAPTURE_TIMESTAMP_MISSING"
E_CAMERA_SNAPSHOT_STALE = "E_CAMERA_SNAPSHOT_STALE"
E_CAMERA_FRAME_MISSING = "E_CAMERA_FRAME_MISSING"
E_IMAGE_REF_MISSING = "E_IMAGE_REF_MISSING"
E_DEPTH_REF_MISSING = "E_DEPTH_REF_MISSING"
E_ALIGNED_DEPTH_REQUIRED = "E_ALIGNED_DEPTH_REQUIRED"
E_CAMERA_INFO_REF_MISSING = "E_CAMERA_INFO_REF_MISSING"
E_METADATA_REF_MISSING = "E_METADATA_REF_MISSING"
E_TF_SNAPSHOT_REF_MISSING = "E_TF_SNAPSHOT_REF_MISSING"
E_LIVE_CAMERA_DISABLED = "E_LIVE_CAMERA_DISABLED"
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

SNAPSHOT_FIELDS = (
    "snapshot_id",
    "scene_version",
    "capture_timestamp",
    "ttl_ms",
    "source",
    "frame_id",
    "rgb_ref",
    "image_ref",
    "aligned_depth_ref",
    "depth_ref",
    "camera_info_ref",
    "metadata_ref",
    "tf_snapshot_ref",
    "extrinsics_ref",
    "width",
    "height",
    "color_encoding",
    "depth_encoding",
    "camera_frame",
    "alignment_status",
    "depth_aligned",
    "sync_status",
    "depth_available",
    "camera_info_available",
    "metadata_available",
    "extrinsics_available",
    "depth_required",
    "validity_status",
    "blocking_reasons",
    "warnings",
    "no_motion_snapshot_passed",
    "live_capture_used",
    "live_camera_enabled",
    "live_vlm_called",
    "real_robot_motion_executed",
    "real_robot_command_enabled",
)


@dataclass(frozen=True)
class CameraSnapshotRequest:
    requested: bool = False
    config_path: str | None = None
    snapshot: Dict[str, Any] | None = None


def load_camera_snapshot_config(path: str | Path | None) -> Dict[str, Any] | None:
    if not path:
        return None
    resolved_path = Path(path).expanduser()
    if not resolved_path.is_file():
        return None
    with resolved_path.open("r", encoding="utf-8") as config_file:
        if resolved_path.suffix.lower() == ".json":
            data = json.load(config_file)
        else:
            data = yaml.safe_load(config_file)
    if not isinstance(data, dict):
        return {}
    snapshot = data.get("camera_snapshot")
    return snapshot if isinstance(snapshot, dict) else data


def build_camera_snapshot_request(
    *,
    requested: bool = False,
    config_path: str | Path | None = None,
) -> CameraSnapshotRequest:
    return CameraSnapshotRequest(
        requested=requested,
        config_path=str(Path(config_path).expanduser()) if config_path else None,
        snapshot=load_camera_snapshot_config(config_path),
    )


def evaluate_formal_snapshot_replay(config_path: str | Path) -> Dict[str, Any]:
    result = evaluate_camera_snapshot_contract(
        build_camera_snapshot_request(requested=True, config_path=config_path)
    )
    if result.get("source") not in FORMAL_REALSENSE_SOURCES:
        return {
            **result,
            "validity_status": STATUS_BLOCKED,
            "formal_visual_entry_status": STATUS_BLOCKED,
            "formal_visual_entry_reason": "E_FORMAL_VISUAL_SOURCE_NOT_REALSENSE",
            "allowed_formal_sources": sorted(FORMAL_REALSENSE_SOURCES),
        }
    return {
        **result,
        "formal_visual_entry_status": result.get("validity_status"),
    }


def evaluate_camera_snapshot_contract(
    request: CameraSnapshotRequest | None = None,
    *,
    now: datetime | None = None,
) -> Dict[str, Any]:
    request = request or CameraSnapshotRequest()
    if not request.requested:
        return _not_requested_result()

    snapshot = request.snapshot if isinstance(request.snapshot, dict) else {}
    normalized = _snapshot_fields(snapshot)
    blocking_reasons: list[str] = []
    warnings: list[str] = []

    if not normalized["snapshot_id"]:
        blocking_reasons.append(E_CAMERA_SNAPSHOT_INVALID)
    if not normalized["scene_version"]:
        blocking_reasons.append(E_SCENE_VERSION_MISSING)
    if not normalized["capture_timestamp"]:
        blocking_reasons.append(E_CAPTURE_TIMESTAMP_MISSING)
    if not normalized["frame_id"] or not normalized["camera_frame"]:
        blocking_reasons.append(E_CAMERA_FRAME_MISSING)
    if not normalized["image_ref"]:
        blocking_reasons.append(E_IMAGE_REF_MISSING)
    formal_realsense_source = normalized["source"] in FORMAL_REALSENSE_SOURCES
    if (normalized["depth_required"] or formal_realsense_source) and not normalized["depth_ref"]:
        blocking_reasons.append(E_DEPTH_REF_MISSING)
    if formal_realsense_source:
        if not normalized["depth_aligned"] or normalized["alignment_status"] != "aligned_rgb_depth":
            blocking_reasons.append(E_ALIGNED_DEPTH_REQUIRED)
        if not normalized["camera_info_ref"]:
            blocking_reasons.append(E_CAMERA_INFO_REF_MISSING)
        if not normalized["metadata_ref"]:
            blocking_reasons.append(E_METADATA_REF_MISSING)
        if not normalized["tf_snapshot_ref"]:
            blocking_reasons.append(E_TF_SNAPSHOT_REF_MISSING)
    if normalized["live_camera_enabled"] or normalized["source"] == "live_camera":
        blocking_reasons.append(E_LIVE_CAMERA_DISABLED)
    forbidden_fields = _forbidden_robot_control_fields(snapshot)
    if forbidden_fields:
        blocking_reasons.append(E_ROBOT_COMMAND_NOT_ALLOWED)
        warnings.append(f"forbidden_robot_control_fields={forbidden_fields}")
    if _is_stale(normalized.get("capture_timestamp"), normalized.get("ttl_ms"), now=now):
        blocking_reasons.append(E_CAMERA_SNAPSHOT_STALE)

    blocking_reasons = _unique(blocking_reasons)
    warnings = _unique(warnings)
    validity_status = STATUS_PASS if not blocking_reasons else STATUS_BLOCKED
    no_motion_snapshot_passed = validity_status == STATUS_PASS
    return {
        **normalized,
        "contract_version": CONTRACT_VERSION,
        "snapshot_contract_type": (
            "realsense_scene_snapshot"
            if formal_realsense_source
            else "legacy_declared_snapshot_contract"
        ),
        "teto_version": CURRENT_CAMERA_SNAPSHOT_VERSION,
        "requested": True,
        "config_path": request.config_path,
        "validity_status": validity_status,
        "blocking_reasons": blocking_reasons,
        "warnings": warnings,
        "no_motion_snapshot_passed": no_motion_snapshot_passed,
        "live_capture_used": False,
        "live_camera_enabled": normalized["live_camera_enabled"],
        "live_vlm_called": False,
        "real_robot_motion_executed": False,
        "real_robot_command_enabled": False,
        "forbidden_robot_control_fields": forbidden_fields,
        "safety_boundary": _safety_boundary(),
        "next_safe_action": _next_safe_action(
            validity_status,
            blocking_reasons,
            source=normalized.get("source"),
        ),
    }


def format_camera_snapshot_report(result: Dict[str, Any]) -> str:
    return "\n".join(
        [
            "# TETO V2.8.2 Camera Snapshot Contract Report",
            "",
            "## Overall Status",
            "",
            f"- validity_status: {_format_value(result.get('validity_status'))}",
            f"- snapshot_id: {_format_value(result.get('snapshot_id'))}",
            f"- scene_version: {_format_value(result.get('scene_version'))}",
            f"- no_motion_snapshot_passed: {_format_value(result.get('no_motion_snapshot_passed'))}",
            f"- blocking_reasons: {_format_value(result.get('blocking_reasons'))}",
            f"- warnings: {_format_value(result.get('warnings'))}",
            f"- next_safe_action: {_format_value(result.get('next_safe_action'))}",
            "",
            "## Snapshot Metadata",
            "",
            "| Field | Value |",
            "| --- | --- |",
            *[f"| {field} | {_format_value(result.get(field))} |" for field in SNAPSHOT_FIELDS],
            "",
            "## No-Motion Safety Boundary",
            "",
            "V2.8.2 validates a declared camera snapshot manifest only. It does not capture from a live camera, does not call live Qwen or any live VLM, does not connect to a real UR5, does not use ROS2, MoveIt, RTDE, URScript, Dashboard, a trajectory planner, or tcp_pose_world execution, and does not generate joint targets or robot commands.",
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
        "teto_version": CURRENT_CAMERA_SNAPSHOT_VERSION,
        "requested": False,
        "validity_status": STATUS_NOT_REQUESTED,
        "blocking_reasons": [],
        "warnings": [],
        "no_motion_snapshot_passed": False,
        "live_capture_used": False,
        "live_camera_enabled": False,
        "live_vlm_called": False,
        "real_robot_motion_executed": False,
        "real_robot_command_enabled": False,
        "safety_boundary": _safety_boundary(),
        "next_safe_action": None,
    }


def _snapshot_fields(snapshot: Dict[str, Any]) -> Dict[str, Any]:
    rgb_ref = _string(snapshot.get("rgb_ref")) or _string(snapshot.get("image_ref"))
    aligned_depth_ref = _string(snapshot.get("aligned_depth_ref")) or _string(
        snapshot.get("depth_ref")
    )
    tf_snapshot_ref = _string(snapshot.get("tf_snapshot_ref")) or _string(snapshot.get("extrinsics_ref"))
    alignment_status = _string(snapshot.get("alignment_status"))
    return {
        "snapshot_id": _string(snapshot.get("snapshot_id")),
        "scene_version": _string(snapshot.get("scene_version")),
        "capture_timestamp": _string(snapshot.get("capture_timestamp")),
        "ttl_ms": _optional_int(snapshot.get("ttl_ms")),
        "source": _string(snapshot.get("source")) or "live_disabled",
        "frame_id": _string(snapshot.get("frame_id")),
        "rgb_ref": rgb_ref,
        "image_ref": rgb_ref,
        "aligned_depth_ref": aligned_depth_ref,
        "depth_ref": aligned_depth_ref,
        "camera_info_ref": _string(snapshot.get("camera_info_ref")),
        "metadata_ref": _string(snapshot.get("metadata_ref")),
        "tf_snapshot_ref": tf_snapshot_ref,
        "extrinsics_ref": _string(snapshot.get("extrinsics_ref")) or tf_snapshot_ref,
        "width": _optional_int(snapshot.get("width")),
        "height": _optional_int(snapshot.get("height")),
        "color_encoding": _string(snapshot.get("color_encoding")),
        "depth_encoding": _string(snapshot.get("depth_encoding")),
        "camera_frame": _string(snapshot.get("camera_frame")),
        "alignment_status": alignment_status,
        "depth_aligned": snapshot.get("depth_aligned") is True
        or alignment_status == "aligned_rgb_depth",
        "sync_status": _string(snapshot.get("sync_status")),
        "depth_available": snapshot.get("depth_available") is True,
        "camera_info_available": snapshot.get("camera_info_available") is True,
        "metadata_available": snapshot.get("metadata_available") is True,
        "extrinsics_available": snapshot.get("extrinsics_available") is True,
        "depth_required": snapshot.get("depth_required") is True,
        "validity_status": STATUS_NOT_REQUESTED,
        "blocking_reasons": [],
        "warnings": [],
        "no_motion_snapshot_passed": False,
        "live_capture_used": False,
        "live_camera_enabled": snapshot.get("live_camera_enabled") is True,
        "live_vlm_called": False,
        "real_robot_motion_executed": False,
        "real_robot_command_enabled": False,
    }


def _is_stale(
    capture_timestamp: Any,
    ttl_ms: Any,
    *,
    now: datetime | None = None,
) -> bool:
    if not isinstance(capture_timestamp, str) or not isinstance(ttl_ms, int):
        return False
    captured_at = _parse_datetime(capture_timestamp)
    if captured_at is None:
        return True
    age_ms = ((now or datetime.now(timezone.utc)) - captured_at).total_seconds() * 1000
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
        "no_moveit": True,
        "no_rtde": True,
        "no_urscript": True,
        "no_dashboard": True,
        "no_trajectory": True,
        "no_tcp_pose_world": True,
        "no_joint_targets": True,
        "no_robot_command": True,
        "no_automatic_retry_motion": True,
    }


def _next_safe_action(
    validity_status: str,
    blocking_reasons: list[str],
    *,
    source: str | None = None,
) -> str:
    if validity_status == STATUS_PASS:
        if source in FORMAL_REALSENSE_SOURCES:
            return "Use this validated RealSense snapshot as replayable no-motion scene evidence."
        return "Use this offline/manual snapshot only as replayable no-motion scene evidence."
    return (
        "Fix the camera snapshot manifest and rerun contract validation without enabling "
        "live capture, live VLM, or robot control."
    )


def _string(value: Any) -> str | None:
    return value if isinstance(value, str) and value else None


def _optional_int(value: Any) -> int | None:
    if isinstance(value, bool):
        return None
    if isinstance(value, int):
        return value
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


__all__ = [
    "CONTRACT_VERSION",
    "CURRENT_CAMERA_SNAPSHOT_VERSION",
    "STATUS_PASS",
    "STATUS_BLOCKED",
    "STATUS_NOT_REQUESTED",
    "FORMAL_REALSENSE_SOURCES",
    "E_CAMERA_SNAPSHOT_INVALID",
    "E_SCENE_VERSION_MISSING",
    "E_CAPTURE_TIMESTAMP_MISSING",
    "E_CAMERA_SNAPSHOT_STALE",
    "E_CAMERA_FRAME_MISSING",
    "E_IMAGE_REF_MISSING",
    "E_DEPTH_REF_MISSING",
    "E_ALIGNED_DEPTH_REQUIRED",
    "E_CAMERA_INFO_REF_MISSING",
    "E_METADATA_REF_MISSING",
    "E_TF_SNAPSHOT_REF_MISSING",
    "E_LIVE_CAMERA_DISABLED",
    "E_ROBOT_COMMAND_NOT_ALLOWED",
    "FORBIDDEN_ROBOT_CONTROL_FIELDS",
    "SNAPSHOT_FIELDS",
    "CameraSnapshotRequest",
    "load_camera_snapshot_config",
    "build_camera_snapshot_request",
    "evaluate_formal_snapshot_replay",
    "evaluate_camera_snapshot_contract",
    "format_camera_snapshot_report",
]
