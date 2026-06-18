from __future__ import annotations

import importlib.util
import json
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict

import yaml

from src.camera_snapshot import CameraSnapshotRequest, evaluate_camera_snapshot_contract


CONTRACT_VERSION = "teto_camera_source_adapter.v1"
CURRENT_CAMERA_SOURCE_VERSION = "TETO V2.9.3"

STATUS_PASS = "PASS"
STATUS_BLOCKED = "BLOCKED"
STATUS_SAFE_DISABLED = "SAFE_DISABLED"
STATUS_NOT_REQUESTED = "NOT_REQUESTED"

MODE_OFFLINE_FILE = "offline_file"
MODE_MANUAL_SNAPSHOT = "manual_snapshot"
MODE_LIVE_DISABLED = "live_disabled"
MODE_REALSENSE_REPLAY = "realsense_replay"
MODE_REALSENSE_ONE_SHOT = "optional_realsense_one_shot"

E_LIVE_CAMERA_CAPTURE_NOT_ALLOWED = "E_LIVE_CAMERA_CAPTURE_NOT_ALLOWED"
E_CONTINUOUS_CAPTURE_DISABLED = "E_CONTINUOUS_CAPTURE_DISABLED"
E_CAMERA_BACKEND_UNAVAILABLE = "E_CAMERA_BACKEND_UNAVAILABLE"
E_IMAGE_REF_MISSING = "E_IMAGE_REF_MISSING"
E_CAMERA_INFO_MISSING = "E_CAMERA_INFO_MISSING"
E_CAMERA_FRAME_MISSING = "E_CAMERA_FRAME_MISSING"
E_CAPTURE_TIMESTAMP_MISSING = "E_CAPTURE_TIMESTAMP_MISSING"
E_LIVE_VLM_DISABLED = "E_LIVE_VLM_DISABLED"
E_ROBOT_COMMAND_NOT_ALLOWED = "E_ROBOT_COMMAND_NOT_ALLOWED"
E_UNSUPPORTED_SOURCE_MODE = "E_UNSUPPORTED_SOURCE_MODE"

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

CAMERA_SOURCE_FIELDS = (
    "camera_source_requested",
    "camera_source_status",
    "source_mode",
    "snapshot_id",
    "scene_version",
    "capture_timestamp",
    "frame_id",
    "camera_frame",
    "rgb_ref",
    "image_ref",
    "depth_ref",
    "camera_info_ref",
    "metadata_ref",
    "tf_snapshot_ref",
    "extrinsics_ref",
    "width",
    "height",
    "color_encoding",
    "depth_encoding",
    "depth_available",
    "camera_info_available",
    "metadata_available",
    "extrinsics_available",
    "alignment_status",
    "sync_status",
    "capture_backend",
    "capture_backend_available",
    "capture_method",
    "one_shot_capture_used",
    "continuous_capture_used",
    "live_camera_capture_allowed",
    "live_camera_capture_used",
    "blocking_reasons",
    "warnings",
    "next_safe_action",
    "no_motion_camera_adapter_passed",
    "live_vlm_called",
    "real_robot_motion_executed",
    "real_robot_command_enabled",
    "robot_command_generated",
    "trajectory_generated",
    "joint_targets_generated",
    "tcp_pose_world_generated",
)


@dataclass(frozen=True)
class CameraSourceAdapterRequest:
    requested: bool = False
    config_path: str | None = None
    source_mode: str | None = None
    allow_live_camera_capture: bool = False
    config: Dict[str, Any] | None = None


def load_camera_source_config(path: str | Path | None) -> Dict[str, Any]:
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
    config = data.get("camera_source_adapter") or data.get("camera_source")
    return config if isinstance(config, dict) else data


def build_camera_source_adapter_request(
    *,
    requested: bool = False,
    config_path: str | Path | None = None,
    source_mode: str | None = None,
    allow_live_camera_capture: bool = False,
) -> CameraSourceAdapterRequest:
    config = load_camera_source_config(config_path)
    return CameraSourceAdapterRequest(
        requested=requested,
        config_path=str(Path(config_path).expanduser()) if config_path else None,
        source_mode=source_mode,
        allow_live_camera_capture=allow_live_camera_capture,
        config=config,
    )


def evaluate_camera_source_adapter(request: CameraSourceAdapterRequest | None = None) -> Dict[str, Any]:
    request = request or CameraSourceAdapterRequest()
    if not request.requested:
        return _not_requested_result()
    config = request.config if isinstance(request.config, dict) else {}
    source_mode = request.source_mode or _string(config.get("source_mode")) or MODE_LIVE_DISABLED
    snapshot = _snapshot_from_config(config, source_mode)
    snapshot_contract = evaluate_camera_snapshot_contract(CameraSnapshotRequest(requested=True, snapshot=snapshot))

    blocking_reasons: list[str] = []
    warnings = list(config.get("warnings") or []) if isinstance(config.get("warnings"), list) else []
    allow_live = request.allow_live_camera_capture or config.get("allow_live_camera_capture") is True
    continuous_requested = config.get("continuous_capture_enabled") is True
    capture_backend = _string(config.get("capture_backend")) or _default_backend(source_mode)
    capture_method = _string(config.get("capture_method")) or _default_capture_method(source_mode)
    if source_mode == MODE_REALSENSE_REPLAY:
        capture_backend = "realsense_snapshot_replay"
        capture_method = "realsense_snapshot_replay_manifest"
    capture_backend_available = _backend_available(capture_backend, source_mode)
    one_shot_capture_used = False

    if source_mode not in {
        MODE_OFFLINE_FILE,
        MODE_MANUAL_SNAPSHOT,
        MODE_LIVE_DISABLED,
        MODE_REALSENSE_REPLAY,
        MODE_REALSENSE_ONE_SHOT,
    }:
        blocking_reasons.append(E_UNSUPPORTED_SOURCE_MODE)
    if continuous_requested:
        blocking_reasons.append(E_CONTINUOUS_CAPTURE_DISABLED)
    if source_mode == MODE_REALSENSE_ONE_SHOT:
        if not allow_live:
            blocking_reasons.append(E_LIVE_CAMERA_CAPTURE_NOT_ALLOWED)
        elif not capture_backend_available:
            blocking_reasons.append(E_CAMERA_BACKEND_UNAVAILABLE)
        else:
            one_shot_capture_used = True
    if source_mode in {
        MODE_OFFLINE_FILE,
        MODE_MANUAL_SNAPSHOT,
        MODE_REALSENSE_REPLAY,
        MODE_REALSENSE_ONE_SHOT,
    }:
        if not snapshot.get("image_ref"):
            blocking_reasons.append(E_IMAGE_REF_MISSING)
        if config.get("camera_info_required", True) is not False and not snapshot.get("camera_info_ref"):
            blocking_reasons.append(E_CAMERA_INFO_MISSING)
        if not snapshot.get("frame_id") or not snapshot.get("camera_frame"):
            blocking_reasons.append(E_CAMERA_FRAME_MISSING)
        if not snapshot.get("capture_timestamp"):
            blocking_reasons.append(E_CAPTURE_TIMESTAMP_MISSING)
    if config.get("live_vlm_called") is True or snapshot.get("live_vlm_called") is True:
        blocking_reasons.append(E_LIVE_VLM_DISABLED)
    forbidden_fields = _forbidden_robot_control_fields(config)
    if forbidden_fields:
        blocking_reasons.append(E_ROBOT_COMMAND_NOT_ALLOWED)
        warnings.append(f"forbidden_robot_control_fields={forbidden_fields}")

    if source_mode in {
        MODE_OFFLINE_FILE,
        MODE_MANUAL_SNAPSHOT,
        MODE_REALSENSE_REPLAY,
        MODE_REALSENSE_ONE_SHOT,
    }:
        for reason in snapshot_contract.get("blocking_reasons", []):
            if reason == "E_IMAGE_REF_MISSING":
                blocking_reasons.append(E_IMAGE_REF_MISSING)
            elif reason == "E_CAMERA_FRAME_MISSING":
                blocking_reasons.append(E_CAMERA_FRAME_MISSING)
            elif reason == "E_CAPTURE_TIMESTAMP_MISSING":
                blocking_reasons.append(E_CAPTURE_TIMESTAMP_MISSING)
            else:
                blocking_reasons.append(str(reason))

    blocking_reasons = _unique(blocking_reasons)
    warnings = _unique(warnings)
    status = (
        STATUS_SAFE_DISABLED
        if source_mode == MODE_LIVE_DISABLED and not blocking_reasons
        else STATUS_PASS
        if not blocking_reasons
        else STATUS_BLOCKED
    )
    no_motion_passed = status in {STATUS_PASS, STATUS_SAFE_DISABLED}
    live_capture_used = one_shot_capture_used

    return {
        **_source_fields(snapshot),
        "contract_version": CONTRACT_VERSION,
        "teto_version": CURRENT_CAMERA_SOURCE_VERSION,
        "camera_source_requested": True,
        "requested": True,
        "config_path": request.config_path,
        "camera_source_status": status,
        "source_mode": source_mode,
        "capture_backend": capture_backend,
        "capture_backend_available": capture_backend_available,
        "capture_method": capture_method,
        "one_shot_capture_used": one_shot_capture_used,
        "continuous_capture_used": False,
        "live_camera_capture_allowed": allow_live,
        "live_camera_capture_used": live_capture_used,
        "blocking_reasons": blocking_reasons,
        "warnings": warnings,
        "next_safe_action": _next_safe_action(status, source_mode),
        "no_motion_camera_adapter_passed": no_motion_passed,
        "live_vlm_called": False,
        "real_robot_motion_executed": False,
        "real_robot_command_enabled": False,
        "robot_command_generated": False,
        "trajectory_generated": False,
        "joint_targets_generated": False,
        "tcp_pose_world_generated": False,
        "camera_snapshot": snapshot_contract,
        "snapshot_contract": snapshot,
        "forbidden_robot_control_fields": forbidden_fields,
        "safety_boundary": _safety_boundary(),
    }


def format_camera_source_report(result: Dict[str, Any]) -> str:
    return "\n".join(
        [
            "# TETO V2.9.3 Camera Source Adapter Report",
            "",
            "## Overall Status",
            "",
            f"- camera_source_status: {_format_value(result.get('camera_source_status'))}",
            f"- source_mode: {_format_value(result.get('source_mode'))}",
            f"- snapshot_id: {_format_value(result.get('snapshot_id'))}",
            f"- scene_version: {_format_value(result.get('scene_version'))}",
            f"- no_motion_camera_adapter_passed: {_format_value(result.get('no_motion_camera_adapter_passed'))}",
            f"- blocking_reasons: {_format_value(result.get('blocking_reasons'))}",
            f"- warnings: {_format_value(result.get('warnings'))}",
            f"- next_safe_action: {_format_value(result.get('next_safe_action'))}",
            "",
            "## Camera Source Fields",
            "",
            "| Field | Value |",
            "| --- | --- |",
            *[f"| {field} | {_format_value(result.get(field))} |" for field in CAMERA_SOURCE_FIELDS],
            "",
            "## No-Motion Safety Boundary",
            "",
            "V2.9.3 adapts offline file, manual snapshot, live-disabled, or explicitly allowed one-shot camera source declarations into a TETO camera snapshot contract. It is no-motion, no-live-VLM, no-real-robot, no-ROS2, and no-MoveIt evidence only. It is not a continuous live camera loop, does not call live Qwen or any live VLM, does not connect to a real UR5, does not use ROS2, MoveIt, RTDE, URScript, Dashboard, a trajectory planner, or tcp_pose_world execution, and does not generate joint targets, trajectories, robot commands, or real execution requests.",
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
        "teto_version": CURRENT_CAMERA_SOURCE_VERSION,
        "camera_source_requested": False,
        "requested": False,
        "camera_source_status": STATUS_NOT_REQUESTED,
        "blocking_reasons": [],
        "warnings": [],
        "next_safe_action": None,
        "no_motion_camera_adapter_passed": False,
        "one_shot_capture_used": False,
        "continuous_capture_used": False,
        "live_camera_capture_allowed": False,
        "live_camera_capture_used": False,
        "live_vlm_called": False,
        "real_robot_motion_executed": False,
        "real_robot_command_enabled": False,
        "robot_command_generated": False,
        "trajectory_generated": False,
        "joint_targets_generated": False,
        "tcp_pose_world_generated": False,
        "safety_boundary": _safety_boundary(),
    }


def _snapshot_from_config(config: Dict[str, Any], source_mode: str) -> Dict[str, Any]:
    snapshot = config.get("camera_snapshot") if isinstance(config.get("camera_snapshot"), dict) else {}
    merged = {**config, **snapshot}
    return {
        "snapshot_id": _string(merged.get("snapshot_id")) or f"{source_mode}_snapshot_placeholder_001",
        "scene_version": _string(merged.get("scene_version")) or "camera_source_adapter_scene_v1",
        "capture_timestamp": _string(merged.get("capture_timestamp")) or _timestamp(),
        "ttl_ms": merged.get("ttl_ms", 315360000000),
        "source": source_mode,
        "frame_id": _string(merged.get("frame_id")),
        "rgb_ref": _string(merged.get("rgb_ref")) or _string(merged.get("image_ref")),
        "image_ref": _string(merged.get("rgb_ref")) or _string(merged.get("image_ref")),
        "depth_ref": _string(merged.get("depth_ref")),
        "camera_info_ref": _string(merged.get("camera_info_ref")),
        "metadata_ref": _string(merged.get("metadata_ref")),
        "tf_snapshot_ref": _string(merged.get("tf_snapshot_ref"))
        or _string(merged.get("extrinsics_ref")),
        "extrinsics_ref": _string(merged.get("extrinsics_ref")),
        "width": _optional_int(merged.get("width")),
        "height": _optional_int(merged.get("height")),
        "color_encoding": _string(merged.get("color_encoding")) or "rgb8",
        "depth_encoding": _string(merged.get("depth_encoding")) or "uint16_mm",
        "camera_frame": _string(merged.get("camera_frame")),
        "alignment_status": _string(merged.get("alignment_status")) or "offline_declared",
        "sync_status": _string(merged.get("sync_status")) or "offline_manifest",
        "depth_available": merged.get("depth_available") is True,
        "camera_info_available": merged.get("camera_info_available") is True,
        "metadata_available": merged.get("metadata_available") is True,
        "extrinsics_available": merged.get("extrinsics_available") is True,
        "depth_required": merged.get("depth_required") is True,
        "live_camera_enabled": False,
        "live_vlm_called": merged.get("live_vlm_called") is True,
    }


def _source_fields(snapshot: Dict[str, Any]) -> Dict[str, Any]:
    return {
        "snapshot_id": snapshot.get("snapshot_id"),
        "scene_version": snapshot.get("scene_version"),
        "capture_timestamp": snapshot.get("capture_timestamp"),
        "frame_id": snapshot.get("frame_id"),
        "camera_frame": snapshot.get("camera_frame"),
        "rgb_ref": snapshot.get("rgb_ref") or snapshot.get("image_ref"),
        "image_ref": snapshot.get("image_ref"),
        "depth_ref": snapshot.get("depth_ref"),
        "camera_info_ref": snapshot.get("camera_info_ref"),
        "metadata_ref": snapshot.get("metadata_ref"),
        "tf_snapshot_ref": snapshot.get("tf_snapshot_ref") or snapshot.get("extrinsics_ref"),
        "extrinsics_ref": snapshot.get("extrinsics_ref"),
        "width": snapshot.get("width"),
        "height": snapshot.get("height"),
        "color_encoding": snapshot.get("color_encoding"),
        "depth_encoding": snapshot.get("depth_encoding"),
        "depth_available": snapshot.get("depth_available") is True,
        "camera_info_available": snapshot.get("camera_info_available") is True,
        "metadata_available": snapshot.get("metadata_available") is True,
        "extrinsics_available": snapshot.get("extrinsics_available") is True,
        "alignment_status": snapshot.get("alignment_status"),
        "sync_status": snapshot.get("sync_status"),
    }


def _backend_available(capture_backend: str | None, source_mode: str) -> bool:
    if source_mode != MODE_REALSENSE_ONE_SHOT:
        return True
    if capture_backend != "realsense_one_shot":
        return False
    return importlib.util.find_spec("pyrealsense2") is not None


def _default_backend(source_mode: str) -> str:
    if source_mode == MODE_REALSENSE_ONE_SHOT:
        return "realsense_one_shot"
    if source_mode == MODE_REALSENSE_REPLAY:
        return "realsense_snapshot_replay"
    return source_mode


def _default_capture_method(source_mode: str) -> str:
    if source_mode == MODE_REALSENSE_ONE_SHOT:
        return "one_shot_snapshot_contract_only"
    if source_mode == MODE_REALSENSE_REPLAY:
        return "realsense_snapshot_replay_manifest"
    if source_mode == MODE_LIVE_DISABLED:
        return "disabled_no_capture"
    return "declared_snapshot_manifest"


def _next_safe_action(status: str, source_mode: str) -> str:
    if status == STATUS_PASS:
        return "Use the generated camera snapshot contract only as no-motion evidence."
    if status == STATUS_SAFE_DISABLED:
        return "Live camera capture remains disabled; use offline/manual snapshot evidence."
    return "Fix camera source evidence and rerun without live VLM, continuous capture, or robot control."


def _timestamp() -> str:
    return datetime.now(timezone.utc).replace(microsecond=0).isoformat().replace("+00:00", "Z")


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
        "no_continuous_live_camera_loop": True,
        "live_camera_capture_default_disabled": True,
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


def _string(value: Any) -> str | None:
    return value if isinstance(value, str) and value else None


def _optional_int(value: Any) -> int | None:
    if isinstance(value, bool):
        return None
    return value if isinstance(value, int) else None


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
