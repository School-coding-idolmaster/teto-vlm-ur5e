from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict

import yaml

from src.camera_snapshot import build_camera_snapshot_request, evaluate_camera_snapshot_contract
from src.grounding_result import (
    build_grounding_result_request,
    evaluate_grounding_result_contract,
)


CONTRACT_VERSION = "teto_real_scene_shadow_pipeline.v1"
CURRENT_REAL_SCENE_SHADOW_VERSION = "TETO V2.9.0"

STATUS_SHADOW_ACCEPTED = "SHADOW_ACCEPTED"
STATUS_BLOCKED = "BLOCKED"
STATUS_NOT_REQUESTED = "NOT_REQUESTED"

E_SNAPSHOT_MISMATCH = "E_SNAPSHOT_MISMATCH"
E_SCENE_VERSION_MISMATCH = "E_SCENE_VERSION_MISMATCH"
E_CAMERA_SNAPSHOT_INVALID = "E_CAMERA_SNAPSHOT_INVALID"
E_NO_TARGET = "E_NO_TARGET"
E_INVALID_BBOX = "E_INVALID_BBOX"
E_INVALID_PIXEL_CENTER = "E_INVALID_PIXEL_CENTER"
E_LOW_CONFIDENCE = "E_LOW_CONFIDENCE"
E_LIVE_VLM_DISABLED = "E_LIVE_VLM_DISABLED"
E_LIVE_CAMERA_DISABLED = "E_LIVE_CAMERA_DISABLED"
E_ROBOT_COMMAND_NOT_ALLOWED = "E_ROBOT_COMMAND_NOT_ALLOWED"

DEFAULT_OVERALL_CONFIDENCE_THRESHOLD = 0.6


@dataclass(frozen=True)
class RealSceneShadowRequest:
    requested: bool = False
    config_path: str | None = None
    camera_snapshot_config: str | None = None
    grounding_result_path: str | None = None
    overall_confidence_threshold: float = DEFAULT_OVERALL_CONFIDENCE_THRESHOLD


def load_real_scene_shadow_config(path: str | Path | None) -> Dict[str, Any]:
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
    return data if isinstance(data, dict) else {}


def build_real_scene_shadow_request(
    *,
    requested: bool = False,
    config_path: str | Path | None = None,
    grounding_result_path: str | Path | None = None,
) -> RealSceneShadowRequest:
    config = load_real_scene_shadow_config(config_path)
    camera_snapshot_config = config.get("camera_snapshot_config")
    configured_grounding = grounding_result_path or config.get("grounding_result")
    threshold = _optional_float(config.get("overall_confidence_threshold"))
    return RealSceneShadowRequest(
        requested=requested,
        config_path=str(Path(config_path).expanduser()) if config_path else None,
        camera_snapshot_config=str(Path(camera_snapshot_config).expanduser())
        if isinstance(camera_snapshot_config, str)
        else None,
        grounding_result_path=str(Path(configured_grounding).expanduser())
        if isinstance(configured_grounding, str)
        else None,
        overall_confidence_threshold=threshold
        if threshold is not None
        else DEFAULT_OVERALL_CONFIDENCE_THRESHOLD,
    )


def evaluate_real_scene_shadow_pipeline(request: RealSceneShadowRequest | None = None) -> Dict[str, Any]:
    request = request or RealSceneShadowRequest()
    if not request.requested:
        return _not_requested_result()

    snapshot = evaluate_camera_snapshot_contract(
        build_camera_snapshot_request(requested=True, config_path=request.camera_snapshot_config)
    )
    grounding = evaluate_grounding_result_contract(
        build_grounding_result_request(requested=True, result_path=request.grounding_result_path)
    )
    return evaluate_real_scene_shadow_from_contracts(
        snapshot,
        grounding,
        config_path=request.config_path,
        grounding_result_path=request.grounding_result_path,
        overall_confidence_threshold=request.overall_confidence_threshold,
    )


def evaluate_real_scene_shadow_from_contracts(
    camera_snapshot: Dict[str, Any],
    grounding_result: Dict[str, Any],
    *,
    config_path: str | None = None,
    grounding_result_path: str | None = None,
    overall_confidence_threshold: float = DEFAULT_OVERALL_CONFIDENCE_THRESHOLD,
) -> Dict[str, Any]:
    blocking_reasons: list[str] = []
    warnings = list(grounding_result.get("warnings") or [])

    snapshot_id_match = camera_snapshot.get("snapshot_id") == grounding_result.get("snapshot_id")
    scene_version_match = camera_snapshot.get("scene_version") == grounding_result.get("scene_version")
    width = camera_snapshot.get("width")
    height = camera_snapshot.get("height")
    bbox_valid = _bbox_valid(grounding_result.get("bbox_xyxy"), width, height)
    pixel_center_valid = _pixel_center_valid(grounding_result.get("pixel_center"), width, height)
    overall_confidence = grounding_result.get("overall_confidence")
    confidence_check_passed = (
        isinstance(overall_confidence, (int, float))
        and float(overall_confidence) >= float(overall_confidence_threshold)
    )

    if camera_snapshot.get("validity_status") != "PASS":
        blocking_reasons.append(E_CAMERA_SNAPSHOT_INVALID)
    if not snapshot_id_match:
        blocking_reasons.append(E_SNAPSHOT_MISMATCH)
    if not scene_version_match:
        blocking_reasons.append(E_SCENE_VERSION_MISMATCH)
    if grounding_result.get("grounded") is not True:
        blocking_reasons.append(E_NO_TARGET)
    if grounding_result.get("rejected") is True:
        blocking_reasons.append(
            grounding_result.get("rejection_reason")
            or grounding_result.get("error_code")
            or E_NO_TARGET
        )
    if not bbox_valid:
        blocking_reasons.append(E_INVALID_BBOX)
    if not pixel_center_valid:
        blocking_reasons.append(E_INVALID_PIXEL_CENTER)
    if not confidence_check_passed:
        blocking_reasons.append(E_LOW_CONFIDENCE)
    if grounding_result.get("live_vlm_called") is True:
        blocking_reasons.append(E_LIVE_VLM_DISABLED)
    if grounding_result.get("live_camera_used") is True:
        blocking_reasons.append(E_LIVE_CAMERA_DISABLED)
    if grounding_result.get("forbidden_robot_control_fields"):
        blocking_reasons.append(E_ROBOT_COMMAND_NOT_ALLOWED)

    blocking_reasons = _unique(blocking_reasons)
    status = STATUS_SHADOW_ACCEPTED if not blocking_reasons else STATUS_BLOCKED
    no_motion_shadow_passed = status == STATUS_SHADOW_ACCEPTED
    return {
        "contract_version": CONTRACT_VERSION,
        "teto_version": CURRENT_REAL_SCENE_SHADOW_VERSION,
        "requested": True,
        "config_path": config_path,
        "grounding_result_path": grounding_result_path,
        "camera_snapshot": camera_snapshot,
        "grounding_result": grounding_result,
        "snapshot_id": camera_snapshot.get("snapshot_id"),
        "grounding_id": grounding_result.get("grounding_id"),
        "scene_version": camera_snapshot.get("scene_version"),
        "shadow_pipeline_status": status,
        "semantic_gate_passed": no_motion_shadow_passed,
        "snapshot_scene_version_match": scene_version_match,
        "snapshot_id_match": snapshot_id_match,
        "confidence_check_passed": confidence_check_passed,
        "bbox_valid": bbox_valid,
        "pixel_center_valid": pixel_center_valid,
        "overall_confidence_threshold": overall_confidence_threshold,
        "no_motion_shadow_passed": no_motion_shadow_passed,
        "blocking_reasons": blocking_reasons,
        "warnings": _unique(warnings),
        "next_safe_action": (
            "Replay this accepted real-scene shadow bundle only as no-motion evidence."
            if no_motion_shadow_passed
            else "Fix snapshot/grounding evidence and rerun shadow validation without live capture, live VLM, or robot control."
        ),
        "replay_ready": True,
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


def format_real_scene_shadow_report(result: Dict[str, Any]) -> str:
    return "\n".join(
        [
            "# TETO V2.9.0 Real-Scene No-Motion Shadow Pipeline Report",
            "",
            "## Overall Status",
            "",
            f"- shadow_pipeline_status: {_format_value(result.get('shadow_pipeline_status'))}",
            f"- semantic_gate_passed: {_format_value(result.get('semantic_gate_passed'))}",
            f"- no_motion_shadow_passed: {_format_value(result.get('no_motion_shadow_passed'))}",
            f"- replay_ready: {_format_value(result.get('replay_ready'))}",
            f"- blocking_reasons: {_format_value(result.get('blocking_reasons'))}",
            f"- warnings: {_format_value(result.get('warnings'))}",
            f"- next_safe_action: {_format_value(result.get('next_safe_action'))}",
            "",
            "## Contract Join",
            "",
            "| Field | Value |",
            "| --- | --- |",
            f"| snapshot_id | {_format_value(result.get('snapshot_id'))} |",
            f"| grounding_id | {_format_value(result.get('grounding_id'))} |",
            f"| scene_version | {_format_value(result.get('scene_version'))} |",
            f"| snapshot_id_match | {_format_value(result.get('snapshot_id_match'))} |",
            f"| snapshot_scene_version_match | {_format_value(result.get('snapshot_scene_version_match'))} |",
            f"| confidence_check_passed | {_format_value(result.get('confidence_check_passed'))} |",
            f"| bbox_valid | {_format_value(result.get('bbox_valid'))} |",
            f"| pixel_center_valid | {_format_value(result.get('pixel_center_valid'))} |",
            "",
            "## No-Motion Safety Boundary",
            "",
            "V2.9.0 links an offline/manual camera snapshot contract with an offline/mock VLM grounding result. It does not capture from a live camera, does not call live Qwen or any live VLM, does not connect to a real UR5, does not use ROS2, MoveIt, RTDE, URScript, Dashboard, a trajectory planner, or tcp_pose_world execution, and does not generate joint targets, trajectories, robot commands, or real execution requests.",
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
        "teto_version": CURRENT_REAL_SCENE_SHADOW_VERSION,
        "requested": False,
        "shadow_pipeline_status": STATUS_NOT_REQUESTED,
        "semantic_gate_passed": False,
        "no_motion_shadow_passed": False,
        "blocking_reasons": [],
        "warnings": [],
        "next_safe_action": None,
        "replay_ready": False,
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


def _bbox_valid(value: Any, width: Any, height: Any) -> bool:
    if not isinstance(value, list) or len(value) != 4:
        return False
    if not isinstance(width, int) or not isinstance(height, int):
        return False
    if any(isinstance(item, bool) or not isinstance(item, (int, float)) for item in value):
        return False
    x1, y1, x2, y2 = [float(item) for item in value]
    return 0 <= x1 < x2 <= width and 0 <= y1 < y2 <= height


def _pixel_center_valid(value: Any, width: Any, height: Any) -> bool:
    if not isinstance(value, list) or len(value) != 2:
        return False
    if not isinstance(width, int) or not isinstance(height, int):
        return False
    if any(isinstance(item, bool) or not isinstance(item, (int, float)) for item in value):
        return False
    x, y = [float(item) for item in value]
    return 0 <= x <= width and 0 <= y <= height


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
