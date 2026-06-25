from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict

import yaml

from src.camera_snapshot import (
    CameraSnapshotRequest,
    build_camera_snapshot_request,
    evaluate_camera_snapshot_contract,
)
from src.camera_source_adapter import (
    CameraSourceAdapterRequest,
    evaluate_camera_source_adapter,
    load_camera_source_config,
)
from src.geometry_validity import (
    DEFAULT_CONFIDENCE_THRESHOLD,
    DEFAULT_MIN_BBOX_AREA_PX,
    evaluate_geometry_validity_from_contracts,
    load_geometry_validity_config,
)
from src.projector_shadow import evaluate_projector_shadow_from_contracts, load_projector_shadow_config
from src.real_scene_shadow_pipeline import evaluate_real_scene_shadow_from_contracts
from src.grounding.vlm_adapter import (
    VLMGroundingAdapterRequest,
    evaluate_vlm_grounding_adapter,
    load_vlm_grounding_config,
    normalize_command,
)


CONTRACT_VERSION = "teto_perception_shadow_pipeline.v1"
CURRENT_PERCEPTION_SHADOW_VERSION = "TETO V2.9.5"

STATUS_PASS = "PASS"
STATUS_BLOCKED = "BLOCKED"
STATUS_NOT_REQUESTED = "NOT_REQUESTED"

E_CAMERA_SOURCE_BLOCKED = "E_CAMERA_SOURCE_BLOCKED"
E_CAMERA_SNAPSHOT_INVALID = "E_CAMERA_SNAPSHOT_INVALID"
E_VLM_GROUNDING_BLOCKED = "E_VLM_GROUNDING_BLOCKED"
E_SEMANTIC_GATE_BLOCKED = "E_SEMANTIC_GATE_BLOCKED"
E_GEOMETRY_VALIDITY_BLOCKED = "E_GEOMETRY_VALIDITY_BLOCKED"
E_PROJECTOR_BLOCKED = "E_PROJECTOR_BLOCKED"
E_PIPELINE_ID_MISMATCH = "E_PIPELINE_ID_MISMATCH"
E_SCENE_VERSION_MISMATCH = "E_SCENE_VERSION_MISMATCH"
E_LIVE_CAMERA_DISABLED = "E_LIVE_CAMERA_DISABLED"
E_LIVE_VLM_DISABLED = "E_LIVE_VLM_DISABLED"
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

PERCEPTION_SHADOW_FIELDS = (
    "perception_shadow_requested",
    "perception_shadow_status",
    "user_command",
    "normalized_command",
    "snapshot_id",
    "grounding_id",
    "scene_version",
    "camera_source_status",
    "camera_snapshot_validity_status",
    "vlm_grounding_status",
    "real_scene_shadow_status",
    "semantic_gate_passed",
    "geometry_validity_status",
    "projector_status",
    "bbox_xyxy",
    "pixel_center",
    "target_label",
    "target_object_id",
    "semantic_confidence",
    "grounding_confidence",
    "overall_confidence",
    "depth_value_m",
    "camera_point_m",
    "world_point_m",
    "world_frame",
    "camera_frame",
    "workspace_check_passed",
    "replay_ready",
    "blocking_reasons",
    "warnings",
    "next_safe_action",
    "no_motion_perception_passed",
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
class PerceptionShadowPipelineRequest:
    requested: bool = False
    config_path: str | None = None
    user_command: str | None = None
    camera_source_config: str | None = None
    camera_snapshot_config: str | None = None
    vlm_grounding_config: str | None = None
    geometry_validity_config: str | None = None
    projector_shadow_config: str | None = None
    config: Dict[str, Any] | None = None


def load_perception_shadow_config(path: str | Path | None) -> Dict[str, Any]:
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
    config = data.get("perception_shadow_pipeline") or data.get("perception_shadow")
    return config if isinstance(config, dict) else data


def build_perception_shadow_request(
    *,
    requested: bool = False,
    config_path: str | Path | None = None,
    user_command: str | None = None,
    camera_source_config: str | Path | None = None,
    camera_snapshot_config: str | Path | None = None,
    vlm_grounding_config: str | Path | None = None,
    geometry_validity_config: str | Path | None = None,
    projector_shadow_config: str | Path | None = None,
) -> PerceptionShadowPipelineRequest:
    config = load_perception_shadow_config(config_path)
    return PerceptionShadowPipelineRequest(
        requested=requested,
        config_path=str(Path(config_path).expanduser()) if config_path else None,
        user_command=user_command,
        camera_source_config=_configured_path(camera_source_config or config.get("camera_source_config")),
        camera_snapshot_config=_configured_path(camera_snapshot_config or config.get("camera_snapshot_config")),
        vlm_grounding_config=_configured_path(vlm_grounding_config or config.get("vlm_grounding_config")),
        geometry_validity_config=_configured_path(geometry_validity_config or config.get("geometry_validity_config")),
        projector_shadow_config=_configured_path(projector_shadow_config or config.get("projector_shadow_config")),
        config=config,
    )


def evaluate_perception_shadow_pipeline(
    request: PerceptionShadowPipelineRequest | None = None,
) -> Dict[str, Any]:
    request = request or PerceptionShadowPipelineRequest()
    if not request.requested:
        return _not_requested_result()

    config = request.config if isinstance(request.config, dict) else {}
    user_command = request.user_command or _string(config.get("user_command"))
    normalized_command = normalize_command(user_command)
    warnings: list[str] = []
    blocking_reasons: list[str] = []

    camera_source_config = _nested_config(config, "camera_source_adapter") or load_camera_source_config(
        request.camera_source_config
    )
    camera_source = evaluate_camera_source_adapter(
        CameraSourceAdapterRequest(
            requested=True,
            config_path=request.camera_source_config,
            config=camera_source_config,
        )
    )
    if camera_source.get("camera_source_status") != STATUS_PASS:
        blocking_reasons.extend(_stage_reasons(camera_source, E_CAMERA_SOURCE_BLOCKED))
    warnings.extend(_list(camera_source.get("warnings")))

    camera_snapshot = _evaluate_snapshot(request, config, camera_source)
    if camera_snapshot.get("validity_status") != STATUS_PASS:
        blocking_reasons.extend(_stage_reasons(camera_snapshot, E_CAMERA_SNAPSHOT_INVALID))
    warnings.extend(_list(camera_snapshot.get("warnings")))

    vlm_config = _nested_config(config, "vlm_grounding_adapter") or load_vlm_grounding_config(
        request.vlm_grounding_config
    )
    vlm_grounding = evaluate_vlm_grounding_adapter(
        VLMGroundingAdapterRequest(
            requested=True,
            config_path=request.vlm_grounding_config,
            user_command=user_command,
            config=vlm_config,
        )
    )
    if vlm_grounding.get("vlm_grounding_status") != STATUS_PASS:
        blocking_reasons.extend(_stage_reasons(vlm_grounding, E_VLM_GROUNDING_BLOCKED))
    warnings.extend(_list(vlm_grounding.get("warnings")))

    real_scene_shadow = evaluate_real_scene_shadow_from_contracts(
        camera_snapshot,
        vlm_grounding,
        config_path=request.config_path,
    )
    if real_scene_shadow.get("shadow_pipeline_status") != "SHADOW_ACCEPTED":
        blocking_reasons.extend(_stage_reasons(real_scene_shadow, E_SEMANTIC_GATE_BLOCKED))
    warnings.extend(_list(real_scene_shadow.get("warnings")))

    geometry_config = _nested_config(config, "geometry_validity") or load_geometry_validity_config(
        request.geometry_validity_config
    )
    thresholds = geometry_config.get("thresholds") if isinstance(geometry_config.get("thresholds"), dict) else {}
    geometry = evaluate_geometry_validity_from_contracts(
        camera_snapshot,
        vlm_grounding,
        config_path=request.geometry_validity_config,
        confidence_threshold=_optional_float(
            thresholds.get("confidence_threshold", geometry_config.get("confidence_threshold"))
        )
        or DEFAULT_CONFIDENCE_THRESHOLD,
        min_bbox_area_px=_optional_float(thresholds.get("min_bbox_area_px", geometry_config.get("min_bbox_area_px")))
        or DEFAULT_MIN_BBOX_AREA_PX,
        depth_required=geometry_config.get("depth_required")
        if isinstance(geometry_config.get("depth_required"), bool)
        else None,
    )
    if geometry.get("geometry_validity_status") != STATUS_PASS:
        blocking_reasons.extend(_stage_reasons(geometry, E_GEOMETRY_VALIDITY_BLOCKED))
    warnings.extend(_list(geometry.get("warnings")))

    projector_config = _nested_config(config, "projector_shadow") or load_projector_shadow_config(
        request.projector_shadow_config
    )
    projector = evaluate_projector_shadow_from_contracts(
        geometry,
        projector_config=projector_config,
        config_path=request.projector_shadow_config,
        geometry_validity_config=request.geometry_validity_config,
    )
    if projector.get("projector_status") != STATUS_PASS:
        blocking_reasons.extend(_stage_reasons(projector, E_PROJECTOR_BLOCKED))
    warnings.extend(_list(projector.get("warnings")))

    blocking_reasons.extend(_identity_mismatch_reasons(camera_source, camera_snapshot, vlm_grounding, geometry, projector))
    if _any_flag(
        "live_camera_used",
        camera_source,
        camera_snapshot,
        vlm_grounding,
        real_scene_shadow,
        geometry,
        projector,
    ) or camera_source.get("live_camera_capture_used") is True or camera_snapshot.get("live_capture_used") is True:
        blocking_reasons.append(E_LIVE_CAMERA_DISABLED)
    if _any_flag("live_vlm_called", camera_source, camera_snapshot, vlm_grounding, real_scene_shadow, geometry, projector):
        blocking_reasons.append(E_LIVE_VLM_DISABLED)

    forbidden_fields = _unique(
        _forbidden_robot_control_fields(config)
        + _forbidden_from_contracts(camera_source, camera_snapshot, vlm_grounding, real_scene_shadow, geometry, projector)
    )
    if forbidden_fields:
        blocking_reasons.append(E_ROBOT_COMMAND_NOT_ALLOWED)
        warnings.append(f"forbidden_robot_control_fields={forbidden_fields}")

    blocking_reasons = _unique([str(reason) for reason in blocking_reasons if reason])
    warnings = _unique([str(warning) for warning in warnings if warning])
    status = STATUS_PASS if not blocking_reasons else STATUS_BLOCKED
    no_motion_passed = status == STATUS_PASS

    return {
        "contract_version": CONTRACT_VERSION,
        "teto_version": CURRENT_PERCEPTION_SHADOW_VERSION,
        "perception_shadow_requested": True,
        "requested": True,
        "config_path": request.config_path,
        "perception_shadow_status": status,
        "user_command": user_command,
        "normalized_command": normalized_command,
        "snapshot_id": _first_value(projector, geometry, real_scene_shadow, camera_snapshot, key="snapshot_id"),
        "grounding_id": _first_value(projector, geometry, real_scene_shadow, vlm_grounding, key="grounding_id"),
        "scene_version": _first_value(projector, geometry, real_scene_shadow, camera_snapshot, key="scene_version"),
        "camera_source_status": camera_source.get("camera_source_status"),
        "camera_snapshot_validity_status": camera_snapshot.get("validity_status"),
        "vlm_grounding_status": vlm_grounding.get("vlm_grounding_status"),
        "real_scene_shadow_status": real_scene_shadow.get("shadow_pipeline_status"),
        "semantic_gate_passed": real_scene_shadow.get("semantic_gate_passed") is True,
        "geometry_validity_status": geometry.get("geometry_validity_status"),
        "projector_status": projector.get("projector_status"),
        "bbox_xyxy": vlm_grounding.get("bbox_xyxy"),
        "pixel_center": projector.get("pixel_center") or vlm_grounding.get("pixel_center"),
        "target_label": vlm_grounding.get("target_label"),
        "target_object_id": vlm_grounding.get("target_object_id"),
        "semantic_confidence": vlm_grounding.get("semantic_confidence"),
        "grounding_confidence": vlm_grounding.get("grounding_confidence"),
        "overall_confidence": vlm_grounding.get("overall_confidence"),
        "depth_value_m": projector.get("depth_value_m"),
        "camera_point_m": projector.get("camera_point_m"),
        "world_point_m": projector.get("world_point_m"),
        "world_frame": projector.get("world_frame"),
        "camera_frame": projector.get("camera_frame") or camera_snapshot.get("camera_frame"),
        "workspace_check_passed": projector.get("workspace_check_passed"),
        "replay_ready": bool(no_motion_passed and real_scene_shadow.get("replay_ready") is True),
        "blocking_reasons": blocking_reasons,
        "warnings": warnings,
        "next_safe_action": _next_safe_action(status),
        "no_motion_perception_passed": no_motion_passed,
        "live_camera_used": False,
        "live_vlm_called": False,
        "real_robot_motion_executed": False,
        "real_robot_command_enabled": False,
        "robot_command_generated": False,
        "trajectory_generated": False,
        "joint_targets_generated": False,
        "tcp_pose_world_generated": False,
        "camera_source": camera_source,
        "camera_snapshot": camera_snapshot,
        "vlm_grounding": vlm_grounding,
        "real_scene_shadow": real_scene_shadow,
        "geometry_validity": geometry,
        "projector_shadow": projector,
        "forbidden_robot_control_fields": forbidden_fields,
        "safety_boundary": _safety_boundary(),
    }


def format_perception_shadow_report(result: Dict[str, Any]) -> str:
    return "\n".join(
        [
            "# TETO V2.9.5 Full Perception Shadow Pipeline Report",
            "",
            "## Overall Status",
            "",
            f"- perception_shadow_status: {_format_value(result.get('perception_shadow_status'))}",
            f"- user_command: {_format_value(result.get('user_command'))}",
            f"- normalized_command: {_format_value(result.get('normalized_command'))}",
            f"- snapshot_id: {_format_value(result.get('snapshot_id'))}",
            f"- grounding_id: {_format_value(result.get('grounding_id'))}",
            f"- scene_version: {_format_value(result.get('scene_version'))}",
            f"- semantic_gate_passed: {_format_value(result.get('semantic_gate_passed'))}",
            f"- no_motion_perception_passed: {_format_value(result.get('no_motion_perception_passed'))}",
            f"- replay_ready: {_format_value(result.get('replay_ready'))}",
            f"- blocking_reasons: {_format_value(result.get('blocking_reasons'))}",
            f"- warnings: {_format_value(result.get('warnings'))}",
            f"- next_safe_action: {_format_value(result.get('next_safe_action'))}",
            "",
            "## Stage Status",
            "",
            "| Stage | Status |",
            "| --- | --- |",
            f"| camera_source | {_format_value(result.get('camera_source_status'))} |",
            f"| camera_snapshot | {_format_value(result.get('camera_snapshot_validity_status'))} |",
            f"| vlm_grounding | {_format_value(result.get('vlm_grounding_status'))} |",
            f"| real_scene_shadow | {_format_value(result.get('real_scene_shadow_status'))} |",
            f"| geometry_validity | {_format_value(result.get('geometry_validity_status'))} |",
            f"| projector_shadow | {_format_value(result.get('projector_status'))} |",
            "",
            "## Perception Shadow Fields",
            "",
            "| Field | Value |",
            "| --- | --- |",
            *[f"| {field} | {_format_value(result.get(field))} |" for field in PERCEPTION_SHADOW_FIELDS],
            "",
            "## No-Motion Safety Boundary",
            "",
            "V2.9.5 composes text command, offline/manual camera source, camera snapshot validation, mock/offline/manual VLM grounding, semantic gate, geometry validity, and 2D-to-3D projector shadow into world_point_m evidence. It is no-motion, no-live-camera, no-live-VLM, no-real-robot, no-ROS2, and no-MoveIt evidence only. It is not a ROS2 bridge, not MoveIt planning, and not real UR5 execution. It does not call live Qwen or any live VLM, does not open a continuous live camera loop, does not use RTDE, URScript, Dashboard, a trajectory planner, or tcp_pose_world execution, and does not generate joint targets, trajectories, robot commands, automatic retry motion, or real execution requests.",
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
        "teto_version": CURRENT_PERCEPTION_SHADOW_VERSION,
        "perception_shadow_requested": False,
        "requested": False,
        "perception_shadow_status": STATUS_NOT_REQUESTED,
        "semantic_gate_passed": False,
        "no_motion_perception_passed": False,
        "replay_ready": False,
        "blocking_reasons": [],
        "warnings": [],
        "next_safe_action": None,
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


def _evaluate_snapshot(
    request: PerceptionShadowPipelineRequest,
    config: Dict[str, Any],
    camera_source: Dict[str, Any],
) -> Dict[str, Any]:
    inline_snapshot = _nested_config(config, "camera_snapshot")
    if inline_snapshot:
        return evaluate_camera_snapshot_contract(CameraSnapshotRequest(requested=True, snapshot=inline_snapshot))
    if request.camera_snapshot_config:
        return evaluate_camera_snapshot_contract(
            build_camera_snapshot_request(requested=True, config_path=request.camera_snapshot_config)
        )
    snapshot = camera_source.get("snapshot_contract") if isinstance(camera_source.get("snapshot_contract"), dict) else {}
    return evaluate_camera_snapshot_contract(CameraSnapshotRequest(requested=True, snapshot=snapshot))


def _identity_mismatch_reasons(*contracts: Dict[str, Any]) -> list[str]:
    snapshot_ids = _unique([str(contract.get("snapshot_id")) for contract in contracts if contract.get("snapshot_id")])
    scene_versions = _unique([str(contract.get("scene_version")) for contract in contracts if contract.get("scene_version")])
    reasons: list[str] = []
    if len(snapshot_ids) > 1:
        reasons.append(E_PIPELINE_ID_MISMATCH)
    if len(scene_versions) > 1:
        reasons.append(E_SCENE_VERSION_MISMATCH)
    return reasons


def _stage_reasons(contract: Dict[str, Any], fallback: str) -> list[str]:
    reasons = _list(contract.get("blocking_reasons"))
    return reasons or [fallback]


def _forbidden_from_contracts(*contracts: Dict[str, Any]) -> list[str]:
    found: list[str] = []
    for contract in contracts:
        found.extend(_list(contract.get("forbidden_robot_control_fields")))
    return found


def _any_flag(key: str, *contracts: Dict[str, Any]) -> bool:
    return any(contract.get(key) is True for contract in contracts)


def _first_value(*contracts: Dict[str, Any], key: str) -> Any:
    for contract in contracts:
        value = contract.get(key)
        if value is not None:
            return value
    return None


def _nested_config(config: Dict[str, Any], key: str) -> Dict[str, Any]:
    value = config.get(key)
    return value if isinstance(value, dict) else {}


def _configured_path(value: Any) -> str | None:
    return str(Path(value).expanduser()) if isinstance(value, (str, Path)) and str(value) else None


def _safety_boundary() -> Dict[str, bool]:
    return {
        "no_live_camera_capture": True,
        "no_continuous_live_camera_loop": True,
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
        "no_real_robot_backend": True,
        "no_real_execution_request": True,
        "no_automatic_retry_motion": True,
    }


def _next_safe_action(status: str) -> str:
    if status == STATUS_PASS:
        return "Use this full perception shadow bundle only as no-motion world_point_m evidence."
    return "Fix the blocked perception stage and rerun without live camera, live VLM, ROS2, MoveIt, or robot control."


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


def _list(value: Any) -> list[str]:
    return [str(item) for item in value] if isinstance(value, list) else []


def _string(value: Any) -> str | None:
    return value if isinstance(value, str) and value else None


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
