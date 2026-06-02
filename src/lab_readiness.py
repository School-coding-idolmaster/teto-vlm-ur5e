from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict

import yaml


CONTRACT_VERSION = "teto_lab_readiness.v1"
CURRENT_READINESS_VERSION = "TETO V2.8.1"

STATUS_READY_FOR_SHADOW_MODE = "READY_FOR_SHADOW_MODE"
STATUS_CONFIG_ONLY = "CONFIG_ONLY"
STATUS_NOT_READY = "NOT_READY"
STATUS_BLOCKED = "BLOCKED"
STATUS_DISABLED = "DISABLED"
STATUS_NOT_REQUESTED = "NOT_REQUESTED"

E_MISSING_LAB_BACKEND_CONFIG = "E_MISSING_LAB_BACKEND_CONFIG"
E_REAL_ROBOT_MOTION_NOT_ALLOWED = "E_REAL_ROBOT_MOTION_NOT_ALLOWED"
E_LIVE_CAMERA_DISABLED = "E_LIVE_CAMERA_DISABLED"
E_LIVE_VLM_DISABLED = "E_LIVE_VLM_DISABLED"
E_MISSING_CAMERA_EXTRINSICS = "E_MISSING_CAMERA_EXTRINSICS"
E_MISSING_TCP_CALIBRATION = "E_MISSING_TCP_CALIBRATION"
E_MISSING_ROBOT_CALIBRATION = "E_MISSING_ROBOT_CALIBRATION"
E_MISSING_SAFETY_STATUS_CHECK = "E_MISSING_SAFETY_STATUS_CHECK"
E_MISSING_SPEED_SCALING_CHECK = "E_MISSING_SPEED_SCALING_CHECK"
E_SHADOW_MODE_NOT_ENABLED = "E_SHADOW_MODE_NOT_ENABLED"

NO_MOTION_SAFETY_BOUNDARY = {
    "no_real_ur5_connection": True,
    "no_ros2": True,
    "no_moveit": True,
    "no_rtde": True,
    "no_urscript": True,
    "no_dashboard": True,
    "no_real_robot_command": True,
    "no_trajectory": True,
    "no_tcp_pose_world": True,
    "no_live_camera_capture": True,
    "no_live_vlm_call": True,
    "no_robot_motion": True,
    "no_automatic_retry_motion": True,
}


@dataclass(frozen=True)
class LabBackendReadinessRequest:
    requested: bool = False
    config_path: str | None = None
    config: Dict[str, Any] | None = None
    check_lab_backend: bool = False
    check_camera: bool = False
    check_live_vlm: bool = False
    check_shadow_mode: bool = False
    permit_live_camera: bool = False
    permit_live_vlm: bool = False
    permit_real_robot_backend: bool = False
    permit_robot_motion: bool = False


@dataclass(frozen=True)
class LabBackendReadinessResult:
    requested: bool
    status: str
    blocking_reasons: list[str]
    warnings: list[str]
    fields: Dict[str, Any]


@dataclass(frozen=True)
class CameraReadinessResult:
    requested: bool
    status: str
    blocking_reasons: list[str]
    warnings: list[str]
    fields: Dict[str, Any]


@dataclass(frozen=True)
class LiveVLMReadinessResult:
    requested: bool
    status: str
    blocking_reasons: list[str]
    warnings: list[str]
    fields: Dict[str, Any]


@dataclass(frozen=True)
class ShadowModeReadinessResult:
    requested: bool
    status: str
    blocking_reasons: list[str]
    warnings: list[str]
    fields: Dict[str, Any]


def load_lab_readiness_config(path: str | Path | None) -> Dict[str, Any] | None:
    if not path:
        return None
    resolved_path = Path(path).expanduser()
    if not resolved_path.is_file():
        return None
    with resolved_path.open("r", encoding="utf-8") as config_file:
        data = yaml.safe_load(config_file)
    return data if isinstance(data, dict) else {}


def evaluate_lab_backend_readiness(
    request: LabBackendReadinessRequest | None = None,
) -> Dict[str, Any]:
    request = request or LabBackendReadinessRequest()
    if not request.check_lab_backend:
        return _result(
            requested=False,
            status=STATUS_NOT_REQUESTED,
            fields=_lab_backend_fields({}),
        )

    config = _section(request.config, "lab_backend")
    fields = _lab_backend_fields(config)
    blocking_reasons: list[str] = []
    warnings: list[str] = []
    if request.config is None:
        blocking_reasons.append(E_MISSING_LAB_BACKEND_CONFIG)
        status = STATUS_CONFIG_ONLY
    else:
        if fields["allow_robot_motion"] or fields["real_robot_command_enabled"]:
            blocking_reasons.append(E_REAL_ROBOT_MOTION_NOT_ALLOWED)
        if fields.get("allow_real_robot_backend") and not request.permit_real_robot_backend:
            blocking_reasons.append(E_REAL_ROBOT_MOTION_NOT_ALLOWED)
        if not fields["tcp_calibration_configured"]:
            blocking_reasons.append(E_MISSING_TCP_CALIBRATION)
        if not fields["robot_calibration_configured"]:
            blocking_reasons.append(E_MISSING_ROBOT_CALIBRATION)
        if not fields["safety_status_check_available"]:
            blocking_reasons.append(E_MISSING_SAFETY_STATUS_CHECK)
        if not fields["speed_scaling_check_available"]:
            blocking_reasons.append(E_MISSING_SPEED_SCALING_CHECK)
        status = STATUS_READY_FOR_SHADOW_MODE if not blocking_reasons else STATUS_BLOCKED
    return _result(
        requested=True,
        status=status,
        blocking_reasons=blocking_reasons,
        warnings=warnings,
        fields=fields,
    )


def evaluate_camera_readiness(
    request: LabBackendReadinessRequest | None = None,
) -> Dict[str, Any]:
    request = request or LabBackendReadinessRequest()
    if not request.check_camera:
        return _result(
            requested=False,
            status=STATUS_NOT_REQUESTED,
            fields=_camera_fields({}),
        )

    config = _section(request.config, "camera")
    fields = _camera_fields(config)
    blocking_reasons: list[str] = []
    warnings: list[str] = []
    if request.config is None:
        status = STATUS_CONFIG_ONLY
    else:
        if fields["allow_live_camera"] and not request.permit_live_camera:
            blocking_reasons.append(E_LIVE_CAMERA_DISABLED)
        if not fields["camera_configured"]:
            warnings.append("W_CAMERA_CONFIG_MISSING")
        if not fields["extrinsics_configured"]:
            blocking_reasons.append(E_MISSING_CAMERA_EXTRINSICS)
        status = STATUS_READY_FOR_SHADOW_MODE if not blocking_reasons else STATUS_BLOCKED
    return _result(
        requested=True,
        status=status,
        blocking_reasons=blocking_reasons,
        warnings=warnings,
        fields=fields,
    )


def evaluate_live_vlm_readiness(
    request: LabBackendReadinessRequest | None = None,
) -> Dict[str, Any]:
    request = request or LabBackendReadinessRequest()
    if not request.check_live_vlm:
        return _result(
            requested=False,
            status=STATUS_NOT_REQUESTED,
            fields=_live_vlm_fields({}),
        )

    config = _section(request.config, "live_vlm")
    fields = _live_vlm_fields(config)
    blocking_reasons: list[str] = []
    warnings: list[str] = []
    if request.config is None:
        status = STATUS_CONFIG_ONLY
    else:
        if fields["allow_live_vlm"] and not request.permit_live_vlm:
            blocking_reasons.append(E_LIVE_VLM_DISABLED)
        if not fields["endpoint_configured"]:
            warnings.append("W_VLM_ENDPOINT_MISSING")
        status = STATUS_READY_FOR_SHADOW_MODE if not blocking_reasons else STATUS_BLOCKED
    return _result(
        requested=True,
        status=status,
        blocking_reasons=blocking_reasons,
        warnings=warnings,
        fields=fields,
    )


def evaluate_shadow_mode_readiness(
    request: LabBackendReadinessRequest | None = None,
) -> Dict[str, Any]:
    request = request or LabBackendReadinessRequest()
    if not request.check_shadow_mode:
        return _result(
            requested=False,
            status=STATUS_NOT_REQUESTED,
            fields=_shadow_fields({}),
        )

    config = _section(request.config, "shadow_mode")
    fields = _shadow_fields(config)
    blocking_reasons: list[str] = []
    if request.config is None:
        status = STATUS_CONFIG_ONLY
    else:
        if not fields["shadow_mode_enabled"]:
            blocking_reasons.append(E_SHADOW_MODE_NOT_ENABLED)
        if not fields["no_motion_enforced"]:
            blocking_reasons.append(E_REAL_ROBOT_MOTION_NOT_ALLOWED)
        status = STATUS_READY_FOR_SHADOW_MODE if not blocking_reasons else STATUS_NOT_READY
    return _result(
        requested=True,
        status=status,
        blocking_reasons=blocking_reasons,
        fields=fields,
    )


def evaluate_lab_readiness(request: LabBackendReadinessRequest) -> Dict[str, Any]:
    lab = evaluate_lab_backend_readiness(request)
    camera = evaluate_camera_readiness(request)
    live_vlm = evaluate_live_vlm_readiness(request)
    shadow = evaluate_shadow_mode_readiness(request)
    requested = any(
        component.get("requested") is True
        for component in (lab, camera, live_vlm, shadow)
    )
    statuses = [
        component.get("status")
        for component in (lab, camera, live_vlm, shadow)
        if component.get("requested") is True
    ]
    blocking_reasons = _unique(
        list(lab.get("blocking_reasons") or [])
        + list(camera.get("blocking_reasons") or [])
        + list(live_vlm.get("blocking_reasons") or [])
        + list(shadow.get("blocking_reasons") or [])
    )
    no_motion_passed = bool(requested and statuses and all(status == STATUS_READY_FOR_SHADOW_MODE for status in statuses))
    if not requested:
        status = STATUS_NOT_REQUESTED
    elif statuses and all(component_status == STATUS_CONFIG_ONLY for component_status in statuses):
        status = STATUS_CONFIG_ONLY
    elif blocking_reasons:
        status = STATUS_BLOCKED
    elif no_motion_passed:
        status = STATUS_READY_FOR_SHADOW_MODE
    elif STATUS_CONFIG_ONLY in statuses:
        status = STATUS_CONFIG_ONLY
    else:
        status = STATUS_NOT_READY
    next_safe_action = (
        "Provide local no-motion readiness config and rerun config-only checks."
        if status == STATUS_CONFIG_ONLY
        else (
            "Resolve blocking readiness fields while keeping all live and motion flags disabled."
            if blocking_reasons
            else "Proceed only with shadow-mode evidence review; V3.0 is required before any first real UR5 small motion."
        )
    )
    return {
        "contract_version": CONTRACT_VERSION,
        "teto_version": CURRENT_READINESS_VERSION,
        "requested": requested,
        "config_path": request.config_path,
        "status": status,
        "readiness_statuses": {
            "lab_backend": _evidence_status(lab["status"]),
            "camera": _evidence_status(camera["status"]),
            "live_vlm": _evidence_status(live_vlm["status"]),
            "shadow_mode": _evidence_status(shadow["status"]),
        },
        "lab_backend_readiness_status": lab["status"],
        "camera_readiness_status": camera["status"],
        "live_vlm_readiness_status": live_vlm["status"],
        "shadow_mode_readiness_status": shadow["status"],
        "no_motion_readiness_passed": no_motion_passed,
        "allow_robot_motion": lab["fields"].get("allow_robot_motion", False) is True,
        "allow_live_camera": camera["fields"].get("allow_live_camera", False) is True,
        "allow_live_vlm": live_vlm["fields"].get("allow_live_vlm", False) is True,
        "real_robot_command_enabled": lab["fields"].get("real_robot_command_enabled", False) is True,
        "real_robot_motion_executed": False,
        "live_camera_used": False,
        "live_vlm_called": False,
        "blocking_reasons": blocking_reasons,
        "warnings": _unique(
            list(lab.get("warnings") or [])
            + list(camera.get("warnings") or [])
            + list(live_vlm.get("warnings") or [])
            + list(shadow.get("warnings") or [])
        ),
        "next_safe_action": next_safe_action,
        "safety_flags": _safety_flags(
            allow_robot_motion=lab["fields"].get("allow_robot_motion", False) is True,
            allow_live_camera=camera["fields"].get("allow_live_camera", False) is True,
            allow_live_vlm=live_vlm["fields"].get("allow_live_vlm", False) is True,
            real_robot_command_enabled=lab["fields"].get("real_robot_command_enabled", False) is True,
        ),
        "safety_boundary": dict(NO_MOTION_SAFETY_BOUNDARY),
        "lab_backend": lab,
        "camera": camera,
        "live_vlm": live_vlm,
        "shadow_mode": shadow,
    }


def format_lab_readiness_report(result: Dict[str, Any]) -> str:
    lab = _component(result, "lab_backend")
    camera = _component(result, "camera")
    live_vlm = _component(result, "live_vlm")
    shadow = _component(result, "shadow_mode")
    safety_flags = result.get("safety_flags") if isinstance(result.get("safety_flags"), dict) else {}
    return "\n".join(
        [
            "# TETO V2.8.1 Readiness Evidence Polish Report",
            "",
            "## Version",
            "",
            f"- TETO version: {_format_value(result.get('teto_version'))}",
            f"- contract_version: {_format_value(result.get('contract_version'))}",
            f"- config_path: {_format_value(result.get('config_path'))}",
            "",
            "## Overall Status",
            "",
            f"- overall_status: {_format_value(result.get('status'))}",
            f"- no_motion_readiness_passed: {_format_value(result.get('no_motion_readiness_passed'))}",
            f"- readiness_statuses: {_format_value(result.get('readiness_statuses'))}",
            f"- blocking_reasons: {_format_value(result.get('blocking_reasons'))}",
            f"- warnings: {_format_value(result.get('warnings'))}",
            f"- next_safe_action: {_format_value(result.get('next_safe_action'))}",
            "",
            "## Readiness Contracts",
            "",
            "| Contract | Evidence Status | Raw Status | Blocking Reasons | Warnings |",
            "| --- | --- | --- | --- | --- |",
            _component_row("Lab Backend", lab),
            _component_row("Camera", camera),
            _component_row("Live VLM", live_vlm),
            _component_row("Shadow Mode", shadow),
            "",
            "## Key Readiness Fields",
            "",
            "| Field | Value |",
            "| --- | --- |",
            f"| lab_backend_readiness_status | {_format_value(result.get('lab_backend_readiness_status'))} |",
            f"| camera_readiness_status | {_format_value(result.get('camera_readiness_status'))} |",
            f"| live_vlm_readiness_status | {_format_value(result.get('live_vlm_readiness_status'))} |",
            f"| shadow_mode_readiness_status | {_format_value(result.get('shadow_mode_readiness_status'))} |",
            f"| allow_robot_motion | {_format_value(result.get('allow_robot_motion'))} |",
            f"| allow_live_camera | {_format_value(result.get('allow_live_camera'))} |",
            f"| allow_live_vlm | {_format_value(result.get('allow_live_vlm'))} |",
            f"| real_robot_command_enabled | {_format_value(result.get('real_robot_command_enabled'))} |",
            f"| real_robot_motion_executed | {_format_value(result.get('real_robot_motion_executed', False))} |",
            f"| live_camera_used | {_format_value(result.get('live_camera_used', False))} |",
            f"| live_vlm_called | {_format_value(result.get('live_vlm_called', False))} |",
            "",
            "## No-Motion Safety Boundary",
            "",
            "V2.8.1 is readiness evidence polish only. It is config-only and shadow-mode preparation; it does not connect to a real UR5, does not capture from a live camera, does not call live Qwen or any live VLM, does not generate real robot commands, does not generate trajectories, and does not execute tcp_pose_world.",
            "",
            "| Safety Flag | Value |",
            "| --- | --- |",
            *[
                f"| {key} | {_format_value(value)} |"
                for key, value in sorted(safety_flags.items())
            ],
            "",
            "Required false flags: allow_live_camera=false, allow_live_vlm=false, real_robot_command_enabled=false, real_robot_motion_executed=false.",
            "",
        ]
    )


def build_lab_readiness_request(
    *,
    check_lab_backend: bool = False,
    check_camera: bool = False,
    check_live_vlm: bool = False,
    check_shadow_mode: bool = False,
    config_path: str | Path | None = None,
    permit_live_camera: bool = False,
    permit_live_vlm: bool = False,
    permit_real_robot_backend: bool = False,
    permit_robot_motion: bool = False,
) -> LabBackendReadinessRequest:
    config = load_lab_readiness_config(config_path)
    requested = check_lab_backend or check_camera or check_live_vlm or check_shadow_mode
    return LabBackendReadinessRequest(
        requested=requested,
        config_path=str(Path(config_path).expanduser()) if config_path else None,
        config=config,
        check_lab_backend=check_lab_backend,
        check_camera=check_camera,
        check_live_vlm=check_live_vlm,
        check_shadow_mode=check_shadow_mode,
        permit_live_camera=permit_live_camera,
        permit_live_vlm=permit_live_vlm,
        permit_real_robot_backend=permit_real_robot_backend,
        permit_robot_motion=permit_robot_motion,
    )


def _section(config: Dict[str, Any] | None, name: str) -> Dict[str, Any]:
    if not isinstance(config, dict):
        return {}
    value = config.get(name)
    if isinstance(value, dict):
        return value
    return config


def _lab_backend_fields(config: Dict[str, Any]) -> Dict[str, Any]:
    return {
        "backend_name": _string(config.get("backend_name")),
        "backend_type": _string(config.get("backend_type")),
        "robot_model": _string(config.get("robot_model")),
        "robot_ip_configured": _bool(config.get("robot_ip_configured")),
        "tcp_calibration_configured": _bool(config.get("tcp_calibration_configured")),
        "robot_calibration_configured": _bool(config.get("robot_calibration_configured")),
        "camera_extrinsics_configured": _bool(config.get("camera_extrinsics_configured")),
        "robot_mode_check_available": _bool(config.get("robot_mode_check_available")),
        "safety_status_check_available": _bool(config.get("safety_status_check_available")),
        "program_state_check_available": _bool(config.get("program_state_check_available")),
        "speed_scaling_check_available": _bool(config.get("speed_scaling_check_available")),
        "allow_real_robot_backend": _bool(config.get("allow_real_robot_backend")),
        "allow_robot_motion": _bool(config.get("allow_robot_motion")),
        "real_robot_command_enabled": _bool(config.get("real_robot_command_enabled")),
    }


def _camera_fields(config: Dict[str, Any]) -> Dict[str, Any]:
    return {
        "camera_backend": _string(config.get("camera_backend")),
        "camera_configured": _bool(config.get("camera_configured")),
        "rgb_stream_configured": _bool(config.get("rgb_stream_configured")),
        "depth_stream_configured": _bool(config.get("depth_stream_configured")),
        "camera_info_configured": _bool(config.get("camera_info_configured")),
        "metadata_configured": _bool(config.get("metadata_configured")),
        "extrinsics_configured": _bool(config.get("extrinsics_configured")),
        "allow_live_camera": _bool(config.get("allow_live_camera")),
    }


def _live_vlm_fields(config: Dict[str, Any]) -> Dict[str, Any]:
    return {
        "vlm_backend": _string(config.get("vlm_backend")),
        "model_name": _string(config.get("model_name")),
        "endpoint_configured": _bool(config.get("endpoint_configured")),
        "schema_output_supported": _bool(config.get("schema_output_supported")),
        "allow_live_vlm": _bool(config.get("allow_live_vlm")),
    }


def _shadow_fields(config: Dict[str, Any]) -> Dict[str, Any]:
    return {
        "shadow_mode_enabled": _bool(config.get("shadow_mode_enabled", True)),
        "fake_execution_enabled": _bool(config.get("fake_execution_enabled", True)),
        "no_motion_enforced": _bool(config.get("no_motion_enforced", True)),
        "evidence_export_enabled": _bool(config.get("evidence_export_enabled", True)),
        "manual_confirmation_required": _bool(config.get("manual_confirmation_required", True)),
    }


def _result(
    *,
    requested: bool,
    status: str,
    fields: Dict[str, Any],
    blocking_reasons: list[str] | None = None,
    warnings: list[str] | None = None,
) -> Dict[str, Any]:
    return {
        "contract_version": CONTRACT_VERSION,
        "requested": requested,
        "status": status,
        "blocking_reasons": _unique(blocking_reasons or []),
        "warnings": _unique(warnings or []),
        "fields": fields,
        "safety_boundary": dict(NO_MOTION_SAFETY_BOUNDARY),
    }


def _component(result: Dict[str, Any], name: str) -> Dict[str, Any]:
    value = result.get(name)
    return value if isinstance(value, dict) else {}


def _component_row(label: str, component: Dict[str, Any]) -> str:
    status = component.get("status", STATUS_NOT_REQUESTED)
    return (
        f"| {label} | {_evidence_status(status)} | {_format_value(status)} | "
        f"{_format_value(component.get('blocking_reasons', []))} | "
        f"{_format_value(component.get('warnings', []))} |"
    )


def _evidence_status(status: Any) -> str:
    if status == STATUS_READY_FOR_SHADOW_MODE:
        return "PASS"
    if status in {STATUS_BLOCKED, STATUS_NOT_READY, STATUS_CONFIG_ONLY, STATUS_DISABLED}:
        return str(status)
    return STATUS_NOT_REQUESTED


def _safety_flags(
    *,
    allow_robot_motion: bool,
    allow_live_camera: bool,
    allow_live_vlm: bool,
    real_robot_command_enabled: bool,
) -> Dict[str, bool]:
    return {
        "allow_robot_motion": allow_robot_motion,
        "allow_live_camera": allow_live_camera,
        "allow_live_vlm": allow_live_vlm,
        "real_robot_backend_used": False,
        "real_robot_command_enabled": real_robot_command_enabled,
        "real_robot_motion_executed": False,
        "live_camera_used": False,
        "live_vlm_called": False,
        "ros2_used": False,
        "moveit_used": False,
        "rtde_used": False,
        "urscript_used": False,
        "dashboard_used": False,
        "trajectory_generated": False,
        "tcp_pose_world_executed": False,
        "automatic_retry_motion_executed": False,
    }


def _bool(value: Any) -> bool:
    return value is True


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
        return str(value)
    return str(value)
