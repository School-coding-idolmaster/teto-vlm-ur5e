from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict

import yaml


CONTRACT_VERSION = "teto_ur5_read_only_state.v1"
CURRENT_UR5_READ_ONLY_STATE_VERSION = "TETO V2.11.0"

STATUS_NOT_REQUESTED = "NOT_REQUESTED"
STATUS_PASS = "PASS"
STATUS_BLOCKED = "BLOCKED"
READ_ONLY_STATE_CONTRACT_READY = "READ_ONLY_STATE_CONTRACT_READY"

E_ROBOT_MODEL_UNDECLARED = "E_ROBOT_MODEL_UNDECLARED"
E_ROBOT_IP_UNDECLARED = "E_ROBOT_IP_UNDECLARED"
E_READ_ONLY_MODE_REQUIRED = "E_READ_ONLY_MODE_REQUIRED"
E_RTDE_WRITE_NOT_ALLOWED = "E_RTDE_WRITE_NOT_ALLOWED"
E_DASHBOARD_COMMAND_NOT_ALLOWED = "E_DASHBOARD_COMMAND_NOT_ALLOWED"
E_REQUIRED_STATE_FIELD_MISSING = "E_REQUIRED_STATE_FIELD_MISSING"
E_STATE_TTL_MISSING = "E_STATE_TTL_MISSING"
E_EXECUTION_NOT_ALLOWED_IN_READ_ONLY = "E_EXECUTION_NOT_ALLOWED_IN_READ_ONLY"
E_REAL_ROBOT_NOT_ALLOWED = "E_REAL_ROBOT_NOT_ALLOWED"

W_ROBOT_IP_UNAVAILABLE_FOR_SHADOW = "W_ROBOT_IP_UNAVAILABLE_FOR_SHADOW"
W_LIVE_RTDE_NOT_CONNECTED_BY_DESIGN = "W_LIVE_RTDE_NOT_CONNECTED_BY_DESIGN"
W_LIVE_DASHBOARD_NOT_CONNECTED_BY_DESIGN = "W_LIVE_DASHBOARD_NOT_CONNECTED_BY_DESIGN"

REQUIRED_STATE_FIELDS = (
    "robot_mode",
    "safety_status",
    "program_state",
    "speed_scaling",
    "protective_stop",
    "emergency_stop",
    "teach_mode",
    "remote_control_mode",
    "calibration_status",
)

UR5_READ_ONLY_STATE_FIELDS = (
    "ur5_read_only_state_requested",
    "ur5_read_only_state_status",
    "read_only_state_status",
    "read_only_state_contract_ready",
    "robot_model",
    "robot_ip_declared",
    "read_only_mode",
    "rtde_read_enabled",
    "rtde_write_enabled",
    "rtde_write_attempted",
    "dashboard_read_enabled",
    "dashboard_command_enabled",
    "dashboard_command_attempted",
    "required_state_fields_declared",
    "state_ttl_ms",
    "manual_confirmation_required",
    "execution_allowed",
    "real_robot_enabled",
    "blocking_reasons",
    "warnings",
)


@dataclass(frozen=True)
class UR5ReadOnlyStateRequest:
    requested: bool = False
    config_path: str | None = None
    config: Dict[str, Any] | None = None


def load_ur5_read_only_state_config(path: str | Path | None) -> Dict[str, Any]:
    return _load_config(path, "ur5_read_only_state")


def build_ur5_read_only_state_request(
    *,
    requested: bool = False,
    config_path: str | Path | None = None,
    config: Dict[str, Any] | None = None,
) -> UR5ReadOnlyStateRequest:
    loaded_config = config if isinstance(config, dict) else load_ur5_read_only_state_config(config_path)
    return UR5ReadOnlyStateRequest(
        requested=requested,
        config_path=str(Path(config_path).expanduser()) if config_path else None,
        config=loaded_config,
    )


def evaluate_ur5_read_only_state(request: UR5ReadOnlyStateRequest | None = None) -> Dict[str, Any]:
    request = request or UR5ReadOnlyStateRequest()
    if not request.requested:
        return _not_requested_result()

    config = request.config if isinstance(request.config, dict) else {}
    robot_model = _string(config.get("robot_model"))
    robot_ip = _string(config.get("robot_ip"))
    robot_ip_declared = bool(robot_ip)
    read_only_mode = config.get("read_only_mode") is True
    rtde_read_enabled = config.get("rtde_read_enabled", False)
    dashboard_read_enabled = config.get("dashboard_read_enabled", False)
    rtde_write_enabled_requested = config.get("rtde_write_enabled", False) is True
    dashboard_command_enabled_requested = config.get("dashboard_command_enabled", False) is True
    required_state_fields = _declared_state_fields(config)
    missing_state_fields = [field for field in REQUIRED_STATE_FIELDS if field not in required_state_fields]
    state_ttl_ms = config.get("state_ttl_ms")
    manual_confirmation_required = config.get("manual_confirmation_required", True) is True
    execution_allowed_requested = config.get("execution_allowed", False) is True
    real_robot_enabled_requested = config.get("real_robot_enabled", False) is True

    blocking_reasons: list[str] = []
    warnings: list[str] = _string_list(config.get("warnings"))

    if not robot_model:
        blocking_reasons.append(E_ROBOT_MODEL_UNDECLARED)
    if not robot_ip_declared:
        blocking_reasons.append(E_ROBOT_IP_UNDECLARED)
    elif robot_ip in {"unavailable", "unavailable_for_shadow", "future_only"}:
        warnings.append(W_ROBOT_IP_UNAVAILABLE_FOR_SHADOW)
    if not read_only_mode:
        blocking_reasons.append(E_READ_ONLY_MODE_REQUIRED)
    if rtde_write_enabled_requested or config.get("rtde_write_attempted", False) is True:
        blocking_reasons.append(E_RTDE_WRITE_NOT_ALLOWED)
    if dashboard_command_enabled_requested or config.get("dashboard_command_attempted", False) is True:
        blocking_reasons.append(E_DASHBOARD_COMMAND_NOT_ALLOWED)
    if missing_state_fields:
        blocking_reasons.append(E_REQUIRED_STATE_FIELD_MISSING)
    if state_ttl_ms is None:
        blocking_reasons.append(E_STATE_TTL_MISSING)
    if execution_allowed_requested:
        blocking_reasons.append(E_EXECUTION_NOT_ALLOWED_IN_READ_ONLY)
    if real_robot_enabled_requested:
        blocking_reasons.append(E_REAL_ROBOT_NOT_ALLOWED)
    if rtde_read_enabled in {False, "declared_future_only"}:
        warnings.append(W_LIVE_RTDE_NOT_CONNECTED_BY_DESIGN)
    if dashboard_read_enabled in {False, "declared_future_only"}:
        warnings.append(W_LIVE_DASHBOARD_NOT_CONNECTED_BY_DESIGN)

    blocking_reasons = _unique(blocking_reasons)
    warnings = _unique(warnings)
    status = STATUS_PASS if not blocking_reasons else STATUS_BLOCKED
    read_only_status = READ_ONLY_STATE_CONTRACT_READY if status == STATUS_PASS else STATUS_BLOCKED

    return {
        "contract_version": CONTRACT_VERSION,
        "schema_version": CONTRACT_VERSION,
        "teto_version": CURRENT_UR5_READ_ONLY_STATE_VERSION,
        "ur5_read_only_state_requested": True,
        "requested": True,
        "config_path": request.config_path,
        "ur5_read_only_state_status": status,
        "read_only_state_status": read_only_status,
        "read_only_state_contract_ready": status == STATUS_PASS,
        "robot_model": robot_model,
        "robot_ip": robot_ip,
        "robot_ip_declared": robot_ip_declared,
        "read_only_mode": read_only_mode,
        "rtde_read_enabled": rtde_read_enabled,
        "rtde_write_enabled": False,
        "rtde_write_attempted": False,
        "dashboard_read_enabled": dashboard_read_enabled,
        "dashboard_command_enabled": False,
        "dashboard_command_attempted": False,
        "required_state_fields_declared": required_state_fields,
        "missing_state_fields": missing_state_fields,
        "state_ttl_ms": state_ttl_ms,
        "manual_confirmation_required": manual_confirmation_required,
        "execution_allowed": False,
        "real_robot_enabled": False,
        "real_robot_motion_executed": False,
        "blocking_reasons": blocking_reasons,
        "warnings": warnings,
        "next_safe_action": _next_safe_action(status),
        "safety_boundary": _safety_boundary(),
    }


def format_ur5_read_only_state_report(result: Dict[str, Any]) -> str:
    return "\n".join(
        [
            "# TETO V2.11.0 UR5 Read-Only State Contract Report",
            "",
            "## Overall Status",
            "",
            f"- ur5_read_only_state_status: {_format_value(result.get('ur5_read_only_state_status'))}",
            f"- read_only_state_status: {_format_value(result.get('read_only_state_status'))}",
            f"- robot_model: {_format_value(result.get('robot_model'))}",
            f"- robot_ip_declared: {_format_value(result.get('robot_ip_declared'))}",
            f"- required_state_fields_declared: {_format_value(result.get('required_state_fields_declared'))}",
            f"- blocking_reasons: {_format_value(result.get('blocking_reasons'))}",
            f"- warnings: {_format_value(result.get('warnings'))}",
            "",
            "## Contract Fields",
            "",
            "| Field | Value |",
            "| --- | --- |",
            *[f"| {field} | {_format_value(result.get(field))} |" for field in UR5_READ_ONLY_STATE_FIELDS],
            "",
            "## No-Motion Safety Boundary",
            "",
            "V2.11.0 validates a future UR5 read-only state monitor declaration only. It does not connect to RTDE or Dashboard live sockets, does not write RTDE values, does not send Dashboard commands, and does not command a real UR5.",
            "",
        ]
    )


def _declared_state_fields(config: Dict[str, Any]) -> list[str]:
    fields = config.get("required_state_fields")
    if isinstance(fields, dict):
        return [str(key) for key, value in fields.items() if value]
    if isinstance(fields, (list, tuple)):
        return [str(field) for field in fields if field]
    return []


def _not_requested_result() -> Dict[str, Any]:
    return {
        "contract_version": CONTRACT_VERSION,
        "schema_version": CONTRACT_VERSION,
        "teto_version": CURRENT_UR5_READ_ONLY_STATE_VERSION,
        "ur5_read_only_state_requested": False,
        "requested": False,
        "ur5_read_only_state_status": STATUS_NOT_REQUESTED,
        "read_only_state_status": STATUS_NOT_REQUESTED,
        "read_only_state_contract_ready": False,
        "robot_ip_declared": False,
        "read_only_mode": True,
        "rtde_read_enabled": False,
        "rtde_write_enabled": False,
        "rtde_write_attempted": False,
        "dashboard_read_enabled": False,
        "dashboard_command_enabled": False,
        "dashboard_command_attempted": False,
        "required_state_fields_declared": [],
        "manual_confirmation_required": True,
        "execution_allowed": False,
        "real_robot_enabled": False,
        "real_robot_motion_executed": False,
        "blocking_reasons": [],
        "warnings": [],
        "next_safe_action": _next_safe_action(STATUS_NOT_REQUESTED),
        "safety_boundary": _safety_boundary(),
    }


def _next_safe_action(status: str) -> str:
    if status == STATUS_PASS:
        return "Use as read-only UR5 state monitor declaration evidence; do not connect live control sockets."
    if status == STATUS_BLOCKED:
        return "Fix read-only state declarations while keeping RTDE writes, Dashboard commands, and execution disabled."
    return "Request UR5 read-only state contract check with safe declarations."


def _safety_boundary() -> Dict[str, Any]:
    return {
        "read_only_mode": True,
        "rtde_write_enabled": False,
        "rtde_write_attempted": False,
        "dashboard_command_enabled": False,
        "dashboard_command_attempted": False,
        "execution_allowed": False,
        "real_robot_enabled": False,
        "real_robot_motion_executed": False,
    }


def _load_config(path: Any, root_key: str) -> Dict[str, Any]:
    if not path:
        return {}
    resolved_path = Path(str(path)).expanduser()
    if not resolved_path.is_file():
        return {}
    with resolved_path.open("r", encoding="utf-8") as config_file:
        data = json.load(config_file) if resolved_path.suffix.lower() == ".json" else yaml.safe_load(config_file)
    if not isinstance(data, dict):
        return {}
    config = data.get(root_key)
    return config if isinstance(config, dict) else data


def _string(value: Any) -> str | None:
    if isinstance(value, str):
        stripped = value.strip()
        return stripped or None
    if value is None:
        return None
    return str(value).strip() or None


def _string_list(value: Any) -> list[str]:
    if isinstance(value, (list, tuple)):
        return [str(item) for item in value if item]
    if value:
        return [str(value)]
    return []


def _unique(values: list[str]) -> list[str]:
    output: list[str] = []
    seen: set[str] = set()
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
