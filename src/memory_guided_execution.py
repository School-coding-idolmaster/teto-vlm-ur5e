from __future__ import annotations

import math
from dataclasses import asdict, dataclass
from typing import Any


WORKING_MEMORY_VERSION = "teto_memory_guided_execution.v1"
REOBSERVATION_POLICY_VERSION = "teto_event_triggered_reobservation.v1"
SCENE_MONITOR_VERSION = "teto_scene_monitor_policy.v1"

STATUS_PASS = "PASS"
STATUS_WARN = "WARN"
STATUS_REOBSERVE_REQUIRED = "REOBSERVE_REQUIRED"
STATUS_ABORT_REQUIRED = "ABORT_REQUIRED"

E_POSITION_ERROR_TOO_LARGE = "E_POSITION_ERROR_TOO_LARGE"
E_DIRECTION_CHECK_FAILED = "E_DIRECTION_CHECK_FAILED"
E_TARGET_LOST = "E_TARGET_LOST"
E_SCENE_STALE = "E_SCENE_STALE"
E_DEPTH_INVALID = "E_DEPTH_INVALID"
E_TF_STALE = "E_TF_STALE"
E_CAMERA_MONITOR_UNAVAILABLE = "E_CAMERA_MONITOR_UNAVAILABLE"
E_REPEATED_SUBGOAL_FAILURE = "E_REPEATED_SUBGOAL_FAILURE"
E_UNEXPECTED_OBSTACLE = "E_UNEXPECTED_OBSTACLE"
E_SCENE_MONITOR_REQUESTED_REOBSERVATION = "E_SCENE_MONITOR_REQUESTED_REOBSERVATION"


@dataclass(frozen=True)
class SceneMonitorResult:
    monitor_type: str = "none"
    camera_check_status: str = "NOT_AVAILABLE"
    target_visible: bool | None = None
    target_moved: bool | None = None
    depth_valid: bool | None = None
    tf_fresh: bool | None = None
    tf_valid: bool | None = None
    unexpected_obstacle: bool | None = None
    scene_snapshot_id: str | None = None
    scene_freshness_status: str = "unknown"
    frequency_mode: str = "unavailable"
    snapshot_expired: bool | None = None
    requires_vlm_reobserve: bool = False
    requires_llm_replan: bool = False
    monitor_latency_ms: float | None = None
    monitor_frequency_hz: float | None = None

    def to_dict(self) -> dict[str, Any]:
        return {
            "scene_monitor_version": SCENE_MONITOR_VERSION,
            **asdict(self),
        }


@dataclass(frozen=True)
class ReobservationPolicyRequest:
    working_memory: dict[str, Any]
    latest_measured_tcp_m: list[float] | None
    subgoal_target_tcp_m: list[float] | None
    position_error_m: float | None
    position_error_limit_m: float
    direction_check_passed: bool | None
    scene_monitor_result: dict[str, Any] | SceneMonitorResult | None = None
    subgoal_failed: bool = False
    subgoal_failure_count: int = 0
    repeated_failure_limit: int = 2
    camera_unavailable_policy: str = "warn_only"


def make_scene_monitor_result(
    value: dict[str, Any] | SceneMonitorResult | None = None,
    *,
    monitor_frequency_hz: float | None = None,
) -> dict[str, Any]:
    if isinstance(value, SceneMonitorResult):
        result = value.to_dict()
    elif isinstance(value, dict):
        result = {
            "scene_monitor_version": SCENE_MONITOR_VERSION,
            "monitor_type": str(value.get("monitor_type") or "none"),
            "camera_check_status": str(value.get("camera_check_status") or "NOT_AVAILABLE"),
            "target_visible": _optional_bool(value.get("target_visible")),
            "target_moved": _optional_bool(value.get("target_moved")),
            "depth_valid": _optional_bool(value.get("depth_valid")),
            "tf_fresh": _optional_bool(value.get("tf_fresh", value.get("tf_valid"))),
            "tf_valid": _optional_bool(value.get("tf_valid", value.get("tf_fresh"))),
            "unexpected_obstacle": _optional_bool(value.get("unexpected_obstacle")),
            "scene_snapshot_id": _optional_string(value.get("scene_snapshot_id")),
            "scene_freshness_status": str(value.get("scene_freshness_status") or "unknown"),
            "frequency_mode": str(value.get("frequency_mode") or "unavailable"),
            "snapshot_expired": _optional_bool(value.get("snapshot_expired")),
            "requires_vlm_reobserve": value.get("requires_vlm_reobserve") is True,
            "requires_llm_replan": value.get("requires_llm_replan") is True,
            "monitor_latency_ms": _optional_number(value.get("monitor_latency_ms")),
            "monitor_frequency_hz": _optional_number(value.get("monitor_frequency_hz")),
        }
    else:
        result = SceneMonitorResult().to_dict()
    if result.get("monitor_frequency_hz") is None:
        result["monitor_frequency_hz"] = _optional_number(monitor_frequency_hz)
    return result


def build_working_memory(
    *,
    task_goal: str,
    goal_type: str,
    target_delta_m: list[float] | None,
    target_point_base_m: list[float] | None = None,
    latest_verified_tcp_m: list[float] | None = None,
    path_strategy: str = "decomposed_autoregressive_subgoals",
    motion_mode: str = "unknown",
) -> dict[str, Any]:
    delta = _vector3(target_delta_m)
    return {
        "working_memory_version": WORKING_MEMORY_VERSION,
        "task_goal": str(task_goal or ""),
        "goal_type": goal_type if goal_type in {"relative_motion", "move_to_object"} else "unknown",
        "target_delta_m": delta,
        "target_point_base_m": _vector3(target_point_base_m),
        "path_strategy": str(path_strategy or "decomposed_autoregressive_subgoals"),
        "motion_mode": str(motion_mode or "unknown"),
        "latest_verified_tcp_m": _vector3(latest_verified_tcp_m),
        "remaining_delta_m": list(delta) if delta is not None else None,
        "completed_substeps": 0,
        "stable_substep_count": 0,
        "last_error_m": None,
        "last_direction_check_passed": None,
        "scene_snapshot_id": None,
        "scene_freshness_status": "unknown",
        "execution_load_mode": "full_observation",
        "llm_call_suppressed": False,
        "vlm_call_suppressed": False,
        "reobserve_required": False,
        "reobserve_reason": None,
        "replan_required": False,
    }


def update_working_memory(
    memory: dict[str, Any],
    *,
    latest_verified_tcp_m: list[float] | None,
    measured_total_delta_m: list[float] | None,
    completed_substeps: int,
    last_error_m: float | None,
    scene_monitor_result: dict[str, Any] | SceneMonitorResult | None,
    reobservation_policy_result: dict[str, Any] | None,
    adaptive_policy_result: dict[str, Any] | None = None,
    stable_substep_count: int | None = None,
    last_direction_check_passed: bool | None = None,
) -> dict[str, Any]:
    result = dict(memory if isinstance(memory, dict) else {})
    target_delta = _vector3(result.get("target_delta_m"))
    measured = _vector3(measured_total_delta_m)
    monitor = make_scene_monitor_result(scene_monitor_result)
    policy = reobservation_policy_result if isinstance(reobservation_policy_result, dict) else {}
    adaptive = adaptive_policy_result if isinstance(adaptive_policy_result, dict) else {}
    result.update(
        {
            "working_memory_version": WORKING_MEMORY_VERSION,
            "latest_verified_tcp_m": _vector3(latest_verified_tcp_m),
            "remaining_delta_m": (
                [round(target_delta[index] - measured[index], 6) for index in range(3)]
                if target_delta is not None and measured is not None
                else target_delta
            ),
            "completed_substeps": max(0, int(completed_substeps)),
            "stable_substep_count": max(
                0,
                int(
                    stable_substep_count
                    if stable_substep_count is not None
                    else result.get("stable_substep_count") or 0
                ),
            ),
            "last_error_m": _optional_number(last_error_m),
            "last_direction_check_passed": last_direction_check_passed,
            "scene_snapshot_id": monitor.get("scene_snapshot_id"),
            "scene_freshness_status": monitor.get("scene_freshness_status") or "unknown",
            "execution_load_mode": adaptive.get(
                "execution_load_mode",
                result.get("execution_load_mode") or "full_observation",
            ),
            "llm_call_suppressed": adaptive.get("llm_call_policy") == "suppressed",
            "vlm_call_suppressed": adaptive.get("vlm_call_policy")
            in {"suppressed", "monitor_only"},
            "reobserve_required": (
                adaptive.get("reobserve_required") is True
                if adaptive
                else policy.get("reobserve_required") is True
            ),
            "reobserve_reason": adaptive.get("reobserve_reason") or policy.get("reobserve_reason"),
            "replan_required": adaptive.get("replan_required") is True,
        }
    )
    return result


def evaluate_event_triggered_reobservation(
    request: ReobservationPolicyRequest,
) -> dict[str, Any]:
    memory = request.working_memory if isinstance(request.working_memory, dict) else {}
    monitor = make_scene_monitor_result(request.scene_monitor_result)
    goal_type = str(memory.get("goal_type") or "unknown")
    reasons: list[str] = []
    warnings: list[str] = []
    abort_required = False

    if (
        request.position_error_m is not None
        and math.isfinite(float(request.position_error_m))
        and float(request.position_error_m) > float(request.position_error_limit_m)
    ):
        reasons.append(E_POSITION_ERROR_TOO_LARGE)
    if request.direction_check_passed is False:
        reasons.append(E_DIRECTION_CHECK_FAILED)
    if monitor.get("target_visible") is False:
        reasons.append(E_TARGET_LOST)
    if str(monitor.get("scene_freshness_status") or "").lower() == "stale":
        reasons.append(E_SCENE_STALE)
    if monitor.get("depth_valid") is False:
        reasons.append(E_DEPTH_INVALID)
    if monitor.get("tf_fresh") is False:
        reasons.append(E_TF_STALE)
    if monitor.get("unexpected_obstacle") is True:
        reasons.append(E_UNEXPECTED_OBSTACLE)
        abort_required = True
    if request.subgoal_failure_count >= max(1, int(request.repeated_failure_limit)):
        reasons.append(E_REPEATED_SUBGOAL_FAILURE)
        abort_required = True
    if monitor.get("requires_vlm_reobserve") is True:
        reasons.append(E_SCENE_MONITOR_REQUESTED_REOBSERVATION)

    monitor_unavailable = str(monitor.get("camera_check_status")) in {
        "NOT_AVAILABLE",
        "FAIL",
    }
    unavailable_policy = (
        request.camera_unavailable_policy
        if request.camera_unavailable_policy in {"warn_only", "block"}
        else "warn_only"
    )
    if monitor_unavailable:
        if goal_type == "move_to_object" or unavailable_policy == "block":
            reasons.append(E_CAMERA_MONITOR_UNAVAILABLE)
        else:
            warnings.append(E_CAMERA_MONITOR_UNAVAILABLE)

    reasons = _unique(reasons)
    warnings = _unique(warnings)
    reobserve_required = bool(reasons)
    replan_required = reobserve_required and not abort_required
    continue_allowed = not reobserve_required and not request.subgoal_failed
    if request.subgoal_failed and not reasons:
        continue_allowed = False
        reasons.append(E_POSITION_ERROR_TOO_LARGE)
        reobserve_required = True
        replan_required = True
    status = (
        STATUS_ABORT_REQUIRED
        if abort_required
        else STATUS_REOBSERVE_REQUIRED
        if reobserve_required
        else STATUS_WARN
        if warnings
        else STATUS_PASS
    )
    return {
        "reobservation_policy_version": REOBSERVATION_POLICY_VERSION,
        "policy_status": status,
        "continue_allowed": continue_allowed,
        "reobserve_required": reobserve_required,
        "replan_required": replan_required,
        "abort_required": abort_required,
        "reobserve_reason": reasons[0] if reasons else None,
        "trigger_reasons": reasons,
        "warnings": warnings,
        "goal_type": goal_type,
        "camera_unavailable_policy": unavailable_policy,
        "vlm_reobserve_called": False,
        "llm_reobserve_called": False,
    }


def _vector3(value: Any) -> list[float] | None:
    if not isinstance(value, (list, tuple)) or len(value) != 3:
        return None
    try:
        result = [round(float(item), 6) for item in value]
    except (TypeError, ValueError):
        return None
    return result if all(math.isfinite(item) for item in result) else None


def _optional_bool(value: Any) -> bool | None:
    return value if isinstance(value, bool) else None


def _optional_number(value: Any) -> float | None:
    if isinstance(value, bool) or value is None:
        return None
    try:
        result = float(value)
    except (TypeError, ValueError):
        return None
    return result if math.isfinite(result) else None


def _optional_string(value: Any) -> str | None:
    if value is None:
        return None
    text = str(value).strip()
    return text or None


def _unique(values: list[str]) -> list[str]:
    output: list[str] = []
    for value in values:
        if value and value not in output:
            output.append(value)
    return output
