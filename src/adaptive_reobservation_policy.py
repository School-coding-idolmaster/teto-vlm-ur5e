from __future__ import annotations

from dataclasses import dataclass
from typing import Any

from src.memory_guided_execution import (
    E_REPEATED_SUBGOAL_FAILURE,
    E_UNEXPECTED_OBSTACLE,
    ReobservationPolicyRequest,
    evaluate_event_triggered_reobservation,
    make_scene_monitor_result,
)


ADAPTIVE_POLICY_VERSION = "teto_adaptive_reobservation_policy.v1"

INITIAL_GROUNDING = "INITIAL_GROUNDING"
STABLE_SUBGOAL_EXECUTION = "STABLE_SUBGOAL_EXECUTION"
LIGHTWEIGHT_MONITORING = "LIGHTWEIGHT_MONITORING"
RECOVERY_REOBSERVATION = "RECOVERY_REOBSERVATION"
REPLAN_AFTER_REOBSERVATION = "REPLAN_AFTER_REOBSERVATION"
ABORTED = "ABORTED"
COMPLETED = "COMPLETED"

E_TARGET_MOVED = "E_TARGET_MOVED"
E_MEMORY_TARGET_INCONSISTENT = "E_MEMORY_TARGET_INCONSISTENT"
E_SNAPSHOT_EXPIRED = "E_SNAPSHOT_EXPIRED"


@dataclass(frozen=True)
class AdaptiveReobservationPolicyRequest:
    working_memory: dict[str, Any]
    current_execution_phase: str
    subgoal_index: int
    position_error_m: float | None
    position_error_limit_m: float
    direction_check_passed: bool | None
    subgoal_failure_count: int
    scene_monitor_result: dict[str, Any] | None
    target_task_type: str
    camera_monitor_available: bool | None
    stable_substep_count: int = 0
    last_reobserve_time: float | None = None
    memory_target_consistent: bool | None = True
    snapshot_expired: bool | None = False
    subgoal_failed: bool = False
    config: dict[str, Any] | None = None


def evaluate_adaptive_reobservation_policy(
    request: AdaptiveReobservationPolicyRequest,
) -> dict[str, Any]:
    config = request.config if isinstance(request.config, dict) else {}
    monitor = make_scene_monitor_result(request.scene_monitor_result)
    camera_policy = str(config.get("camera_unavailable_policy") or "warn_only")
    repeated_failure_limit = _positive_int(config.get("repeated_failure_limit"), 2)
    monitor_available = (
        request.camera_monitor_available
        if isinstance(request.camera_monitor_available, bool)
        else monitor.get("camera_check_status") not in {"NOT_AVAILABLE", "FAIL"}
    )
    base = evaluate_event_triggered_reobservation(
        ReobservationPolicyRequest(
            working_memory=request.working_memory,
            latest_measured_tcp_m=None,
            subgoal_target_tcp_m=None,
            position_error_m=request.position_error_m,
            position_error_limit_m=request.position_error_limit_m,
            direction_check_passed=request.direction_check_passed,
            scene_monitor_result=monitor,
            subgoal_failed=request.subgoal_failed,
            subgoal_failure_count=request.subgoal_failure_count,
            repeated_failure_limit=repeated_failure_limit,
            camera_unavailable_policy=camera_policy,
        )
    )
    trigger_reasons = list(base.get("trigger_reasons") or [])
    if monitor.get("target_moved") is True:
        trigger_reasons.append(E_TARGET_MOVED)
    if request.memory_target_consistent is False:
        trigger_reasons.append(E_MEMORY_TARGET_INCONSISTENT)
    if request.snapshot_expired is True or monitor.get("snapshot_expired") is True:
        trigger_reasons.append(E_SNAPSHOT_EXPIRED)
    trigger_reasons = _unique(trigger_reasons)

    phase = _phase(request.current_execution_phase)
    abort_required = base.get("abort_required") is True
    reobserve_required = bool(trigger_reasons)
    replan_required = bool(
        reobserve_required
        and (
            base.get("replan_required") is True
            or monitor.get("requires_llm_replan") is True
            or any(
                reason
                in {
                    E_TARGET_MOVED,
                    E_MEMORY_TARGET_INCONSISTENT,
                    E_SNAPSHOT_EXPIRED,
                }
                for reason in trigger_reasons
            )
        )
    )

    if phase == INITIAL_GROUNDING and not reobserve_required:
        return _result(
            phase=phase,
            policy_status="PASS",
            execution_load_mode="full_observation",
            llm_call_policy="allowed_on_event",
            vlm_call_policy=(
                "required_now" if request.target_task_type == "move_to_object" else "allowed_on_event"
            ),
            camera_monitor_frequency_mode="normal" if monitor_available else "unavailable",
            reobserve_required=False,
            replan_required=False,
            abort_required=False,
            reasons=[],
            warnings=base.get("warnings"),
            load_reduction_active=False,
            why_llm_not_called=None,
        )
    if phase == COMPLETED and not reobserve_required:
        return _result(
            phase=phase,
            policy_status="PASS",
            execution_load_mode="lightweight_monitor",
            llm_call_policy="suppressed",
            vlm_call_policy="suppressed",
            camera_monitor_frequency_mode="reduced" if monitor_available else "unavailable",
            reobserve_required=False,
            replan_required=False,
            abort_required=False,
            reasons=[],
            warnings=base.get("warnings"),
            load_reduction_active=True,
            why_llm_not_called="task_completed_no_semantic_recovery_needed",
        )
    if phase == ABORTED or abort_required:
        return _result(
            phase=ABORTED,
            policy_status="BLOCKED",
            execution_load_mode="abort",
            llm_call_policy="allowed_on_event",
            vlm_call_policy="allowed_on_event",
            camera_monitor_frequency_mode="elevated" if monitor_available else "unavailable",
            reobserve_required=reobserve_required,
            replan_required=False,
            abort_required=True,
            reasons=trigger_reasons or [E_REPEATED_SUBGOAL_FAILURE],
            warnings=base.get("warnings"),
            load_reduction_active=False,
            why_llm_not_called="safety_abort_precedes_semantic_recovery",
        )
    if reobserve_required:
        return _result(
            phase=RECOVERY_REOBSERVATION,
            policy_status="BLOCKED",
            execution_load_mode="recovery_reobserve",
            llm_call_policy="required_now",
            vlm_call_policy="required_now",
            camera_monitor_frequency_mode="elevated" if monitor_available else "unavailable",
            reobserve_required=True,
            replan_required=replan_required,
            abort_required=False,
            reasons=trigger_reasons,
            warnings=base.get("warnings"),
            load_reduction_active=False,
            why_llm_not_called=None,
        )

    reduced_after = _positive_int(config.get("stable_substeps_before_reduced"), 1)
    frequency_mode = (
        "reduced"
        if monitor_available and request.stable_substep_count >= reduced_after
        else "normal"
        if monitor_available
        else "unavailable"
    )
    lightweight_phase = (
        LIGHTWEIGHT_MONITORING
        if phase in {STABLE_SUBGOAL_EXECUTION, LIGHTWEIGHT_MONITORING}
        else phase
    )
    return _result(
        phase=lightweight_phase,
        policy_status="WARN" if base.get("warnings") else "PASS",
        execution_load_mode="lightweight_monitor",
        llm_call_policy="suppressed",
        vlm_call_policy="monitor_only" if monitor_available else "suppressed",
        camera_monitor_frequency_mode=frequency_mode,
        reobserve_required=False,
        replan_required=False,
        abort_required=False,
        reasons=[],
        warnings=base.get("warnings"),
        load_reduction_active=True,
        why_llm_not_called="stable_execution_uses_deterministic_feedback_and_low_cost_monitor",
    )


def _result(
    *,
    phase: str,
    policy_status: str,
    execution_load_mode: str,
    llm_call_policy: str,
    vlm_call_policy: str,
    camera_monitor_frequency_mode: str,
    reobserve_required: bool,
    replan_required: bool,
    abort_required: bool,
    reasons: list[str],
    warnings: list[str] | None,
    load_reduction_active: bool,
    why_llm_not_called: str | None,
) -> dict[str, Any]:
    return {
        "adaptive_reobservation_policy_version": ADAPTIVE_POLICY_VERSION,
        "policy_status": policy_status,
        "execution_phase": phase,
        "execution_load_mode": execution_load_mode,
        "llm_call_policy": llm_call_policy,
        "vlm_call_policy": vlm_call_policy,
        "camera_monitor_frequency_mode": camera_monitor_frequency_mode,
        "reobserve_required": reobserve_required,
        "replan_required": replan_required,
        "abort_required": abort_required,
        "reobserve_reason": reasons[0] if reasons else None,
        "trigger_reasons": reasons,
        "warnings": list(warnings or []),
        "load_reduction_active": load_reduction_active,
        "why_llm_not_called": why_llm_not_called,
        "why_reobserve_triggered": reasons[0] if reasons else None,
        "llm_reobserve_called": False,
        "vlm_reobserve_called": False,
    }


def _phase(value: Any) -> str:
    text = str(value or STABLE_SUBGOAL_EXECUTION)
    allowed = {
        INITIAL_GROUNDING,
        STABLE_SUBGOAL_EXECUTION,
        LIGHTWEIGHT_MONITORING,
        RECOVERY_REOBSERVATION,
        REPLAN_AFTER_REOBSERVATION,
        ABORTED,
        COMPLETED,
    }
    return text if text in allowed else STABLE_SUBGOAL_EXECUTION


def _positive_int(value: Any, default: int) -> int:
    try:
        result = int(value)
    except (TypeError, ValueError):
        return default
    return result if result > 0 else default


def _unique(values: list[str]) -> list[str]:
    output: list[str] = []
    for value in values:
        if value and value not in output:
            output.append(value)
    return output
