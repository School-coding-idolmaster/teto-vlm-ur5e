from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Callable


EXECUTOR_VERSION = "teto_v3_0_14_authoritative_gateway_vector_executor_v1"
EPS = 1e-9


@dataclass(frozen=True)
class GuardedVectorExecutionRequest:
    autoregressive_plan: dict[str, Any]
    real_execution_requested: bool = False
    enable_real_autoregressive_execution: bool = False
    armed_long_motion_test: bool = False
    pose_reader: Callable[[], dict[str, Any] | None] | None = None
    substep_executor: Callable[[dict[str, Any], dict[str, Any], dict[str, Any]], dict[str, Any]] | None = None
    authoritative_substep_gateway: Callable[[dict[str, Any]], dict[str, Any]] | None = None
    preflight_blocking_reasons: tuple[str, ...] = ()
    config: dict[str, Any] | None = None


def execute_guarded_vector_motion(request: GuardedVectorExecutionRequest) -> dict[str, Any]:
    plan = request.autoregressive_plan if isinstance(request.autoregressive_plan, dict) else {}
    config = request.config if isinstance(request.config, dict) else {}
    armed = bool(
        request.real_execution_requested
        and request.enable_real_autoregressive_execution
        and request.armed_long_motion_test
    )
    base = {
        "guarded_vector_executor_version": EXECUTOR_VERSION,
        "real_autoregressive_execution_enabled": armed,
        "real_autoregressive_execution_armed": request.armed_long_motion_test is True,
        "arming_flags_satisfied": armed,
        "real_execution_requested": request.real_execution_requested is True,
        "real_execution_started": False,
        "execution_attempted": False,
        "real_execution_attempted": False,
        "real_motion_command_sent": False,
        "real_execution_completed": False,
        "real_execution_aborted": False,
        "real_autoregressive_substeps_attempted": 0,
        "real_autoregressive_substeps_completed": 0,
        "final_real_execution_status": "NOT_REQUESTED",
        "final_abort_reason": None,
        "per_substep_real_execution_evidence": [],
        "execute_trajectory_called": False,
        "trajectory_sent": False,
        "real_robot_motion_executed": False,
        "real_robot_motion_executed_evidence_source": "no_real_execution_attempt",
        "post_motion_verification_failed_after_motion": False,
        "operator_console_used": False,
        "manual_y_confirmation_required": False,
    }
    if not armed:
        if request.real_execution_requested:
            missing = []
            if not request.enable_real_autoregressive_execution:
                missing.append("E_REAL_AUTOREGRESSIVE_EXECUTION_NOT_ENABLED")
            if not request.armed_long_motion_test:
                missing.append("E_LONG_MOTION_TEST_NOT_ARMED")
            return {
                **base,
                "final_real_execution_status": "BLOCKED",
                "final_abort_reason": missing[0] if missing else "E_REAL_EXECUTION_NOT_ARMED",
                "preflight_blocking_reasons": missing,
            }
        return base
    preflight_blockers = [str(reason) for reason in request.preflight_blocking_reasons if str(reason)]
    if preflight_blockers:
        return {
            **base,
            "final_real_execution_status": "BLOCKED",
            "final_abort_reason": preflight_blockers[0],
            "preflight_blocking_reasons": preflight_blockers,
        }
    if plan.get("final_plan_status") != "PASS" or plan.get("motion_contract_type") != "vector_relative":
        return {
            **base,
            "final_real_execution_status": "BLOCKED",
            "final_abort_reason": "E_VECTOR_AUTOREGRESSIVE_PLAN_NOT_READY",
        }
    max_total = _positive(config.get("max_real_autoregressive_total_distance_m"), 0.35)
    max_substep = _positive(config.get("max_real_autoregressive_substep_distance_m"), 0.02)
    requested_norm = _number(plan.get("requested_distance_norm_m"))
    steps = plan.get("substeps") if isinstance(plan.get("substeps"), list) else []
    if requested_norm is None or requested_norm > max_total + EPS:
        return {**base, "final_real_execution_status": "BLOCKED", "final_abort_reason": "E_REAL_VECTOR_TOTAL_EXCEEDS_LIMIT"}
    if any((_number(step.get("substep_delta_norm_m")) or math.inf) > max_substep + EPS for step in steps):
        return {**base, "final_real_execution_status": "BLOCKED", "final_abort_reason": "E_REAL_VECTOR_SUBSTEP_EXCEEDS_LIMIT"}
    if not callable(request.authoritative_substep_gateway):
        return {
            **base,
            "final_real_execution_status": "BLOCKED",
            "final_abort_reason": "E_AUTHORITATIVE_SUBSTEP_GATEWAY_UNAVAILABLE",
        }

    evidence = []
    completed = 0
    attempted = 0
    abort_reason = None

    for step in steps:
        attempted += 1
        gateway = request.authoritative_substep_gateway(step)
        gateway = gateway if isinstance(gateway, dict) else {}
        authoritative = gateway.get("substep_gateway_authoritative") is True
        measured_pose = gateway.get("substep_current_tcp_pose_source") == "measured_gateway"
        no_synthetic_state = gateway.get("synthetic_safety_state_used") is False
        no_synthetic_confirmation = gateway.get("synthetic_confirmation_used") is False
        gateway_status = str(
            gateway.get("gateway_result_status")
            or gateway.get("cartesian_motion_execution_status")
            or "BLOCKED"
        )
        verification_status = str(gateway.get("post_motion_verification_status") or "NOT_RUN")
        verification_ok = bool(
            authoritative
            and measured_pose
            and no_synthetic_state
            and no_synthetic_confirmation
            and gateway_status == "PASS"
            and verification_status == "PASS"
            and gateway.get("continue_allowed") is True
        )
        step_abort = (
            gateway.get("gateway_blocking_reason")
            or gateway.get("abort_reason")
            or (
                "E_SUBSTEP_GATEWAY_NOT_AUTHORITATIVE"
                if not authoritative
                else "E_MEASURED_CURRENT_TCP_POSE_REQUIRED"
                if not measured_pose
                else "E_SYNTHETIC_SAFETY_STATE_FORBIDDEN"
                if not no_synthetic_state
                else "E_SYNTHETIC_CONFIRMATION_FORBIDDEN"
                if not no_synthetic_confirmation
                else "E_AUTHORITATIVE_SUBSTEP_GATEWAY_BLOCKED"
            )
        )
        if verification_ok:
            completed += 1
        evidence.append(
            {
                "substep_index": step.get("substep_index"),
                "substep_gateway_called": True,
                "substep_gateway_authoritative": authoritative,
                "substep_current_tcp_pose_source": gateway.get("substep_current_tcp_pose_source"),
                "synthetic_safety_state_used": gateway.get("synthetic_safety_state_used"),
                "synthetic_confirmation_used": gateway.get("synthetic_confirmation_used"),
                "target_generated_from": gateway.get("target_generated_from"),
                "pre_step_tcp_pose": gateway.get("pre_step_tcp_pose"),
                "target_tcp_pose": gateway.get("target_tcp_pose"),
                "post_step_tcp_pose": gateway.get("post_step_tcp_pose"),
                "gateway_result_status": gateway_status,
                "gateway_blocking_reason": gateway.get("gateway_blocking_reason"),
                "execute_trajectory_called": gateway.get("execute_trajectory_called") is True,
                "trajectory_sent": gateway.get("trajectory_sent") is True,
                "real_execution_attempted": gateway.get("real_execution_attempted") is True,
                "real_motion_command_sent": gateway.get("real_motion_command_sent") is True,
                "real_robot_motion_executed": gateway.get("real_robot_motion_executed") is True,
                "real_robot_motion_executed_evidence_source": gateway.get(
                    "real_robot_motion_executed_evidence_source"
                ),
                "post_motion_verification_status": verification_status,
                "continue_allowed": verification_ok,
                "abort_reason": None if verification_ok else step_abort,
                "authoritative_gateway_result": gateway,
            }
        )
        if not verification_ok:
            abort_reason = step_abort
            break

    aborted = abort_reason is not None
    complete = not aborted and completed == len(steps)
    return {
        **base,
        "real_execution_started": attempted > 0,
        "real_execution_completed": complete,
        "real_execution_aborted": aborted,
        "real_autoregressive_substeps_attempted": attempted,
        "real_autoregressive_substeps_completed": completed,
        "final_real_execution_status": "PASS" if complete else "ABORTED" if aborted else "BLOCKED",
        "final_abort_reason": abort_reason,
        "per_substep_real_execution_evidence": evidence,
        "execute_trajectory_called": any(item["execute_trajectory_called"] for item in evidence),
        "trajectory_sent": any(item["trajectory_sent"] for item in evidence),
        "execution_attempted": attempted > 0,
        "real_execution_attempted": any(
            item["real_execution_attempted"] or item["execute_trajectory_called"]
            for item in evidence
        ),
        "real_motion_command_sent": any(
            item["real_motion_command_sent"] or item["trajectory_sent"]
            for item in evidence
        ),
        "real_robot_motion_executed": any(
            item["real_robot_motion_executed"]
            or item["real_motion_command_sent"]
            or item["trajectory_sent"]
            or item["execute_trajectory_called"]
            for item in evidence
        ),
        "real_robot_motion_executed_evidence_source": _execution_evidence_source(evidence),
        "post_motion_verification_status": (
            "FAILED"
            if any(item["post_motion_verification_status"] == "FAILED" for item in evidence)
            else "PASS"
            if complete
            else "NOT_RUN"
        ),
        "post_motion_verification_failed_after_motion": any(
            item["post_motion_verification_status"] == "FAILED"
            and (
                item["real_robot_motion_executed"]
                or item["real_motion_command_sent"]
                or item["trajectory_sent"]
                or item["execute_trajectory_called"]
            )
            for item in evidence
        ),
    }


def _execution_evidence_source(evidence: list[dict[str, Any]]) -> str:
    if any(item["real_robot_motion_executed"] for item in evidence):
        return "authoritative_gateway_real_robot_motion_executed"
    if any(item["trajectory_sent"] or item["real_motion_command_sent"] for item in evidence):
        return "authoritative_gateway_real_motion_command_sent"
    if any(item["execute_trajectory_called"] for item in evidence):
        return "authoritative_gateway_execute_trajectory_attempted"
    return "no_real_execution_attempt"


def _pose(value: Any) -> dict[str, Any] | None:
    if not isinstance(value, dict):
        return None
    position = _vector(value.get("position_m"))
    orientation = value.get("orientation_xyzw")
    if position is None or not isinstance(orientation, (list, tuple)) or len(orientation) != 4:
        return None
    return {
        "frame": str(value.get("frame") or "base_link"),
        "position_m": position,
        "orientation_xyzw": [float(item) for item in orientation],
    }


def _vector(value: Any) -> list[float] | None:
    if isinstance(value, dict):
        value = [value.get("x"), value.get("y"), value.get("z")]
    if not isinstance(value, (list, tuple)) or len(value) != 3:
        return None
    try:
        return [float(item) for item in value]
    except (TypeError, ValueError):
        return None


def _add(left: list[float], right: list[float]) -> list[float]:
    return [round(left[index] + right[index], 6) for index in range(3)]


def _norm(vector: list[float]) -> float:
    return math.sqrt(sum(value * value for value in vector))


def _number(value: Any) -> float | None:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if math.isfinite(number) else None


def _positive(value: Any, default: float) -> float:
    number = _number(value)
    return number if number is not None and number > 0.0 else default
