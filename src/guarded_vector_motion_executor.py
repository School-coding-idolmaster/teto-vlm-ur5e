from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Callable


EXECUTOR_VERSION = "teto_v3_0_13_guarded_real_vector_executor_v1"
EPS = 1e-9


@dataclass(frozen=True)
class GuardedVectorExecutionRequest:
    autoregressive_plan: dict[str, Any]
    real_execution_requested: bool = False
    enable_real_autoregressive_execution: bool = False
    armed_long_motion_test: bool = False
    pose_reader: Callable[[], dict[str, Any] | None] | None = None
    substep_executor: Callable[[dict[str, Any], dict[str, Any], dict[str, Any]], dict[str, Any]] | None = None
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
        "operator_console_used": False,
        "manual_y_confirmation_required": False,
    }
    if not armed:
        return base
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
    if not callable(request.pose_reader) or not callable(request.substep_executor):
        return {**base, "final_real_execution_status": "BLOCKED", "final_abort_reason": "E_REAL_EXECUTION_ADAPTER_MISSING"}

    distance_tolerance = _positive(config.get("post_step_distance_tolerance_m"), 0.005)
    orthogonal_tolerance = _positive(config.get("max_orthogonal_drift_m"), 0.005)
    evidence = []
    completed = 0
    attempted = 0
    abort_reason = None

    for step in steps:
        pre_pose = _pose(request.pose_reader())
        if pre_pose is None:
            abort_reason = "E_CURRENT_TCP_POSE_MISSING"
            break
        intended = _vector(step.get("target_delta_m") or step.get("substep_delta_m"))
        if intended is None:
            abort_reason = "E_INVALID_SUBSTEP_DELTA"
            break
        target = {
            "frame": pre_pose["frame"],
            "position_m": _add(pre_pose["position_m"], intended),
            "orientation_xyzw": list(pre_pose["orientation_xyzw"]),
        }
        attempted += 1
        execution = request.substep_executor(pre_pose, target, step)
        execution = execution if isinstance(execution, dict) else {}
        execution_ok = (
            execution.get("moveit_pose_executor_status") == "PASS"
            and execution.get("execute_success") is True
            and execution.get("real_robot_motion_executed") is True
        )
        post_pose = _pose(request.pose_reader()) if execution_ok else None
        actual = (
            [post_pose["position_m"][index] - pre_pose["position_m"][index] for index in range(3)]
            if post_pose is not None
            else None
        )
        intended_norm = _norm(intended)
        actual_norm = _norm(actual) if actual is not None else None
        projection = (
            sum(actual[index] * intended[index] for index in range(3)) / intended_norm
            if actual is not None and intended_norm > EPS
            else None
        )
        orthogonal = None
        direction_ok = False
        distance_ok = False
        if actual is not None and projection is not None:
            unit = [value / intended_norm for value in intended]
            parallel = [projection * value for value in unit]
            orthogonal = _norm([actual[index] - parallel[index] for index in range(3)])
            direction_ok = projection > EPS
            distance_ok = abs(actual_norm - intended_norm) <= distance_tolerance + EPS
        verification_ok = bool(
            execution_ok
            and direction_ok
            and distance_ok
            and orthogonal is not None
            and orthogonal <= orthogonal_tolerance + EPS
        )
        step_abort = None
        if not execution_ok:
            step_abort = "E_MOVEIT_SUBSTEP_EXECUTION_FAILED"
        elif post_pose is None:
            step_abort = "E_POST_STEP_TCP_POSE_MISSING"
        elif not direction_ok:
            step_abort = "E_VECTOR_DIRECTION_PROJECTION_FAILED"
        elif not distance_ok:
            step_abort = "E_VECTOR_SUBSTEP_DISTANCE_VERIFICATION_FAILED"
        elif orthogonal is None or orthogonal > orthogonal_tolerance + EPS:
            step_abort = "E_VECTOR_ORTHOGONAL_DRIFT_EXCEEDED"
        if verification_ok:
            completed += 1
        evidence.append(
            {
                "substep_index": step.get("substep_index"),
                "pre_step_tcp_pose": pre_pose,
                "target_tcp_pose": target,
                "moveit_plan_request_created": execution.get("moveit_pose_plan_requested") is True,
                "planner_audit_status": execution.get("planner_audit_status"),
                "planner_risk_status": execution.get("planner_risk_status"),
                "planner_risk_warnings": execution.get("planner_risk_warnings", []),
                "planner_risk_infos": execution.get("planner_risk_infos", []),
                "execute_trajectory_called": execution.get("moveit_execute_called") is True,
                "trajectory_sent": execution.get("trajectory_sent") is True,
                "moveit_execute_error_code_name": execution.get("moveit_execute_error_code_name"),
                "post_step_tcp_pose": post_pose,
                "actual_delta_m": actual,
                "actual_delta_norm_m": round(actual_norm, 6) if actual_norm is not None else None,
                "intended_delta_m": intended,
                "intended_delta_norm_m": round(intended_norm, 6),
                "projection_along_intended_m": round(projection, 6) if projection is not None else None,
                "orthogonal_error_m": round(orthogonal, 6) if orthogonal is not None else None,
                "vector_direction_check_passed": direction_ok,
                "post_step_verification_status": "PASS" if verification_ok else "FAILED",
                "continue_allowed": verification_ok,
                "abort_reason": step_abort,
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
        "real_robot_motion_executed": completed > 0,
    }


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
