from __future__ import annotations

import math
from typing import Any


PLANNER_VERSION = "teto_v3_0_13_vector_autoregressive_long_motion_v1"
ABORT_POLICY_VERSION = "teto_v3_0_13_vector_abort_policy_v1"
EPS = 1e-9
AXES = ("x", "y", "z")


def plan_vector_autoregressive_motion(
    *,
    intent: dict[str, Any],
    delta: list[float],
    current_tcp_pose: dict[str, Any] | list[float] | None,
    config: dict[str, Any],
    gateway_evidence: dict[str, Any],
) -> dict[str, Any]:
    frame = _string(intent.get("motion_frame") or intent.get("frame")) or "base_link"
    norm = round(_norm(delta), 6)
    max_one_shot = _positive(config.get("max_one_shot_distance_m"), 0.05)
    max_substep = _positive(
        config.get("max_decomposed_substep_distance_m", config.get("max_substep_distance_m")),
        0.02,
    )
    max_total = _positive(
        config.get("max_decomposed_total_distance_m", config.get("long_motion_total_limit_m")),
        0.20,
    )
    execution_mode = _string(config.get("substep_execution_mode")) or "offline_preview"
    if execution_mode not in {"offline_preview", "contract_only"}:
        execution_mode = "offline_preview"
    base = _base(
        delta=delta,
        norm=norm,
        frame=frame,
        vector_source=_string(intent.get("vector_source")) or "delta_json",
        max_one_shot=max_one_shot,
        max_substep=max_substep,
        max_total=max_total,
        execution_mode=execution_mode,
        decomposition_enabled=config.get("enable_long_step_decomposition", True) is True,
    )
    if intent.get("parse_status") not in {None, "PASS"}:
        return {**base, "final_plan_status": "INVALID_REQUEST", "final_blocking_reason": "E_CANONICAL_MOTION_INTENT_NOT_READY"}
    if intent.get("intent") not in {None, "relative_cartesian_motion", "cartesian_offset"}:
        return {**base, "final_plan_status": "INVALID_REQUEST", "final_blocking_reason": "E_UNSUPPORTED_INTENT"}
    if norm <= EPS:
        return {**base, "final_plan_status": "INVALID_REQUEST", "final_blocking_reason": "E_INVALID_REQUESTED_MOTION"}
    requested = _number(intent.get("requested_distance_norm_m", intent.get("requested_distance_m")))
    if requested is not None and not math.isclose(requested, norm, abs_tol=1e-6):
        return {**base, "final_plan_status": "INVALID_REQUEST", "final_blocking_reason": "E_REQUESTED_DISTANCE_CONFLICTS_WITH_DELTA"}
    if not base["decomposition_enabled"]:
        return {**base, "final_plan_status": "BLOCKED", "final_blocking_reason": "E_DECOMPOSITION_DISABLED"}
    if norm <= max_substep + EPS:
        return {
            **base,
            "planned_execution_style": "one_shot_not_expanded",
            "final_plan_status": "INVALID_REQUEST",
            "final_blocking_reason": "E_DECOMPOSITION_NOT_REQUIRED",
        }
    if norm > max_total + EPS:
        return {**base, "final_plan_status": "BLOCKED", "final_blocking_reason": "E_LONG_MOTION_TOTAL_EXCEEDS_LIMIT"}
    inherited = _gateway_blockers(gateway_evidence)
    if inherited:
        return {
            **base,
            "hard_safety_gate_failures": inherited,
            "final_plan_status": "BLOCKED",
            "final_blocking_reason": inherited[0],
        }

    step_count = math.ceil(norm / max_substep)
    vectors = _equal_step_vectors(delta, step_count)
    distances = [round(_norm(vector), 6) for vector in vectors]
    unit = [value / norm for value in delta]
    decomposition = {
        "substep_count": step_count,
        "decomposed_substeps_m": vectors,
        "decomposed_substep_distances_m": distances,
        "decomposed_total_distance_m": round(sum(distances), 6),
        "vector_direction_unit_m": _components(unit),
    }
    pose = _pose(current_tcp_pose)
    if pose is None:
        return {
            **base,
            **decomposition,
            "decomposed_motion_allowed": True,
            "substeps": [_missing_pose_step(step_count, vectors[0], distances[0], frame, unit)],
            "final_plan_status": "NEEDS_CURRENT_TCP",
            "final_blocking_reason": "E_CURRENT_TCP_POSE_MISSING",
        }

    bounds = _workspace_bounds(config.get("workspace_bounds"))
    session_origin = _pose(config.get("session_origin_tcp_pose")) or pose
    session_limit = _number(config.get("session_radius_limit_m"))
    risk_status = _string(config.get("planner_risk_status")) or "NOT_APPLICABLE"
    risk_warnings = _strings(config.get("planner_risk_warnings"))
    risk_infos = _strings(config.get("planner_risk_infos"))
    risk_blocking = config.get("planner_risk_blocking_enabled") is True
    fail_index = _integer(config.get("simulate_verification_failure_at_substep"))
    reverse_index = _integer(config.get("simulate_direction_mismatch_at_substep"))
    latest = pose
    cumulative = [0.0, 0.0, 0.0]
    substeps = []
    final_status = "PASS"
    final_abort_reason = None

    for index, (vector, distance) in enumerate(zip(vectors, distances), start=1):
        before = list(latest["position_m"])
        target = _add(before, vector)
        workspace_ok = _in_workspace(target, bounds)
        session_ok = session_limit is None or _norm([target[i] - session_origin["position_m"][i] for i in range(3)]) <= session_limit + EPS
        simulated = _add(before, [-value for value in vector]) if reverse_index == index else list(target)
        direction_ok = _projection([simulated[i] - before[i] for i in range(3)], vector) > EPS
        abort_reason = None
        verification_status = "SIMULATED_PASS"
        if not workspace_ok:
            abort_reason = "E_DECOMPOSED_WORKSPACE_ENVELOPE_EXCEEDED"
        elif not session_ok:
            abort_reason = "E_SESSION_RADIUS_EXCEEDS_LIMIT"
        elif risk_blocking and risk_status in {"WARN", "HIGH", "BLOCKED"}:
            abort_reason = "E_PLANNER_RISK_BLOCKING_ENABLED"
        elif fail_index == index:
            abort_reason = "E_SIMULATED_POST_STEP_VERIFICATION_FAILED"
            verification_status = "FAILED"
        elif not direction_ok:
            abort_reason = "E_SIMULATED_POST_STEP_DIRECTION_MISMATCH"
        if abort_reason and verification_status != "FAILED":
            verification_status = "ABORTED"
        cumulative_after = _add(cumulative, vector)
        substeps.append(
            {
                "substep_index": index,
                "substep_count": step_count,
                "substep_distance_m": distance,
                "substep_delta_m": _components(vector),
                "substep_delta_norm_m": distance,
                "cumulative_distance_before_m": round(_norm(cumulative), 6),
                "cumulative_distance_after_m": round(_norm(cumulative_after), 6),
                "cumulative_delta_before_m": _components(cumulative),
                "cumulative_delta_after_m": _components(cumulative_after),
                "cumulative_distance_norm_before_m": round(_norm(cumulative), 6),
                "cumulative_distance_norm_after_m": round(_norm(cumulative_after), 6),
                "direction_axis": None,
                "direction_sign": None,
                "motion_frame": frame,
                "current_tcp_pose_source": "provided_mock" if index == 1 else "simulated_latest_verified",
                "current_tcp_pose_available": True,
                "current_tcp_pose_m": before,
                "target_pose_generation_status": "PASS",
                "target_tcp_pose_m": target,
                "target_generated_from": "current_tcp_pose" if index == 1 else "latest_verified_tcp_pose",
                "target_delta_m": vector,
                "vector_direction_unit_m": _components(unit),
                "step_distance_within_limit": distance <= max_substep + EPS,
                "step_norm_within_limit": distance <= max_substep + EPS,
                "vector_total_within_limit": norm <= max_total + EPS,
                "workspace_envelope_within_limit": workspace_ok,
                "session_envelope_within_limit": session_ok,
                "planner_audit_status": _string(config.get("planner_audit_status")) or "NOT_RUN_OFFLINE",
                "planner_risk_status": risk_status,
                "planner_risk_warnings": risk_warnings,
                "planner_risk_infos": risk_infos,
                "execution_status": "SKIPPED_CONTRACT_ONLY" if execution_mode == "contract_only" else "SKIPPED_OFFLINE_PREVIEW",
                "execute_trajectory_called": False,
                "trajectory_sent": False,
                "real_robot_motion_executed": False,
                "post_step_verification_status": verification_status,
                "simulated_actual_tcp_pose_m": simulated,
                "direction_check_passed": direction_ok,
                "vector_direction_check_passed": direction_ok,
                "continue_allowed": abort_reason is None,
                "abort_reason": abort_reason,
            }
        )
        cumulative = cumulative_after
        if abort_reason:
            final_status = "ABORTED"
            final_abort_reason = abort_reason
            break
        latest = {**latest, "position_m": simulated}

    return {
        **base,
        **decomposition,
        "decomposed_motion_allowed": True,
        "substeps": substeps,
        "completed_substep_count": sum(step["continue_allowed"] for step in substeps),
        "latest_simulated_verified_tcp_pose_m": list(latest["position_m"]),
        "final_plan_status": final_status,
        "final_abort_reason": final_abort_reason,
    }


def _base(
    *,
    delta: list[float],
    norm: float,
    frame: str,
    vector_source: str,
    max_one_shot: float,
    max_substep: float,
    max_total: float,
    execution_mode: str,
    decomposition_enabled: bool,
) -> dict[str, Any]:
    return {
        "autoregressive_motion_planner_version": PLANNER_VERSION,
        "abort_policy_version": ABORT_POLICY_VERSION,
        "planned_execution_style": "decomposed_autoregressive_vector_preview",
        "substep_execution_mode": execution_mode,
        "real_substep_execution_enabled": False,
        "execution_permission_decided_by_parser": False,
        "safety_gate_still_required": True,
        "vector_motion_supported": True,
        "motion_contract_type": "vector_relative",
        "delta_m": list(delta),
        "vector_delta_m": _components(delta),
        "requested_distance_m": norm,
        "requested_distance_norm_m": norm,
        "vector_components_m": _components(delta),
        "vector_component_count_nonzero": sum(abs(value) > EPS for value in delta),
        "vector_motion_frame": frame,
        "legacy_axis_compatible": False,
        "vector_source": vector_source,
        "direction_axis": None,
        "direction_sign": None,
        "motion_frame": frame,
        "one_shot_vector_motion_allowed": False,
        "one_shot_real_motion_allowed": False,
        "decomposition_enabled": decomposition_enabled,
        "decomposed_motion_allowed": False,
        "max_one_shot_distance_m": max_one_shot,
        "max_decomposed_substep_distance_m": max_substep,
        "max_decomposed_total_distance_m": max_total,
        "substep_count": 0,
        "decomposed_substeps_m": [],
        "decomposed_total_distance_m": 0.0,
        "autoregressive_target_generation": True,
        "targets_generated_from_latest_verified_tcp": True,
        "execute_trajectory_called": False,
        "trajectory_sent": False,
        "real_robot_motion_executed": False,
        "moveit_plan_request_created": False,
        "one_shot_target_pose_created": False,
        "operator_console_used": False,
        "manual_confirmation_required": False,
        "substeps": [],
        "completed_substep_count": 0,
        "latest_simulated_verified_tcp_pose_m": None,
        "hard_safety_gate_failures": [],
        "final_plan_status": "BLOCKED",
        "final_blocking_reason": None,
        "final_abort_reason": None,
    }


def _equal_step_vectors(delta: list[float], count: int) -> list[list[float]]:
    common = [[round(value / count, 6) for value in delta] for _ in range(count - 1)]
    consumed = [sum(vector[index] for vector in common) for index in range(3)]
    return common + [[round(delta[index] - consumed[index], 6) for index in range(3)]]


def _missing_pose_step(count: int, delta: list[float], distance: float, frame: str, unit: list[float]) -> dict[str, Any]:
    return {
        "substep_index": 1,
        "substep_count": count,
        "substep_distance_m": distance,
        "substep_delta_m": _components(delta),
        "substep_delta_norm_m": distance,
        "cumulative_distance_before_m": 0.0,
        "cumulative_distance_after_m": 0.0,
        "cumulative_delta_before_m": _components([0.0, 0.0, 0.0]),
        "cumulative_delta_after_m": None,
        "cumulative_distance_norm_before_m": 0.0,
        "cumulative_distance_norm_after_m": None,
        "direction_axis": None,
        "direction_sign": None,
        "motion_frame": frame,
        "current_tcp_pose_source": "unavailable",
        "current_tcp_pose_available": False,
        "current_tcp_pose_m": None,
        "target_pose_generation_status": "BLOCKED",
        "target_tcp_pose_m": None,
        "target_generated_from": None,
        "target_delta_m": None,
        "vector_direction_unit_m": _components(unit),
        "step_distance_within_limit": True,
        "step_norm_within_limit": True,
        "vector_total_within_limit": True,
        "workspace_envelope_within_limit": None,
        "session_envelope_within_limit": None,
        "planner_audit_status": "NOT_RUN_OFFLINE",
        "planner_risk_status": "NOT_APPLICABLE",
        "planner_risk_warnings": [],
        "planner_risk_infos": [],
        "execution_status": "SKIPPED_OFFLINE_PREVIEW",
        "execute_trajectory_called": False,
        "trajectory_sent": False,
        "real_robot_motion_executed": False,
        "post_step_verification_status": "NOT_RUN_OFFLINE",
        "simulated_actual_tcp_pose_m": None,
        "direction_check_passed": None,
        "vector_direction_check_passed": None,
        "continue_allowed": False,
        "abort_reason": "E_CURRENT_TCP_POSE_MISSING",
    }


def _gateway_blockers(gateway: dict[str, Any]) -> list[str]:
    if not gateway or gateway.get("cartesian_motion_gateway_status") == "PASS":
        return []
    return _strings(gateway.get("blocking_reasons")) or ["E_INHERITED_HARD_SAFETY_GATE_FAILURE"]


def _pose(value: Any) -> dict[str, Any] | None:
    if isinstance(value, list):
        value = {"position_m": value}
    if not isinstance(value, dict):
        return None
    position = _vector(value.get("position_m"))
    if position is None:
        return None
    orientation = value.get("orientation_xyzw", [0.0, 0.0, 0.0, 1.0])
    if not isinstance(orientation, (list, tuple)) or len(orientation) != 4:
        return None
    return {
        "frame": _string(value.get("frame")) or "base_link",
        "position_m": position,
        "orientation_xyzw": [float(item) for item in orientation],
    }


def _workspace_bounds(value: Any) -> dict[str, list[float]]:
    result = {"x": [-1.0, 1.0], "y": [-1.0, 1.0], "z": [0.0, 2.0]}
    if isinstance(value, dict):
        for axis in AXES:
            bounds = value.get(axis)
            if isinstance(bounds, (list, tuple)) and len(bounds) == 2:
                result[axis] = [float(bounds[0]), float(bounds[1])]
    return result


def _in_workspace(position: list[float], bounds: dict[str, list[float]]) -> bool:
    return all(bounds[axis][0] - EPS <= position[index] <= bounds[axis][1] + EPS for index, axis in enumerate(AXES))


def _components(vector: list[float]) -> dict[str, float]:
    return {axis: round(float(vector[index]), 6) for index, axis in enumerate(AXES)}


def _add(left: list[float], right: list[float]) -> list[float]:
    return [round(left[index] + right[index], 6) for index in range(3)]


def _projection(left: list[float], right: list[float]) -> float:
    return sum(left[index] * right[index] for index in range(3))


def _vector(value: Any) -> list[float] | None:
    if isinstance(value, dict):
        value = [value.get("x"), value.get("y"), value.get("z")]
    if not isinstance(value, (list, tuple)) or len(value) != 3:
        return None
    try:
        vector = [float(item) for item in value]
    except (TypeError, ValueError):
        return None
    return vector if all(math.isfinite(item) for item in vector) else None


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


def _integer(value: Any) -> int | None:
    try:
        number = int(value)
    except (TypeError, ValueError):
        return None
    return number if number > 0 else None


def _string(value: Any) -> str | None:
    return value.strip() if isinstance(value, str) and value.strip() else None


def _strings(value: Any) -> list[str]:
    return [str(item) for item in value] if isinstance(value, (list, tuple)) else []
