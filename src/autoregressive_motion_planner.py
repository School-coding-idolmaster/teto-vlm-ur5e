from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any


PLANNER_VERSION = "teto_v3_0_12_offline_autoregressive_long_motion_v1"
ABORT_POLICY_VERSION = "teto_v3_0_12_offline_abort_policy_v1"
EPS = 1e-9

STATUS_PASS = "PASS"
STATUS_BLOCKED = "BLOCKED"
STATUS_ABORTED = "ABORTED"
STATUS_NEEDS_CURRENT_TCP = "NEEDS_CURRENT_TCP"
STATUS_INVALID_REQUEST = "INVALID_REQUEST"

AXIS_INDEX = {"x": 0, "y": 1, "z": 2}


@dataclass(frozen=True)
class AutoregressiveMotionPlannerRequest:
    canonical_motion_intent: dict[str, Any]
    current_tcp_pose: dict[str, Any] | list[float] | None
    config: dict[str, Any] | None = None
    gateway_evidence: dict[str, Any] | None = None


def plan_offline_autoregressive_motion(
    request: AutoregressiveMotionPlannerRequest,
) -> dict[str, Any]:
    intent = request.canonical_motion_intent if isinstance(request.canonical_motion_intent, dict) else {}
    config = request.config if isinstance(request.config, dict) else {}
    gateway = request.gateway_evidence if isinstance(request.gateway_evidence, dict) else {}

    axis = _string(intent.get("direction_axis"))
    sign = _string(intent.get("direction_sign"))
    frame = _string(intent.get("motion_frame") or intent.get("frame")) or "base_link"
    delta = _vector3(intent.get("delta_m") or intent.get("cartesian_offset_m"))
    requested_distance = _number(intent.get("requested_distance_m"))
    if requested_distance is None and delta is not None:
        requested_distance = _distance(delta)
    requested_distance = round(requested_distance, 6) if requested_distance is not None else None

    decomposition_enabled = config.get("enable_long_step_decomposition", True) is True
    max_one_shot = _positive_number(config.get("max_one_shot_distance_m"), 0.05)
    max_substep = _positive_number(
        config.get("max_decomposed_substep_distance_m", config.get("max_substep_distance_m")),
        0.02,
    )
    max_total = _positive_number(
        config.get("max_decomposed_total_distance_m", config.get("long_motion_total_limit_m")),
        0.20,
    )
    min_final = _positive_number(config.get("min_final_substep_distance_m"), 0.001)
    execution_mode = _string(config.get("substep_execution_mode")) or "offline_preview"
    if execution_mode not in {"offline_preview", "contract_only"}:
        execution_mode = "offline_preview"
    workspace_bounds = _workspace_bounds(config.get("workspace_bounds"))
    session_radius_limit = _number(config.get("session_radius_limit_m"))
    pose = _pose(request.current_tcp_pose)
    session_origin = _pose(config.get("session_origin_tcp_pose")) or pose

    base = _base_evidence(
        axis=axis,
        sign=sign,
        frame=frame,
        requested_distance=requested_distance,
        decomposition_enabled=decomposition_enabled,
        max_one_shot=max_one_shot,
        max_substep=max_substep,
        max_total=max_total,
        execution_mode=execution_mode,
    )

    invalid_reason = _intent_validation_reason(
        intent=intent,
        axis=axis,
        sign=sign,
        delta=delta,
        requested_distance=requested_distance,
    )
    if invalid_reason:
        return {**base, "final_plan_status": STATUS_INVALID_REQUEST, "final_blocking_reason": invalid_reason}
    if not decomposition_enabled:
        return {**base, "final_plan_status": STATUS_BLOCKED, "final_blocking_reason": "E_DECOMPOSITION_DISABLED"}
    if requested_distance <= max_substep + EPS:
        return {
            **base,
            "planned_execution_style": "one_shot_not_expanded",
            "final_plan_status": STATUS_INVALID_REQUEST,
            "final_blocking_reason": "E_DECOMPOSITION_NOT_REQUIRED",
        }
    if requested_distance > max_total + EPS:
        return {
            **base,
            "decomposition_enabled": True,
            "decomposed_motion_allowed": False,
            "final_plan_status": STATUS_BLOCKED,
            "final_blocking_reason": "E_LONG_MOTION_TOTAL_EXCEEDS_LIMIT",
        }
    inherited_blockers = _gateway_blockers(gateway)
    if inherited_blockers:
        return {
            **base,
            "decomposed_motion_allowed": False,
            "hard_safety_gate_failures": inherited_blockers,
            "final_plan_status": STATUS_BLOCKED,
            "final_blocking_reason": inherited_blockers[0],
        }

    decomposition = _decompose_relative_motion(
        delta,
        max_substep_distance_m=max_substep,
        min_final_substep_distance_m=min_final,
    )
    distances = decomposition["planned_substep_distances_m"]
    vectors = decomposition["planned_substep_vectors_m"]
    if any(distance > max_substep + EPS for distance in distances):
        return {
            **base,
            "decomposed_motion_allowed": False,
            "final_plan_status": STATUS_BLOCKED,
            "final_blocking_reason": "E_SUBSTEP_DISTANCE_EXCEEDS_LIMIT",
        }
    if pose is None:
        unavailable_step = _unavailable_substep(
            substep_count=len(distances),
            distance=distances[0] if distances else None,
            axis=axis,
            sign=sign,
            frame=frame,
        )
        return {
            **base,
            **_decomposition_fields(distances, vectors),
            "decomposed_motion_allowed": True,
            "substeps": [unavailable_step],
            "final_plan_status": STATUS_NEEDS_CURRENT_TCP,
            "final_blocking_reason": "E_CURRENT_TCP_POSE_MISSING",
        }

    substeps: list[dict[str, Any]] = []
    latest_verified = pose
    cumulative = 0.0
    final_status = STATUS_PASS
    final_abort_reason = None
    risk_status = _string(config.get("planner_risk_status")) or "NOT_APPLICABLE"
    risk_warnings = _string_list(config.get("planner_risk_warnings"))
    risk_infos = _string_list(config.get("planner_risk_infos"))
    risk_blocking = config.get("planner_risk_blocking_enabled") is True
    verification_failure_index = _integer(config.get("simulate_verification_failure_at_substep"))
    direction_mismatch_index = _integer(config.get("simulate_direction_mismatch_at_substep"))

    for index, (distance, vector) in enumerate(zip(distances, vectors), start=1):
        if latest_verified is None:
            final_status = STATUS_ABORTED
            final_abort_reason = "E_LATEST_VERIFIED_TCP_POSE_MISSING"
            break
        current_position = list(latest_verified["position_m"])
        target_position = [
            round(current_position[component] + float(vector[component]), 6)
            for component in range(3)
        ]
        workspace_ok = _point_in_workspace(target_position, workspace_bounds)
        session_ok = (
            True
            if session_radius_limit is None or session_origin is None
            else _distance_between(session_origin["position_m"], target_position) <= session_radius_limit + EPS
        )
        abort_reason = None
        if not workspace_ok:
            abort_reason = "E_DECOMPOSED_WORKSPACE_ENVELOPE_EXCEEDED"
        elif not session_ok:
            abort_reason = "E_SESSION_RADIUS_EXCEEDS_LIMIT"
        elif risk_blocking and risk_status in {"WARN", "HIGH", "BLOCKED"}:
            abort_reason = "E_PLANNER_RISK_BLOCKING_ENABLED"

        simulated_position = list(target_position)
        verification_status = "SIMULATED_PASS"
        if verification_failure_index == index:
            verification_status = "FAILED"
            abort_reason = abort_reason or "E_SIMULATED_POST_STEP_VERIFICATION_FAILED"
        if direction_mismatch_index == index:
            simulated_position[AXIS_INDEX[axis]] = round(
                current_position[AXIS_INDEX[axis]] - float(vector[AXIS_INDEX[axis]]),
                6,
            )
        direction_check = _direction_check(
            current_position,
            simulated_position,
            axis=axis,
            sign=sign,
        )
        if not direction_check:
            abort_reason = abort_reason or "E_SIMULATED_POST_STEP_DIRECTION_MISMATCH"
        if abort_reason:
            verification_status = "FAILED" if verification_status == "FAILED" else "ABORTED"

        cumulative_after = round(cumulative + distance, 6)
        source = "provided_mock" if index == 1 else "simulated_latest_verified"
        generated_from = "current_tcp_pose" if index == 1 else "simulated_latest_verified_tcp_pose"
        substeps.append(
            {
                "substep_index": index,
                "substep_count": len(distances),
                "substep_distance_m": distance,
                "cumulative_distance_before_m": round(cumulative, 6),
                "cumulative_distance_after_m": cumulative_after,
                "direction_axis": axis,
                "direction_sign": sign,
                "motion_frame": frame,
                "current_tcp_pose_source": source,
                "current_tcp_pose_available": True,
                "current_tcp_pose_m": current_position,
                "target_pose_generation_status": STATUS_PASS,
                "target_tcp_pose_m": target_position,
                "target_generated_from": generated_from,
                "target_delta_m": vector,
                "step_distance_within_limit": distance <= max_substep + EPS,
                "workspace_envelope_within_limit": workspace_ok,
                "session_envelope_within_limit": session_ok,
                "planner_audit_status": _string(config.get("planner_audit_status")) or "NOT_RUN_OFFLINE",
                "planner_risk_status": risk_status,
                "planner_risk_warnings": risk_warnings,
                "planner_risk_infos": risk_infos,
                "execution_status": (
                    "SKIPPED_CONTRACT_ONLY"
                    if execution_mode == "contract_only"
                    else "SKIPPED_OFFLINE_PREVIEW"
                ),
                "execute_trajectory_called": False,
                "trajectory_sent": False,
                "real_robot_motion_executed": False,
                "post_step_verification_status": verification_status,
                "simulated_actual_tcp_pose_m": simulated_position,
                "direction_check_passed": direction_check,
                "continue_allowed": abort_reason is None,
                "abort_reason": abort_reason,
            }
        )
        cumulative = cumulative_after
        if abort_reason:
            final_status = STATUS_ABORTED
            final_abort_reason = abort_reason
            break
        latest_verified = {
            "frame": frame,
            "position_m": simulated_position,
            "orientation_xyzw": list(latest_verified["orientation_xyzw"]),
        }

    return {
        **base,
        **_decomposition_fields(distances, vectors),
        "decomposed_motion_allowed": True,
        "substeps": substeps,
        "completed_substep_count": sum(step["continue_allowed"] for step in substeps),
        "latest_simulated_verified_tcp_pose_m": (
            list(latest_verified["position_m"]) if latest_verified is not None else None
        ),
        "final_plan_status": final_status,
        "final_abort_reason": final_abort_reason,
    }


def _base_evidence(
    *,
    axis: str | None,
    sign: str | None,
    frame: str,
    requested_distance: float | None,
    decomposition_enabled: bool,
    max_one_shot: float,
    max_substep: float,
    max_total: float,
    execution_mode: str,
) -> dict[str, Any]:
    return {
        "autoregressive_motion_planner_version": PLANNER_VERSION,
        "abort_policy_version": ABORT_POLICY_VERSION,
        "planned_execution_style": "decomposed_autoregressive_plan",
        "substep_execution_mode": execution_mode,
        "real_substep_execution_enabled": False,
        "execution_permission_decided_by_parser": False,
        "safety_gate_still_required": True,
        "decomposition_enabled": decomposition_enabled,
        "decomposed_motion_allowed": False,
        "requested_distance_m": requested_distance,
        "direction_axis": axis,
        "direction_sign": sign,
        "motion_frame": frame,
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
        "final_plan_status": STATUS_BLOCKED,
        "final_blocking_reason": None,
        "final_abort_reason": None,
    }


def _decomposition_fields(distances: list[float], vectors: list[list[float]]) -> dict[str, Any]:
    return {
        "substep_count": len(distances),
        "decomposed_substeps_m": vectors,
        "decomposed_substep_distances_m": distances,
        "decomposed_total_distance_m": round(sum(distances), 6),
    }


def _decompose_relative_motion(
    offset_m: list[float],
    *,
    max_substep_distance_m: float,
    min_final_substep_distance_m: float,
) -> dict[str, Any]:
    total_distance = _distance(offset_m)
    max_substep = max(float(max_substep_distance_m), EPS)
    min_final = max(float(min_final_substep_distance_m), 0.0)
    unit = [float(value) / total_distance for value in offset_m]
    full_count = int(total_distance // max_substep)
    remainder = total_distance - (full_count * max_substep)
    distances = [max_substep for _ in range(full_count)]
    if remainder > EPS:
        if distances and remainder < min_final - EPS:
            step_count = max(1, math.ceil(total_distance / max_substep))
            distances = [total_distance / step_count for _ in range(step_count)]
            remainder = 0.0
        else:
            distances.append(remainder)
    distances = [round(float(value), 6) for value in distances if value > EPS]
    vectors = [
        [round(component * distance, 6) for component in unit]
        for distance in distances
    ]
    return {
        "planned_substep_count": len(distances),
        "planned_substep_distances_m": distances,
        "planned_substep_vectors_m": vectors,
        "decomposition_remainder_m": round(float(remainder), 6) if remainder > EPS else 0.0,
    }


def _unavailable_substep(
    *,
    substep_count: int,
    distance: float | None,
    axis: str,
    sign: str,
    frame: str,
) -> dict[str, Any]:
    return {
        "substep_index": 1,
        "substep_count": substep_count,
        "substep_distance_m": distance,
        "cumulative_distance_before_m": 0.0,
        "cumulative_distance_after_m": 0.0,
        "direction_axis": axis,
        "direction_sign": sign,
        "motion_frame": frame,
        "current_tcp_pose_source": "unavailable",
        "current_tcp_pose_available": False,
        "current_tcp_pose_m": None,
        "target_pose_generation_status": STATUS_BLOCKED,
        "target_tcp_pose_m": None,
        "target_generated_from": None,
        "target_delta_m": None,
        "step_distance_within_limit": distance is not None,
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
        "continue_allowed": False,
        "abort_reason": "E_CURRENT_TCP_POSE_MISSING",
    }


def _intent_validation_reason(
    *,
    intent: dict[str, Any],
    axis: str | None,
    sign: str | None,
    delta: list[float] | None,
    requested_distance: float | None,
) -> str | None:
    if intent.get("parse_status") not in {None, STATUS_PASS}:
        return "E_CANONICAL_MOTION_INTENT_NOT_READY"
    if intent.get("intent") not in {None, "relative_cartesian_motion", "cartesian_offset"}:
        return "E_UNSUPPORTED_INTENT"
    if axis not in AXIS_INDEX or sign not in {"+", "-"}:
        return "E_DIRECTION_AXIS_OR_SIGN_INVALID"
    if delta is None or requested_distance is None or requested_distance <= EPS:
        return "E_INVALID_REQUESTED_MOTION"
    nonzero = [index for index, value in enumerate(delta) if abs(value) > EPS]
    if nonzero != [AXIS_INDEX[axis]]:
        return "E_DIRECTION_AXIS_CONFLICTS_WITH_DELTA"
    expected_positive = sign == "+"
    if (delta[AXIS_INDEX[axis]] > 0.0) != expected_positive:
        return "E_DIRECTION_SIGN_CONFLICTS_WITH_DELTA"
    if not math.isclose(_distance(delta), requested_distance, abs_tol=1e-6):
        return "E_REQUESTED_DISTANCE_CONFLICTS_WITH_DELTA"
    return None


def _gateway_blockers(gateway: dict[str, Any]) -> list[str]:
    if not gateway:
        return []
    if gateway.get("cartesian_motion_gateway_status") == STATUS_PASS:
        return []
    return _string_list(gateway.get("blocking_reasons")) or ["E_INHERITED_HARD_SAFETY_GATE_FAILURE"]


def _pose(value: Any) -> dict[str, Any] | None:
    if isinstance(value, list):
        position = _vector3(value)
        return (
            {"frame": "base_link", "position_m": position, "orientation_xyzw": [0.0, 0.0, 0.0, 1.0]}
            if position is not None
            else None
        )
    if not isinstance(value, dict):
        return None
    position = _vector3(value.get("position_m"))
    orientation = value.get("orientation_xyzw")
    if position is None:
        return None
    if not isinstance(orientation, (list, tuple)) or len(orientation) != 4:
        orientation = [0.0, 0.0, 0.0, 1.0]
    try:
        orientation = [float(item) for item in orientation]
    except (TypeError, ValueError):
        return None
    return {
        "frame": _string(value.get("frame")) or "base_link",
        "position_m": position,
        "orientation_xyzw": orientation,
    }


def _workspace_bounds(value: Any) -> dict[str, list[float]]:
    defaults = {"x": [-1.0, 1.0], "y": [-1.0, 1.0], "z": [0.0, 2.0]}
    if not isinstance(value, dict):
        return defaults
    result = dict(defaults)
    for axis in AXIS_INDEX:
        bounds = value.get(axis)
        if isinstance(bounds, (list, tuple)) and len(bounds) == 2:
            try:
                low, high = float(bounds[0]), float(bounds[1])
            except (TypeError, ValueError):
                continue
            if low <= high:
                result[axis] = [low, high]
    return result


def _point_in_workspace(position: list[float], bounds: dict[str, list[float]]) -> bool:
    return all(
        bounds[axis][0] - EPS <= position[index] <= bounds[axis][1] + EPS
        for axis, index in AXIS_INDEX.items()
    )


def _direction_check(
    before: list[float],
    after: list[float],
    *,
    axis: str,
    sign: str,
) -> bool:
    actual = after[AXIS_INDEX[axis]] - before[AXIS_INDEX[axis]]
    return actual > EPS if sign == "+" else actual < -EPS


def _vector3(value: Any) -> list[float] | None:
    if not isinstance(value, (list, tuple)) or len(value) != 3:
        return None
    try:
        vector = [float(item) for item in value]
    except (TypeError, ValueError):
        return None
    return vector if all(math.isfinite(item) for item in vector) else None


def _distance(vector: list[float]) -> float:
    return math.sqrt(sum(float(item) ** 2 for item in vector))


def _distance_between(left: list[float], right: list[float]) -> float:
    return _distance([float(a) - float(b) for a, b in zip(left, right)])


def _number(value: Any) -> float | None:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if math.isfinite(number) else None


def _positive_number(value: Any, default: float) -> float:
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


def _string_list(value: Any) -> list[str]:
    if not isinstance(value, (list, tuple)):
        return []
    return [str(item) for item in value if str(item).strip()]
