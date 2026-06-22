from __future__ import annotations

import math
from typing import Any


CONTRACT_VERSION = "teto_shared_bounded_relative_motion.v1"
SHARED_MAX_RELATIVE_MOTION_DISTANCE_M = 0.50
DEFAULT_MAX_SUBSTEP_M = 0.05
REAL_ONE_SHOT_CAP_M = 0.05
E_RELATIVE_MOTION_RANGE_EXCEEDED = "E_RELATIVE_MOTION_RANGE_EXCEEDED"
EPS = 1e-9


def build_bounded_relative_motion_contract(
    requested_vector_m: list[float],
    *,
    max_substep_m: float = DEFAULT_MAX_SUBSTEP_M,
    execution_backend: str = "dry_run",
    backend_policy_id: str = "shared_bounded_relative_motion",
    start_pose: dict[str, Any] | None = None,
) -> dict[str, Any]:
    vector = [round(float(value), 6) for value in requested_vector_m]
    distance = round(_norm(vector), 6)
    within = distance <= SHARED_MAX_RELATIVE_MOTION_DISTANCE_M + EPS
    normalized = (
        [round(value / distance, 6) for value in vector]
        if distance > EPS
        else [0.0, 0.0, 0.0]
    )
    subgoals = decompose_relative_motion(vector, max_substep_m=max_substep_m) if within else []
    planned = planned_subgoals(start_pose, subgoals)
    return {
        "bounded_relative_motion_contract_version": CONTRACT_VERSION,
        "intent_name": "relative_cartesian_motion",
        "motion_contract_type": "decomposed_relative_motion",
        "requested_vector_m": vector,
        "requested_distance_m": distance,
        "requested_distance_norm_m": distance,
        "normalized_direction_vector": normalized,
        "shared_max_total_distance_m": SHARED_MAX_RELATIVE_MOTION_DISTANCE_M,
        "shared_max_relative_motion_distance_m": SHARED_MAX_RELATIVE_MOTION_DISTANCE_M,
        "distance_within_shared_envelope": within,
        "decomposition_required": distance > float(max_substep_m) + EPS,
        "max_substep_m": round(float(max_substep_m), 6),
        "subgoal_count": len(subgoals),
        "decomposed_substeps_m": subgoals,
        "planned_subgoals": planned,
        "execution_backend": execution_backend,
        "backend_policy_id": backend_policy_id,
        "one_shot_execution_allowed": distance <= REAL_ONE_SHOT_CAP_M + EPS,
        "shared_envelope_error_code": None if within else E_RELATIVE_MOTION_RANGE_EXCEEDED,
    }


def decompose_relative_motion(
    requested_vector_m: list[float],
    *,
    max_substep_m: float = DEFAULT_MAX_SUBSTEP_M,
) -> list[list[float]]:
    vector = [float(value) for value in requested_vector_m]
    distance = _norm(vector)
    if distance <= EPS:
        return []
    max_step = float(max_substep_m)
    count = max(1, int(math.ceil(distance / max_step)))
    unit = [value / distance for value in vector]
    distances = [max_step] * (count - 1)
    distances.append(distance - sum(distances))
    steps = [
        [round(unit[index] * step_distance, 6) for index in range(3)]
        for step_distance in distances[:-1]
    ]
    consumed = [sum(step[index] for step in steps) for index in range(3)]
    steps.append([round(vector[index] - consumed[index], 6) for index in range(3)])
    return steps


def planned_subgoals(
    start_pose: dict[str, Any] | None,
    subgoal_vectors: list[list[float]],
) -> list[dict[str, Any]]:
    if not isinstance(start_pose, dict) or not isinstance(start_pose.get("position_m"), list):
        return [
            {"subgoal_index": index, "delta_m": vector, "planned_pose": None}
            for index, vector in enumerate(subgoal_vectors, start=1)
        ]
    position = [float(value) for value in start_pose["position_m"]]
    orientation = list(start_pose.get("orientation_xyzw") or [0.0, 0.0, 0.0, 1.0])
    frame = str(start_pose.get("frame") or "base_link")
    result = []
    for index, vector in enumerate(subgoal_vectors, start=1):
        position = [round(position[axis] + vector[axis], 6) for axis in range(3)]
        result.append(
            {
                "subgoal_index": index,
                "delta_m": vector,
                "planned_pose": {
                    "frame": frame,
                    "position_m": list(position),
                    "orientation_xyzw": orientation,
                },
            }
        )
    return result


def _norm(vector: list[float]) -> float:
    return math.sqrt(sum(float(value) ** 2 for value in vector))
