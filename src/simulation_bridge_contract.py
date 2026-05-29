from typing import Any, Dict, List

from src.execution_readiness_contract import evaluate_execution_readiness


CONTRACT_VERSION = "teto_simulation_bridge.v1"
DEFAULT_TASK_TYPE = "hover_to_object"


def evaluate_simulation_bridge_eligibility(
    normalized_result: Dict[str, Any] | None,
    execution_readiness_result: Dict[str, Any] | None = None,
) -> Dict[str, Any]:
    normalized = _extract_normalized_json(normalized_result)
    readiness = execution_readiness_result
    if not isinstance(readiness, dict):
        readiness = evaluate_execution_readiness(normalized)

    scene = _dict_value(normalized.get("scene") if normalized else {})
    target = _dict_value(normalized.get("target") if normalized else {})
    task_type = DEFAULT_TASK_TYPE
    target_label = target.get("label", "unknown")
    world_point_m = _world_point_m(normalized, readiness)
    scene_version = scene.get("scene_version")
    ttl_ms = _ttl_ms(normalized, readiness)

    blocking_reasons: List[str] = []
    missing_runtime_inputs: List[str] = []
    if readiness.get("ready") is not True:
        blocking_reasons.append("E_NOT_EXECUTION_READY")
        missing_runtime_inputs.append("execution_readiness")
    if world_point_m is None:
        blocking_reasons.append("E_NO_WORLD_POINT")
        missing_runtime_inputs.append("world_point_m")
    if not scene_version:
        blocking_reasons.append("E_NO_SCENE_VERSION")
        missing_runtime_inputs.append("scene_version")
    if ttl_ms is None:
        blocking_reasons.append("E_NO_TTL")
        missing_runtime_inputs.append("ttl_ms")

    return {
        "contract_version": CONTRACT_VERSION,
        "simulation_ready": not blocking_reasons,
        "blocking_reasons": _unique(blocking_reasons),
        "missing_runtime_inputs": _unique(missing_runtime_inputs),
        "task_type": task_type,
        "target_label": target_label,
        "world_point_m": world_point_m,
        "scene_version": scene_version,
        "ttl_ms": ttl_ms,
        "allow_robot_motion": False,
    }


def build_simulation_task(
    normalized_result: Dict[str, Any],
    execution_readiness_result: Dict[str, Any] | None = None,
) -> Dict[str, Any]:
    eligibility = evaluate_simulation_bridge_eligibility(normalized_result, execution_readiness_result)
    if eligibility["simulation_ready"] is not True:
        raise ValueError(f"simulation bridge is not ready: {', '.join(eligibility['blocking_reasons'])}")

    return {
        "task_type": eligibility["task_type"],
        "target_label": eligibility["target_label"],
        "target_world_point": eligibility["world_point_m"],
        "scene_version": eligibility["scene_version"],
        "ttl_ms": eligibility["ttl_ms"],
    }


def evaluate_replay_record_for_simulation(
    replay_record: Dict[str, Any],
    result_record: Dict[str, Any],
) -> Dict[str, Any]:
    normalized = _extract_normalized_json(result_record)
    readiness = evaluate_execution_readiness(normalized)
    eligibility = evaluate_simulation_bridge_eligibility(normalized, readiness)
    simulation_task = None
    if eligibility["simulation_ready"] is True and normalized is not None:
        simulation_task = build_simulation_task(normalized, readiness)

    return {
        "scene_version": replay_record.get("scene_version", "unknown") if isinstance(replay_record, dict) else "unknown",
        "simulation_ready": eligibility["simulation_ready"],
        "blocking_reasons": eligibility["blocking_reasons"],
        "missing_runtime_inputs": eligibility["missing_runtime_inputs"],
        "task_type": eligibility["task_type"],
        "target_label": eligibility["target_label"],
        "world_point_m": eligibility["world_point_m"],
        "simulation_task": simulation_task,
        "contract_version": CONTRACT_VERSION,
        "allow_robot_motion": False,
    }


def _extract_normalized_json(value: Dict[str, Any] | None) -> Dict[str, Any] | None:
    if not isinstance(value, dict):
        return None
    normalized = value.get("normalized_json")
    if isinstance(normalized, dict):
        return normalized
    if any(key in value for key in ("scene", "target", "geometry_2d", "error", "manipulation_assessment")):
        return value
    return None


def _world_point_m(normalized: Dict[str, Any] | None, readiness: Dict[str, Any]) -> Any:
    for value in (
        _nested_value(normalized, ("geometry_3d", "world_point_m")),
        _nested_value(normalized, ("projector", "world_point_m")),
        _nested_value(normalized, ("simulation", "world_point_m")),
        normalized.get("world_point_m") if isinstance(normalized, dict) else None,
        readiness.get("world_point_m"),
    ):
        if value is not None:
            return value
    return None


def _ttl_ms(normalized: Dict[str, Any] | None, readiness: Dict[str, Any]) -> Any:
    for value in (
        _nested_value(normalized, ("scene", "ttl_ms")),
        _nested_value(normalized, ("scene", "scene_ttl_ms")),
        _nested_value(normalized, ("simulation", "ttl_ms")),
        normalized.get("ttl_ms") if isinstance(normalized, dict) else None,
        readiness.get("ttl_ms"),
    ):
        if value is not None:
            return value
    return None


def _nested_value(value: Dict[str, Any] | None, path: tuple[str, ...]) -> Any:
    current: Any = value
    for key in path:
        if not isinstance(current, dict):
            return None
        current = current.get(key)
    return current


def _dict_value(value: Any) -> Dict[str, Any]:
    return value if isinstance(value, dict) else {}


def _unique(values: List[str]) -> List[str]:
    seen = set()
    result = []
    for value in values:
        if value not in seen:
            seen.add(value)
            result.append(value)
    return result
