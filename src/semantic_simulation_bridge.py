from __future__ import annotations

import json
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any, Dict

from src.simulation_micro_motion import (
    DEFAULT_MICRO_MOTION_DELTA_RAD,
    DEFAULT_MICRO_MOTION_JOINT,
    DEFAULT_MICRO_MOTION_TOLERANCE_RAD,
    MICRO_MOTION_COMMAND_TYPE,
    SimulationMicroMotionRequest,
)


SEMANTIC_BRIDGE_STATUS_NOT_REQUESTED = "NOT_REQUESTED"
SEMANTIC_BRIDGE_STATUS_OK = "OK"
SEMANTIC_BRIDGE_STATUS_BLOCKED_BY_SEMANTIC_GATE = "BLOCKED_BY_SEMANTIC_GATE"
SEMANTIC_BRIDGE_STATUS_BLOCKED_BY_PRECHECK = "BLOCKED_BY_PRECHECK"
SEMANTIC_BRIDGE_STATUS_FAILED = "FAILED"
DEFAULT_SEMANTIC_CONFIDENCE_THRESHOLD = 0.5
UNKNOWN_TARGET_LABELS = {"", "unknown", "none", "no_target"}
AUDITED_NON_EXECUTABLE_FIELDS = (
    "world_point",
    "world_point_m",
    "camera_point_m",
    "pose_candidates",
    "tcp_pose_world",
    "trajectory",
    "moveit_goal",
    "urscript",
)


@dataclass(frozen=True)
class SemanticSimulationBridgeRequest:
    semantic_task_contract: Dict[str, Any]
    semantic_task_contract_path: str | None = None
    confidence_threshold: float = DEFAULT_SEMANTIC_CONFIDENCE_THRESHOLD
    joint_name: str = DEFAULT_MICRO_MOTION_JOINT
    requested_delta_rad: float = DEFAULT_MICRO_MOTION_DELTA_RAD
    tolerance_rad: float = DEFAULT_MICRO_MOTION_TOLERANCE_RAD


@dataclass(frozen=True)
class SemanticSimulationBridgeResult:
    requested: bool
    status: str
    gate_passed: bool
    blocking_reasons: list[str]
    semantic_task_contract_path: str | None
    semantic_task_id: str | None
    semantic_scene_version: str | None
    semantic_intent: str | None
    semantic_user_command: str | None
    semantic_target_label: str | None
    semantic_confidence_overall: float | None
    semantic_confidence_semantic: float | None
    semantic_bbox_xyxy: list[float] | None
    semantic_pixel_center: list[float] | None
    audited_non_executable_fields: list[str]
    triggered_simulation_micro_motion: bool
    simulation_micro_motion_request: Dict[str, Any] | None
    simulation_only: bool
    real_robot_allowed: bool
    safety_boundary: Dict[str, bool]

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)


def load_semantic_task_contract(path: str | Path) -> Dict[str, Any]:
    contract_path = Path(path).expanduser()
    with contract_path.open("r", encoding="utf-8") as contract_file:
        data = json.load(contract_file)
    if not isinstance(data, dict):
        raise ValueError("semantic task contract JSON must contain an object")
    return data


def build_demo_semantic_task_contract() -> Dict[str, Any]:
    return {
        "schema_version": "teto.semantic_task_contract.v1",
        "task_id": "demo_semantic_bridge_hover_red_mug",
        "scene_version": "demo_scene_v2_6_0",
        "intent": {
            "name": "hover_to_object",
            "user_command": "hover over the red mug",
        },
        "target": {
            "label": "red_mug",
            "bbox_xyxy": [120, 80, 260, 240],
        },
        "geometry": {
            "pixel_center": [190, 160],
            "grounded": True,
        },
        "confidence": {
            "semantic": 0.91,
            "geometry": 0.86,
            "overall": 0.88,
        },
        "error": {
            "code": "OK",
            "message": "",
        },
        "rejected": False,
        "unsafe": False,
    }


def extract_semantic_contract_summary(contract: Dict[str, Any]) -> Dict[str, Any]:
    normalized = _dict_at(contract, "normalized_json")
    target = _dict_at(contract, "target")
    geometry = _dict_at(contract, "geometry")
    confidence = _dict_at(contract, "confidence")
    intent = _dict_at(contract, "intent")
    error = _dict_at(contract, "error")

    return {
        "task_id": _string_or_none(contract.get("task_id")),
        "scene_version": _string_or_none(contract.get("scene_version")),
        "intent": _string_or_none(intent.get("name") or contract.get("intent_name")),
        "user_command": _string_or_none(intent.get("user_command") or contract.get("user_command")),
        "target_label": _string_or_none(target.get("label") or contract.get("target_label")),
        "bbox_xyxy": _bbox_or_none(target.get("bbox_xyxy")) or _bbox_or_none(normalized.get("bbox_xyxy")),
        "pixel_center": _pixel_or_none(geometry.get("pixel_center")) or _pixel_or_none(normalized.get("pixel_center")),
        "grounded": geometry.get("grounded", normalized.get("grounded")),
        "confidence_overall": _number_or_none(confidence.get("overall") or contract.get("confidence_overall")),
        "confidence_semantic": _number_or_none(confidence.get("semantic") or contract.get("confidence_semantic")),
        "confidence_geometry": _number_or_none(confidence.get("geometry") or contract.get("confidence_geometry")),
        "error_code": _string_or_none(error.get("code")),
        "rejected": contract.get("rejected") is True,
        "unsafe": contract.get("unsafe") is True,
        "stale": contract.get("stale") is True or contract.get("scene_status") == "stale",
        "audited_non_executable_fields": _find_audited_non_executable_fields(contract),
    }


def evaluate_semantic_contract_for_simulation_bridge(
    contract: Dict[str, Any],
    *,
    confidence_threshold: float = DEFAULT_SEMANTIC_CONFIDENCE_THRESHOLD,
    semantic_task_contract_path: str | None = None,
) -> Dict[str, Any]:
    if not isinstance(contract, dict):
        return _gate_result(
            summary={},
            passed=False,
            blocking_reasons=["E_INVALID_SEMANTIC_CONTRACT"],
            confidence_threshold=confidence_threshold,
            semantic_task_contract_path=semantic_task_contract_path,
        )

    summary = extract_semantic_contract_summary(contract)
    reasons: list[str] = []
    error_code = summary.get("error_code")
    target_label = str(summary.get("target_label") or "").strip().lower()
    overall = summary.get("confidence_overall")
    semantic = summary.get("confidence_semantic")

    if error_code != "OK":
        reasons.append(str(error_code or "E_INVALID_SEMANTIC_CONTRACT"))
    if summary.get("rejected") is True:
        reasons.append("E_SEMANTIC_BRIDGE_NOT_ELIGIBLE")
    if summary.get("unsafe") is True:
        reasons.append("E_UNSAFE_TARGET")
    if summary.get("stale") is True:
        reasons.append("E_STATE_STALE")
    if not summary.get("task_id"):
        reasons.append("E_INVALID_SEMANTIC_CONTRACT")
    if not summary.get("scene_version"):
        reasons.append("E_MISSING_SCENE_VERSION")
    if not summary.get("intent"):
        reasons.append("E_INVALID_SEMANTIC_CONTRACT")
    if target_label in UNKNOWN_TARGET_LABELS:
        reasons.append("E_MISSING_TARGET")
    if summary.get("bbox_xyxy") is None or summary.get("pixel_center") is None:
        reasons.append("E_MISSING_GROUNDING")
    if overall is None or float(overall) < float(confidence_threshold):
        reasons.append("E_LOW_CONFIDENCE")
    if semantic is None or float(semantic) < float(confidence_threshold):
        reasons.append("E_LOW_CONFIDENCE")

    blocking_reasons = _unique([reason for reason in reasons if reason])
    return _gate_result(
        summary=summary,
        passed=not blocking_reasons,
        blocking_reasons=blocking_reasons,
        confidence_threshold=confidence_threshold,
        semantic_task_contract_path=semantic_task_contract_path,
    )


def build_simulation_micro_motion_request_from_semantic_contract(
    contract: Dict[str, Any],
    *,
    confidence_threshold: float = DEFAULT_SEMANTIC_CONFIDENCE_THRESHOLD,
    joint_name: str = DEFAULT_MICRO_MOTION_JOINT,
    requested_delta_rad: float = DEFAULT_MICRO_MOTION_DELTA_RAD,
    tolerance_rad: float = DEFAULT_MICRO_MOTION_TOLERANCE_RAD,
) -> SimulationMicroMotionRequest:
    gate = evaluate_semantic_contract_for_simulation_bridge(
        contract,
        confidence_threshold=confidence_threshold,
    )
    if gate.get("passed") is not True:
        raise ValueError("semantic contract is not eligible for simulation bridge")
    return SimulationMicroMotionRequest(
        joint_name=joint_name,
        requested_delta_rad=requested_delta_rad,
        tolerance_rad=tolerance_rad,
    )


def build_semantic_simulation_bridge_result(
    request: SemanticSimulationBridgeRequest,
) -> Dict[str, Any]:
    gate = evaluate_semantic_contract_for_simulation_bridge(
        request.semantic_task_contract,
        confidence_threshold=request.confidence_threshold,
        semantic_task_contract_path=request.semantic_task_contract_path,
    )
    summary = gate.get("summary") if isinstance(gate.get("summary"), dict) else {}
    gate_passed = gate.get("passed") is True
    micro_motion_request = None
    if gate_passed:
        micro_motion_request = build_simulation_micro_motion_request_from_semantic_contract(
            request.semantic_task_contract,
            confidence_threshold=request.confidence_threshold,
            joint_name=request.joint_name,
            requested_delta_rad=request.requested_delta_rad,
            tolerance_rad=request.tolerance_rad,
        )

    result = SemanticSimulationBridgeResult(
        requested=True,
        status=SEMANTIC_BRIDGE_STATUS_OK if gate_passed else SEMANTIC_BRIDGE_STATUS_BLOCKED_BY_SEMANTIC_GATE,
        gate_passed=gate_passed,
        blocking_reasons=list(gate.get("blocking_reasons") or []),
        semantic_task_contract_path=request.semantic_task_contract_path,
        semantic_task_id=summary.get("task_id"),
        semantic_scene_version=summary.get("scene_version"),
        semantic_intent=summary.get("intent"),
        semantic_user_command=summary.get("user_command"),
        semantic_target_label=summary.get("target_label"),
        semantic_confidence_overall=summary.get("confidence_overall"),
        semantic_confidence_semantic=summary.get("confidence_semantic"),
        semantic_bbox_xyxy=summary.get("bbox_xyxy"),
        semantic_pixel_center=summary.get("pixel_center"),
        audited_non_executable_fields=list(summary.get("audited_non_executable_fields") or []),
        triggered_simulation_micro_motion=gate_passed,
        simulation_micro_motion_request=(
            {
                "joint_name": micro_motion_request.joint_name,
                "requested_delta_rad": micro_motion_request.requested_delta_rad,
                "tolerance_rad": micro_motion_request.tolerance_rad,
                "command_type": MICRO_MOTION_COMMAND_TYPE,
                "triggered_by_semantic_bridge": True,
            }
            if micro_motion_request
            else None
        ),
        simulation_only=True,
        real_robot_allowed=False,
        safety_boundary=_safety_boundary(),
    ).to_dict()
    result["semantic_gate"] = gate
    return result


def format_semantic_simulation_bridge_report(
    bridge_result: Dict[str, Any],
    *,
    evidence_files: list[Dict[str, str | None]] | None = None,
) -> str:
    evidence_files = evidence_files or []
    return "\n".join(
        [
            "# TETO V2.6.0 Semantic-to-Simulation Motion Bridge Report",
            "",
            "## Semantic Contract Summary",
            "",
            f"- semantic_task_id: {_format_value(bridge_result.get('semantic_task_id'))}",
            f"- semantic_scene_version: {_format_value(bridge_result.get('semantic_scene_version'))}",
            f"- semantic_intent: {_format_value(bridge_result.get('semantic_intent'))}",
            f"- semantic_user_command: {_format_value(bridge_result.get('semantic_user_command'))}",
            f"- semantic_target_label: {_format_value(bridge_result.get('semantic_target_label'))}",
            f"- semantic_confidence_overall: {_format_value(bridge_result.get('semantic_confidence_overall'))}",
            f"- semantic_confidence_semantic: {_format_value(bridge_result.get('semantic_confidence_semantic'))}",
            f"- audited_non_executable_fields: {_format_value(bridge_result.get('audited_non_executable_fields'))}",
            "",
            "## Semantic Gate Result",
            "",
            f"- semantic_gate_passed: {_format_value(bridge_result.get('gate_passed'))}",
            f"- semantic_bridge_status: {_format_value(bridge_result.get('status'))}",
            f"- semantic_bridge_blocking_reasons: {_format_value(bridge_result.get('blocking_reasons'))}",
            "",
            "## Bridge Decision",
            "",
            f"- triggered_simulation_micro_motion: {_format_value(bridge_result.get('triggered_simulation_micro_motion'))}",
            f"- command_type: {_format_value(((bridge_result.get('simulation_micro_motion_request') or {}).get('command_type')))}",
            f"- joint_name: {_format_value(((bridge_result.get('simulation_micro_motion_request') or {}).get('joint_name')))}",
            f"- requested_delta_rad: {_format_value(((bridge_result.get('simulation_micro_motion_request') or {}).get('requested_delta_rad')))}",
            "",
            "## Simulation Motion Trigger",
            "",
            "If the semantic gate passes, this bridge triggers only the established simulation-only micro-motion proof pulse after the simulation motion precheck.",
            "",
            "## Evidence Files",
            "",
            *[f"- {item.get('name')}: {_format_value(item.get('path'))}" for item in evidence_files],
            "",
            "## Safety Boundary",
            "",
            "This bridge consumes an existing semantic task contract.",
            "It does not call a live camera or live VLM.",
            "It does not execute target poses, tcp_pose_world, trajectories, MoveIt goals, URScript, or real robot commands.",
            "If the semantic gate passes, it only triggers a local Isaac Sim simulation-only micro-motion proof pulse.",
            "",
        ]
    )


def _gate_result(
    *,
    summary: Dict[str, Any],
    passed: bool,
    blocking_reasons: list[str],
    confidence_threshold: float,
    semantic_task_contract_path: str | None,
) -> Dict[str, Any]:
    return {
        "passed": passed,
        "status": "PASS" if passed else "BLOCKED",
        "blocking_reasons": _unique(blocking_reasons),
        "confidence_threshold": float(confidence_threshold),
        "semantic_task_contract_path": semantic_task_contract_path,
        "summary": summary,
        "safety_boundary": _safety_boundary(),
    }


def _safety_boundary() -> Dict[str, bool]:
    return {
        "no_live_camera_used": True,
        "no_live_vlm_used": True,
        "no_ros2_used": True,
        "no_moveit_used": True,
        "no_rtde_used": True,
        "no_urscript_used": True,
        "no_dashboard_used": True,
        "no_real_ur5_used": True,
        "no_trajectory_generated": True,
        "no_tcp_pose_world_executed": True,
        "simulation_only": True,
        "real_robot_allowed": False,
    }


def _dict_at(data: Dict[str, Any], key: str) -> Dict[str, Any]:
    value = data.get(key)
    return value if isinstance(value, dict) else {}


def _bbox_or_none(value: Any) -> list[float] | None:
    if not isinstance(value, list) or len(value) != 4:
        return None
    numbers = [_number_or_none(item) for item in value]
    if any(item is None for item in numbers):
        return None
    x1, y1, x2, y2 = [float(item) for item in numbers]
    if x2 <= x1 or y2 <= y1:
        return None
    return [x1, y1, x2, y2]


def _pixel_or_none(value: Any) -> list[float] | None:
    if not isinstance(value, list) or len(value) != 2:
        return None
    numbers = [_number_or_none(item) for item in value]
    if any(item is None for item in numbers):
        return None
    return [float(numbers[0]), float(numbers[1])]


def _number_or_none(value: Any) -> float | None:
    if isinstance(value, bool):
        return None
    if isinstance(value, (int, float)):
        return float(value)
    return None


def _string_or_none(value: Any) -> str | None:
    if isinstance(value, str) and value.strip():
        return value.strip()
    return None


def _find_audited_non_executable_fields(data: Any, *, prefix: str = "") -> list[str]:
    found: list[str] = []
    if isinstance(data, dict):
        for key, value in data.items():
            path = f"{prefix}.{key}" if prefix else str(key)
            if key in AUDITED_NON_EXECUTABLE_FIELDS:
                found.append(path)
            found.extend(_find_audited_non_executable_fields(value, prefix=path))
    elif isinstance(data, list):
        for index, item in enumerate(data):
            found.extend(_find_audited_non_executable_fields(item, prefix=f"{prefix}[{index}]"))
    return _unique(found)


def _unique(values: list[str]) -> list[str]:
    unique_values: list[str] = []
    for value in values:
        if value not in unique_values:
            unique_values.append(value)
    return unique_values


def _format_value(value: Any) -> str:
    if value is None:
        return "null"
    if isinstance(value, (dict, list)):
        return json.dumps(value, ensure_ascii=False)
    return str(value)
