from __future__ import annotations

import json
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any, Callable, Dict

from src.robot_prim_inspector import UR5E_ARM_JOINT_NAMES, categorize_joint_name


MICRO_MOTION_STATUS_NOT_REQUESTED = "NOT_REQUESTED"
MICRO_MOTION_STATUS_OK = "OK"
MICRO_MOTION_STATUS_DRY_RUN_ONLY = "DRY_RUN_ONLY"
MICRO_MOTION_STATUS_BLOCKED_BY_PRECHECK = "BLOCKED_BY_PRECHECK"
MICRO_MOTION_STATUS_FAILED = "FAILED"
MICRO_MOTION_COMMAND_TYPE = "ISAAC_SIMULATION_API_LOCAL_ONLY"
DEFAULT_MICRO_MOTION_JOINT = "wrist_3_joint"
DEFAULT_MICRO_MOTION_DELTA_RAD = 0.01
DEFAULT_MICRO_MOTION_TOLERANCE_RAD = 0.005
MAX_MICRO_MOTION_ABS_DELTA_RAD = 0.01


@dataclass(frozen=True)
class SimulationMicroMotionRequest:
    joint_name: str = DEFAULT_MICRO_MOTION_JOINT
    requested_delta_rad: float = DEFAULT_MICRO_MOTION_DELTA_RAD
    tolerance_rad: float = DEFAULT_MICRO_MOTION_TOLERANCE_RAD
    max_abs_delta_rad: float = MAX_MICRO_MOTION_ABS_DELTA_RAD


@dataclass(frozen=True)
class SimulationMicroMotionResult:
    requested: bool
    status: str
    simulation_only: bool
    real_robot_allowed: bool
    real_robot_motion_executed: bool
    robot_motion_executed: bool
    command_type: str
    joint_name: str | None
    requested_delta_rad: float | None
    actual_delta_rad: float | None
    tolerance_rad: float | None
    delta_within_tolerance: bool
    before_joint_position_rad: float | None
    after_joint_position_rad: float | None
    blocking_reasons: list[str]
    warnings: list[str]
    errors: list[str]

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)


def validate_micro_motion_request(
    request: SimulationMicroMotionRequest,
    *,
    available_joint_names: list[str] | None = None,
) -> list[str]:
    errors = []
    joint_name = str(request.joint_name or "")
    available = set(available_joint_names or UR5E_ARM_JOINT_NAMES)
    if categorize_joint_name(joint_name) != "arm":
        errors.append("E_MICRO_MOTION_JOINT_NOT_ARM")
    if joint_name not in available:
        errors.append("E_MICRO_MOTION_JOINT_UNKNOWN")
    if abs(float(request.requested_delta_rad)) > float(request.max_abs_delta_rad):
        errors.append("E_MICRO_MOTION_DELTA_TOO_LARGE")
    if float(request.requested_delta_rad) == 0.0:
        errors.append("E_MICRO_MOTION_DELTA_ZERO")
    if float(request.tolerance_rad) <= 0.0:
        errors.append("E_MICRO_MOTION_TOLERANCE_INVALID")
    return _unique(errors)


def compute_joint_delta(before_position: float | None, after_position: float | None) -> float | None:
    if before_position is None or after_position is None:
        return None
    return float(after_position) - float(before_position)


def build_joint_diff_summary(result: Dict[str, Any]) -> Dict[str, Any]:
    motion = result.get("motion") if isinstance(result.get("motion"), dict) else result
    before_position = _number_or_none(motion.get("before_joint_position_rad"))
    after_position = _number_or_none(motion.get("after_joint_position_rad"))
    requested_delta = _number_or_none(motion.get("requested_delta_rad"))
    actual_delta = _number_or_none(motion.get("actual_delta_rad"))
    tolerance = _number_or_none(motion.get("tolerance_rad"))
    if actual_delta is None:
        actual_delta = compute_joint_delta(before_position, after_position)
    delta_error = None
    if requested_delta is not None and actual_delta is not None:
        delta_error = actual_delta - requested_delta
    return {
        "joint_name": motion.get("joint_name"),
        "before_joint_position_rad": before_position,
        "after_joint_position_rad": after_position,
        "requested_delta_rad": requested_delta,
        "actual_delta_rad": actual_delta,
        "delta_error_rad": delta_error,
        "tolerance_rad": tolerance,
        "delta_within_tolerance": motion.get("delta_within_tolerance", False) is True,
        "simulation_micro_motion_status": result.get("simulation_micro_motion_status") or motion.get("status"),
    }


def format_joint_diff_table(result: Dict[str, Any]) -> str:
    summary = build_joint_diff_summary(result)
    return "\n".join(
        [
            "| Joint name | Before rad | After rad | Requested delta rad | Actual delta rad | Delta error rad | Tolerance rad | Within tolerance |",
            "| --- | ---: | ---: | ---: | ---: | ---: | ---: | --- |",
            (
                f"| {_format_value(summary.get('joint_name'))} "
                f"| {_format_value(summary.get('before_joint_position_rad'))} "
                f"| {_format_value(summary.get('after_joint_position_rad'))} "
                f"| {_format_value(summary.get('requested_delta_rad'))} "
                f"| {_format_value(summary.get('actual_delta_rad'))} "
                f"| {_format_value(summary.get('delta_error_rad'))} "
                f"| {_format_value(summary.get('tolerance_rad'))} "
                f"| {_format_value(summary.get('delta_within_tolerance'))} |"
            ),
        ]
    )


def normalize_motion_evidence_paths(result: Dict[str, Any]) -> Dict[str, str | None]:
    motion = result.get("motion") if isinstance(result.get("motion"), dict) else {}
    return {
        "simulation_motion_result.json": result.get("simulation_motion_result_path")
        or motion.get("simulation_motion_result_path"),
        "simulation_motion_report.md": result.get("simulation_motion_report_path")
        or motion.get("simulation_motion_report_path"),
        "before_articulation_state.json": result.get("before_joint_state_path")
        or motion.get("before_joint_state_path"),
        "after_articulation_state.json": result.get("after_joint_state_path")
        or motion.get("after_joint_state_path"),
    }


def summarize_motion_evidence(result: Dict[str, Any]) -> Dict[str, Any]:
    evidence_paths = normalize_motion_evidence_paths(result)
    evidence_files = [
        {"name": name, "path": path}
        for name, path in evidence_paths.items()
        if path
    ]
    return {
        "motion_evidence_available": bool(
            result.get("simulation_micro_motion_requested", result.get("requested", False))
        )
        and len(evidence_files) == 4,
        "motion_evidence_files": evidence_files,
        "motion_diff_summary": build_joint_diff_summary(result),
        "simulation_only": result.get("simulation_only", True),
        "real_robot_motion_executed": result.get("real_robot_motion_executed", False),
    }


def execute_simulation_micro_motion(
    request: SimulationMicroMotionRequest,
    *,
    simulation_motion_precheck: Dict[str, Any] | None,
    articulation_readiness: Dict[str, Any] | None,
    before_articulation_state: Dict[str, Any] | None,
    dry_run: bool = False,
    motion_executor: Callable[[SimulationMicroMotionRequest, Dict[str, Any]], Dict[str, Any]] | None = None,
) -> Dict[str, Any]:
    precheck = simulation_motion_precheck if isinstance(simulation_motion_precheck, dict) else {}
    readiness = articulation_readiness if isinstance(articulation_readiness, dict) else {}
    before_state = before_articulation_state if isinstance(before_articulation_state, dict) else {}
    available_joint_names = _available_arm_joint_names(before_state, precheck)
    validation_errors = validate_micro_motion_request(request, available_joint_names=available_joint_names)
    before_joint_position = _joint_position(before_state, request.joint_name)

    if dry_run:
        result = _base_result(
            request=request,
            status=MICRO_MOTION_STATUS_DRY_RUN_ONLY,
            before_joint_position=before_joint_position,
            after_joint_position=before_joint_position,
            actual_delta=None,
            delta_within_tolerance=False,
            robot_motion_executed=False,
            blocking_reasons=_unique(["DRY_RUN_ONLY"] + _precheck_blocking_reasons(precheck) + validation_errors),
            warnings=["Dry-run only; no Isaac simulation micro-motion was executed."],
            errors=[],
        )
        return _with_articulation_state_snapshots(
            result,
            before_state=before_state,
            after_state=before_state,
            precheck=precheck,
            readiness=readiness,
        )

    precheck_blockers = _precheck_gate_blocking_reasons(
        precheck=precheck,
        readiness=readiness,
        before_state=before_state,
    )
    if precheck_blockers:
        result = _base_result(
            request=request,
            status=MICRO_MOTION_STATUS_BLOCKED_BY_PRECHECK,
            before_joint_position=before_joint_position,
            after_joint_position=before_joint_position,
            actual_delta=None,
            delta_within_tolerance=False,
            robot_motion_executed=False,
            blocking_reasons=_unique(precheck_blockers + validation_errors),
            warnings=[],
            errors=["E_SIMULATION_MOTION_PRECHECK_NOT_READY"],
        )
        return _with_articulation_state_snapshots(
            result,
            before_state=before_state,
            after_state=before_state,
            precheck=precheck,
            readiness=readiness,
        )

    if validation_errors:
        result = _base_result(
            request=request,
            status=MICRO_MOTION_STATUS_FAILED,
            before_joint_position=before_joint_position,
            after_joint_position=before_joint_position,
            actual_delta=None,
            delta_within_tolerance=False,
            robot_motion_executed=False,
            blocking_reasons=validation_errors,
            warnings=[],
            errors=validation_errors,
        )
        return _with_articulation_state_snapshots(
            result,
            before_state=before_state,
            after_state=before_state,
            precheck=precheck,
            readiness=readiness,
        )

    try:
        if motion_executor is None:
            raise RuntimeError("simulation micro-motion executor is not available")
        executor_result = motion_executor(request, before_state)
        after_state = executor_result.get("after_articulation_state", executor_result)
        before_state = executor_result.get("before_articulation_state", before_state)
        before_joint_position = _joint_position(before_state, request.joint_name)
        after_joint_position = _joint_position(after_state, request.joint_name)
        actual_delta = compute_joint_delta(before_joint_position, after_joint_position)
        delta_within_tolerance = (
            actual_delta is not None
            and abs(float(actual_delta) - float(request.requested_delta_rad)) <= float(request.tolerance_rad)
        )
        status = MICRO_MOTION_STATUS_OK if delta_within_tolerance else MICRO_MOTION_STATUS_FAILED
        errors = [] if delta_within_tolerance else ["E_SIMULATION_MICRO_MOTION_TOLERANCE_EXCEEDED"]
        result = _base_result(
            request=request,
            status=status,
            before_joint_position=before_joint_position,
            after_joint_position=after_joint_position,
            actual_delta=actual_delta,
            delta_within_tolerance=delta_within_tolerance,
            robot_motion_executed=delta_within_tolerance,
            blocking_reasons=[] if delta_within_tolerance else errors,
            warnings=[],
            errors=errors,
        )
        return _with_articulation_state_snapshots(
            result,
            before_state=before_state,
            after_state=after_state,
            precheck=precheck,
            readiness=readiness,
        )
    except Exception as exc:
        result = _base_result(
            request=request,
            status=MICRO_MOTION_STATUS_FAILED,
            before_joint_position=before_joint_position,
            after_joint_position=before_joint_position,
            actual_delta=None,
            delta_within_tolerance=False,
            robot_motion_executed=False,
            blocking_reasons=["E_SIMULATION_MICRO_MOTION_FAILED"],
            warnings=[],
            errors=[str(exc)],
        )
        return _with_articulation_state_snapshots(
            result,
            before_state=before_state,
            after_state=before_state,
            precheck=precheck,
            readiness=readiness,
        )


def format_simulation_micro_motion_report(result: Dict[str, Any]) -> str:
    motion = result.get("motion") if isinstance(result.get("motion"), dict) else result
    precheck = result.get("precheck") if isinstance(result.get("precheck"), dict) else {}
    evidence = summarize_motion_evidence(result)
    return "\n".join(
        [
            "# TETO V2.6.0 Simulation Micro-Motion Evidence Report",
            "",
            "This is simulation-only micro-motion.",
            "No real robot command was generated.",
            "No ROS2 / MoveIt / RTDE / URScript / real UR5 control chain was used.",
            "The motion was executed only through the local Isaac Sim simulation API.",
            "",
            "## Status",
            "",
            f"- simulation_micro_motion_status: {_format_value(result.get('simulation_micro_motion_status') or motion.get('status'))}",
            f"- simulation_only: {_format_value(result.get('simulation_only', motion.get('simulation_only')))}",
            f"- real_robot_allowed: {_format_value(result.get('real_robot_allowed', motion.get('real_robot_allowed')))}",
            f"- real_robot_motion_executed: {_format_value(result.get('real_robot_motion_executed', motion.get('real_robot_motion_executed')))}",
            f"- robot_motion_executed: {_format_value(result.get('robot_motion_executed', motion.get('robot_motion_executed')))}",
            f"- command_type: {_format_value(motion.get('command_type'))}",
            "",
            "## Precheck Summary",
            "",
            f"- simulation_motion_precheck_status: {_format_value(precheck.get('simulation_motion_precheck_status'))}",
            f"- ready_for_simulation_motion: {_format_value(precheck.get('ready_for_simulation_motion'))}",
            f"- articulation_readiness_status: {_format_value(precheck.get('articulation_readiness_status'))}",
            f"- articulation_state_status: {_format_value(precheck.get('articulation_state_status'))}",
            f"- blocking_reasons: {_format_value(precheck.get('blocking_reasons'))}",
            f"- warnings: {_format_value(precheck.get('warnings'))}",
            f"- errors: {_format_value(precheck.get('errors'))}",
            "",
            "## Joint Diff Summary",
            "",
            f"- joint_name: {_format_value(motion.get('joint_name'))}",
            f"- before_joint_position_rad: {_format_value(motion.get('before_joint_position_rad'))}",
            f"- after_joint_position_rad: {_format_value(motion.get('after_joint_position_rad'))}",
            f"- requested_delta_rad: {_format_value(motion.get('requested_delta_rad'))}",
            f"- actual_delta_rad: {_format_value(motion.get('actual_delta_rad'))}",
            f"- tolerance_rad: {_format_value(motion.get('tolerance_rad'))}",
            f"- delta_within_tolerance: {_format_value(motion.get('delta_within_tolerance'))}",
            "",
            format_joint_diff_table(result),
            "",
            "## Evidence Files",
            "",
            *[
                f"- {item['name']}: {_format_value(item['path'])}"
                for item in evidence.get("motion_evidence_files", [])
            ],
            "",
            "## Safety Boundary",
            "",
            "- simulation_only: True",
            "- real_robot_allowed: False",
            "- real_robot_motion_executed: False",
            "- real robot control chain used: False",
            "",
        ]
    )


def write_simulation_micro_motion_artifacts(
    result: Dict[str, Any],
    output_dir: str | Path,
) -> Dict[str, Path]:
    run_dir = Path(output_dir)
    run_dir.mkdir(parents=True, exist_ok=True)
    motion_result_path = run_dir / "simulation_motion_result.json"
    motion_report_path = run_dir / "simulation_motion_report.md"
    before_state_path = run_dir / "before_articulation_state.json"
    after_state_path = run_dir / "after_articulation_state.json"
    with motion_result_path.open("w", encoding="utf-8") as result_file:
        json.dump(result, result_file, ensure_ascii=False, indent=2)
        result_file.write("\n")
    motion_report_path.write_text(format_simulation_micro_motion_report(result), encoding="utf-8")
    with before_state_path.open("w", encoding="utf-8") as before_file:
        json.dump(result.get("before_articulation_state", {}), before_file, ensure_ascii=False, indent=2)
        before_file.write("\n")
    with after_state_path.open("w", encoding="utf-8") as after_file:
        json.dump(result.get("after_articulation_state", {}), after_file, ensure_ascii=False, indent=2)
        after_file.write("\n")
    return {
        "simulation_motion_result_path": motion_result_path,
        "simulation_motion_report_path": motion_report_path,
        "before_articulation_state_path": before_state_path,
        "after_articulation_state_path": after_state_path,
    }


def _base_result(
    *,
    request: SimulationMicroMotionRequest,
    status: str,
    before_joint_position: float | None,
    after_joint_position: float | None,
    actual_delta: float | None,
    delta_within_tolerance: bool,
    robot_motion_executed: bool,
    blocking_reasons: list[str],
    warnings: list[str],
    errors: list[str],
) -> Dict[str, Any]:
    result = SimulationMicroMotionResult(
        requested=True,
        status=status,
        simulation_only=True,
        real_robot_allowed=False,
        real_robot_motion_executed=False,
        robot_motion_executed=robot_motion_executed,
        command_type=MICRO_MOTION_COMMAND_TYPE,
        joint_name=request.joint_name,
        requested_delta_rad=float(request.requested_delta_rad),
        actual_delta_rad=actual_delta,
        tolerance_rad=float(request.tolerance_rad),
        delta_within_tolerance=delta_within_tolerance,
        before_joint_position_rad=before_joint_position,
        after_joint_position_rad=after_joint_position,
        blocking_reasons=_unique(blocking_reasons),
        warnings=_unique(warnings),
        errors=_unique(errors),
    ).to_dict()
    result.update(
        {
            "control_enabled": False,
            "simulation_control_enabled": robot_motion_executed,
            "motion_generated": robot_motion_executed,
            "command_generated": False,
            "simulation_command_generated": robot_motion_executed,
            "joint_targets_generated": False,
            "simulation_joint_delta_generated": robot_motion_executed,
            "trajectory_generated": False,
            "tcp_pose_world_generated": False,
            "safety_boundary": {
                "simulation_only": True,
                "no_real_robot": True,
                "local_isaac_api_only": True,
                "no_trajectory": True,
                "no_tcp_pose_world": True,
            },
        }
    )
    return result


def _with_articulation_state_snapshots(
    result: Dict[str, Any],
    *,
    before_state: Dict[str, Any],
    after_state: Dict[str, Any],
    precheck: Dict[str, Any],
    readiness: Dict[str, Any],
) -> Dict[str, Any]:
    enriched = {
        **result,
        "simulation_micro_motion_requested": True,
        "simulation_micro_motion_status": result["status"],
        "precheck": {
            "simulation_motion_precheck_status": precheck.get("status"),
            "ready_for_simulation_motion": precheck.get("ready", False),
            "articulation_readiness_status": readiness.get("readiness_status", readiness.get("status")),
            "articulation_state_status": before_state.get("status"),
            "blocking_reasons": _precheck_blocking_reasons(precheck),
            "warnings": _list(precheck.get("warnings")),
            "errors": _list(precheck.get("errors")),
        },
        "motion": {
            "command_type": result["command_type"],
            "joint_name": result["joint_name"],
            "requested_delta_rad": result["requested_delta_rad"],
            "actual_delta_rad": result["actual_delta_rad"],
            "tolerance_rad": result["tolerance_rad"],
            "delta_within_tolerance": result["delta_within_tolerance"],
            "before_joint_position_rad": result["before_joint_position_rad"],
            "after_joint_position_rad": result["after_joint_position_rad"],
            "before_joint_state_path": None,
            "after_joint_state_path": None,
            "simulation_motion_result_path": None,
            "simulation_motion_report_path": None,
        },
        "before_articulation_state": before_state,
        "after_articulation_state": after_state,
    }
    enriched["motion_diff_summary"] = build_joint_diff_summary(enriched)
    return enriched


def _precheck_gate_blocking_reasons(
    *,
    precheck: Dict[str, Any],
    readiness: Dict[str, Any],
    before_state: Dict[str, Any],
) -> list[str]:
    blockers = []
    if precheck.get("status") != "READY_FOR_SIMULATION_MOTION":
        blockers.append("E_SIMULATION_MOTION_PRECHECK_NOT_READY")
    if precheck.get("ready") is not True:
        blockers.append("E_READY_FOR_SIMULATION_MOTION_FALSE")
    if readiness.get("readiness_status", readiness.get("status")) != "READY":
        blockers.append("E_ARTICULATION_READINESS_NOT_READY")
    if before_state.get("status") != "OK":
        blockers.append("E_ARTICULATION_STATE_NOT_OK")
    return _unique(blockers + _precheck_blocking_reasons(precheck))


def _precheck_blocking_reasons(precheck: Dict[str, Any]) -> list[str]:
    return _list(precheck.get("blocking_reasons"))


def _available_arm_joint_names(before_state: Dict[str, Any], precheck: Dict[str, Any]) -> list[str]:
    names = _list(before_state.get("observed_arm_joint_names")) or _list(precheck.get("observed_arm_joint_names"))
    return names or list(UR5E_ARM_JOINT_NAMES)


def _joint_position(state: Dict[str, Any], joint_name: str) -> float | None:
    for row in state.get("joint_state_table") or []:
        if isinstance(row, dict) and row.get("joint_name") == joint_name:
            value = row.get("position")
            if isinstance(value, bool):
                return None
            if isinstance(value, (int, float)):
                return float(value)
    return None


def _list(value: Any) -> list[str]:
    if not isinstance(value, list):
        return []
    return [str(item) for item in value]


def _unique(values: list[str]) -> list[str]:
    unique_values = []
    for value in values:
        if value and value not in unique_values:
            unique_values.append(value)
    return unique_values


def _format_value(value: Any) -> str:
    if value is None:
        return "null"
    if isinstance(value, (list, dict)):
        return json.dumps(value, ensure_ascii=False)
    return str(value)


def _number_or_none(value: Any) -> float | None:
    if isinstance(value, bool):
        return None
    if isinstance(value, (int, float)):
        return float(value)
    return None
