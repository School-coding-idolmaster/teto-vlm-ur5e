from __future__ import annotations

import json
from pathlib import Path
from typing import Any, Dict

from src.robot_prim_inspector import UR5E_ARM_JOINT_NAMES


PRECHECK_STATUS_NOT_REQUESTED = "NOT_REQUESTED"
PRECHECK_STATUS_READY = "READY_FOR_SIMULATION_MOTION"
PRECHECK_STATUS_NOT_READY = "NOT_READY"
PRECHECK_STATUS_ERROR = "ERROR"

CHECKED_REQUIREMENTS = [
    "robot_asset_loaded",
    "robot_prim_exists",
    "robot_prim_path",
    "articulation_readiness_ready",
    "articulation_root_found",
    "six_standard_ur5e_arm_joints",
    "articulation_state_ok",
    "articulation_state_observable",
    "observed_joint_count_at_least_6",
    "arm_joint_count_6",
    "no_missing_arm_joints",
    "joint_positions_available",
    "joint_velocities_available",
    "joint_limits_available",
    "arm_joint_positions_within_limits",
]


def evaluate_simulation_motion_precheck(
    *,
    requested: bool = False,
    robot_asset_loaded: bool = False,
    robot_prim_exists: bool = False,
    robot_prim_path: str | None = None,
    robot_prim_inspection: Dict[str, Any] | None = None,
    articulation_readiness: Dict[str, Any] | None = None,
    articulation_state: Dict[str, Any] | None = None,
) -> Dict[str, Any]:
    return build_simulation_motion_precheck_report(
        requested=requested,
        robot_asset_loaded=robot_asset_loaded,
        robot_prim_exists=robot_prim_exists,
        robot_prim_path=robot_prim_path,
        robot_prim_inspection=robot_prim_inspection,
        articulation_readiness=articulation_readiness,
        articulation_state=articulation_state,
    )


def build_simulation_motion_precheck_report(
    *,
    requested: bool = False,
    robot_asset_loaded: bool = False,
    robot_prim_exists: bool = False,
    robot_prim_path: str | None = None,
    robot_prim_inspection: Dict[str, Any] | None = None,
    articulation_readiness: Dict[str, Any] | None = None,
    articulation_state: Dict[str, Any] | None = None,
) -> Dict[str, Any]:
    if not requested:
        return _report(
            requested=False,
            status=PRECHECK_STATUS_NOT_REQUESTED,
            ready=False,
            robot_prim_path=robot_prim_path,
        )

    try:
        inspection = robot_prim_inspection if isinstance(robot_prim_inspection, dict) else {}
        readiness = articulation_readiness if isinstance(articulation_readiness, dict) else {}
        state = articulation_state if isinstance(articulation_state, dict) else {}
        resolved_robot_prim_path = robot_prim_path or readiness.get("robot_prim_path") or state.get("robot_prim_path") or inspection.get("robot_prim_path")
        expected_arm_joint_names = _string_list(state.get("expected_arm_joint_names")) or list(UR5E_ARM_JOINT_NAMES)
        observed_arm_joint_names = _string_list(state.get("observed_arm_joint_names"))
        missing_arm_joint_names = _string_list(state.get("missing_arm_joint_names"))
        extra_joint_names = _string_list(state.get("extra_joint_names"))
        joint_rows = [_normalize_precheck_row(row) for row in _joint_state_table(state)]
        arm_rows = [row for row in joint_rows if row["category"] == "arm"]
        warnings = []
        errors = []
        missing_requirements = []
        blocking_reasons = []

        def require(condition: bool, requirement: str, reason: str) -> None:
            if not condition:
                missing_requirements.append(requirement)
                blocking_reasons.append(reason)

        readiness_status = readiness.get("readiness_status", readiness.get("status"))
        articulation_ready = readiness.get("articulation_ready") is True
        articulation_root_found = readiness.get("articulation_root_found") is True or inspection.get("articulation_root_found") is True
        readiness_arm_count = int(readiness.get("arm_joint_count") or 0)

        state_status = state.get("status")
        observed_joint_count = int(state.get("observed_joint_count") or len(joint_rows))
        arm_joint_count = int(state.get("arm_joint_count") or len(observed_arm_joint_names))
        joint_positions_available = state.get("joint_positions_available") is True
        joint_velocities_available = state.get("joint_velocities_available") is True
        joint_limits_available = state.get("joint_limits_available") is True
        articulation_state_observable = state.get("articulation_state_observable") is True

        require(robot_asset_loaded is True, "robot_asset_loaded", "E_ROBOT_ASSET_NOT_LOADED")
        require(robot_prim_exists is True, "robot_prim_exists", "E_ROBOT_PRIM_NOT_FOUND")
        require(bool(resolved_robot_prim_path), "robot_prim_path", "E_ROBOT_PRIM_PATH_MISSING")
        require(readiness_status == "READY", "articulation_readiness_ready", "E_ARTICULATION_READINESS_NOT_READY")
        require(articulation_ready, "articulation_ready", "E_ARTICULATION_NOT_READY")
        require(articulation_root_found, "articulation_root_found", "E_ARTICULATION_ROOT_NOT_FOUND")
        require(readiness_arm_count == 6, "six_standard_ur5e_arm_joints", "E_ARM_JOINT_COUNT_NOT_READY")
        require(state_status == "OK", "articulation_state_ok", "E_ARTICULATION_STATE_NOT_OK")
        require(articulation_state_observable, "articulation_state_observable", "E_ARTICULATION_STATE_NOT_OBSERVABLE")
        require(observed_joint_count >= 6, "observed_joint_count_at_least_6", "E_OBSERVED_JOINT_COUNT_TOO_LOW")
        require(arm_joint_count == 6, "arm_joint_count_6", "E_ARM_JOINT_COUNT_INVALID")
        require(missing_arm_joint_names == [], "no_missing_arm_joints", "E_MISSING_ARM_JOINTS")
        require(joint_positions_available, "joint_positions_available", "E_JOINT_POSITIONS_UNAVAILABLE")
        require(joint_velocities_available, "joint_velocities_available", "E_JOINT_VELOCITIES_UNAVAILABLE")
        require(joint_limits_available, "joint_limits_available", "E_JOINT_LIMITS_UNAVAILABLE")

        arm_missing_limits = [row["joint_name"] for row in arm_rows if row["limit_available"] is not True]
        if arm_missing_limits:
            warnings.append(f"joint limits unavailable for: {', '.join(arm_missing_limits)}")
            if "joint_limits_available" not in missing_requirements:
                missing_requirements.append("joint_limits_available")
                blocking_reasons.append("E_JOINT_LIMITS_UNAVAILABLE")

        arm_out_of_limit = [row["joint_name"] for row in arm_rows if row["within_limit"] is False]
        if arm_out_of_limit:
            errors.append(f"E_JOINT_LIMIT_VIOLATION: {', '.join(arm_out_of_limit)}")
            missing_requirements.append("arm_joint_positions_within_limits")
            blocking_reasons.append("E_JOINT_LIMIT_VIOLATION")

        arm_position_unknown = [
            row["joint_name"]
            for row in arm_rows
            if row["limit_available"] is True and row["position"] is None
        ]
        if arm_position_unknown:
            warnings.append(f"joint positions unavailable for limit check: {', '.join(arm_position_unknown)}")
            if "joint_positions_available" not in missing_requirements:
                missing_requirements.append("joint_positions_available")
                blocking_reasons.append("E_JOINT_POSITIONS_UNAVAILABLE")

        all_arm_within_limits = bool(arm_rows) and all(row["within_limit"] is True for row in arm_rows)
        require(all_arm_within_limits, "arm_joint_positions_within_limits", "E_ARM_JOINT_LIMIT_CHECK_NOT_READY")

        ready = not blocking_reasons
        return _report(
            requested=True,
            status=PRECHECK_STATUS_READY if ready else PRECHECK_STATUS_NOT_READY,
            ready=ready,
            robot_prim_path=resolved_robot_prim_path,
            checked_requirements=CHECKED_REQUIREMENTS,
            missing_requirements=_unique(missing_requirements),
            blocking_reasons=_unique(blocking_reasons),
            warnings=_unique(warnings + _string_list(state.get("warnings"))),
            errors=_unique(errors + _string_list(state.get("errors"))),
            arm_joint_count=arm_joint_count,
            observed_joint_count=observed_joint_count,
            expected_arm_joint_names=expected_arm_joint_names,
            observed_arm_joint_names=observed_arm_joint_names,
            missing_arm_joint_names=missing_arm_joint_names,
            extra_joint_names=extra_joint_names,
            non_arm_extra_joints=extra_joint_names,
            joint_limits_available=joint_limits_available and not arm_missing_limits,
            joint_positions_within_limits=all_arm_within_limits,
            joint_precheck_table=joint_rows,
        )
    except Exception as exc:
        return _report(
            requested=True,
            status=PRECHECK_STATUS_ERROR,
            ready=False,
            robot_prim_path=robot_prim_path,
            missing_requirements=["simulation_motion_precheck"],
            blocking_reasons=["E_SIMULATION_MOTION_PRECHECK_FAILED"],
            errors=[str(exc)],
        )


def format_simulation_motion_precheck_summary(precheck: Dict[str, Any]) -> str:
    return "\n".join(
        [
            f"simulation_motion_precheck_status: {precheck.get('status')}",
            f"ready_for_simulation_motion: {precheck.get('ready')}",
            f"blocking_reasons: {precheck.get('blocking_reasons', [])}",
            f"warnings: {precheck.get('warnings', [])}",
            f"errors: {precheck.get('errors', [])}",
        ]
    )


def write_simulation_motion_precheck_artifacts(
    precheck: Dict[str, Any],
    output_dir: str | Path,
) -> Dict[str, Path]:
    run_dir = Path(output_dir)
    run_dir.mkdir(parents=True, exist_ok=True)
    json_path = run_dir / "simulation_motion_precheck.json"
    report_path = run_dir / "simulation_motion_precheck_report.md"
    with json_path.open("w", encoding="utf-8") as json_file:
        json.dump(precheck, json_file, ensure_ascii=False, indent=2)
        json_file.write("\n")
    report_path.write_text(_build_precheck_markdown(precheck), encoding="utf-8")
    return {
        "simulation_motion_precheck_path": json_path,
        "simulation_motion_precheck_report_path": report_path,
    }


def _report(
    *,
    requested: bool,
    status: str,
    ready: bool,
    robot_prim_path: str | None,
    checked_requirements: list[str] | None = None,
    missing_requirements: list[str] | None = None,
    blocking_reasons: list[str] | None = None,
    warnings: list[str] | None = None,
    errors: list[str] | None = None,
    arm_joint_count: int = 0,
    observed_joint_count: int = 0,
    expected_arm_joint_names: list[str] | None = None,
    observed_arm_joint_names: list[str] | None = None,
    missing_arm_joint_names: list[str] | None = None,
    extra_joint_names: list[str] | None = None,
    non_arm_extra_joints: list[str] | None = None,
    joint_limits_available: bool = False,
    joint_positions_within_limits: bool = False,
    joint_precheck_table: list[Dict[str, Any]] | None = None,
) -> Dict[str, Any]:
    return {
        "requested": requested,
        "status": status,
        "ready": ready,
        "metadata_only": True,
        "simulation_only": True,
        "control_enabled": False,
        "motion_generated": False,
        "command_generated": False,
        "joint_targets_generated": False,
        "trajectory_generated": False,
        "tcp_pose_world_generated": False,
        "robot_motion_executed": False,
        "real_robot_allowed": False,
        "robot_prim_path": robot_prim_path,
        "checked_requirements": checked_requirements or [],
        "missing_requirements": missing_requirements or [],
        "blocking_reasons": blocking_reasons or [],
        "warnings": warnings or [],
        "errors": errors or [],
        "arm_joint_count": int(arm_joint_count),
        "observed_joint_count": int(observed_joint_count),
        "expected_arm_joint_names": expected_arm_joint_names or list(UR5E_ARM_JOINT_NAMES),
        "observed_arm_joint_names": observed_arm_joint_names or [],
        "missing_arm_joint_names": missing_arm_joint_names or [],
        "extra_joint_names": extra_joint_names or [],
        "non_arm_extra_joints": non_arm_extra_joints or [],
        "joint_limits_available": joint_limits_available,
        "joint_positions_within_limits": joint_positions_within_limits,
        "joint_precheck_table": joint_precheck_table or [],
        "safety_boundary": {
            "metadata_only": True,
            "simulation_only": True,
            "no_robot_motion": True,
            "no_joint_targets": True,
            "no_trajectory": True,
            "no_tcp_pose_world": True,
            "no_ros2_moveit_rtde_urscript": True,
            "no_real_robot": True,
        },
    }


def _joint_state_table(state: Dict[str, Any]) -> list[Dict[str, Any]]:
    rows = state.get("joint_state_table")
    if not isinstance(rows, list):
        return []
    return [row for row in rows if isinstance(row, dict)]


def _normalize_precheck_row(row: Dict[str, Any]) -> Dict[str, Any]:
    category = row.get("category") if row.get("category") in {"arm", "structural", "gripper_or_tool", "unknown"} else "unknown"
    limit_available = row.get("limit_available") is True
    within_limit = row.get("within_limit") if row.get("within_limit") in {True, False} else None
    precheck_passed = category != "arm" or (limit_available and within_limit is True)
    blocking_reason = None
    if category == "arm" and not limit_available:
        blocking_reason = "E_JOINT_LIMITS_UNAVAILABLE"
    elif category == "arm" and within_limit is not True:
        blocking_reason = "E_JOINT_LIMIT_CHECK_NOT_READY"
    return {
        "joint_name": str(row.get("joint_name") or "unknown"),
        "category": category,
        "position": _number_or_none(row.get("position")),
        "velocity": _number_or_none(row.get("velocity")),
        "lower_limit": _number_or_none(row.get("lower_limit")),
        "upper_limit": _number_or_none(row.get("upper_limit")),
        "limit_available": limit_available,
        "within_limit": within_limit,
        "precheck_passed": precheck_passed,
        "blocking_reason": blocking_reason,
        "metadata_only": True,
        "control_target_generated": False,
    }


def _build_precheck_markdown(precheck: Dict[str, Any]) -> str:
    lines = [
        "# TETO Simulation Motion Precheck Report",
        "",
        "This report is simulation-only precheck evidence. It is metadata/state/readiness only and does not move the robot.",
        "",
        "## Summary",
        "",
        format_simulation_motion_precheck_summary(precheck),
    ]
    return "\n".join(lines) + "\n"


def _string_list(value: Any) -> list[str]:
    if not isinstance(value, list):
        return []
    return [str(item) for item in value]


def _number_or_none(value: Any) -> float | None:
    if value is None:
        return None
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def _unique(values: list[str]) -> list[str]:
    seen = set()
    result = []
    for value in values:
        if value not in seen:
            result.append(value)
            seen.add(value)
    return result
