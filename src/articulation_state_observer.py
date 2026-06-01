from __future__ import annotations

from typing import Any, Dict, Iterable

from src.robot_prim_inspector import UR5E_ARM_JOINT_NAMES, categorize_joint_name


OBSERVATION_STATUS_NOT_REQUESTED = "NOT_REQUESTED"
OBSERVATION_STATUS_OK = "OK"
OBSERVATION_STATUS_NOT_OBSERVABLE = "NOT_OBSERVABLE"
OBSERVATION_STATUS_NOT_READY = "NOT_READY"
OBSERVATION_STATUS_ERROR = "ERROR"


def build_articulation_state_report(
    *,
    requested: bool = False,
    robot_prim_path: str | None = None,
    robot_prim_inspection: Dict[str, Any] | None = None,
    articulation_readiness: Dict[str, Any] | None = None,
    joint_state_table: list[Dict[str, Any]] | None = None,
    status: str | None = None,
    warnings: list[str] | None = None,
    errors: list[str] | None = None,
) -> Dict[str, Any]:
    if not requested:
        return _report(
            requested=False,
            status=OBSERVATION_STATUS_NOT_REQUESTED,
            robot_prim_path=robot_prim_path,
            joint_state_table=[],
            warnings=warnings or [],
            errors=errors or [],
        )

    inspection = robot_prim_inspection if isinstance(robot_prim_inspection, dict) else {}
    readiness = articulation_readiness if isinstance(articulation_readiness, dict) else {}
    normalized_rows = [_normalize_joint_state_row(row) for row in (joint_state_table or _rows_from_inspection(inspection))]
    normalized_warnings = list(warnings or [])
    normalized_errors = list(errors or [])
    observed_arm_joint_names = [row["joint_name"] for row in normalized_rows if row["category"] == "arm"]
    missing_arm_joint_names = [
        joint_name for joint_name in UR5E_ARM_JOINT_NAMES if joint_name not in set(observed_arm_joint_names)
    ]
    extra_joint_names = [row["joint_name"] for row in normalized_rows if row["category"] != "arm"]

    if not inspection.get("robot_prim_exists"):
        resolved_status = status or OBSERVATION_STATUS_NOT_OBSERVABLE
    elif readiness and readiness.get("articulation_ready") is not True:
        resolved_status = status or OBSERVATION_STATUS_NOT_READY
    elif not normalized_rows:
        resolved_status = status or OBSERVATION_STATUS_NOT_OBSERVABLE
    else:
        resolved_status = status or OBSERVATION_STATUS_OK

    missing_limit_names = [
        row["joint_name"]
        for row in normalized_rows
        if row["category"] == "arm" and row["limit_available"] is not True
    ]
    if missing_limit_names and not any("joint limits unavailable" in item for item in normalized_warnings):
        normalized_warnings.append(f"joint limits unavailable for: {', '.join(missing_limit_names)}")

    out_of_limit_names = [row["joint_name"] for row in normalized_rows if row["within_limit"] is False]
    if out_of_limit_names and not any("E_JOINT_LIMIT_VIOLATION" in item for item in normalized_errors):
        normalized_errors.append(f"E_JOINT_LIMIT_VIOLATION: {', '.join(out_of_limit_names)}")

    return _report(
        requested=True,
        status=resolved_status,
        robot_prim_path=robot_prim_path or inspection.get("robot_prim_path") or readiness.get("robot_prim_path"),
        joint_state_table=normalized_rows,
        observed_arm_joint_names=observed_arm_joint_names,
        missing_arm_joint_names=missing_arm_joint_names,
        extra_joint_names=extra_joint_names,
        warnings=normalized_warnings,
        errors=normalized_errors,
    )


def observe_articulation_state(
    world=None,
    *,
    robot_prim_path: str,
    robot_prim_inspection: Dict[str, Any] | None = None,
    articulation_readiness: Dict[str, Any] | None = None,
) -> Dict[str, Any]:
    try:
        inspection = robot_prim_inspection if isinstance(robot_prim_inspection, dict) else {}
        stage = _stage_from_world(world)
        rows = []
        for row in _rows_from_inspection(inspection):
            joint_prim_path = row.get("joint_prim_path")
            prim = _get_stage_prim(stage, joint_prim_path)
            rows.append(
                {
                    **row,
                    "position": _first_attribute_value(
                        prim,
                        "state:angular:physics:position",
                        "state:linear:physics:position",
                        "physics:position",
                    ),
                    "velocity": _first_attribute_value(
                        prim,
                        "state:angular:physics:velocity",
                        "state:linear:physics:velocity",
                        "physics:velocity",
                    ),
                    "lower_limit": _first_attribute_value(prim, "physics:lowerLimit", "lowerLimit"),
                    "upper_limit": _first_attribute_value(prim, "physics:upperLimit", "upperLimit"),
                }
            )

        return build_articulation_state_report(
            requested=True,
            robot_prim_path=robot_prim_path,
            robot_prim_inspection=inspection,
            articulation_readiness=articulation_readiness,
            joint_state_table=rows,
        )
    except Exception as exc:
        return build_articulation_state_report(
            requested=True,
            robot_prim_path=robot_prim_path,
            robot_prim_inspection=robot_prim_inspection,
            articulation_readiness=articulation_readiness,
            status=OBSERVATION_STATUS_ERROR,
            warnings=[str(exc)],
        )


def _report(
    *,
    requested: bool,
    status: str,
    robot_prim_path: str | None,
    joint_state_table: list[Dict[str, Any]],
    observed_arm_joint_names: list[str] | None = None,
    missing_arm_joint_names: list[str] | None = None,
    extra_joint_names: list[str] | None = None,
    warnings: list[str] | None = None,
    errors: list[str] | None = None,
) -> Dict[str, Any]:
    rows = joint_state_table or []
    return {
        "requested": requested,
        "status": status,
        "metadata_only": True,
        "control_enabled": False,
        "motion_generated": False,
        "command_generated": False,
        "joint_targets_generated": False,
        "robot_prim_path": robot_prim_path,
        "articulation_state_observable": requested and status == OBSERVATION_STATUS_OK,
        "arm_joint_count": len(observed_arm_joint_names or []),
        "observed_joint_count": len(rows),
        "expected_arm_joint_names": list(UR5E_ARM_JOINT_NAMES),
        "observed_arm_joint_names": observed_arm_joint_names if observed_arm_joint_names is not None else [],
        "missing_arm_joint_names": (
            missing_arm_joint_names if missing_arm_joint_names is not None else list(UR5E_ARM_JOINT_NAMES)
        ),
        "extra_joint_names": extra_joint_names if extra_joint_names is not None else [],
        "joint_positions_available": any(row.get("position") is not None for row in rows),
        "joint_velocities_available": any(row.get("velocity") is not None for row in rows),
        "joint_limits_available": _arm_joint_limits_available(rows),
        "joint_state_table": rows,
        "warnings": warnings or [],
        "errors": errors or [],
        "safety_boundary": {
            "read_only": True,
            "no_robot_motion": True,
            "no_joint_targets": True,
            "no_tcp_pose_world": True,
            "no_trajectory": True,
            "no_ros2_moveit_rtde_urscript": True,
        },
    }


def _rows_from_inspection(inspection: Dict[str, Any]) -> list[Dict[str, Any]]:
    table = inspection.get("joint_metadata_table")
    if isinstance(table, list) and table:
        return [row for row in table if isinstance(row, dict)]

    joint_names = _string_list(inspection.get("joint_names")) or _string_list(
        (inspection.get("joint_metadata_summary") or {}).get("possible_dof_names")
        if isinstance(inspection.get("joint_metadata_summary"), dict)
        else []
    )
    joint_paths = _string_list(inspection.get("joint_prim_paths"))
    return [
        {
            "joint_name": joint_name,
            "joint_prim_path": _list_get(joint_paths, index),
            "category": categorize_joint_name(joint_name),
        }
        for index, joint_name in enumerate(joint_names)
    ]


def _normalize_joint_state_row(row: Dict[str, Any]) -> Dict[str, Any]:
    joint_name = str(row.get("joint_name") or "unknown")
    category = row.get("category") if row.get("category") in {"arm", "structural", "gripper_or_tool", "unknown"} else categorize_joint_name(joint_name)
    position = _number_or_none(row.get("position"))
    velocity = _number_or_none(row.get("velocity"))
    lower_limit = _number_or_none(row.get("lower_limit"))
    upper_limit = _number_or_none(row.get("upper_limit"))
    limit_available = lower_limit is not None and upper_limit is not None
    within_limit = None
    if limit_available and position is not None:
        within_limit = lower_limit <= position <= upper_limit

    return {
        "joint_name": joint_name,
        "category": category,
        "position": position,
        "velocity": velocity,
        "lower_limit": lower_limit,
        "upper_limit": upper_limit,
        "limit_available": limit_available,
        "within_limit": within_limit,
        "metadata_only": True,
        "control_target_generated": False,
        "joint_prim_path": row.get("joint_prim_path"),
    }


def _arm_joint_limits_available(rows: list[Dict[str, Any]]) -> bool:
    arm_rows = [row for row in rows if row.get("category") == "arm"]
    return arm_rows != [] and all(row.get("limit_available") is True for row in arm_rows)


def _stage_from_world(world):
    stage = getattr(world, "stage", None)
    if stage is not None:
        return stage
    try:
        import omni.usd

        return omni.usd.get_context().get_stage()
    except Exception:
        return None


def _get_stage_prim(stage, prim_path: str | None):
    if stage is None or not prim_path or not hasattr(stage, "GetPrimAtPath"):
        return None
    try:
        prim = stage.GetPrimAtPath(prim_path)
    except Exception:
        return None
    if not _prim_is_valid(prim):
        return None
    return prim


def _first_attribute_value(prim, *attribute_names: str) -> Any:
    for name in attribute_names:
        value = _attribute_value(prim, name)
        if value is not None:
            return value
    return None


def _attribute_value(prim, attribute_name: str) -> Any:
    if prim is None:
        return None
    get_attribute = getattr(prim, "GetAttribute", None)
    if not callable(get_attribute):
        attributes = getattr(prim, "attributes", {})
        return attributes.get(attribute_name) if isinstance(attributes, dict) else None
    try:
        attribute = get_attribute(attribute_name)
        if attribute is None:
            return None
        get = getattr(attribute, "Get", None)
        return get() if callable(get) else None
    except Exception:
        return None


def _prim_is_valid(prim) -> bool:
    if prim is None:
        return False
    is_valid = getattr(prim, "IsValid", None)
    return bool(is_valid()) if callable(is_valid) else bool(prim)


def _number_or_none(value: Any) -> float | None:
    if isinstance(value, bool):
        return None
    if isinstance(value, (int, float)):
        return float(value)
    return None


def _string_list(value: Any) -> list[str]:
    if not isinstance(value, list):
        return []
    return [str(item) for item in value]


def _list_get(values: Iterable[Any], index: int, default=None):
    values = list(values)
    return values[index] if index < len(values) else default
