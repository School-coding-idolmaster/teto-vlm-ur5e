from __future__ import annotations

from typing import Any, Dict

from src.robot_prim_inspector import UR5E_ARM_JOINT_NAMES


READINESS_STATUS_NOT_REQUESTED = "NOT_REQUESTED"
READINESS_STATUS_READY = "READY"
READINESS_STATUS_NOT_READY = "NOT_READY"
READINESS_STATUS_EVALUATION_FAILED = "EVALUATION_FAILED"
REQUIRED_ARM_JOINT_COUNT = 6


def build_articulation_readiness_report(
    *,
    requested: bool = False,
    robot_prim_path: str | None = None,
    robot_prim_inspection: Dict[str, Any] | None = None,
    has_robot_structure_report: bool = False,
    readiness_status: str | None = None,
    warnings: list[str] | None = None,
) -> Dict[str, Any]:
    if not requested:
        return _report(
            requested=False,
            readiness_status=READINESS_STATUS_NOT_REQUESTED,
            robot_prim_path=robot_prim_path,
            has_robot_structure_report=has_robot_structure_report,
            warnings=warnings or [],
        )

    try:
        inspection = robot_prim_inspection if isinstance(robot_prim_inspection, dict) else {}
        joint_summary = inspection.get("joint_metadata_summary")
        if not isinstance(joint_summary, dict):
            joint_summary = {}

        resolved_robot_prim_path = robot_prim_path or inspection.get("robot_prim_path")
        robot_prim_exists = bool(inspection.get("robot_prim_exists"))
        articulation_root_found = bool(inspection.get("articulation_root_found"))
        arm_joint_names = _string_list(joint_summary.get("arm_joint_names"))
        arm_joint_count = len(arm_joint_names)
        missing_arm_joint_names = [
            joint_name for joint_name in UR5E_ARM_JOINT_NAMES if joint_name not in set(arm_joint_names)
        ]
        joint_names = _string_list(inspection.get("joint_names")) or _string_list(
            joint_summary.get("possible_dof_names")
        )
        extra_joint_like_names = [
            joint_name for joint_name in joint_names if joint_name not in set(UR5E_ARM_JOINT_NAMES)
        ]
        visual_like_prim_count = int(inspection.get("visual_like_prim_count") or 0)
        collision_like_prim_count = int(inspection.get("collision_like_prim_count") or 0)
        has_visual_prims = visual_like_prim_count > 0
        has_collision_prims = collision_like_prim_count > 0

        missing_requirements = []
        if not robot_prim_exists:
            missing_requirements.append("robot_prim")
        if not articulation_root_found:
            missing_requirements.append("articulation_root")
        if arm_joint_count != REQUIRED_ARM_JOINT_COUNT or missing_arm_joint_names:
            missing_requirements.append("six_standard_ur5e_arm_joints")
        if not has_visual_prims:
            missing_requirements.append("visual_prims")
        if not has_collision_prims:
            missing_requirements.append("collision_prims")

        resolved_status = readiness_status or (
            READINESS_STATUS_READY if not missing_requirements else READINESS_STATUS_NOT_READY
        )
        return _report(
            requested=True,
            readiness_status=resolved_status,
            robot_prim_path=resolved_robot_prim_path,
            articulation_root_found=articulation_root_found,
            arm_joint_count=arm_joint_count,
            arm_joint_names=arm_joint_names,
            missing_arm_joint_names=missing_arm_joint_names,
            extra_joint_like_names=extra_joint_like_names,
            has_visual_prims=has_visual_prims,
            has_collision_prims=has_collision_prims,
            has_robot_structure_report=has_robot_structure_report,
            missing_requirements=missing_requirements,
            warnings=warnings or [],
        )
    except Exception as exc:
        return _report(
            requested=True,
            readiness_status=READINESS_STATUS_EVALUATION_FAILED,
            robot_prim_path=robot_prim_path,
            has_robot_structure_report=has_robot_structure_report,
            missing_requirements=["readiness_evaluation"],
            warnings=[str(exc)],
        )


def _report(
    *,
    requested: bool,
    readiness_status: str,
    robot_prim_path: str | None,
    articulation_root_found: bool = False,
    arm_joint_count: int = 0,
    arm_joint_names: list[str] | None = None,
    missing_arm_joint_names: list[str] | None = None,
    extra_joint_like_names: list[str] | None = None,
    has_visual_prims: bool = False,
    has_collision_prims: bool = False,
    has_robot_structure_report: bool = False,
    missing_requirements: list[str] | None = None,
    warnings: list[str] | None = None,
) -> Dict[str, Any]:
    articulation_ready = requested and readiness_status == READINESS_STATUS_READY
    return {
        "requested": requested,
        "readiness_status": readiness_status,
        "articulation_ready": articulation_ready,
        "control_enabled": False,
        "motion_generated": False,
        "command_generated": False,
        "robot_prim_path": robot_prim_path,
        "articulation_root_found": articulation_root_found,
        "arm_joint_count": int(arm_joint_count),
        "required_arm_joint_count": REQUIRED_ARM_JOINT_COUNT,
        "arm_joint_names": arm_joint_names or [],
        "missing_arm_joint_names": missing_arm_joint_names or [],
        "extra_joint_like_names": extra_joint_like_names or [],
        "has_visual_prims": has_visual_prims,
        "has_collision_prims": has_collision_prims,
        "has_robot_structure_report": has_robot_structure_report,
        "missing_requirements": missing_requirements or [],
        "warnings": warnings or [],
        "safety_boundary": {
            "read_only": True,
            "no_robot_motion": True,
            "no_joint_targets": True,
            "no_tcp_pose_world": True,
            "no_ros2_moveit_rtde_urscript": True,
        },
    }


def _string_list(value: Any) -> list[str]:
    if not isinstance(value, list):
        return []
    return [str(item) for item in value]
