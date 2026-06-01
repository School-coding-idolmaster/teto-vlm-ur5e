from __future__ import annotations

import json
from pathlib import Path
from typing import Any, Dict

from src.simulation_micro_motion import write_simulation_micro_motion_artifacts


EVIDENCE_MANIFEST_SCHEMA_VERSION = "teto_evidence_manifest.v1"


def export_simulation_evidence(
    result: Dict[str, Any],
    run_dir: str | Path,
    *,
    demo_command: str | None = None,
) -> Dict[str, Path]:
    output_dir = Path(run_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    summary_path = output_dir / "summary.md"
    demo_command_path = output_dir / "demo_command.txt"
    pose_delta_path = output_dir / "pose_delta.md"
    manifest_path = output_dir / "evidence_manifest.json"
    robot_prim_inspection_path = output_dir / "robot_prim_inspection.json"
    robot_structure_report_path = output_dir / "robot_structure_report.md"
    articulation_readiness_path = output_dir / "articulation_readiness.json"
    articulation_state_path = output_dir / "articulation_state.json"
    articulation_state_report_path = output_dir / "articulation_state_report.md"
    simulation_motion_precheck_path = output_dir / "simulation_motion_precheck.json"
    simulation_motion_precheck_report_path = output_dir / "simulation_motion_precheck_report.md"
    simulation_motion_result_path = output_dir / "simulation_motion_result.json"
    simulation_motion_report_path = output_dir / "simulation_motion_report.md"
    before_articulation_state_path = output_dir / "before_articulation_state.json"
    after_articulation_state_path = output_dir / "after_articulation_state.json"

    object_info = _simulation_object_info(result)
    robot_asset_info = _robot_asset_info(result)
    robot_prim_inspection_info = _robot_prim_inspection_info(result)
    articulation_readiness_info = _articulation_readiness_info(result)
    articulation_state_info = _articulation_state_info(result)
    simulation_motion_precheck_info = _simulation_motion_precheck_info(result)
    simulation_micro_motion_info = _simulation_micro_motion_info(result)
    structure_report_requested = bool(robot_prim_inspection_info.get("requested"))
    readiness_requested = bool(articulation_readiness_info.get("requested"))
    state_requested = bool(articulation_state_info.get("requested"))
    precheck_requested = bool(simulation_motion_precheck_info.get("requested"))
    micro_motion_requested = bool(simulation_micro_motion_info.get("requested"))
    run_id = output_dir.name
    created_at = result.get("finished_at") or result.get("started_at")
    report_path = result.get("report_path")
    structure_report_ref = str(robot_structure_report_path) if structure_report_requested else None

    summary_path.write_text(
        _build_summary_markdown(
            result,
            object_info=object_info,
            robot_asset_info=robot_asset_info,
            robot_prim_inspection_info=robot_prim_inspection_info,
            articulation_readiness_info=articulation_readiness_info,
            articulation_state_info=articulation_state_info,
            simulation_motion_precheck_info=simulation_motion_precheck_info,
            simulation_micro_motion_info=simulation_micro_motion_info,
            robot_structure_report_path=structure_report_ref,
            run_id=run_id,
            created_at=created_at,
            report_path=report_path,
        ),
        encoding="utf-8",
    )
    demo_command_path.write_text(
        _build_demo_command_text(result, demo_command=demo_command),
        encoding="utf-8",
    )
    pose_delta_path.write_text(
        _build_pose_delta_markdown(result, object_info=object_info),
        encoding="utf-8",
    )
    if robot_prim_inspection_info.get("requested"):
        with robot_prim_inspection_path.open("w", encoding="utf-8") as inspection_file:
            json.dump(robot_prim_inspection_info, inspection_file, ensure_ascii=False, indent=2)
            inspection_file.write("\n")
        robot_structure_report_path.write_text(
            _build_robot_structure_report_markdown(
                result,
                robot_asset_info=robot_asset_info,
                robot_prim_inspection_info=robot_prim_inspection_info,
                articulation_readiness_info=articulation_readiness_info,
                articulation_state_info=articulation_state_info,
                simulation_motion_precheck_info=simulation_motion_precheck_info,
                run_id=run_id,
                report_path=report_path,
            ),
            encoding="utf-8",
        )
    if readiness_requested:
        with articulation_readiness_path.open("w", encoding="utf-8") as readiness_file:
            json.dump(articulation_readiness_info, readiness_file, ensure_ascii=False, indent=2)
            readiness_file.write("\n")
    if state_requested:
        with articulation_state_path.open("w", encoding="utf-8") as state_file:
            json.dump(articulation_state_info, state_file, ensure_ascii=False, indent=2)
            state_file.write("\n")
        articulation_state_report_path.write_text(
            _build_articulation_state_report_markdown(
                articulation_state_info,
                run_id=run_id,
                report_path=report_path,
            ),
            encoding="utf-8",
        )
    if precheck_requested:
        with simulation_motion_precheck_path.open("w", encoding="utf-8") as precheck_file:
            json.dump(simulation_motion_precheck_info, precheck_file, ensure_ascii=False, indent=2)
            precheck_file.write("\n")
        simulation_motion_precheck_report_path.write_text(
            _build_simulation_motion_precheck_report_markdown(
                simulation_motion_precheck_info,
                run_id=run_id,
                report_path=report_path,
            ),
            encoding="utf-8",
        )
    if micro_motion_requested:
        write_simulation_micro_motion_artifacts(simulation_micro_motion_info, output_dir)

    manifest = {
        "schema_version": EVIDENCE_MANIFEST_SCHEMA_VERSION,
        "run_id": run_id,
        "created_at": created_at,
        "teto_version": result.get("teto_version"),
        "mode": result.get("mode"),
        "status": result.get("status"),
        "report_path": str(report_path) if report_path else None,
        "summary_path": str(summary_path),
        "demo_command_path": str(demo_command_path),
        "pose_delta_path": str(pose_delta_path),
        "robot_asset": robot_asset_info,
        "robot_prim_inspection": robot_prim_inspection_info,
        "robot_prim_inspection_path": str(robot_prim_inspection_path)
        if robot_prim_inspection_info.get("requested")
        else None,
        "robot_structure_report_path": structure_report_ref,
        "articulation_readiness": articulation_readiness_info,
        "articulation_readiness_path": str(articulation_readiness_path) if readiness_requested else None,
        "articulation_state": articulation_state_info,
        "articulation_state_path": str(articulation_state_path) if state_requested else None,
        "articulation_state_report_path": str(articulation_state_report_path) if state_requested else None,
        "simulation_motion_precheck": simulation_motion_precheck_info,
        "simulation_motion_precheck_path": str(simulation_motion_precheck_path) if precheck_requested else None,
        "simulation_motion_precheck_report_path": (
            str(simulation_motion_precheck_report_path) if precheck_requested else None
        ),
        "simulation_micro_motion": simulation_micro_motion_info,
        "simulation_motion_result_path": str(simulation_motion_result_path) if micro_motion_requested else None,
        "simulation_motion_report_path": str(simulation_motion_report_path) if micro_motion_requested else None,
        "before_articulation_state_path": str(before_articulation_state_path) if micro_motion_requested else None,
        "after_articulation_state_path": str(after_articulation_state_path) if micro_motion_requested else None,
        "screenshot_before_path": None,
        "screenshot_after_path": None,
        "video_path": None,
    }
    with manifest_path.open("w", encoding="utf-8") as manifest_file:
        json.dump(manifest, manifest_file, ensure_ascii=False, indent=2)
        manifest_file.write("\n")

    return {
        "summary_path": summary_path,
        "demo_command_path": demo_command_path,
        "pose_delta_path": pose_delta_path,
        "evidence_manifest_path": manifest_path,
        "robot_prim_inspection_path": robot_prim_inspection_path,
        "robot_structure_report_path": robot_structure_report_path,
        "articulation_readiness_path": articulation_readiness_path,
        "articulation_state_path": articulation_state_path,
        "articulation_state_report_path": articulation_state_report_path,
        "simulation_motion_precheck_path": simulation_motion_precheck_path,
        "simulation_motion_precheck_report_path": simulation_motion_precheck_report_path,
        "simulation_motion_result_path": simulation_motion_result_path,
        "simulation_motion_report_path": simulation_motion_report_path,
        "before_articulation_state_path": before_articulation_state_path,
        "after_articulation_state_path": after_articulation_state_path,
    }


def _simulation_object_info(result: Dict[str, Any]) -> Dict[str, Any]:
    return {
        "object_type": result.get("simulation_object_type") or result.get("object_type"),
        "prim_path": result.get("simulation_object_prim_path") or result.get("cube_prim_path"),
        "initial_position": (
            result.get("simulation_object_initial_position")
            or result.get("cube_initial_position")
            or result.get("cube_position")
        ),
        "target_position": result.get("simulation_object_target_position") or result.get("cube_target_position"),
        "final_position": result.get("simulation_object_final_position") or result.get("cube_final_position"),
        "displacement": result.get("simulation_object_displacement") or result.get("cube_displacement"),
        "moved": result.get("simulation_object_moved")
        if result.get("simulation_object_moved") is not None
        else result.get("cube_moved"),
    }


def _robot_asset_info(result: Dict[str, Any]) -> Dict[str, Any]:
    return {
        "check_requested": result.get("robot_asset_check_requested"),
        "load_requested": result.get("robot_asset_load_requested"),
        "robot_type": result.get("robot_type"),
        "robot_prim_path": result.get("robot_prim_path"),
        "robot_asset_path": result.get("robot_asset_path"),
        "robot_asset_source": result.get("robot_asset_source"),
        "robot_asset_available": result.get("robot_asset_available"),
        "robot_asset_loaded": result.get("robot_asset_loaded"),
        "robot_prim_exists": result.get("robot_prim_exists"),
        "robot_asset_status": result.get("robot_asset_status"),
        "robot_asset_blocking_reason": result.get("robot_asset_blocking_reason"),
    }


def _robot_prim_inspection_info(result: Dict[str, Any]) -> Dict[str, Any]:
    inspection = result.get("robot_prim_inspection")
    if not isinstance(inspection, dict):
        inspection = {}
    return {
        "requested": result.get("robot_prim_inspection_requested", inspection.get("requested", False)),
        "robot_prim_path": inspection.get("robot_prim_path"),
        "robot_prim_exists": inspection.get("robot_prim_exists", False),
        "robot_root_type_name": inspection.get("robot_root_type_name"),
        "total_descendant_prim_count": inspection.get("total_descendant_prim_count", 0),
        "link_like_prim_count": inspection.get("link_like_prim_count", 0),
        "joint_like_prim_count": inspection.get("joint_like_prim_count", 0),
        "visual_like_prim_count": inspection.get("visual_like_prim_count", 0),
        "collision_like_prim_count": inspection.get("collision_like_prim_count", 0),
        "articulation_root_found": inspection.get("articulation_root_found", False),
        "physics_schema_summary": inspection.get("physics_schema_summary", []),
        "joint_names": inspection.get("joint_names", []),
        "joint_prim_paths": inspection.get("joint_prim_paths", []),
        "possible_dof_names": inspection.get("possible_dof_names", []),
        "possible_dof_count": inspection.get("possible_dof_count", 0),
        "joint_metadata_summary": inspection.get("joint_metadata_summary", {}),
        "joint_metadata_table": inspection.get("joint_metadata_table", []),
        "inspection_status": inspection.get("inspection_status", "NOT_REQUESTED"),
        "inspection_warnings": inspection.get("inspection_warnings", []),
    }


def _articulation_readiness_info(result: Dict[str, Any]) -> Dict[str, Any]:
    readiness = result.get("articulation_readiness")
    if not isinstance(readiness, dict):
        readiness = {}
    return {
        "requested": result.get("articulation_readiness_requested", readiness.get("requested", False)),
        "readiness_status": readiness.get("readiness_status", "NOT_REQUESTED"),
        "articulation_ready": readiness.get("articulation_ready", False),
        "control_enabled": readiness.get("control_enabled", False),
        "motion_generated": readiness.get("motion_generated", False),
        "command_generated": readiness.get("command_generated", False),
        "robot_prim_path": readiness.get("robot_prim_path"),
        "articulation_root_found": readiness.get("articulation_root_found", False),
        "arm_joint_count": readiness.get("arm_joint_count", 0),
        "required_arm_joint_count": readiness.get("required_arm_joint_count", 6),
        "arm_joint_names": readiness.get("arm_joint_names", []),
        "missing_arm_joint_names": readiness.get("missing_arm_joint_names", []),
        "extra_joint_like_names": readiness.get("extra_joint_like_names", []),
        "has_visual_prims": readiness.get("has_visual_prims", False),
        "has_collision_prims": readiness.get("has_collision_prims", False),
        "has_robot_structure_report": readiness.get("has_robot_structure_report", False),
        "missing_requirements": readiness.get("missing_requirements", []),
        "warnings": readiness.get("warnings", []),
        "safety_boundary": readiness.get(
            "safety_boundary",
            {
                "read_only": True,
                "no_robot_motion": True,
                "no_joint_targets": True,
                "no_tcp_pose_world": True,
                "no_ros2_moveit_rtde_urscript": True,
            },
        ),
    }


def _articulation_state_info(result: Dict[str, Any]) -> Dict[str, Any]:
    state = result.get("articulation_state")
    if not isinstance(state, dict):
        state = {}
    return {
        "requested": result.get("articulation_state_observation_requested", state.get("requested", False)),
        "status": state.get("status", "NOT_REQUESTED"),
        "metadata_only": state.get("metadata_only", True),
        "control_enabled": state.get("control_enabled", False),
        "motion_generated": state.get("motion_generated", False),
        "command_generated": state.get("command_generated", False),
        "joint_targets_generated": state.get("joint_targets_generated", False),
        "robot_prim_path": state.get("robot_prim_path"),
        "articulation_state_observable": result.get(
            "articulation_state_observable",
            state.get("articulation_state_observable", False),
        ),
        "arm_joint_count": state.get("arm_joint_count", 0),
        "observed_joint_count": state.get("observed_joint_count", 0),
        "expected_arm_joint_names": state.get("expected_arm_joint_names", []),
        "observed_arm_joint_names": state.get("observed_arm_joint_names", []),
        "missing_arm_joint_names": state.get("missing_arm_joint_names", []),
        "extra_joint_names": state.get("extra_joint_names", []),
        "joint_positions_available": state.get("joint_positions_available", False),
        "joint_velocities_available": state.get("joint_velocities_available", False),
        "joint_limits_available": state.get("joint_limits_available", False),
        "joint_state_table": state.get("joint_state_table", []),
        "warnings": state.get("warnings", []),
        "errors": state.get("errors", []),
        "safety_boundary": state.get(
            "safety_boundary",
            {
                "read_only": True,
                "no_robot_motion": True,
                "no_joint_targets": True,
                "no_tcp_pose_world": True,
                "no_trajectory": True,
                "no_ros2_moveit_rtde_urscript": True,
            },
        ),
    }


def _simulation_motion_precheck_info(result: Dict[str, Any]) -> Dict[str, Any]:
    precheck = result.get("simulation_motion_precheck")
    if not isinstance(precheck, dict):
        precheck = {}
    return {
        "requested": result.get("simulation_motion_precheck_requested", precheck.get("requested", False)),
        "status": result.get("simulation_motion_precheck_status", precheck.get("status", "NOT_REQUESTED")),
        "ready": result.get("ready_for_simulation_motion", precheck.get("ready", False)),
        "metadata_only": precheck.get("metadata_only", True),
        "simulation_only": precheck.get("simulation_only", True),
        "control_enabled": precheck.get("control_enabled", False),
        "motion_generated": precheck.get("motion_generated", False),
        "command_generated": precheck.get("command_generated", False),
        "joint_targets_generated": precheck.get("joint_targets_generated", False),
        "trajectory_generated": precheck.get("trajectory_generated", False),
        "tcp_pose_world_generated": precheck.get("tcp_pose_world_generated", False),
        "robot_motion_executed": precheck.get("robot_motion_executed", False),
        "real_robot_allowed": precheck.get("real_robot_allowed", False),
        "robot_prim_path": precheck.get("robot_prim_path"),
        "checked_requirements": precheck.get("checked_requirements", []),
        "missing_requirements": precheck.get("missing_requirements", []),
        "blocking_reasons": precheck.get("blocking_reasons", []),
        "warnings": precheck.get("warnings", []),
        "errors": precheck.get("errors", []),
        "arm_joint_count": precheck.get("arm_joint_count", 0),
        "observed_joint_count": precheck.get("observed_joint_count", 0),
        "expected_arm_joint_names": precheck.get("expected_arm_joint_names", []),
        "observed_arm_joint_names": precheck.get("observed_arm_joint_names", []),
        "missing_arm_joint_names": precheck.get("missing_arm_joint_names", []),
        "extra_joint_names": precheck.get("extra_joint_names", []),
        "non_arm_extra_joints": precheck.get("non_arm_extra_joints", []),
        "joint_limits_available": precheck.get("joint_limits_available", False),
        "joint_positions_within_limits": precheck.get("joint_positions_within_limits", False),
        "joint_precheck_table": precheck.get("joint_precheck_table", []),
        "safety_boundary": precheck.get(
            "safety_boundary",
            {
                "metadata_only": True,
                "simulation_only": True,
                "no_robot_motion": True,
                "no_joint_targets": True,
                "no_trajectory": True,
                "no_tcp_pose_world": True,
                "no_ros2_moveit_rtde_urscript": True,
                "no_real_robot": True,
            },
        ),
    }


def _simulation_micro_motion_info(result: Dict[str, Any]) -> Dict[str, Any]:
    motion = result.get("motion") if isinstance(result.get("motion"), dict) else {}
    precheck = result.get("precheck") if isinstance(result.get("precheck"), dict) else {}
    return {
        "requested": result.get("simulation_micro_motion_requested", False),
        "simulation_micro_motion_status": result.get(
            "simulation_micro_motion_status",
            "NOT_REQUESTED",
        ),
        "simulation_only": result.get("simulation_only", True),
        "real_robot_allowed": result.get("real_robot_allowed", False),
        "real_robot_motion_executed": result.get("real_robot_motion_executed", False),
        "robot_motion_executed": result.get("robot_motion_executed", False),
        "control_enabled": result.get("control_enabled", False),
        "simulation_control_enabled": result.get("simulation_control_enabled", False),
        "motion_generated": result.get("motion_generated", False),
        "command_generated": result.get("command_generated", False),
        "simulation_command_generated": result.get("simulation_command_generated", False),
        "joint_targets_generated": result.get("joint_targets_generated", False),
        "simulation_joint_delta_generated": result.get("simulation_joint_delta_generated", False),
        "trajectory_generated": result.get("trajectory_generated", False),
        "tcp_pose_world_generated": result.get("tcp_pose_world_generated", False),
        "precheck": precheck,
        "motion": motion,
        "before_articulation_state": result.get("before_articulation_state", {}),
        "after_articulation_state": result.get("after_articulation_state", {}),
        "warnings": result.get("simulation_micro_motion_warnings", []),
        "errors": result.get("simulation_micro_motion_errors", []),
        "blocking_reasons": result.get("simulation_micro_motion_blocking_reasons", []),
    }


def _build_summary_markdown(
    result: Dict[str, Any],
    *,
    object_info: Dict[str, Any],
    robot_asset_info: Dict[str, Any],
    robot_prim_inspection_info: Dict[str, Any],
    articulation_readiness_info: Dict[str, Any],
    articulation_state_info: Dict[str, Any],
    simulation_motion_precheck_info: Dict[str, Any],
    simulation_micro_motion_info: Dict[str, Any],
    robot_structure_report_path: str | None,
    run_id: str,
    created_at: str | None,
    report_path: str | None,
) -> str:
    error = result.get("error") if isinstance(result.get("error"), dict) else {}
    joint_summary = robot_prim_inspection_info.get("joint_metadata_summary") or {}
    return "\n".join(
        [
            "# TETO Simulation Evidence Summary",
            "",
            f"- TETO version: {_format_value(result.get('teto_version'))}",
            f"- run_id: {_format_value(run_id)}",
            f"- timestamp: {_format_value(created_at)}",
            f"- mode: {_format_value(result.get('mode'))}",
            f"- status: {_format_value(result.get('status'))}",
            f"- error.code: {_format_value(error.get('code'))}",
            f"- world_reset: {_format_value(result.get('world_reset'))}",
            f"- steps: {_format_value(result.get('steps_completed'))}/{_format_value(result.get('steps_requested'))}",
            f"- allow_robot_motion: {_format_value(result.get('allow_robot_motion'))}",
            f"- object_type: {_format_value(object_info.get('object_type'))}",
            f"- object prim path: {_format_value(object_info.get('prim_path'))}",
            f"- initial position: {_format_value(object_info.get('initial_position'))}",
            f"- target position: {_format_value(object_info.get('target_position'))}",
            f"- final position: {_format_value(object_info.get('final_position'))}",
            f"- displacement: {_format_value(object_info.get('displacement'))}",
            f"- report path: {_format_value(report_path)}",
            "",
            "## Robot Asset",
            "",
            f"- check requested: {_format_value(robot_asset_info.get('check_requested'))}",
            f"- load requested: {_format_value(robot_asset_info.get('load_requested'))}",
            f"- robot_type: {_format_value(robot_asset_info.get('robot_type'))}",
            f"- robot prim path: {_format_value(robot_asset_info.get('robot_prim_path'))}",
            f"- robot asset path: {_format_value(robot_asset_info.get('robot_asset_path'))}",
            f"- robot asset source: {_format_value(robot_asset_info.get('robot_asset_source'))}",
            f"- robot asset available: {_format_value(robot_asset_info.get('robot_asset_available'))}",
            f"- robot asset loaded: {_format_value(robot_asset_info.get('robot_asset_loaded'))}",
            f"- robot prim exists: {_format_value(robot_asset_info.get('robot_prim_exists'))}",
            f"- robot asset status: {_format_value(robot_asset_info.get('robot_asset_status'))}",
            f"- robot asset blocking reason: {_format_value(robot_asset_info.get('robot_asset_blocking_reason'))}",
            "",
            "## Robot Prim Inspection",
            "",
            f"- requested: {_format_value(robot_prim_inspection_info.get('requested'))}",
            f"- robot prim path: {_format_value(robot_prim_inspection_info.get('robot_prim_path'))}",
            f"- robot prim exists: {_format_value(robot_prim_inspection_info.get('robot_prim_exists'))}",
            f"- robot root type name: {_format_value(robot_prim_inspection_info.get('robot_root_type_name'))}",
            f"- total descendant prim count: {_format_value(robot_prim_inspection_info.get('total_descendant_prim_count'))}",
            f"- link-like prim count: {_format_value(robot_prim_inspection_info.get('link_like_prim_count'))}",
            f"- joint-like prim count: {_format_value(robot_prim_inspection_info.get('joint_like_prim_count'))}",
            f"- visual-like prim count: {_format_value(robot_prim_inspection_info.get('visual_like_prim_count'))}",
            f"- collision-like prim count: {_format_value(robot_prim_inspection_info.get('collision_like_prim_count'))}",
            f"- articulation root found: {_format_value(robot_prim_inspection_info.get('articulation_root_found'))}",
            f"- possible DOF count: {_format_value(robot_prim_inspection_info.get('possible_dof_count'))}",
            f"- Robot structure report: {_format_value(robot_structure_report_path)}",
            f"- inspection status: {_format_value(robot_prim_inspection_info.get('inspection_status'))}",
            f"- inspection warnings: {_format_value(robot_prim_inspection_info.get('inspection_warnings'))}",
            "",
            "### Joint Metadata Classification",
            "",
            "| Category | Count | Names |",
            "| --- | ---: | --- |",
            _joint_category_row("Arm joints", joint_summary.get("arm_joint_count"), joint_summary.get("arm_joint_names")),
            _joint_category_row(
                "Structural joints",
                joint_summary.get("structural_joint_count"),
                joint_summary.get("structural_joint_names"),
            ),
            _joint_category_row(
                "Gripper/tool joints",
                joint_summary.get("gripper_or_tool_joint_count"),
                joint_summary.get("gripper_or_tool_joint_names"),
            ),
            _joint_category_row(
                "Unknown joints",
                joint_summary.get("unknown_joint_count"),
                joint_summary.get("unknown_joint_names"),
            ),
            "",
            "These entries are read-only USD metadata records. They are not joint targets, not joint commands, and do not indicate robot control capability.",
            "",
            "## Articulation Readiness",
            "",
            f"- requested: {_format_value(articulation_readiness_info.get('requested'))}",
            f"- readiness_status: {_format_value(articulation_readiness_info.get('readiness_status'))}",
            f"- articulation_ready: {_format_value(articulation_readiness_info.get('articulation_ready'))}",
            f"- control_enabled: {_format_value(articulation_readiness_info.get('control_enabled'))}",
            f"- motion_generated: {_format_value(articulation_readiness_info.get('motion_generated'))}",
            f"- command_generated: {_format_value(articulation_readiness_info.get('command_generated'))}",
            f"- missing_requirements: {_format_value(articulation_readiness_info.get('missing_requirements'))}",
            f"- warnings: {_format_value(articulation_readiness_info.get('warnings'))}",
            "",
            "## Articulation State Observation",
            "",
            f"- requested: {_format_value(articulation_state_info.get('requested'))}",
            f"- status: {_format_value(articulation_state_info.get('status'))}",
            f"- articulation_state_observable: {_format_value(articulation_state_info.get('articulation_state_observable'))}",
            f"- metadata_only: {_format_value(articulation_state_info.get('metadata_only'))}",
            f"- control_enabled: {_format_value(articulation_state_info.get('control_enabled'))}",
            f"- motion_generated: {_format_value(articulation_state_info.get('motion_generated'))}",
            f"- command_generated: {_format_value(articulation_state_info.get('command_generated'))}",
            f"- joint_targets_generated: {_format_value(articulation_state_info.get('joint_targets_generated'))}",
            f"- observed_joint_count: {_format_value(articulation_state_info.get('observed_joint_count'))}",
            f"- arm_joint_count: {_format_value(articulation_state_info.get('arm_joint_count'))}",
            f"- observed_arm_joint_names: {_format_value(articulation_state_info.get('observed_arm_joint_names'))}",
            f"- missing_arm_joint_names: {_format_value(articulation_state_info.get('missing_arm_joint_names'))}",
            f"- extra_joint_names: {_format_value(articulation_state_info.get('extra_joint_names'))}",
            f"- joint_limits_available: {_format_value(articulation_state_info.get('joint_limits_available'))}",
            f"- warnings: {_format_value(articulation_state_info.get('warnings'))}",
            f"- errors: {_format_value(articulation_state_info.get('errors'))}",
            "",
            "## Simulation Motion Precheck Summary",
            "",
            f"- requested: {_format_value(simulation_motion_precheck_info.get('requested'))}",
            f"- simulation_motion_precheck_status: {_format_value(simulation_motion_precheck_info.get('status'))}",
            f"- ready_for_simulation_motion: {_format_value(simulation_motion_precheck_info.get('ready'))}",
            f"- metadata_only: {_format_value(simulation_motion_precheck_info.get('metadata_only'))}",
            f"- simulation_only: {_format_value(simulation_motion_precheck_info.get('simulation_only'))}",
            f"- control_enabled: {_format_value(simulation_motion_precheck_info.get('control_enabled'))}",
            f"- motion_generated: {_format_value(simulation_motion_precheck_info.get('motion_generated'))}",
            f"- command_generated: {_format_value(simulation_motion_precheck_info.get('command_generated'))}",
            f"- joint_targets_generated: {_format_value(simulation_motion_precheck_info.get('joint_targets_generated'))}",
            f"- trajectory_generated: {_format_value(simulation_motion_precheck_info.get('trajectory_generated'))}",
            f"- tcp_pose_world_generated: {_format_value(simulation_motion_precheck_info.get('tcp_pose_world_generated'))}",
            f"- robot_motion_executed: {_format_value(simulation_motion_precheck_info.get('robot_motion_executed'))}",
            f"- blocking_reasons: {_format_value(simulation_motion_precheck_info.get('blocking_reasons'))}",
            f"- warnings: {_format_value(simulation_motion_precheck_info.get('warnings'))}",
            f"- errors: {_format_value(simulation_motion_precheck_info.get('errors'))}",
            "",
            "## Simulation Micro-Motion Summary",
            "",
            f"- requested: {_format_value(simulation_micro_motion_info.get('requested'))}",
            f"- simulation_micro_motion_status: {_format_value(simulation_micro_motion_info.get('simulation_micro_motion_status'))}",
            f"- simulation_only: {_format_value(simulation_micro_motion_info.get('simulation_only'))}",
            f"- real_robot_allowed: {_format_value(simulation_micro_motion_info.get('real_robot_allowed'))}",
            f"- real_robot_motion_executed: {_format_value(simulation_micro_motion_info.get('real_robot_motion_executed'))}",
            f"- robot_motion_executed: {_format_value(simulation_micro_motion_info.get('robot_motion_executed'))}",
            f"- joint_name: {_format_value((simulation_micro_motion_info.get('motion') or {}).get('joint_name'))}",
            f"- requested_delta_rad: {_format_value((simulation_micro_motion_info.get('motion') or {}).get('requested_delta_rad'))}",
            f"- actual_delta_rad: {_format_value((simulation_micro_motion_info.get('motion') or {}).get('actual_delta_rad'))}",
            f"- delta_within_tolerance: {_format_value((simulation_micro_motion_info.get('motion') or {}).get('delta_within_tolerance'))}",
            f"- before_joint_state_path: {_format_value((simulation_micro_motion_info.get('motion') or {}).get('before_joint_state_path'))}",
            f"- after_joint_state_path: {_format_value((simulation_micro_motion_info.get('motion') or {}).get('after_joint_state_path'))}",
            f"- simulation_motion_result_path: {_format_value((simulation_micro_motion_info.get('motion') or {}).get('simulation_motion_result_path'))}",
            f"- simulation_motion_report_path: {_format_value((simulation_micro_motion_info.get('motion') or {}).get('simulation_motion_report_path'))}",
            "",
        ]
    )


def _build_robot_structure_report_markdown(
    result: Dict[str, Any],
    *,
    robot_asset_info: Dict[str, Any],
    robot_prim_inspection_info: Dict[str, Any],
    articulation_readiness_info: Dict[str, Any],
    articulation_state_info: Dict[str, Any],
    simulation_motion_precheck_info: Dict[str, Any],
    run_id: str,
    report_path: str | None,
) -> str:
    error = result.get("error") if isinstance(result.get("error"), dict) else {}
    joint_summary = robot_prim_inspection_info.get("joint_metadata_summary") or {}
    joint_table = robot_prim_inspection_info.get("joint_metadata_table") or []
    return "\n".join(
        [
            "# TETO UR5e Structure Report",
            "",
            "## Basic Information",
            "",
            f"- TETO version: {_format_value(result.get('teto_version'))}",
            f"- run_id: {_format_value(run_id)}",
            f"- mode: {_format_value(result.get('mode'))}",
            f"- status: {_format_value(result.get('status'))}",
            f"- robot_asset_path: {_format_value(robot_asset_info.get('robot_asset_path'))}",
            f"- robot_prim_path: {_format_value(robot_asset_info.get('robot_prim_path') or robot_prim_inspection_info.get('robot_prim_path'))}",
            f"- robot_asset_loaded: {_format_value(robot_asset_info.get('robot_asset_loaded'))}",
            f"- robot_prim_exists: {_format_value(robot_prim_inspection_info.get('robot_prim_exists'))}",
            f"- inspection_status: {_format_value(robot_prim_inspection_info.get('inspection_status'))}",
            f"- report_path: {_format_value(report_path)}",
            "",
            "## Asset Load Summary",
            "",
            f"- robot_asset_available: {_format_value(robot_asset_info.get('robot_asset_available'))}",
            f"- robot_asset_loaded: {_format_value(robot_asset_info.get('robot_asset_loaded'))}",
            f"- robot_prim_exists: {_format_value(robot_asset_info.get('robot_prim_exists'))}",
            f"- blocking_reason: {_format_value(robot_asset_info.get('robot_asset_blocking_reason'))}",
            f"- error.code: {_format_value(error.get('code'))}",
            "",
            "## Prim Structure Summary",
            "",
            f"- robot_root_type_name: {_format_value(robot_prim_inspection_info.get('robot_root_type_name'))}",
            f"- total_descendant_prim_count: {_format_value(robot_prim_inspection_info.get('total_descendant_prim_count'))}",
            f"- link_like_prim_count: {_format_value(robot_prim_inspection_info.get('link_like_prim_count'))}",
            f"- joint_like_prim_count: {_format_value(robot_prim_inspection_info.get('joint_like_prim_count'))}",
            f"- visual_like_prim_count: {_format_value(robot_prim_inspection_info.get('visual_like_prim_count'))}",
            f"- collision_like_prim_count: {_format_value(robot_prim_inspection_info.get('collision_like_prim_count'))}",
            f"- articulation_root_found: {_format_value(robot_prim_inspection_info.get('articulation_root_found'))}",
            f"- inspection_warnings: {_format_value(robot_prim_inspection_info.get('inspection_warnings'))}",
            "",
            "## Joint Metadata Classification",
            "",
            "| Category | Count | Names |",
            "| --- | ---: | --- |",
            _joint_category_row("Arm joints", joint_summary.get("arm_joint_count"), joint_summary.get("arm_joint_names")),
            _joint_category_row(
                "Structural joints",
                joint_summary.get("structural_joint_count"),
                joint_summary.get("structural_joint_names"),
            ),
            _joint_category_row(
                "Gripper/tool joints",
                joint_summary.get("gripper_or_tool_joint_count"),
                joint_summary.get("gripper_or_tool_joint_names"),
            ),
            _joint_category_row(
                "Unknown joints",
                joint_summary.get("unknown_joint_count"),
                joint_summary.get("unknown_joint_names"),
            ),
            "",
            "## Joint Metadata Table",
            "",
            "| Joint name | Category | Prim path | Type | UR5e arm joint | metadata_only | control_target_generated |",
            "| --- | --- | --- | --- | --- | --- | --- |",
            *[_joint_metadata_table_row(row) for row in joint_table],
            "",
            "## Articulation Readiness",
            "",
            f"- readiness_status: {_format_value(articulation_readiness_info.get('readiness_status'))}",
            f"- articulation_ready: {_format_value(articulation_readiness_info.get('articulation_ready'))}",
            f"- control_enabled: {_format_value(articulation_readiness_info.get('control_enabled'))}",
            f"- motion_generated: {_format_value(articulation_readiness_info.get('motion_generated'))}",
            f"- command_generated: {_format_value(articulation_readiness_info.get('command_generated'))}",
            f"- missing_requirements: {_format_value(articulation_readiness_info.get('missing_requirements'))}",
            f"- warnings: {_format_value(articulation_readiness_info.get('warnings'))}",
            "",
            "## Articulation State Observation",
            "",
            f"- status: {_format_value(articulation_state_info.get('status'))}",
            f"- articulation_state_observable: {_format_value(articulation_state_info.get('articulation_state_observable'))}",
            f"- metadata_only: {_format_value(articulation_state_info.get('metadata_only'))}",
            f"- control_enabled: {_format_value(articulation_state_info.get('control_enabled'))}",
            f"- motion_generated: {_format_value(articulation_state_info.get('motion_generated'))}",
            f"- command_generated: {_format_value(articulation_state_info.get('command_generated'))}",
            f"- joint_targets_generated: {_format_value(articulation_state_info.get('joint_targets_generated'))}",
            f"- observed_arm_joint_names: {_format_value(articulation_state_info.get('observed_arm_joint_names'))}",
            f"- missing_arm_joint_names: {_format_value(articulation_state_info.get('missing_arm_joint_names'))}",
            f"- extra_joint_names: {_format_value(articulation_state_info.get('extra_joint_names'))}",
            f"- joint_limits_available: {_format_value(articulation_state_info.get('joint_limits_available'))}",
            f"- warnings: {_format_value(articulation_state_info.get('warnings'))}",
            f"- errors: {_format_value(articulation_state_info.get('errors'))}",
            "",
            "## Simulation Motion Precheck",
            "",
            f"- status: {_format_value(simulation_motion_precheck_info.get('status'))}",
            f"- ready_for_simulation_motion: {_format_value(simulation_motion_precheck_info.get('ready'))}",
            f"- simulation_only: {_format_value(simulation_motion_precheck_info.get('simulation_only'))}",
            f"- control_enabled: {_format_value(simulation_motion_precheck_info.get('control_enabled'))}",
            f"- motion_generated: {_format_value(simulation_motion_precheck_info.get('motion_generated'))}",
            f"- command_generated: {_format_value(simulation_motion_precheck_info.get('command_generated'))}",
            f"- joint_targets_generated: {_format_value(simulation_motion_precheck_info.get('joint_targets_generated'))}",
            f"- trajectory_generated: {_format_value(simulation_motion_precheck_info.get('trajectory_generated'))}",
            f"- tcp_pose_world_generated: {_format_value(simulation_motion_precheck_info.get('tcp_pose_world_generated'))}",
            f"- robot_motion_executed: {_format_value(simulation_motion_precheck_info.get('robot_motion_executed'))}",
            f"- blocking_reasons: {_format_value(simulation_motion_precheck_info.get('blocking_reasons'))}",
            f"- warnings: {_format_value(simulation_motion_precheck_info.get('warnings'))}",
            f"- errors: {_format_value(simulation_motion_precheck_info.get('errors'))}",
            "",
            "## Safety Boundary",
            "",
            "This report is generated from read-only USD metadata inspection. It does not contain robot commands, joint targets, joint angles, tcp_pose_world, URScript, MoveIt requests, RTDE commands, or real robot control.",
            "",
            "## Presentation Summary",
            "",
            "- UR5e official asset can be locally loaded into Isaac Sim.",
            "- TETO can read and summarize the robot prim hierarchy.",
            "- TETO can classify joint-like metadata into arm / structural / gripper-tool / unknown categories.",
            "- The current stage is robot asset understanding and evidence export, not robot control.",
            "- This prepares the next stages such as articulation readiness checks or simulation-side planning integration.",
            "",
        ]
    )


def _build_demo_command_text(result: Dict[str, Any], *, demo_command: str | None) -> str:
    if demo_command:
        command = demo_command
    else:
        command = "Command not captured; reconstructed execution settings are listed below."

    lines = [
        command,
        "",
        f"mode={_format_value(result.get('mode'))}",
        f"steps_requested={_format_value(result.get('steps_requested'))}",
        f"move_object={_format_value(result.get('simulation_object_move_requested'))}",
        f"object_type={_format_value(result.get('simulation_object_type') or result.get('object_type'))}",
        f"check_robot_asset={_format_value(result.get('robot_asset_check_requested'))}",
        f"load_robot_asset={_format_value(result.get('robot_asset_load_requested'))}",
        f"robot_type={_format_value(result.get('robot_type'))}",
        f"robot_asset_path={_format_value(result.get('robot_asset_path'))}",
        f"inspect_robot_prim={_format_value(result.get('robot_prim_inspection_requested'))}",
        f"check_articulation_readiness={_format_value(result.get('articulation_readiness_requested'))}",
        f"observe_articulation_state={_format_value(result.get('articulation_state_observation_requested'))}",
        f"check_simulation_motion_precheck={_format_value(result.get('simulation_motion_precheck_requested'))}",
        f"execute_simulation_micro_motion={_format_value(result.get('simulation_micro_motion_requested'))}",
        f"micro_motion_joint={_format_value((result.get('motion') or {}).get('joint_name'))}",
        f"micro_motion_delta_rad={_format_value((result.get('motion') or {}).get('requested_delta_rad'))}",
    ]
    return "\n".join(lines) + "\n"


def _build_articulation_state_report_markdown(
    state: Dict[str, Any],
    *,
    run_id: str,
    report_path: str | None,
) -> str:
    rows = state.get("joint_state_table") if isinstance(state.get("joint_state_table"), list) else []
    return "\n".join(
        [
            "# TETO Articulation State Observation Report",
            "",
            "This report is metadata/state observation only. It does not contain robot commands, joint targets, trajectories, tcp_pose_world, URScript, MoveIt requests, RTDE commands, ROS2 messages, or real robot control.",
            "",
            "## Basic Information",
            "",
            f"- run_id: {_format_value(run_id)}",
            f"- report_path: {_format_value(report_path)}",
            f"- status: {_format_value(state.get('status'))}",
            f"- articulation_state_observable: {_format_value(state.get('articulation_state_observable'))}",
            f"- metadata_only: {_format_value(state.get('metadata_only'))}",
            f"- control_enabled: {_format_value(state.get('control_enabled'))}",
            f"- motion_generated: {_format_value(state.get('motion_generated'))}",
            f"- command_generated: {_format_value(state.get('command_generated'))}",
            f"- joint_targets_generated: {_format_value(state.get('joint_targets_generated'))}",
            "",
            "## Joint Summary",
            "",
            f"- expected_arm_joint_names: {_format_value(state.get('expected_arm_joint_names'))}",
            f"- observed_arm_joint_names: {_format_value(state.get('observed_arm_joint_names'))}",
            f"- missing_arm_joint_names: {_format_value(state.get('missing_arm_joint_names'))}",
            f"- extra_joint_names: {_format_value(state.get('extra_joint_names'))}",
            f"- joint_positions_available: {_format_value(state.get('joint_positions_available'))}",
            f"- joint_velocities_available: {_format_value(state.get('joint_velocities_available'))}",
            f"- joint_limits_available: {_format_value(state.get('joint_limits_available'))}",
            f"- warnings: {_format_value(state.get('warnings'))}",
            f"- errors: {_format_value(state.get('errors'))}",
            "",
            "## Joint State Table",
            "",
            "| Joint name | Category | Position | Velocity | Lower limit | Upper limit | Limit available | Within limit | metadata_only | control_target_generated |",
            "| --- | --- | ---: | ---: | ---: | ---: | --- | --- | --- | --- |",
            *[_joint_state_table_row(row) for row in rows],
            "",
            "## Safety Boundary",
            "",
            f"- read_only: {_format_value((state.get('safety_boundary') or {}).get('read_only'))}",
            f"- no_robot_motion: {_format_value((state.get('safety_boundary') or {}).get('no_robot_motion'))}",
            f"- no_joint_targets: {_format_value((state.get('safety_boundary') or {}).get('no_joint_targets'))}",
            f"- no_tcp_pose_world: {_format_value((state.get('safety_boundary') or {}).get('no_tcp_pose_world'))}",
            f"- no_trajectory: {_format_value((state.get('safety_boundary') or {}).get('no_trajectory'))}",
            f"- no_ros2_moveit_rtde_urscript: {_format_value((state.get('safety_boundary') or {}).get('no_ros2_moveit_rtde_urscript'))}",
            "",
        ]
    )


def _build_simulation_motion_precheck_report_markdown(
    precheck: Dict[str, Any],
    *,
    run_id: str,
    report_path: str | None,
) -> str:
    rows = precheck.get("joint_precheck_table") if isinstance(precheck.get("joint_precheck_table"), list) else []
    return "\n".join(
        [
            "# TETO Simulation Motion Precheck Report",
            "",
            "This report is simulation-only precheck evidence. It is metadata/state/readiness only. This version does not move the robot and does not contain robot commands, joint targets, trajectories, tcp_pose_world, URScript, MoveIt requests, RTDE commands, ROS2 messages, or real robot control.",
            "",
            "## Basic Information",
            "",
            f"- run_id: {_format_value(run_id)}",
            f"- report_path: {_format_value(report_path)}",
            f"- simulation_motion_precheck_status: {_format_value(precheck.get('status'))}",
            f"- ready_for_simulation_motion: {_format_value(precheck.get('ready'))}",
            f"- metadata_only: {_format_value(precheck.get('metadata_only'))}",
            f"- simulation_only: {_format_value(precheck.get('simulation_only'))}",
            f"- control_enabled: {_format_value(precheck.get('control_enabled'))}",
            f"- motion_generated: {_format_value(precheck.get('motion_generated'))}",
            f"- command_generated: {_format_value(precheck.get('command_generated'))}",
            f"- joint_targets_generated: {_format_value(precheck.get('joint_targets_generated'))}",
            f"- trajectory_generated: {_format_value(precheck.get('trajectory_generated'))}",
            f"- tcp_pose_world_generated: {_format_value(precheck.get('tcp_pose_world_generated'))}",
            f"- robot_motion_executed: {_format_value(precheck.get('robot_motion_executed'))}",
            f"- real_robot_allowed: {_format_value(precheck.get('real_robot_allowed'))}",
            "",
            "## Requirement Summary",
            "",
            f"- checked_requirements: {_format_value(precheck.get('checked_requirements'))}",
            f"- missing_requirements: {_format_value(precheck.get('missing_requirements'))}",
            f"- blocking_reasons: {_format_value(precheck.get('blocking_reasons'))}",
            f"- warnings: {_format_value(precheck.get('warnings'))}",
            f"- errors: {_format_value(precheck.get('errors'))}",
            "",
            "## Joint Summary",
            "",
            f"- expected_arm_joint_names: {_format_value(precheck.get('expected_arm_joint_names'))}",
            f"- observed_arm_joint_names: {_format_value(precheck.get('observed_arm_joint_names'))}",
            f"- missing_arm_joint_names: {_format_value(precheck.get('missing_arm_joint_names'))}",
            f"- extra_joint_names: {_format_value(precheck.get('extra_joint_names'))}",
            f"- non_arm_extra_joints: {_format_value(precheck.get('non_arm_extra_joints'))}",
            f"- joint_limits_available: {_format_value(precheck.get('joint_limits_available'))}",
            f"- joint_positions_within_limits: {_format_value(precheck.get('joint_positions_within_limits'))}",
            "",
            "## Joint Limit Check Table",
            "",
            "| Joint name | Category | Position | Velocity | Lower limit | Upper limit | Limit available | Within limit | Precheck passed | Blocking reason | metadata_only | control_target_generated |",
            "| --- | --- | ---: | ---: | ---: | ---: | --- | --- | --- | --- | --- | --- |",
            *[_joint_precheck_table_row(row) for row in rows],
            "",
            "## Safety Boundary",
            "",
            f"- metadata_only: {_format_value((precheck.get('safety_boundary') or {}).get('metadata_only'))}",
            f"- simulation_only: {_format_value((precheck.get('safety_boundary') or {}).get('simulation_only'))}",
            f"- no_robot_motion: {_format_value((precheck.get('safety_boundary') or {}).get('no_robot_motion'))}",
            f"- no_joint_targets: {_format_value((precheck.get('safety_boundary') or {}).get('no_joint_targets'))}",
            f"- no_trajectory: {_format_value((precheck.get('safety_boundary') or {}).get('no_trajectory'))}",
            f"- no_tcp_pose_world: {_format_value((precheck.get('safety_boundary') or {}).get('no_tcp_pose_world'))}",
            f"- no_ros2_moveit_rtde_urscript: {_format_value((precheck.get('safety_boundary') or {}).get('no_ros2_moveit_rtde_urscript'))}",
            f"- no_real_robot: {_format_value((precheck.get('safety_boundary') or {}).get('no_real_robot'))}",
            "",
        ]
    )


def _build_pose_delta_markdown(result: Dict[str, Any], *, object_info: Dict[str, Any]) -> str:
    return "\n".join(
        [
            "# Pose Delta",
            "",
            f"- initial_position: {_format_value(object_info.get('initial_position'))}",
            f"- target_position: {_format_value(object_info.get('target_position'))}",
            f"- final_position: {_format_value(object_info.get('final_position'))}",
            f"- displacement: {_format_value(object_info.get('displacement'))}",
            f"- moved: {_format_value(object_info.get('moved'))}",
            "",
        ]
    )


def _format_value(value: Any) -> str:
    if value is None:
        return "null"
    if isinstance(value, (list, dict)):
        return json.dumps(value, ensure_ascii=False)
    return str(value)


def _joint_category_row(label: str, count: Any, names: Any) -> str:
    normalized_names = names if isinstance(names, list) else []
    names_text = ", ".join(str(name) for name in normalized_names) if normalized_names else "-"
    return f"| {label} | {_format_value(count if count is not None else 0)} | {names_text} |"


def _joint_metadata_table_row(row: Dict[str, Any]) -> str:
    return (
        f"| {_format_value(row.get('joint_name'))} "
        f"| {_format_value(row.get('category'))} "
        f"| {_format_value(row.get('joint_prim_path'))} "
        f"| {_format_value(row.get('joint_type_name'))} "
        f"| {_format_value(row.get('is_ur5e_arm_joint'))} "
        f"| {_format_value(row.get('metadata_only'))} "
        f"| {_format_value(row.get('control_target_generated'))} |"
    )


def _joint_state_table_row(row: Dict[str, Any]) -> str:
    return (
        f"| {_format_value(row.get('joint_name'))} "
        f"| {_format_value(row.get('category'))} "
        f"| {_format_value(row.get('position'))} "
        f"| {_format_value(row.get('velocity'))} "
        f"| {_format_value(row.get('lower_limit'))} "
        f"| {_format_value(row.get('upper_limit'))} "
        f"| {_format_value(row.get('limit_available'))} "
        f"| {_format_value(row.get('within_limit'))} "
        f"| {_format_value(row.get('metadata_only'))} "
        f"| {_format_value(row.get('control_target_generated'))} |"
    )


def _joint_precheck_table_row(row: Dict[str, Any]) -> str:
    return (
        f"| {_format_value(row.get('joint_name'))} "
        f"| {_format_value(row.get('category'))} "
        f"| {_format_value(row.get('position'))} "
        f"| {_format_value(row.get('velocity'))} "
        f"| {_format_value(row.get('lower_limit'))} "
        f"| {_format_value(row.get('upper_limit'))} "
        f"| {_format_value(row.get('limit_available'))} "
        f"| {_format_value(row.get('within_limit'))} "
        f"| {_format_value(row.get('precheck_passed'))} "
        f"| {_format_value(row.get('blocking_reason'))} "
        f"| {_format_value(row.get('metadata_only'))} "
        f"| {_format_value(row.get('control_target_generated'))} |"
    )
