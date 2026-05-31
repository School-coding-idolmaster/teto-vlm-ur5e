from __future__ import annotations

import json
from pathlib import Path
from typing import Any, Dict


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

    object_info = _simulation_object_info(result)
    robot_asset_info = _robot_asset_info(result)
    robot_prim_inspection_info = _robot_prim_inspection_info(result)
    run_id = output_dir.name
    created_at = result.get("finished_at") or result.get("started_at")
    report_path = result.get("report_path")

    summary_path.write_text(
        _build_summary_markdown(
            result,
            object_info=object_info,
            robot_asset_info=robot_asset_info,
            robot_prim_inspection_info=robot_prim_inspection_info,
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
        "inspection_status": inspection.get("inspection_status", "NOT_REQUESTED"),
        "inspection_warnings": inspection.get("inspection_warnings", []),
    }


def _build_summary_markdown(
    result: Dict[str, Any],
    *,
    object_info: Dict[str, Any],
    robot_asset_info: Dict[str, Any],
    robot_prim_inspection_info: Dict[str, Any],
    run_id: str,
    created_at: str | None,
    report_path: str | None,
) -> str:
    error = result.get("error") if isinstance(result.get("error"), dict) else {}
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
            f"- inspection status: {_format_value(robot_prim_inspection_info.get('inspection_status'))}",
            f"- inspection warnings: {_format_value(robot_prim_inspection_info.get('inspection_warnings'))}",
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
    ]
    return "\n".join(lines) + "\n"


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
