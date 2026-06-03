from __future__ import annotations

import json
from pathlib import Path
from typing import Any, Dict

from src.camera_snapshot import format_camera_snapshot_report
from src.camera_source_adapter import format_camera_source_report
from src.geometry_validity import format_geometry_validity_report
from src.projector_shadow import format_projector_shadow_report
from src.simulation_micro_motion import (
    normalize_motion_evidence_paths,
    summarize_motion_evidence,
    write_simulation_micro_motion_artifacts,
)
from src.lab_readiness import format_lab_readiness_report
from src.perception_shadow_pipeline import format_perception_shadow_report
from src.planner_gateway_shadow import format_planner_gateway_shadow_report
from src.real_scene_shadow_pipeline import format_real_scene_shadow_report
from src.ros2_interface_readiness import format_ros2_interface_readiness_report
from src.ros2_message_exporter import format_ros2_message_export_report
from src.semantic_simulation_bridge import format_semantic_simulation_bridge_report
from src.simulated_task_execution import format_simulated_task_execution_report
from src.vlm_grounding_adapter import format_vlm_grounding_report


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
    semantic_bridge_result_path = output_dir / "semantic_simulation_bridge_result.json"
    semantic_bridge_report_path = output_dir / "semantic_simulation_bridge_report.md"
    semantic_task_contract_copy_path = output_dir / "semantic_task_contract_copy.json"
    simulated_task_execution_result_path = output_dir / "simulated_task_execution_result.json"
    simulated_task_execution_report_path = output_dir / "simulated_task_execution_report.md"
    execution_feedback_path = output_dir / "execution_feedback.json"
    execution_attempt_record_path = output_dir / "execution_attempt_record.json"
    failure_analysis_path = output_dir / "failure_analysis.json"
    retry_fallback_recommendation_path = output_dir / "retry_fallback_recommendation.json"
    lab_readiness_result_path = output_dir / "lab_readiness_result.json"
    lab_readiness_report_path = output_dir / "lab_readiness_report.md"
    camera_readiness_result_path = output_dir / "camera_readiness_result.json"
    live_vlm_readiness_result_path = output_dir / "live_vlm_readiness_result.json"
    shadow_mode_readiness_result_path = output_dir / "shadow_mode_readiness_result.json"
    camera_source_result_path = output_dir / "camera_source_result.json"
    camera_source_report_path = output_dir / "camera_source_report.md"
    camera_snapshot_result_path = output_dir / "camera_snapshot_result.json"
    camera_snapshot_report_path = output_dir / "camera_snapshot_report.md"
    vlm_grounding_result_path = output_dir / "vlm_grounding_result.json"
    vlm_grounding_report_path = output_dir / "vlm_grounding_report.md"
    geometry_validity_result_path = output_dir / "geometry_validity_result.json"
    geometry_validity_report_path = output_dir / "geometry_validity_report.md"
    projector_shadow_result_path = output_dir / "projector_shadow_result.json"
    projector_shadow_report_path = output_dir / "projector_shadow_report.md"
    real_scene_shadow_result_path = output_dir / "real_scene_shadow_result.json"
    real_scene_shadow_report_path = output_dir / "real_scene_shadow_report.md"
    perception_shadow_result_path = output_dir / "perception_shadow_result.json"
    perception_shadow_report_path = output_dir / "perception_shadow_report.md"
    planner_gateway_shadow_result_path = output_dir / "planner_gateway_shadow_result.json"
    planner_gateway_shadow_report_path = output_dir / "planner_gateway_shadow_report.md"
    ros2_interface_readiness_result_path = output_dir / "ros2_interface_readiness_result.json"
    ros2_interface_readiness_report_path = output_dir / "ros2_interface_readiness_report.md"
    ros2_message_export_result_path = output_dir / "ros2_message_export_result.json"
    ros2_message_export_report_path = output_dir / "ros2_message_export_report.md"

    object_info = _simulation_object_info(result)
    robot_asset_info = _robot_asset_info(result)
    robot_prim_inspection_info = _robot_prim_inspection_info(result)
    articulation_readiness_info = _articulation_readiness_info(result)
    articulation_state_info = _articulation_state_info(result)
    simulation_motion_precheck_info = _simulation_motion_precheck_info(result)
    simulation_micro_motion_info = _simulation_micro_motion_info(result)
    semantic_bridge_info = _semantic_bridge_info(result)
    simulated_task_execution_info = _simulated_task_execution_info(result)
    lab_readiness_info = _lab_readiness_info(result)
    camera_source_info = _camera_source_info(result)
    camera_snapshot_info = _camera_snapshot_info(result)
    vlm_grounding_info = _vlm_grounding_info(result)
    geometry_validity_info = _geometry_validity_info(result)
    projector_shadow_info = _projector_shadow_info(result)
    real_scene_shadow_info = _real_scene_shadow_info(result)
    perception_shadow_info = _perception_shadow_info(result)
    planner_gateway_shadow_info = _planner_gateway_shadow_info(result)
    ros2_interface_readiness_info = _ros2_interface_readiness_info(result)
    ros2_message_export_info = _ros2_message_export_info(result)
    structure_report_requested = bool(robot_prim_inspection_info.get("requested"))
    readiness_requested = bool(articulation_readiness_info.get("requested"))
    state_requested = bool(articulation_state_info.get("requested"))
    precheck_requested = bool(simulation_motion_precheck_info.get("requested"))
    micro_motion_requested = bool(simulation_micro_motion_info.get("requested"))
    semantic_bridge_requested = bool(semantic_bridge_info.get("requested"))
    simulated_task_execution_requested = bool(
        simulated_task_execution_info.get("safe_simulated_task_execution_requested")
    )
    lab_readiness_requested = bool(lab_readiness_info.get("requested"))
    camera_source_requested = bool(camera_source_info.get("requested"))
    camera_snapshot_requested = bool(camera_snapshot_info.get("requested"))
    vlm_grounding_requested = bool(vlm_grounding_info.get("requested"))
    geometry_validity_requested = bool(geometry_validity_info.get("requested"))
    projector_shadow_requested = bool(projector_shadow_info.get("requested"))
    real_scene_shadow_requested = bool(real_scene_shadow_info.get("requested"))
    perception_shadow_requested = bool(perception_shadow_info.get("requested"))
    planner_gateway_shadow_requested = bool(planner_gateway_shadow_info.get("requested"))
    ros2_interface_readiness_requested = bool(ros2_interface_readiness_info.get("requested"))
    ros2_message_export_requested = bool(ros2_message_export_info.get("requested"))
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
            semantic_bridge_info=semantic_bridge_info,
            simulated_task_execution_info=simulated_task_execution_info,
            lab_readiness_info=lab_readiness_info,
            camera_source_info=camera_source_info,
            camera_snapshot_info=camera_snapshot_info,
            vlm_grounding_info=vlm_grounding_info,
            geometry_validity_info=geometry_validity_info,
            projector_shadow_info=projector_shadow_info,
            real_scene_shadow_info=real_scene_shadow_info,
            perception_shadow_info=perception_shadow_info,
            planner_gateway_shadow_info=planner_gateway_shadow_info,
            ros2_interface_readiness_info=ros2_interface_readiness_info,
            ros2_message_export_info=ros2_message_export_info,
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
    if semantic_bridge_requested:
        contract_copy = result.get("semantic_contract_copy")
        if isinstance(contract_copy, dict):
            with semantic_task_contract_copy_path.open("w", encoding="utf-8") as contract_file:
                json.dump(contract_copy, contract_file, ensure_ascii=False, indent=2)
                contract_file.write("\n")
        semantic_bridge_info["semantic_simulation_bridge_result_path"] = str(semantic_bridge_result_path)
        semantic_bridge_info["semantic_simulation_bridge_report_path"] = str(semantic_bridge_report_path)
        semantic_bridge_info["semantic_task_contract_copy_path"] = (
            str(semantic_task_contract_copy_path) if isinstance(contract_copy, dict) else None
        )
        semantic_bridge_info["semantic_bridge_files"] = _semantic_bridge_files(semantic_bridge_info)
        with semantic_bridge_result_path.open("w", encoding="utf-8") as bridge_file:
            json.dump(semantic_bridge_info, bridge_file, ensure_ascii=False, indent=2)
            bridge_file.write("\n")
        semantic_bridge_report_path.write_text(
            format_semantic_simulation_bridge_report(
                semantic_bridge_info,
                evidence_files=semantic_bridge_info["semantic_bridge_files"],
            ),
            encoding="utf-8",
        )
    if simulated_task_execution_requested:
        simulated_task_execution_info["simulated_task_execution_result_path"] = str(
            simulated_task_execution_result_path
        )
        simulated_task_execution_info["simulated_task_execution_report_path"] = str(
            simulated_task_execution_report_path
        )
        simulated_task_execution_info["execution_feedback_path"] = str(execution_feedback_path)
        simulated_task_execution_info["execution_attempt_record_path"] = str(execution_attempt_record_path)
        simulated_task_execution_info["failure_analysis_path"] = str(failure_analysis_path)
        simulated_task_execution_info["retry_fallback_recommendation_path"] = str(
            retry_fallback_recommendation_path
        )
        simulated_task_execution_info["simulated_task_execution_files"] = _simulated_task_execution_files(
            simulated_task_execution_info
        )
        with simulated_task_execution_result_path.open("w", encoding="utf-8") as execution_file:
            json.dump(simulated_task_execution_info, execution_file, ensure_ascii=False, indent=2)
            execution_file.write("\n")
        _write_json_artifact(execution_feedback_path, simulated_task_execution_info.get("execution_feedback", {}))
        _write_json_artifact(
            execution_attempt_record_path,
            simulated_task_execution_info.get("execution_attempt_record", {}),
        )
        _write_json_artifact(failure_analysis_path, simulated_task_execution_info.get("failure_analysis", {}))
        _write_json_artifact(
            retry_fallback_recommendation_path,
            simulated_task_execution_info.get("retry_fallback_recommendation", {}),
        )
        simulated_task_execution_report_path.write_text(
            format_simulated_task_execution_report(
                _simulated_task_execution_report_context(
                    result,
                    simulated_task_execution_info,
                    simulation_motion_precheck_info,
                    simulation_micro_motion_info,
                    semantic_bridge_info,
                ),
                evidence_files=simulated_task_execution_info["simulated_task_execution_files"],
            ),
            encoding="utf-8",
        )
    if lab_readiness_requested:
        lab_readiness_info["lab_readiness_result_path"] = str(lab_readiness_result_path)
        lab_readiness_info["lab_readiness_report_path"] = str(lab_readiness_report_path)
        lab_readiness_info["camera_readiness_result_path"] = str(camera_readiness_result_path)
        lab_readiness_info["live_vlm_readiness_result_path"] = str(live_vlm_readiness_result_path)
        lab_readiness_info["shadow_mode_readiness_result_path"] = str(shadow_mode_readiness_result_path)
        lab_readiness_info["readiness_evidence_files"] = _readiness_evidence_files(lab_readiness_info)
        _write_json_artifact(lab_readiness_result_path, lab_readiness_info)
        lab_readiness_report_path.write_text(format_lab_readiness_report(lab_readiness_info), encoding="utf-8")
        _write_json_artifact(camera_readiness_result_path, lab_readiness_info.get("camera", {}))
        _write_json_artifact(live_vlm_readiness_result_path, lab_readiness_info.get("live_vlm", {}))
        _write_json_artifact(shadow_mode_readiness_result_path, lab_readiness_info.get("shadow_mode", {}))
    if camera_source_requested:
        camera_source_info["camera_source_result_path"] = str(camera_source_result_path)
        camera_source_info["camera_source_report_path"] = str(camera_source_report_path)
        camera_source_info["camera_source_evidence_files"] = _camera_source_evidence_files(camera_source_info)
        _write_json_artifact(camera_source_result_path, camera_source_info)
        camera_source_report_path.write_text(
            format_camera_source_report(camera_source_info),
            encoding="utf-8",
        )
    if camera_snapshot_requested:
        camera_snapshot_info["camera_snapshot_result_path"] = str(camera_snapshot_result_path)
        camera_snapshot_info["camera_snapshot_report_path"] = str(camera_snapshot_report_path)
        camera_snapshot_info["camera_snapshot_evidence_files"] = _camera_snapshot_evidence_files(
            camera_snapshot_info
        )
        _write_json_artifact(camera_snapshot_result_path, camera_snapshot_info)
        camera_snapshot_report_path.write_text(
            format_camera_snapshot_report(camera_snapshot_info),
            encoding="utf-8",
        )
    if vlm_grounding_requested:
        vlm_grounding_info["vlm_grounding_result_path"] = str(vlm_grounding_result_path)
        vlm_grounding_info["vlm_grounding_report_path"] = str(vlm_grounding_report_path)
        vlm_grounding_info["vlm_grounding_evidence_files"] = _vlm_grounding_evidence_files(vlm_grounding_info)
        _write_json_artifact(vlm_grounding_result_path, vlm_grounding_info)
        vlm_grounding_report_path.write_text(
            format_vlm_grounding_report(vlm_grounding_info),
            encoding="utf-8",
        )
    if geometry_validity_requested:
        geometry_validity_info["geometry_validity_result_path"] = str(geometry_validity_result_path)
        geometry_validity_info["geometry_validity_report_path"] = str(geometry_validity_report_path)
        geometry_validity_info["geometry_validity_evidence_files"] = _geometry_validity_evidence_files(
            geometry_validity_info
        )
        _write_json_artifact(geometry_validity_result_path, geometry_validity_info)
        geometry_validity_report_path.write_text(
            format_geometry_validity_report(geometry_validity_info),
            encoding="utf-8",
        )
    if projector_shadow_requested:
        projector_shadow_info["projector_shadow_result_path"] = str(projector_shadow_result_path)
        projector_shadow_info["projector_shadow_report_path"] = str(projector_shadow_report_path)
        projector_shadow_info["projector_shadow_evidence_files"] = _projector_shadow_evidence_files(
            projector_shadow_info
        )
        _write_json_artifact(projector_shadow_result_path, projector_shadow_info)
        projector_shadow_report_path.write_text(
            format_projector_shadow_report(projector_shadow_info),
            encoding="utf-8",
        )
    if real_scene_shadow_requested:
        real_scene_shadow_info["real_scene_shadow_result_path"] = str(real_scene_shadow_result_path)
        real_scene_shadow_info["real_scene_shadow_report_path"] = str(real_scene_shadow_report_path)
        real_scene_shadow_info["real_scene_shadow_evidence_files"] = _real_scene_shadow_evidence_files(
            real_scene_shadow_info
        )
        _write_json_artifact(real_scene_shadow_result_path, real_scene_shadow_info)
        real_scene_shadow_report_path.write_text(
            format_real_scene_shadow_report(real_scene_shadow_info),
            encoding="utf-8",
        )
    if perception_shadow_requested:
        perception_shadow_info["perception_shadow_result_path"] = str(perception_shadow_result_path)
        perception_shadow_info["perception_shadow_report_path"] = str(perception_shadow_report_path)
        perception_shadow_info["perception_shadow_evidence_files"] = _perception_shadow_evidence_files(
            perception_shadow_info
        )
        _write_json_artifact(perception_shadow_result_path, perception_shadow_info)
        perception_shadow_report_path.write_text(
            format_perception_shadow_report(perception_shadow_info),
            encoding="utf-8",
        )
    if planner_gateway_shadow_requested:
        planner_gateway_shadow_info["planner_gateway_shadow_result_path"] = str(
            planner_gateway_shadow_result_path
        )
        planner_gateway_shadow_info["planner_gateway_shadow_report_path"] = str(
            planner_gateway_shadow_report_path
        )
        planner_gateway_shadow_info["planner_gateway_shadow_evidence_files"] = (
            _planner_gateway_shadow_evidence_files(planner_gateway_shadow_info)
        )
        _write_json_artifact(planner_gateway_shadow_result_path, planner_gateway_shadow_info)
        planner_gateway_shadow_report_path.write_text(
            format_planner_gateway_shadow_report(planner_gateway_shadow_info),
            encoding="utf-8",
        )
    if ros2_interface_readiness_requested:
        ros2_interface_readiness_info["ros2_interface_readiness_result_path"] = str(
            ros2_interface_readiness_result_path
        )
        ros2_interface_readiness_info["ros2_interface_readiness_report_path"] = str(
            ros2_interface_readiness_report_path
        )
        ros2_interface_readiness_info["ros2_interface_readiness_evidence_files"] = (
            _ros2_interface_readiness_evidence_files(ros2_interface_readiness_info)
        )
        _write_json_artifact(ros2_interface_readiness_result_path, ros2_interface_readiness_info)
        ros2_interface_readiness_report_path.write_text(
            format_ros2_interface_readiness_report(ros2_interface_readiness_info),
            encoding="utf-8",
        )
    if ros2_message_export_requested:
        ros2_message_export_info["ros2_message_export_result_path"] = str(ros2_message_export_result_path)
        ros2_message_export_info["ros2_message_export_report_path"] = str(ros2_message_export_report_path)
        ros2_message_export_info["ros2_message_export_evidence_files"] = _ros2_message_export_evidence_files(
            ros2_message_export_info
        )
        _write_json_artifact(ros2_message_export_result_path, ros2_message_export_info)
        ros2_message_export_report_path.write_text(
            format_ros2_message_export_report(ros2_message_export_info),
            encoding="utf-8",
        )
    motion_evidence_summary = summarize_motion_evidence(simulation_micro_motion_info)
    execution_evidence_files = (
        _simulated_task_execution_files(simulated_task_execution_info)
        if simulated_task_execution_requested
        else []
    )
    latest_execution_summary = _latest_execution_summary(
        simulated_task_execution_info,
        simulation_motion_precheck_info,
        simulation_micro_motion_info,
        semantic_bridge_info,
    )
    replay_bundle_files = (
        _replay_bundle_files(
            execution_evidence_files=execution_evidence_files,
            motion_evidence_files=motion_evidence_summary["motion_evidence_files"],
            semantic_bridge_files=_semantic_bridge_files(semantic_bridge_info) if semantic_bridge_requested else [],
        )
        if simulated_task_execution_requested
        else []
    )
    safety_boundary_confirmed = _safety_boundary_confirmed(result, simulated_task_execution_info)
    no_automatic_retry_executed = (
        (simulated_task_execution_info.get("retry_fallback_recommendation") or {}).get(
            "automatic_retry_executed",
            False,
        )
        is False
    )

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
        "semantic_bridge_requested": semantic_bridge_requested,
        "semantic_bridge_status": semantic_bridge_info.get("status"),
        "semantic_bridge_evidence_available": semantic_bridge_requested,
        "semantic_bridge_files": _semantic_bridge_files(semantic_bridge_info) if semantic_bridge_requested else [],
        "semantic_task_contract_path": semantic_bridge_info.get("semantic_task_contract_path"),
        "semantic_task_id": semantic_bridge_info.get("semantic_task_id"),
        "semantic_intent": semantic_bridge_info.get("semantic_intent"),
        "semantic_target_label": semantic_bridge_info.get("semantic_target_label"),
        "semantic_gate_passed": semantic_bridge_info.get("gate_passed"),
        "semantic_bridge_blocking_reasons": semantic_bridge_info.get("blocking_reasons", []),
        "triggered_simulation_micro_motion": semantic_bridge_info.get("triggered_simulation_micro_motion", False),
        "simulated_task_execution_requested": simulated_task_execution_requested,
        "simulated_task_execution_status": simulated_task_execution_info.get("simulated_task_status"),
        "execution_attempt_id": simulated_task_execution_info.get("execution_attempt_id"),
        "execution_feedback_status": simulated_task_execution_info.get("execution_feedback_status"),
        "failure_reason": simulated_task_execution_info.get("failure_reason"),
        "retry_recommended": simulated_task_execution_info.get("retry_recommended"),
        "fallback_recommended": simulated_task_execution_info.get("fallback_recommended"),
        "fallback_type": simulated_task_execution_info.get("fallback_type"),
        "post_motion_state_check_status": (
            (simulated_task_execution_info.get("post_motion_state_check") or {}).get(
                "post_motion_state_check_status"
            )
        ),
        "simulated_task_execution_files": execution_evidence_files,
        "execution_evidence_available": simulated_task_execution_requested and bool(execution_evidence_files),
        "execution_evidence_files": execution_evidence_files,
        "replay_ready": simulated_task_execution_info.get("replay_ready", False),
        "replay_index_hint": "Use replay_bundle_files to inspect semantic, motion, and execution evidence in order.",
        "replay_bundle_files": replay_bundle_files,
        "latest_execution_summary": latest_execution_summary,
        "safety_boundary_confirmed": safety_boundary_confirmed,
        "no_automatic_retry_executed": no_automatic_retry_executed,
        "lab_readiness_requested": lab_readiness_requested,
        "lab_readiness_status": lab_readiness_info.get("status"),
        "lab_backend_readiness_status": lab_readiness_info.get("lab_backend_readiness_status"),
        "camera_readiness_status": lab_readiness_info.get("camera_readiness_status"),
        "live_vlm_readiness_status": lab_readiness_info.get("live_vlm_readiness_status"),
        "shadow_mode_readiness_status": lab_readiness_info.get("shadow_mode_readiness_status"),
        "readiness_evidence_available": lab_readiness_requested and bool(_readiness_evidence_files(lab_readiness_info)),
        "no_motion_readiness_passed": lab_readiness_info.get("no_motion_readiness_passed", False),
        "readiness_statuses": _readiness_statuses(lab_readiness_info),
        "blocking_reasons": list(lab_readiness_info.get("blocking_reasons") or result.get("blocking_reasons") or []),
        "safety_flags": _readiness_safety_flags(lab_readiness_info),
        "allow_robot_motion": lab_readiness_info.get("allow_robot_motion", result.get("allow_robot_motion", False)),
        "allow_live_camera": lab_readiness_info.get("allow_live_camera", False),
        "allow_live_vlm": lab_readiness_info.get("allow_live_vlm", False),
        "readiness_evidence_files": (
            _readiness_evidence_files(lab_readiness_info) if lab_readiness_requested else []
        ),
        "next_safe_action": lab_readiness_info.get("next_safe_action"),
        "camera_source_evidence_available": camera_source_requested,
        "camera_source_status": camera_source_info.get("camera_source_status"),
        "source_mode": camera_source_info.get("source_mode"),
        "snapshot_id": camera_source_info.get("snapshot_id"),
        "scene_version": camera_source_info.get("scene_version") or result.get("scene_version"),
        "capture_timestamp": camera_source_info.get("capture_timestamp"),
        "frame_id": camera_source_info.get("frame_id"),
        "camera_frame": camera_source_info.get("camera_frame"),
        "image_ref": camera_source_info.get("image_ref"),
        "depth_ref": camera_source_info.get("depth_ref"),
        "camera_info_ref": camera_source_info.get("camera_info_ref"),
        "metadata_ref": camera_source_info.get("metadata_ref"),
        "extrinsics_ref": camera_source_info.get("extrinsics_ref"),
        "depth_available": camera_source_info.get("depth_available"),
        "camera_info_available": camera_source_info.get("camera_info_available"),
        "one_shot_capture_used": camera_source_info.get("one_shot_capture_used", False),
        "continuous_capture_used": camera_source_info.get("continuous_capture_used", False),
        "live_camera_capture_allowed": camera_source_info.get("live_camera_capture_allowed", False),
        "live_camera_capture_used": camera_source_info.get("live_camera_capture_used", False),
        "no_motion_camera_adapter_passed": camera_source_info.get("no_motion_camera_adapter_passed", False),
        "camera_source_blocking_reasons": camera_source_info.get("blocking_reasons", []),
        "camera_source_warnings": camera_source_info.get("warnings", []),
        "camera_source_next_safe_action": camera_source_info.get("next_safe_action"),
        "camera_source_evidence_files": (
            _camera_source_evidence_files(camera_source_info) if camera_source_requested else []
        ),
        "camera_snapshot_evidence_available": camera_snapshot_requested,
        "camera_snapshot_id": camera_snapshot_info.get("snapshot_id"),
        "scene_version": camera_snapshot_info.get("scene_version") or result.get("scene_version"),
        "camera_snapshot_validity_status": camera_snapshot_info.get("validity_status"),
        "camera_snapshot_blocking_reasons": camera_snapshot_info.get("blocking_reasons", []),
        "camera_snapshot_warnings": camera_snapshot_info.get("warnings", []),
        "no_motion_snapshot_passed": camera_snapshot_info.get("no_motion_snapshot_passed", False),
        "camera_snapshot_evidence_files": (
            _camera_snapshot_evidence_files(camera_snapshot_info) if camera_snapshot_requested else []
        ),
        "live_capture_used": camera_snapshot_info.get("live_capture_used", False),
        "live_camera_enabled": camera_snapshot_info.get("live_camera_enabled", False),
        "live_camera_used": (
            camera_snapshot_info.get("live_capture_used", False)
            or camera_source_info.get("live_camera_capture_used", False)
            or lab_readiness_info.get("live_camera_used", False)
        ),
        "live_vlm_called": (
            camera_snapshot_info.get("live_vlm_called", False)
            or camera_source_info.get("live_vlm_called", False)
            or vlm_grounding_info.get("live_vlm_called", False)
            or lab_readiness_info.get("live_vlm_called", False)
        ),
        "real_robot_motion_executed": (
            camera_snapshot_info.get("real_robot_motion_executed", False)
            or camera_source_info.get("real_robot_motion_executed", False)
            or lab_readiness_info.get("real_robot_motion_executed", False)
            or result.get("real_robot_motion_executed", False)
        ),
        "real_robot_command_enabled": (
            camera_snapshot_info.get("real_robot_command_enabled", False)
            or camera_source_info.get("real_robot_command_enabled", False)
            or lab_readiness_info.get("real_robot_command_enabled", False)
        ),
        "motion_evidence_available": motion_evidence_summary["motion_evidence_available"],
        "motion_evidence_files": motion_evidence_summary["motion_evidence_files"],
        "motion_diff_summary": motion_evidence_summary["motion_diff_summary"],
        "simulation_only": result.get("simulation_only", True),
        "simulation_motion_result_path": str(simulation_motion_result_path) if micro_motion_requested else None,
        "simulation_motion_report_path": str(simulation_motion_report_path) if micro_motion_requested else None,
        "before_articulation_state_path": str(before_articulation_state_path) if micro_motion_requested else None,
        "after_articulation_state_path": str(after_articulation_state_path) if micro_motion_requested else None,
        "semantic_simulation_bridge_result_path": str(semantic_bridge_result_path)
        if semantic_bridge_requested
        else None,
        "semantic_simulation_bridge_report_path": str(semantic_bridge_report_path)
        if semantic_bridge_requested
        else None,
        "semantic_task_contract_copy_path": (
            str(semantic_task_contract_copy_path)
            if semantic_bridge_requested and semantic_task_contract_copy_path.exists()
            else None
        ),
        "simulated_task_execution_result_path": str(simulated_task_execution_result_path)
        if simulated_task_execution_requested
        else None,
        "simulated_task_execution_report_path": str(simulated_task_execution_report_path)
        if simulated_task_execution_requested
        else None,
        "execution_feedback_path": str(execution_feedback_path) if simulated_task_execution_requested else None,
        "execution_attempt_record_path": (
            str(execution_attempt_record_path) if simulated_task_execution_requested else None
        ),
        "failure_analysis_path": str(failure_analysis_path) if simulated_task_execution_requested else None,
        "retry_fallback_recommendation_path": (
            str(retry_fallback_recommendation_path) if simulated_task_execution_requested else None
        ),
        "lab_readiness_result_path": str(lab_readiness_result_path) if lab_readiness_requested else None,
        "lab_readiness_report_path": str(lab_readiness_report_path) if lab_readiness_requested else None,
        "camera_readiness_result_path": str(camera_readiness_result_path) if lab_readiness_requested else None,
        "live_vlm_readiness_result_path": str(live_vlm_readiness_result_path) if lab_readiness_requested else None,
        "shadow_mode_readiness_result_path": str(shadow_mode_readiness_result_path) if lab_readiness_requested else None,
        "camera_source_result_path": str(camera_source_result_path) if camera_source_requested else None,
        "camera_source_report_path": str(camera_source_report_path) if camera_source_requested else None,
        "camera_snapshot_result_path": str(camera_snapshot_result_path) if camera_snapshot_requested else None,
        "camera_snapshot_report_path": str(camera_snapshot_report_path) if camera_snapshot_requested else None,
        "vlm_grounding_evidence_available": vlm_grounding_requested,
        "vlm_grounding_status": vlm_grounding_info.get("vlm_grounding_status"),
        "vlm_grounding_requested": vlm_grounding_requested,
        "grounding_id": vlm_grounding_info.get("grounding_id")
        or geometry_validity_info.get("grounding_id")
        or real_scene_shadow_info.get("grounding_id"),
        "snapshot_id": vlm_grounding_info.get("snapshot_id")
        or geometry_validity_info.get("snapshot_id")
        or real_scene_shadow_info.get("snapshot_id")
        or camera_source_info.get("snapshot_id"),
        "scene_version": vlm_grounding_info.get("scene_version")
        or geometry_validity_info.get("scene_version")
        or real_scene_shadow_info.get("scene_version")
        or camera_snapshot_info.get("scene_version")
        or camera_source_info.get("scene_version")
        or result.get("scene_version"),
        "user_command": vlm_grounding_info.get("user_command"),
        "normalized_command": vlm_grounding_info.get("normalized_command"),
        "adapter_mode": vlm_grounding_info.get("adapter_mode"),
        "target_label": vlm_grounding_info.get("target_label"),
        "target_object_id": vlm_grounding_info.get("target_object_id"),
        "bbox_xyxy": vlm_grounding_info.get("bbox_xyxy"),
        "pixel_center": vlm_grounding_info.get("pixel_center"),
        "mask_ref": vlm_grounding_info.get("mask_ref"),
        "semantic_confidence": vlm_grounding_info.get("semantic_confidence"),
        "grounding_confidence": vlm_grounding_info.get("grounding_confidence"),
        "overall_confidence": vlm_grounding_info.get("overall_confidence"),
        "grounded": vlm_grounding_info.get("grounded"),
        "rejected": vlm_grounding_info.get("rejected"),
        "rejection_reason": vlm_grounding_info.get("rejection_reason"),
        "error_code": vlm_grounding_info.get("error_code"),
        "no_motion_grounding_passed": vlm_grounding_info.get("no_motion_grounding_passed", False),
        "vlm_grounding_blocking_reasons": vlm_grounding_info.get("blocking_reasons", []),
        "vlm_grounding_warnings": vlm_grounding_info.get("warnings", []),
        "vlm_grounding_next_safe_action": vlm_grounding_info.get("next_safe_action"),
        "vlm_grounding_evidence_files": (
            _vlm_grounding_evidence_files(vlm_grounding_info) if vlm_grounding_requested else []
        ),
        "vlm_grounding_result_path": str(vlm_grounding_result_path) if vlm_grounding_requested else None,
        "vlm_grounding_report_path": str(vlm_grounding_report_path) if vlm_grounding_requested else None,
        "geometry_validity_evidence_available": geometry_validity_requested,
        "geometry_validity_status": geometry_validity_info.get("geometry_validity_status"),
        "geometry_validity_requested": geometry_validity_requested,
        "snapshot_id": geometry_validity_info.get("snapshot_id") or vlm_grounding_info.get("snapshot_id") or real_scene_shadow_info.get("snapshot_id") or camera_source_info.get("snapshot_id"),
        "grounding_id": geometry_validity_info.get("grounding_id") or vlm_grounding_info.get("grounding_id") or real_scene_shadow_info.get("grounding_id"),
        "scene_version": geometry_validity_info.get("scene_version")
        or vlm_grounding_info.get("scene_version")
        or real_scene_shadow_info.get("scene_version")
        or camera_snapshot_info.get("scene_version")
        or camera_source_info.get("scene_version")
        or result.get("scene_version"),
        "bbox_valid": geometry_validity_info.get("bbox_valid"),
        "pixel_center_valid": geometry_validity_info.get("pixel_center_valid"),
        "bbox_inside_image": geometry_validity_info.get("bbox_inside_image"),
        "pixel_center_inside_image": geometry_validity_info.get("pixel_center_inside_image"),
        "confidence_check_passed": geometry_validity_info.get("confidence_check_passed"),
        "ttl_check_passed": geometry_validity_info.get("ttl_check_passed"),
        "depth_required": geometry_validity_info.get("depth_required"),
        "depth_available": geometry_validity_info.get("depth_available")
        if geometry_validity_requested
        else camera_source_info.get("depth_available"),
        "camera_frame_available": geometry_validity_info.get("camera_frame_available"),
        "geometry_validity_blocking_reasons": geometry_validity_info.get("blocking_reasons", []),
        "geometry_validity_warnings": geometry_validity_info.get("warnings", []),
        "geometry_validity_next_safe_action": geometry_validity_info.get("next_safe_action"),
        "no_motion_geometry_passed": geometry_validity_info.get("no_motion_geometry_passed", False),
        "geometry_validity_evidence_files": (
            _geometry_validity_evidence_files(geometry_validity_info) if geometry_validity_requested else []
        ),
        "geometry_validity_result_path": str(geometry_validity_result_path)
        if geometry_validity_requested
        else None,
        "geometry_validity_report_path": str(geometry_validity_report_path)
        if geometry_validity_requested
        else None,
        "projector_shadow_evidence_available": projector_shadow_requested,
        "projector_requested": projector_shadow_info.get("projector_requested", False),
        "projector_status": projector_shadow_info.get("projector_status"),
        "snapshot_id": projector_shadow_info.get("snapshot_id")
        or geometry_validity_info.get("snapshot_id")
        or vlm_grounding_info.get("snapshot_id")
        or real_scene_shadow_info.get("snapshot_id")
        or camera_source_info.get("snapshot_id"),
        "grounding_id": projector_shadow_info.get("grounding_id")
        or geometry_validity_info.get("grounding_id")
        or vlm_grounding_info.get("grounding_id")
        or real_scene_shadow_info.get("grounding_id"),
        "scene_version": projector_shadow_info.get("scene_version")
        or geometry_validity_info.get("scene_version")
        or vlm_grounding_info.get("scene_version")
        or real_scene_shadow_info.get("scene_version")
        or camera_snapshot_info.get("scene_version")
        or camera_source_info.get("scene_version")
        or result.get("scene_version"),
        "pixel_center": projector_shadow_info.get("pixel_center") or vlm_grounding_info.get("pixel_center"),
        "depth_value_m": projector_shadow_info.get("depth_value_m"),
        "depth_valid": projector_shadow_info.get("depth_valid"),
        "camera_intrinsics_available": projector_shadow_info.get("camera_intrinsics_available"),
        "camera_frame": projector_shadow_info.get("camera_frame") or camera_source_info.get("camera_frame"),
        "world_frame": projector_shadow_info.get("world_frame"),
        "camera_point_m": projector_shadow_info.get("camera_point_m"),
        "world_point_m": projector_shadow_info.get("world_point_m"),
        "projection_confidence": projector_shadow_info.get("projection_confidence"),
        "projection_method": projector_shadow_info.get("projection_method"),
        "tf_available": projector_shadow_info.get("tf_available"),
        "tf_source": projector_shadow_info.get("tf_source"),
        "real_tf_used": projector_shadow_info.get("real_tf_used", False),
        "ros2_tf_used": projector_shadow_info.get("ros2_tf_used", False),
        "workspace_check_passed": projector_shadow_info.get("workspace_check_passed"),
        "projector_blocking_reasons": projector_shadow_info.get("blocking_reasons", []),
        "projector_warnings": projector_shadow_info.get("warnings", []),
        "projector_next_safe_action": projector_shadow_info.get("next_safe_action"),
        "no_motion_projector_passed": projector_shadow_info.get("no_motion_projector_passed", False),
        "projector_shadow_evidence_files": (
            _projector_shadow_evidence_files(projector_shadow_info) if projector_shadow_requested else []
        ),
        "projector_shadow_result_path": str(projector_shadow_result_path)
        if projector_shadow_requested
        else None,
        "projector_shadow_report_path": str(projector_shadow_report_path)
        if projector_shadow_requested
        else None,
        "real_scene_shadow_evidence_available": real_scene_shadow_requested,
        "shadow_pipeline_status": real_scene_shadow_info.get("shadow_pipeline_status"),
        "semantic_gate_passed": (
            real_scene_shadow_info.get("semantic_gate_passed", False)
            if real_scene_shadow_requested
            else semantic_bridge_info.get("gate_passed")
        ),
        "real_scene_shadow_semantic_gate_passed": real_scene_shadow_info.get(
            "semantic_gate_passed",
            False,
        ),
        "no_motion_shadow_passed": real_scene_shadow_info.get("no_motion_shadow_passed", False),
        "real_scene_shadow_blocking_reasons": real_scene_shadow_info.get("blocking_reasons", []),
        "real_scene_shadow_warnings": real_scene_shadow_info.get("warnings", []),
        "real_scene_shadow_next_safe_action": real_scene_shadow_info.get("next_safe_action"),
        "real_scene_shadow_replay_ready": real_scene_shadow_info.get("replay_ready", False),
        "blocking_reasons": (
            projector_shadow_info.get("blocking_reasons")
            if projector_shadow_requested
            else geometry_validity_info.get("blocking_reasons")
            if geometry_validity_requested
            else vlm_grounding_info.get("blocking_reasons")
            if vlm_grounding_requested
            else real_scene_shadow_info.get("blocking_reasons")
            if real_scene_shadow_requested
            else camera_source_info.get("blocking_reasons")
            if camera_source_requested
            else ros2_interface_readiness_info.get("blocking_reasons")
            if ros2_interface_readiness_requested
            else list(lab_readiness_info.get("blocking_reasons") or result.get("blocking_reasons") or [])
        ),
        "warnings": (
            projector_shadow_info.get("warnings", [])
            if projector_shadow_requested
            else geometry_validity_info.get("warnings", [])
            if geometry_validity_requested
            else vlm_grounding_info.get("warnings", [])
            if vlm_grounding_requested
            else real_scene_shadow_info.get("warnings", [])
            if real_scene_shadow_requested
            else camera_source_info.get("warnings", [])
            if camera_source_requested
            else ros2_interface_readiness_info.get("warnings", [])
            if ros2_interface_readiness_requested
            else []
        ),
        "next_safe_action": (
            projector_shadow_info.get("next_safe_action")
            if projector_shadow_requested
            else geometry_validity_info.get("next_safe_action")
            if geometry_validity_requested
            else vlm_grounding_info.get("next_safe_action")
            if vlm_grounding_requested
            else real_scene_shadow_info.get("next_safe_action")
            if real_scene_shadow_requested
            else camera_source_info.get("next_safe_action")
            if camera_source_requested
            else ros2_interface_readiness_info.get("next_safe_action")
            if ros2_interface_readiness_requested
            else lab_readiness_info.get("next_safe_action")
        ),
        "replay_ready": real_scene_shadow_info.get(
            "replay_ready",
            False,
        )
        if real_scene_shadow_requested
        else simulated_task_execution_info.get(
            "replay_ready",
            False,
        ),
        "robot_command_generated": (
            projector_shadow_info.get("robot_command_generated", False)
            or vlm_grounding_info.get("robot_command_generated", False)
            or camera_source_info.get("robot_command_generated", False)
            or real_scene_shadow_info.get("robot_command_generated", False)
        ),
        "trajectory_generated": (
            projector_shadow_info.get("trajectory_generated", False)
            or vlm_grounding_info.get("trajectory_generated", False)
            or camera_source_info.get("trajectory_generated", False)
            or real_scene_shadow_info.get("trajectory_generated", False)
        ),
        "joint_targets_generated": (
            projector_shadow_info.get("joint_targets_generated", False)
            or vlm_grounding_info.get("joint_targets_generated", False)
            or camera_source_info.get("joint_targets_generated", False)
            or real_scene_shadow_info.get("joint_targets_generated", False)
        ),
        "tcp_pose_world_generated": (
            projector_shadow_info.get("tcp_pose_world_generated", False)
            or vlm_grounding_info.get("tcp_pose_world_generated", False)
            or camera_source_info.get("tcp_pose_world_generated", False)
            or real_scene_shadow_info.get("tcp_pose_world_generated", False)
        ),
        "real_scene_shadow_evidence_files": (
            _real_scene_shadow_evidence_files(real_scene_shadow_info) if real_scene_shadow_requested else []
        ),
        "real_scene_shadow_result_path": str(real_scene_shadow_result_path) if real_scene_shadow_requested else None,
        "real_scene_shadow_report_path": str(real_scene_shadow_report_path) if real_scene_shadow_requested else None,
        "perception_shadow_evidence_available": perception_shadow_requested,
        "perception_shadow_status": perception_shadow_info.get("perception_shadow_status"),
        "perception_shadow_requested": perception_shadow_requested,
        "user_command": perception_shadow_info.get("user_command")
        if perception_shadow_requested
        else vlm_grounding_info.get("user_command"),
        "normalized_command": perception_shadow_info.get("normalized_command")
        if perception_shadow_requested
        else vlm_grounding_info.get("normalized_command"),
        "snapshot_id": perception_shadow_info.get("snapshot_id")
        or projector_shadow_info.get("snapshot_id")
        or geometry_validity_info.get("snapshot_id")
        or vlm_grounding_info.get("snapshot_id")
        or real_scene_shadow_info.get("snapshot_id")
        or camera_source_info.get("snapshot_id"),
        "grounding_id": perception_shadow_info.get("grounding_id")
        or projector_shadow_info.get("grounding_id")
        or geometry_validity_info.get("grounding_id")
        or vlm_grounding_info.get("grounding_id")
        or real_scene_shadow_info.get("grounding_id"),
        "scene_version": perception_shadow_info.get("scene_version")
        or projector_shadow_info.get("scene_version")
        or geometry_validity_info.get("scene_version")
        or vlm_grounding_info.get("scene_version")
        or real_scene_shadow_info.get("scene_version")
        or camera_snapshot_info.get("scene_version")
        or camera_source_info.get("scene_version")
        or result.get("scene_version"),
        "camera_source_status": perception_shadow_info.get("camera_source_status")
        if perception_shadow_requested
        else camera_source_info.get("camera_source_status"),
        "vlm_grounding_status": perception_shadow_info.get("vlm_grounding_status")
        if perception_shadow_requested
        else vlm_grounding_info.get("vlm_grounding_status"),
        "real_scene_shadow_status": perception_shadow_info.get("real_scene_shadow_status")
        if perception_shadow_requested
        else real_scene_shadow_info.get("shadow_pipeline_status"),
        "geometry_validity_status": perception_shadow_info.get("geometry_validity_status")
        if perception_shadow_requested
        else geometry_validity_info.get("geometry_validity_status"),
        "projector_status": perception_shadow_info.get("projector_status")
        if perception_shadow_requested
        else projector_shadow_info.get("projector_status"),
        "semantic_gate_passed": perception_shadow_info.get("semantic_gate_passed")
        if perception_shadow_requested
        else (
            real_scene_shadow_info.get("semantic_gate_passed", False)
            if real_scene_shadow_requested
            else semantic_bridge_info.get("gate_passed")
        ),
        "no_motion_perception_passed": perception_shadow_info.get("no_motion_perception_passed", False),
        "target_label": perception_shadow_info.get("target_label") or vlm_grounding_info.get("target_label"),
        "target_object_id": perception_shadow_info.get("target_object_id") or vlm_grounding_info.get("target_object_id"),
        "bbox_xyxy": perception_shadow_info.get("bbox_xyxy") or vlm_grounding_info.get("bbox_xyxy"),
        "pixel_center": perception_shadow_info.get("pixel_center")
        or projector_shadow_info.get("pixel_center")
        or vlm_grounding_info.get("pixel_center"),
        "overall_confidence": perception_shadow_info.get("overall_confidence")
        if perception_shadow_requested
        else vlm_grounding_info.get("overall_confidence"),
        "depth_value_m": perception_shadow_info.get("depth_value_m")
        if perception_shadow_requested
        else projector_shadow_info.get("depth_value_m"),
        "camera_point_m": perception_shadow_info.get("camera_point_m")
        if perception_shadow_requested
        else projector_shadow_info.get("camera_point_m"),
        "world_point_m": perception_shadow_info.get("world_point_m")
        if perception_shadow_requested
        else projector_shadow_info.get("world_point_m"),
        "workspace_check_passed": perception_shadow_info.get("workspace_check_passed")
        if perception_shadow_requested
        else projector_shadow_info.get("workspace_check_passed"),
        "replay_ready": perception_shadow_info.get("replay_ready")
        if perception_shadow_requested
        else (
            real_scene_shadow_info.get("replay_ready", False)
            if real_scene_shadow_requested
            else simulated_task_execution_info.get("replay_ready", False)
        ),
        "blocking_reasons": perception_shadow_info.get("blocking_reasons")
        if perception_shadow_requested
        else (
            projector_shadow_info.get("blocking_reasons")
            if projector_shadow_requested
            else geometry_validity_info.get("blocking_reasons")
            if geometry_validity_requested
            else vlm_grounding_info.get("blocking_reasons")
            if vlm_grounding_requested
            else real_scene_shadow_info.get("blocking_reasons")
            if real_scene_shadow_requested
            else camera_source_info.get("blocking_reasons")
            if camera_source_requested
            else list(lab_readiness_info.get("blocking_reasons") or result.get("blocking_reasons") or [])
        ),
        "warnings": perception_shadow_info.get("warnings", [])
        if perception_shadow_requested
        else (
            projector_shadow_info.get("warnings", [])
            if projector_shadow_requested
            else geometry_validity_info.get("warnings", [])
            if geometry_validity_requested
            else vlm_grounding_info.get("warnings", [])
            if vlm_grounding_requested
            else real_scene_shadow_info.get("warnings", [])
            if real_scene_shadow_requested
            else camera_source_info.get("warnings", [])
            if camera_source_requested
            else []
        ),
        "next_safe_action": perception_shadow_info.get("next_safe_action")
        if perception_shadow_requested
        else (
            projector_shadow_info.get("next_safe_action")
            if projector_shadow_requested
            else geometry_validity_info.get("next_safe_action")
            if geometry_validity_requested
            else vlm_grounding_info.get("next_safe_action")
            if vlm_grounding_requested
            else real_scene_shadow_info.get("next_safe_action")
            if real_scene_shadow_requested
            else camera_source_info.get("next_safe_action")
            if camera_source_requested
            else lab_readiness_info.get("next_safe_action")
        ),
        "live_camera_used": perception_shadow_info.get("live_camera_used", False)
        if perception_shadow_requested
        else (
            camera_snapshot_info.get("live_capture_used", False)
            or camera_source_info.get("live_camera_capture_used", False)
            or lab_readiness_info.get("live_camera_used", False)
        ),
        "live_vlm_called": perception_shadow_info.get("live_vlm_called", False)
        if perception_shadow_requested
        else (
            camera_snapshot_info.get("live_vlm_called", False)
            or camera_source_info.get("live_vlm_called", False)
            or vlm_grounding_info.get("live_vlm_called", False)
            or lab_readiness_info.get("live_vlm_called", False)
        ),
        "real_robot_motion_executed": perception_shadow_info.get("real_robot_motion_executed", False)
        if perception_shadow_requested
        else (
            camera_snapshot_info.get("real_robot_motion_executed", False)
            or camera_source_info.get("real_robot_motion_executed", False)
            or lab_readiness_info.get("real_robot_motion_executed", False)
            or result.get("real_robot_motion_executed", False)
        ),
        "real_robot_command_enabled": perception_shadow_info.get("real_robot_command_enabled", False)
        if perception_shadow_requested
        else (
            camera_snapshot_info.get("real_robot_command_enabled", False)
            or camera_source_info.get("real_robot_command_enabled", False)
            or lab_readiness_info.get("real_robot_command_enabled", False)
        ),
        "robot_command_generated": perception_shadow_info.get("robot_command_generated", False)
        if perception_shadow_requested
        else (
            projector_shadow_info.get("robot_command_generated", False)
            or vlm_grounding_info.get("robot_command_generated", False)
            or camera_source_info.get("robot_command_generated", False)
            or real_scene_shadow_info.get("robot_command_generated", False)
        ),
        "trajectory_generated": perception_shadow_info.get("trajectory_generated", False)
        if perception_shadow_requested
        else (
            projector_shadow_info.get("trajectory_generated", False)
            or vlm_grounding_info.get("trajectory_generated", False)
            or camera_source_info.get("trajectory_generated", False)
            or real_scene_shadow_info.get("trajectory_generated", False)
        ),
        "joint_targets_generated": perception_shadow_info.get("joint_targets_generated", False)
        if perception_shadow_requested
        else (
            projector_shadow_info.get("joint_targets_generated", False)
            or vlm_grounding_info.get("joint_targets_generated", False)
            or camera_source_info.get("joint_targets_generated", False)
            or real_scene_shadow_info.get("joint_targets_generated", False)
        ),
        "tcp_pose_world_generated": perception_shadow_info.get("tcp_pose_world_generated", False)
        if perception_shadow_requested
        else (
            projector_shadow_info.get("tcp_pose_world_generated", False)
            or vlm_grounding_info.get("tcp_pose_world_generated", False)
            or camera_source_info.get("tcp_pose_world_generated", False)
            or real_scene_shadow_info.get("tcp_pose_world_generated", False)
        ),
        "perception_shadow_evidence_files": (
            _perception_shadow_evidence_files(perception_shadow_info) if perception_shadow_requested else []
        ),
        "perception_shadow_result_path": str(perception_shadow_result_path)
        if perception_shadow_requested
        else None,
        "perception_shadow_report_path": str(perception_shadow_report_path)
        if perception_shadow_requested
        else None,
        "planner_gateway_shadow_evidence_available": planner_gateway_shadow_requested,
        "planner_gateway_shadow_requested": planner_gateway_shadow_requested,
        "planner_gateway_shadow_status": planner_gateway_shadow_info.get("planner_gateway_shadow_status"),
        "gateway_request_id": planner_gateway_shadow_info.get("gateway_request_id")
        if planner_gateway_shadow_requested
        else None,
        "task_id": planner_gateway_shadow_info.get("task_id") if planner_gateway_shadow_requested else None,
        "user_command": planner_gateway_shadow_info.get("user_command")
        if planner_gateway_shadow_requested
        else (
            perception_shadow_info.get("user_command")
            if perception_shadow_requested
            else vlm_grounding_info.get("user_command")
        ),
        "normalized_command": planner_gateway_shadow_info.get("normalized_command")
        if planner_gateway_shadow_requested
        else (
            perception_shadow_info.get("normalized_command")
            if perception_shadow_requested
            else vlm_grounding_info.get("normalized_command")
        ),
        "intent_name": planner_gateway_shadow_info.get("intent_name") if planner_gateway_shadow_requested else None,
        "target_label": planner_gateway_shadow_info.get("target_label")
        if planner_gateway_shadow_requested
        else perception_shadow_info.get("target_label")
        or vlm_grounding_info.get("target_label"),
        "target_object_id": planner_gateway_shadow_info.get("target_object_id")
        if planner_gateway_shadow_requested
        else perception_shadow_info.get("target_object_id")
        or vlm_grounding_info.get("target_object_id"),
        "snapshot_id": planner_gateway_shadow_info.get("snapshot_id")
        if planner_gateway_shadow_requested
        else perception_shadow_info.get("snapshot_id")
        or projector_shadow_info.get("snapshot_id")
        or geometry_validity_info.get("snapshot_id")
        or vlm_grounding_info.get("snapshot_id")
        or real_scene_shadow_info.get("snapshot_id")
        or camera_source_info.get("snapshot_id"),
        "grounding_id": planner_gateway_shadow_info.get("grounding_id")
        if planner_gateway_shadow_requested
        else perception_shadow_info.get("grounding_id")
        or projector_shadow_info.get("grounding_id")
        or geometry_validity_info.get("grounding_id")
        or vlm_grounding_info.get("grounding_id")
        or real_scene_shadow_info.get("grounding_id"),
        "scene_version": planner_gateway_shadow_info.get("scene_version")
        if planner_gateway_shadow_requested
        else perception_shadow_info.get("scene_version")
        or projector_shadow_info.get("scene_version")
        or geometry_validity_info.get("scene_version")
        or vlm_grounding_info.get("scene_version")
        or real_scene_shadow_info.get("scene_version")
        or camera_snapshot_info.get("scene_version")
        or camera_source_info.get("scene_version")
        or result.get("scene_version"),
        "world_frame": planner_gateway_shadow_info.get("world_frame")
        if planner_gateway_shadow_requested
        else (
            ros2_message_export_info.get("world_frame")
            if ros2_message_export_requested
            else ros2_interface_readiness_info.get("world_frame")
            if ros2_interface_readiness_requested
            else projector_shadow_info.get("world_frame")
        ),
        "world_point_m": planner_gateway_shadow_info.get("world_point_m")
        if planner_gateway_shadow_requested
        else (
            perception_shadow_info.get("world_point_m")
            if perception_shadow_requested
            else projector_shadow_info.get("world_point_m")
        ),
        "bounded_target_point_m": ros2_message_export_info.get("bounded_target_point_m")
        if ros2_message_export_requested
        else planner_gateway_shadow_info.get("bounded_target_point_m")
        if planner_gateway_shadow_requested
        else None,
        "hover_offset_m": planner_gateway_shadow_info.get("hover_offset_m") if planner_gateway_shadow_requested else None,
        "workspace_check_passed": planner_gateway_shadow_info.get("workspace_check_passed")
        if planner_gateway_shadow_requested
        else (
            perception_shadow_info.get("workspace_check_passed")
            if perception_shadow_requested
            else projector_shadow_info.get("workspace_check_passed")
        ),
        "confidence_check_passed": planner_gateway_shadow_info.get("confidence_check_passed")
        if planner_gateway_shadow_requested
        else geometry_validity_info.get("confidence_check_passed"),
        "planner_input_ready": planner_gateway_shadow_info.get("planner_input_ready", False),
        "manual_confirmation_required": planner_gateway_shadow_info.get(
            "manual_confirmation_required",
            True,
        ),
        "execution_allowed": False,
        "ros2_publish_enabled": False,
        "ros2_publish_attempted": False,
        "moveit_called": False,
        "trajectory_generated": planner_gateway_shadow_info.get("trajectory_generated", False)
        if planner_gateway_shadow_requested
        else False,
        "tcp_pose_world_generated": planner_gateway_shadow_info.get("tcp_pose_world_generated", False)
        if planner_gateway_shadow_requested
        else False,
        "joint_targets_generated": planner_gateway_shadow_info.get("joint_targets_generated", False)
        if planner_gateway_shadow_requested
        else False,
        "robot_command_generated": planner_gateway_shadow_info.get("robot_command_generated", False)
        if planner_gateway_shadow_requested
        else False,
        "real_robot_motion_executed": planner_gateway_shadow_info.get("real_robot_motion_executed", False)
        if planner_gateway_shadow_requested
        else False,
        "blocking_reasons": ros2_message_export_info.get("blocking_reasons", [])
        if ros2_message_export_requested
        else planner_gateway_shadow_info.get("blocking_reasons", [])
        if planner_gateway_shadow_requested
        else (
            perception_shadow_info.get("blocking_reasons", [])
            if perception_shadow_requested
            else projector_shadow_info.get("blocking_reasons")
            if projector_shadow_requested
            else geometry_validity_info.get("blocking_reasons")
            if geometry_validity_requested
            else vlm_grounding_info.get("blocking_reasons")
            if vlm_grounding_requested
            else real_scene_shadow_info.get("blocking_reasons")
            if real_scene_shadow_requested
            else camera_source_info.get("blocking_reasons")
            if camera_source_requested
            else list(lab_readiness_info.get("blocking_reasons") or result.get("blocking_reasons") or [])
        ),
        "warnings": ros2_message_export_info.get("warnings", [])
        if ros2_message_export_requested
        else planner_gateway_shadow_info.get("warnings", [])
        if planner_gateway_shadow_requested
        else (
            perception_shadow_info.get("warnings", [])
            if perception_shadow_requested
            else projector_shadow_info.get("warnings", [])
            if projector_shadow_requested
            else geometry_validity_info.get("warnings", [])
            if geometry_validity_requested
            else vlm_grounding_info.get("warnings", [])
            if vlm_grounding_requested
            else real_scene_shadow_info.get("warnings", [])
            if real_scene_shadow_requested
            else camera_source_info.get("warnings", [])
            if camera_source_requested
            else []
        ),
        "next_safe_action": ros2_message_export_info.get("next_safe_action")
        if ros2_message_export_requested
        else planner_gateway_shadow_info.get("next_safe_action")
        if planner_gateway_shadow_requested
        else (
            perception_shadow_info.get("next_safe_action")
            if perception_shadow_requested
            else projector_shadow_info.get("next_safe_action")
            if projector_shadow_requested
            else geometry_validity_info.get("next_safe_action")
            if geometry_validity_requested
            else vlm_grounding_info.get("next_safe_action")
            if vlm_grounding_requested
            else real_scene_shadow_info.get("next_safe_action")
            if real_scene_shadow_requested
            else camera_source_info.get("next_safe_action")
            if camera_source_requested
            else lab_readiness_info.get("next_safe_action")
        ),
        "replay_ready": planner_gateway_shadow_info.get("replay_ready", False)
        if planner_gateway_shadow_requested
        else (
            perception_shadow_info.get("replay_ready", False)
            if perception_shadow_requested
            else real_scene_shadow_info.get("replay_ready", False)
            if real_scene_shadow_requested
            else simulated_task_execution_info.get("replay_ready", False)
        ),
        "planner_gateway_shadow_evidence_files": (
            _planner_gateway_shadow_evidence_files(planner_gateway_shadow_info)
            if planner_gateway_shadow_requested
            else []
        ),
        "planner_gateway_shadow_result_path": str(planner_gateway_shadow_result_path)
        if planner_gateway_shadow_requested
        else None,
        "planner_gateway_shadow_report_path": str(planner_gateway_shadow_report_path)
        if planner_gateway_shadow_requested
        else None,
        "ros2_interface_readiness_evidence_available": ros2_interface_readiness_requested,
        "ros2_interface_readiness_status": ros2_interface_readiness_info.get(
            "ros2_interface_readiness_status"
        ),
        "ros2_environment_declared": ros2_interface_readiness_info.get("ros2_environment_declared"),
        "ros_distro": ros2_interface_readiness_info.get("ros_distro"),
        "ros_domain_id": ros2_interface_readiness_info.get("ros_domain_id"),
        "planner_gateway_interface_mode": ros2_message_export_info.get("planner_gateway_interface_mode")
        if ros2_message_export_requested
        else ros2_interface_readiness_info.get("planner_gateway_interface_mode"),
        "planner_gateway_endpoint": ros2_message_export_info.get("planner_gateway_endpoint")
        if ros2_message_export_requested
        else ros2_interface_readiness_info.get("planner_gateway_endpoint"),
        "message_schema": ros2_message_export_info.get("message_schema")
        if ros2_message_export_requested
        else ros2_interface_readiness_info.get("message_schema"),
        "robot_base_frame": ros2_message_export_info.get("robot_base_frame")
        if ros2_message_export_requested
        else ros2_interface_readiness_info.get("robot_base_frame"),
        "camera_frame": ros2_message_export_info.get("camera_frame")
        if ros2_message_export_requested
        else ros2_interface_readiness_info.get("camera_frame")
        if ros2_interface_readiness_requested
        else projector_shadow_info.get("camera_frame") or camera_source_info.get("camera_frame"),
        "shadow_only": ros2_interface_readiness_info.get("shadow_only"),
        "moveit_enabled": ros2_interface_readiness_info.get("moveit_enabled", False),
        "ros2_interface_readiness_blocking_reasons": ros2_interface_readiness_info.get(
            "blocking_reasons",
            [],
        ),
        "ros2_interface_readiness_warnings": ros2_interface_readiness_info.get("warnings", []),
        "ros2_interface_readiness_evidence_files": (
            _ros2_interface_readiness_evidence_files(ros2_interface_readiness_info)
            if ros2_interface_readiness_requested
            else []
        ),
        "ros2_interface_readiness_result_path": str(ros2_interface_readiness_result_path)
        if ros2_interface_readiness_requested
        else None,
        "ros2_interface_readiness_report_path": str(ros2_interface_readiness_report_path)
        if ros2_interface_readiness_requested
        else None,
        "ros2_message_export_evidence_available": ros2_message_export_requested,
        "ros2_message_export_status": ros2_message_export_info.get("ros2_message_export_status"),
        "message_export_status": ros2_message_export_info.get("message_export_status"),
        "message_id": ros2_message_export_info.get("message_id"),
        "fake_publish_only": ros2_message_export_info.get("fake_publish_only", True),
        "ros2_message_export_blocking_reasons": ros2_message_export_info.get("blocking_reasons", []),
        "ros2_message_export_warnings": ros2_message_export_info.get("warnings", []),
        "ros2_message_export_evidence_files": (
            _ros2_message_export_evidence_files(ros2_message_export_info)
            if ros2_message_export_requested
            else []
        ),
        "ros2_message_export_result_path": str(ros2_message_export_result_path)
        if ros2_message_export_requested
        else None,
        "ros2_message_export_report_path": str(ros2_message_export_report_path)
        if ros2_message_export_requested
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
        "semantic_simulation_bridge_result_path": semantic_bridge_result_path,
        "semantic_simulation_bridge_report_path": semantic_bridge_report_path,
        "semantic_task_contract_copy_path": semantic_task_contract_copy_path,
        "simulated_task_execution_result_path": simulated_task_execution_result_path,
        "simulated_task_execution_report_path": simulated_task_execution_report_path,
        "execution_feedback_path": execution_feedback_path,
        "execution_attempt_record_path": execution_attempt_record_path,
        "failure_analysis_path": failure_analysis_path,
        "retry_fallback_recommendation_path": retry_fallback_recommendation_path,
        "lab_readiness_result_path": lab_readiness_result_path,
        "lab_readiness_report_path": lab_readiness_report_path,
        "camera_readiness_result_path": camera_readiness_result_path,
        "live_vlm_readiness_result_path": live_vlm_readiness_result_path,
        "shadow_mode_readiness_result_path": shadow_mode_readiness_result_path,
        "camera_source_result_path": camera_source_result_path,
        "camera_source_report_path": camera_source_report_path,
        "camera_snapshot_result_path": camera_snapshot_result_path,
        "camera_snapshot_report_path": camera_snapshot_report_path,
        "vlm_grounding_result_path": vlm_grounding_result_path,
        "vlm_grounding_report_path": vlm_grounding_report_path,
        "geometry_validity_result_path": geometry_validity_result_path,
        "geometry_validity_report_path": geometry_validity_report_path,
        "projector_shadow_result_path": projector_shadow_result_path,
        "projector_shadow_report_path": projector_shadow_report_path,
        "real_scene_shadow_result_path": real_scene_shadow_result_path,
        "real_scene_shadow_report_path": real_scene_shadow_report_path,
        "perception_shadow_result_path": perception_shadow_result_path,
        "perception_shadow_report_path": perception_shadow_report_path,
        "planner_gateway_shadow_result_path": planner_gateway_shadow_result_path,
        "planner_gateway_shadow_report_path": planner_gateway_shadow_report_path,
        "ros2_interface_readiness_result_path": ros2_interface_readiness_result_path,
        "ros2_interface_readiness_report_path": ros2_interface_readiness_report_path,
        "ros2_message_export_result_path": ros2_message_export_result_path,
        "ros2_message_export_report_path": ros2_message_export_report_path,
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
    info = {
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
    evidence = summarize_motion_evidence(info)
    info.update(evidence)
    return info


def _semantic_bridge_info(result: Dict[str, Any]) -> Dict[str, Any]:
    bridge = result.get("semantic_bridge") if isinstance(result.get("semantic_bridge"), dict) else {}
    return {
        **bridge,
        "requested": result.get("semantic_simulation_bridge_requested", bridge.get("requested", False)) is True,
        "status": result.get("semantic_bridge_status", bridge.get("status", "NOT_REQUESTED")),
        "gate_passed": result.get("semantic_gate_passed", bridge.get("gate_passed", False)) is True,
        "blocking_reasons": result.get("semantic_bridge_blocking_reasons", bridge.get("blocking_reasons", [])),
        "semantic_task_contract_path": result.get(
            "semantic_task_contract_path",
            bridge.get("semantic_task_contract_path"),
        ),
        "semantic_task_id": result.get("semantic_task_id", bridge.get("semantic_task_id")),
        "semantic_scene_version": result.get("semantic_scene_version", bridge.get("semantic_scene_version")),
        "semantic_intent": result.get("semantic_intent", bridge.get("semantic_intent")),
        "semantic_user_command": result.get("semantic_user_command", bridge.get("semantic_user_command")),
        "semantic_target_label": result.get("semantic_target_label", bridge.get("semantic_target_label")),
        "semantic_confidence_overall": result.get(
            "semantic_confidence_overall",
            bridge.get("semantic_confidence_overall"),
        ),
        "triggered_simulation_micro_motion": result.get(
            "triggered_simulation_micro_motion",
            bridge.get("triggered_simulation_micro_motion", False),
        )
        is True,
        "semantic_simulation_bridge_result_path": result.get("semantic_simulation_bridge_result_path"),
        "semantic_simulation_bridge_report_path": result.get("semantic_simulation_bridge_report_path"),
        "semantic_task_contract_copy_path": result.get("semantic_task_contract_copy_path"),
    }


def _semantic_bridge_files(semantic_bridge_info: Dict[str, Any]) -> list[Dict[str, str | None]]:
    return [
        {
            "name": "semantic_simulation_bridge_result.json",
            "path": semantic_bridge_info.get("semantic_simulation_bridge_result_path"),
        },
        {
            "name": "semantic_simulation_bridge_report.md",
            "path": semantic_bridge_info.get("semantic_simulation_bridge_report_path"),
        },
        {
            "name": "semantic_task_contract_copy.json",
            "path": semantic_bridge_info.get("semantic_task_contract_copy_path"),
        },
    ]


def _simulated_task_execution_info(result: Dict[str, Any]) -> Dict[str, Any]:
    execution = (
        result.get("simulated_task_execution")
        if isinstance(result.get("simulated_task_execution"), dict)
        else {}
    )
    return {
        **execution,
        "safe_simulated_task_execution_requested": result.get(
            "safe_simulated_task_execution_requested",
            execution.get("safe_simulated_task_execution_requested", False),
        )
        is True,
        "execution_attempt_id": result.get("execution_attempt_id", execution.get("execution_attempt_id")),
        "execution_max_attempts": result.get("execution_max_attempts", execution.get("execution_max_attempts", 1)),
        "execution_attempt_index": result.get("execution_attempt_index", execution.get("execution_attempt_index", 1)),
        "simulated_task_status": result.get("simulated_task_status", execution.get("simulated_task_status")),
        "execution_feedback_status": result.get(
            "execution_feedback_status",
            execution.get("execution_feedback_status"),
        ),
        "failure_reason": result.get("failure_reason", execution.get("failure_reason")),
        "retry_recommended": result.get("retry_recommended", execution.get("retry_recommended", False)),
        "fallback_recommended": result.get("fallback_recommended", execution.get("fallback_recommended", False)),
        "fallback_type": result.get("fallback_type", execution.get("fallback_type")),
        "replay_ready": result.get("replay_ready", execution.get("replay_ready", False)),
        "post_motion_state_check": result.get(
            "post_motion_state_check",
            execution.get("post_motion_state_check", {}),
        ),
        "execution_feedback": result.get("execution_feedback", execution.get("execution_feedback", {})),
        "execution_attempt_record": result.get(
            "execution_attempt_record",
            execution.get("execution_attempt_record", {}),
        ),
        "failure_analysis": result.get("failure_analysis", execution.get("failure_analysis", {})),
        "retry_fallback_recommendation": result.get(
            "retry_fallback_recommendation",
            execution.get("retry_fallback_recommendation", {}),
        ),
        "safety_boundary": result.get("safety_boundary", execution.get("safety_boundary", {})),
        "simulated_task_execution_result_path": result.get("simulated_task_execution_result_path"),
        "simulated_task_execution_report_path": result.get("simulated_task_execution_report_path"),
        "execution_feedback_path": result.get("execution_feedback_path"),
        "execution_attempt_record_path": result.get("execution_attempt_record_path"),
        "failure_analysis_path": result.get("failure_analysis_path"),
        "retry_fallback_recommendation_path": result.get("retry_fallback_recommendation_path"),
    }


def _simulated_task_execution_files(execution_info: Dict[str, Any]) -> list[Dict[str, str | None]]:
    return [
        {"name": "simulated_task_execution_result.json", "path": execution_info.get("simulated_task_execution_result_path")},
        {"name": "simulated_task_execution_report.md", "path": execution_info.get("simulated_task_execution_report_path")},
        {"name": "execution_feedback.json", "path": execution_info.get("execution_feedback_path")},
        {"name": "execution_attempt_record.json", "path": execution_info.get("execution_attempt_record_path")},
        {"name": "failure_analysis.json", "path": execution_info.get("failure_analysis_path")},
        {
            "name": "retry_fallback_recommendation.json",
            "path": execution_info.get("retry_fallback_recommendation_path"),
        },
    ]


def _lab_readiness_info(result: Dict[str, Any]) -> Dict[str, Any]:
    readiness = result.get("lab_readiness") if isinstance(result.get("lab_readiness"), dict) else {}
    return {
        **readiness,
        "requested": result.get("lab_readiness_requested", readiness.get("requested", False)) is True,
        "status": result.get("lab_readiness_status", readiness.get("status", "NOT_REQUESTED")),
        "lab_backend_readiness_status": result.get(
            "lab_backend_readiness_status",
            readiness.get("lab_backend_readiness_status", "NOT_REQUESTED"),
        ),
        "camera_readiness_status": result.get(
            "camera_readiness_status",
            readiness.get("camera_readiness_status", "NOT_REQUESTED"),
        ),
        "live_vlm_readiness_status": result.get(
            "live_vlm_readiness_status",
            readiness.get("live_vlm_readiness_status", "NOT_REQUESTED"),
        ),
        "shadow_mode_readiness_status": result.get(
            "shadow_mode_readiness_status",
            readiness.get("shadow_mode_readiness_status", "NOT_REQUESTED"),
        ),
        "no_motion_readiness_passed": result.get(
            "no_motion_readiness_passed",
            readiness.get("no_motion_readiness_passed", False),
        )
        is True,
        "allow_robot_motion": result.get("allow_robot_motion", readiness.get("allow_robot_motion", False)) is True,
        "allow_live_camera": result.get("allow_live_camera", readiness.get("allow_live_camera", False)) is True,
        "allow_live_vlm": result.get("allow_live_vlm", readiness.get("allow_live_vlm", False)) is True,
        "real_robot_command_enabled": result.get(
            "real_robot_command_enabled",
            readiness.get("real_robot_command_enabled", False),
        )
        is True,
        "blocking_reasons": result.get("readiness_blocking_reasons", readiness.get("blocking_reasons", [])),
        "next_safe_action": result.get("next_safe_action", readiness.get("next_safe_action")),
    }


def _readiness_evidence_files(lab_readiness_info: Dict[str, Any]) -> list[Dict[str, str | None]]:
    return [
        {"name": "lab_readiness_result.json", "path": lab_readiness_info.get("lab_readiness_result_path")},
        {"name": "lab_readiness_report.md", "path": lab_readiness_info.get("lab_readiness_report_path")},
        {"name": "camera_readiness_result.json", "path": lab_readiness_info.get("camera_readiness_result_path")},
        {"name": "live_vlm_readiness_result.json", "path": lab_readiness_info.get("live_vlm_readiness_result_path")},
        {
            "name": "shadow_mode_readiness_result.json",
            "path": lab_readiness_info.get("shadow_mode_readiness_result_path"),
        },
    ]


def _readiness_statuses(lab_readiness_info: Dict[str, Any]) -> Dict[str, Any]:
    statuses = lab_readiness_info.get("readiness_statuses")
    if isinstance(statuses, dict):
        return statuses
    return {
        "lab_backend": _readiness_evidence_status(lab_readiness_info.get("lab_backend_readiness_status")),
        "camera": _readiness_evidence_status(lab_readiness_info.get("camera_readiness_status")),
        "live_vlm": _readiness_evidence_status(lab_readiness_info.get("live_vlm_readiness_status")),
        "shadow_mode": _readiness_evidence_status(lab_readiness_info.get("shadow_mode_readiness_status")),
    }


def _readiness_evidence_status(status: Any) -> str:
    if status == "READY_FOR_SHADOW_MODE":
        return "PASS"
    if status in {"BLOCKED", "NOT_READY", "CONFIG_ONLY", "DISABLED"}:
        return str(status)
    return "NOT_REQUESTED"


def _readiness_safety_flags(lab_readiness_info: Dict[str, Any]) -> Dict[str, bool]:
    safety_flags = lab_readiness_info.get("safety_flags")
    if isinstance(safety_flags, dict):
        return {str(key): bool(value) for key, value in safety_flags.items()}
    return {
        "allow_robot_motion": lab_readiness_info.get("allow_robot_motion", False) is True,
        "allow_live_camera": lab_readiness_info.get("allow_live_camera", False) is True,
        "allow_live_vlm": lab_readiness_info.get("allow_live_vlm", False) is True,
        "real_robot_backend_used": False,
        "real_robot_command_enabled": lab_readiness_info.get("real_robot_command_enabled", False) is True,
        "real_robot_motion_executed": lab_readiness_info.get("real_robot_motion_executed", False) is True,
        "live_camera_used": lab_readiness_info.get("live_camera_used", False) is True,
        "live_vlm_called": lab_readiness_info.get("live_vlm_called", False) is True,
        "ros2_used": False,
        "moveit_used": False,
        "rtde_used": False,
        "urscript_used": False,
        "dashboard_used": False,
        "trajectory_generated": False,
        "tcp_pose_world_executed": False,
        "automatic_retry_motion_executed": False,
    }


def _camera_source_info(result: Dict[str, Any]) -> Dict[str, Any]:
    camera_source = result.get("camera_source") if isinstance(result.get("camera_source"), dict) else {}
    return {
        **camera_source,
        "requested": result.get("camera_source_requested", camera_source.get("requested", False)) is True,
        "camera_source_requested": result.get(
            "camera_source_requested",
            camera_source.get("camera_source_requested", False),
        )
        is True,
        "camera_source_status": result.get(
            "camera_source_status",
            camera_source.get("camera_source_status", "NOT_REQUESTED"),
        ),
        "source_mode": result.get("camera_source_mode", camera_source.get("source_mode")),
        "snapshot_id": result.get("camera_source_snapshot_id", camera_source.get("snapshot_id")),
        "scene_version": result.get("camera_source_scene_version", camera_source.get("scene_version")),
        "blocking_reasons": result.get(
            "camera_source_blocking_reasons",
            camera_source.get("blocking_reasons", []),
        ),
        "warnings": result.get("camera_source_warnings", camera_source.get("warnings", [])),
        "next_safe_action": result.get(
            "camera_source_next_safe_action",
            camera_source.get("next_safe_action"),
        ),
        "no_motion_camera_adapter_passed": result.get(
            "no_motion_camera_adapter_passed",
            camera_source.get("no_motion_camera_adapter_passed", False),
        )
        is True,
        "one_shot_capture_used": camera_source.get("one_shot_capture_used", False) is True,
        "continuous_capture_used": camera_source.get("continuous_capture_used", False) is True,
        "live_camera_capture_allowed": camera_source.get("live_camera_capture_allowed", False) is True,
        "live_camera_capture_used": camera_source.get("live_camera_capture_used", False) is True,
        "live_vlm_called": camera_source.get("live_vlm_called", False) is True,
        "real_robot_motion_executed": camera_source.get("real_robot_motion_executed", False) is True,
        "real_robot_command_enabled": camera_source.get("real_robot_command_enabled", False) is True,
        "robot_command_generated": camera_source.get("robot_command_generated", False) is True,
        "trajectory_generated": camera_source.get("trajectory_generated", False) is True,
        "joint_targets_generated": camera_source.get("joint_targets_generated", False) is True,
        "tcp_pose_world_generated": camera_source.get("tcp_pose_world_generated", False) is True,
    }


def _camera_source_evidence_files(camera_source_info: Dict[str, Any]) -> list[Dict[str, str | None]]:
    return [
        {
            "name": "camera_source_result.json",
            "path": camera_source_info.get("camera_source_result_path"),
        },
        {
            "name": "camera_source_report.md",
            "path": camera_source_info.get("camera_source_report_path"),
        },
    ]


def _camera_snapshot_info(result: Dict[str, Any]) -> Dict[str, Any]:
    snapshot = result.get("camera_snapshot") if isinstance(result.get("camera_snapshot"), dict) else {}
    return {
        **snapshot,
        "requested": result.get("camera_snapshot_requested", snapshot.get("requested", False)) is True,
        "snapshot_id": result.get("camera_snapshot_id", snapshot.get("snapshot_id")),
        "scene_version": result.get("camera_snapshot_scene_version", snapshot.get("scene_version")),
        "validity_status": result.get(
            "camera_snapshot_validity_status",
            snapshot.get("validity_status", "NOT_REQUESTED"),
        ),
        "blocking_reasons": result.get(
            "camera_snapshot_blocking_reasons",
            snapshot.get("blocking_reasons", []),
        ),
        "warnings": result.get("camera_snapshot_warnings", snapshot.get("warnings", [])),
        "no_motion_snapshot_passed": result.get(
            "no_motion_snapshot_passed",
            snapshot.get("no_motion_snapshot_passed", False),
        )
        is True,
        "live_capture_used": result.get("live_capture_used", snapshot.get("live_capture_used", False)) is True,
        "live_camera_enabled": result.get(
            "live_camera_enabled",
            snapshot.get("live_camera_enabled", False),
        )
        is True,
        "live_vlm_called": result.get("live_vlm_called", snapshot.get("live_vlm_called", False)) is True,
        "real_robot_motion_executed": result.get(
            "real_robot_motion_executed",
            snapshot.get("real_robot_motion_executed", False),
        )
        is True,
        "real_robot_command_enabled": result.get(
            "real_robot_command_enabled",
            snapshot.get("real_robot_command_enabled", False),
        )
        is True,
    }


def _camera_snapshot_evidence_files(camera_snapshot_info: Dict[str, Any]) -> list[Dict[str, str | None]]:
    return [
        {
            "name": "camera_snapshot_result.json",
            "path": camera_snapshot_info.get("camera_snapshot_result_path"),
        },
        {
            "name": "camera_snapshot_report.md",
            "path": camera_snapshot_info.get("camera_snapshot_report_path"),
        },
    ]


def _vlm_grounding_info(result: Dict[str, Any]) -> Dict[str, Any]:
    grounding = result.get("vlm_grounding") if isinstance(result.get("vlm_grounding"), dict) else {}
    return {
        **grounding,
        "requested": result.get("vlm_grounding_requested", grounding.get("requested", False)) is True,
        "vlm_grounding_requested": result.get(
            "vlm_grounding_requested",
            grounding.get("vlm_grounding_requested", False),
        )
        is True,
        "vlm_grounding_status": result.get(
            "vlm_grounding_status",
            grounding.get("vlm_grounding_status", "NOT_REQUESTED"),
        ),
        "grounding_id": result.get("vlm_grounding_id", grounding.get("grounding_id")),
        "snapshot_id": result.get("vlm_grounding_snapshot_id", grounding.get("snapshot_id")),
        "scene_version": result.get("vlm_grounding_scene_version", grounding.get("scene_version")),
        "user_command": result.get("vlm_grounding_user_command", grounding.get("user_command")),
        "normalized_command": result.get(
            "vlm_grounding_normalized_command",
            grounding.get("normalized_command"),
        ),
        "adapter_mode": result.get("vlm_grounding_adapter_mode", grounding.get("adapter_mode")),
        "target_label": result.get("vlm_grounding_target_label", grounding.get("target_label")),
        "blocking_reasons": result.get(
            "vlm_grounding_blocking_reasons",
            grounding.get("blocking_reasons", []),
        ),
        "warnings": result.get("vlm_grounding_warnings", grounding.get("warnings", [])),
        "next_safe_action": result.get(
            "vlm_grounding_next_safe_action",
            grounding.get("next_safe_action"),
        ),
        "no_motion_grounding_passed": result.get(
            "no_motion_grounding_passed",
            grounding.get("no_motion_grounding_passed", False),
        )
        is True,
        "live_camera_used": grounding.get("live_camera_used", False) is True,
        "live_vlm_called": grounding.get("live_vlm_called", False) is True,
        "real_robot_motion_executed": grounding.get("real_robot_motion_executed", False) is True,
        "real_robot_command_enabled": grounding.get("real_robot_command_enabled", False) is True,
        "robot_command_generated": grounding.get("robot_command_generated", False) is True,
        "trajectory_generated": grounding.get("trajectory_generated", False) is True,
        "joint_targets_generated": grounding.get("joint_targets_generated", False) is True,
        "tcp_pose_world_generated": grounding.get("tcp_pose_world_generated", False) is True,
    }


def _vlm_grounding_evidence_files(vlm_grounding_info: Dict[str, Any]) -> list[Dict[str, str | None]]:
    return [
        {
            "name": "vlm_grounding_result.json",
            "path": vlm_grounding_info.get("vlm_grounding_result_path"),
        },
        {
            "name": "vlm_grounding_report.md",
            "path": vlm_grounding_info.get("vlm_grounding_report_path"),
        },
    ]


def _geometry_validity_info(result: Dict[str, Any]) -> Dict[str, Any]:
    geometry = result.get("geometry_validity") if isinstance(result.get("geometry_validity"), dict) else {}
    return {
        **geometry,
        "requested": result.get("geometry_validity_requested", geometry.get("requested", False)) is True,
        "geometry_validity_requested": result.get(
            "geometry_validity_requested",
            geometry.get("geometry_validity_requested", False),
        )
        is True,
        "geometry_validity_status": result.get(
            "geometry_validity_status",
            geometry.get("geometry_validity_status", "NOT_REQUESTED"),
        ),
        "snapshot_id": result.get("geometry_validity_snapshot_id", geometry.get("snapshot_id")),
        "grounding_id": result.get("geometry_validity_grounding_id", geometry.get("grounding_id")),
        "scene_version": result.get("geometry_validity_scene_version", geometry.get("scene_version")),
        "blocking_reasons": result.get(
            "geometry_validity_blocking_reasons",
            geometry.get("blocking_reasons", []),
        ),
        "warnings": result.get("geometry_validity_warnings", geometry.get("warnings", [])),
        "next_safe_action": result.get(
            "geometry_validity_next_safe_action",
            geometry.get("next_safe_action"),
        ),
        "no_motion_geometry_passed": result.get(
            "no_motion_geometry_passed",
            geometry.get("no_motion_geometry_passed", False),
        )
        is True,
        "live_camera_used": geometry.get("live_camera_used", False) is True,
        "live_vlm_called": geometry.get("live_vlm_called", False) is True,
        "real_robot_motion_executed": geometry.get("real_robot_motion_executed", False) is True,
        "real_robot_command_enabled": geometry.get("real_robot_command_enabled", False) is True,
        "robot_command_generated": geometry.get("robot_command_generated", False) is True,
        "trajectory_generated": geometry.get("trajectory_generated", False) is True,
        "joint_targets_generated": geometry.get("joint_targets_generated", False) is True,
        "tcp_pose_world_generated": geometry.get("tcp_pose_world_generated", False) is True,
    }


def _geometry_validity_evidence_files(geometry_validity_info: Dict[str, Any]) -> list[Dict[str, str | None]]:
    return [
        {
            "name": "geometry_validity_result.json",
            "path": geometry_validity_info.get("geometry_validity_result_path"),
        },
        {
            "name": "geometry_validity_report.md",
            "path": geometry_validity_info.get("geometry_validity_report_path"),
        },
    ]


def _projector_shadow_info(result: Dict[str, Any]) -> Dict[str, Any]:
    projector = result.get("projector_shadow") if isinstance(result.get("projector_shadow"), dict) else {}
    return {
        **projector,
        "requested": result.get("projector_shadow_requested", projector.get("requested", False)) is True,
        "projector_requested": result.get("projector_requested", projector.get("projector_requested", False))
        is True,
        "projector_status": result.get("projector_status", projector.get("projector_status", "NOT_REQUESTED")),
        "snapshot_id": result.get("projector_snapshot_id", projector.get("snapshot_id")),
        "grounding_id": result.get("projector_grounding_id", projector.get("grounding_id")),
        "scene_version": result.get("projector_scene_version", projector.get("scene_version")),
        "blocking_reasons": result.get(
            "projector_blocking_reasons",
            projector.get("blocking_reasons", []),
        ),
        "warnings": result.get("projector_warnings", projector.get("warnings", [])),
        "next_safe_action": result.get("projector_next_safe_action", projector.get("next_safe_action")),
        "no_motion_projector_passed": result.get(
            "no_motion_projector_passed",
            projector.get("no_motion_projector_passed", False),
        )
        is True,
        "real_tf_used": projector.get("real_tf_used", False) is True,
        "ros2_tf_used": projector.get("ros2_tf_used", False) is True,
        "live_camera_used": projector.get("live_camera_used", False) is True,
        "live_vlm_called": projector.get("live_vlm_called", False) is True,
        "real_robot_motion_executed": projector.get("real_robot_motion_executed", False) is True,
        "real_robot_command_enabled": projector.get("real_robot_command_enabled", False) is True,
        "robot_command_generated": projector.get("robot_command_generated", False) is True,
        "trajectory_generated": projector.get("trajectory_generated", False) is True,
        "joint_targets_generated": projector.get("joint_targets_generated", False) is True,
        "tcp_pose_world_generated": projector.get("tcp_pose_world_generated", False) is True,
    }


def _projector_shadow_evidence_files(projector_shadow_info: Dict[str, Any]) -> list[Dict[str, str | None]]:
    return [
        {
            "name": "projector_shadow_result.json",
            "path": projector_shadow_info.get("projector_shadow_result_path"),
        },
        {
            "name": "projector_shadow_report.md",
            "path": projector_shadow_info.get("projector_shadow_report_path"),
        },
    ]


def _real_scene_shadow_info(result: Dict[str, Any]) -> Dict[str, Any]:
    shadow = result.get("real_scene_shadow") if isinstance(result.get("real_scene_shadow"), dict) else {}
    return {
        **shadow,
        "requested": result.get("real_scene_shadow_requested", shadow.get("requested", False)) is True,
        "snapshot_id": result.get("real_scene_shadow_snapshot_id", shadow.get("snapshot_id")),
        "grounding_id": result.get("real_scene_shadow_grounding_id", shadow.get("grounding_id")),
        "scene_version": result.get("real_scene_shadow_scene_version", shadow.get("scene_version")),
        "shadow_pipeline_status": result.get(
            "real_scene_shadow_status",
            shadow.get("shadow_pipeline_status", "NOT_REQUESTED"),
        ),
        "semantic_gate_passed": result.get("semantic_gate_passed", shadow.get("semantic_gate_passed", False))
        is True,
        "no_motion_shadow_passed": result.get(
            "no_motion_shadow_passed",
            shadow.get("no_motion_shadow_passed", False),
        )
        is True,
        "blocking_reasons": result.get(
            "real_scene_shadow_blocking_reasons",
            shadow.get("blocking_reasons", []),
        ),
        "warnings": result.get("real_scene_shadow_warnings", shadow.get("warnings", [])),
        "next_safe_action": result.get(
            "real_scene_shadow_next_safe_action",
            shadow.get("next_safe_action"),
        ),
        "replay_ready": result.get("real_scene_shadow_replay_ready", shadow.get("replay_ready", False)) is True,
        "live_camera_used": shadow.get("live_camera_used", False) is True,
        "live_vlm_called": shadow.get("live_vlm_called", False) is True,
        "real_robot_motion_executed": shadow.get("real_robot_motion_executed", False) is True,
        "real_robot_command_enabled": shadow.get("real_robot_command_enabled", False) is True,
        "robot_command_generated": shadow.get("robot_command_generated", False) is True,
        "trajectory_generated": shadow.get("trajectory_generated", False) is True,
        "joint_targets_generated": shadow.get("joint_targets_generated", False) is True,
        "tcp_pose_world_generated": shadow.get("tcp_pose_world_generated", False) is True,
    }


def _real_scene_shadow_evidence_files(real_scene_shadow_info: Dict[str, Any]) -> list[Dict[str, str | None]]:
    return [
        {
            "name": "real_scene_shadow_result.json",
            "path": real_scene_shadow_info.get("real_scene_shadow_result_path"),
        },
        {
            "name": "real_scene_shadow_report.md",
            "path": real_scene_shadow_info.get("real_scene_shadow_report_path"),
        },
    ]


def _perception_shadow_info(result: Dict[str, Any]) -> Dict[str, Any]:
    perception = result.get("perception_shadow") if isinstance(result.get("perception_shadow"), dict) else {}
    return {
        **perception,
        "requested": result.get("perception_shadow_requested", perception.get("requested", False)) is True,
        "perception_shadow_requested": result.get(
            "perception_shadow_requested",
            perception.get("perception_shadow_requested", False),
        )
        is True,
        "perception_shadow_status": result.get(
            "perception_shadow_status",
            perception.get("perception_shadow_status", "NOT_REQUESTED"),
        ),
        "snapshot_id": result.get("perception_shadow_snapshot_id", perception.get("snapshot_id")),
        "grounding_id": result.get("perception_shadow_grounding_id", perception.get("grounding_id")),
        "scene_version": result.get("perception_shadow_scene_version", perception.get("scene_version")),
        "user_command": result.get("perception_shadow_user_command", perception.get("user_command")),
        "normalized_command": result.get(
            "perception_shadow_normalized_command",
            perception.get("normalized_command"),
        ),
        "camera_source_status": result.get(
            "perception_shadow_camera_source_status",
            perception.get("camera_source_status"),
        ),
        "camera_snapshot_validity_status": result.get(
            "perception_shadow_camera_snapshot_validity_status",
            perception.get("camera_snapshot_validity_status"),
        ),
        "vlm_grounding_status": result.get(
            "perception_shadow_vlm_grounding_status",
            perception.get("vlm_grounding_status"),
        ),
        "real_scene_shadow_status": result.get(
            "perception_shadow_real_scene_shadow_status",
            perception.get("real_scene_shadow_status"),
        ),
        "geometry_validity_status": result.get(
            "perception_shadow_geometry_validity_status",
            perception.get("geometry_validity_status"),
        ),
        "projector_status": result.get(
            "perception_shadow_projector_status",
            perception.get("projector_status"),
        ),
        "target_label": result.get("perception_shadow_target_label", perception.get("target_label")),
        "semantic_gate_passed": perception.get("semantic_gate_passed", False) is True,
        "no_motion_perception_passed": result.get(
            "no_motion_perception_passed",
            perception.get("no_motion_perception_passed", False),
        )
        is True,
        "replay_ready": result.get("perception_shadow_replay_ready", perception.get("replay_ready", False)) is True,
        "blocking_reasons": result.get(
            "perception_shadow_blocking_reasons",
            perception.get("blocking_reasons", []),
        ),
        "warnings": result.get("perception_shadow_warnings", perception.get("warnings", [])),
        "next_safe_action": result.get(
            "perception_shadow_next_safe_action",
            perception.get("next_safe_action"),
        ),
        "live_camera_used": perception.get("live_camera_used", False) is True,
        "live_vlm_called": perception.get("live_vlm_called", False) is True,
        "real_robot_motion_executed": perception.get("real_robot_motion_executed", False) is True,
        "real_robot_command_enabled": perception.get("real_robot_command_enabled", False) is True,
        "robot_command_generated": perception.get("robot_command_generated", False) is True,
        "trajectory_generated": perception.get("trajectory_generated", False) is True,
        "joint_targets_generated": perception.get("joint_targets_generated", False) is True,
        "tcp_pose_world_generated": perception.get("tcp_pose_world_generated", False) is True,
    }


def _perception_shadow_evidence_files(perception_shadow_info: Dict[str, Any]) -> list[Dict[str, str | None]]:
    return [
        {
            "name": "perception_shadow_result.json",
            "path": perception_shadow_info.get("perception_shadow_result_path"),
        },
        {
            "name": "perception_shadow_report.md",
            "path": perception_shadow_info.get("perception_shadow_report_path"),
        },
    ]


def _planner_gateway_shadow_info(result: Dict[str, Any]) -> Dict[str, Any]:
    gateway = result.get("planner_gateway_shadow") if isinstance(result.get("planner_gateway_shadow"), dict) else {}
    return {
        **gateway,
        "requested": result.get("planner_gateway_shadow_requested", gateway.get("requested", False)) is True,
        "planner_gateway_shadow_requested": result.get(
            "planner_gateway_shadow_requested",
            gateway.get("planner_gateway_shadow_requested", False),
        )
        is True,
        "planner_gateway_shadow_status": result.get(
            "planner_gateway_shadow_status",
            gateway.get("planner_gateway_shadow_status", "NOT_REQUESTED"),
        ),
        "gateway_request_id": result.get(
            "planner_gateway_shadow_gateway_request_id",
            gateway.get("gateway_request_id"),
        ),
        "task_id": result.get("planner_gateway_shadow_task_id", gateway.get("task_id")),
        "user_command": gateway.get("user_command"),
        "normalized_command": gateway.get("normalized_command"),
        "intent_name": result.get("planner_gateway_shadow_intent_name", gateway.get("intent_name")),
        "target_label": result.get("planner_gateway_shadow_target_label", gateway.get("target_label")),
        "target_object_id": gateway.get("target_object_id"),
        "snapshot_id": result.get("planner_gateway_shadow_snapshot_id", gateway.get("snapshot_id")),
        "grounding_id": result.get("planner_gateway_shadow_grounding_id", gateway.get("grounding_id")),
        "scene_version": result.get("planner_gateway_shadow_scene_version", gateway.get("scene_version")),
        "world_frame": result.get("planner_gateway_shadow_world_frame", gateway.get("world_frame")),
        "camera_frame": gateway.get("camera_frame"),
        "world_point_m": result.get("planner_gateway_shadow_world_point_m", gateway.get("world_point_m")),
        "bounded_target_point_m": result.get(
            "planner_gateway_shadow_bounded_target_point_m",
            gateway.get("bounded_target_point_m"),
        ),
        "hover_offset_m": gateway.get("hover_offset_m"),
        "workspace_check_passed": result.get(
            "planner_gateway_shadow_workspace_check_passed",
            gateway.get("workspace_check_passed"),
        ),
        "confidence_check_passed": result.get(
            "planner_gateway_shadow_confidence_check_passed",
            gateway.get("confidence_check_passed"),
        ),
        "ttl_check_passed": gateway.get("ttl_check_passed"),
        "semantic_gate_passed": gateway.get("semantic_gate_passed", False) is True,
        "geometry_validity_status": gateway.get("geometry_validity_status"),
        "projector_status": gateway.get("projector_status"),
        "planner_input_ready": result.get("planner_input_ready", gateway.get("planner_input_ready", False)) is True,
        "manual_confirmation_required": result.get(
            "planner_gateway_shadow_manual_confirmation_required",
            gateway.get("manual_confirmation_required", True),
        )
        is True,
        "execution_allowed": gateway.get("execution_allowed", False) is True,
        "ros2_publish_enabled": gateway.get("ros2_publish_enabled", False) is True,
        "ros2_publish_attempted": gateway.get("ros2_publish_attempted", False) is True,
        "moveit_called": gateway.get("moveit_called", False) is True,
        "trajectory_generated": gateway.get("trajectory_generated", False) is True,
        "tcp_pose_world_generated": gateway.get("tcp_pose_world_generated", False) is True,
        "joint_targets_generated": gateway.get("joint_targets_generated", False) is True,
        "robot_command_generated": gateway.get("robot_command_generated", False) is True,
        "real_robot_motion_executed": gateway.get("real_robot_motion_executed", False) is True,
        "blocking_reasons": result.get(
            "planner_gateway_shadow_blocking_reasons",
            gateway.get("blocking_reasons", []),
        ),
        "warnings": result.get("planner_gateway_shadow_warnings", gateway.get("warnings", [])),
        "next_safe_action": result.get(
            "planner_gateway_shadow_next_safe_action",
            gateway.get("next_safe_action"),
        ),
        "replay_ready": result.get("planner_gateway_shadow_replay_ready", gateway.get("replay_ready", False)) is True,
    }


def _planner_gateway_shadow_evidence_files(
    planner_gateway_shadow_info: Dict[str, Any],
) -> list[Dict[str, str | None]]:
    return [
        {
            "name": "planner_gateway_shadow_result.json",
            "path": planner_gateway_shadow_info.get("planner_gateway_shadow_result_path"),
        },
        {
            "name": "planner_gateway_shadow_report.md",
            "path": planner_gateway_shadow_info.get("planner_gateway_shadow_report_path"),
        },
    ]


def _ros2_interface_readiness_info(result: Dict[str, Any]) -> Dict[str, Any]:
    readiness = (
        result.get("ros2_interface_readiness")
        if isinstance(result.get("ros2_interface_readiness"), dict)
        else {}
    )
    return {
        **readiness,
        "requested": result.get("ros2_interface_readiness_requested", readiness.get("requested", False)) is True,
        "ros2_interface_readiness_requested": result.get(
            "ros2_interface_readiness_requested",
            readiness.get("ros2_interface_readiness_requested", False),
        )
        is True,
        "ros2_interface_readiness_status": result.get(
            "ros2_interface_readiness_status",
            readiness.get("ros2_interface_readiness_status", "NOT_REQUESTED"),
        ),
        "ros2_environment_declared": result.get(
            "ros2_environment_declared",
            readiness.get("ros2_environment_declared", False),
        )
        is True,
        "ros_distro": result.get("ros_distro", readiness.get("ros_distro")),
        "ros_domain_id": result.get("ros_domain_id", readiness.get("ros_domain_id")),
        "planner_gateway_interface_mode": result.get(
            "planner_gateway_interface_mode",
            readiness.get("planner_gateway_interface_mode"),
        ),
        "planner_gateway_endpoint": result.get(
            "planner_gateway_endpoint",
            readiness.get("planner_gateway_endpoint"),
        ),
        "message_schema": result.get("message_schema", readiness.get("message_schema")),
        "world_frame": result.get("ros2_interface_world_frame", readiness.get("world_frame")),
        "robot_base_frame": result.get("robot_base_frame", readiness.get("robot_base_frame")),
        "camera_frame": result.get("ros2_interface_camera_frame", readiness.get("camera_frame")),
        "target_frame": result.get("target_frame", readiness.get("target_frame")),
        "shadow_only": result.get("shadow_only", readiness.get("shadow_only", True)) is True,
        "ros2_publish_enabled": readiness.get("ros2_publish_enabled", False) is True,
        "ros2_publish_attempted": readiness.get("ros2_publish_attempted", False) is True,
        "moveit_enabled": readiness.get("moveit_enabled", False) is True,
        "moveit_called": readiness.get("moveit_called", False) is True,
        "execution_allowed": readiness.get("execution_allowed", False) is True,
        "trajectory_generated": readiness.get("trajectory_generated", False) is True,
        "tcp_pose_world_generated": readiness.get("tcp_pose_world_generated", False) is True,
        "joint_targets_generated": readiness.get("joint_targets_generated", False) is True,
        "robot_command_generated": readiness.get("robot_command_generated", False) is True,
        "real_robot_motion_executed": readiness.get("real_robot_motion_executed", False) is True,
        "blocking_reasons": result.get(
            "ros2_interface_blocking_reasons",
            readiness.get("blocking_reasons", []),
        ),
        "warnings": result.get("ros2_interface_warnings", readiness.get("warnings", [])),
        "next_safe_action": result.get(
            "ros2_interface_next_safe_action",
            readiness.get("next_safe_action"),
        ),
        "ros2_interface_readiness_result_path": result.get(
            "ros2_interface_readiness_result_path",
            readiness.get("ros2_interface_readiness_result_path"),
        ),
        "ros2_interface_readiness_report_path": result.get(
            "ros2_interface_readiness_report_path",
            readiness.get("ros2_interface_readiness_report_path"),
        ),
    }


def _ros2_interface_readiness_evidence_files(
    ros2_interface_readiness_info: Dict[str, Any],
) -> list[Dict[str, str | None]]:
    return [
        {
            "name": "ros2_interface_readiness_result.json",
            "path": ros2_interface_readiness_info.get("ros2_interface_readiness_result_path"),
        },
        {
            "name": "ros2_interface_readiness_report.md",
            "path": ros2_interface_readiness_info.get("ros2_interface_readiness_report_path"),
        },
    ]


def _ros2_message_export_info(result: Dict[str, Any]) -> Dict[str, Any]:
    message_export = (
        result.get("ros2_message_export")
        if isinstance(result.get("ros2_message_export"), dict)
        else {}
    )
    return {
        **message_export,
        "requested": result.get("ros2_message_export_requested", message_export.get("requested", False)) is True,
        "ros2_message_export_requested": result.get(
            "ros2_message_export_requested",
            message_export.get("ros2_message_export_requested", False),
        )
        is True,
        "ros2_message_export_status": result.get(
            "ros2_message_export_status",
            message_export.get("ros2_message_export_status", "NOT_REQUESTED"),
        ),
        "message_export_status": result.get(
            "message_export_status",
            message_export.get("message_export_status", "NOT_REQUESTED"),
        ),
        "message_id": result.get("ros2_message_id", message_export.get("message_id")),
        "message_schema": result.get("ros2_message_schema", message_export.get("message_schema")),
        "fake_publish_only": result.get("fake_publish_only", message_export.get("fake_publish_only", True)) is True,
        "ros2_publish_enabled": message_export.get("ros2_publish_enabled", False) is True,
        "ros2_publish_attempted": message_export.get("ros2_publish_attempted", False) is True,
        "planner_gateway_interface_mode": message_export.get("planner_gateway_interface_mode"),
        "planner_gateway_endpoint": message_export.get("planner_gateway_endpoint"),
        "bounded_target_point_m": message_export.get("bounded_target_point_m"),
        "world_frame": message_export.get("world_frame"),
        "robot_base_frame": message_export.get("robot_base_frame"),
        "camera_frame": message_export.get("camera_frame"),
        "execution_allowed": message_export.get("execution_allowed", False) is True,
        "moveit_called": message_export.get("moveit_called", False) is True,
        "trajectory_generated": message_export.get("trajectory_generated", False) is True,
        "tcp_pose_world_generated": message_export.get("tcp_pose_world_generated", False) is True,
        "joint_targets_generated": message_export.get("joint_targets_generated", False) is True,
        "robot_command_generated": message_export.get("robot_command_generated", False) is True,
        "real_robot_motion_executed": message_export.get("real_robot_motion_executed", False) is True,
        "blocking_reasons": result.get(
            "ros2_message_export_blocking_reasons",
            message_export.get("blocking_reasons", []),
        ),
        "warnings": result.get("ros2_message_export_warnings", message_export.get("warnings", [])),
        "next_safe_action": result.get(
            "ros2_message_export_next_safe_action",
            message_export.get("next_safe_action"),
        ),
        "ros2_message_export_result_path": result.get(
            "ros2_message_export_result_path",
            message_export.get("ros2_message_export_result_path"),
        ),
        "ros2_message_export_report_path": result.get(
            "ros2_message_export_report_path",
            message_export.get("ros2_message_export_report_path"),
        ),
    }


def _ros2_message_export_evidence_files(
    ros2_message_export_info: Dict[str, Any],
) -> list[Dict[str, str | None]]:
    return [
        {
            "name": "ros2_message_export_result.json",
            "path": ros2_message_export_info.get("ros2_message_export_result_path"),
        },
        {
            "name": "ros2_message_export_report.md",
            "path": ros2_message_export_info.get("ros2_message_export_report_path"),
        },
    ]


def _latest_execution_summary(
    execution_info: Dict[str, Any],
    precheck_info: Dict[str, Any],
    micro_motion_info: Dict[str, Any],
    semantic_bridge_info: Dict[str, Any],
) -> Dict[str, Any]:
    post_check = execution_info.get("post_motion_state_check") or {}
    return {
        "execution_attempt_id": execution_info.get("execution_attempt_id"),
        "semantic_bridge_status": semantic_bridge_info.get("status"),
        "semantic_gate_passed": semantic_bridge_info.get("gate_passed"),
        "simulation_motion_precheck_status": precheck_info.get("status"),
        "simulation_micro_motion_status": micro_motion_info.get("simulation_micro_motion_status"),
        "post_motion_state_check_status": post_check.get("post_motion_state_check_status"),
        "simulated_task_status": execution_info.get("simulated_task_status"),
        "execution_feedback_status": execution_info.get("execution_feedback_status"),
        "failure_reason": execution_info.get("failure_reason"),
        "retry_recommended": execution_info.get("retry_recommended"),
        "fallback_recommended": execution_info.get("fallback_recommended"),
        "fallback_type": execution_info.get("fallback_type"),
        "replay_ready": execution_info.get("replay_ready", False),
    }


def _replay_bundle_files(
    *,
    execution_evidence_files: list[Dict[str, str | None]],
    motion_evidence_files: list[Dict[str, str | None]],
    semantic_bridge_files: list[Dict[str, str | None]],
) -> list[Dict[str, str | None]]:
    return [
        *semantic_bridge_files,
        *motion_evidence_files,
        *execution_evidence_files,
    ]


def _safety_boundary_confirmed(result: Dict[str, Any], execution_info: Dict[str, Any]) -> bool:
    safety = result.get("safety") if isinstance(result.get("safety"), dict) else {}
    execution_safety = (
        execution_info.get("safety_boundary")
        if isinstance(execution_info.get("safety_boundary"), dict)
        else {}
    )
    return all(
        [
            result.get("simulation_only", safety.get("simulation_only", True)) is True,
            result.get("real_robot_motion_executed", safety.get("real_robot_motion_executed", False)) is False,
            safety.get("no_live_camera_used", execution_safety.get("no_live_camera_used", True)) is True,
            safety.get("no_live_vlm_used", execution_safety.get("no_live_vlm_used", True)) is True,
            safety.get("no_ros2_used", execution_safety.get("no_ros2_used", True)) is True,
            safety.get("no_moveit_used", execution_safety.get("no_moveit_used", True)) is True,
            safety.get("no_rtde_used", execution_safety.get("no_rtde_used", True)) is True,
            safety.get("no_urscript_used", execution_safety.get("no_urscript_used", True)) is True,
            safety.get("no_dashboard_used", execution_safety.get("no_dashboard_used", True)) is True,
            safety.get("no_real_ur5_used", execution_safety.get("no_real_ur5_used", True)) is True,
            safety.get("no_trajectory_generated", execution_safety.get("no_trajectory_generated", True)) is True,
            safety.get("no_tcp_pose_world_executed", execution_safety.get("no_tcp_pose_world_executed", True)) is True,
            execution_safety.get("automatic_retry_executed", False) is False,
        ]
    )


def _write_json_artifact(path: Path, payload: Dict[str, Any]) -> None:
    with path.open("w", encoding="utf-8") as output_file:
        json.dump(payload, output_file, ensure_ascii=False, indent=2)
        output_file.write("\n")


def _simulated_task_execution_report_context(
    result: Dict[str, Any],
    execution_info: Dict[str, Any],
    precheck_info: Dict[str, Any],
    micro_motion_info: Dict[str, Any],
    semantic_bridge_info: Dict[str, Any],
) -> Dict[str, Any]:
    motion = micro_motion_info.get("motion") if isinstance(micro_motion_info.get("motion"), dict) else {}
    return {
        **execution_info,
        "semantic_bridge_status": semantic_bridge_info.get("status"),
        "semantic_gate_passed": semantic_bridge_info.get("gate_passed"),
        "simulation_motion_precheck_status": precheck_info.get("status"),
        "ready_for_simulation_motion": precheck_info.get("ready"),
        "simulation_micro_motion_status": micro_motion_info.get("simulation_micro_motion_status"),
        "actual_delta_rad": result.get("actual_delta_rad", motion.get("actual_delta_rad")),
        "delta_within_tolerance": result.get("delta_within_tolerance", motion.get("delta_within_tolerance")),
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
    semantic_bridge_info: Dict[str, Any],
    simulated_task_execution_info: Dict[str, Any],
    lab_readiness_info: Dict[str, Any],
    camera_source_info: Dict[str, Any],
    camera_snapshot_info: Dict[str, Any],
    vlm_grounding_info: Dict[str, Any],
    geometry_validity_info: Dict[str, Any],
    projector_shadow_info: Dict[str, Any],
    real_scene_shadow_info: Dict[str, Any],
    perception_shadow_info: Dict[str, Any],
    planner_gateway_shadow_info: Dict[str, Any],
    ros2_interface_readiness_info: Dict[str, Any],
    ros2_message_export_info: Dict[str, Any],
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
            "## Camera Source Evidence Summary",
            "",
            f"- camera_source_evidence_available: {_format_value(camera_source_info.get('requested'))}",
            f"- camera_source_status: {_format_value(camera_source_info.get('camera_source_status'))}",
            f"- source_mode: {_format_value(camera_source_info.get('source_mode'))}",
            f"- snapshot_id: {_format_value(camera_source_info.get('snapshot_id'))}",
            f"- scene_version: {_format_value(camera_source_info.get('scene_version'))}",
            f"- capture_timestamp: {_format_value(camera_source_info.get('capture_timestamp'))}",
            f"- frame_id: {_format_value(camera_source_info.get('frame_id'))}",
            f"- camera_frame: {_format_value(camera_source_info.get('camera_frame'))}",
            f"- image_ref: {_format_value(camera_source_info.get('image_ref'))}",
            f"- depth_ref: {_format_value(camera_source_info.get('depth_ref'))}",
            f"- camera_info_ref: {_format_value(camera_source_info.get('camera_info_ref'))}",
            f"- metadata_ref: {_format_value(camera_source_info.get('metadata_ref'))}",
            f"- extrinsics_ref: {_format_value(camera_source_info.get('extrinsics_ref'))}",
            f"- depth_available: {_format_value(camera_source_info.get('depth_available'))}",
            f"- camera_info_available: {_format_value(camera_source_info.get('camera_info_available'))}",
            f"- one_shot_capture_used: {_format_value(camera_source_info.get('one_shot_capture_used'))}",
            f"- continuous_capture_used: {_format_value(camera_source_info.get('continuous_capture_used'))}",
            f"- live_camera_capture_allowed: {_format_value(camera_source_info.get('live_camera_capture_allowed'))}",
            f"- live_camera_capture_used: {_format_value(camera_source_info.get('live_camera_capture_used'))}",
            f"- no_motion_camera_adapter_passed: {_format_value(camera_source_info.get('no_motion_camera_adapter_passed'))}",
            f"- blocking_reasons: {_format_value(camera_source_info.get('blocking_reasons'))}",
            f"- warnings: {_format_value(camera_source_info.get('warnings'))}",
            f"- next_safe_action: {_format_value(camera_source_info.get('next_safe_action'))}",
            f"- live_vlm_called: {_format_value(camera_source_info.get('live_vlm_called'))}",
            f"- real_robot_motion_executed: {_format_value(camera_source_info.get('real_robot_motion_executed'))}",
            f"- real_robot_command_enabled: {_format_value(camera_source_info.get('real_robot_command_enabled'))}",
            f"- robot_command_generated: {_format_value(camera_source_info.get('robot_command_generated'))}",
            f"- trajectory_generated: {_format_value(camera_source_info.get('trajectory_generated'))}",
            f"- joint_targets_generated: {_format_value(camera_source_info.get('joint_targets_generated'))}",
            f"- tcp_pose_world_generated: {_format_value(camera_source_info.get('tcp_pose_world_generated'))}",
            "This V2.9.3 camera source adapter evidence converts offline/manual/live-disabled declarations, and future explicitly allowed one-shot capture declarations, into a TETO camera snapshot contract. It is no-motion, no-live-VLM, no-real-robot, no-ROS2, and no-MoveIt evidence only; it is not a continuous live camera loop and does not generate trajectory, tcp_pose_world, joint targets, or robot commands.",
            "",
            "## Camera Snapshot Evidence Summary",
            "",
            f"- camera_snapshot_evidence_available: {_format_value(camera_snapshot_info.get('requested'))}",
            f"- camera_snapshot_id: {_format_value(camera_snapshot_info.get('snapshot_id'))}",
            f"- scene_version: {_format_value(camera_snapshot_info.get('scene_version'))}",
            f"- camera_snapshot_validity_status: {_format_value(camera_snapshot_info.get('validity_status'))}",
            f"- camera_snapshot_blocking_reasons: {_format_value(camera_snapshot_info.get('blocking_reasons'))}",
            f"- camera_snapshot_warnings: {_format_value(camera_snapshot_info.get('warnings'))}",
            f"- no_motion_snapshot_passed: {_format_value(camera_snapshot_info.get('no_motion_snapshot_passed'))}",
            f"- live_capture_used: {_format_value(camera_snapshot_info.get('live_capture_used'))}",
            f"- live_camera_enabled: {_format_value(camera_snapshot_info.get('live_camera_enabled'))}",
            f"- live_vlm_called: {_format_value(camera_snapshot_info.get('live_vlm_called'))}",
            f"- real_robot_motion_executed: {_format_value(camera_snapshot_info.get('real_robot_motion_executed'))}",
            f"- real_robot_command_enabled: {_format_value(camera_snapshot_info.get('real_robot_command_enabled'))}",
            "This V2.8.2 camera snapshot evidence validates a declared offline/manual snapshot manifest only. It does not capture a live camera frame, call live Qwen/VLM, connect to a real UR5, generate trajectory, execute tcp_pose_world, or produce robot control fields.",
            "",
            "## VLM Grounding Evidence Summary",
            "",
            f"- vlm_grounding_evidence_available: {_format_value(vlm_grounding_info.get('requested'))}",
            f"- vlm_grounding_status: {_format_value(vlm_grounding_info.get('vlm_grounding_status'))}",
            f"- grounding_id: {_format_value(vlm_grounding_info.get('grounding_id'))}",
            f"- snapshot_id: {_format_value(vlm_grounding_info.get('snapshot_id'))}",
            f"- scene_version: {_format_value(vlm_grounding_info.get('scene_version'))}",
            f"- user_command: {_format_value(vlm_grounding_info.get('user_command'))}",
            f"- normalized_command: {_format_value(vlm_grounding_info.get('normalized_command'))}",
            f"- adapter_mode: {_format_value(vlm_grounding_info.get('adapter_mode'))}",
            f"- target_label: {_format_value(vlm_grounding_info.get('target_label'))}",
            f"- target_object_id: {_format_value(vlm_grounding_info.get('target_object_id'))}",
            f"- bbox_xyxy: {_format_value(vlm_grounding_info.get('bbox_xyxy'))}",
            f"- pixel_center: {_format_value(vlm_grounding_info.get('pixel_center'))}",
            f"- semantic_confidence: {_format_value(vlm_grounding_info.get('semantic_confidence'))}",
            f"- grounding_confidence: {_format_value(vlm_grounding_info.get('grounding_confidence'))}",
            f"- overall_confidence: {_format_value(vlm_grounding_info.get('overall_confidence'))}",
            f"- grounded: {_format_value(vlm_grounding_info.get('grounded'))}",
            f"- rejected: {_format_value(vlm_grounding_info.get('rejected'))}",
            f"- rejection_reason: {_format_value(vlm_grounding_info.get('rejection_reason'))}",
            f"- error_code: {_format_value(vlm_grounding_info.get('error_code'))}",
            f"- no_motion_grounding_passed: {_format_value(vlm_grounding_info.get('no_motion_grounding_passed'))}",
            f"- blocking_reasons: {_format_value(vlm_grounding_info.get('blocking_reasons'))}",
            f"- warnings: {_format_value(vlm_grounding_info.get('warnings'))}",
            f"- next_safe_action: {_format_value(vlm_grounding_info.get('next_safe_action'))}",
            f"- live_camera_used: {_format_value(vlm_grounding_info.get('live_camera_used'))}",
            f"- live_vlm_called: {_format_value(vlm_grounding_info.get('live_vlm_called'))}",
            f"- real_robot_motion_executed: {_format_value(vlm_grounding_info.get('real_robot_motion_executed'))}",
            f"- real_robot_command_enabled: {_format_value(vlm_grounding_info.get('real_robot_command_enabled'))}",
            f"- robot_command_generated: {_format_value(vlm_grounding_info.get('robot_command_generated'))}",
            f"- trajectory_generated: {_format_value(vlm_grounding_info.get('trajectory_generated'))}",
            f"- joint_targets_generated: {_format_value(vlm_grounding_info.get('joint_targets_generated'))}",
            f"- tcp_pose_world_generated: {_format_value(vlm_grounding_info.get('tcp_pose_world_generated'))}",
            "This V2.9.4 VLM grounding adapter converts text command plus declared camera snapshot metadata into offline/mock/manual grounding result evidence. It is not live camera, not real VLM execution, not ROS2 bridge, not MoveIt planning, not real UR5 execution, and it does not generate trajectory, tcp_pose_world, URScript, joint targets, or robot commands.",
            "",
            "## Geometry Validity Evidence Summary",
            "",
            f"- geometry_validity_evidence_available: {_format_value(geometry_validity_info.get('requested'))}",
            f"- geometry_validity_status: {_format_value(geometry_validity_info.get('geometry_validity_status'))}",
            f"- snapshot_id: {_format_value(geometry_validity_info.get('snapshot_id'))}",
            f"- grounding_id: {_format_value(geometry_validity_info.get('grounding_id'))}",
            f"- scene_version: {_format_value(geometry_validity_info.get('scene_version'))}",
            f"- bbox_valid: {_format_value(geometry_validity_info.get('bbox_valid'))}",
            f"- pixel_center_valid: {_format_value(geometry_validity_info.get('pixel_center_valid'))}",
            f"- bbox_inside_image: {_format_value(geometry_validity_info.get('bbox_inside_image'))}",
            f"- pixel_center_inside_image: {_format_value(geometry_validity_info.get('pixel_center_inside_image'))}",
            f"- confidence_check_passed: {_format_value(geometry_validity_info.get('confidence_check_passed'))}",
            f"- ttl_check_passed: {_format_value(geometry_validity_info.get('ttl_check_passed'))}",
            f"- depth_required: {_format_value(geometry_validity_info.get('depth_required'))}",
            f"- depth_available: {_format_value(geometry_validity_info.get('depth_available'))}",
            f"- camera_frame_available: {_format_value(geometry_validity_info.get('camera_frame_available'))}",
            f"- no_motion_geometry_passed: {_format_value(geometry_validity_info.get('no_motion_geometry_passed'))}",
            f"- blocking_reasons: {_format_value(geometry_validity_info.get('blocking_reasons'))}",
            f"- warnings: {_format_value(geometry_validity_info.get('warnings'))}",
            f"- next_safe_action: {_format_value(geometry_validity_info.get('next_safe_action'))}",
            f"- live_camera_used: {_format_value(geometry_validity_info.get('live_camera_used'))}",
            f"- live_vlm_called: {_format_value(geometry_validity_info.get('live_vlm_called'))}",
            f"- real_robot_motion_executed: {_format_value(geometry_validity_info.get('real_robot_motion_executed'))}",
            f"- real_robot_command_enabled: {_format_value(geometry_validity_info.get('real_robot_command_enabled'))}",
            f"- robot_command_generated: {_format_value(geometry_validity_info.get('robot_command_generated'))}",
            f"- trajectory_generated: {_format_value(geometry_validity_info.get('trajectory_generated'))}",
            f"- joint_targets_generated: {_format_value(geometry_validity_info.get('joint_targets_generated'))}",
            f"- tcp_pose_world_generated: {_format_value(geometry_validity_info.get('tcp_pose_world_generated'))}",
            "This V2.9.1 geometry validity evidence checks declared bbox, pixel_center, frame, depth, TTL, and confidence before a future 2D-to-3D projector. It is no-motion, no-live-camera, no-live-VLM, and no-real-robot evidence only; it does not connect to ROS2, MoveIt, RTDE, URScript, Dashboard, or a real UR5, and it does not generate trajectory, tcp_pose_world, joint targets, or robot commands.",
            "",
            "## Projector Shadow Evidence Summary",
            "",
            f"- projector_shadow_evidence_available: {_format_value(projector_shadow_info.get('requested'))}",
            f"- projector_requested: {_format_value(projector_shadow_info.get('projector_requested'))}",
            f"- projector_status: {_format_value(projector_shadow_info.get('projector_status'))}",
            f"- snapshot_id: {_format_value(projector_shadow_info.get('snapshot_id'))}",
            f"- grounding_id: {_format_value(projector_shadow_info.get('grounding_id'))}",
            f"- scene_version: {_format_value(projector_shadow_info.get('scene_version'))}",
            f"- pixel_center: {_format_value(projector_shadow_info.get('pixel_center'))}",
            f"- depth_value_m: {_format_value(projector_shadow_info.get('depth_value_m'))}",
            f"- depth_valid: {_format_value(projector_shadow_info.get('depth_valid'))}",
            f"- camera_intrinsics_available: {_format_value(projector_shadow_info.get('camera_intrinsics_available'))}",
            f"- camera_frame: {_format_value(projector_shadow_info.get('camera_frame'))}",
            f"- world_frame: {_format_value(projector_shadow_info.get('world_frame'))}",
            f"- camera_point_m: {_format_value(projector_shadow_info.get('camera_point_m'))}",
            f"- world_point_m: {_format_value(projector_shadow_info.get('world_point_m'))}",
            f"- projection_confidence: {_format_value(projector_shadow_info.get('projection_confidence'))}",
            f"- projection_method: {_format_value(projector_shadow_info.get('projection_method'))}",
            f"- tf_available: {_format_value(projector_shadow_info.get('tf_available'))}",
            f"- tf_source: {_format_value(projector_shadow_info.get('tf_source'))}",
            f"- real_tf_used: {_format_value(projector_shadow_info.get('real_tf_used'))}",
            f"- ros2_tf_used: {_format_value(projector_shadow_info.get('ros2_tf_used'))}",
            f"- workspace_check_passed: {_format_value(projector_shadow_info.get('workspace_check_passed'))}",
            f"- no_motion_projector_passed: {_format_value(projector_shadow_info.get('no_motion_projector_passed'))}",
            f"- blocking_reasons: {_format_value(projector_shadow_info.get('blocking_reasons'))}",
            f"- warnings: {_format_value(projector_shadow_info.get('warnings'))}",
            f"- next_safe_action: {_format_value(projector_shadow_info.get('next_safe_action'))}",
            f"- live_camera_used: {_format_value(projector_shadow_info.get('live_camera_used'))}",
            f"- live_vlm_called: {_format_value(projector_shadow_info.get('live_vlm_called'))}",
            f"- real_robot_motion_executed: {_format_value(projector_shadow_info.get('real_robot_motion_executed'))}",
            f"- real_robot_command_enabled: {_format_value(projector_shadow_info.get('real_robot_command_enabled'))}",
            f"- robot_command_generated: {_format_value(projector_shadow_info.get('robot_command_generated'))}",
            f"- trajectory_generated: {_format_value(projector_shadow_info.get('trajectory_generated'))}",
            f"- joint_targets_generated: {_format_value(projector_shadow_info.get('joint_targets_generated'))}",
            f"- tcp_pose_world_generated: {_format_value(projector_shadow_info.get('tcp_pose_world_generated'))}",
            "This V2.9.2 projector shadow evidence converts declared pixel_center, depth, camera intrinsics, and mock/config transform into camera_point_m and world_point_m for V3.0 preparation. It is no-motion, no-live-camera, no-live-VLM, no-real-robot, and no-ROS2-TF evidence only; it is not live camera, not live VLM, not ROS2 tf2, not MoveIt planning, not real UR5 execution, and it does not generate trajectory, tcp_pose_world, URScript, joint targets, or robot commands.",
            "",
            "## Real-Scene Shadow Pipeline Summary",
            "",
            f"- real_scene_shadow_evidence_available: {_format_value(real_scene_shadow_info.get('requested'))}",
            f"- snapshot_id: {_format_value(real_scene_shadow_info.get('snapshot_id'))}",
            f"- grounding_id: {_format_value(real_scene_shadow_info.get('grounding_id'))}",
            f"- scene_version: {_format_value(real_scene_shadow_info.get('scene_version'))}",
            f"- shadow_pipeline_status: {_format_value(real_scene_shadow_info.get('shadow_pipeline_status'))}",
            f"- semantic_gate_passed: {_format_value(real_scene_shadow_info.get('semantic_gate_passed'))}",
            f"- no_motion_shadow_passed: {_format_value(real_scene_shadow_info.get('no_motion_shadow_passed'))}",
            f"- blocking_reasons: {_format_value(real_scene_shadow_info.get('blocking_reasons'))}",
            f"- warnings: {_format_value(real_scene_shadow_info.get('warnings'))}",
            f"- next_safe_action: {_format_value(real_scene_shadow_info.get('next_safe_action'))}",
            f"- replay_ready: {_format_value(real_scene_shadow_info.get('replay_ready'))}",
            f"- live_camera_used: {_format_value(real_scene_shadow_info.get('live_camera_used'))}",
            f"- live_vlm_called: {_format_value(real_scene_shadow_info.get('live_vlm_called'))}",
            f"- real_robot_motion_executed: {_format_value(real_scene_shadow_info.get('real_robot_motion_executed'))}",
            f"- real_robot_command_enabled: {_format_value(real_scene_shadow_info.get('real_robot_command_enabled'))}",
            f"- robot_command_generated: {_format_value(real_scene_shadow_info.get('robot_command_generated'))}",
            f"- trajectory_generated: {_format_value(real_scene_shadow_info.get('trajectory_generated'))}",
            f"- joint_targets_generated: {_format_value(real_scene_shadow_info.get('joint_targets_generated'))}",
            f"- tcp_pose_world_generated: {_format_value(real_scene_shadow_info.get('tcp_pose_world_generated'))}",
            "This V2.9.0 real-scene shadow pipeline joins camera snapshot evidence with offline/mock grounding only. It does not capture live camera frames, call live Qwen/VLM, connect to a real UR5, use ROS2, MoveIt, RTDE, URScript, Dashboard, generate trajectory, execute tcp_pose_world, or produce robot commands.",
            "",
            "## Full Perception Shadow Pipeline Summary",
            "",
            f"- perception_shadow_evidence_available: {_format_value(perception_shadow_info.get('requested'))}",
            f"- perception_shadow_status: {_format_value(perception_shadow_info.get('perception_shadow_status'))}",
            f"- user_command: {_format_value(perception_shadow_info.get('user_command'))}",
            f"- normalized_command: {_format_value(perception_shadow_info.get('normalized_command'))}",
            f"- snapshot_id: {_format_value(perception_shadow_info.get('snapshot_id'))}",
            f"- grounding_id: {_format_value(perception_shadow_info.get('grounding_id'))}",
            f"- scene_version: {_format_value(perception_shadow_info.get('scene_version'))}",
            f"- camera_source_status: {_format_value(perception_shadow_info.get('camera_source_status'))}",
            f"- camera_snapshot_validity_status: {_format_value(perception_shadow_info.get('camera_snapshot_validity_status'))}",
            f"- vlm_grounding_status: {_format_value(perception_shadow_info.get('vlm_grounding_status'))}",
            f"- real_scene_shadow_status: {_format_value(perception_shadow_info.get('real_scene_shadow_status'))}",
            f"- semantic_gate_passed: {_format_value(perception_shadow_info.get('semantic_gate_passed'))}",
            f"- geometry_validity_status: {_format_value(perception_shadow_info.get('geometry_validity_status'))}",
            f"- projector_status: {_format_value(perception_shadow_info.get('projector_status'))}",
            f"- target_label: {_format_value(perception_shadow_info.get('target_label'))}",
            f"- target_object_id: {_format_value(perception_shadow_info.get('target_object_id'))}",
            f"- bbox_xyxy: {_format_value(perception_shadow_info.get('bbox_xyxy'))}",
            f"- pixel_center: {_format_value(perception_shadow_info.get('pixel_center'))}",
            f"- overall_confidence: {_format_value(perception_shadow_info.get('overall_confidence'))}",
            f"- depth_value_m: {_format_value(perception_shadow_info.get('depth_value_m'))}",
            f"- camera_point_m: {_format_value(perception_shadow_info.get('camera_point_m'))}",
            f"- world_point_m: {_format_value(perception_shadow_info.get('world_point_m'))}",
            f"- workspace_check_passed: {_format_value(perception_shadow_info.get('workspace_check_passed'))}",
            f"- replay_ready: {_format_value(perception_shadow_info.get('replay_ready'))}",
            f"- no_motion_perception_passed: {_format_value(perception_shadow_info.get('no_motion_perception_passed'))}",
            f"- blocking_reasons: {_format_value(perception_shadow_info.get('blocking_reasons'))}",
            f"- warnings: {_format_value(perception_shadow_info.get('warnings'))}",
            f"- next_safe_action: {_format_value(perception_shadow_info.get('next_safe_action'))}",
            f"- live_camera_used: {_format_value(perception_shadow_info.get('live_camera_used'))}",
            f"- live_vlm_called: {_format_value(perception_shadow_info.get('live_vlm_called'))}",
            f"- real_robot_motion_executed: {_format_value(perception_shadow_info.get('real_robot_motion_executed'))}",
            f"- real_robot_command_enabled: {_format_value(perception_shadow_info.get('real_robot_command_enabled'))}",
            f"- robot_command_generated: {_format_value(perception_shadow_info.get('robot_command_generated'))}",
            f"- trajectory_generated: {_format_value(perception_shadow_info.get('trajectory_generated'))}",
            f"- joint_targets_generated: {_format_value(perception_shadow_info.get('joint_targets_generated'))}",
            f"- tcp_pose_world_generated: {_format_value(perception_shadow_info.get('tcp_pose_world_generated'))}",
            "This V2.9.5 full perception shadow pipeline composes text command, camera source, camera snapshot, VLM grounding, semantic gate, geometry validity, and 2D-to-3D projector evidence into world_point_m evidence. It is no-motion, no-live-camera, no-live-VLM, no-real-robot, no-ROS2, and no-MoveIt evidence only; it does not generate trajectory, tcp_pose_world, URScript, joint targets, or robot commands.",
            "",
            "## Planner Gateway Shadow Summary",
            "",
            f"- planner_gateway_shadow_evidence_available: {_format_value(planner_gateway_shadow_info.get('requested'))}",
            f"- planner_gateway_shadow_status: {_format_value(planner_gateway_shadow_info.get('planner_gateway_shadow_status'))}",
            f"- gateway_request_id: {_format_value(planner_gateway_shadow_info.get('gateway_request_id'))}",
            f"- task_id: {_format_value(planner_gateway_shadow_info.get('task_id'))}",
            f"- user_command: {_format_value(planner_gateway_shadow_info.get('user_command'))}",
            f"- intent_name: {_format_value(planner_gateway_shadow_info.get('intent_name'))}",
            f"- target_label: {_format_value(planner_gateway_shadow_info.get('target_label'))}",
            f"- snapshot_id: {_format_value(planner_gateway_shadow_info.get('snapshot_id'))}",
            f"- grounding_id: {_format_value(planner_gateway_shadow_info.get('grounding_id'))}",
            f"- scene_version: {_format_value(planner_gateway_shadow_info.get('scene_version'))}",
            f"- world_frame: {_format_value(planner_gateway_shadow_info.get('world_frame'))}",
            f"- world_point_m: {_format_value(planner_gateway_shadow_info.get('world_point_m'))}",
            f"- bounded_target_point_m: {_format_value(planner_gateway_shadow_info.get('bounded_target_point_m'))}",
            f"- hover_offset_m: {_format_value(planner_gateway_shadow_info.get('hover_offset_m'))}",
            f"- workspace_check_passed: {_format_value(planner_gateway_shadow_info.get('workspace_check_passed'))}",
            f"- confidence_check_passed: {_format_value(planner_gateway_shadow_info.get('confidence_check_passed'))}",
            f"- planner_input_ready: {_format_value(planner_gateway_shadow_info.get('planner_input_ready'))}",
            f"- manual_confirmation_required: {_format_value(planner_gateway_shadow_info.get('manual_confirmation_required'))}",
            f"- execution_allowed: {_format_value(planner_gateway_shadow_info.get('execution_allowed'))}",
            f"- ros2_publish_enabled: {_format_value(planner_gateway_shadow_info.get('ros2_publish_enabled'))}",
            f"- ros2_publish_attempted: {_format_value(planner_gateway_shadow_info.get('ros2_publish_attempted'))}",
            f"- moveit_called: {_format_value(planner_gateway_shadow_info.get('moveit_called'))}",
            f"- trajectory_generated: {_format_value(planner_gateway_shadow_info.get('trajectory_generated'))}",
            f"- tcp_pose_world_generated: {_format_value(planner_gateway_shadow_info.get('tcp_pose_world_generated'))}",
            f"- joint_targets_generated: {_format_value(planner_gateway_shadow_info.get('joint_targets_generated'))}",
            f"- robot_command_generated: {_format_value(planner_gateway_shadow_info.get('robot_command_generated'))}",
            f"- real_robot_motion_executed: {_format_value(planner_gateway_shadow_info.get('real_robot_motion_executed'))}",
            f"- blocking_reasons: {_format_value(planner_gateway_shadow_info.get('blocking_reasons'))}",
            f"- warnings: {_format_value(planner_gateway_shadow_info.get('warnings'))}",
            f"- next_safe_action: {_format_value(planner_gateway_shadow_info.get('next_safe_action'))}",
            f"- replay_ready: {_format_value(planner_gateway_shadow_info.get('replay_ready'))}",
            "This V2.10.0 Planner Gateway Shadow Contract converts perception shadow world_point_m into bounded planner input evidence only. It is no-ROS2-publish, no-MoveIt, no-real-robot, and no-trajectory evidence; it does not generate tcp_pose_world, URScript, joint targets, or robot commands.",
            "",
            "## ROS2 Interface Readiness Summary",
            "",
            f"- ros2_interface_readiness_evidence_available: {_format_value(ros2_interface_readiness_info.get('requested'))}",
            f"- ros2_interface_readiness_status: {_format_value(ros2_interface_readiness_info.get('ros2_interface_readiness_status'))}",
            f"- ros2_environment_declared: {_format_value(ros2_interface_readiness_info.get('ros2_environment_declared'))}",
            f"- ros_distro: {_format_value(ros2_interface_readiness_info.get('ros_distro'))}",
            f"- ros_domain_id: {_format_value(ros2_interface_readiness_info.get('ros_domain_id'))}",
            f"- planner_gateway_interface_mode: {_format_value(ros2_interface_readiness_info.get('planner_gateway_interface_mode'))}",
            f"- planner_gateway_endpoint: {_format_value(ros2_interface_readiness_info.get('planner_gateway_endpoint'))}",
            f"- message_schema: {_format_value(ros2_interface_readiness_info.get('message_schema'))}",
            f"- world_frame: {_format_value(ros2_interface_readiness_info.get('world_frame'))}",
            f"- robot_base_frame: {_format_value(ros2_interface_readiness_info.get('robot_base_frame'))}",
            f"- camera_frame: {_format_value(ros2_interface_readiness_info.get('camera_frame'))}",
            f"- shadow_only: {_format_value(ros2_interface_readiness_info.get('shadow_only'))}",
            f"- ros2_publish_enabled: {_format_value(ros2_interface_readiness_info.get('ros2_publish_enabled'))}",
            f"- ros2_publish_attempted: {_format_value(ros2_interface_readiness_info.get('ros2_publish_attempted'))}",
            f"- moveit_enabled: {_format_value(ros2_interface_readiness_info.get('moveit_enabled'))}",
            f"- moveit_called: {_format_value(ros2_interface_readiness_info.get('moveit_called'))}",
            f"- execution_allowed: {_format_value(ros2_interface_readiness_info.get('execution_allowed'))}",
            f"- trajectory_generated: {_format_value(ros2_interface_readiness_info.get('trajectory_generated'))}",
            f"- tcp_pose_world_generated: {_format_value(ros2_interface_readiness_info.get('tcp_pose_world_generated'))}",
            f"- joint_targets_generated: {_format_value(ros2_interface_readiness_info.get('joint_targets_generated'))}",
            f"- robot_command_generated: {_format_value(ros2_interface_readiness_info.get('robot_command_generated'))}",
            f"- real_robot_motion_executed: {_format_value(ros2_interface_readiness_info.get('real_robot_motion_executed'))}",
            f"- blocking_reasons: {_format_value(ros2_interface_readiness_info.get('blocking_reasons'))}",
            f"- warnings: {_format_value(ros2_interface_readiness_info.get('warnings'))}",
            "This V2.10.1 ROS2 Environment / Interface Readiness Check prepares a future shadow bridge declaration only. It does not publish ROS2 messages, call MoveIt, connect to a real UR5, or generate robot motion commands.",
            "",
            "## ROS2 Message Export / Fake Publish Summary",
            "",
            f"- ros2_message_export_evidence_available: {_format_value(ros2_message_export_info.get('requested'))}",
            f"- ros2_message_export_status: {_format_value(ros2_message_export_info.get('ros2_message_export_status'))}",
            f"- message_export_status: {_format_value(ros2_message_export_info.get('message_export_status'))}",
            f"- message_id: {_format_value(ros2_message_export_info.get('message_id'))}",
            f"- message_schema: {_format_value(ros2_message_export_info.get('message_schema'))}",
            f"- fake_publish_only: {_format_value(ros2_message_export_info.get('fake_publish_only'))}",
            f"- ros2_publish_enabled: {_format_value(ros2_message_export_info.get('ros2_publish_enabled'))}",
            f"- ros2_publish_attempted: {_format_value(ros2_message_export_info.get('ros2_publish_attempted'))}",
            f"- planner_gateway_interface_mode: {_format_value(ros2_message_export_info.get('planner_gateway_interface_mode'))}",
            f"- planner_gateway_endpoint: {_format_value(ros2_message_export_info.get('planner_gateway_endpoint'))}",
            f"- bounded_target_point_m: {_format_value(ros2_message_export_info.get('bounded_target_point_m'))}",
            f"- world_frame: {_format_value(ros2_message_export_info.get('world_frame'))}",
            f"- robot_base_frame: {_format_value(ros2_message_export_info.get('robot_base_frame'))}",
            f"- camera_frame: {_format_value(ros2_message_export_info.get('camera_frame'))}",
            f"- execution_allowed: {_format_value(ros2_message_export_info.get('execution_allowed'))}",
            f"- moveit_called: {_format_value(ros2_message_export_info.get('moveit_called'))}",
            f"- trajectory_generated: {_format_value(ros2_message_export_info.get('trajectory_generated'))}",
            f"- tcp_pose_world_generated: {_format_value(ros2_message_export_info.get('tcp_pose_world_generated'))}",
            f"- joint_targets_generated: {_format_value(ros2_message_export_info.get('joint_targets_generated'))}",
            f"- robot_command_generated: {_format_value(ros2_message_export_info.get('robot_command_generated'))}",
            f"- real_robot_motion_executed: {_format_value(ros2_message_export_info.get('real_robot_motion_executed'))}",
            f"- blocking_reasons: {_format_value(ros2_message_export_info.get('blocking_reasons'))}",
            f"- warnings: {_format_value(ros2_message_export_info.get('warnings'))}",
            "This V2.10.2 fake-publish export writes deterministic PlannerRequest JSON evidence only. It does not publish ROS2 messages, call MoveIt, generate trajectory, or control a real UR5.",
            "",
            "## Readiness Evidence Summary",
            "",
            f"- readiness_evidence_available: {_format_value(lab_readiness_info.get('requested'))}",
            f"- lab_backend: {_format_value(_readiness_statuses(lab_readiness_info).get('lab_backend'))}",
            f"- camera: {_format_value(_readiness_statuses(lab_readiness_info).get('camera'))}",
            f"- live_vlm: {_format_value(_readiness_statuses(lab_readiness_info).get('live_vlm'))}",
            f"- shadow_mode: {_format_value(_readiness_statuses(lab_readiness_info).get('shadow_mode'))}",
            f"- no_motion_readiness_passed: {_format_value(lab_readiness_info.get('no_motion_readiness_passed'))}",
            "",
            "## Lab / Camera / VLM No-Motion Readiness Summary",
            "",
            f"- lab_backend_readiness_status: {_format_value(lab_readiness_info.get('lab_backend_readiness_status'))}",
            f"- camera_readiness_status: {_format_value(lab_readiness_info.get('camera_readiness_status'))}",
            f"- live_vlm_readiness_status: {_format_value(lab_readiness_info.get('live_vlm_readiness_status'))}",
            f"- shadow_mode_readiness_status: {_format_value(lab_readiness_info.get('shadow_mode_readiness_status'))}",
            f"- allow_robot_motion: {_format_value(lab_readiness_info.get('allow_robot_motion'))}",
            f"- allow_live_camera: {_format_value(lab_readiness_info.get('allow_live_camera'))}",
            f"- allow_live_vlm: {_format_value(lab_readiness_info.get('allow_live_vlm'))}",
            f"- real_robot_command_enabled: {_format_value(lab_readiness_info.get('real_robot_command_enabled'))}",
            f"- real_robot_motion_executed: {_format_value(lab_readiness_info.get('real_robot_motion_executed', False))}",
            f"- live_camera_used: {_format_value(lab_readiness_info.get('live_camera_used', False))}",
            f"- live_vlm_called: {_format_value(lab_readiness_info.get('live_vlm_called', False))}",
            f"- blocking_reasons: {_format_value(lab_readiness_info.get('blocking_reasons'))}",
            f"- warnings: {_format_value(lab_readiness_info.get('warnings'))}",
            f"- next_safe_action: {_format_value(lab_readiness_info.get('next_safe_action'))}",
            "This V2.8.1 readiness evidence is shadow-mode/no-motion preparation only because it only checks declared config fields and exports evidence. It does not connect to a real UR5, ROS2, MoveIt, RTDE, URScript, Dashboard, a trajectory planner, tcp_pose_world execution, live camera capture, or live VLM/Qwen.",
            "",
            "## Micro-Motion Evidence Summary",
            "",
            f"- requested: {_format_value(simulation_micro_motion_info.get('requested'))}",
            f"- simulation_micro_motion_status: {_format_value(simulation_micro_motion_info.get('simulation_micro_motion_status'))}",
            f"- simulation_only: {_format_value(simulation_micro_motion_info.get('simulation_only'))}",
            f"- real_robot_allowed: {_format_value(simulation_micro_motion_info.get('real_robot_allowed'))}",
            f"- real_robot_motion_executed: {_format_value(simulation_micro_motion_info.get('real_robot_motion_executed'))}",
            f"- robot_motion_executed: {_format_value(simulation_micro_motion_info.get('robot_motion_executed'))}",
            f"- motion_evidence_available: {_format_value(simulation_micro_motion_info.get('motion_evidence_available'))}",
            f"- joint_name: {_format_value((simulation_micro_motion_info.get('motion') or {}).get('joint_name'))}",
            f"- before_joint_position_rad: {_format_value((simulation_micro_motion_info.get('motion') or {}).get('before_joint_position_rad'))}",
            f"- after_joint_position_rad: {_format_value((simulation_micro_motion_info.get('motion') or {}).get('after_joint_position_rad'))}",
            f"- requested_delta_rad: {_format_value((simulation_micro_motion_info.get('motion') or {}).get('requested_delta_rad'))}",
            f"- actual_delta_rad: {_format_value((simulation_micro_motion_info.get('motion') or {}).get('actual_delta_rad'))}",
            f"- tolerance_rad: {_format_value((simulation_micro_motion_info.get('motion') or {}).get('tolerance_rad'))}",
            f"- delta_within_tolerance: {_format_value((simulation_micro_motion_info.get('motion') or {}).get('delta_within_tolerance'))}",
            f"- before_joint_state_path: {_format_value((simulation_micro_motion_info.get('motion') or {}).get('before_joint_state_path'))}",
            f"- after_joint_state_path: {_format_value((simulation_micro_motion_info.get('motion') or {}).get('after_joint_state_path'))}",
            f"- simulation_motion_result_path: {_format_value((simulation_micro_motion_info.get('motion') or {}).get('simulation_motion_result_path'))}",
            f"- simulation_motion_report_path: {_format_value((simulation_micro_motion_info.get('motion') or {}).get('simulation_motion_report_path'))}",
            f"- motion_evidence_files: {_format_value(simulation_micro_motion_info.get('motion_evidence_files'))}",
            "",
            "## Semantic-to-Simulation Bridge Summary",
            "",
            f"- semantic_bridge_status: {_format_value(semantic_bridge_info.get('status'))}",
            f"- semantic_task_id: {_format_value(semantic_bridge_info.get('semantic_task_id'))}",
            f"- semantic_intent: {_format_value(semantic_bridge_info.get('semantic_intent'))}",
            f"- semantic_target_label: {_format_value(semantic_bridge_info.get('semantic_target_label'))}",
            f"- semantic_gate_passed: {_format_value(semantic_bridge_info.get('gate_passed'))}",
            f"- triggered_simulation_micro_motion: {_format_value(semantic_bridge_info.get('triggered_simulation_micro_motion'))}",
            f"- simulation_micro_motion_status: {_format_value(simulation_micro_motion_info.get('simulation_micro_motion_status'))}",
            f"- actual_delta_rad: {_format_value((simulation_micro_motion_info.get('motion') or {}).get('actual_delta_rad'))}",
            f"- delta_within_tolerance: {_format_value((simulation_micro_motion_info.get('motion') or {}).get('delta_within_tolerance'))}",
            f"- semantic_bridge_files: {_format_value(_semantic_bridge_files(semantic_bridge_info) if semantic_bridge_info.get('requested') else [])}",
            "",
            "## Safe Simulated Task Execution Summary",
            "",
            "### Human-readable conclusion",
            "",
            _human_readable_execution_conclusion(
                simulated_task_execution_info,
                simulation_micro_motion_info,
                semantic_bridge_info,
            ),
            "",
            f"- execution_attempt_id: {_format_value(simulated_task_execution_info.get('execution_attempt_id'))}",
            f"- simulated_task_status: {_format_value(simulated_task_execution_info.get('simulated_task_status'))}",
            f"- semantic_bridge_status: {_format_value(semantic_bridge_info.get('status'))}",
            f"- semantic_gate_passed: {_format_value(semantic_bridge_info.get('gate_passed'))}",
            f"- simulation_motion_precheck_status: {_format_value(simulation_motion_precheck_info.get('status'))}",
            f"- simulation_micro_motion_status: {_format_value(simulation_micro_motion_info.get('simulation_micro_motion_status'))}",
            f"- post_motion_state_check_status: {_format_value((simulated_task_execution_info.get('post_motion_state_check') or {}).get('post_motion_state_check_status'))}",
            f"- execution_feedback_status: {_format_value(simulated_task_execution_info.get('execution_feedback_status'))}",
            f"- failure_reason: {_format_value(simulated_task_execution_info.get('failure_reason'))}",
            f"- retry_recommended: {_format_value(simulated_task_execution_info.get('retry_recommended'))}",
            f"- fallback_recommended: {_format_value(simulated_task_execution_info.get('fallback_recommended'))}",
            f"- fallback_type: {_format_value(simulated_task_execution_info.get('fallback_type'))}",
            f"- replay_ready: {_format_value(simulated_task_execution_info.get('replay_ready'))}",
            "",
            "### Evidence Files",
            "",
            *[
                f"- {item.get('name')}: {_format_value(item.get('path'))}"
                for item in (
                    _simulated_task_execution_files(simulated_task_execution_info)
                    if simulated_task_execution_info.get("safe_simulated_task_execution_requested")
                    else []
                )
            ],
            "",
        ]
    )


def _human_readable_execution_conclusion(
    execution_info: Dict[str, Any],
    micro_motion_info: Dict[str, Any],
    semantic_bridge_info: Dict[str, Any],
) -> str:
    status = execution_info.get("simulated_task_status")
    fallback_type = execution_info.get("fallback_type")
    if status == "SUCCEEDED":
        return (
            "SUCCEEDED: semantic gate and simulation precheck passed, the local Isaac Sim "
            "micro-motion evidence was verified, and the replay-ready bundle is complete."
        )
    if status == "DRY_RUN_ONLY":
        return (
            "DRY_RUN_ONLY: this run generated replay-ready dry-run evidence only; it does not "
            "claim a true Isaac joint state change or real robot motion."
        )
    if status == "BLOCKED_BY_SEMANTIC_GATE":
        return (
            "BLOCKED_BY_SEMANTIC_GATE: the semantic gate blocked execution before motion; "
            f"fallback_type={_format_value(fallback_type)}."
        )
    if status == "BLOCKED_BY_PRECHECK":
        return (
            "BLOCKED_BY_PRECHECK: simulation motion precheck was not ready, so no simulated "
            "micro-motion should be treated as accepted evidence."
        )
    if status in {"MOTION_FAILED", "POST_CHECK_FAILED"}:
        return (
            f"{_format_value(status)}: execution evidence was incomplete or outside tolerance; "
            "manual review is required before any further simulated attempt."
        )
    if semantic_bridge_info.get("requested") or micro_motion_info.get("requested"):
        return "Execution evidence is present but did not reach a recognized accepted status."
    return "Safe simulated task execution was not requested for this run."


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
