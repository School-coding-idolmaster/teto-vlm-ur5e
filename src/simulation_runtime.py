from __future__ import annotations

import json
import time
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Any, Dict

from src.articulation_readiness_contract import build_articulation_readiness_report
from src.articulation_state_observer import build_articulation_state_report, observe_articulation_state as observe_articulation_state_in_world
from src.camera_snapshot import build_camera_snapshot_request, evaluate_camera_snapshot_contract
from src.evidence_exporter import export_simulation_evidence
from src.lab_readiness import build_lab_readiness_request, evaluate_lab_readiness
from src.real_scene_shadow_pipeline import (
    build_real_scene_shadow_request,
    evaluate_real_scene_shadow_pipeline,
)
from src.robot_prim_inspector import (
    UR5E_ARM_JOINT_NAMES,
    build_robot_prim_inspection_report,
    inspect_robot_prim as inspect_robot_prim_in_stage,
)
from src.simulation_micro_motion import (
    DEFAULT_MICRO_MOTION_DELTA_RAD,
    DEFAULT_MICRO_MOTION_JOINT,
    DEFAULT_MICRO_MOTION_TOLERANCE_RAD,
    MICRO_MOTION_STATUS_BLOCKED_BY_PRECHECK,
    MICRO_MOTION_STATUS_FAILED,
    MICRO_MOTION_STATUS_NOT_REQUESTED,
    MICRO_MOTION_STATUS_OK,
    SimulationMicroMotionRequest,
    execute_simulation_micro_motion,
    summarize_motion_evidence,
)
from src.simulation_motion_precheck import build_simulation_motion_precheck_report
from src.semantic_simulation_bridge import (
    DEFAULT_SEMANTIC_CONFIDENCE_THRESHOLD,
    SEMANTIC_BRIDGE_STATUS_BLOCKED_BY_PRECHECK,
    SEMANTIC_BRIDGE_STATUS_FAILED,
    SEMANTIC_BRIDGE_STATUS_NOT_REQUESTED,
    SemanticSimulationBridgeRequest,
    build_demo_semantic_task_contract,
    build_semantic_simulation_bridge_result,
    build_simulation_micro_motion_request_from_semantic_contract,
)
from src.simulated_task_execution import (
    SimulatedTaskExecutionRequest,
    execute_safe_simulated_task,
)


REPORT_VERSION = "teto_simulation_execution.v1"
CURRENT_TETO_VERSION = "TETO V2.9.0"
DEFAULT_STEPS = 5
DEFAULT_SIMULATION_OBJECT_TYPE = "cube"
DEFAULT_CUBE_PRIM_PATH = "/World/TETO_Cube"
DEFAULT_CUBE_POSITION = [0.0, 0.0, 0.5]
DEFAULT_CUBE_TARGET_POSITION = [0.3, 0.0, 0.5]
DEFAULT_CUBE_SIZE = 0.2
DEFAULT_ROBOT_TYPE = "ur5"
DEFAULT_ROBOT_PRIM_PATH = "/World/TETO_Robot"
DEFAULT_LOCAL_UR5E_ASSET_PATH = (
    "/home/newusername/Storage/isaac_assets/Isaac/Robots/UniversalRobots/ur5e/ur5e.usd"
)
PROJECT_ROOT = Path(__file__).resolve().parents[1]
SIMULATION_RUNS_ROOT = PROJECT_ROOT / "outputs" / "simulation_runs"
DEFAULT_SIMULATION_TASK = {
    "task_type": "hover_to_object",
    "target_label": "test_target",
    "target_world_point": [0.0, 0.0, 0.1],
    "scene_version": "manual_first_simulation_execution",
    "ttl_ms": 500,
}


@dataclass(frozen=True)
class SimulationObjectSpec:
    object_type: str
    prim_path: str
    initial_position: tuple[float, float, float]
    target_position: tuple[float, float, float]
    size: float | None = None


@dataclass(frozen=True)
class RobotAssetSpec:
    robot_type: str = DEFAULT_ROBOT_TYPE
    robot_prim_path: str = DEFAULT_ROBOT_PRIM_PATH
    robot_asset_path: str | None = None
    asset_source: str = "unavailable"
    expected_asset_kind: str = "usd/usda/usdc"
    allow_network_asset: bool = False


def build_simulation_execution_result(
    *,
    simulation_task: Dict[str, Any] | None = None,
    status: str,
    mode: str,
    steps_requested: int = DEFAULT_STEPS,
    steps_completed: int = 0,
    world_reset: bool = False,
    error_code: str = "OK",
    error_message: str = "",
    object_metadata: Dict[str, Any] | None = None,
    robot_asset_metadata: Dict[str, Any] | None = None,
    robot_prim_inspection_metadata: Dict[str, Any] | None = None,
    articulation_readiness_metadata: Dict[str, Any] | None = None,
    articulation_state_metadata: Dict[str, Any] | None = None,
    simulation_motion_precheck_metadata: Dict[str, Any] | None = None,
    simulation_micro_motion_metadata: Dict[str, Any] | None = None,
    semantic_bridge_metadata: Dict[str, Any] | None = None,
    simulated_task_execution_metadata: Dict[str, Any] | None = None,
    lab_readiness_metadata: Dict[str, Any] | None = None,
    camera_snapshot_metadata: Dict[str, Any] | None = None,
    real_scene_shadow_metadata: Dict[str, Any] | None = None,
    started_at: str | None = None,
    finished_at: str | None = None,
) -> Dict[str, Any]:
    blocking_reasons = []
    if status != "PASS":
        blocking_reasons.append(error_code if error_code != "OK" else "E_SIMULATION_EXECUTION_FAILED")

    result = {
        "report_version": REPORT_VERSION,
        "teto_version": CURRENT_TETO_VERSION,
        "version": CURRENT_TETO_VERSION,
        "status": status,
        "ok": status == "PASS",
        "mode": mode,
        "dry_run": mode == "dry_run",
        "isaac_runtime_used": mode == "isaac",
        "allow_robot_motion": False,
        "consumed_simulation_task": isinstance(simulation_task, dict),
        "simulation_task": simulation_task,
        "world_reset": world_reset,
        "steps_requested": steps_requested,
        "steps_completed": steps_completed,
        "blocking_reasons": blocking_reasons,
        "error": {
            "code": error_code,
            "message": error_message,
        },
        "started_at": started_at,
        "finished_at": finished_at,
    }
    result.update(object_metadata or _simulation_object_report_fields())
    result.update(robot_asset_metadata or _robot_asset_report_fields())
    result.update(robot_prim_inspection_metadata or _robot_prim_inspection_report_fields())
    result.update(articulation_readiness_metadata or _articulation_readiness_report_fields())
    result.update(articulation_state_metadata or _articulation_state_report_fields())
    result.update(simulation_motion_precheck_metadata or _simulation_motion_precheck_report_fields())
    result.update(simulation_micro_motion_metadata or _simulation_micro_motion_report_fields())
    result.update(semantic_bridge_metadata or _semantic_bridge_report_fields())
    result.update(lab_readiness_metadata or _lab_readiness_report_fields())
    result.update(camera_snapshot_metadata or _camera_snapshot_report_fields())
    result.update(real_scene_shadow_metadata or _real_scene_shadow_report_fields())
    if (
        result.get("semantic_simulation_bridge_requested") is True
        and result.get("semantic_gate_passed") is not True
        and result.get("simulation_micro_motion_requested") is not True
    ):
        result["simulation_micro_motion_status"] = "BLOCKED_BY_SEMANTIC_GATE"
    result["semantic_bridge"] = _semantic_bridge_info(result)
    result["motion"] = _motion_info(result)
    result["precheck"] = _precheck_info(result)
    result["safety"] = _safety_report_fields(result)
    result.update(_simulated_task_execution_report_fields(result, simulated_task_execution_metadata))
    result["simulated_task_execution"] = _simulated_task_execution_info(result)
    result["post_motion_state_check"] = result["simulated_task_execution"].get("post_motion_state_check", {})
    result.setdefault("robot_structure_report_generated", False)
    result.setdefault("robot_structure_report_path", None)
    result.setdefault("articulation_readiness_report_generated", False)
    result.setdefault("articulation_readiness_path", None)
    result.setdefault("articulation_state_report_generated", False)
    result.setdefault("articulation_state_path", None)
    result.setdefault("articulation_state_report_path", None)
    result.setdefault("simulation_motion_precheck_report_generated", False)
    result.setdefault("simulation_motion_precheck_path", None)
    result.setdefault("simulation_motion_precheck_report_path", None)
    result.setdefault("simulation_motion_result_path", None)
    result.setdefault("simulation_motion_report_path", None)
    result.setdefault("before_joint_state_path", None)
    result.setdefault("after_joint_state_path", None)
    result.setdefault("motion_evidence_available", False)
    result.setdefault("motion_evidence_files", [])
    result.setdefault("motion_diff_summary", {})
    result.setdefault("robot_motion_executed", False)
    result.setdefault("real_robot_motion_executed", False)
    result.setdefault("control_enabled", False)
    result.setdefault("motion_generated", False)
    result.setdefault("command_generated", False)
    return result


def run_first_simulation_execution(
    simulation_task: Dict[str, Any] | None = None,
    *,
    dry_run: bool = False,
    no_isaac: bool = False,
    steps: int = DEFAULT_STEPS,
    headless: bool = True,
    spawn_cube: bool = False,
    move_object: bool = False,
    move_cube: bool = False,
    cube_prim_path: str = DEFAULT_CUBE_PRIM_PATH,
    cube_position: list[float] | tuple[float, float, float] = tuple(DEFAULT_CUBE_POSITION),
    cube_target_position: list[float] | tuple[float, float, float] = tuple(DEFAULT_CUBE_TARGET_POSITION),
    cube_size: float = DEFAULT_CUBE_SIZE,
    object_spec: SimulationObjectSpec | None = None,
    check_robot_asset: bool = False,
    load_robot_asset: bool = False,
    robot_type: str = DEFAULT_ROBOT_TYPE,
    robot_prim_path: str = DEFAULT_ROBOT_PRIM_PATH,
    robot_asset_path: str | None = None,
    robot_asset_spec: RobotAssetSpec | None = None,
    inspect_robot_prim: bool = False,
    check_articulation_readiness: bool = False,
    observe_articulation_state: bool = False,
    check_simulation_motion_precheck: bool = False,
    execute_simulation_micro_motion: bool = False,
    micro_motion_joint: str = DEFAULT_MICRO_MOTION_JOINT,
    micro_motion_delta_rad: float = DEFAULT_MICRO_MOTION_DELTA_RAD,
    micro_motion_tolerance_rad: float = DEFAULT_MICRO_MOTION_TOLERANCE_RAD,
    semantic_simulation_bridge: bool = False,
    semantic_task_contract: Dict[str, Any] | None = None,
    semantic_task_contract_path: str | None = None,
    semantic_confidence_threshold: float = DEFAULT_SEMANTIC_CONFIDENCE_THRESHOLD,
    safe_simulated_task_execution: bool = False,
    execution_attempt_id: str | None = None,
    execution_max_attempts: int = 1,
    execution_enable_retry_recommendation: bool = False,
    execution_enable_fallback_recommendation: bool = False,
    check_lab_readiness: bool = False,
    lab_readiness_config: str | Path | None = None,
    check_camera_readiness: bool = False,
    check_live_vlm_readiness: bool = False,
    check_shadow_mode_readiness: bool = False,
    check_camera_snapshot: bool = False,
    camera_snapshot_config: str | Path | None = None,
    camera_snapshot_report: bool = False,
    run_real_scene_shadow: bool = False,
    real_scene_shadow_config: str | Path | None = None,
    grounding_result: str | Path | None = None,
    real_scene_shadow_report: bool = False,
    output_dir: str | Path | None = None,
    write_report: bool = False,
    demo_command: str | None = None,
) -> Dict[str, Any]:
    task = simulation_task or dict(DEFAULT_SIMULATION_TASK)
    started_at = _timestamp()
    camera_snapshot_requested = check_camera_snapshot or camera_snapshot_report
    real_scene_shadow_requested = run_real_scene_shadow or real_scene_shadow_report
    lab_readiness_requested = (
        check_lab_readiness
        or check_camera_readiness
        or check_live_vlm_readiness
        or check_shadow_mode_readiness
    )
    if (lab_readiness_requested or camera_snapshot_requested or real_scene_shadow_requested) and not dry_run:
        no_isaac = True
    lab_readiness_request = build_lab_readiness_request(
        check_lab_backend=check_lab_readiness,
        check_camera=check_camera_readiness,
        check_live_vlm=check_live_vlm_readiness,
        check_shadow_mode=check_shadow_mode_readiness,
        config_path=lab_readiness_config,
    )
    lab_readiness_metadata = _lab_readiness_report_fields(
        requested=lab_readiness_requested,
        readiness=evaluate_lab_readiness(lab_readiness_request),
    )
    camera_snapshot_request = build_camera_snapshot_request(
        requested=camera_snapshot_requested,
        config_path=camera_snapshot_config,
    )
    camera_snapshot_metadata = _camera_snapshot_report_fields(
        requested=camera_snapshot_requested,
        snapshot=evaluate_camera_snapshot_contract(camera_snapshot_request),
    )
    real_scene_shadow_request = build_real_scene_shadow_request(
        requested=real_scene_shadow_requested,
        config_path=real_scene_shadow_config,
        grounding_result_path=grounding_result,
    )
    real_scene_shadow_metadata = _real_scene_shadow_report_fields(
        requested=real_scene_shadow_requested,
        shadow=evaluate_real_scene_shadow_pipeline(real_scene_shadow_request),
    )
    effective_move_object = move_object or move_cube
    effective_spawn_cube = spawn_cube or effective_move_object
    effective_semantic_bridge = semantic_simulation_bridge or safe_simulated_task_execution
    bridge_contract = semantic_task_contract if effective_semantic_bridge else None
    if effective_semantic_bridge and bridge_contract is None:
        bridge_contract = build_demo_semantic_task_contract()
    simulated_task_execution_metadata = _simulated_task_execution_request_fields(
        requested=safe_simulated_task_execution,
        execution_attempt_id=execution_attempt_id,
        execution_max_attempts=execution_max_attempts,
        retry_enabled=execution_enable_retry_recommendation,
        fallback_enabled=execution_enable_fallback_recommendation,
    )
    semantic_bridge_metadata = _semantic_bridge_report_fields(
        requested=effective_semantic_bridge,
        contract=bridge_contract,
        contract_path=semantic_task_contract_path,
        confidence_threshold=semantic_confidence_threshold,
        joint_name=micro_motion_joint,
        requested_delta_rad=micro_motion_delta_rad,
        tolerance_rad=micro_motion_tolerance_rad,
    )
    semantic_gate_passed = bool(semantic_bridge_metadata.get("semantic_gate_passed"))
    execute_micro_motion_requested = execute_simulation_micro_motion or semantic_gate_passed
    effective_inspect_robot_prim = inspect_robot_prim or execute_micro_motion_requested
    effective_check_articulation_readiness = check_articulation_readiness or execute_micro_motion_requested
    effective_observe_articulation_state = observe_articulation_state or execute_micro_motion_requested
    effective_check_simulation_motion_precheck = check_simulation_motion_precheck or execute_micro_motion_requested
    micro_motion_request = SimulationMicroMotionRequest(
        joint_name=micro_motion_joint,
        requested_delta_rad=micro_motion_delta_rad,
        tolerance_rad=micro_motion_tolerance_rad,
    )
    if semantic_gate_passed and isinstance(bridge_contract, dict):
        micro_motion_request = build_simulation_micro_motion_request_from_semantic_contract(
            bridge_contract,
            confidence_threshold=semantic_confidence_threshold,
            joint_name=micro_motion_joint,
            requested_delta_rad=micro_motion_delta_rad,
            tolerance_rad=micro_motion_tolerance_rad,
        )
    simulation_object_spec = object_spec or SimulationObjectSpec(
        object_type=DEFAULT_SIMULATION_OBJECT_TYPE,
        prim_path=cube_prim_path,
        initial_position=_position_tuple(cube_position),
        target_position=_position_tuple(cube_target_position),
        size=cube_size,
    )
    resolved_robot_asset_path = _resolve_default_robot_asset_path(
        robot_asset_path,
        dry_run=dry_run,
        no_isaac=no_isaac,
        check_robot_asset=check_robot_asset or execute_micro_motion_requested,
        load_robot_asset=load_robot_asset,
        inspect_robot_prim=effective_inspect_robot_prim,
        check_articulation_readiness=effective_check_articulation_readiness,
        observe_articulation_state=effective_observe_articulation_state,
        check_simulation_motion_precheck=effective_check_simulation_motion_precheck,
    )
    effective_load_robot_asset = load_robot_asset or bool(
        resolved_robot_asset_path
        and not dry_run
        and not no_isaac
        and (check_robot_asset or execute_micro_motion_requested)
        and (
            effective_inspect_robot_prim
            or effective_check_articulation_readiness
            or effective_observe_articulation_state
            or effective_check_simulation_motion_precheck
        )
    )
    effective_check_robot_asset = check_robot_asset or execute_micro_motion_requested or effective_load_robot_asset
    effective_robot_asset_spec = robot_asset_spec or RobotAssetSpec(
        robot_type=robot_type,
        robot_prim_path=robot_prim_path,
        robot_asset_path=resolved_robot_asset_path,
    )

    if steps <= 0:
        return _finalize_result(
            build_simulation_execution_result(
                simulation_task=task,
                status="FAIL",
                mode=_mode_name(dry_run=dry_run, no_isaac=no_isaac),
                steps_requested=steps,
                object_metadata=_simulation_object_report_fields(
                    spec=simulation_object_spec if effective_spawn_cube else None,
                    spawned=False,
                    move_requested=effective_move_object,
                ),
                robot_asset_metadata=_robot_asset_report_fields(
                    spec=effective_robot_asset_spec if effective_check_robot_asset else None,
                    check_requested=effective_check_robot_asset,
                    load_requested=effective_load_robot_asset,
                ),
                robot_prim_inspection_metadata=_robot_prim_inspection_report_fields(
                    spec=effective_robot_asset_spec,
                    requested=effective_inspect_robot_prim,
                ),
                articulation_readiness_metadata=_articulation_readiness_report_fields(
                    spec=effective_robot_asset_spec,
                    requested=effective_check_articulation_readiness,
                ),
                articulation_state_metadata=_articulation_state_report_fields(
                    spec=effective_robot_asset_spec,
                    requested=effective_observe_articulation_state,
                ),
                simulation_motion_precheck_metadata=_simulation_motion_precheck_report_fields(
                    spec=effective_robot_asset_spec,
                    requested=effective_check_simulation_motion_precheck,
                ),
                semantic_bridge_metadata=semantic_bridge_metadata,
                simulated_task_execution_metadata=simulated_task_execution_metadata,
                lab_readiness_metadata=lab_readiness_metadata,
                camera_snapshot_metadata=camera_snapshot_metadata,
                real_scene_shadow_metadata=real_scene_shadow_metadata,
                error_code="E_INVALID_STEPS",
                error_message="steps must be a positive integer",
                started_at=started_at,
                finished_at=_timestamp(),
            ),
            output_dir=output_dir,
            write_report=write_report,
            demo_command=demo_command,
        )

    missing_fields = _missing_task_fields(task)
    if missing_fields:
        return _finalize_result(
            build_simulation_execution_result(
                simulation_task=task,
                status="FAIL",
                mode=_mode_name(dry_run=dry_run, no_isaac=no_isaac),
                steps_requested=steps,
                object_metadata=_simulation_object_report_fields(
                    spec=simulation_object_spec if effective_spawn_cube else None,
                    spawned=False,
                    move_requested=effective_move_object,
                ),
                robot_asset_metadata=_robot_asset_report_fields(
                    spec=effective_robot_asset_spec if effective_check_robot_asset else None,
                    check_requested=effective_check_robot_asset,
                    load_requested=effective_load_robot_asset,
                ),
                robot_prim_inspection_metadata=_robot_prim_inspection_report_fields(
                    spec=effective_robot_asset_spec,
                    requested=effective_inspect_robot_prim,
                ),
                articulation_readiness_metadata=_articulation_readiness_report_fields(
                    spec=effective_robot_asset_spec,
                    requested=effective_check_articulation_readiness,
                ),
                articulation_state_metadata=_articulation_state_report_fields(
                    spec=effective_robot_asset_spec,
                    requested=effective_observe_articulation_state,
                ),
                simulation_motion_precheck_metadata=_simulation_motion_precheck_report_fields(
                    spec=effective_robot_asset_spec,
                    requested=effective_check_simulation_motion_precheck,
                ),
                semantic_bridge_metadata=semantic_bridge_metadata,
                simulated_task_execution_metadata=simulated_task_execution_metadata,
                lab_readiness_metadata=lab_readiness_metadata,
                camera_snapshot_metadata=camera_snapshot_metadata,
                real_scene_shadow_metadata=real_scene_shadow_metadata,
                error_code="E_INVALID_SIMULATION_TASK",
                error_message=f"missing simulation task fields: {', '.join(missing_fields)}",
                started_at=started_at,
                finished_at=_timestamp(),
            ),
            output_dir=output_dir,
            write_report=write_report,
            demo_command=demo_command,
        )

    if dry_run or no_isaac:
        robot_asset_metadata = (
            evaluate_robot_asset_availability(
                effective_robot_asset_spec,
                check_requested=effective_check_robot_asset,
                load_requested=effective_load_robot_asset,
                dry_run=True,
            )
            if effective_check_robot_asset
            else _robot_asset_report_fields()
        )
        robot_prim_inspection_metadata = _dry_run_robot_prim_inspection_report_fields(
            spec=effective_robot_asset_spec,
            requested=effective_inspect_robot_prim,
            prim_exists=bool(robot_asset_metadata.get("robot_prim_exists")),
        )
        articulation_readiness_metadata = _articulation_readiness_report_fields(
            spec=effective_robot_asset_spec,
            requested=effective_check_articulation_readiness,
            inspection=robot_prim_inspection_metadata.get("robot_prim_inspection"),
        )
        articulation_state_metadata = _articulation_state_report_fields(
            spec=effective_robot_asset_spec,
            requested=effective_observe_articulation_state,
            inspection=robot_prim_inspection_metadata.get("robot_prim_inspection"),
            readiness=articulation_readiness_metadata.get("articulation_readiness"),
        )
        simulation_motion_precheck_metadata = _simulation_motion_precheck_report_fields(
            spec=effective_robot_asset_spec,
            requested=effective_check_simulation_motion_precheck,
            robot_asset=robot_asset_metadata,
            inspection=robot_prim_inspection_metadata.get("robot_prim_inspection"),
            readiness=articulation_readiness_metadata.get("articulation_readiness"),
            state=articulation_state_metadata.get("articulation_state"),
        )
        simulation_micro_motion_metadata = _simulation_micro_motion_report_fields(
            request=micro_motion_request,
            requested=execute_micro_motion_requested,
            dry_run=True,
            precheck=simulation_motion_precheck_metadata.get("simulation_motion_precheck"),
            readiness=articulation_readiness_metadata.get("articulation_readiness"),
            before_state=articulation_state_metadata.get("articulation_state"),
        )
        if effective_load_robot_asset and not robot_asset_metadata["robot_asset_available"]:
            return _finalize_result(
                build_simulation_execution_result(
                    simulation_task=task,
                    status="FAIL",
                    mode=_mode_name(dry_run=dry_run, no_isaac=no_isaac),
                    steps_requested=steps,
                    world_reset=True,
                    object_metadata=_simulation_object_report_fields(
                        spec=simulation_object_spec if effective_spawn_cube else None,
                        spawned=effective_spawn_cube,
                        move_requested=effective_move_object,
                        moved=effective_move_object,
                        final_position=simulation_object_spec.target_position if effective_move_object else None,
                    ),
                    robot_asset_metadata=_robot_asset_report_fields(
                        spec=effective_robot_asset_spec,
                        check_requested=effective_check_robot_asset,
                        load_requested=True,
                        available=False,
                        loaded=False,
                        prim_exists=False,
                        status="LOAD_FAILED",
                        blocking_reason="E_ROBOT_ASSET_UNAVAILABLE",
                    ),
                    robot_prim_inspection_metadata=robot_prim_inspection_metadata,
                    articulation_readiness_metadata=articulation_readiness_metadata,
                    articulation_state_metadata=articulation_state_metadata,
                    simulation_motion_precheck_metadata=simulation_motion_precheck_metadata,
                    simulation_micro_motion_metadata=simulation_micro_motion_metadata,
                    semantic_bridge_metadata=semantic_bridge_metadata,
                    simulated_task_execution_metadata=simulated_task_execution_metadata,
                    lab_readiness_metadata=lab_readiness_metadata,
                    camera_snapshot_metadata=camera_snapshot_metadata,
                    real_scene_shadow_metadata=real_scene_shadow_metadata,
                    error_code="E_ROBOT_ASSET_LOAD_FAILED",
                    error_message="robot asset path is unavailable",
                    started_at=started_at,
                    finished_at=_timestamp(),
                ),
                output_dir=output_dir,
                write_report=write_report,
                demo_command=demo_command,
            )
        return _finalize_result(
            build_simulation_execution_result(
                simulation_task=task,
                status="PASS",
                mode=_mode_name(dry_run=dry_run, no_isaac=no_isaac),
                steps_requested=steps,
                steps_completed=steps,
                world_reset=True,
                object_metadata=_simulation_object_report_fields(
                    spec=simulation_object_spec if effective_spawn_cube else None,
                    spawned=effective_spawn_cube,
                    move_requested=effective_move_object,
                    moved=effective_move_object,
                    final_position=simulation_object_spec.target_position if effective_move_object else None,
                ),
                robot_asset_metadata=robot_asset_metadata,
                robot_prim_inspection_metadata=robot_prim_inspection_metadata,
                articulation_readiness_metadata=articulation_readiness_metadata,
                articulation_state_metadata=articulation_state_metadata,
                simulation_motion_precheck_metadata=simulation_motion_precheck_metadata,
                simulation_micro_motion_metadata=simulation_micro_motion_metadata,
                semantic_bridge_metadata=semantic_bridge_metadata,
                simulated_task_execution_metadata=simulated_task_execution_metadata,
                lab_readiness_metadata=lab_readiness_metadata,
                camera_snapshot_metadata=camera_snapshot_metadata,
                real_scene_shadow_metadata=real_scene_shadow_metadata,
                started_at=started_at,
                finished_at=_timestamp(),
            ),
            output_dir=output_dir,
            write_report=write_report,
            demo_command=demo_command,
        )

    return _run_true_isaac_runtime(
        simulation_task=task,
        steps=steps,
        headless=headless,
        spawn_object=effective_spawn_cube,
        move_object=effective_move_object,
        object_spec=simulation_object_spec,
        check_robot_asset=effective_check_robot_asset,
        load_robot_asset=effective_load_robot_asset,
        robot_asset_spec=effective_robot_asset_spec,
        inspect_robot_prim=effective_inspect_robot_prim,
        check_articulation_readiness=effective_check_articulation_readiness,
        observe_articulation_state=effective_observe_articulation_state,
        check_simulation_motion_precheck=effective_check_simulation_motion_precheck,
        execute_simulation_micro_motion=execute_micro_motion_requested,
        micro_motion_request=micro_motion_request,
        semantic_bridge_metadata=semantic_bridge_metadata,
        simulated_task_execution_metadata=simulated_task_execution_metadata,
        started_at=started_at,
        output_dir=output_dir,
        write_report=write_report,
        demo_command=demo_command,
    )


def write_simulation_execution_result(
    result: Dict[str, Any],
    output_dir: str | Path | None = None,
    *,
    demo_command: str | None = None,
) -> Path:
    run_dir = Path(output_dir).expanduser() if output_dir else _create_run_dir()
    run_dir.mkdir(parents=True, exist_ok=True)
    report_path = run_dir / "simulation_execution_result.json"
    structure_report_path = run_dir / "robot_structure_report.md"
    articulation_readiness_path = run_dir / "articulation_readiness.json"
    articulation_state_path = run_dir / "articulation_state.json"
    articulation_state_report_path = run_dir / "articulation_state_report.md"
    simulation_motion_precheck_path = run_dir / "simulation_motion_precheck.json"
    simulation_motion_precheck_report_path = run_dir / "simulation_motion_precheck_report.md"
    simulation_motion_result_path = run_dir / "simulation_motion_result.json"
    simulation_motion_report_path = run_dir / "simulation_motion_report.md"
    before_articulation_state_path = run_dir / "before_articulation_state.json"
    after_articulation_state_path = run_dir / "after_articulation_state.json"
    semantic_bridge_result_path = run_dir / "semantic_simulation_bridge_result.json"
    semantic_bridge_report_path = run_dir / "semantic_simulation_bridge_report.md"
    semantic_task_contract_copy_path = run_dir / "semantic_task_contract_copy.json"
    simulated_task_execution_result_path = run_dir / "simulated_task_execution_result.json"
    simulated_task_execution_report_path = run_dir / "simulated_task_execution_report.md"
    execution_feedback_path = run_dir / "execution_feedback.json"
    execution_attempt_record_path = run_dir / "execution_attempt_record.json"
    failure_analysis_path = run_dir / "failure_analysis.json"
    retry_fallback_recommendation_path = run_dir / "retry_fallback_recommendation.json"
    result["report_path"] = str(report_path)
    structure_report_requested = bool(result.get("robot_prim_inspection_requested"))
    result["robot_structure_report_generated"] = structure_report_requested
    result["robot_structure_report_path"] = str(structure_report_path) if structure_report_requested else None
    articulation_readiness_requested = bool(result.get("articulation_readiness_requested"))
    result["articulation_readiness_report_generated"] = articulation_readiness_requested
    result["articulation_readiness_path"] = (
        str(articulation_readiness_path) if articulation_readiness_requested else None
    )
    articulation_state_requested = bool(result.get("articulation_state_observation_requested"))
    result["articulation_state_report_generated"] = articulation_state_requested
    result["articulation_state_path"] = str(articulation_state_path) if articulation_state_requested else None
    result["articulation_state_report_path"] = (
        str(articulation_state_report_path) if articulation_state_requested else None
    )
    simulation_motion_precheck_requested = bool(result.get("simulation_motion_precheck_requested"))
    result["simulation_motion_precheck_report_generated"] = simulation_motion_precheck_requested
    result["simulation_motion_precheck_path"] = (
        str(simulation_motion_precheck_path) if simulation_motion_precheck_requested else None
    )
    result["simulation_motion_precheck_report_path"] = (
        str(simulation_motion_precheck_report_path) if simulation_motion_precheck_requested else None
    )
    simulation_micro_motion_requested = bool(result.get("simulation_micro_motion_requested"))
    result["simulation_motion_result_path"] = (
        str(simulation_motion_result_path) if simulation_micro_motion_requested else None
    )
    result["simulation_motion_report_path"] = (
        str(simulation_motion_report_path) if simulation_micro_motion_requested else None
    )
    result["before_joint_state_path"] = (
        str(before_articulation_state_path) if simulation_micro_motion_requested else None
    )
    result["after_joint_state_path"] = (
        str(after_articulation_state_path) if simulation_micro_motion_requested else None
    )
    if isinstance(result.get("motion"), dict):
        result["motion"]["simulation_motion_result_path"] = result["simulation_motion_result_path"]
        result["motion"]["simulation_motion_report_path"] = result["simulation_motion_report_path"]
        result["motion"]["before_joint_state_path"] = result["before_joint_state_path"]
        result["motion"]["after_joint_state_path"] = result["after_joint_state_path"]
        evidence = summarize_motion_evidence(result)
        result["motion_evidence_available"] = evidence["motion_evidence_available"]
        result["motion_evidence_files"] = evidence["motion_evidence_files"]
        result["motion_diff_summary"] = evidence["motion_diff_summary"]
        result["before_joint_position_rad"] = result["motion"].get("before_joint_position_rad")
        result["after_joint_position_rad"] = result["motion"].get("after_joint_position_rad")
        result["requested_delta_rad"] = result["motion"].get("requested_delta_rad")
        result["actual_delta_rad"] = result["motion"].get("actual_delta_rad")
        result["tolerance_rad"] = result["motion"].get("tolerance_rad")
        result["delta_within_tolerance"] = result["motion"].get("delta_within_tolerance")
    semantic_bridge_requested = bool(result.get("semantic_simulation_bridge_requested"))
    result["semantic_simulation_bridge_result_path"] = (
        str(semantic_bridge_result_path) if semantic_bridge_requested else None
    )
    result["semantic_simulation_bridge_report_path"] = (
        str(semantic_bridge_report_path) if semantic_bridge_requested else None
    )
    result["semantic_task_contract_copy_path"] = (
        str(semantic_task_contract_copy_path) if semantic_bridge_requested else None
    )
    result["semantic_bridge_evidence_available"] = semantic_bridge_requested
    result["semantic_bridge_files"] = _semantic_bridge_file_refs(result)
    result["semantic_bridge"] = _semantic_bridge_info(result)
    result["motion"] = _motion_info(result)
    result["precheck"] = _precheck_info(result)
    result["safety"] = _safety_report_fields(result)
    simulated_task_execution_requested = bool(result.get("safe_simulated_task_execution_requested"))
    if simulated_task_execution_requested:
        result["simulated_task_execution_result_path"] = str(simulated_task_execution_result_path)
        result["simulated_task_execution_report_path"] = str(simulated_task_execution_report_path)
        result["execution_feedback_path"] = str(execution_feedback_path)
        result["execution_attempt_record_path"] = str(execution_attempt_record_path)
        result["failure_analysis_path"] = str(failure_analysis_path)
        result["retry_fallback_recommendation_path"] = str(retry_fallback_recommendation_path)
        result["simulated_task_execution_files"] = _simulated_task_execution_file_refs(result)
        result["simulated_task_execution"] = _simulated_task_execution_info(result)
        result["post_motion_state_check"] = result["simulated_task_execution"].get("post_motion_state_check", {})
    with report_path.open("w", encoding="utf-8") as report_file:
        json.dump(result, report_file, ensure_ascii=False, indent=2)
        report_file.write("\n")
    export_simulation_evidence(result, run_dir, demo_command=demo_command)
    return report_path


def _finalize_result(
    result: Dict[str, Any],
    *,
    output_dir: str | Path | None,
    write_report: bool,
    demo_command: str | None = None,
) -> Dict[str, Any]:
    if write_report:
        write_simulation_execution_result(result, output_dir, demo_command=demo_command)
    return result


def _run_true_isaac_runtime(
    *,
    simulation_task: Dict[str, Any],
    steps: int,
    headless: bool,
    spawn_object: bool,
    move_object: bool,
    object_spec: SimulationObjectSpec,
    check_robot_asset: bool,
    load_robot_asset: bool,
    robot_asset_spec: RobotAssetSpec,
    inspect_robot_prim: bool,
    check_articulation_readiness: bool,
    observe_articulation_state: bool,
    check_simulation_motion_precheck: bool,
    execute_simulation_micro_motion: bool,
    micro_motion_request: SimulationMicroMotionRequest,
    semantic_bridge_metadata: Dict[str, Any] | None,
    simulated_task_execution_metadata: Dict[str, Any] | None,
    started_at: str,
    output_dir: str | Path | None,
    write_report: bool,
    demo_command: str | None = None,
) -> Dict[str, Any]:
    try:
        from isaacsim import SimulationApp
    except Exception as exc:
        return _finalize_result(
            build_simulation_execution_result(
                simulation_task=simulation_task,
                status="FAIL",
                mode="isaac",
                steps_requested=steps,
                object_metadata=_simulation_object_report_fields(
                    spec=object_spec if spawn_object else None,
                    spawned=False,
                    move_requested=move_object,
                ),
                robot_asset_metadata=_robot_asset_report_fields(
                    spec=robot_asset_spec if check_robot_asset else None,
                    check_requested=check_robot_asset,
                    load_requested=load_robot_asset,
                ),
                robot_prim_inspection_metadata=_robot_prim_inspection_report_fields(
                    spec=robot_asset_spec,
                    requested=inspect_robot_prim,
                ),
                articulation_readiness_metadata=_articulation_readiness_report_fields(
                    spec=robot_asset_spec,
                    requested=check_articulation_readiness,
                ),
                articulation_state_metadata=_articulation_state_report_fields(
                    spec=robot_asset_spec,
                    requested=observe_articulation_state,
                ),
                simulation_motion_precheck_metadata=_simulation_motion_precheck_report_fields(
                    spec=robot_asset_spec,
                    requested=check_simulation_motion_precheck,
                ),
                simulation_micro_motion_metadata=_simulation_micro_motion_report_fields(
                    request=micro_motion_request,
                    requested=execute_simulation_micro_motion,
                ),
                semantic_bridge_metadata=semantic_bridge_metadata,
                simulated_task_execution_metadata=simulated_task_execution_metadata,
                error_code="E_ISAAC_RUNTIME_FAILED",
                error_message=str(exc),
                started_at=started_at,
                finished_at=_timestamp(),
            ),
            output_dir=output_dir,
            write_report=write_report,
            demo_command=demo_command,
        )

    return _execute_isaac_world(
        simulation_task=simulation_task,
        simulation_app_cls=SimulationApp,
        world_cls=None,
        steps=steps,
        headless=headless,
        spawn_object=spawn_object,
        move_object=move_object,
        object_spec=object_spec,
        check_robot_asset=check_robot_asset,
        load_robot_asset=load_robot_asset,
        robot_asset_spec=robot_asset_spec,
        inspect_robot_prim=inspect_robot_prim,
        check_articulation_readiness=check_articulation_readiness,
        observe_articulation_state=observe_articulation_state,
        check_simulation_motion_precheck=check_simulation_motion_precheck,
        execute_simulation_micro_motion=execute_simulation_micro_motion,
        micro_motion_request=micro_motion_request,
        semantic_bridge_metadata=semantic_bridge_metadata,
        simulated_task_execution_metadata=simulated_task_execution_metadata,
        started_at=started_at,
        output_dir=output_dir,
        write_report=write_report,
        demo_command=demo_command,
    )


def _execute_isaac_world(
    *,
    simulation_task: Dict[str, Any],
    simulation_app_cls,
    world_cls,
    steps: int,
    headless: bool,
    spawn_object: bool,
    move_object: bool,
    object_spec: SimulationObjectSpec,
    started_at: str,
    output_dir: str | Path | None,
    write_report: bool,
    check_robot_asset: bool = False,
    load_robot_asset: bool = False,
    robot_asset_spec: RobotAssetSpec | None = None,
    inspect_robot_prim: bool = False,
    check_articulation_readiness: bool = False,
    observe_articulation_state: bool = False,
    check_simulation_motion_precheck: bool = False,
    execute_simulation_micro_motion: bool = False,
    micro_motion_request: SimulationMicroMotionRequest | None = None,
    semantic_bridge_metadata: Dict[str, Any] | None = None,
    simulated_task_execution_metadata: Dict[str, Any] | None = None,
    demo_command: str | None = None,
    object_spawner=None,
    object_pose_updater=None,
    robot_asset_loader=None,
    robot_prim_verifier=None,
    robot_prim_inspector=None,
    articulation_state_observer=None,
    simulation_micro_motion_executor=None,
) -> Dict[str, Any]:
    simulation_app = None
    object_spawner = object_spawner or spawn_simulation_object
    object_pose_updater = object_pose_updater or update_simulation_object_pose
    object_handle = None
    object_metadata = _simulation_object_report_fields()
    robot_asset_spec = robot_asset_spec or RobotAssetSpec()
    robot_asset_metadata = _robot_asset_report_fields(
        spec=robot_asset_spec if check_robot_asset else None,
        check_requested=check_robot_asset,
        load_requested=load_robot_asset,
    )
    robot_asset_loader = robot_asset_loader or load_robot_asset_into_stage
    robot_prim_verifier = robot_prim_verifier or verify_robot_prim_exists
    robot_prim_inspector = robot_prim_inspector or inspect_robot_prim_in_stage
    articulation_state_observer = articulation_state_observer or observe_articulation_state_in_world
    robot_prim_inspection_metadata = _robot_prim_inspection_report_fields(
        spec=robot_asset_spec,
        requested=inspect_robot_prim,
    )
    articulation_readiness_metadata = _articulation_readiness_report_fields(
        spec=robot_asset_spec,
        requested=check_articulation_readiness,
        inspection=robot_prim_inspection_metadata.get("robot_prim_inspection"),
    )
    articulation_state_metadata = _articulation_state_report_fields(
        spec=robot_asset_spec,
        requested=observe_articulation_state,
        inspection=robot_prim_inspection_metadata.get("robot_prim_inspection"),
        readiness=articulation_readiness_metadata.get("articulation_readiness"),
    )
    simulation_motion_precheck_metadata = _simulation_motion_precheck_report_fields(
        spec=robot_asset_spec,
        requested=check_simulation_motion_precheck,
        robot_asset=robot_asset_metadata,
        inspection=robot_prim_inspection_metadata.get("robot_prim_inspection"),
        readiness=articulation_readiness_metadata.get("articulation_readiness"),
        state=articulation_state_metadata.get("articulation_state"),
    )
    micro_motion_request = micro_motion_request or SimulationMicroMotionRequest()
    simulation_micro_motion_metadata = _simulation_micro_motion_report_fields(
        request=micro_motion_request,
        requested=execute_simulation_micro_motion,
        precheck=simulation_motion_precheck_metadata.get("simulation_motion_precheck"),
        readiness=articulation_readiness_metadata.get("articulation_readiness"),
        before_state=articulation_state_metadata.get("articulation_state"),
    )
    world_reset = False
    try:
        simulation_app = simulation_app_cls({"headless": headless})
        if world_cls is None:
            world_cls = _load_isaac_world_class()
        world = world_cls()
        world.reset()
        world_reset = True

        if check_robot_asset or load_robot_asset:
            robot_asset_metadata = evaluate_robot_asset_availability(
                robot_asset_spec,
                check_requested=check_robot_asset,
                load_requested=load_robot_asset,
                dry_run=False,
            )
            if load_robot_asset:
                if not robot_asset_metadata["robot_asset_available"]:
                    robot_asset_metadata = _robot_asset_report_fields(
                        spec=robot_asset_spec,
                        check_requested=check_robot_asset,
                        load_requested=True,
                        available=False,
                        loaded=False,
                        prim_exists=False,
                        status="LOAD_FAILED",
                        blocking_reason="E_ROBOT_ASSET_UNAVAILABLE",
                    )
                    return _finalize_result(
                        build_simulation_execution_result(
                            simulation_task=simulation_task,
                            status="FAIL",
                            mode="isaac",
                            steps_requested=steps,
                            world_reset=world_reset,
                            object_metadata=object_metadata,
                            robot_asset_metadata=robot_asset_metadata,
                            robot_prim_inspection_metadata=robot_prim_inspection_metadata,
                            articulation_readiness_metadata=articulation_readiness_metadata,
                            articulation_state_metadata=articulation_state_metadata,
                            simulation_motion_precheck_metadata=simulation_motion_precheck_metadata,
                            simulation_micro_motion_metadata=simulation_micro_motion_metadata,
                            semantic_bridge_metadata=semantic_bridge_metadata,
                            simulated_task_execution_metadata=simulated_task_execution_metadata,
                            error_code="E_ROBOT_ASSET_LOAD_FAILED",
                            error_message="robot asset path is unavailable",
                            started_at=started_at,
                            finished_at=_timestamp(),
                        ),
                        output_dir=output_dir,
                        write_report=write_report,
                        demo_command=demo_command,
                    )
                try:
                    robot_asset_loader(world, robot_asset_spec=robot_asset_spec)
                    prim_exists = robot_prim_verifier(world, robot_asset_spec=robot_asset_spec)
                    robot_asset_metadata = _robot_asset_report_fields(
                        spec=robot_asset_spec,
                        check_requested=check_robot_asset,
                        load_requested=True,
                        available=True,
                        loaded=prim_exists,
                        prim_exists=prim_exists,
                        status="LOADED" if prim_exists else "LOAD_FAILED",
                        blocking_reason=None if prim_exists else "E_ROBOT_PRIM_NOT_FOUND",
                    )
                    if not prim_exists:
                        return _finalize_result(
                            build_simulation_execution_result(
                                simulation_task=simulation_task,
                                status="FAIL",
                                mode="isaac",
                                steps_requested=steps,
                                world_reset=world_reset,
                                object_metadata=object_metadata,
                                robot_asset_metadata=robot_asset_metadata,
                                robot_prim_inspection_metadata=robot_prim_inspection_metadata,
                                articulation_readiness_metadata=articulation_readiness_metadata,
                                articulation_state_metadata=articulation_state_metadata,
                                simulation_motion_precheck_metadata=simulation_motion_precheck_metadata,
                                simulation_micro_motion_metadata=simulation_micro_motion_metadata,
                                semantic_bridge_metadata=semantic_bridge_metadata,
                                simulated_task_execution_metadata=simulated_task_execution_metadata,
                                error_code="E_ROBOT_ASSET_LOAD_FAILED",
                                error_message="robot prim was not found after loading asset",
                                started_at=started_at,
                                finished_at=_timestamp(),
                            ),
                            output_dir=output_dir,
                            write_report=write_report,
                            demo_command=demo_command,
                        )
                except Exception as exc:
                    robot_asset_metadata = _robot_asset_report_fields(
                        spec=robot_asset_spec,
                        check_requested=check_robot_asset,
                        load_requested=True,
                        available=robot_asset_metadata["robot_asset_available"],
                        loaded=False,
                        prim_exists=False,
                        status="LOAD_FAILED",
                        blocking_reason="E_ROBOT_ASSET_LOAD_FAILED",
                    )
                    return _finalize_result(
                        build_simulation_execution_result(
                            simulation_task=simulation_task,
                            status="FAIL",
                            mode="isaac",
                            steps_requested=steps,
                            world_reset=world_reset,
                            object_metadata=object_metadata,
                            robot_asset_metadata=robot_asset_metadata,
                            robot_prim_inspection_metadata=robot_prim_inspection_metadata,
                            articulation_readiness_metadata=articulation_readiness_metadata,
                            articulation_state_metadata=articulation_state_metadata,
                            simulation_motion_precheck_metadata=simulation_motion_precheck_metadata,
                            simulation_micro_motion_metadata=simulation_micro_motion_metadata,
                            semantic_bridge_metadata=semantic_bridge_metadata,
                            simulated_task_execution_metadata=simulated_task_execution_metadata,
                            error_code="E_ROBOT_ASSET_LOAD_FAILED",
                            error_message=str(exc),
                            started_at=started_at,
                            finished_at=_timestamp(),
                        ),
                        output_dir=output_dir,
                        write_report=write_report,
                        demo_command=demo_command,
                    )

        if spawn_object:
            try:
                object_handle, object_metadata = object_spawner(
                    world,
                    object_spec=object_spec,
                )
            except Exception as exc:
                object_metadata = _simulation_object_report_fields(
                    spec=object_spec,
                    spawned=False,
                    move_requested=move_object,
                )
                return _finalize_result(
                    build_simulation_execution_result(
                        simulation_task=simulation_task,
                        status="FAIL",
                        mode="isaac",
                        steps_requested=steps,
                        world_reset=world_reset,
                        object_metadata=object_metadata,
                        robot_asset_metadata=robot_asset_metadata,
                        robot_prim_inspection_metadata=robot_prim_inspection_metadata,
                        articulation_readiness_metadata=articulation_readiness_metadata,
                        articulation_state_metadata=articulation_state_metadata,
                        simulation_motion_precheck_metadata=simulation_motion_precheck_metadata,
                        simulation_micro_motion_metadata=simulation_micro_motion_metadata,
                        semantic_bridge_metadata=semantic_bridge_metadata,
                        simulated_task_execution_metadata=simulated_task_execution_metadata,
                        error_code="E_CUBE_SPAWN_FAILED",
                        error_message=str(exc),
                        started_at=started_at,
                        finished_at=_timestamp(),
                    ),
                    output_dir=output_dir,
                    write_report=write_report,
                    demo_command=demo_command,
                )

        if move_object:
            try:
                object_metadata = object_pose_updater(
                    object_handle,
                    object_spec=object_spec,
                    current_metadata=object_metadata,
                )
            except Exception as exc:
                object_metadata = _simulation_object_report_fields(
                    spec=object_spec,
                    spawned=spawn_object and object_handle is not None,
                    move_requested=True,
                    moved=False,
                    final_position=object_spec.initial_position,
                )
                return _finalize_result(
                    build_simulation_execution_result(
                        simulation_task=simulation_task,
                        status="FAIL",
                        mode="isaac",
                        steps_requested=steps,
                        world_reset=world_reset,
                        object_metadata=object_metadata,
                        robot_asset_metadata=robot_asset_metadata,
                        robot_prim_inspection_metadata=robot_prim_inspection_metadata,
                        articulation_readiness_metadata=articulation_readiness_metadata,
                        articulation_state_metadata=articulation_state_metadata,
                        simulation_motion_precheck_metadata=simulation_motion_precheck_metadata,
                        simulation_micro_motion_metadata=simulation_micro_motion_metadata,
                        semantic_bridge_metadata=semantic_bridge_metadata,
                        simulated_task_execution_metadata=simulated_task_execution_metadata,
                        error_code="E_SIM_OBJECT_MOVE_FAILED",
                        error_message=str(exc),
                        started_at=started_at,
                        finished_at=_timestamp(),
                    ),
                    output_dir=output_dir,
                    write_report=write_report,
                    demo_command=demo_command,
                )

        if inspect_robot_prim:
            robot_prim_inspection_metadata = _robot_prim_inspection_report_fields(
                spec=robot_asset_spec,
                requested=True,
                inspection=robot_prim_inspector(world, robot_prim_path=robot_asset_spec.robot_prim_path),
            )
        if check_articulation_readiness:
            articulation_readiness_metadata = _articulation_readiness_report_fields(
                spec=robot_asset_spec,
                requested=True,
                inspection=robot_prim_inspection_metadata.get("robot_prim_inspection"),
            )
        if observe_articulation_state:
            articulation_state_metadata = _articulation_state_report_fields(
                spec=robot_asset_spec,
                requested=True,
                inspection=robot_prim_inspection_metadata.get("robot_prim_inspection"),
                readiness=articulation_readiness_metadata.get("articulation_readiness"),
                state=articulation_state_observer(
                    world,
                    robot_prim_path=robot_asset_spec.robot_prim_path,
                    robot_prim_inspection=robot_prim_inspection_metadata.get("robot_prim_inspection"),
                    articulation_readiness=articulation_readiness_metadata.get("articulation_readiness"),
                ),
            )
        if check_simulation_motion_precheck:
            simulation_motion_precheck_metadata = _simulation_motion_precheck_report_fields(
                spec=robot_asset_spec,
                requested=True,
                robot_asset=robot_asset_metadata,
                inspection=robot_prim_inspection_metadata.get("robot_prim_inspection"),
                readiness=articulation_readiness_metadata.get("articulation_readiness"),
                state=articulation_state_metadata.get("articulation_state"),
            )
        if execute_simulation_micro_motion:
            executor = simulation_micro_motion_executor or _make_isaac_micro_motion_executor(
                world,
                robot_asset_spec=robot_asset_spec,
                headless=headless,
                articulation_state_observer=articulation_state_observer,
                robot_prim_inspection=robot_prim_inspection_metadata.get("robot_prim_inspection"),
                articulation_readiness=articulation_readiness_metadata.get("articulation_readiness"),
            )
            simulation_micro_motion_metadata = _simulation_micro_motion_report_fields(
                request=micro_motion_request,
                requested=True,
                precheck=simulation_motion_precheck_metadata.get("simulation_motion_precheck"),
                readiness=articulation_readiness_metadata.get("articulation_readiness"),
                before_state=articulation_state_metadata.get("articulation_state"),
                motion_executor=executor,
            )
            after_state = simulation_micro_motion_metadata.get("after_articulation_state")
            if isinstance(after_state, dict):
                articulation_state_metadata = _articulation_state_report_fields(
                    spec=robot_asset_spec,
                    requested=True,
                    inspection=robot_prim_inspection_metadata.get("robot_prim_inspection"),
                    readiness=articulation_readiness_metadata.get("articulation_readiness"),
                    state=after_state,
                )
            if simulation_micro_motion_metadata.get("simulation_micro_motion_status") != MICRO_MOTION_STATUS_OK:
                error_code = (
                    "E_SIMULATION_MOTION_PRECHECK_NOT_READY"
                    if simulation_micro_motion_metadata.get("simulation_micro_motion_status")
                    == MICRO_MOTION_STATUS_BLOCKED_BY_PRECHECK
                    else "E_SIMULATION_MICRO_MOTION_FAILED"
                )
                return _finalize_result(
                    build_simulation_execution_result(
                        simulation_task=simulation_task,
                        status="FAIL",
                        mode="isaac",
                        steps_requested=steps,
                        world_reset=world_reset,
                        object_metadata=object_metadata,
                        robot_asset_metadata=robot_asset_metadata,
                        robot_prim_inspection_metadata=robot_prim_inspection_metadata,
                        articulation_readiness_metadata=articulation_readiness_metadata,
                        articulation_state_metadata=articulation_state_metadata,
                        simulation_motion_precheck_metadata=simulation_motion_precheck_metadata,
                        simulation_micro_motion_metadata=simulation_micro_motion_metadata,
                        semantic_bridge_metadata=_semantic_bridge_status_from_motion(
                            semantic_bridge_metadata,
                            simulation_micro_motion_metadata,
                        ),
                        simulated_task_execution_metadata=simulated_task_execution_metadata,
                        error_code=error_code,
                        error_message=", ".join(
                            simulation_micro_motion_metadata.get("simulation_micro_motion_blocking_reasons") or []
                        ),
                        started_at=started_at,
                        finished_at=_timestamp(),
                    ),
                    output_dir=output_dir,
                    write_report=write_report,
                    demo_command=demo_command,
                )

        steps_completed = 0
        for _ in range(steps):
            world.step(render=not headless)
            steps_completed += 1

        return _finalize_result(
            build_simulation_execution_result(
                simulation_task=simulation_task,
                status="PASS",
                mode="isaac",
                steps_requested=steps,
                steps_completed=steps_completed,
                world_reset=world_reset,
                object_metadata=object_metadata,
                robot_asset_metadata=robot_asset_metadata,
                robot_prim_inspection_metadata=robot_prim_inspection_metadata,
                articulation_readiness_metadata=articulation_readiness_metadata,
                articulation_state_metadata=articulation_state_metadata,
                simulation_motion_precheck_metadata=simulation_motion_precheck_metadata,
                simulation_micro_motion_metadata=simulation_micro_motion_metadata,
                semantic_bridge_metadata=_semantic_bridge_status_from_motion(
                    semantic_bridge_metadata,
                    simulation_micro_motion_metadata,
                ),
                simulated_task_execution_metadata=simulated_task_execution_metadata,
                started_at=started_at,
                finished_at=_timestamp(),
            ),
            output_dir=output_dir,
            write_report=write_report,
            demo_command=demo_command,
        )
    except Exception as exc:
        return _finalize_result(
            build_simulation_execution_result(
                simulation_task=simulation_task,
                status="FAIL",
                mode="isaac",
                steps_requested=steps,
                world_reset=world_reset,
                object_metadata=object_metadata,
                robot_asset_metadata=robot_asset_metadata,
                robot_prim_inspection_metadata=robot_prim_inspection_metadata,
                articulation_readiness_metadata=articulation_readiness_metadata,
                articulation_state_metadata=articulation_state_metadata,
                simulation_motion_precheck_metadata=simulation_motion_precheck_metadata,
                simulation_micro_motion_metadata=simulation_micro_motion_metadata,
                semantic_bridge_metadata=semantic_bridge_metadata,
                simulated_task_execution_metadata=simulated_task_execution_metadata,
                error_code="E_ISAAC_RUNTIME_FAILED",
                error_message=str(exc),
                started_at=started_at,
                finished_at=_timestamp(),
            ),
            output_dir=output_dir,
            write_report=write_report,
            demo_command=demo_command,
        )
    finally:
        if simulation_app is not None:
            simulation_app.close()


def _load_isaac_world_class():
    try:
        from isaacsim.core.api import World
    except ImportError:
        from omni.isaac.core import World
    return World


def spawn_simulation_object(
    world,
    *,
    object_spec: SimulationObjectSpec,
) -> tuple[Any, Dict[str, Any]]:
    if object_spec.object_type != "cube":
        raise ValueError(f"unsupported simulation object type: {object_spec.object_type}")

    try:
        from isaacsim.core.api.objects import VisualCuboid
    except ImportError:
        from omni.isaac.core.objects import VisualCuboid

    simulation_object = VisualCuboid(
        prim_path=object_spec.prim_path,
        name="teto_simulation_object",
        position=list(object_spec.initial_position),
        size=object_spec.size,
    )
    if hasattr(world, "scene") and hasattr(world.scene, "add"):
        world.scene.add(simulation_object)
    return simulation_object, _simulation_object_report_fields(
        spec=object_spec,
        spawned=True,
    )


def update_simulation_object_pose(
    simulation_object,
    *,
    object_spec: SimulationObjectSpec,
    current_metadata: Dict[str, Any] | None = None,
) -> Dict[str, Any]:
    if simulation_object is None:
        raise ValueError("simulation object is required for pose update")
    if not hasattr(simulation_object, "set_world_pose"):
        raise AttributeError("simulation object does not support set_world_pose")

    simulation_object.set_world_pose(position=list(object_spec.target_position))
    return _simulation_object_report_fields(
        spec=object_spec,
        spawned=bool((current_metadata or {}).get("simulation_object_spawned")),
        move_requested=True,
        moved=True,
        final_position=object_spec.target_position,
    )


def resolve_robot_asset_path(robot_asset_spec: RobotAssetSpec) -> Path | None:
    if not robot_asset_spec.robot_asset_path:
        return None
    if _is_network_asset_path(robot_asset_spec.robot_asset_path):
        return None
    return Path(robot_asset_spec.robot_asset_path).expanduser()


def evaluate_robot_asset_availability(
    robot_asset_spec: RobotAssetSpec,
    *,
    check_requested: bool,
    load_requested: bool,
    dry_run: bool = False,
) -> Dict[str, Any]:
    if not check_requested and not load_requested:
        return _robot_asset_report_fields()

    asset_path = robot_asset_spec.robot_asset_path
    if not asset_path:
        return _robot_asset_report_fields(
            spec=robot_asset_spec,
            check_requested=check_requested,
            load_requested=load_requested,
            available=False,
            loaded=False,
            prim_exists=False,
            status="UNAVAILABLE",
            blocking_reason="E_ROBOT_ASSET_UNAVAILABLE",
            asset_source="dry_run" if dry_run else "unavailable",
        )

    if _is_network_asset_path(asset_path):
        return _robot_asset_report_fields(
            spec=robot_asset_spec,
            check_requested=check_requested,
            load_requested=load_requested,
            available=False,
            loaded=False,
            prim_exists=False,
            status="UNAVAILABLE",
            blocking_reason="E_ROBOT_ASSET_UNAVAILABLE",
            asset_source="remote",
        )

    resolved_path = resolve_robot_asset_path(robot_asset_spec)
    is_available = bool(
        resolved_path
        and resolved_path.is_file()
        and resolved_path.suffix.lower() in {".usd", ".usda", ".usdc"}
    )
    simulated_loaded = bool(dry_run and load_requested and is_available)
    return _robot_asset_report_fields(
        spec=robot_asset_spec,
        check_requested=check_requested,
        load_requested=load_requested,
        available=is_available,
        loaded=simulated_loaded,
        prim_exists=simulated_loaded,
        status="LOADED" if simulated_loaded else ("AVAILABLE" if is_available else "UNAVAILABLE"),
        blocking_reason=None if is_available else "E_ROBOT_ASSET_UNAVAILABLE",
        asset_source="local" if is_available else ("dry_run" if dry_run else "unavailable"),
    )


def load_robot_asset_into_stage(world, *, robot_asset_spec: RobotAssetSpec) -> None:
    resolved_path = resolve_robot_asset_path(robot_asset_spec)
    if resolved_path is None or not resolved_path.is_file():
        raise FileNotFoundError(robot_asset_spec.robot_asset_path or "robot asset path is not set")
    if resolved_path.suffix.lower() not in {".usd", ".usda", ".usdc"}:
        raise ValueError(f"unsupported robot asset kind: {resolved_path.suffix}")

    try:
        from isaacsim.core.utils.stage import add_reference_to_stage
    except ImportError:
        from omni.isaac.core.utils.stage import add_reference_to_stage

    add_reference_to_stage(usd_path=str(resolved_path), prim_path=robot_asset_spec.robot_prim_path)


def verify_robot_prim_exists(world, *, robot_asset_spec: RobotAssetSpec) -> bool:
    stage = getattr(world, "stage", None)
    if stage is None:
        try:
            import omni.usd

            stage = omni.usd.get_context().get_stage()
        except Exception:
            stage = None
    if stage is None or not hasattr(stage, "GetPrimAtPath"):
        return False
    prim = stage.GetPrimAtPath(robot_asset_spec.robot_prim_path)
    return bool(prim and hasattr(prim, "IsValid") and prim.IsValid())


def _lab_readiness_report_fields(
    *,
    requested: bool = False,
    readiness: Dict[str, Any] | None = None,
) -> Dict[str, Any]:
    readiness = readiness if isinstance(readiness, dict) else {}
    lab = readiness.get("lab_backend") if isinstance(readiness.get("lab_backend"), dict) else {}
    camera = readiness.get("camera") if isinstance(readiness.get("camera"), dict) else {}
    live_vlm = readiness.get("live_vlm") if isinstance(readiness.get("live_vlm"), dict) else {}
    shadow = readiness.get("shadow_mode") if isinstance(readiness.get("shadow_mode"), dict) else {}
    status = readiness.get("status", "NOT_REQUESTED")
    return {
        "lab_readiness_requested": requested,
        "lab_readiness_status": status,
        "lab_backend_readiness_status": readiness.get("lab_backend_readiness_status", lab.get("status", "NOT_REQUESTED")),
        "camera_readiness_status": readiness.get("camera_readiness_status", camera.get("status", "NOT_REQUESTED")),
        "live_vlm_readiness_status": readiness.get("live_vlm_readiness_status", live_vlm.get("status", "NOT_REQUESTED")),
        "shadow_mode_readiness_status": readiness.get("shadow_mode_readiness_status", shadow.get("status", "NOT_REQUESTED")),
        "no_motion_readiness_passed": readiness.get("no_motion_readiness_passed", False) is True,
        "allow_live_camera": readiness.get("allow_live_camera", False) is True,
        "allow_live_vlm": readiness.get("allow_live_vlm", False) is True,
        "real_robot_command_enabled": readiness.get("real_robot_command_enabled", False) is True,
        "readiness_blocking_reasons": list(readiness.get("blocking_reasons") or []),
        "next_safe_action": readiness.get("next_safe_action"),
        "lab_readiness": readiness
        or {
            "requested": False,
            "status": "NOT_REQUESTED",
            "lab_backend_readiness_status": "NOT_REQUESTED",
            "camera_readiness_status": "NOT_REQUESTED",
            "live_vlm_readiness_status": "NOT_REQUESTED",
            "shadow_mode_readiness_status": "NOT_REQUESTED",
            "no_motion_readiness_passed": False,
            "allow_robot_motion": False,
            "allow_live_camera": False,
            "allow_live_vlm": False,
            "real_robot_command_enabled": False,
            "blocking_reasons": [],
            "readiness_evidence_files": [],
            "next_safe_action": None,
        },
    }


def _camera_snapshot_report_fields(
    *,
    requested: bool = False,
    snapshot: Dict[str, Any] | None = None,
) -> Dict[str, Any]:
    snapshot = snapshot if isinstance(snapshot, dict) else {}
    return {
        "camera_snapshot_requested": requested,
        "camera_snapshot": snapshot
        or {
            "requested": False,
            "validity_status": "NOT_REQUESTED",
            "blocking_reasons": [],
            "warnings": [],
            "no_motion_snapshot_passed": False,
            "live_capture_used": False,
            "live_camera_enabled": False,
            "live_vlm_called": False,
            "real_robot_motion_executed": False,
            "real_robot_command_enabled": False,
        },
        "camera_snapshot_id": snapshot.get("snapshot_id"),
        "camera_snapshot_scene_version": snapshot.get("scene_version"),
        "camera_snapshot_validity_status": snapshot.get("validity_status", "NOT_REQUESTED"),
        "camera_snapshot_blocking_reasons": list(snapshot.get("blocking_reasons") or []),
        "camera_snapshot_warnings": list(snapshot.get("warnings") or []),
        "no_motion_snapshot_passed": snapshot.get("no_motion_snapshot_passed", False) is True,
        "live_capture_used": snapshot.get("live_capture_used", False) is True,
        "live_camera_enabled": snapshot.get("live_camera_enabled", False) is True,
        "live_vlm_called": snapshot.get("live_vlm_called", False) is True,
        "real_robot_command_enabled": snapshot.get("real_robot_command_enabled", False) is True,
    }


def _real_scene_shadow_report_fields(
    *,
    requested: bool = False,
    shadow: Dict[str, Any] | None = None,
) -> Dict[str, Any]:
    shadow = shadow if isinstance(shadow, dict) else {}
    fields = {
        "real_scene_shadow_requested": requested,
        "real_scene_shadow": shadow
        or {
            "requested": False,
            "shadow_pipeline_status": "NOT_REQUESTED",
            "semantic_gate_passed": False,
            "no_motion_shadow_passed": False,
            "blocking_reasons": [],
            "warnings": [],
            "replay_ready": False,
            "live_camera_used": False,
            "live_vlm_called": False,
            "real_robot_motion_executed": False,
            "real_robot_command_enabled": False,
            "robot_command_generated": False,
            "trajectory_generated": False,
            "joint_targets_generated": False,
            "tcp_pose_world_generated": False,
        },
        "real_scene_shadow_status": shadow.get("shadow_pipeline_status", "NOT_REQUESTED"),
        "real_scene_shadow_snapshot_id": shadow.get("snapshot_id"),
        "real_scene_shadow_grounding_id": shadow.get("grounding_id"),
        "real_scene_shadow_scene_version": shadow.get("scene_version"),
        "no_motion_shadow_passed": shadow.get("no_motion_shadow_passed", False) is True,
        "real_scene_shadow_blocking_reasons": list(shadow.get("blocking_reasons") or []),
        "real_scene_shadow_warnings": list(shadow.get("warnings") or []),
        "real_scene_shadow_next_safe_action": shadow.get("next_safe_action"),
        "real_scene_shadow_replay_ready": shadow.get("replay_ready", False) is True,
    }
    if requested:
        fields["semantic_gate_passed"] = shadow.get("semantic_gate_passed", False) is True
    return fields


def _simulation_object_report_fields(
    *,
    spec: SimulationObjectSpec | None = None,
    spawned: bool = False,
    move_requested: bool = False,
    moved: bool = False,
    final_position: list[float] | tuple[float, float, float] | None = None,
) -> Dict[str, Any]:
    object_type = spec.object_type if spec else None
    initial_position = _position_list(spec.initial_position) if spec else None
    target_position = _position_list(spec.target_position) if spec and move_requested else None
    normalized_final_position = _position_list(final_position) if final_position is not None else None
    displacement = _displacement(initial_position, normalized_final_position) if normalized_final_position else None
    object_size = float(spec.size) if spec and spec.size is not None else None

    return {
        "simulation_object_spawned": spawned,
        "simulation_object_moved": moved,
        "simulation_object_move_requested": move_requested,
        "simulation_object_type": object_type,
        "simulation_object_prim_path": spec.prim_path if spec else None,
        "simulation_object_initial_position": initial_position,
        "simulation_object_target_position": target_position,
        "simulation_object_final_position": normalized_final_position,
        "simulation_object_displacement": displacement,
        "simulation_object_size": object_size,
        "object_type": object_type,
        "cube_prim_path": spec.prim_path if spec and spec.object_type == "cube" else None,
        "cube_position": initial_position if spec and spec.object_type == "cube" else None,
        "cube_size": object_size if spec and spec.object_type == "cube" else None,
        "cube_spawned": spawned if spec is None or spec.object_type == "cube" else False,
        "cube_move_requested": move_requested if spec is None or spec.object_type == "cube" else False,
        "cube_moved": moved if spec is None or spec.object_type == "cube" else False,
        "cube_initial_position": initial_position if spec and spec.object_type == "cube" else None,
        "cube_target_position": target_position if spec and spec.object_type == "cube" else None,
        "cube_final_position": normalized_final_position if spec and spec.object_type == "cube" else None,
        "cube_displacement": displacement if spec and spec.object_type == "cube" else None,
    }


def _robot_asset_report_fields(
    *,
    spec: RobotAssetSpec | None = None,
    check_requested: bool = False,
    load_requested: bool = False,
    available: bool = False,
    loaded: bool = False,
    prim_exists: bool = False,
    status: str | None = None,
    blocking_reason: str | None = None,
    asset_source: str | None = None,
) -> Dict[str, Any]:
    resolved_status = status or ("NOT_REQUESTED" if not check_requested and not load_requested else "UNAVAILABLE")
    return {
        "robot_asset_check_requested": check_requested,
        "robot_asset_load_requested": load_requested,
        "robot_type": spec.robot_type if spec else None,
        "robot_prim_path": spec.robot_prim_path if spec else None,
        "robot_asset_path": spec.robot_asset_path if spec else None,
        "robot_asset_source": asset_source or (spec.asset_source if spec else None),
        "robot_asset_available": available,
        "robot_asset_loaded": loaded,
        "robot_prim_exists": prim_exists,
        "robot_asset_status": resolved_status,
        "robot_asset_blocking_reason": blocking_reason,
    }


def _robot_prim_inspection_report_fields(
    *,
    spec: RobotAssetSpec | None = None,
    requested: bool = False,
    inspection: Dict[str, Any] | None = None,
) -> Dict[str, Any]:
    robot_prim_path = spec.robot_prim_path if spec else None
    normalized_inspection = inspection or build_robot_prim_inspection_report(
        requested=requested,
        robot_prim_path=robot_prim_path,
    )
    return {
        "robot_prim_inspection_requested": requested,
        "robot_prim_inspection": normalized_inspection,
    }


def _articulation_readiness_report_fields(
    *,
    spec: RobotAssetSpec | None = None,
    requested: bool = False,
    inspection: Dict[str, Any] | None = None,
) -> Dict[str, Any]:
    readiness = build_articulation_readiness_report(
        requested=requested,
        robot_prim_path=spec.robot_prim_path if spec else None,
        robot_prim_inspection=inspection,
        has_robot_structure_report=bool(inspection and inspection.get("requested")),
    )
    return {
        "articulation_readiness_requested": requested,
        "articulation_readiness": readiness,
    }


def _articulation_state_report_fields(
    *,
    spec: RobotAssetSpec | None = None,
    requested: bool = False,
    inspection: Dict[str, Any] | None = None,
    readiness: Dict[str, Any] | None = None,
    state: Dict[str, Any] | None = None,
) -> Dict[str, Any]:
    normalized_state = state or build_articulation_state_report(
        requested=requested,
        robot_prim_path=spec.robot_prim_path if spec else None,
        robot_prim_inspection=inspection,
        articulation_readiness=readiness,
    )
    report_fields = {
        "articulation_state_observation_requested": requested,
        "articulation_state_observable": normalized_state.get("articulation_state_observable", False),
        "articulation_state": normalized_state,
    }
    if requested:
        report_fields.update(
            {
                "articulation_state_status": normalized_state.get("status"),
                "control_enabled": normalized_state.get("control_enabled", False),
                "motion_generated": normalized_state.get("motion_generated", False),
                "command_generated": normalized_state.get("command_generated", False),
                "joint_targets_generated": normalized_state.get("joint_targets_generated", False),
                "arm_joint_count": normalized_state.get("arm_joint_count", 0),
                "observed_joint_count": normalized_state.get("observed_joint_count", 0),
                "observed_arm_joint_names": normalized_state.get("observed_arm_joint_names", []),
                "missing_arm_joint_names": normalized_state.get("missing_arm_joint_names", []),
                "extra_joint_names": normalized_state.get("extra_joint_names", []),
                "joint_positions_available": normalized_state.get("joint_positions_available", False),
                "joint_velocities_available": normalized_state.get("joint_velocities_available", False),
                "joint_limits_available": normalized_state.get("joint_limits_available", False),
                "articulation_state_warnings": normalized_state.get("warnings", []),
                "articulation_state_errors": normalized_state.get("errors", []),
            }
        )
    return report_fields


def _simulation_motion_precheck_report_fields(
    *,
    spec: RobotAssetSpec | None = None,
    requested: bool = False,
    robot_asset: Dict[str, Any] | None = None,
    inspection: Dict[str, Any] | None = None,
    readiness: Dict[str, Any] | None = None,
    state: Dict[str, Any] | None = None,
) -> Dict[str, Any]:
    robot_asset = robot_asset if isinstance(robot_asset, dict) else {}
    normalized_precheck = build_simulation_motion_precheck_report(
        requested=requested,
        robot_asset_loaded=robot_asset.get("robot_asset_loaded") is True,
        robot_prim_exists=robot_asset.get("robot_prim_exists") is True,
        robot_prim_path=(robot_asset.get("robot_prim_path") or (spec.robot_prim_path if spec else None)),
        robot_prim_inspection=inspection,
        articulation_readiness=readiness,
        articulation_state=state,
    )
    report_fields = {
        "simulation_motion_precheck_requested": requested,
        "simulation_motion_precheck_status": normalized_precheck.get("status"),
        "ready_for_simulation_motion": normalized_precheck.get("ready", False),
        "simulation_motion_precheck": normalized_precheck,
    }
    if requested:
        report_fields.update(
            {
                "control_enabled": normalized_precheck.get("control_enabled", False),
                "motion_generated": normalized_precheck.get("motion_generated", False),
                "command_generated": normalized_precheck.get("command_generated", False),
                "joint_targets_generated": normalized_precheck.get("joint_targets_generated", False),
                "trajectory_generated": normalized_precheck.get("trajectory_generated", False),
                "tcp_pose_world_generated": normalized_precheck.get("tcp_pose_world_generated", False),
                "robot_motion_executed": normalized_precheck.get("robot_motion_executed", False),
                "real_robot_allowed": normalized_precheck.get("real_robot_allowed", False),
                "simulation_motion_precheck_blocking_reasons": normalized_precheck.get("blocking_reasons", []),
                "simulation_motion_precheck_warnings": normalized_precheck.get("warnings", []),
                "simulation_motion_precheck_errors": normalized_precheck.get("errors", []),
            }
        )
    return report_fields


def _simulation_micro_motion_report_fields(
    *,
    request: SimulationMicroMotionRequest | None = None,
    requested: bool = False,
    dry_run: bool = False,
    precheck: Dict[str, Any] | None = None,
    readiness: Dict[str, Any] | None = None,
    before_state: Dict[str, Any] | None = None,
    motion_executor=None,
) -> Dict[str, Any]:
    request = request or SimulationMicroMotionRequest()
    if not requested:
        return {
            "simulation_micro_motion_requested": False,
            "simulation_micro_motion_status": MICRO_MOTION_STATUS_NOT_REQUESTED,
            "simulation_only": True,
            "real_robot_allowed": False,
            "real_robot_motion_executed": False,
            "simulation_control_enabled": False,
            "simulation_command_generated": False,
            "simulation_joint_delta_generated": False,
            "before_articulation_state": {},
            "after_articulation_state": {},
            "precheck": {
                "simulation_motion_precheck_status": None,
                "ready_for_simulation_motion": False,
                "blocking_reasons": [],
                "warnings": [],
                "errors": [],
            },
            "motion": {
                "command_type": None,
                "joint_name": None,
                "requested_delta_rad": None,
                "actual_delta_rad": None,
                "tolerance_rad": None,
                "delta_within_tolerance": False,
                "before_joint_position_rad": None,
                "after_joint_position_rad": None,
                "before_joint_state_path": None,
                "after_joint_state_path": None,
                "simulation_motion_result_path": None,
                "simulation_motion_report_path": None,
            },
        }

    motion_result = execute_simulation_micro_motion(
        request,
        simulation_motion_precheck=precheck,
        articulation_readiness=readiness,
        before_articulation_state=before_state,
        dry_run=dry_run,
        motion_executor=motion_executor,
    )
    return {
        "simulation_micro_motion_requested": True,
        "simulation_micro_motion_status": motion_result.get("simulation_micro_motion_status"),
        "simulation_only": True,
        "real_robot_allowed": False,
        "real_robot_motion_executed": False,
        "robot_motion_executed": motion_result.get("robot_motion_executed", False),
        "control_enabled": motion_result.get("control_enabled", False),
        "simulation_control_enabled": motion_result.get("simulation_control_enabled", False),
        "motion_generated": motion_result.get("motion_generated", False),
        "command_generated": motion_result.get("command_generated", False),
        "simulation_command_generated": motion_result.get("simulation_command_generated", False),
        "joint_targets_generated": motion_result.get("joint_targets_generated", False),
        "simulation_joint_delta_generated": motion_result.get("simulation_joint_delta_generated", False),
        "trajectory_generated": motion_result.get("trajectory_generated", False),
        "tcp_pose_world_generated": motion_result.get("tcp_pose_world_generated", False),
        "precheck": motion_result.get("precheck", {}),
        "motion": motion_result.get("motion", {}),
        "before_articulation_state": motion_result.get("before_articulation_state", {}),
        "after_articulation_state": motion_result.get("after_articulation_state", {}),
        "simulation_micro_motion_blocking_reasons": motion_result.get("blocking_reasons", []),
        "simulation_micro_motion_warnings": motion_result.get("warnings", []),
        "simulation_micro_motion_errors": motion_result.get("errors", []),
    }


def _semantic_bridge_report_fields(
    *,
    requested: bool = False,
    contract: Dict[str, Any] | None = None,
    contract_path: str | None = None,
    confidence_threshold: float = DEFAULT_SEMANTIC_CONFIDENCE_THRESHOLD,
    joint_name: str = DEFAULT_MICRO_MOTION_JOINT,
    requested_delta_rad: float = DEFAULT_MICRO_MOTION_DELTA_RAD,
    tolerance_rad: float = DEFAULT_MICRO_MOTION_TOLERANCE_RAD,
) -> Dict[str, Any]:
    if not requested:
        bridge = {
            "requested": False,
            "status": SEMANTIC_BRIDGE_STATUS_NOT_REQUESTED,
            "gate_passed": False,
            "blocking_reasons": [],
            "semantic_task_contract_path": None,
            "semantic_task_id": None,
            "semantic_scene_version": None,
            "semantic_intent": None,
            "semantic_user_command": None,
            "semantic_target_label": None,
            "semantic_confidence_overall": None,
            "semantic_confidence_semantic": None,
            "semantic_bbox_xyxy": None,
            "semantic_pixel_center": None,
            "audited_non_executable_fields": [],
            "triggered_simulation_micro_motion": False,
            "simulation_micro_motion_request": None,
            "simulation_only": True,
            "real_robot_allowed": False,
            "safety_boundary": _safety_report_fields({}),
            "semantic_gate": {
                "passed": False,
                "status": SEMANTIC_BRIDGE_STATUS_NOT_REQUESTED,
                "blocking_reasons": [],
            },
        }
    else:
        bridge = build_semantic_simulation_bridge_result(
            request=SemanticSimulationBridgeRequest(
                semantic_task_contract=contract if isinstance(contract, dict) else {},
                semantic_task_contract_path=contract_path,
                confidence_threshold=confidence_threshold,
                joint_name=joint_name,
                requested_delta_rad=requested_delta_rad,
                tolerance_rad=tolerance_rad,
            )
        )
    return _flatten_semantic_bridge_fields(bridge, contract)


def _semantic_bridge_status_from_motion(
    semantic_bridge_metadata: Dict[str, Any] | None,
    simulation_micro_motion_metadata: Dict[str, Any] | None,
) -> Dict[str, Any] | None:
    if not semantic_bridge_metadata or not semantic_bridge_metadata.get("semantic_simulation_bridge_requested"):
        return semantic_bridge_metadata
    if semantic_bridge_metadata.get("semantic_gate_passed") is not True:
        return semantic_bridge_metadata

    status = (simulation_micro_motion_metadata or {}).get("simulation_micro_motion_status")
    if status == MICRO_MOTION_STATUS_BLOCKED_BY_PRECHECK:
        bridge_status = SEMANTIC_BRIDGE_STATUS_BLOCKED_BY_PRECHECK
        reasons = _unique(
            list(semantic_bridge_metadata.get("semantic_bridge_blocking_reasons") or [])
            + list((simulation_micro_motion_metadata or {}).get("simulation_micro_motion_blocking_reasons") or [])
        )
    elif status == MICRO_MOTION_STATUS_FAILED:
        bridge_status = SEMANTIC_BRIDGE_STATUS_FAILED
        reasons = _unique(
            list(semantic_bridge_metadata.get("semantic_bridge_blocking_reasons") or [])
            + list((simulation_micro_motion_metadata or {}).get("simulation_micro_motion_blocking_reasons") or [])
        )
    else:
        bridge_status = semantic_bridge_metadata.get("semantic_bridge_status")
        reasons = list(semantic_bridge_metadata.get("semantic_bridge_blocking_reasons") or [])
    bridge = dict(semantic_bridge_metadata)
    bridge["semantic_bridge_status"] = bridge_status
    bridge["semantic_bridge_blocking_reasons"] = reasons
    if isinstance(bridge.get("semantic_bridge"), dict):
        bridge["semantic_bridge"] = {
            **bridge["semantic_bridge"],
            "status": bridge_status,
            "blocking_reasons": reasons,
        }
    return bridge


def _flatten_semantic_bridge_fields(
    bridge: Dict[str, Any],
    contract: Dict[str, Any] | None = None,
) -> Dict[str, Any]:
    contract_copy = contract if isinstance(contract, dict) and bridge.get("requested") else None
    status = bridge.get("status", SEMANTIC_BRIDGE_STATUS_NOT_REQUESTED)
    return {
        "semantic_simulation_bridge_requested": bridge.get("requested") is True,
        "semantic_bridge_requested": bridge.get("requested") is True,
        "semantic_bridge_status": status,
        "semantic_task_contract_path": bridge.get("semantic_task_contract_path"),
        "semantic_task_id": bridge.get("semantic_task_id"),
        "semantic_scene_version": bridge.get("semantic_scene_version"),
        "semantic_intent": bridge.get("semantic_intent"),
        "semantic_user_command": bridge.get("semantic_user_command"),
        "semantic_target_label": bridge.get("semantic_target_label"),
        "semantic_confidence_overall": bridge.get("semantic_confidence_overall"),
        "semantic_confidence_semantic": bridge.get("semantic_confidence_semantic"),
        "semantic_gate_passed": bridge.get("gate_passed") is True,
        "semantic_bridge_blocking_reasons": list(bridge.get("blocking_reasons") or []),
        "triggered_simulation_micro_motion": bridge.get("triggered_simulation_micro_motion") is True,
        "semantic_audited_non_executable_fields": list(bridge.get("audited_non_executable_fields") or []),
        "semantic_contract_copy": contract_copy,
        "semantic_bridge": bridge,
        "semantic_simulation_bridge_result_path": None,
        "semantic_simulation_bridge_report_path": None,
        "semantic_task_contract_copy_path": None,
        "semantic_bridge_evidence_available": False,
        "semantic_bridge_files": [],
    }


def _semantic_bridge_info(result: Dict[str, Any]) -> Dict[str, Any]:
    bridge = result.get("semantic_bridge") if isinstance(result.get("semantic_bridge"), dict) else {}
    info = {
        **bridge,
        "requested": result.get("semantic_simulation_bridge_requested", bridge.get("requested", False)) is True,
        "status": result.get("semantic_bridge_status", bridge.get("status", SEMANTIC_BRIDGE_STATUS_NOT_REQUESTED)),
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
    return info


def _motion_info(result: Dict[str, Any]) -> Dict[str, Any]:
    motion = result.get("motion") if isinstance(result.get("motion"), dict) else {}
    return {
        **motion,
        "simulation_micro_motion_requested": result.get("simulation_micro_motion_requested", False),
        "simulation_micro_motion_status": result.get(
            "simulation_micro_motion_status",
            MICRO_MOTION_STATUS_NOT_REQUESTED,
        ),
        "simulation_only": result.get("simulation_only", True),
        "real_robot_allowed": result.get("real_robot_allowed", False),
        "real_robot_motion_executed": result.get("real_robot_motion_executed", False),
        "command_type": motion.get("command_type"),
        "joint_name": motion.get("joint_name"),
        "requested_delta_rad": motion.get("requested_delta_rad"),
        "actual_delta_rad": motion.get("actual_delta_rad"),
        "tolerance_rad": motion.get("tolerance_rad"),
        "delta_within_tolerance": motion.get("delta_within_tolerance", False),
    }


def _precheck_info(result: Dict[str, Any]) -> Dict[str, Any]:
    precheck = result.get("simulation_motion_precheck")
    if not isinstance(precheck, dict):
        precheck = {}
    return {
        "simulation_motion_precheck_status": result.get(
            "simulation_motion_precheck_status",
            precheck.get("status", "NOT_REQUESTED"),
        ),
        "ready_for_simulation_motion": result.get("ready_for_simulation_motion", precheck.get("ready", False)),
        "blocking_reasons": precheck.get("blocking_reasons", []),
        "warnings": precheck.get("warnings", []),
        "errors": precheck.get("errors", []),
    }


def _simulated_task_execution_request_fields(
    *,
    requested: bool = False,
    execution_attempt_id: str | None = None,
    execution_max_attempts: int = 1,
    retry_enabled: bool = False,
    fallback_enabled: bool = False,
) -> Dict[str, Any]:
    return {
        "safe_simulated_task_execution_requested": requested,
        "execution_attempt_id": execution_attempt_id,
        "execution_max_attempts": int(execution_max_attempts or 1),
        "execution_attempt_index": 1,
        "execution_retry_recommendation_enabled": retry_enabled,
        "execution_fallback_recommendation_enabled": fallback_enabled,
    }


def _simulated_task_execution_report_fields(
    result: Dict[str, Any],
    request_metadata: Dict[str, Any] | None = None,
) -> Dict[str, Any]:
    request_metadata = request_metadata if isinstance(request_metadata, dict) else {}
    request = SimulatedTaskExecutionRequest(
        requested=request_metadata.get("safe_simulated_task_execution_requested") is True,
        execution_attempt_id=request_metadata.get("execution_attempt_id"),
        execution_max_attempts=int(request_metadata.get("execution_max_attempts") or 1),
        execution_attempt_index=int(request_metadata.get("execution_attempt_index") or 1),
        retry_recommendation_enabled=request_metadata.get("execution_retry_recommendation_enabled") is True,
        fallback_recommendation_enabled=request_metadata.get("execution_fallback_recommendation_enabled") is True,
    )
    execution_result = execute_safe_simulated_task(request, result)
    post_check = execution_result.get("post_motion_state_check") if isinstance(execution_result.get("post_motion_state_check"), dict) else {}
    flattened = {
        **execution_result,
        "simulated_task_execution": execution_result,
        "post_motion_state_check": post_check,
        "post_motion_state_check_status": post_check.get("post_motion_state_check_status"),
        "post_check_passed": post_check.get("post_check_passed", False),
        "execution_feedback": execution_result.get("execution_feedback", {}),
        "failure_analysis": execution_result.get("failure_analysis", {}),
        "retry_fallback_recommendation": execution_result.get("retry_fallback_recommendation", {}),
        "execution_attempt_record": execution_result.get("execution_attempt_record", {}),
        "simulated_task_execution_files": [],
    }
    return flattened


def _simulated_task_execution_info(result: Dict[str, Any]) -> Dict[str, Any]:
    execution = result.get("simulated_task_execution") if isinstance(result.get("simulated_task_execution"), dict) else {}
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
        "simulated_task_execution_result_path": result.get("simulated_task_execution_result_path"),
        "simulated_task_execution_report_path": result.get("simulated_task_execution_report_path"),
        "execution_feedback_path": result.get("execution_feedback_path"),
        "execution_attempt_record_path": result.get("execution_attempt_record_path"),
        "failure_analysis_path": result.get("failure_analysis_path"),
        "retry_fallback_recommendation_path": result.get("retry_fallback_recommendation_path"),
    }


def _simulated_task_execution_file_refs(result: Dict[str, Any]) -> list[Dict[str, str | None]]:
    if not result.get("safe_simulated_task_execution_requested"):
        return []
    return [
        {"name": "simulated_task_execution_result.json", "path": result.get("simulated_task_execution_result_path")},
        {"name": "simulated_task_execution_report.md", "path": result.get("simulated_task_execution_report_path")},
        {"name": "execution_feedback.json", "path": result.get("execution_feedback_path")},
        {"name": "execution_attempt_record.json", "path": result.get("execution_attempt_record_path")},
        {"name": "failure_analysis.json", "path": result.get("failure_analysis_path")},
        {
            "name": "retry_fallback_recommendation.json",
            "path": result.get("retry_fallback_recommendation_path"),
        },
    ]


def _semantic_bridge_file_refs(result: Dict[str, Any]) -> list[Dict[str, str | None]]:
    if not result.get("semantic_simulation_bridge_requested"):
        return []
    return [
        {
            "name": "semantic_simulation_bridge_result.json",
            "path": result.get("semantic_simulation_bridge_result_path"),
        },
        {
            "name": "semantic_simulation_bridge_report.md",
            "path": result.get("semantic_simulation_bridge_report_path"),
        },
        {
            "name": "semantic_task_contract_copy.json",
            "path": result.get("semantic_task_contract_copy_path"),
        },
    ]


def _safety_report_fields(result: Dict[str, Any]) -> Dict[str, bool]:
    return {
        "no_live_camera_used": True,
        "no_live_vlm_used": True,
        "no_ros2_used": True,
        "no_moveit_used": True,
        "no_rtde_used": True,
        "no_urscript_used": True,
        "no_dashboard_used": True,
        "no_real_ur5_used": True,
        "no_trajectory_generated": result.get("trajectory_generated", False) is not True,
        "no_tcp_pose_world_executed": True,
        "simulation_only": result.get("simulation_only", True) is True,
        "real_robot_allowed": False,
        "real_robot_motion_executed": False,
    }


def _make_isaac_micro_motion_executor(
    world,
    *,
    robot_asset_spec: RobotAssetSpec,
    headless: bool,
    articulation_state_observer,
    robot_prim_inspection: Dict[str, Any] | None,
    articulation_readiness: Dict[str, Any] | None,
):
    def executor(request: SimulationMicroMotionRequest, before_state: Dict[str, Any]) -> Dict[str, Any]:
        return _execute_micro_motion_via_isaac_api(
            world,
            request=request,
            robot_prim_path=robot_asset_spec.robot_prim_path,
            before_state=before_state,
            headless=headless,
            articulation_state_observer=articulation_state_observer,
            robot_prim_inspection=robot_prim_inspection,
            articulation_readiness=articulation_readiness,
        )

    return executor


def _execute_micro_motion_via_isaac_api(
    world,
    *,
    request: SimulationMicroMotionRequest,
    robot_prim_path: str,
    before_state: Dict[str, Any],
    headless: bool,
    articulation_state_observer,
    robot_prim_inspection: Dict[str, Any] | None,
    articulation_readiness: Dict[str, Any] | None,
) -> Dict[str, Any]:
    handle = _create_isaac_articulation_handle(world, robot_prim_path)
    joint_names = _isaac_joint_names(handle) or _joint_names_from_state(before_state)
    if request.joint_name not in joint_names:
        raise RuntimeError(f"joint not found in Isaac articulation: {request.joint_name}")
    before_positions = _isaac_joint_positions(handle) or _joint_positions_from_state(before_state, joint_names)
    joint_index = joint_names.index(request.joint_name)
    if joint_index >= len(before_positions):
        raise RuntimeError(f"joint position unavailable for: {request.joint_name}")
    target_positions = list(before_positions)
    target_positions[joint_index] = float(target_positions[joint_index]) + float(request.requested_delta_rad)
    before_api_state = _state_from_joint_positions(
        before_state,
        joint_names=joint_names,
        joint_positions=before_positions,
        robot_prim_path=robot_prim_path,
        robot_prim_inspection=robot_prim_inspection,
        articulation_readiness=articulation_readiness,
    )
    _apply_isaac_joint_positions(handle, target_positions)
    if hasattr(world, "step"):
        world.step(render=not headless)
    after_positions = _isaac_joint_positions(handle) or target_positions
    after_api_state = _state_from_joint_positions(
        before_state,
        joint_names=joint_names,
        joint_positions=after_positions,
        robot_prim_path=robot_prim_path,
        robot_prim_inspection=robot_prim_inspection,
        articulation_readiness=articulation_readiness,
    )
    articulation_state_observer(
        world,
        robot_prim_path=robot_prim_path,
        robot_prim_inspection=robot_prim_inspection,
        articulation_readiness=articulation_readiness,
    )
    return {
        "before_articulation_state": before_api_state,
        "after_articulation_state": after_api_state,
    }


def _create_isaac_articulation_handle(world, robot_prim_path: str):
    scene = getattr(world, "scene", None)
    if scene is not None:
        get_object = getattr(scene, "get_object", None)
        if callable(get_object):
            existing = get_object("teto_micro_motion_articulation")
            if existing is not None:
                return existing

    articulation_cls = None
    for module_name, class_name in (
        ("isaacsim.core.prims", "SingleArticulation"),
        ("isaacsim.core.prims", "Articulation"),
        ("omni.isaac.core.articulations", "Articulation"),
    ):
        try:
            module = __import__(module_name, fromlist=[class_name])
            articulation_cls = getattr(module, class_name)
            break
        except Exception:
            articulation_cls = None
    if articulation_cls is None:
        raise RuntimeError("Isaac articulation API is unavailable")

    try:
        handle = articulation_cls(prim_path=robot_prim_path, name="teto_micro_motion_articulation")
    except TypeError:
        handle = articulation_cls(prim_paths_expr=robot_prim_path, name="teto_micro_motion_articulation")

    if scene is not None and hasattr(scene, "add"):
        try:
            handle = scene.add(handle)
        except Exception:
            pass
    initialize = getattr(handle, "initialize", None)
    if callable(initialize):
        try:
            initialize()
        except Exception:
            pass
    return handle


def _isaac_joint_names(handle) -> list[str]:
    for attribute_name in ("dof_names", "joint_names"):
        names = getattr(handle, attribute_name, None)
        if callable(names):
            names = names()
        if isinstance(names, (list, tuple)):
            return [str(name) for name in names]
    get_dof_names = getattr(handle, "get_dof_names", None)
    if callable(get_dof_names):
        names = get_dof_names()
        if isinstance(names, (list, tuple)):
            return [str(name) for name in names]
    return []


def _isaac_joint_positions(handle) -> list[float]:
    get_joint_positions = getattr(handle, "get_joint_positions", None)
    if not callable(get_joint_positions):
        return []
    positions = get_joint_positions()
    return _float_list(positions)


def _apply_isaac_joint_positions(handle, joint_positions: list[float]) -> None:
    set_joint_positions = getattr(handle, "set_joint_positions", None)
    if callable(set_joint_positions):
        set_joint_positions(joint_positions)
        return

    action_cls = None
    for module_name in ("isaacsim.core.utils.types", "omni.isaac.core.utils.types"):
        try:
            module = __import__(module_name, fromlist=["ArticulationAction"])
            action_cls = getattr(module, "ArticulationAction")
            break
        except Exception:
            action_cls = None
    if action_cls is not None:
        action = action_cls(joint_positions=joint_positions)
        controller = getattr(handle, "get_articulation_controller", lambda: None)()
        if controller is not None and hasattr(controller, "apply_action"):
            controller.apply_action(action)
            return
        apply_action = getattr(handle, "apply_action", None)
        if callable(apply_action):
            apply_action(action)
            return
    raise RuntimeError("Isaac articulation handle cannot apply local simulation joint positions")


def _state_from_joint_positions(
    fallback_state: Dict[str, Any],
    *,
    joint_names: list[str],
    joint_positions: list[float],
    robot_prim_path: str,
    robot_prim_inspection: Dict[str, Any] | None,
    articulation_readiness: Dict[str, Any] | None,
) -> Dict[str, Any]:
    fallback_rows = {
        row.get("joint_name"): row for row in fallback_state.get("joint_state_table") or [] if isinstance(row, dict)
    }
    rows = []
    for index, joint_name in enumerate(joint_names):
        fallback_row = fallback_rows.get(joint_name, {})
        rows.append(
            {
                **fallback_row,
                "joint_name": joint_name,
                "category": fallback_row.get("category") or ("arm" if joint_name in set(UR5E_ARM_JOINT_NAMES) else "unknown"),
                "position": _list_get(joint_positions, index),
                "velocity": fallback_row.get("velocity", 0.0),
                "lower_limit": fallback_row.get("lower_limit"),
                "upper_limit": fallback_row.get("upper_limit"),
            }
        )
    return build_articulation_state_report(
        requested=True,
        robot_prim_path=robot_prim_path,
        robot_prim_inspection=robot_prim_inspection,
        articulation_readiness=articulation_readiness,
        joint_state_table=rows,
        status="OK",
    )


def _joint_names_from_state(state: Dict[str, Any]) -> list[str]:
    return [row.get("joint_name") for row in state.get("joint_state_table") or [] if isinstance(row, dict)]


def _joint_positions_from_state(state: Dict[str, Any], joint_names: list[str]) -> list[float]:
    row_by_name = {row.get("joint_name"): row for row in state.get("joint_state_table") or [] if isinstance(row, dict)}
    positions = []
    for joint_name in joint_names:
        value = row_by_name.get(joint_name, {}).get("position")
        if not isinstance(value, (int, float)) or isinstance(value, bool):
            raise RuntimeError(f"joint position unavailable for: {joint_name}")
        positions.append(float(value))
    return positions


def _joint_position_from_state(state: Dict[str, Any], joint_name: str) -> float | None:
    for row in state.get("joint_state_table") or []:
        if isinstance(row, dict) and row.get("joint_name") == joint_name:
            value = row.get("position")
            return float(value) if isinstance(value, (int, float)) and not isinstance(value, bool) else None
    return None


def _float_list(values: Any) -> list[float]:
    if values is None:
        return []
    if hasattr(values, "tolist"):
        values = values.tolist()
    if not isinstance(values, (list, tuple)):
        return []
    return [float(value) for value in values]


def _list_get(values: list[Any], index: int, default=None):
    return values[index] if index < len(values) else default


def _dry_run_robot_prim_inspection_report_fields(
    *,
    spec: RobotAssetSpec,
    requested: bool,
    prim_exists: bool,
) -> Dict[str, Any]:
    if not requested:
        return _robot_prim_inspection_report_fields(spec=spec, requested=False)
    if prim_exists:
        return _robot_prim_inspection_report_fields(
            spec=spec,
            requested=True,
            inspection=build_robot_prim_inspection_report(
                requested=True,
                robot_prim_path=spec.robot_prim_path,
                robot_prim_exists=True,
                robot_root_type_name="DryRunRobot",
                inspection_warnings=["Dry-run simulated robot prim inspection; no Isaac stage was read."],
            ),
        )
    return _robot_prim_inspection_report_fields(
        spec=spec,
        requested=True,
        inspection=build_robot_prim_inspection_report(
            requested=True,
            robot_prim_path=spec.robot_prim_path,
            robot_prim_exists=False,
        ),
    )


def _resolve_default_robot_asset_path(
    robot_asset_path: str | None,
    *,
    dry_run: bool,
    no_isaac: bool,
    check_robot_asset: bool,
    load_robot_asset: bool,
    inspect_robot_prim: bool,
    check_articulation_readiness: bool,
    observe_articulation_state: bool,
    check_simulation_motion_precheck: bool,
) -> str | None:
    if robot_asset_path or dry_run or no_isaac or load_robot_asset or not check_robot_asset:
        return robot_asset_path
    if not (
        inspect_robot_prim
        or check_articulation_readiness
        or observe_articulation_state
        or check_simulation_motion_precheck
    ):
        return robot_asset_path
    local_path = Path(DEFAULT_LOCAL_UR5E_ASSET_PATH)
    return str(local_path) if local_path.is_file() else robot_asset_path


def _position_tuple(position: list[float] | tuple[float, float, float]) -> tuple[float, float, float]:
    values = [float(value) for value in position]
    if len(values) != 3:
        raise ValueError("position must contain exactly 3 values")
    return values[0], values[1], values[2]


def _position_list(position: list[float] | tuple[float, float, float]) -> list[float]:
    return [float(value) for value in position]


def _displacement(initial_position: list[float] | None, final_position: list[float] | None) -> list[float] | None:
    if initial_position is None or final_position is None:
        return None
    return [final - initial for initial, final in zip(initial_position, final_position)]


def _create_run_dir() -> Path:
    SIMULATION_RUNS_ROOT.mkdir(parents=True, exist_ok=True)
    while True:
        run_name = datetime.now().strftime("run_%Y%m%d_%H%M%S")
        run_dir = SIMULATION_RUNS_ROOT / run_name
        try:
            run_dir.mkdir()
            return run_dir
        except FileExistsError:
            time.sleep(0.05)


def _mode_name(*, dry_run: bool, no_isaac: bool) -> str:
    if dry_run:
        return "dry_run"
    if no_isaac:
        return "no_isaac"
    return "isaac"


def _is_network_asset_path(asset_path: str) -> bool:
    lowered = asset_path.lower()
    return lowered.startswith(("http://", "https://", "omniverse://", "ov://"))


def _missing_task_fields(task: Dict[str, Any]) -> list[str]:
    required_fields = ("task_type", "target_label", "target_world_point", "scene_version", "ttl_ms")
    if not isinstance(task, dict):
        return list(required_fields)
    return [field for field in required_fields if task.get(field) in (None, "")]


def _timestamp() -> str:
    return datetime.now().strftime("%Y-%m-%d %H:%M:%S")
