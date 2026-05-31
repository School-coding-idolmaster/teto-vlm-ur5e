from __future__ import annotations

import json
import time
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Any, Dict

from src.evidence_exporter import export_simulation_evidence
from src.robot_prim_inspector import build_robot_prim_inspection_report, inspect_robot_prim as inspect_robot_prim_in_stage


REPORT_VERSION = "teto_simulation_execution.v1"
CURRENT_TETO_VERSION = "TETO V2.1.1"
DEFAULT_STEPS = 5
DEFAULT_SIMULATION_OBJECT_TYPE = "cube"
DEFAULT_CUBE_PRIM_PATH = "/World/TETO_Cube"
DEFAULT_CUBE_POSITION = [0.0, 0.0, 0.5]
DEFAULT_CUBE_TARGET_POSITION = [0.3, 0.0, 0.5]
DEFAULT_CUBE_SIZE = 0.2
DEFAULT_ROBOT_TYPE = "ur5"
DEFAULT_ROBOT_PRIM_PATH = "/World/TETO_Robot"
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
    started_at: str | None = None,
    finished_at: str | None = None,
) -> Dict[str, Any]:
    blocking_reasons = []
    if status != "PASS":
        blocking_reasons.append(error_code if error_code != "OK" else "E_SIMULATION_EXECUTION_FAILED")

    result = {
        "report_version": REPORT_VERSION,
        "teto_version": CURRENT_TETO_VERSION,
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
    output_dir: str | Path | None = None,
    write_report: bool = False,
    demo_command: str | None = None,
) -> Dict[str, Any]:
    task = simulation_task or dict(DEFAULT_SIMULATION_TASK)
    started_at = _timestamp()
    effective_move_object = move_object or move_cube
    effective_spawn_cube = spawn_cube or effective_move_object
    simulation_object_spec = object_spec or SimulationObjectSpec(
        object_type=DEFAULT_SIMULATION_OBJECT_TYPE,
        prim_path=cube_prim_path,
        initial_position=_position_tuple(cube_position),
        target_position=_position_tuple(cube_target_position),
        size=cube_size,
    )
    effective_check_robot_asset = check_robot_asset or load_robot_asset
    effective_robot_asset_spec = robot_asset_spec or RobotAssetSpec(
        robot_type=robot_type,
        robot_prim_path=robot_prim_path,
        robot_asset_path=robot_asset_path,
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
                    load_requested=load_robot_asset,
                ),
                robot_prim_inspection_metadata=_robot_prim_inspection_report_fields(
                    spec=effective_robot_asset_spec,
                    requested=inspect_robot_prim,
                ),
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
                    load_requested=load_robot_asset,
                ),
                robot_prim_inspection_metadata=_robot_prim_inspection_report_fields(
                    spec=effective_robot_asset_spec,
                    requested=inspect_robot_prim,
                ),
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
                load_requested=load_robot_asset,
                dry_run=True,
            )
            if effective_check_robot_asset
            else _robot_asset_report_fields()
        )
        robot_prim_inspection_metadata = _dry_run_robot_prim_inspection_report_fields(
            spec=effective_robot_asset_spec,
            requested=inspect_robot_prim,
            prim_exists=bool(robot_asset_metadata.get("robot_prim_exists")),
        )
        if load_robot_asset and not robot_asset_metadata["robot_asset_available"]:
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
        load_robot_asset=load_robot_asset,
        robot_asset_spec=effective_robot_asset_spec,
        inspect_robot_prim=inspect_robot_prim,
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
    result["report_path"] = str(report_path)
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
    demo_command: str | None = None,
    object_spawner=None,
    object_pose_updater=None,
    robot_asset_loader=None,
    robot_prim_verifier=None,
    robot_prim_inspector=None,
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
    robot_prim_inspection_metadata = _robot_prim_inspection_report_fields(
        spec=robot_asset_spec,
        requested=inspect_robot_prim,
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
