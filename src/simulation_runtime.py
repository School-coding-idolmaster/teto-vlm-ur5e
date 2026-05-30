from __future__ import annotations

import json
import time
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Any, Dict


REPORT_VERSION = "teto_simulation_execution.v1"
CURRENT_TETO_VERSION = "TETO V2.0.2"
DEFAULT_STEPS = 5
DEFAULT_SIMULATION_OBJECT_TYPE = "cube"
DEFAULT_CUBE_PRIM_PATH = "/World/TETO_Cube"
DEFAULT_CUBE_POSITION = [0.0, 0.0, 0.5]
DEFAULT_CUBE_TARGET_POSITION = [0.3, 0.0, 0.5]
DEFAULT_CUBE_SIZE = 0.2
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
    output_dir: str | Path | None = None,
    write_report: bool = False,
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
                error_code="E_INVALID_STEPS",
                error_message="steps must be a positive integer",
                started_at=started_at,
                finished_at=_timestamp(),
            ),
            output_dir=output_dir,
            write_report=write_report,
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
                error_code="E_INVALID_SIMULATION_TASK",
                error_message=f"missing simulation task fields: {', '.join(missing_fields)}",
                started_at=started_at,
                finished_at=_timestamp(),
            ),
            output_dir=output_dir,
            write_report=write_report,
        )

    if dry_run or no_isaac:
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
                started_at=started_at,
                finished_at=_timestamp(),
            ),
            output_dir=output_dir,
            write_report=write_report,
        )

    return _run_true_isaac_runtime(
        simulation_task=task,
        steps=steps,
        headless=headless,
        spawn_object=effective_spawn_cube,
        move_object=effective_move_object,
        object_spec=simulation_object_spec,
        started_at=started_at,
        output_dir=output_dir,
        write_report=write_report,
    )


def write_simulation_execution_result(
    result: Dict[str, Any],
    output_dir: str | Path | None = None,
) -> Path:
    run_dir = Path(output_dir).expanduser() if output_dir else _create_run_dir()
    run_dir.mkdir(parents=True, exist_ok=True)
    report_path = run_dir / "simulation_execution_result.json"
    result["report_path"] = str(report_path)
    with report_path.open("w", encoding="utf-8") as report_file:
        json.dump(result, report_file, ensure_ascii=False, indent=2)
        report_file.write("\n")
    return report_path


def _finalize_result(
    result: Dict[str, Any],
    *,
    output_dir: str | Path | None,
    write_report: bool,
) -> Dict[str, Any]:
    if write_report:
        write_simulation_execution_result(result, output_dir)
    return result


def _run_true_isaac_runtime(
    *,
    simulation_task: Dict[str, Any],
    steps: int,
    headless: bool,
    spawn_object: bool,
    move_object: bool,
    object_spec: SimulationObjectSpec,
    started_at: str,
    output_dir: str | Path | None,
    write_report: bool,
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
                error_code="E_ISAAC_RUNTIME_FAILED",
                error_message=str(exc),
                started_at=started_at,
                finished_at=_timestamp(),
            ),
            output_dir=output_dir,
            write_report=write_report,
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
        started_at=started_at,
        output_dir=output_dir,
        write_report=write_report,
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
    object_spawner=None,
    object_pose_updater=None,
) -> Dict[str, Any]:
    simulation_app = None
    object_spawner = object_spawner or spawn_simulation_object
    object_pose_updater = object_pose_updater or update_simulation_object_pose
    object_handle = None
    object_metadata = _simulation_object_report_fields()
    world_reset = False
    try:
        simulation_app = simulation_app_cls({"headless": headless})
        if world_cls is None:
            world_cls = _load_isaac_world_class()
        world = world_cls()
        world.reset()
        world_reset = True

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
                        error_code="E_CUBE_SPAWN_FAILED",
                        error_message=str(exc),
                        started_at=started_at,
                        finished_at=_timestamp(),
                    ),
                    output_dir=output_dir,
                    write_report=write_report,
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
                        error_code="E_SIM_OBJECT_MOVE_FAILED",
                        error_message=str(exc),
                        started_at=started_at,
                        finished_at=_timestamp(),
                    ),
                    output_dir=output_dir,
                    write_report=write_report,
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
                started_at=started_at,
                finished_at=_timestamp(),
            ),
            output_dir=output_dir,
            write_report=write_report,
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
                error_code="E_ISAAC_RUNTIME_FAILED",
                error_message=str(exc),
                started_at=started_at,
                finished_at=_timestamp(),
            ),
            output_dir=output_dir,
            write_report=write_report,
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


def _missing_task_fields(task: Dict[str, Any]) -> list[str]:
    required_fields = ("task_type", "target_label", "target_world_point", "scene_version", "ttl_ms")
    if not isinstance(task, dict):
        return list(required_fields)
    return [field for field in required_fields if task.get(field) in (None, "")]


def _timestamp() -> str:
    return datetime.now().strftime("%Y-%m-%d %H:%M:%S")
