from __future__ import annotations

import json
import time
from datetime import datetime
from pathlib import Path
from typing import Any, Dict


REPORT_VERSION = "teto_simulation_execution.v1"
CURRENT_TETO_VERSION = "TETO V2.0.1"
DEFAULT_STEPS = 5
DEFAULT_CUBE_PRIM_PATH = "/World/TETO_Cube"
DEFAULT_CUBE_POSITION = [0.0, 0.0, 0.5]
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
    cube_metadata: Dict[str, Any] | None = None,
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
    result.update(cube_metadata or _cube_report_fields(cube_spawned=False))
    return result


def run_first_simulation_execution(
    simulation_task: Dict[str, Any] | None = None,
    *,
    dry_run: bool = False,
    no_isaac: bool = False,
    steps: int = DEFAULT_STEPS,
    headless: bool = True,
    spawn_cube: bool = False,
    cube_prim_path: str = DEFAULT_CUBE_PRIM_PATH,
    cube_position: list[float] | tuple[float, float, float] = tuple(DEFAULT_CUBE_POSITION),
    cube_size: float = DEFAULT_CUBE_SIZE,
    output_dir: str | Path | None = None,
    write_report: bool = False,
) -> Dict[str, Any]:
    task = simulation_task or dict(DEFAULT_SIMULATION_TASK)
    started_at = _timestamp()

    if steps <= 0:
        return _finalize_result(
            build_simulation_execution_result(
                simulation_task=task,
                status="FAIL",
                mode=_mode_name(dry_run=dry_run, no_isaac=no_isaac),
                steps_requested=steps,
                cube_metadata=_cube_report_fields(
                    cube_spawned=False,
                    prim_path=cube_prim_path if spawn_cube else None,
                    position=cube_position if spawn_cube else None,
                    size=cube_size if spawn_cube else None,
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
                cube_metadata=_cube_report_fields(
                    cube_spawned=False,
                    prim_path=cube_prim_path if spawn_cube else None,
                    position=cube_position if spawn_cube else None,
                    size=cube_size if spawn_cube else None,
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
                cube_metadata=_cube_report_fields(
                    cube_spawned=spawn_cube,
                    prim_path=cube_prim_path if spawn_cube else None,
                    position=cube_position if spawn_cube else None,
                    size=cube_size if spawn_cube else None,
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
        spawn_cube=spawn_cube,
        cube_prim_path=cube_prim_path,
        cube_position=cube_position,
        cube_size=cube_size,
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
    spawn_cube: bool,
    cube_prim_path: str,
    cube_position: list[float] | tuple[float, float, float],
    cube_size: float,
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
                cube_metadata=_cube_report_fields(
                    cube_spawned=False,
                    prim_path=cube_prim_path if spawn_cube else None,
                    position=cube_position if spawn_cube else None,
                    size=cube_size if spawn_cube else None,
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
        spawn_cube=spawn_cube,
        cube_prim_path=cube_prim_path,
        cube_position=cube_position,
        cube_size=cube_size,
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
    spawn_cube: bool,
    cube_prim_path: str,
    cube_position: list[float] | tuple[float, float, float],
    cube_size: float,
    started_at: str,
    output_dir: str | Path | None,
    write_report: bool,
    cube_spawner=None,
) -> Dict[str, Any]:
    simulation_app = None
    cube_spawner = cube_spawner or _spawn_cube_in_world
    cube_metadata = _cube_report_fields(cube_spawned=False)
    world_reset = False
    try:
        simulation_app = simulation_app_cls({"headless": headless})
        if world_cls is None:
            world_cls = _load_isaac_world_class()
        world = world_cls()
        world.reset()
        world_reset = True

        if spawn_cube:
            try:
                cube_metadata = cube_spawner(
                    world,
                    prim_path=cube_prim_path,
                    position=cube_position,
                    size=cube_size,
                )
            except Exception as exc:
                cube_metadata = _cube_report_fields(
                    cube_spawned=False,
                    prim_path=cube_prim_path,
                    position=cube_position,
                    size=cube_size,
                )
                return _finalize_result(
                    build_simulation_execution_result(
                        simulation_task=simulation_task,
                        status="FAIL",
                        mode="isaac",
                        steps_requested=steps,
                        world_reset=world_reset,
                        cube_metadata=cube_metadata,
                        error_code="E_CUBE_SPAWN_FAILED",
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
                cube_metadata=cube_metadata,
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
                cube_metadata=cube_metadata,
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


def _spawn_cube_in_world(
    world,
    *,
    prim_path: str,
    position: list[float] | tuple[float, float, float],
    size: float,
) -> Dict[str, Any]:
    try:
        from isaacsim.core.api.objects import VisualCuboid
    except ImportError:
        from omni.isaac.core.objects import VisualCuboid

    cube = VisualCuboid(
        prim_path=prim_path,
        name="teto_cube",
        position=list(position),
        size=size,
    )
    if hasattr(world, "scene") and hasattr(world.scene, "add"):
        world.scene.add(cube)
    return _cube_report_fields(
        cube_spawned=True,
        prim_path=prim_path,
        position=position,
        size=size,
    )


def _cube_report_fields(
    *,
    cube_spawned: bool,
    prim_path: str | None = None,
    position: list[float] | tuple[float, float, float] | None = None,
    size: float | None = None,
) -> Dict[str, Any]:
    normalized_position = [float(value) for value in position] if position is not None else None
    normalized_size = float(size) if size is not None else None
    object_type = "cube" if cube_spawned or prim_path or position is not None or size is not None else None
    return {
        "simulation_object_spawned": cube_spawned,
        "object_type": object_type,
        "cube_prim_path": prim_path,
        "cube_position": normalized_position,
        "cube_size": normalized_size,
        "cube_spawned": cube_spawned,
    }


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
