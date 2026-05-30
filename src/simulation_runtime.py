from __future__ import annotations

import json
import time
from datetime import datetime
from pathlib import Path
from typing import Any, Dict


REPORT_VERSION = "teto_simulation_execution.v1"
CURRENT_TETO_VERSION = "TETO V2.0.0"
DEFAULT_STEPS = 5
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
    started_at: str | None = None,
    finished_at: str | None = None,
) -> Dict[str, Any]:
    blocking_reasons = []
    if status != "PASS":
        blocking_reasons.append(error_code if error_code != "OK" else "E_SIMULATION_EXECUTION_FAILED")

    return {
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


def run_first_simulation_execution(
    simulation_task: Dict[str, Any] | None = None,
    *,
    dry_run: bool = False,
    no_isaac: bool = False,
    steps: int = DEFAULT_STEPS,
    headless: bool = True,
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
                started_at=started_at,
                finished_at=_timestamp(),
            ),
            output_dir=output_dir,
            write_report=write_report,
        )

    simulation_app = None
    try:
        from isaacsim import SimulationApp

        simulation_app = SimulationApp({"headless": headless})

        from omni.isaac.core import World

        world = World()
        world.reset()
        steps_completed = 0
        for _ in range(steps):
            world.step(render=not headless)
            steps_completed += 1

        return _finalize_result(
            build_simulation_execution_result(
                simulation_task=task,
                status="PASS",
                mode="isaac",
                steps_requested=steps,
                steps_completed=steps_completed,
                world_reset=True,
                started_at=started_at,
                finished_at=_timestamp(),
            ),
            output_dir=output_dir,
            write_report=write_report,
        )
    except Exception as exc:
        return _finalize_result(
            build_simulation_execution_result(
                simulation_task=task,
                status="FAIL",
                mode="isaac",
                steps_requested=steps,
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
