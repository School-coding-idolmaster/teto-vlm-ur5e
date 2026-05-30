import argparse
import json
import sys
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from src.simulation_runtime import DEFAULT_SIMULATION_TASK, run_first_simulation_execution


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Run TETO V2.0.2 simulation object pose update smoke test.")
    parser.add_argument("--dry-run", action="store_true", help="Do not import Isaac; produce a test execution report.")
    parser.add_argument("--no-isaac", action="store_true", help="Pure Python test mode without Isaac imports.")
    parser.add_argument("--spawn-cube", action="store_true", help="Spawn a visible cube in the Isaac World.")
    parser.add_argument(
        "--move-object",
        action="store_true",
        help="Run the default simulation object pose update smoke test.",
    )
    parser.add_argument(
        "--move-cube",
        action="store_true",
        help="Alias for --move-object using the default cube fixture.",
    )
    parser.add_argument("--task-json", help="Path to a simulation_task JSON file.")
    parser.add_argument("--steps", type=int, default=5, help="Number of simulation steps to run.")
    parser.add_argument("--gui", action="store_true", help="Run Isaac with GUI instead of headless mode.")
    parser.add_argument("--output-dir", help="Directory where simulation_execution_result.json is written.")
    return parser


def main() -> int:
    args = build_parser().parse_args()
    simulation_task = _load_simulation_task(args.task_json)
    result = run_first_simulation_execution(
        simulation_task,
        dry_run=args.dry_run,
        no_isaac=args.no_isaac,
        steps=args.steps,
        headless=not args.gui,
        spawn_cube=args.spawn_cube,
        move_object=args.move_object,
        move_cube=args.move_cube,
        output_dir=args.output_dir,
        write_report=True,
    )
    report_path = Path(str(result["report_path"]))
    print_summary(result, report_path)
    return 0 if result.get("ok") else 1


def print_summary(result: dict, report_path: Path) -> None:
    print("=" * 50)
    print("TETO V2.0.2 SIMULATION OBJECT POSE UPDATE")
    print("=" * 50)
    print(f"Status: {result['status']}")
    print(f"Mode: {result['mode']}")
    print(f"World reset: {result['world_reset']}")
    print(f"Steps: {result['steps_completed']}/{result['steps_requested']}")
    print(f"allow_robot_motion: {result['allow_robot_motion']}")
    print(f"cube_spawned: {result.get('cube_spawned')}")
    print(f"cube_prim_path: {result.get('cube_prim_path')}")
    print(f"cube_position: {result.get('cube_position')}")
    print(f"cube_size: {result.get('cube_size')}")
    print(f"cube_moved: {result.get('cube_moved')}")
    print(f"cube_initial_position: {result.get('cube_initial_position')}")
    print(f"cube_target_position: {result.get('cube_target_position')}")
    print(f"cube_final_position: {result.get('cube_final_position')}")
    print(f"cube_displacement: {result.get('cube_displacement')}")
    print(f"Report: {report_path}")
    if result.get("blocking_reasons"):
        print(f"Blocking reasons: {', '.join(result['blocking_reasons'])}")
    error = result.get("error", {})
    if error.get("code") != "OK":
        print(f"Error: {error.get('code')}: {error.get('message')}")
    print("=" * 50)


def _load_simulation_task(task_json: str | None) -> dict:
    if not task_json:
        return dict(DEFAULT_SIMULATION_TASK)
    path = Path(task_json).expanduser()
    with path.open("r", encoding="utf-8") as task_file:
        data = json.load(task_file)
    if isinstance(data, dict) and isinstance(data.get("simulation_task"), dict):
        return data["simulation_task"]
    if isinstance(data, dict):
        return data
    raise ValueError("simulation task JSON must contain an object")


if __name__ == "__main__":
    raise SystemExit(main())
