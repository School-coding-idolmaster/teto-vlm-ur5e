import argparse
import json
import sys
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from src.simulation_runtime import DEFAULT_SIMULATION_TASK, run_first_simulation_execution


SIMULATION_RUNS_ROOT = PROJECT_ROOT / "outputs" / "simulation_runs"


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Run TETO V2.0.0 First Simulation Execution.")
    parser.add_argument("--dry-run", action="store_true", help="Do not import Isaac; produce a test execution report.")
    parser.add_argument("--no-isaac", action="store_true", help="Pure Python test mode without Isaac imports.")
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
    )
    report_path = _write_report(result, args.output_dir)
    print_summary(result, report_path)
    return 0 if result.get("ok") else 1


def print_summary(result: dict, report_path: Path) -> None:
    print("=" * 50)
    print("TETO V2.0.0 FIRST SIMULATION EXECUTION")
    print("=" * 50)
    print(f"Status: {result['status']}")
    print(f"Mode: {result['mode']}")
    print(f"World reset: {result['world_reset']}")
    print(f"Steps: {result['steps_completed']}/{result['steps_requested']}")
    print(f"allow_robot_motion: {result['allow_robot_motion']}")
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


def _write_report(result: dict, output_dir: str | None) -> Path:
    run_dir = Path(output_dir).expanduser() if output_dir else _create_run_dir()
    run_dir.mkdir(parents=True, exist_ok=True)
    report_path = run_dir / "simulation_execution_result.json"
    with report_path.open("w", encoding="utf-8") as report_file:
        json.dump(result, report_file, ensure_ascii=False, indent=2)
        report_file.write("\n")
    return report_path


def _create_run_dir() -> Path:
    from datetime import datetime
    import time

    SIMULATION_RUNS_ROOT.mkdir(parents=True, exist_ok=True)
    while True:
        run_name = datetime.now().strftime("run_%Y%m%d_%H%M%S")
        run_dir = SIMULATION_RUNS_ROOT / run_name
        try:
            run_dir.mkdir()
            return run_dir
        except FileExistsError:
            time.sleep(0.05)


if __name__ == "__main__":
    raise SystemExit(main())
