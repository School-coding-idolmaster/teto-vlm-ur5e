import argparse
import json
import shlex
import sys
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from src.simulation_runtime import DEFAULT_SIMULATION_TASK, run_first_simulation_execution


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Run TETO V2.1.3 robot structure evidence export smoke test.")
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
    parser.add_argument(
        "--check-robot-asset",
        action="store_true",
        help="Check robot asset availability without requiring a successful load.",
    )
    parser.add_argument(
        "--load-robot-asset",
        action="store_true",
        help="Load the specified local robot asset and verify the robot prim exists.",
    )
    parser.add_argument("--robot-asset-path", help="Local USD/USDA/USDC robot asset path to check or load.")
    parser.add_argument("--robot-type", default="ur5", help="Robot type label for report metadata.")
    parser.add_argument("--robot-prim-path", default="/World/TETO_Robot", help="Prim path for a loaded robot asset.")
    parser.add_argument(
        "--inspect-robot-prim",
        action="store_true",
        help="Read robot prim structure and metadata without commanding robot motion.",
    )
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
        check_robot_asset=args.check_robot_asset,
        load_robot_asset=args.load_robot_asset,
        robot_type=args.robot_type,
        robot_prim_path=args.robot_prim_path,
        robot_asset_path=args.robot_asset_path,
        inspect_robot_prim=args.inspect_robot_prim,
        output_dir=args.output_dir,
        write_report=True,
        demo_command=shlex.join([sys.executable, *sys.argv]),
    )
    report_path = Path(str(result["report_path"]))
    print_summary(result, report_path)
    return 0 if result.get("ok") else 1


def print_summary(result: dict, report_path: Path) -> None:
    print("=" * 50)
    print("TETO V2.1.3 ROBOT STRUCTURE EVIDENCE EXPORT")
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
    print(f"robot_asset_check_requested: {result.get('robot_asset_check_requested')}")
    print(f"robot_asset_load_requested: {result.get('robot_asset_load_requested')}")
    print(f"robot_type: {result.get('robot_type')}")
    print(f"robot_prim_path: {result.get('robot_prim_path')}")
    print(f"robot_asset_path: {result.get('robot_asset_path')}")
    print(f"robot_asset_source: {result.get('robot_asset_source')}")
    print(f"robot_asset_available: {result.get('robot_asset_available')}")
    print(f"robot_asset_loaded: {result.get('robot_asset_loaded')}")
    print(f"robot_prim_exists: {result.get('robot_prim_exists')}")
    print(f"robot_asset_status: {result.get('robot_asset_status')}")
    print(f"robot_asset_blocking_reason: {result.get('robot_asset_blocking_reason')}")
    inspection = result.get("robot_prim_inspection") or {}
    print(f"robot_prim_inspection_requested: {result.get('robot_prim_inspection_requested')}")
    print(f"robot_prim_inspection_status: {inspection.get('inspection_status')}")
    print(f"robot_root_type_name: {inspection.get('robot_root_type_name')}")
    print(f"total_descendant_prim_count: {inspection.get('total_descendant_prim_count')}")
    print(f"link_like_prim_count: {inspection.get('link_like_prim_count')}")
    print(f"joint_like_prim_count: {inspection.get('joint_like_prim_count')}")
    print(f"visual_like_prim_count: {inspection.get('visual_like_prim_count')}")
    print(f"collision_like_prim_count: {inspection.get('collision_like_prim_count')}")
    print(f"articulation_root_found: {inspection.get('articulation_root_found')}")
    joint_summary = inspection.get("joint_metadata_summary") or {}
    print(f"arm_joint_count: {joint_summary.get('arm_joint_count')}")
    print(f"arm_joint_names: {joint_summary.get('arm_joint_names')}")
    print(f"structural_joint_count: {joint_summary.get('structural_joint_count')}")
    print(f"structural_joint_names: {joint_summary.get('structural_joint_names')}")
    print(f"gripper_or_tool_joint_count: {joint_summary.get('gripper_or_tool_joint_count')}")
    print(f"gripper_or_tool_joint_names: {joint_summary.get('gripper_or_tool_joint_names')}")
    print(f"unknown_joint_count: {joint_summary.get('unknown_joint_count')}")
    print(f"unknown_joint_names: {joint_summary.get('unknown_joint_names')}")
    print(f"robot_structure_report_generated: {result.get('robot_structure_report_generated')}")
    print(f"robot_structure_report_path: {result.get('robot_structure_report_path')}")
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
