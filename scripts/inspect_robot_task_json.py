import argparse
import sys
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from src.robot_task_inspector import format_items, format_summary, inspect_robot_task_run


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Inspect saved robot_task_json results without rerunning VLM inference.")
    parser.add_argument("--run-dir", help="robot_task_json run directory. Defaults to the latest run.")
    parser.add_argument("--limit", type=int, help="Maximum number of detailed items to display.")
    parser.add_argument("--details", action="store_true", help="Show per-item inspection details.")
    parser.add_argument("--indexes", action="store_true", help="Show scene/replay index summary; included in summary output.")
    return parser


def main() -> int:
    args = build_parser().parse_args()
    inspection = inspect_robot_task_run(args.run_dir)
    print(format_summary(inspection))
    if args.details:
        print()
        print(format_items(inspection.get("items", []), limit=args.limit))
    return 0 if inspection.get("ok") else 1


if __name__ == "__main__":
    raise SystemExit(main())
