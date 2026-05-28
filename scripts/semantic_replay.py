import argparse
import sys
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from src.robot_task_inspector import (
    export_replay_subset,
    filter_replay_records,
    format_replay_detail,
    format_replay_records,
    format_replay_stats,
    get_replay_record_detail,
    load_replay_index,
    summarize_replay_records,
)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Inspect and export semantic replay samples from a saved robot_task_json run."
    )
    parser.add_argument("run_dir", help="robot_task_json run directory containing replay_index.json.")
    parser.add_argument("--list", action="store_true", help="List replay records.")
    parser.add_argument("--stats", action="store_true", help="Show replay statistics.")
    parser.add_argument("--show", type=int, help="Show one replay record and its matching results.jsonl record.")
    parser.add_argument("--export", help="Export the selected replay subset as JSONL.")
    parser.add_argument("--positive", action="store_true", help="Select positive replay samples.")
    parser.add_argument("--hard-negative", action="store_true", help="Select hard negative replay samples.")
    parser.add_argument("--reason", help="Select records with this rejection_reason.")
    parser.add_argument("--error-code", help="Select records with this error_code.")
    parser.add_argument("--candidate", choices=["true", "false"], help="Select records by candidate flag.")
    parser.add_argument("--grounded", choices=["true", "false"], help="Select records by grounded flag.")
    return parser


def main() -> int:
    args = build_parser().parse_args()
    candidate = _optional_bool(args.candidate)
    grounded = _optional_bool(args.grounded)
    has_action = args.list or args.stats or args.show is not None or args.export

    if args.show is not None:
        detail = get_replay_record_detail(args.run_dir, args.show)
        print(format_replay_detail(detail))
        if not detail.get("ok"):
            return 1
        if args.export:
            print()

    loaded = load_replay_index(args.run_dir)
    if not loaded.get("ok"):
        if args.show is None:
            print(f"Error: {loaded.get('message', 'replay_index.json unavailable')}")
        return 1

    selected = filter_replay_records(
        loaded["records"],
        positive=True if args.positive else None,
        hard_negative=True if args.hard_negative else None,
        reason=args.reason,
        error_code=args.error_code,
        candidate=candidate,
        grounded=grounded,
    )

    should_stats = args.stats or not has_action
    should_list = args.list or not has_action
    if should_stats:
        print(format_replay_stats(summarize_replay_records(selected)))
    if should_list:
        if should_stats:
            print()
        print(format_replay_records(selected))

    if args.export:
        result = export_replay_subset(
            args.run_dir,
            args.export,
            positive=True if args.positive else None,
            hard_negative=True if args.hard_negative else None,
            reason=args.reason,
            error_code=args.error_code,
            candidate=candidate,
            grounded=grounded,
        )
        if not result.get("ok"):
            print(f"Error: {result.get('message', 'export failed')}")
            return 1
        print(f"Exported {result['export_count']} replay records to {result['export_path']}")
    return 0


def _optional_bool(value: str | None) -> bool | None:
    if value is None:
        return None
    return value == "true"


if __name__ == "__main__":
    raise SystemExit(main())
