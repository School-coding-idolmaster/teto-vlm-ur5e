import argparse
import sys
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from src.robot_task_inspector import (
    export_replay_subset,
    filter_replay_records,
    format_active_filter_lines,
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
    parser.add_argument("--limit", type=int, help="Maximum number of records to display with --list.")
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
    if args.limit is not None and args.limit <= 0:
        print("--limit must be a positive integer")
        return 1
    candidate = _optional_bool(args.candidate)
    grounded = _optional_bool(args.grounded)
    has_action = args.list or args.stats or args.show is not None or args.export
    active_filters = _active_filters(args, candidate, grounded)

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
        selected_stats = summarize_replay_records(selected)
        selected_stats["total_count"] = len(loaded["records"])
        source = {
            "run_dir": loaded["run_dir"],
            "replay_index_path": loaded["replay_index_path"],
            "results_path": loaded["results_path"],
            "run_id": loaded.get("run_id", ""),
        }
        if active_filters:
            source["filtered_total"] = len(selected)
        print(format_replay_stats(selected_stats, source=source, active_filters=active_filters))
    if should_list:
        if should_stats:
            print()
        if active_filters:
            print("\n".join(format_active_filter_lines(active_filters)))
        print(format_replay_records(selected, limit=args.limit))

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


def _active_filters(args: argparse.Namespace, candidate: bool | None, grounded: bool | None) -> dict:
    filters = {}
    if args.positive:
        filters["positive"] = True
    if args.hard_negative:
        filters["hard_negative"] = True
    if args.reason:
        filters["reason"] = args.reason
    if args.error_code:
        filters["error_code"] = args.error_code
    if candidate is not None:
        filters["candidate"] = candidate
    if grounded is not None:
        filters["grounded"] = grounded
    return filters


if __name__ == "__main__":
    raise SystemExit(main())
