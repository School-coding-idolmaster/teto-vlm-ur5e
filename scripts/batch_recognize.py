"""LEGACY/DEBUG RGB-only batch entrypoint.

This is not the current default real or Isaac path. Current real default:
`scripts/start_teto_real_full_stack.sh` / `scripts/teto_operator_console.py`.
Current Isaac default: `scripts/start_teto_isaac_gui_operator.sh`.
Dry-run, plan-only, fake, RGB-only, or Isaac evidence from this script is not
REAL_PATH success evidence.
"""

import argparse
import sys
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from src.batch_recognition import run_batch_recognition
from src.prompt_utils import build_prompt, list_prompt_types


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "LEGACY/DEBUG RGB-only batch semantic demo. It does not create formal "
            "RealSense Scene Snapshots."
        )
    )
    parser.add_argument("--input-dir", required=True, help="Legacy folder containing RGB images")
    parser.add_argument("--prompt-type", default="describe_image", choices=list_prompt_types())
    parser.add_argument("--prompt", help="Custom prompt, or user instruction for robot_task_json.")
    parser.add_argument("--backend", default="qwen", choices=["mock", "qwen", "local"], help="Inference backend")
    parser.add_argument("--output-root")
    return parser


def print_summary(result: dict) -> None:
    print("=" * 50)
    print("BATCH RECOGNITION RESULT")
    print("=" * 50)
    print(f"Run: {result['run_name']}")
    print(f"Input: {result['input_dir']}")
    print(f"Output: {result['output_dir']}")
    print(f"Prompt type: {result['prompt_type']}")
    print(f"Backend: {result['backend']}")
    print(f"Total: {result['total']}")
    print(f"Success: {result['success']}")
    print(f"Failed: {result['failed']}")
    print(f"Results: {result['results_path']}")
    print(f"Summary: {result['summary_path']}")
    print(f"Errors: {result['errors_path']}")
    print(f"Index updated: {result['index_path']}")
    print("=" * 50)


def main() -> int:
    args = build_parser().parse_args()
    print(
        "LEGACY/DEBUG: RGB-only batch input; generated records are legacy semantic "
        "replay records, not RealSense snapshots."
    )
    prompt_type = args.prompt_type
    prompt = args.prompt
    if args.prompt and args.prompt_type != "robot_task_json":
        prompt_type = "freeform"
    elif args.prompt_type == "robot_task_json":
        prompt = build_prompt(args.prompt_type, args.prompt)

    result = run_batch_recognition(
        args.input_dir,
        prompt_type=prompt_type,
        prompt=prompt,
        backend=args.backend,
        output_root=args.output_root,
    )
    if not result.get("ok"):
        print(result.get("message", "Batch recognition failed."))
        return 1

    print_summary(result)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
