import argparse
import json
from pathlib import Path

from src.image_utils import prepare_image_dataset
from src.prompt_utils import build_prompt, get_prompt, list_prompt_types
from src.recognition_results import save_single_recognition_result
from src.vlm_infer import VLMInferencer


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Unified CLI for the TETO V1 VLM Pipeline.")
    subparsers = parser.add_subparsers(dest="command", required=True)

    subparsers.add_parser("check-env", help="Check Python, dependencies, and GPU status")

    demo_parser = subparsers.add_parser("demo", help="Run the demo")
    demo_parser.add_argument("--image", required=True)
    demo_parser.add_argument("--prompt-type", default="describe_image", choices=list_prompt_types())
    demo_parser.add_argument("--prompt", help="Custom prompt, or user instruction for robot_task_json.")
    demo_parser.add_argument("--results", help="Deprecated. Single recognition now writes an isolated result folder.")

    prepare_parser = subparsers.add_parser("prepare-images", help="Prepare image datasets for VLM use")
    prepare_parser.add_argument("--input-dir", required=True)
    prepare_parser.add_argument("--output-dir", default="data/processed/auto")
    prepare_parser.add_argument("--manifest", default="outputs/results/prepared_images.jsonl")
    prepare_parser.add_argument("--max-size", type=int, default=1024)
    prepare_parser.add_argument("--quality", type=int, default=90)
    prepare_parser.add_argument("--no-recursive", action="store_true")
    return parser


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()

    if args.command == "check-env":
        from scripts.check_env import print_env_info

        print_env_info()
        return 0

    if args.command == "demo":
        image_path = Path(args.image).expanduser()
        if not image_path.exists():
            print(f"Image file not found: {image_path}")
            return 1
        prompt = (
            build_prompt(args.prompt_type, args.prompt)
            if args.prompt_type == "robot_task_json"
            else args.prompt if args.prompt else get_prompt(args.prompt_type)
        )
        try:
            result = VLMInferencer().infer(image_path, prompt)
            result_info = save_single_recognition_result(
                image_path,
                args.prompt_type,
                prompt,
                "mock",
                result,
            )
        except Exception as exc:
            print(exc)
            return 1
        print(json.dumps({"result": result, "saved_to": result_info}, ensure_ascii=False, indent=2))
        return 0

    if args.command == "prepare-images":
        try:
            stats = prepare_image_dataset(
                args.input_dir,
                output_dir=args.output_dir,
                manifest_path=args.manifest,
                max_size=args.max_size,
                quality=args.quality,
                recursive=not args.no_recursive,
            )
        except Exception as exc:
            print(exc)
            return 1
        print(json.dumps(stats, ensure_ascii=False, indent=2))
        return 0

    parser.print_help()
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
