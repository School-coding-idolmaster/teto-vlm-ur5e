import argparse
import sys
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from src.display_utils import print_vlm_result
from src.prompt_utils import build_prompt, get_prompt, list_prompt_types
from src.recognition_results import save_single_recognition_result
from src.vlm_infer import VLMInferencer


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "LEGACY/DEBUG RGB-only semantic demo. This is not a RealSense Scene "
            "Snapshot pipeline and must not be used as robot visual input."
        )
    )
    parser.add_argument("--image", required=True, help="Legacy local RGB image path")
    parser.add_argument("--prompt-type", default="describe_image", choices=list_prompt_types())
    parser.add_argument("--prompt", help="Custom prompt, or user instruction for robot_task_json.")
    parser.add_argument("--backend", default="qwen", choices=["mock", "qwen", "local"], help="Inference backend")
    parser.add_argument("--model-path", help="Local Qwen/Ollama model name. Defaults to qwen2.5vl:3b.")
    parser.add_argument("--results", help="Deprecated. Single recognition now writes an isolated result folder.")
    return parser


def main() -> int:
    args = build_parser().parse_args()
    print(
        "LEGACY/DEBUG: RGB-only semantic demo; no depth, camera_info, metadata, "
        "TF, or RealSense snapshot identity is provided."
    )
    image_path = Path(args.image).expanduser()
    if not image_path.exists():
        print(f"Image file not found: {image_path}")
        print("Pass an existing image path. TETO will prepare a model-safe copy automatically.")
        return 1

    prompt = (
        build_prompt(args.prompt_type, args.prompt)
        if args.prompt_type == "robot_task_json"
        else args.prompt if args.prompt else get_prompt(args.prompt_type)
    )
    inferencer = VLMInferencer(backend=args.backend, model_path=args.model_path)

    try:
        result = inferencer.infer(image_path, prompt)
        selected_backend = "qwen" if args.backend == "local" else args.backend
        result_info = save_single_recognition_result(
            image_path,
            args.prompt_type,
            prompt,
            selected_backend,
            result,
        )
    except Exception as exc:
        print(exc)
        return 1

    print_vlm_result(result)
    print(f"Result saved to: {result_info['result_path']}")
    print(f"Summary: {result_info['summary_path']}")
    print(f"Index updated: {result_info['index_path']}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
