import json
import os
import subprocess
import sys
import time
from pathlib import Path

from src.batch_recognition import run_batch_recognition
from src.display_utils import print_vlm_result
from src.image_utils import batch_convert_images, convert_image, load_image_processing_config
from src.output_paths import (
    IMAGE_CONVERSION_BATCH_ROOT,
    create_image_conversion_single_dir,
)
from src.prompt_utils import build_prompt, get_prompt, list_prompt_types
from src.recognition_results import SingleRecognitionRecorder
from src.robot_task_inspector import format_items, format_summary, inspect_robot_task_run
from src.teto_chat import TETOChatSession, is_exit_message
from src.vlm_infer import VLMInferencer


RED = "\033[91m"
RESET = "\033[0m"
PROJECT_ROOT = Path(__file__).resolve().parent
DEFAULT_BACKEND = "qwen"
DEFAULT_PROMPT_TYPE = "describe_image"
PROMPT_ALIASES = {
    "describe": "describe_image",
    "objects": "locate_objects",
    "locate": "locate_objects",
    "spatial": "spatial_relationship",
    "relation": "spatial_relationship",
    "relations": "spatial_relationship",
    "srl": "spatial_relationship_lite",
    "srl_json": "spatial_relationship_json",
    "json": "spatial_relationship_json",
    "manipulation": "manipulation_spatial_analysis",
    "grasp": "manipulation_spatial_analysis",
    "candidate": "manipulation_candidate",
    "robot": "robot_task_json",
    "robot_json": "robot_task_json",
    "task_json": "robot_task_json",
}
PROMPT_KEYWORD_HINTS = [
    ("describe", "describe_image", "Brief image description"),
    ("objects / locate", "locate_objects", "Visible objects and positions"),
    ("spatial / relation", "spatial_relationship", "Basic spatial relationships"),
    ("srl", "spatial_relationship_lite", "Simple SRL with common spatial terms"),
    ("srl_json / json", "spatial_relationship_json", "SRL result in JSON format"),
    ("manipulation / grasp", "manipulation_spatial_analysis", "Robot manipulation spatial analysis"),
    ("candidate", "manipulation_candidate", "Best visible object for manipulation"),
    ("robot_json / task_json", "robot_task_json", "Controlled task JSON for pipeline handoff"),
]
PROMPT_DESCRIPTIONS = {
    "describe_image": "Brief image description",
    "locate_objects": "Visible objects and approximate positions",
    "spatial_relationship": "Spatial relationships between main objects",
    "spatial_relationship_lite": "SRL using simple terms like left, right, near, on",
    "spatial_relationship_json": "SRL output with objects, relations, and confidence",
    "robot_instruction_parse": "Convert instruction into a simple robot action plan",
    "manipulation_candidate": "Find a visible object suitable for manipulation",
    "manipulation_spatial_analysis": "Manipulation-focused objects, relations, target, obstacles",
    "robot_task_json": "Controlled JSON intermediate representation, not robot control",
}


def startup_animation():
    clear_command = "cls" if os.name == "nt" else "clear"
    os.system(clear_command)
    title = r"""
████████╗███████╗████████╗ ██████╗
╚══██╔══╝██╔════╝╚══██╔══╝██╔═══██╗
   ██║   █████╗     ██║   ██║   ██║
   ██║   ██╔══╝     ██║   ██║   ██║
   ██║   ███████╗   ██║   ╚██████╔╝
   ╚═╝   ╚══════╝   ╚═╝    ╚═════╝

              TETO V1.7.0
           -- Test Launcher --
"""
    print(RED + title + RESET)
    time.sleep(0.4)


def _clean_path(value: str) -> str:
    return value.strip().strip("'").strip('"')


def print_menu():
    print("=" * 40)
    print("              TETO V1.7.0")
    print("             Test Launcher")
    print("=" * 40)
    print("1. Convert images")
    print("2. Run the demo")
    print("3. Just chat with TETO")
    print("4. Check environment")
    print("5. Quit")


def print_convert_menu():
    print("==============================")
    print("Convert images")
    print("==============================")
    print("1. Single image")
    print("2. Batch images")
    print("3. Back")


def print_demo_menu():
    print("==============================")
    print("Run the demo")
    print("==============================")
    print("1. Single image recognition")
    print("2. Batch image recognition")
    print("3. Back")


def _unique_output_path(path: Path) -> Path:
    if not path.exists():
        return path
    timestamp = time.strftime("%Y%m%d_%H%M%S")
    return path.with_name(f"{path.stem}_{timestamp}{path.suffix}")


def _format_from_suffix(path: Path) -> str:
    suffix = path.suffix.lower()
    if suffix in {".jpg", ".jpeg"}:
        return "JPEG"
    if suffix == ".png":
        return "PNG"
    if suffix == ".webp":
        return "WEBP"
    return "JPEG"


def handle_convert_single_image():
    image_path = _clean_path(input("Image path: "))
    if not _validate_image_path(image_path):
        return

    source = Path(image_path).expanduser()
    run_dir, metadata = create_image_conversion_single_dir()
    processed_dir = run_dir / "processed"
    processed_dir.mkdir(parents=True, exist_ok=True)
    output_path = _unique_output_path(processed_dir / f"{source.stem}.jpg")
    summary_path = run_dir / "summary.json"
    errors_path = run_dir / "errors.log"

    try:
        converted_path = convert_image(source, output_path, format=_format_from_suffix(output_path))
    except Exception as exc:
        errors_path.write_text(f"{source}\t{exc}\n", encoding="utf-8")
        summary = {
            "run_name": metadata["run_name"],
            "created_at": metadata["created_at"],
            "input_path": str(source),
            "output_dir": str(run_dir),
            "processed_dir": str(processed_dir),
            "total": 1,
            "success": 0,
            "failed": 1,
        }
        with summary_path.open("w", encoding="utf-8") as summary_file:
            json.dump(summary, summary_file, ensure_ascii=False, indent=2)
            summary_file.write("\n")
        print(exc)
        return

    errors_path.write_text("", encoding="utf-8")
    summary = {
        "run_name": metadata["run_name"],
        "created_at": metadata["created_at"],
        "input_path": str(source),
        "output_dir": str(run_dir),
        "processed_dir": str(processed_dir),
        "output_path": str(converted_path),
        "total": 1,
        "success": 1,
        "failed": 0,
    }
    with summary_path.open("w", encoding="utf-8") as summary_file:
        json.dump(summary, summary_file, ensure_ascii=False, indent=2)
        summary_file.write("\n")

    print(f"Converted image: {converted_path}")
    print(f"Summary: {summary_path}")


def handle_batch_convert_images():
    default_input = PROJECT_ROOT / "data" / "raw"
    input_value = _clean_path(input(f"Input folder [{default_input}]: "))
    input_dir = Path(input_value).expanduser() if input_value else default_input
    image_config = load_image_processing_config()

    result = batch_convert_images(
        input_dir,
        output_root=IMAGE_CONVERSION_BATCH_ROOT,
        max_size=image_config["max_size"],
        quality=image_config["quality"],
    )
    if not result.get("ok"):
        print(result.get("message", "Batch conversion failed."))
        return

    print("=" * 50)
    print("BATCH CONVERT RESULT")
    print("=" * 50)
    print(f"Input: {result['input_dir']}")
    print(f"Output: {result['output_dir']}")
    print(f"Max size: {result['max_size']}")
    print(f"Quality: {result['quality']}")
    print(f"Total: {result['total']}")
    print(f"Success: {result['success']}")
    print(f"Failed: {result['failed']}")
    print(f"Manifest: {result['manifest_path']}")
    print(f"Summary: {result['summary_path']}")
    print("=" * 50)


def handle_convert_images():
    while True:
        print()
        print_convert_menu()
        choice = input("Select a conversion option: ").strip()

        if choice == "1":
            handle_convert_single_image()
        elif choice == "2":
            handle_batch_convert_images()
        elif choice == "3":
            return
        else:
            print("Invalid option. Please choose 1, 2, or 3.")


def read_prompt_type() -> str:
    prompt_types = list_prompt_types()
    print_prompt_helper()
    value = input("Prompt type: ").strip()
    return PROMPT_ALIASES.get(value.lower(), value) if value else DEFAULT_PROMPT_TYPE


def print_prompt_helper() -> None:
    prompt_types = list_prompt_types()
    print()
    print("=" * 72)
    print("Prompt helper")
    print("=" * 72)
    print(f"Press Enter to use default: {DEFAULT_PROMPT_TYPE}")
    print("Type a prompt type, a shortcut keyword, or your own free-form question.")
    print()
    print("Built-in prompt types")
    print("-" * 72)
    for prompt_type in prompt_types:
        description = PROMPT_DESCRIPTIONS.get(prompt_type, "")
        print(f"  {prompt_type:<32} {description}")
    print()
    print("Shortcut keywords")
    print("-" * 72)
    for keyword, prompt_type, description in PROMPT_KEYWORD_HINTS:
        print(f"  {keyword:<22} -> {prompt_type:<30} {description}")
    print()
    print("Examples")
    print("-" * 72)
    print("  srl")
    print("  spatial_relationship_json")
    print("  robot_task_json")
    print("  List objects near the center and describe what blocks them.")
    print("=" * 72)


def resolve_prompt_input(value: str, default_prompt_type: str = DEFAULT_PROMPT_TYPE) -> tuple[str, str]:
    prompt_types = list_prompt_types()
    selected = value.strip()
    if not selected:
        return default_prompt_type, get_prompt(default_prompt_type)

    prompt_type = PROMPT_ALIASES.get(selected.lower(), selected)
    if prompt_type in prompt_types:
        return prompt_type, get_prompt(prompt_type)

    return "freeform", selected


def _read_single_image_question(question_count: int) -> tuple[str, str] | None:
    if question_count == 0:
        print("Ask anything about this image.")
        print_prompt_helper()
    value = input(f"Question / prompt [{DEFAULT_PROMPT_TYPE}]: ").strip()
    if is_exit_message(value):
        return None
    prompt_type, prompt = resolve_prompt_input(value)
    if prompt_type == "robot_task_json":
        instruction = input("User instruction [unknown]: ").strip()
        prompt = build_prompt(prompt_type, instruction)
    return prompt_type, prompt


def _prompt_with_follow_up_context(prompt: str, history: list[dict[str, str]]) -> str:
    if not history:
        return prompt

    lines = [
        "We are discussing the same image. Use the previous Q&A as context, but answer the latest question directly.",
        "",
        "Previous Q&A:",
    ]
    for item in history[-6:]:
        lines.append(f"Q: {item['prompt']}")
        lines.append(f"A: {item['response']}")
    lines.extend(["", "Latest question:", prompt])
    return "\n".join(lines)


def read_backend() -> str:
    return DEFAULT_BACKEND


def read_chat_backend() -> str:
    return DEFAULT_BACKEND


def handle_single_image_recognition():
    image_path = _clean_path(input("Image path: "))
    if not _validate_image_path(image_path):
        return

    backend = read_backend()

    try:
        inferencer = VLMInferencer(backend=backend)
    except Exception as exc:
        print(exc)
        return

    recorder = SingleRecognitionRecorder(image_path, backend)
    history = []
    print("Type `back`, `q`, `quit`, or `exit` when you are done with this image.")

    while True:
        question = _read_single_image_question(len(history))
        if question is None:
            break

        prompt_type, prompt = question
        prompt_for_model = _prompt_with_follow_up_context(prompt, history)
        try:
            result = inferencer.infer(image_path, prompt_for_model)
            result["prompt_type"] = prompt_type
            result["prompt"] = prompt
            recorder.record_success(prompt_type, prompt, result)
            history.append({"prompt": prompt, "response": result.get("response", "")})
        except Exception as exc:
            recorder.record_failure(prompt_type, prompt, str(exc))
            print(exc)
            continue

        print_vlm_result(result)
        print(f"Saved in: {recorder.run_dir}")

    if not history and not recorder.items:
        print("No recognition questions were saved.")
        return

    result_info = recorder.finalize()
    print(f"Result saved to: {result_info['result_path']}")
    print(f"Summary: {result_info['summary_path']}")
    print(f"Index updated: {result_info['index_path']}")


def handle_batch_image_recognition():
    default_input = PROJECT_ROOT / "data" / "processed"
    input_value = _clean_path(input(f"Input folder [{default_input}]: "))
    input_dir = Path(input_value).expanduser() if input_value else default_input
    print_prompt_helper()
    prompt_value = input(f"Prompt / keyword [{DEFAULT_PROMPT_TYPE}]: ").strip()
    prompt_type, prompt = resolve_prompt_input(prompt_value)
    if prompt_type == "robot_task_json":
        instruction = input("User instruction applied to every image [unknown]: ").strip()
        prompt = build_prompt(prompt_type, instruction)
    backend = read_backend()

    try:
        result = run_batch_recognition(
            input_dir,
            prompt_type=prompt_type,
            prompt=prompt,
            backend=backend,
        )
    except Exception as exc:
        print(exc)
        return

    if not result.get("ok"):
        print(result.get("message", "Batch recognition failed."))
        return

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
    maybe_show_robot_task_inspection(result)


def maybe_show_robot_task_inspection(result: dict) -> None:
    if result.get("prompt_type") != "robot_task_json":
        return

    run_dir = result.get("run_dir") or result.get("output_dir")
    inspection = inspect_robot_task_run(run_dir)
    print()
    print(format_summary(inspection))
    if not inspection.get("ok"):
        return

    choice = input("Show detailed inspection? [y/N]: ").strip().lower()
    if choice == "y":
        print()
        print(format_items(inspection.get("items", [])))


def handle_run_the_demo():
    while True:
        print()
        print_demo_menu()
        choice = input("Select a demo option: ").strip()

        if choice == "1":
            handle_single_image_recognition()
        elif choice == "2":
            handle_batch_image_recognition()
        elif choice == "3":
            return
        else:
            print("Invalid option. Please choose 1, 2, or 3.")


def handle_just_chat_with_teto():
    print("==============================")
    print("Just chat with TETO")
    print("==============================")
    backend = read_chat_backend()
    session = TETOChatSession(backend=backend)
    print(f"Backend: {backend}")
    print("Type `back`, `q`, `quit`, or `exit` to return.")

    while True:
        message = input("You: ").strip()
        if is_exit_message(message):
            print("TETO: Back to the main menu.")
            return

        try:
            reply = session.reply(message)
        except Exception as exc:
            print(f"TETO chat error: {exc}")
            return
        print(f"TETO: {reply}")


def _validate_image_path(image_path: str) -> bool:
    if not image_path:
        print("Image path is required.")
        return False

    path = Path(image_path).expanduser()
    if not path.exists():
        print(f"Image file not found: {path}")
        print("Please choose an existing image path.")
        return False
    if not path.is_file():
        print(f"Image path is not a file: {path}")
        return False

    return True


def handle_check_environment():
    script_path = PROJECT_ROOT / "scripts" / "check_env.py"
    completed = subprocess.run([sys.executable, str(script_path)], cwd=PROJECT_ROOT, check=False)
    if completed.returncode != 0:
        print("Environment check did not complete successfully.")


def main():
    startup_animation()

    while True:
        print()
        print_menu()
        choice = input("Select an option: ").strip()

        if choice == "1":
            handle_convert_images()
        elif choice == "2":
            handle_run_the_demo()
        elif choice == "3":
            handle_just_chat_with_teto()
        elif choice == "4":
            handle_check_environment()
        elif choice == "5":
            print("Bye.")
            break
        else:
            print("Invalid option. Please choose 1, 2, 3, 4, or 5.")


if __name__ == "__main__":
    main()
