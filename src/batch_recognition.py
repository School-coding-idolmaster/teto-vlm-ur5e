import json
from pathlib import Path
from typing import Dict, List

from src.output_paths import (
    BATCH_RECOGNITION_ROOT,
    append_results_index,
    create_batch_recognition_dir,
    create_robot_task_json_dir,
    results_index_path,
)
from src.json_validator import attach_robot_task_json_fields
from src.prompt_utils import get_prompt
from src.robot_task_inspector import write_scene_and_replay_indexes, write_smoke_report
from src.execution_readiness_contract import evaluate_execution_readiness
from src.simulation_bridge_contract import evaluate_simulation_bridge_eligibility
from src.vlm_infer import VLMInferencer


SUPPORTED_IMAGE_EXTENSIONS = {".jpg", ".jpeg", ".png", ".webp"}
DEFAULT_OUTPUT_ROOT = BATCH_RECOGNITION_ROOT
ROBOT_TASK_PROMPT_TYPE = "robot_task_json"
CURRENT_TETO_VERSION = "TETO V2.2.0"


def _normalize_path(path) -> Path:
    return Path(path).expanduser()


def _create_run_dir(prompt_type: str, output_root=None):
    if prompt_type == ROBOT_TASK_PROMPT_TYPE:
        return create_robot_task_json_dir(output_root)
    return create_batch_recognition_dir(output_root or DEFAULT_OUTPUT_ROOT)


def _find_images(input_dir: Path) -> List[Path]:
    return [
        path
        for path in sorted(input_dir.rglob("*"))
        if path.is_file() and path.suffix.lower() in SUPPORTED_IMAGE_EXTENSIONS
    ]


def _append_index(index_path: Path, summary: Dict[str, object]) -> None:
    index_item = {
        "type": "batch_recognition",
        "run_name": summary["run_name"],
        "created_at": summary["created_at"],
        "input_dir": summary["input_dir"],
        "output_dir": summary["output_dir"],
        "prompt_type": summary["prompt_type"],
        "backend": summary["backend"],
        "total": summary["total"],
        "success": summary["success"],
        "failed": summary["failed"],
    }
    append_results_index(index_item, index_path)


def _attach_simulation_bridge_result(prompt_type: str, item: Dict[str, object]) -> None:
    if prompt_type != ROBOT_TASK_PROMPT_TYPE:
        return
    normalized = item.get("normalized_json")
    readiness = evaluate_execution_readiness(normalized if isinstance(normalized, dict) else None)
    item["simulation_bridge_result"] = evaluate_simulation_bridge_eligibility(
        normalized if isinstance(normalized, dict) else None,
        readiness,
    )


def run_batch_recognition(
    input_dir,
    prompt_type="describe_image",
    prompt=None,
    backend="mock",
    output_root=None,
) -> Dict[str, object]:
    source_dir = _normalize_path(input_dir)
    if not source_dir.exists():
        return {
            "ok": False,
            "message": f"Input directory not found: {source_dir}",
            "input_dir": str(source_dir),
        }
    if not source_dir.is_dir():
        return {
            "ok": False,
            "message": f"Input path is not a directory: {source_dir}",
            "input_dir": str(source_dir),
        }

    image_paths = _find_images(source_dir)
    if not image_paths:
        supported = ", ".join(sorted(SUPPORTED_IMAGE_EXTENSIONS))
        return {
            "ok": False,
            "message": f"No supported images found in {source_dir}. Supported formats: {supported}",
            "input_dir": str(source_dir),
        }

    prompt = prompt if prompt is not None else get_prompt(prompt_type)
    selected_backend = (backend or "mock").strip().lower()
    run_dir, run_metadata = _create_run_dir(prompt_type, output_root)
    results_path = run_dir / "results.jsonl"
    summary_path = run_dir / "summary.json"
    errors_path = run_dir / "errors.log"
    input_manifest_path = run_dir / "input_manifest.json"
    index_path = results_index_path()

    input_manifest = {
        "version": CURRENT_TETO_VERSION,
        "prompt_type": prompt_type,
        "backend": selected_backend,
        "image_count": len(image_paths),
        "image_paths": [str(path) for path in image_paths],
        "created_at": run_metadata["created_at"],
        "output_dir": str(run_dir),
    }
    if prompt_type == ROBOT_TASK_PROMPT_TYPE:
        input_manifest["user_instruction"] = prompt
        with input_manifest_path.open("w", encoding="utf-8") as manifest_file:
            json.dump(input_manifest, manifest_file, ensure_ascii=False, indent=2)
            manifest_file.write("\n")

    success = 0
    failed = 0
    error_lines = []
    inferencer = VLMInferencer(backend=selected_backend)

    with results_path.open("w", encoding="utf-8") as results_file:
        for index, image_path in enumerate(image_paths, start=1):
            print(f"[{index}/{len(image_paths)}] processing {image_path.name}")
            item = {
                "image_path": str(image_path),
                "prompt_type": prompt_type,
                "prompt": prompt,
                "backend": selected_backend,
                "run_name": run_metadata["run_name"],
                "item_index": index,
                "created_at": run_metadata["created_at"],
                "response": "",
                "status": "success",
                "error": "",
            }
            try:
                result = inferencer.infer(image_path, prompt)
                item["response"] = result.get("response", "")
                attach_robot_task_json_fields(prompt_type, item)
                _attach_simulation_bridge_result(prompt_type, item)
                success += 1
            except Exception as exc:
                failed += 1
                item["status"] = "failed"
                item["error"] = str(exc)
                error_lines.append(f"{image_path}\t{exc}")
                attach_robot_task_json_fields(prompt_type, item)
                _attach_simulation_bridge_result(prompt_type, item)

            results_file.write(json.dumps(item, ensure_ascii=False) + "\n")

    summary = {
        "run_name": run_metadata["run_name"],
        "created_at": run_metadata["created_at"],
        "input_dir": str(source_dir),
        "output_dir": str(run_dir),
        "prompt_type": prompt_type,
        "backend": selected_backend,
        "total": len(image_paths),
        "success": success,
        "failed": failed,
    }
    with summary_path.open("w", encoding="utf-8") as summary_file:
        json.dump(summary, summary_file, ensure_ascii=False, indent=2)
        summary_file.write("\n")

    errors_path.write_text("\n".join(error_lines) + ("\n" if error_lines else ""), encoding="utf-8")
    _append_index(index_path, summary)
    smoke_report_paths = {}
    scene_replay_paths = {}
    if prompt_type == ROBOT_TASK_PROMPT_TYPE:
        smoke_report_paths = write_smoke_report(run_dir)
        scene_replay_paths = write_scene_and_replay_indexes(run_dir)

    result = {
        "ok": True,
        "run_dir": str(run_dir),
        "results_path": str(results_path),
        "summary_path": str(summary_path),
        "errors_path": str(errors_path),
        "index_path": str(index_path),
        **smoke_report_paths,
        **scene_replay_paths,
        **summary,
    }
    if prompt_type == ROBOT_TASK_PROMPT_TYPE:
        result["input_manifest_path"] = str(input_manifest_path)
    return result
