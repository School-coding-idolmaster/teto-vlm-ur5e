import json
from pathlib import Path
from typing import Dict, List

from src.json_validator import attach_robot_task_json_fields
from src.output_paths import append_results_index, create_single_recognition_dir, results_index_path


def save_single_recognition_result(
    image_path,
    prompt_type: str,
    prompt: str,
    backend: str,
    response: Dict[str, object],
    output_root=None,
) -> Dict[str, object]:
    run_dir, metadata = create_single_recognition_dir(output_root)
    result_path = run_dir / "result.json"
    summary_path = run_dir / "summary.json"
    index_path = results_index_path()

    result_payload = {
        "record_type": "legacy_rgb_only_record",
        "is_realsense_scene_snapshot": False,
        "image_path": str(Path(image_path).expanduser()),
        "prompt_type": prompt_type,
        "prompt": prompt,
        "backend": backend,
        "response": response.get("response", ""),
        "status": "success",
        "error": "",
    }
    attach_robot_task_json_fields(prompt_type, result_payload)
    with result_path.open("w", encoding="utf-8") as result_file:
        json.dump(result_payload, result_file, ensure_ascii=False, indent=2)
        result_file.write("\n")

    summary = {
        "type": "single_recognition",
        "record_type": "legacy_rgb_only_record",
        "is_realsense_scene_snapshot": False,
        "run_name": metadata["run_name"],
        "created_at": metadata["created_at"],
        "image_path": str(Path(image_path).expanduser()),
        "output_dir": str(run_dir),
        "prompt_type": prompt_type,
        "backend": backend,
        "total": 1,
        "success": 1,
        "failed": 0,
    }
    with summary_path.open("w", encoding="utf-8") as summary_file:
        json.dump(summary, summary_file, ensure_ascii=False, indent=2)
        summary_file.write("\n")

    append_results_index(summary, index_path)

    return {
        "result_path": str(result_path),
        "summary_path": str(summary_path),
        "index_path": str(index_path),
        **summary,
    }


class SingleRecognitionRecorder:
    def __init__(self, image_path, backend: str, output_root=None):
        self.run_dir, self.metadata = create_single_recognition_dir(output_root)
        self.result_path = self.run_dir / "result.json"
        self.summary_path = self.run_dir / "summary.json"
        self.index_path = results_index_path()
        self.image_path = str(Path(image_path).expanduser())
        self.backend = backend
        self.items: List[Dict[str, object]] = []
        self.index_written = False

    def record_success(self, prompt_type: str, prompt: str, response: Dict[str, object]) -> None:
        item = {
            "record_type": "legacy_rgb_only_record",
            "is_realsense_scene_snapshot": False,
            "image_path": self.image_path,
            "prompt_type": prompt_type,
            "prompt": prompt,
            "backend": self.backend,
            "response": response.get("response", ""),
            "status": "success",
            "error": "",
        }
        attach_robot_task_json_fields(prompt_type, item)
        self.items.append(item)
        self.write_files()

    def record_failure(self, prompt_type: str, prompt: str, error: str) -> None:
        self.items.append(
            {
                "record_type": "legacy_rgb_only_record",
                "is_realsense_scene_snapshot": False,
                "image_path": self.image_path,
                "prompt_type": prompt_type,
                "prompt": prompt,
                "backend": self.backend,
                "response": "",
                "status": "failed",
                "error": error,
            }
        )
        self.write_files()

    def write_files(self) -> None:
        payload = {
            "run_name": self.metadata["run_name"],
            "created_at": self.metadata["created_at"],
            "image_path": self.image_path,
            "backend": self.backend,
            "items": self.items,
        }
        with self.result_path.open("w", encoding="utf-8") as result_file:
            json.dump(payload, result_file, ensure_ascii=False, indent=2)
            result_file.write("\n")

        summary = self.summary()
        with self.summary_path.open("w", encoding="utf-8") as summary_file:
            json.dump(summary, summary_file, ensure_ascii=False, indent=2)
            summary_file.write("\n")

    def summary(self) -> Dict[str, object]:
        success = sum(1 for item in self.items if item["status"] == "success")
        failed = sum(1 for item in self.items if item["status"] == "failed")
        prompt_types = sorted({str(item["prompt_type"]) for item in self.items})
        return {
            "type": "single_recognition",
            "run_name": self.metadata["run_name"],
            "created_at": self.metadata["created_at"],
            "image_path": self.image_path,
            "output_dir": str(self.run_dir),
            "prompt_type": ",".join(prompt_types) if prompt_types else "",
            "backend": self.backend,
            "total": len(self.items),
            "success": success,
            "failed": failed,
        }

    def finalize(self) -> Dict[str, object]:
        self.write_files()
        summary = self.summary()
        if self.items and not self.index_written:
            append_results_index(summary, self.index_path)
            self.index_written = True
        return {
            "result_path": str(self.result_path),
            "summary_path": str(self.summary_path),
            "index_path": str(self.index_path),
            **summary,
        }
