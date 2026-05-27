import json
import time
from datetime import datetime
from pathlib import Path
from typing import Dict, Tuple


PROJECT_ROOT = Path(__file__).resolve().parents[1]
OUTPUTS_ROOT = PROJECT_ROOT / "outputs"

IMAGE_CONVERSION_ROOT = OUTPUTS_ROOT / "image_conversion"
IMAGE_CONVERSION_SINGLE_ROOT = IMAGE_CONVERSION_ROOT / "single"
IMAGE_CONVERSION_BATCH_ROOT = IMAGE_CONVERSION_ROOT / "batch"

RESULTS_ROOT = OUTPUTS_ROOT / "results"
SINGLE_RECOGNITION_ROOT = RESULTS_ROOT / "single_recognition"
BATCH_RECOGNITION_ROOT = RESULTS_ROOT / "batch_recognition"
ROBOT_TASK_JSON_ROOT = RESULTS_ROOT / "robot_task_json"
RESULTS_INDEX_PATH = RESULTS_ROOT / "index.jsonl"

LOGS_ROOT = OUTPUTS_ROOT / "logs"


def _normalize_path(path) -> Path:
    return Path(path).expanduser()


def _timestamp_metadata(prefix: str) -> Dict[str, str]:
    now = datetime.now()
    return {
        "run_name": now.strftime(f"{prefix}_%Y%m%d_%H%M%S"),
        "created_at": now.strftime("%Y-%m-%d %H:%M:%S"),
    }


def _create_timestamped_dir(root: Path, prefix: str) -> Tuple[Path, Dict[str, str]]:
    root.mkdir(parents=True, exist_ok=True)
    while True:
        metadata = _timestamp_metadata(prefix)
        run_dir = root / metadata["run_name"]
        try:
            run_dir.mkdir()
            return run_dir, metadata
        except FileExistsError:
            time.sleep(0.05)


def create_image_conversion_single_dir(output_root=None) -> Tuple[Path, Dict[str, str]]:
    root = _normalize_path(output_root) if output_root else IMAGE_CONVERSION_SINGLE_ROOT
    return _create_timestamped_dir(root, "single")


def create_image_conversion_batch_dir(output_root=None) -> Path:
    root = _normalize_path(output_root) if output_root else IMAGE_CONVERSION_BATCH_ROOT
    run_dir, _ = _create_timestamped_dir(root, "batch")
    return run_dir


def create_single_recognition_dir(output_root=None) -> Tuple[Path, Dict[str, str]]:
    root = _normalize_path(output_root) if output_root else SINGLE_RECOGNITION_ROOT
    return _create_timestamped_dir(root, "single")


def create_batch_recognition_dir(output_root=None) -> Tuple[Path, Dict[str, str]]:
    root = _normalize_path(output_root) if output_root else BATCH_RECOGNITION_ROOT
    return _create_timestamped_dir(root, "run")


def create_robot_task_json_dir(output_root=None) -> Tuple[Path, Dict[str, str]]:
    root = _normalize_path(output_root) if output_root else ROBOT_TASK_JSON_ROOT
    return _create_timestamped_dir(root, "run")


def results_index_path() -> Path:
    RESULTS_ROOT.mkdir(parents=True, exist_ok=True)
    return RESULTS_INDEX_PATH


def append_results_index(item: Dict[str, object], index_path=None) -> Path:
    destination = _normalize_path(index_path) if index_path else results_index_path()
    destination.parent.mkdir(parents=True, exist_ok=True)
    with destination.open("a", encoding="utf-8") as index_file:
        index_file.write(json.dumps(item, ensure_ascii=False) + "\n")
    return destination
