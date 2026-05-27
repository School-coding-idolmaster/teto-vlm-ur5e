import json
from pathlib import Path


def _ensure_parent(path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)


def save_annotation(image_path, annotation, output_json):
    destination = Path(output_json).expanduser()
    _ensure_parent(destination)
    payload = {
        "image_path": str(Path(image_path).expanduser()),
        "annotation": annotation,
    }
    with destination.open("w", encoding="utf-8") as file:
        json.dump(payload, file, ensure_ascii=False, indent=2)
    return destination


def load_annotations(json_path):
    source = Path(json_path).expanduser()
    if not source.exists():
        raise FileNotFoundError(f"Annotation file not found: {source}")
    with source.open("r", encoding="utf-8") as file:
        return json.load(file)


def append_result(result, output_jsonl):
    destination = Path(output_jsonl).expanduser()
    _ensure_parent(destination)
    with destination.open("a", encoding="utf-8") as file:
        json.dump(result, file, ensure_ascii=False)
        file.write("\n")
    return destination

