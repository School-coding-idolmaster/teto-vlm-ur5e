import json
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional

from src.output_paths import ROBOT_TASK_JSON_ROOT


UNKNOWN = "unknown"


def find_latest_robot_task_run(root: Path | str | None = None) -> Optional[Path]:
    root_path = Path(root if root is not None else ROBOT_TASK_JSON_ROOT).expanduser()
    if not root_path.exists() or not root_path.is_dir():
        return None

    run_dirs = [path for path in root_path.glob("run_*") if path.is_dir()]
    if not run_dirs:
        return None
    return sorted(run_dirs, key=lambda path: path.name)[-1]


def load_results_jsonl(results_path: Path | str) -> List[Dict[str, Any]]:
    path = Path(results_path).expanduser()
    items: List[Dict[str, Any]] = []
    if not path.exists():
        raise FileNotFoundError(f"results.jsonl not found: {path}")

    with path.open("r", encoding="utf-8") as results_file:
        for line_number, line in enumerate(results_file, start=1):
            text = line.strip()
            if not text:
                continue
            try:
                data = json.loads(text)
                if isinstance(data, dict):
                    items.append(data)
                else:
                    items.append(_bad_line_item(line_number, "top-level JSON value must be an object"))
            except json.JSONDecodeError as exc:
                items.append(_bad_line_item(line_number, f"{exc.msg} at column {exc.colno}"))
    return items


def inspect_robot_task_run(run_dir: Path | str | None = None) -> Dict[str, Any]:
    selected_run_dir = Path(run_dir).expanduser() if run_dir else find_latest_robot_task_run()
    if selected_run_dir is None:
        return {
            "ok": False,
            "message": f"No robot_task_json runs found under {ROBOT_TASK_JSON_ROOT}",
            "run_dir": "",
            "results_path": "",
            "summary": _empty_summary(),
            "items": [],
        }
    if not selected_run_dir.exists() or not selected_run_dir.is_dir():
        return {
            "ok": False,
            "message": f"Run directory not found: {selected_run_dir}",
            "run_dir": str(selected_run_dir),
            "results_path": str(selected_run_dir / "results.jsonl"),
            "summary": _empty_summary(),
            "items": [],
        }

    results_path = selected_run_dir / "results.jsonl"
    if not results_path.exists():
        return {
            "ok": False,
            "message": f"results.jsonl not found: {results_path}",
            "run_dir": str(selected_run_dir),
            "results_path": str(results_path),
            "summary": _empty_summary(),
            "items": [],
        }

    raw_items = load_results_jsonl(results_path)
    items = [_inspect_item(index, item) for index, item in enumerate(raw_items, start=1)]
    return {
        "ok": True,
        "message": "",
        "run_dir": str(selected_run_dir),
        "results_path": str(results_path),
        "summary": _build_summary(items),
        "items": items,
    }


def format_summary(inspection: Dict[str, Any]) -> str:
    summary = inspection.get("summary", _empty_summary())
    lines = [
        "=" * 60,
        "TETO robot_task_json inspector",
        "=" * 60,
        f"Run dir: {inspection.get('run_dir', '')}",
        f"Results: {inspection.get('results_path', '')}",
        "",
    ]
    if not inspection.get("ok", False) and inspection.get("message"):
        lines.extend([f"Error: {inspection['message']}", ""])
    lines.extend(
        [
            "Summary",
            f"  total:              {summary['total']}",
            f"  parse_success:      {summary['parse_success']}",
            f"  parse_failed:       {summary['parse_failed']}",
            f"  validation_passed:  {summary['validation_passed']}",
            f"  validation_warning: {summary['validation_warning']}",
            f"  validation_failed:  {summary['validation_failed']}",
            f"  unsafe_count:       {summary['unsafe_count']}",
            f"  rejected_count:     {summary['rejected_count']}",
            f"  grounding_count:    {summary['grounding_count']}",
            f"  grounding_missing_count: {summary['grounding_missing_count']}",
        ]
    )
    return "\n".join(lines)


def format_items(items: Iterable[Dict[str, Any]], limit: int | None = None) -> str:
    selected_items = list(items)
    if limit is not None:
        selected_items = selected_items[: max(limit, 0)]
    if not selected_items:
        return "Items\n" + "-" * 60 + "\n(no items)"

    lines = ["Items", "-" * 60]
    for item in selected_items:
        lines.extend(
            [
                f"[{item['index']}]",
                f"image_path: {item['image_path']}",
                f"parse_status: {item['parse_status']}",
                f"validation_status: {item['validation_status']}",
                f"target.label: {item['target_label']}",
                f"candidate: {_format_value(item['candidate'])}",
                f"difficulty: {item['difficulty']}",
                f"error.code: {item['error_code']}",
                f"bbox_xyxy: {_format_value(item['bbox_xyxy'])}",
                f"pixel_center: {_format_value(item['pixel_center'])}",
                f"image_size: {item['image_size']}",
                f"geometry_2d.confidence: {_format_value(item['geometry_2d_confidence'])}",
            ]
        )
        warnings = item.get("validation_warnings", [])
        if warnings:
            lines.append("validation_warnings:")
            for warning in warnings:
                lines.append(f"  - {warning}")
        else:
            lines.append("validation_warnings: []")
        errors = item.get("validation_errors", [])
        if errors:
            lines.append("validation_errors:")
            for error in errors:
                lines.append(f"  - {error}")
        else:
            lines.append("validation_errors: []")
        if item.get("line_error"):
            lines.append(f"line_error: {item['line_error']}")
        lines.append("")
    return "\n".join(lines).rstrip()


def _inspect_item(index: int, item: Dict[str, Any]) -> Dict[str, Any]:
    target_label = _first_value(
        item,
        ("normalized_json", "target", "label"),
        ("parsed_json", "target", "label"),
        default=UNKNOWN,
    )
    candidate = _first_value(
        item,
        ("normalized_json", "target", "candidate"),
        ("normalized_json", "manipulation_assessment", "candidate"),
        ("parsed_json", "target", "candidate"),
        default=UNKNOWN,
    )
    difficulty = _first_value(
        item,
        ("normalized_json", "manipulation", "difficulty"),
        ("normalized_json", "manipulation_assessment", "difficulty"),
        ("parsed_json", "manipulation", "difficulty"),
        default=UNKNOWN,
    )
    error_code = _first_value(
        item,
        ("normalized_json", "error", "code"),
        ("parsed_json", "error", "code"),
        default=UNKNOWN,
    )
    bbox_xyxy = _first_value(
        item,
        ("normalized_json", "target", "bbox_xyxy"),
        ("parsed_json", "target", "bbox_xyxy"),
        ("parsed_json", "target", "bbox"),
        ("parsed_json", "geometry_2d", "bbox_xyxy"),
        default=None,
    )
    pixel_center = _first_value(
        item,
        ("normalized_json", "geometry_2d", "pixel_center"),
        ("parsed_json", "geometry_2d", "pixel_center"),
        ("parsed_json", "target", "pixel_center"),
        default=None,
    )
    image_width = _first_value(item, ("normalized_json", "geometry_2d", "image_width"), default=None)
    image_height = _first_value(item, ("normalized_json", "geometry_2d", "image_height"), default=None)
    geometry_confidence = _first_value(
        item,
        ("normalized_json", "geometry_2d", "confidence"),
        ("parsed_json", "geometry_2d", "confidence"),
        default=None,
    )
    warnings = item.get("validation_warnings", [])
    if not isinstance(warnings, list):
        warnings = [str(warnings)]
    errors = item.get("validation_errors", [])
    if not isinstance(errors, list):
        errors = [str(errors)]

    return {
        "index": index,
        "image_path": item.get("image_path", UNKNOWN),
        "parse_status": item.get("parse_status", UNKNOWN),
        "validation_status": item.get("validation_status", UNKNOWN),
        "target_label": _format_value(target_label),
        "candidate": candidate,
        "difficulty": _format_value(difficulty),
        "error_code": _format_value(error_code),
        "bbox_xyxy": bbox_xyxy,
        "pixel_center": pixel_center,
        "image_size": _format_image_size(image_width, image_height),
        "geometry_2d_confidence": geometry_confidence,
        "validation_warnings": warnings,
        "validation_errors": errors,
        "line_error": item.get("line_error", ""),
        "unsafe": error_code == "E_UNSAFE" or difficulty == "unsafe",
        "rejected": candidate is False,
        "grounded": bbox_xyxy is not None or pixel_center is not None,
    }


def _build_summary(items: List[Dict[str, Any]]) -> Dict[str, int]:
    summary = _empty_summary()
    summary["total"] = len(items)
    for item in items:
        parse_status = item.get("parse_status")
        validation_status = item.get("validation_status")
        if parse_status == "success":
            summary["parse_success"] += 1
        elif parse_status == "failed":
            summary["parse_failed"] += 1

        if validation_status == "passed":
            summary["validation_passed"] += 1
        elif validation_status == "warning":
            summary["validation_warning"] += 1
        elif validation_status == "failed":
            summary["validation_failed"] += 1

        if item.get("unsafe"):
            summary["unsafe_count"] += 1
        if item.get("rejected"):
            summary["rejected_count"] += 1
        if item.get("grounded"):
            summary["grounding_count"] += 1
        else:
            summary["grounding_missing_count"] += 1
    return summary


def _empty_summary() -> Dict[str, int]:
    return {
        "total": 0,
        "parse_success": 0,
        "parse_failed": 0,
        "validation_passed": 0,
        "validation_warning": 0,
        "validation_failed": 0,
        "unsafe_count": 0,
        "rejected_count": 0,
        "grounding_count": 0,
        "grounding_missing_count": 0,
    }


def _bad_line_item(line_number: int, message: str) -> Dict[str, Any]:
    return {
        "image_path": f"line {line_number}",
        "parse_status": "failed",
        "validation_status": "failed",
        "normalized_json": {"error": {"code": "E_PARSE"}},
        "validation_warnings": [],
        "validation_errors": [message],
        "line_error": message,
    }


def _first_value(item: Dict[str, Any], *paths: tuple[str, ...], default: Any = UNKNOWN) -> Any:
    for path in paths:
        value = _get_nested(item, path)
        if value is not None:
            return value
    return default


def _get_nested(item: Dict[str, Any], path: tuple[str, ...]) -> Any:
    current: Any = item
    for key in path:
        if not isinstance(current, dict) or key not in current:
            return None
        current = current[key]
    return current


def _format_value(value: Any) -> str:
    if value is True:
        return "true"
    if value is False:
        return "false"
    if value is None:
        return "null"
    return str(value)


def _format_image_size(width: Any, height: Any) -> str:
    if width is None or height is None:
        return "unknown"
    return f"{width}x{height}"
