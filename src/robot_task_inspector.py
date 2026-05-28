import json
from collections import Counter
from datetime import datetime
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
        "smoke_report_md_path": str(selected_run_dir / "smoke_report.md"),
        "smoke_report_json_path": str(selected_run_dir / "smoke_report.json"),
        "summary": _build_summary(items),
        "indexes": inspect_run_indexes(selected_run_dir),
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
            f"  total:              {_summary_value(summary, 'total_count', 'total')}",
            f"  parse_success:      {_summary_value(summary, 'parse_success_count', 'parse_success')}",
            f"  parse_failed:       {_summary_value(summary, 'parse_failed_count', 'parse_failed')}",
            f"  validation_passed:  {_summary_value(summary, 'validation_passed_count', 'validation_passed')}",
            f"  validation_warning: {_summary_value(summary, 'validation_warning_count', 'validation_warning')}",
            f"  validation_failed:  {_summary_value(summary, 'validation_failed_count', 'validation_failed')}",
            f"  unsafe_count:       {_summary_value(summary, 'unsafe_count')}",
            f"  rejected_count:     {_summary_value(summary, 'rejected_count')}",
            f"  grounding_count:    {_summary_value(summary, 'grounding_count')}",
            f"  grounding_missing_count: {_summary_value(summary, 'grounding_missing_count')}",
            f"  no_target_count:    {_summary_value(summary, 'no_target_count')}",
        ]
    )
    if inspection.get("ok", False) and inspection.get("smoke_report_md_path"):
        lines.extend(
            [
                "",
                *format_index_summary_lines(inspection.get("indexes", {})),
                "",
                "Smoke report:",
                f"- Markdown: {inspection['smoke_report_md_path']}",
                f"- JSON: {inspection['smoke_report_json_path']}",
            ]
        )
    return "\n".join(lines)


def inspect_run_indexes(run_dir: Path | str) -> Dict[str, Any]:
    run_path = Path(run_dir).expanduser()
    scene_path = run_path / "scene_index.json"
    replay_path = run_path / "replay_index.json"
    warnings: List[str] = []
    errors: List[str] = []

    scene_index, scene_data = summarize_scene_index(scene_path)
    replay_index, replay_data = summarize_replay_index(replay_path)

    if not scene_index["exists"]:
        warnings.append(f"scene_index.json missing: {scene_path}")
    if not replay_index["exists"]:
        warnings.append(f"replay_index.json missing: {replay_path}")
    if scene_index.get("read_error"):
        errors.append(f"scene_index.json read failed: {scene_index['read_error']}")
    if replay_index.get("read_error"):
        errors.append(f"replay_index.json read failed: {replay_index['read_error']}")

    if scene_data is not None and replay_data is not None:
        verification = verify_scene_and_replay_indexes(scene_data, replay_data)
        warnings.extend(verification["warnings"])
        errors.extend(verification["errors"])

    return {
        "scene_index": scene_index,
        "replay_index": replay_index,
        "consistency": _consistency_result(warnings, errors),
    }


def load_replay_index(run_dir: Path | str) -> Dict[str, Any]:
    run_path = Path(run_dir).expanduser()
    replay_path = run_path / "replay_index.json"
    if not run_path.exists() or not run_path.is_dir():
        return {
            "ok": False,
            "message": f"Run directory not found: {run_path}",
            "run_dir": str(run_path),
            "replay_index_path": str(replay_path),
            "results_path": str(run_path / "results.jsonl"),
            "records": [],
            "replay_index": {},
        }
    if not replay_path.exists():
        return {
            "ok": False,
            "message": f"replay_index.json not found: {replay_path}",
            "run_dir": str(run_path),
            "replay_index_path": str(replay_path),
            "results_path": str(run_path / "results.jsonl"),
            "records": [],
            "replay_index": {},
        }

    replay_index, error = _read_index_json(replay_path)
    if error:
        return {
            "ok": False,
            "message": f"replay_index.json read failed: {error}",
            "run_dir": str(run_path),
            "replay_index_path": str(replay_path),
            "results_path": str(run_path / "results.jsonl"),
            "records": [],
            "replay_index": {},
        }

    records = replay_index.get("records", [])
    if not isinstance(records, list):
        return {
            "ok": False,
            "message": "replay_index.records is not a list",
            "run_dir": str(run_path),
            "replay_index_path": str(replay_path),
            "results_path": str(run_path / "results.jsonl"),
            "records": [],
            "replay_index": replay_index,
        }

    return {
        "ok": True,
        "message": "",
        "run_dir": str(run_path),
        "replay_index_path": str(replay_path),
        "results_path": str(replay_index.get("results_path") or run_path / "results.jsonl"),
        "records": [record for record in records if isinstance(record, dict)],
        "replay_index": replay_index,
    }


def filter_replay_records(
    records: Iterable[Dict[str, Any]],
    positive: bool | None = None,
    hard_negative: bool | None = None,
    reason: str | None = None,
    error_code: str | None = None,
    candidate: bool | None = None,
    grounded: bool | None = None,
) -> List[Dict[str, Any]]:
    selected: List[Dict[str, Any]] = []
    for record in records:
        if positive is not None and record.get("positive_replay_sample") is not positive:
            continue
        if hard_negative is not None and record.get("hard_negative_sample") is not hard_negative:
            continue
        if reason is not None and record.get("rejection_reason") != reason:
            continue
        if error_code is not None and record.get("error_code") != error_code:
            continue
        if candidate is not None and record.get("candidate") is not candidate:
            continue
        if grounded is not None and record.get("grounded") is not grounded:
            continue
        selected.append(record)
    return selected


def summarize_replay_records(records: Iterable[Dict[str, Any]]) -> Dict[str, Any]:
    selected = [record for record in records if isinstance(record, dict)]
    rejection_reasons = Counter(str(record.get("rejection_reason")) for record in selected if record.get("rejection_reason"))
    error_codes = Counter(str(record.get("error_code")) for record in selected if record.get("error_code"))
    return {
        "total_count": len(selected),
        "positive_replay_sample_count": sum(1 for record in selected if record.get("positive_replay_sample") is True),
        "hard_negative_sample_count": sum(1 for record in selected if record.get("hard_negative_sample") is True),
        "rejection_reasons": dict(sorted(rejection_reasons.items())),
        "error_codes": dict(sorted(error_codes.items())),
        "grounded_count": sum(1 for record in selected if record.get("grounded") is True),
        "ungrounded_count": sum(1 for record in selected if record.get("grounded") is False),
        "candidate_count": sum(1 for record in selected if record.get("candidate") is True),
        "non_candidate_count": sum(1 for record in selected if record.get("candidate") is False),
    }


def get_replay_record_detail(run_dir: Path | str, replay_record_index: int) -> Dict[str, Any]:
    loaded = load_replay_index(run_dir)
    if not loaded.get("ok"):
        return {"ok": False, "message": loaded.get("message", "replay index load failed")}

    records = loaded["records"]
    if replay_record_index < 0 or replay_record_index >= len(records):
        return {
            "ok": False,
            "message": f"replay record index out of range: {replay_record_index}",
            "replay_record_index": replay_record_index,
        }

    replay_record = records[replay_record_index]
    result_index = replay_record.get("result_record_index")
    if not isinstance(result_index, int):
        return {
            "ok": False,
            "message": f"result_record_index is not an integer: {result_index}",
            "replay_record_index": replay_record_index,
            "replay_record": replay_record,
        }

    results = _load_replay_results(loaded["results_path"])
    if not results.get("ok"):
        return {
            "ok": False,
            "message": results["message"],
            "replay_record_index": replay_record_index,
            "replay_record": replay_record,
        }
    result_records = results["records"]
    if result_index < 0 or result_index >= len(result_records):
        return {
            "ok": False,
            "message": f"result_record_index out of range: {result_index}",
            "replay_record_index": replay_record_index,
            "result_record_index": result_index,
            "replay_record": replay_record,
        }

    result_record = result_records[result_index]
    return {
        "ok": True,
        "message": "",
        "run_dir": loaded["run_dir"],
        "replay_index_path": loaded["replay_index_path"],
        "results_path": loaded["results_path"],
        "replay_record_index": replay_record_index,
        "result_record_index": result_index,
        "replay_record": replay_record,
        "result_record": result_record,
        "normalized_summary": _normalized_replay_summary(result_record),
        "audit": _replay_audit_summary(result_record),
    }


def export_replay_subset(
    run_dir: Path | str,
    export_path: Path | str,
    positive: bool | None = None,
    hard_negative: bool | None = None,
    reason: str | None = None,
    error_code: str | None = None,
    candidate: bool | None = None,
    grounded: bool | None = None,
) -> Dict[str, Any]:
    loaded = load_replay_index(run_dir)
    if not loaded.get("ok"):
        return {"ok": False, "message": loaded.get("message", "replay index load failed"), "export_path": str(export_path)}

    results = _load_replay_results(loaded["results_path"])
    if not results.get("ok"):
        return {"ok": False, "message": results["message"], "export_path": str(export_path)}

    selected = filter_replay_records(
        loaded["records"],
        positive=positive,
        hard_negative=hard_negative,
        reason=reason,
        error_code=error_code,
        candidate=candidate,
        grounded=grounded,
    )
    result_records = results["records"]
    export_items = []
    for replay_record in selected:
        result_index = replay_record.get("result_record_index")
        if not isinstance(result_index, int):
            return {"ok": False, "message": f"result_record_index is not an integer: {result_index}", "export_path": str(export_path)}
        if result_index < 0 or result_index >= len(result_records):
            return {"ok": False, "message": f"result_record_index out of range: {result_index}", "export_path": str(export_path)}
        export_items.append(_replay_export_item(loaded, replay_record, result_records[result_index]))

    path = Path(export_path).expanduser()
    if path.parent != Path("."):
        path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as output_file:
        for item in export_items:
            output_file.write(json.dumps(item, ensure_ascii=False) + "\n")

    return {
        "ok": True,
        "message": "",
        "export_path": str(path),
        "export_count": len(export_items),
    }


def format_replay_records(records: Iterable[Dict[str, Any]]) -> str:
    lines = []
    for index, record in enumerate(records):
        parts = [
            f"[{index}]",
            f"scene={record.get('scene_version', UNKNOWN)}",
            f"label={record.get('target_label', UNKNOWN)}",
            f"candidate={_format_replay_value(record.get('candidate', UNKNOWN))}",
            f"error={record.get('error_code', UNKNOWN)}",
            f"positive={_format_replay_value(record.get('positive_replay_sample', False))}",
            f"hard_negative={_format_replay_value(record.get('hard_negative_sample', False))}",
            f"grounded={_format_replay_value(record.get('grounded', False))}",
        ]
        if record.get("rejection_reason"):
            parts.append(f"rejection={record['rejection_reason']}")
        if record.get("image_path"):
            parts.append(f"image={record['image_path']}")
        lines.append(" ".join(parts))
    return "\n".join(lines) if lines else "(no replay records)"


def format_replay_stats(stats: Dict[str, Any]) -> str:
    lines = [
        "Replay statistics",
        f"  total: {stats.get('total_count', 0)}",
        f"  positive: {stats.get('positive_replay_sample_count', 0)}",
        f"  hard_negative: {stats.get('hard_negative_sample_count', 0)}",
        f"  grounded: {stats.get('grounded_count', 0)}",
        f"  ungrounded: {stats.get('ungrounded_count', 0)}",
        f"  candidate: {stats.get('candidate_count', 0)}",
        f"  non_candidate: {stats.get('non_candidate_count', 0)}",
        "  rejection_reasons:",
    ]
    _append_count_lines(lines, stats.get("rejection_reasons", {}))
    lines.append("  error_codes:")
    _append_count_lines(lines, stats.get("error_codes", {}))
    return "\n".join(lines)


def format_replay_detail(detail: Dict[str, Any]) -> str:
    if not detail.get("ok"):
        return f"Error: {detail.get('message', 'replay detail unavailable')}"
    lines = [
        f"Replay record [{detail['replay_record_index']}]",
        json.dumps(detail["replay_record"], ensure_ascii=False, indent=2),
        "",
        f"Result record [{detail['result_record_index']}]",
        f"  image_path: {detail['result_record'].get('image_path', UNKNOWN)}",
        f"  parse_status: {detail['result_record'].get('parse_status', UNKNOWN)}",
        f"  validation_status: {detail['result_record'].get('validation_status', UNKNOWN)}",
        "",
        "normalized_json summary",
        json.dumps(detail["normalized_summary"], ensure_ascii=False, indent=2),
        "",
        "audit",
        json.dumps(detail["audit"], ensure_ascii=False, indent=2),
    ]
    return "\n".join(lines)


def summarize_scene_index(path: Path | str) -> tuple[Dict[str, Any], Dict[str, Any] | None]:
    index_path = Path(path).expanduser()
    summary: Dict[str, Any] = {
        "exists": index_path.exists(),
        "path": str(index_path),
    }
    if not index_path.exists():
        return summary, None

    data, error = _read_index_json(index_path)
    if error:
        summary["read_error"] = error
        return summary, None

    scenes = data.get("scenes", [])
    if not isinstance(scenes, list):
        scenes = []
    for key in [
        "scene_index_version",
        "run_id",
        "total_count",
        "valid_scene_count",
        "invalid_scene_count",
        "candidate_scene_count",
        "rejected_scene_count",
        "no_target_count",
        "grounding_count",
        "grounding_missing_count",
    ]:
        summary[key] = data.get(key, 0 if key.endswith("_count") else "")
    summary["scene_count"] = len(scenes)
    return summary, data


def summarize_replay_index(path: Path | str) -> tuple[Dict[str, Any], Dict[str, Any] | None]:
    index_path = Path(path).expanduser()
    summary: Dict[str, Any] = {
        "exists": index_path.exists(),
        "path": str(index_path),
    }
    if not index_path.exists():
        return summary, None

    data, error = _read_index_json(index_path)
    if error:
        summary["read_error"] = error
        return summary, None

    records = data.get("records", [])
    if not isinstance(records, list):
        records = []
    reasons = Counter(
        str(record.get("rejection_reason"))
        for record in records
        if isinstance(record, dict) and record.get("rejection_reason")
    )
    summary.update(
        {
            "replay_index_version": data.get("replay_index_version", ""),
            "run_id": data.get("run_id", ""),
            "record_count": len(records),
            "positive_replay_sample_count": sum(
                1 for record in records if isinstance(record, dict) and record.get("positive_replay_sample") is True
            ),
            "hard_negative_sample_count": sum(
                1 for record in records if isinstance(record, dict) and record.get("hard_negative_sample") is True
            ),
            "rejection_reasons": dict(sorted(reasons.items())),
        }
    )
    return summary, data


def verify_scene_and_replay_indexes(scene_index: Dict[str, Any], replay_index: Dict[str, Any]) -> Dict[str, Any]:
    warnings: List[str] = []
    errors: List[str] = []
    scenes = scene_index.get("scenes", [])
    records = replay_index.get("records", [])
    if not isinstance(scenes, list):
        scenes = []
        errors.append("scene_index.scenes is not a list")
    if not isinstance(records, list):
        records = []
        errors.append("replay_index.records is not a list")

    total_count = scene_index.get("total_count")
    if len(scenes) != total_count:
        errors.append(f"scene_index.scenes count {len(scenes)} does not match total_count {total_count}")
    if len(records) != total_count:
        errors.append(f"replay_index.records count {len(records)} does not match scene_index total_count {total_count}")

    scene_run_id = scene_index.get("run_id")
    replay_run_id = replay_index.get("run_id")
    if scene_run_id != replay_run_id:
        errors.append(f"run_id mismatch: scene_index={scene_run_id}, replay_index={replay_run_id}")

    scene_versions = scene_index.get("scene_versions")
    if not isinstance(scene_versions, list):
        scene_versions = [scene.get("scene_version") for scene in scenes if isinstance(scene, dict)]
    replay_versions = [record.get("scene_version") for record in records if isinstance(record, dict)]
    if set(scene_versions) != set(replay_versions):
        warnings.append("scene_version set mismatch between scene_index and replay_index")

    return _consistency_result(warnings, errors)


def format_index_summary_lines(indexes: Dict[str, Any]) -> List[str]:
    if not indexes:
        return []
    scene_index = indexes.get("scene_index", {})
    replay_index = indexes.get("replay_index", {})
    consistency = indexes.get("consistency", {"status": "warning", "warnings": [], "errors": []})
    lines = ["Index summary"]
    if scene_index.get("exists"):
        lines.extend(
            [
                "  scene_index:",
                f"    path: {_display_index_path(scene_index.get('path', ''))}",
                f"    scenes: {scene_index.get('scene_count', 0)}",
                f"    valid: {scene_index.get('valid_scene_count', 0)}",
                f"    rejected: {scene_index.get('rejected_scene_count', 0)}",
                f"    no_target: {scene_index.get('no_target_count', 0)}",
                f"    grounding: {scene_index.get('grounding_count', 0)}",
                f"    grounding_missing: {scene_index.get('grounding_missing_count', 0)}",
            ]
        )
    else:
        lines.append("  scene_index: missing")

    if replay_index.get("exists"):
        lines.extend(
            [
                "  replay_index:",
                f"    path: {_display_index_path(replay_index.get('path', ''))}",
                f"    records: {replay_index.get('record_count', 0)}",
                f"    positive_replay_samples: {replay_index.get('positive_replay_sample_count', 0)}",
                f"    hard_negative_samples: {replay_index.get('hard_negative_sample_count', 0)}",
                "    rejection_reasons:",
            ]
        )
        reasons = replay_index.get("rejection_reasons", {})
        if reasons:
            for reason, count in reasons.items():
                lines.append(f"      {reason}: {count}")
        else:
            lines.append("      none: 0")
    else:
        lines.append("  replay_index: missing")

    lines.append(f"  index_consistency: {consistency.get('status', 'warning')}")
    for error in consistency.get("errors", []):
        lines.append(f"    error: {error}")
    for warning in consistency.get("warnings", []):
        lines.append(f"    warning: {warning}")
    return lines


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
                f"scene_version: {item['scene_version']}",
                f"scene.status: {item['scene_status']}",
                f"target_id: {item['target_id']}",
                f"target.label: {item['target_label']}",
                f"candidate: {_format_value(item['candidate'])}",
                f"difficulty: {item['difficulty']}",
                f"error.code: {item['error_code']}",
                f"bbox_xyxy: {_format_value(item['bbox_xyxy'])}",
                f"pixel_center: {_format_value(item['pixel_center'])}",
                f"image_size: {item['image_size']}",
                f"geometry_2d.confidence: {_format_value(item['geometry_2d_confidence'])}",
                f"raw_bbox_xyxy: {_format_value(item['raw_bbox_xyxy'])}",
                f"raw_pixel_center: {_format_value(item['raw_pixel_center'])}",
                f"raw_geometry_2d.confidence: {_format_value(item['raw_geometry_2d_confidence'])}",
            ]
        )
        _append_list(lines, "validation_warnings", item.get("validation_warnings", []))
        _append_list(lines, "pre_normalization_errors", item.get("pre_normalization_errors", []))
        _append_list(lines, "post_normalization_errors", item.get("post_normalization_errors", []))
        if item.get("line_error"):
            lines.append(f"line_error: {item['line_error']}")
        lines.append("")
    return "\n".join(lines).rstrip()


def write_smoke_report(run_dir: Path | str) -> Dict[str, str]:
    inspection = inspect_robot_task_run(run_dir)
    if not inspection.get("ok"):
        raise FileNotFoundError(inspection.get("message", "robot_task_json inspection failed"))

    run_path = Path(str(inspection["run_dir"]))
    md_path = run_path / "smoke_report.md"
    json_path = run_path / "smoke_report.json"
    report = {
        "run_path": inspection["run_dir"],
        "results_path": inspection["results_path"],
        "summary": inspection["summary"],
        "items": inspection["items"],
    }
    json_path.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    md_path.write_text(_format_smoke_report_markdown(report), encoding="utf-8")
    return {
        "smoke_report_md_path": str(md_path),
        "smoke_report_json_path": str(json_path),
    }


def build_scene_index(inspection: Dict[str, Any]) -> Dict[str, Any]:
    run_path = Path(str(inspection.get("run_dir", "")))
    items = list(inspection.get("items", []))
    summary = inspection.get("summary", _empty_summary())
    scenes = [_scene_index_record(item) for item in items]
    return {
        "scene_index_version": "teto_scene_index.v1",
        "run_id": run_path.name,
        "generated_at": _generated_at(),
        "results_path": inspection.get("results_path", ""),
        "smoke_report_json_path": inspection.get("smoke_report_json_path", str(run_path / "smoke_report.json")),
        "total_count": _summary_value(summary, "total_count", "total"),
        "valid_scene_count": sum(1 for scene in scenes if scene["scene_status"] == "valid"),
        "invalid_scene_count": sum(1 for scene in scenes if scene["scene_status"] == "invalid"),
        "candidate_scene_count": sum(1 for scene in scenes if scene["candidate"] is True),
        "rejected_scene_count": sum(1 for scene in scenes if scene["candidate"] is False),
        "no_target_count": _summary_value(summary, "no_target_count"),
        "unsafe_count": _summary_value(summary, "unsafe_count"),
        "grounding_count": _summary_value(summary, "grounding_count"),
        "grounding_missing_count": _summary_value(summary, "grounding_missing_count"),
        "scene_versions": [scene["scene_version"] for scene in scenes],
        "scenes": scenes,
    }


def build_replay_index(inspection: Dict[str, Any]) -> Dict[str, Any]:
    run_path = Path(str(inspection.get("run_dir", "")))
    results_path = inspection.get("results_path", "")
    return {
        "replay_index_version": "teto_replay_index.v1",
        "run_id": run_path.name,
        "generated_at": _generated_at(),
        "results_path": results_path,
        "records": [_replay_index_record(item, results_path) for item in inspection.get("items", [])],
    }


def write_scene_and_replay_indexes(run_dir: Path | str) -> Dict[str, str]:
    inspection = inspect_robot_task_run(run_dir)
    if not inspection.get("ok"):
        raise FileNotFoundError(inspection.get("message", "robot_task_json inspection failed"))

    run_path = Path(str(inspection["run_dir"]))
    scene_index_path = run_path / "scene_index.json"
    replay_index_path = run_path / "replay_index.json"
    scene_index_path.write_text(
        json.dumps(build_scene_index(inspection), ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    replay_index_path.write_text(
        json.dumps(build_replay_index(inspection), ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    return {
        "scene_index_path": str(scene_index_path),
        "replay_index_path": str(replay_index_path),
    }


def _read_index_json(path: Path) -> tuple[Dict[str, Any], str]:
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except OSError as exc:
        return {}, str(exc)
    except json.JSONDecodeError as exc:
        return {}, f"{exc.msg} at column {exc.colno}"
    if not isinstance(data, dict):
        return {}, "top-level JSON value must be an object"
    return data, ""


def _load_replay_results(results_path: Path | str) -> Dict[str, Any]:
    path = Path(results_path).expanduser()
    if not path.exists():
        return {
            "ok": False,
            "message": f"results.jsonl not found: {path}",
            "results_path": str(path),
            "records": [],
        }
    try:
        records = load_results_jsonl(path)
    except OSError as exc:
        return {
            "ok": False,
            "message": f"results.jsonl read failed: {exc}",
            "results_path": str(path),
            "records": [],
        }
    return {
        "ok": True,
        "message": "",
        "results_path": str(path),
        "records": records,
    }


def _normalized_replay_summary(result_record: Dict[str, Any]) -> Dict[str, Any]:
    normalized = result_record.get("normalized_json", {})
    if not isinstance(normalized, dict):
        normalized = {}
    return {
        "scene": normalized.get("scene", {}),
        "target": normalized.get("target", {}),
        "geometry_2d": normalized.get("geometry_2d", {}),
        "manipulation_assessment": normalized.get("manipulation_assessment", {}),
        "error": normalized.get("error", {}),
    }


def _replay_audit_summary(result_record: Dict[str, Any]) -> Dict[str, Any]:
    raw_bbox = _first_value(
        result_record,
        ("parsed_json", "target", "bbox_xyxy"),
        ("parsed_json", "target", "bbox"),
        ("parsed_json", "geometry_2d", "bbox_xyxy"),
        default=None,
    )
    raw_pixel_center = _first_value(
        result_record,
        ("parsed_json", "geometry_2d", "pixel_center"),
        ("parsed_json", "target", "pixel_center"),
        default=None,
    )
    return {
        "raw_bbox_xyxy": raw_bbox,
        "raw_pixel_center": raw_pixel_center,
        "pre_normalization_errors": result_record.get("pre_normalization_errors", []),
        "post_normalization_errors": result_record.get("post_normalization_errors", []),
    }


def _replay_export_item(
    loaded: Dict[str, Any],
    replay_record: Dict[str, Any],
    result_record: Dict[str, Any],
) -> Dict[str, Any]:
    return {
        "source_run_dir": loaded["run_dir"],
        "source_replay_index_path": loaded["replay_index_path"],
        "source_results_path": loaded["results_path"],
        "result_record_index": replay_record.get("result_record_index"),
        "scene_version": replay_record.get("scene_version", UNKNOWN),
        "image_path": replay_record.get("image_path", UNKNOWN),
        "target_label": replay_record.get("target_label", UNKNOWN),
        "candidate": replay_record.get("candidate"),
        "error_code": replay_record.get("error_code", UNKNOWN),
        "positive_replay_sample": replay_record.get("positive_replay_sample", False),
        "hard_negative_sample": replay_record.get("hard_negative_sample", False),
        "rejection_reason": replay_record.get("rejection_reason", ""),
        "replay_record": replay_record,
        "result_record": result_record,
    }


def _append_count_lines(lines: List[str], counts: Dict[str, Any]) -> None:
    if not counts:
        lines.append("    none: 0")
        return
    for key, count in counts.items():
        lines.append(f"    {key}: {count}")


def _consistency_result(warnings: List[str], errors: List[str]) -> Dict[str, Any]:
    status = "ok"
    if errors:
        status = "error"
    elif warnings:
        status = "warning"
    return {
        "status": status,
        "warnings": warnings,
        "errors": errors,
    }


def _display_index_path(value: Any) -> str:
    text = str(value)
    return Path(text).name if text else ""


def _inspect_item(index: int, item: Dict[str, Any]) -> Dict[str, Any]:
    target_label = _first_value(
        item,
        ("normalized_json", "target", "label"),
        ("parsed_json", "target", "label"),
        default=UNKNOWN,
    )
    target_id = _first_value(
        item,
        ("normalized_json", "target", "target_id"),
        default=UNKNOWN,
    )
    scene_version = _first_value(
        item,
        ("normalized_json", "scene", "scene_version"),
        default=UNKNOWN,
    )
    scene_status = _first_value(
        item,
        ("normalized_json", "scene", "status"),
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
        default=None,
    )
    pixel_center = _first_value(
        item,
        ("normalized_json", "geometry_2d", "pixel_center"),
        default=None,
    )
    image_width = _first_value(item, ("normalized_json", "geometry_2d", "image_width"), default=None)
    image_height = _first_value(item, ("normalized_json", "geometry_2d", "image_height"), default=None)
    geometry_confidence = _first_value(
        item,
        ("normalized_json", "geometry_2d", "confidence"),
        default=None,
    )
    raw_bbox_xyxy = _first_value(
        item,
        ("parsed_json", "target", "bbox_xyxy"),
        ("parsed_json", "target", "bbox"),
        ("parsed_json", "geometry_2d", "bbox_xyxy"),
        default=None,
    )
    raw_pixel_center = _first_value(
        item,
        ("parsed_json", "geometry_2d", "pixel_center"),
        ("parsed_json", "target", "pixel_center"),
        default=None,
    )
    raw_geometry_confidence = _first_value(
        item,
        ("parsed_json", "geometry_2d", "confidence"),
        ("parsed_json", "target", "grounding_confidence"),
        default=None,
    )
    warnings = item.get("validation_warnings", [])
    if not isinstance(warnings, list):
        warnings = [str(warnings)]
    errors = item.get("validation_errors", [])
    if not isinstance(errors, list):
        errors = [str(errors)]
    pre_errors = item.get("pre_normalization_errors", item.get("raw_validation_errors", errors))
    if not isinstance(pre_errors, list):
        pre_errors = [str(pre_errors)]
    post_errors = item.get("post_normalization_errors", item.get("normalized_validation_errors", errors))
    if not isinstance(post_errors, list):
        post_errors = [str(post_errors)]

    return {
        "index": index,
        "image_path": item.get("image_path", UNKNOWN),
        "parse_status": item.get("parse_status", UNKNOWN),
        "validation_status": item.get("validation_status", UNKNOWN),
        "scene_version": _format_value(scene_version),
        "scene_status": _format_value(scene_status),
        "target_id": _format_value(target_id),
        "target_label": _format_value(target_label),
        "candidate": candidate,
        "difficulty": _format_value(difficulty),
        "error_code": _format_value(error_code),
        "bbox_xyxy": bbox_xyxy,
        "pixel_center": pixel_center,
        "image_size": _format_image_size(image_width, image_height),
        "geometry_2d_confidence": geometry_confidence,
        "raw_bbox_xyxy": raw_bbox_xyxy,
        "raw_pixel_center": raw_pixel_center,
        "raw_geometry_2d_confidence": raw_geometry_confidence,
        "validation_warnings": warnings,
        "validation_errors": errors,
        "raw_validation_errors": pre_errors,
        "pre_normalization_errors": pre_errors,
        "normalized_validation_errors": post_errors,
        "post_normalization_errors": post_errors,
        "line_error": item.get("line_error", ""),
        "unsafe": error_code == "E_UNSAFE" or difficulty == "unsafe",
        "rejected": candidate is False,
        "grounded": bbox_xyxy is not None or pixel_center is not None,
        "no_target": error_code == "E_NO_TARGET",
    }


def _scene_index_record(item: Dict[str, Any]) -> Dict[str, Any]:
    return {
        "scene_version": item.get("scene_version", UNKNOWN),
        "scene_status": item.get("scene_status", UNKNOWN),
        "image_path": item.get("image_path", UNKNOWN),
        "target_id": item.get("target_id", UNKNOWN),
        "target_label": item.get("target_label", UNKNOWN),
        "candidate": item.get("candidate", UNKNOWN),
        "error_code": item.get("error_code", UNKNOWN),
        "grounded": bool(item.get("grounded")),
        "bbox_xyxy": item.get("bbox_xyxy"),
        "pixel_center": item.get("pixel_center"),
        "geometry_2d_confidence": item.get("geometry_2d_confidence"),
        "result_record_index": max(int(item.get("index", 1)) - 1, 0),
    }


def _replay_index_record(item: Dict[str, Any], results_path: str) -> Dict[str, Any]:
    scene_status = item.get("scene_status", UNKNOWN)
    candidate = item.get("candidate", UNKNOWN)
    error_code = item.get("error_code", UNKNOWN)
    parse_status = item.get("parse_status", UNKNOWN)
    validation_status = item.get("validation_status", UNKNOWN)
    positive = scene_status == "valid" and candidate is True and error_code == "OK"
    hard_negative = (
        candidate is False
        or error_code != "OK"
        or scene_status != "valid"
        or parse_status == "failed"
        or validation_status == "failed"
        or item.get("no_target") is True
        or item.get("unsafe") is True
    )
    geometry_confidence = item.get("geometry_2d_confidence")
    return {
        "scene_version": item.get("scene_version", UNKNOWN),
        "image_path": item.get("image_path", UNKNOWN),
        "result_jsonl_path": results_path,
        "result_record_index": max(int(item.get("index", 1)) - 1, 0),
        "scene_status": scene_status,
        "target_id": item.get("target_id", UNKNOWN),
        "target_label": item.get("target_label", UNKNOWN),
        "candidate": candidate,
        "error_code": error_code,
        "grounded": bool(item.get("grounded")),
        "bbox_xyxy": item.get("bbox_xyxy"),
        "pixel_center": item.get("pixel_center"),
        "confidence": {
            "semantic": None,
            "geometry": geometry_confidence,
            "overall": geometry_confidence,
        },
        "positive_replay_sample": positive,
        "hard_negative_sample": hard_negative,
        "rejection_reason": _rejection_reason(error_code, scene_status, candidate),
    }


def _rejection_reason(error_code: Any, scene_status: Any, candidate: Any) -> str:
    if error_code != "OK":
        return str(error_code)
    if scene_status != "valid":
        return f"scene_status:{scene_status}"
    if candidate is False:
        return "not_candidate"
    return ""


def _build_summary(items: List[Dict[str, Any]]) -> Dict[str, int]:
    summary = _empty_summary()
    summary["total"] = len(items)
    summary["total_count"] = len(items)
    for item in items:
        parse_status = item.get("parse_status")
        validation_status = item.get("validation_status")
        if parse_status == "success":
            summary["parse_success"] += 1
            summary["parse_success_count"] += 1
        elif parse_status == "failed":
            summary["parse_failed"] += 1
            summary["parse_failed_count"] += 1

        if validation_status == "passed":
            summary["validation_passed"] += 1
            summary["validation_passed_count"] += 1
        elif validation_status == "warning":
            summary["validation_warning"] += 1
            summary["validation_warning_count"] += 1
        elif validation_status == "failed":
            summary["validation_failed"] += 1
            summary["validation_failed_count"] += 1

        if item.get("unsafe"):
            summary["unsafe_count"] += 1
        if item.get("rejected"):
            summary["rejected_count"] += 1
        if item.get("grounded"):
            summary["grounding_count"] += 1
        else:
            summary["grounding_missing_count"] += 1
        if item.get("no_target"):
            summary["no_target_count"] += 1
    return summary


def _empty_summary() -> Dict[str, int]:
    return {
        "total": 0,
        "total_count": 0,
        "parse_success": 0,
        "parse_success_count": 0,
        "parse_failed": 0,
        "parse_failed_count": 0,
        "validation_passed": 0,
        "validation_passed_count": 0,
        "validation_warning": 0,
        "validation_warning_count": 0,
        "validation_failed": 0,
        "validation_failed_count": 0,
        "unsafe_count": 0,
        "rejected_count": 0,
        "grounding_count": 0,
        "grounding_missing_count": 0,
        "no_target_count": 0,
    }


def _bad_line_item(line_number: int, message: str) -> Dict[str, Any]:
    return {
        "image_path": f"line {line_number}",
        "parse_status": "failed",
        "validation_status": "failed",
        "normalized_json": {"error": {"code": "E_PARSE"}},
        "validation_warnings": [],
        "validation_errors": [message],
        "raw_validation_errors": [message],
        "pre_normalization_errors": [message],
        "normalized_validation_errors": [message],
        "post_normalization_errors": [message],
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


def _format_replay_value(value: Any) -> str:
    if value is True:
        return "True"
    if value is False:
        return "False"
    if value is None:
        return "null"
    return str(value)


def _format_image_size(width: Any, height: Any) -> str:
    if width is None or height is None:
        return "unknown"
    return f"{width}x{height}"


def _summary_value(summary: Dict[str, Any], key: str, fallback_key: str | None = None) -> Any:
    if key in summary:
        return summary[key]
    if fallback_key and fallback_key in summary:
        return summary[fallback_key]
    return 0


def _generated_at() -> str:
    return datetime.now().isoformat(timespec="seconds")


def _append_list(lines: List[str], label: str, values: List[Any]) -> None:
    if values:
        lines.append(f"{label}:")
        for value in values:
            lines.append(f"  - {value}")
    else:
        lines.append(f"{label}: []")


def _format_smoke_report_markdown(report: Dict[str, Any]) -> str:
    summary = report["summary"]
    lines = [
        "# TETO robot_task_json smoke report",
        "",
        f"Run path: {report['run_path']}",
        f"Results: {report['results_path']}",
        "",
        "## Summary",
        "",
        f"- total_count: {summary['total_count']}",
        f"- parse_success_count: {summary['parse_success_count']}",
        f"- validation_passed_count: {summary['validation_passed_count']}",
        f"- validation_failed_count: {summary['validation_failed_count']}",
        f"- rejected_count: {summary['rejected_count']}",
        f"- unsafe_count: {summary['unsafe_count']}",
        f"- grounding_count: {summary['grounding_count']}",
        f"- grounding_missing_count: {summary['grounding_missing_count']}",
        f"- no_target_count: {summary['no_target_count']}",
        f"- parse_failed_count: {summary['parse_failed_count']}",
        "",
        "## Items",
        "",
    ]
    for item in report["items"]:
        lines.extend(
            [
                f"### [{item['index']}]",
                "",
                f"- image_path: {item['image_path']}",
                f"- parse_status: {item['parse_status']}",
                f"- validation_status: {item['validation_status']}",
                f"- scene_version: {item['scene_version']}",
                f"- scene.status: {item['scene_status']}",
                f"- target_id: {item['target_id']}",
                f"- target.label: {item['target_label']}",
                f"- candidate: {_format_value(item['candidate'])}",
                f"- difficulty: {item['difficulty']}",
                f"- error.code: {item['error_code']}",
                f"- bbox_xyxy: {_format_value(item['bbox_xyxy'])}",
                f"- pixel_center: {_format_value(item['pixel_center'])}",
                f"- image_size: {item['image_size']}",
                f"- geometry_2d.confidence: {_format_value(item['geometry_2d_confidence'])}",
                f"- raw_bbox_xyxy: {_format_value(item['raw_bbox_xyxy'])}",
                f"- raw_pixel_center: {_format_value(item['raw_pixel_center'])}",
                f"- raw_geometry_2d.confidence: {_format_value(item['raw_geometry_2d_confidence'])}",
                f"- validation_warnings: {item['validation_warnings']}",
                f"- pre_normalization_errors: {item['pre_normalization_errors']}",
                f"- post_normalization_errors: {item['post_normalization_errors']}",
                "",
            ]
        )
    return "\n".join(lines)
