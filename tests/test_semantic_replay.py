import json
import subprocess
import sys
from pathlib import Path

from src.robot_task_inspector import (
    export_replay_subset,
    filter_replay_records,
    format_active_filter_lines,
    format_replay_detail,
    format_replay_records,
    format_replay_stats,
    get_replay_record_detail,
    load_replay_index,
    summarize_replay_records,
    write_scene_and_replay_indexes,
)


def _write_jsonl(path: Path, rows: list[dict]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        "".join(json.dumps(row, ensure_ascii=False) + "\n" for row in rows),
        encoding="utf-8",
    )


def _make_replay_run(tmp_path: Path) -> Path:
    run_dir = tmp_path / "run_20260529_150000"
    _write_jsonl(
        run_dir / "results.jsonl",
        [
            {
                "image_path": "/tmp/camera.jpg",
                "parse_status": "success",
                "validation_status": "passed",
                "parsed_json": {
                    "target": {"label": "camera", "bbox_xyxy": [1, 2, 11, 22]},
                    "geometry_2d": {"pixel_center": [6, 12]},
                },
                "normalized_json": {
                    "schema_version": "teto_robot_task.v1",
                    "scene": {"scene_version": "run_20260529_150000_item_001", "status": "valid"},
                    "target": {"label": "camera", "target_id": "obj_001", "bbox_xyxy": [1, 2, 11, 22]},
                    "geometry_2d": {
                        "pixel_center": [6, 12],
                        "image_width": 100,
                        "image_height": 80,
                        "confidence": 0.6,
                    },
                    "manipulation_assessment": {"candidate": True, "difficulty": "easy"},
                    "error": {"code": "OK"},
                },
                "pre_normalization_errors": [],
                "post_normalization_errors": [],
            },
            {
                "image_path": "/tmp/empty.jpg",
                "parse_status": "success",
                "validation_status": "warning",
                "parsed_json": {
                    "target": {"label": "unknown", "bbox_xyxy": [0, 0, 10, 10]},
                    "geometry_2d": {"pixel_center": [5, 5]},
                },
                "normalized_json": {
                    "schema_version": "teto_robot_task.v1",
                    "scene": {"scene_version": "run_20260529_150000_item_002", "status": "valid"},
                    "target": {"label": "unknown", "target_id": "unknown", "bbox_xyxy": None},
                    "geometry_2d": {
                        "pixel_center": None,
                        "image_width": 100,
                        "image_height": 80,
                        "confidence": 0.0,
                    },
                    "manipulation_assessment": {"candidate": False, "difficulty": "unknown"},
                    "error": {"code": "E_NO_TARGET"},
                },
                "pre_normalization_errors": ["raw target was unknown"],
                "post_normalization_errors": [],
            },
            {
                "image_path": "/tmp/person.jpg",
                "parse_status": "success",
                "validation_status": "warning",
                "normalized_json": {
                    "schema_version": "teto_robot_task.v1",
                    "scene": {"scene_version": "run_20260529_150000_item_003", "status": "valid"},
                    "target": {"label": "person", "target_id": "unknown", "bbox_xyxy": None},
                    "geometry_2d": {
                        "pixel_center": None,
                        "image_width": 100,
                        "image_height": 80,
                        "confidence": 0.0,
                    },
                    "manipulation_assessment": {"candidate": False, "difficulty": "unsafe"},
                    "error": {"code": "E_UNSAFE"},
                },
                "pre_normalization_errors": [],
                "post_normalization_errors": [],
            },
        ],
    )
    write_scene_and_replay_indexes(run_dir)
    return run_dir


def test_semantic_replay_stats_counts_positive_and_hard_negative(tmp_path):
    run_dir = _make_replay_run(tmp_path)
    loaded = load_replay_index(run_dir)

    stats = summarize_replay_records(loaded["records"])

    assert stats["total_count"] == 3
    assert stats["positive_replay_sample_count"] == 1
    assert stats["hard_negative_sample_count"] == 2
    assert stats["grounded_count"] == 1
    assert stats["ungrounded_count"] == 2
    assert stats["candidate_count"] == 1
    assert stats["non_candidate_count"] == 2
    assert stats["error_codes"] == {"E_NO_TARGET": 1, "E_UNSAFE": 1, "OK": 1}


def test_semantic_replay_stats_displays_run_sources(tmp_path):
    run_dir = _make_replay_run(tmp_path)
    loaded = load_replay_index(run_dir)
    stats = summarize_replay_records(loaded["records"])
    text = format_replay_stats(
        stats,
        source={
            "run_dir": loaded["run_dir"],
            "replay_index_path": loaded["replay_index_path"],
            "results_path": loaded["results_path"],
            "run_id": loaded["run_id"],
        },
    )

    assert f"run_dir: {run_dir}" in text
    assert "replay_index: replay_index.json" in text
    assert "results: results.jsonl" in text
    assert f"run_id: {run_dir.name}" in text


def test_semantic_replay_filters_hard_negative_by_reason(tmp_path):
    run_dir = _make_replay_run(tmp_path)
    records = load_replay_index(run_dir)["records"]

    selected = filter_replay_records(records, hard_negative=True, reason="E_NO_TARGET")

    assert len(selected) == 1
    assert selected[0]["scene_version"] == "run_20260529_150000_item_002"
    assert selected[0]["hard_negative_sample"] is True


def test_semantic_replay_filters_positive_samples(tmp_path):
    run_dir = _make_replay_run(tmp_path)
    records = load_replay_index(run_dir)["records"]

    selected = filter_replay_records(records, positive=True)

    assert len(selected) == 1
    assert selected[0]["positive_replay_sample"] is True
    assert selected[0]["target_label"] == "camera"


def test_semantic_replay_list_limit_limits_display_only(tmp_path):
    run_dir = _make_replay_run(tmp_path)
    records = load_replay_index(run_dir)["records"]
    selected = filter_replay_records(records, hard_negative=True)

    text = format_replay_records(selected, limit=1)

    assert len(selected) == 2
    assert "Showing 1 of 2 records" in text
    assert "[0] scene=run_20260529_150000_item_002" in text
    assert "run_20260529_150000_item_003" not in text


def test_semantic_replay_list_displays_active_filters():
    text = "\n".join(
        format_active_filter_lines(
            {
                "hard_negative": True,
                "reason": "E_NO_TARGET",
                "candidate": False,
                "grounded": False,
            }
        )
    )

    assert "Active filters:" in text
    assert "hard_negative: true" in text
    assert "reason: E_NO_TARGET" in text
    assert "candidate: false" in text
    assert "grounded: false" in text


def test_semantic_replay_show_record_returns_replay_and_result_record(tmp_path):
    run_dir = _make_replay_run(tmp_path)

    detail = get_replay_record_detail(run_dir, 1)

    assert detail["ok"] is True
    assert detail["replay_record"]["rejection_reason"] == "E_NO_TARGET"
    assert detail["result_record"]["image_path"] == "/tmp/empty.jpg"
    assert detail["normalized_summary"]["error"]["code"] == "E_NO_TARGET"
    assert detail["audit"]["raw_bbox_xyxy"] == [0, 0, 10, 10]
    assert detail["audit"]["raw_pixel_center"] == [5, 5]
    assert detail["audit"]["pre_normalization_errors"] == ["raw target was unknown"]


def test_semantic_replay_show_displays_audit_sections(tmp_path):
    run_dir = _make_replay_run(tmp_path)

    text = format_replay_detail(get_replay_record_detail(run_dir, 1))

    assert "Replay record" in text
    assert "Result record" in text
    assert "Normalized result" in text
    assert "Audit raw fields" in text
    assert "raw_bbox_xyxy: [0, 0, 10, 10]" in text
    assert "raw_pixel_center: [5, 5]" in text
    assert "pre_normalization_errors: ['raw target was unknown']" in text


def test_semantic_replay_show_displays_planner_eligibility_for_rejected_record(tmp_path):
    run_dir = _make_replay_run(tmp_path)

    text = format_replay_detail(get_replay_record_detail(run_dir, 1))

    assert "Planner gateway eligibility" in text
    assert "eligible: false" in text
    assert "status: rejected" in text
    assert "E_NO_TARGET" in text
    assert "E_NOT_CANDIDATE" in text
    assert "planner_input: null" in text
    assert "allow_robot_motion: false" in text


def test_semantic_replay_show_displays_planner_input_for_eligible_record(tmp_path):
    run_dir = _make_replay_run(tmp_path)

    text = format_replay_detail(get_replay_record_detail(run_dir, 0))

    assert "Planner gateway eligibility" in text
    assert "eligible: true" in text
    assert "Planner input skeleton" in text
    assert "contract_version: teto_planner_gateway_input.v1" in text
    assert "intent.name: hover_to_object" in text
    assert "target.target_id: obj_001" in text
    assert "target.label: camera" in text
    assert "dry_run_only: true" in text
    assert "allow_robot_motion: false" in text


def test_semantic_replay_show_planner_section_does_not_include_control_fields(tmp_path):
    run_dir = _make_replay_run(tmp_path)

    text = format_replay_detail(get_replay_record_detail(run_dir, 0))

    assert "URScript" not in text
    assert "joint_angles" not in text
    assert "trajectory" not in text
    assert "tcp_pose_world" not in text


def test_semantic_replay_show_uses_normalized_fields_for_planner_eligibility(tmp_path):
    run_dir = _make_replay_run(tmp_path)

    text = format_replay_detail(get_replay_record_detail(run_dir, 1))

    assert "raw_bbox_xyxy: [0, 0, 10, 10]" in text
    assert "raw_pixel_center: [5, 5]" in text
    assert "eligible: false" in text
    assert "E_MISSING_BBOX" in text
    assert "E_MISSING_PIXEL_CENTER" in text


def test_semantic_replay_export_subset_writes_jsonl(tmp_path):
    run_dir = _make_replay_run(tmp_path)
    export_path = tmp_path / "hard_negatives.jsonl"

    result = export_replay_subset(run_dir, export_path, hard_negative=True)

    assert result["ok"] is True
    assert result["export_count"] == 2
    rows = [json.loads(line) for line in export_path.read_text(encoding="utf-8").splitlines()]
    assert len(rows) == 2
    assert rows[0]["source_run_dir"] == str(run_dir)
    assert rows[0]["source_replay_index_path"] == str(run_dir / "replay_index.json")
    assert rows[0]["source_results_path"] == str(run_dir / "results.jsonl")
    assert rows[0]["result_record_index"] == 1
    assert rows[0]["scene_version"] == "run_20260529_150000_item_002"
    assert rows[0]["target_label"] == "unknown"
    assert rows[0]["candidate"] is False
    assert rows[0]["error_code"] == "E_NO_TARGET"
    assert rows[0]["positive_replay_sample"] is False
    assert rows[0]["hard_negative_sample"] is True
    assert rows[0]["rejection_reason"] == "E_NO_TARGET"
    assert rows[0]["replay_record"]["scene_version"] == "run_20260529_150000_item_002"
    assert rows[0]["result_record"]["image_path"] == "/tmp/empty.jpg"


def test_semantic_replay_missing_replay_index_does_not_traceback(tmp_path):
    run_dir = tmp_path / "run_20260529_150001_legacy"
    run_dir.mkdir()

    loaded = load_replay_index(run_dir)

    assert loaded["ok"] is False
    assert "replay_index.json not found" in loaded["message"]


def test_semantic_replay_result_record_index_out_of_range_reports_error(tmp_path):
    run_dir = _make_replay_run(tmp_path)
    replay_index_path = run_dir / "replay_index.json"
    replay_index = json.loads(replay_index_path.read_text(encoding="utf-8"))
    replay_index["records"][0]["result_record_index"] = 99
    replay_index_path.write_text(json.dumps(replay_index), encoding="utf-8")

    detail = get_replay_record_detail(run_dir, 0)

    assert detail["ok"] is False
    assert "result_record_index out of range" in detail["message"]


def test_semantic_replay_cli_lists_records(tmp_path):
    run_dir = _make_replay_run(tmp_path)

    completed = subprocess.run(
        [sys.executable, "scripts/semantic_replay.py", str(run_dir), "--list"],
        cwd=Path(__file__).resolve().parents[1],
        text=True,
        capture_output=True,
        check=False,
    )

    assert completed.returncode == 0
    assert "[0] scene=run_20260529_150000_item_001" in completed.stdout
    assert "label=camera" in completed.stdout
    assert "positive=True" in completed.stdout
    assert "hard_negative=True" in completed.stdout
    assert "rejection=E_NO_TARGET" in completed.stdout


def test_semantic_replay_limit_invalid_value_reports_error(tmp_path):
    run_dir = _make_replay_run(tmp_path)

    completed = subprocess.run(
        [sys.executable, "scripts/semantic_replay.py", str(run_dir), "--list", "--limit", "0"],
        cwd=Path(__file__).resolve().parents[1],
        text=True,
        capture_output=True,
        check=False,
    )

    assert completed.returncode == 1
    assert "--limit must be a positive integer" in completed.stdout
    assert "Traceback" not in completed.stderr


def test_semantic_replay_cli_exports_hard_negatives(tmp_path):
    run_dir = _make_replay_run(tmp_path)
    export_path = tmp_path / "cli_hard_negatives.jsonl"

    completed = subprocess.run(
        [
            sys.executable,
            "scripts/semantic_replay.py",
            str(run_dir),
            "--hard-negative",
            "--export",
            str(export_path),
        ],
        cwd=Path(__file__).resolve().parents[1],
        text=True,
        capture_output=True,
        check=False,
    )

    assert completed.returncode == 0
    assert "Exported 2 replay records" in completed.stdout
    rows = [json.loads(line) for line in export_path.read_text(encoding="utf-8").splitlines()]
    assert len(rows) == 2
    assert all(row["hard_negative_sample"] is True for row in rows)
