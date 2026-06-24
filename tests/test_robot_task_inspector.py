import json
import subprocess
import sys
from pathlib import Path

import pytest

from src import robot_task_inspector
from src.robot_task_inspector import (
    build_replay_index,
    build_scene_index,
    format_items,
    format_summary,
    inspect_run_indexes,
    inspect_robot_task_run,
    load_results_jsonl,
    write_scene_and_replay_indexes,
    write_smoke_report,
)
import teto_V1


pytestmark = [pytest.mark.legacy, pytest.mark.debug]


def _write_jsonl(path: Path, rows: list[dict]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        "".join(json.dumps(row, ensure_ascii=False) + "\n" for row in rows),
        encoding="utf-8",
    )


def test_inspector_handles_success_failed_and_unsafe_count(tmp_path):
    run_dir = tmp_path / "run_20260527_120000"
    _write_jsonl(
        run_dir / "results.jsonl",
        [
            {
                "image_path": "/tmp/cup.jpg",
                "parse_status": "success",
                "validation_status": "passed",
                "normalized_json": {
                    "target": {"label": "red cup"},
                    "manipulation_assessment": {"candidate": True, "difficulty": "easy"},
                    "error": {"code": "OK"},
                },
                "validation_warnings": [],
                "validation_errors": [],
            },
            {
                "image_path": "/tmp/bird.jpg",
                "parse_status": "success",
                "validation_status": "warning",
                "normalized_json": {
                    "target": {"label": "bird"},
                    "manipulation_assessment": {"candidate": False, "difficulty": "unsafe"},
                    "error": {"code": "E_UNSAFE"},
                },
                "validation_warnings": ["living target detected; safety patch forced candidate=false"],
                "validation_errors": [],
            },
            {
                "image_path": "/tmp/bad.jpg",
                "parse_status": "failed",
                "validation_status": "failed",
                "normalized_json": {
                    "target": {"label": "unknown"},
                    "manipulation_assessment": {"candidate": False, "difficulty": "unknown"},
                    "error": {"code": "E_PARSE"},
                },
                "validation_errors": ["invalid JSON: Expecting value"],
            },
        ],
    )

    inspection = inspect_robot_task_run(run_dir)

    assert inspection["ok"] is True
    assert inspection["summary"]["total"] == 3
    assert inspection["summary"]["total_count"] == 3
    assert inspection["summary"]["parse_success"] == 2
    assert inspection["summary"]["parse_success_count"] == 2
    assert inspection["summary"]["parse_failed"] == 1
    assert inspection["summary"]["parse_failed_count"] == 1
    assert inspection["summary"]["validation_passed"] == 1
    assert inspection["summary"]["validation_passed_count"] == 1
    assert inspection["summary"]["validation_warning"] == 1
    assert inspection["summary"]["validation_warning_count"] == 1
    assert inspection["summary"]["validation_failed"] == 1
    assert inspection["summary"]["validation_failed_count"] == 1
    assert inspection["summary"]["unsafe_count"] == 1
    assert inspection["summary"]["rejected_count"] == 2
    assert inspection["summary"]["grounding_count"] == 0
    assert inspection["summary"]["grounding_missing_count"] == 3
    assert inspection["summary"]["no_target_count"] == 0
    assert inspection["items"][1]["target_label"] == "bird"
    assert inspection["items"][1]["candidate"] is False


def test_inspector_falls_back_to_parsed_json_without_normalized_json(tmp_path):
    run_dir = tmp_path / "run_20260527_120001"
    _write_jsonl(
        run_dir / "results.jsonl",
        [
            {
                "image_path": "/tmp/fallback.jpg",
                "parse_status": "success",
                "validation_status": "passed",
                "parsed_json": {
                    "target": {"label": "box", "candidate": True},
                    "manipulation": {"difficulty": "medium"},
                    "error": {"code": "OK"},
                },
            }
        ],
    )

    item = inspect_robot_task_run(run_dir)["items"][0]

    assert item["target_label"] == "box"
    assert item["candidate"] is True
    assert item["difficulty"] == "medium"
    assert item["validation_warnings"] == []
    assert item["validation_errors"] == []


def test_empty_results_jsonl_returns_zero_summary(tmp_path):
    run_dir = tmp_path / "run_20260527_120002"
    run_dir.mkdir()
    (run_dir / "results.jsonl").write_text("", encoding="utf-8")

    inspection = inspect_robot_task_run(run_dir)

    assert inspection["ok"] is True
    assert inspection["summary"]["total"] == 0
    assert inspection["items"] == []


def test_run_dir_missing_is_graceful(tmp_path):
    inspection = inspect_robot_task_run(tmp_path / "missing_run")

    assert inspection["ok"] is False
    assert "Run directory not found" in inspection["message"]
    assert inspection["summary"]["total"] == 0


def test_missing_robot_task_root_is_graceful(tmp_path, monkeypatch):
    missing_root = tmp_path / "outputs" / "results" / "robot_task_json"
    monkeypatch.setattr(robot_task_inspector, "ROBOT_TASK_JSON_ROOT", missing_root)

    inspection = inspect_robot_task_run()

    assert inspection["ok"] is False
    assert "No robot_task_json runs found" in inspection["message"]


def test_bad_json_line_becomes_item_and_later_lines_continue(tmp_path):
    results_path = tmp_path / "run_20260527_120003" / "results.jsonl"
    results_path.parent.mkdir()
    results_path.write_text(
        '{"image_path": "/tmp/one.jpg", "parse_status": "success", "validation_status": "passed"}\n'
        "{bad json\n"
        '{"image_path": "/tmp/two.jpg", "parse_status": "success", "validation_status": "passed"}\n',
        encoding="utf-8",
    )

    items = load_results_jsonl(results_path)
    inspection = inspect_robot_task_run(results_path.parent)

    assert len(items) == 3
    assert inspection["summary"]["total"] == 3
    assert inspection["summary"]["parse_failed"] == 1
    assert inspection["items"][1]["error_code"] == "E_PARSE"
    assert inspection["items"][1]["line_error"]


def test_detail_format_displays_pre_and_post_normalization_errors(tmp_path):
    run_dir = tmp_path / "run_20260527_120003_errors"
    _write_jsonl(
        run_dir / "results.jsonl",
        [
            {
                "image_path": "/tmp/failed.jpg",
                "parse_status": "success",
                "validation_status": "failed",
                "normalized_json": {
                    "target": {"label": "unknown"},
                    "manipulation_assessment": {"candidate": False, "difficulty": "unknown"},
                    "error": {"code": "E_NO_TARGET"},
                },
                "pre_normalization_errors": ["unknown target should use a no-target error code"],
                "post_normalization_errors": [],
            }
        ],
    )

    text = format_items(inspect_robot_task_run(run_dir)["items"])

    assert "pre_normalization_errors:" in text
    assert "  - unknown target should use a no-target error code" in text
    assert "post_normalization_errors: []" in text


def test_detail_format_displays_grounding_fields(tmp_path):
    run_dir = tmp_path / "run_20260527_120003_grounding"
    _write_jsonl(
        run_dir / "results.jsonl",
        [
            {
                "image_path": "/tmp/red_cup.jpg",
                "parse_status": "success",
                "validation_status": "warning",
                "normalized_json": {
                    "target": {"label": "red cup", "bbox_xyxy": [420, 180, 610, 430]},
                    "geometry_2d": {
                        "pixel_center": [515, 305],
                        "image_width": 1280,
                        "image_height": 720,
                        "confidence": 0.78,
                    },
                    "manipulation_assessment": {"candidate": True, "difficulty": "easy"},
                    "error": {"code": "OK"},
                },
                "validation_warnings": ["pixel_center was inferred from bbox_xyxy"],
            }
        ],
    )

    inspection = inspect_robot_task_run(run_dir)
    text = format_items(inspection["items"])

    assert inspection["summary"]["grounding_count"] == 1
    assert inspection["summary"]["grounding_missing_count"] == 0
    assert "bbox_xyxy: [420, 180, 610, 430]" in text
    assert "pixel_center: [515, 305]" in text
    assert "image_size: 1280x720" in text
    assert "geometry_2d.confidence: 0.78" in text


def test_unsafe_and_rejected_counts_are_distinct(tmp_path):
    run_dir = tmp_path / "run_20260527_120003_counts"
    _write_jsonl(
        run_dir / "results.jsonl",
        [
            {
                "image_path": "/tmp/cup.jpg",
                "parse_status": "success",
                "validation_status": "passed",
                "normalized_json": {
                    "target": {"label": "cup"},
                    "manipulation_assessment": {"candidate": True, "difficulty": "easy"},
                    "error": {"code": "OK"},
                },
            },
            {
                "image_path": "/tmp/empty.jpg",
                "parse_status": "success",
                "validation_status": "failed",
                "normalized_json": {
                    "target": {"label": "unknown"},
                    "manipulation_assessment": {"candidate": False, "difficulty": "unknown"},
                    "error": {"code": "E_NO_TARGET"},
                },
            },
            {
                "image_path": "/tmp/person.jpg",
                "parse_status": "success",
                "validation_status": "failed",
                "normalized_json": {
                    "target": {"label": "person"},
                    "manipulation_assessment": {"candidate": False, "difficulty": "unsafe"},
                    "error": {"code": "E_UNSAFE"},
                },
            },
        ],
    )

    summary = inspect_robot_task_run(run_dir)["summary"]

    assert summary["unsafe_count"] == 1
    assert summary["rejected_count"] == 2
    assert summary["grounding_count"] == 0
    assert summary["grounding_missing_count"] == 3
    assert summary["no_target_count"] == 1


def test_legacy_result_without_geometry_does_not_crash(tmp_path):
    run_dir = tmp_path / "run_20260527_120003_legacy"
    _write_jsonl(
        run_dir / "results.jsonl",
        [
            {
                "image_path": "/tmp/legacy.jpg",
                "parse_status": "success",
                "validation_status": "passed",
                "normalized_json": {
                    "target": {"label": "box"},
                    "manipulation_assessment": {"candidate": True, "difficulty": "easy"},
                    "error": {"code": "OK"},
                },
            }
        ],
    )

    inspection = inspect_robot_task_run(run_dir)
    text = format_items(inspection["items"])

    assert inspection["summary"]["grounding_count"] == 0
    assert inspection["summary"]["grounding_missing_count"] == 1
    assert "bbox_xyxy: null" in text
    assert "pixel_center: null" in text
    assert "image_size: unknown" in text


def test_grounding_uses_normalized_fields_not_raw_parsed_bbox(tmp_path):
    run_dir = tmp_path / "run_20260527_120003_normalized_grounding"
    _write_jsonl(
        run_dir / "results.jsonl",
        [
            {
                "image_path": "/tmp/no_target.jpg",
                "parse_status": "success",
                "validation_status": "passed",
                "parsed_json": {
                    "target": {
                        "label": "unknown",
                        "bbox_xyxy": [0, 0, 588, 756],
                    },
                    "geometry_2d": {
                        "pixel_center": [294, 294],
                        "confidence": 1.0,
                    },
                },
                "normalized_json": {
                    "target": {
                        "label": "unknown",
                        "bbox_xyxy": None,
                    },
                    "geometry_2d": {
                        "pixel_center": None,
                        "image_width": 768,
                        "image_height": 1024,
                        "confidence": 0.0,
                    },
                    "manipulation_assessment": {"candidate": False, "difficulty": "unknown"},
                    "error": {"code": "E_NO_TARGET"},
                },
            }
        ],
    )

    inspection = inspect_robot_task_run(run_dir)
    item = inspection["items"][0]

    assert item["bbox_xyxy"] is None
    assert item["pixel_center"] is None
    assert item["geometry_2d_confidence"] == 0.0
    assert item["raw_bbox_xyxy"] == [0, 0, 588, 756]
    assert item["raw_pixel_center"] == [294, 294]
    assert item["raw_geometry_2d_confidence"] == 1.0
    assert item["grounded"] is False
    assert inspection["summary"]["grounding_count"] == 0
    assert inspection["summary"]["grounding_missing_count"] == 1
    assert inspection["summary"]["no_target_count"] == 1


def test_smoke_report_files_are_generated(tmp_path):
    run_dir = tmp_path / "run_20260527_120003_report"
    _write_jsonl(
        run_dir / "results.jsonl",
        [
            {
                "image_path": "/tmp/report.jpg",
                "parse_status": "success",
                "validation_status": "warning",
                "normalized_json": {
                    "scene": {
                        "scene_version": "run_20260527_120003_report_item_001",
                        "status": "valid",
                    },
                    "target": {"label": "box", "target_id": "obj_001", "bbox_xyxy": [1, 2, 11, 22]},
                    "geometry_2d": {
                        "pixel_center": [6, 12],
                        "image_width": 100,
                        "image_height": 80,
                        "confidence": 0.5,
                    },
                    "manipulation_assessment": {"candidate": True, "difficulty": "easy"},
                    "error": {"code": "OK"},
                },
                "validation_warnings": [],
                "pre_normalization_errors": [],
                "post_normalization_errors": [],
            }
        ],
    )

    paths = write_smoke_report(run_dir)
    md_text = Path(paths["smoke_report_md_path"]).read_text(encoding="utf-8")
    json_data = json.loads(Path(paths["smoke_report_json_path"]).read_text(encoding="utf-8"))

    assert "total_count: 1" in md_text
    assert "pre_normalization_errors" in md_text
    assert "post_normalization_errors" in md_text
    assert json_data["summary"]["total"] == 1
    assert json_data["summary"]["total_count"] == 1
    assert json_data["summary"]["parse_success_count"] == 1
    assert json_data["summary"]["grounding_count"] == 1
    assert json_data["items"][0]["target_label"] == "box"
    assert json_data["items"][0]["target_id"] == "obj_001"
    assert json_data["items"][0]["scene_version"] == "run_20260527_120003_report_item_001"
    assert json_data["items"][0]["scene_status"] == "valid"
    assert "scene_version: run_20260527_120003_report_item_001" in md_text
    assert "target_id: obj_001" in md_text


def test_inspector_displays_scene_snapshot_fields(tmp_path):
    run_dir = tmp_path / "run_20260527_120003_scene"
    _write_jsonl(
        run_dir / "results.jsonl",
        [
            {
                "image_path": "/tmp/scene.jpg",
                "parse_status": "success",
                "validation_status": "passed",
                "normalized_json": {
                    "scene": {
                        "scene_version": "run_20260527_120003_scene_item_001",
                        "status": "valid",
                    },
                    "target": {
                        "label": "camera",
                        "target_id": "obj_001",
                        "bbox_xyxy": [1, 2, 11, 22],
                    },
                    "geometry_2d": {
                        "pixel_center": [6, 12],
                        "image_width": 100,
                        "image_height": 80,
                        "confidence": 0.5,
                    },
                    "manipulation_assessment": {"candidate": True, "difficulty": "easy"},
                    "error": {"code": "OK"},
                },
            }
        ],
    )

    inspection = inspect_robot_task_run(run_dir)
    item = inspection["items"][0]
    text = format_items(inspection["items"])

    assert item["scene_version"] == "run_20260527_120003_scene_item_001"
    assert item["scene_status"] == "valid"
    assert item["target_id"] == "obj_001"
    assert "scene_version: run_20260527_120003_scene_item_001" in text
    assert "scene.status: valid" in text
    assert "target_id: obj_001" in text


def test_scene_index_files_are_generated(tmp_path):
    run_dir = tmp_path / "run_20260527_120003_scene_index"
    _write_jsonl(
        run_dir / "results.jsonl",
        [
            {
                "image_path": "/tmp/camera.jpg",
                "parse_status": "success",
                "validation_status": "passed",
                "normalized_json": {
                    "scene": {"scene_version": "run_scene_index_item_001", "status": "valid"},
                    "target": {"label": "camera", "target_id": "obj_001", "bbox_xyxy": [1, 2, 11, 22]},
                    "geometry_2d": {"pixel_center": [6, 12], "confidence": 0.6},
                    "manipulation_assessment": {"candidate": True, "difficulty": "easy"},
                    "error": {"code": "OK"},
                },
            }
        ],
    )

    paths = write_scene_and_replay_indexes(run_dir)
    scene_index = json.loads(Path(paths["scene_index_path"]).read_text(encoding="utf-8"))

    assert Path(paths["scene_index_path"]).exists()
    assert scene_index["scene_index_version"] == "teto_scene_index.v1"
    assert scene_index["run_id"] == run_dir.name
    assert scene_index["results_path"] == str(run_dir / "results.jsonl")
    assert scene_index["scene_versions"] == ["run_scene_index_item_001"]


def test_scene_index_counts_valid_invalid_rejected(tmp_path):
    run_dir = tmp_path / "run_20260527_120003_scene_counts"
    _write_jsonl(
        run_dir / "results.jsonl",
        [
            {
                "image_path": "/tmp/camera.jpg",
                "parse_status": "success",
                "validation_status": "passed",
                "normalized_json": {
                    "scene": {"scene_version": "run_scene_counts_item_001", "status": "valid"},
                    "target": {"label": "camera", "target_id": "obj_001", "bbox_xyxy": [1, 2, 11, 22]},
                    "geometry_2d": {"pixel_center": [6, 12], "confidence": 0.6},
                    "manipulation_assessment": {"candidate": True, "difficulty": "easy"},
                    "error": {"code": "OK"},
                },
            },
            {
                "image_path": "/tmp/person.jpg",
                "parse_status": "success",
                "validation_status": "failed",
                "normalized_json": {
                    "scene": {"scene_version": "run_scene_counts_item_002", "status": "invalid"},
                    "target": {"label": "person", "target_id": "unknown", "bbox_xyxy": None},
                    "geometry_2d": {"pixel_center": None, "confidence": 0.0},
                    "manipulation_assessment": {"candidate": False, "difficulty": "unsafe"},
                    "error": {"code": "E_UNSAFE"},
                },
            },
        ],
    )

    scene_index = build_scene_index(inspect_robot_task_run(run_dir))

    assert scene_index["total_count"] == 2
    assert scene_index["valid_scene_count"] == 1
    assert scene_index["invalid_scene_count"] == 1
    assert scene_index["candidate_scene_count"] == 1
    assert scene_index["rejected_scene_count"] == 1
    assert scene_index["unsafe_count"] == 1
    assert scene_index["grounding_count"] == 1
    assert scene_index["grounding_missing_count"] == 1


def test_scene_index_records_scene_version_and_target_id(tmp_path):
    run_dir = tmp_path / "run_20260527_120003_scene_record"
    _write_jsonl(
        run_dir / "results.jsonl",
        [
            {
                "image_path": "/tmp/camera.jpg",
                "parse_status": "success",
                "validation_status": "passed",
                "normalized_json": {
                    "scene": {"scene_version": "run_scene_record_item_001", "status": "valid"},
                    "target": {"label": "camera", "target_id": "obj_001", "bbox_xyxy": [1, 2, 11, 22]},
                    "geometry_2d": {"pixel_center": [6, 12], "confidence": 0.6},
                    "manipulation_assessment": {"candidate": True, "difficulty": "easy"},
                    "error": {"code": "OK"},
                },
            }
        ],
    )

    record = build_scene_index(inspect_robot_task_run(run_dir))["scenes"][0]

    assert record["scene_version"] == "run_scene_record_item_001"
    assert record["scene_status"] == "valid"
    assert record["target_id"] == "obj_001"
    assert record["target_label"] == "camera"
    assert record["grounded"] is True
    assert record["bbox_xyxy"] == [1, 2, 11, 22]
    assert record["pixel_center"] == [6, 12]
    assert record["geometry_2d_confidence"] == 0.6
    assert record["result_record_index"] == 0


def test_replay_index_files_are_generated(tmp_path):
    run_dir = tmp_path / "run_20260527_120003_replay_index"
    _write_jsonl(
        run_dir / "results.jsonl",
        [
            {
                "image_path": "/tmp/camera.jpg",
                "parse_status": "success",
                "validation_status": "passed",
                "normalized_json": {
                    "scene": {"scene_version": "run_replay_index_item_001", "status": "valid"},
                    "target": {"label": "camera", "target_id": "obj_001", "bbox_xyxy": [1, 2, 11, 22]},
                    "geometry_2d": {"pixel_center": [6, 12], "confidence": 0.6},
                    "manipulation_assessment": {"candidate": True, "difficulty": "easy"},
                    "error": {"code": "OK"},
                },
            }
        ],
    )

    paths = write_scene_and_replay_indexes(run_dir)
    replay_index = json.loads(Path(paths["replay_index_path"]).read_text(encoding="utf-8"))

    assert Path(paths["replay_index_path"]).exists()
    assert replay_index["replay_index_version"] == "teto_replay_index.v1"
    assert replay_index["run_id"] == run_dir.name
    assert replay_index["results_path"] == str(run_dir / "results.jsonl")
    assert replay_index["records"][0]["result_jsonl_path"] == str(run_dir / "results.jsonl")


def test_replay_index_positive_and_hard_negative_flags(tmp_path):
    run_dir = tmp_path / "run_20260527_120003_replay_flags"
    _write_jsonl(
        run_dir / "results.jsonl",
        [
            {
                "image_path": "/tmp/camera.jpg",
                "parse_status": "success",
                "validation_status": "passed",
                "normalized_json": {
                    "scene": {"scene_version": "run_replay_flags_item_001", "status": "valid"},
                    "target": {"label": "camera", "target_id": "obj_001", "bbox_xyxy": [1, 2, 11, 22]},
                    "geometry_2d": {"pixel_center": [6, 12], "confidence": 0.6},
                    "manipulation_assessment": {"candidate": True, "difficulty": "easy"},
                    "error": {"code": "OK"},
                },
            },
            {
                "image_path": "/tmp/empty.jpg",
                "parse_status": "success",
                "validation_status": "failed",
                "normalized_json": {
                    "scene": {"scene_version": "run_replay_flags_item_002", "status": "invalid"},
                    "target": {"label": "unknown", "target_id": "unknown", "bbox_xyxy": None},
                    "geometry_2d": {"pixel_center": None, "confidence": 0.0},
                    "manipulation_assessment": {"candidate": False, "difficulty": "unknown"},
                    "error": {"code": "E_NO_TARGET"},
                },
            },
        ],
    )

    records = build_replay_index(inspect_robot_task_run(run_dir))["records"]

    assert records[0]["positive_replay_sample"] is True
    assert records[0]["hard_negative_sample"] is False
    assert records[0]["rejection_reason"] == ""
    assert records[0]["confidence"] == {"semantic": None, "geometry": 0.6, "overall": 0.6}
    assert records[1]["positive_replay_sample"] is False
    assert records[1]["hard_negative_sample"] is True
    assert records[1]["rejection_reason"] == "E_NO_TARGET"


def test_inspect_run_indexes_reports_scene_and_replay_summary(tmp_path):
    run_dir = tmp_path / "run_20260527_120003_index_summary"
    _write_jsonl(
        run_dir / "results.jsonl",
        [
            {
                "image_path": "/tmp/camera.jpg",
                "parse_status": "success",
                "validation_status": "passed",
                "normalized_json": {
                    "scene": {"scene_version": "run_index_summary_item_001", "status": "valid"},
                    "target": {"label": "camera", "target_id": "obj_001", "bbox_xyxy": [1, 2, 11, 22]},
                    "geometry_2d": {"pixel_center": [6, 12], "confidence": 0.6},
                    "manipulation_assessment": {"candidate": True, "difficulty": "easy"},
                    "error": {"code": "OK"},
                },
            },
            {
                "image_path": "/tmp/empty.jpg",
                "parse_status": "success",
                "validation_status": "failed",
                "normalized_json": {
                    "scene": {"scene_version": "run_index_summary_item_002", "status": "valid"},
                    "target": {"label": "unknown", "target_id": "unknown", "bbox_xyxy": None},
                    "geometry_2d": {"pixel_center": None, "confidence": 0.0},
                    "manipulation_assessment": {"candidate": False, "difficulty": "unknown"},
                    "error": {"code": "E_NO_TARGET"},
                },
            },
        ],
    )
    write_scene_and_replay_indexes(run_dir)

    indexes = inspect_run_indexes(run_dir)

    assert indexes["scene_index"]["exists"] is True
    assert indexes["scene_index"]["scene_count"] == 2
    assert indexes["scene_index"]["total_count"] == 2
    assert indexes["replay_index"]["exists"] is True
    assert indexes["replay_index"]["record_count"] == 2
    assert indexes["replay_index"]["positive_replay_sample_count"] == 1
    assert indexes["replay_index"]["hard_negative_sample_count"] == 1
    assert indexes["consistency"]["status"] == "ok"


def test_inspect_run_indexes_reports_rejection_reason_counts(tmp_path):
    run_dir = tmp_path / "run_20260527_120003_rejection_counts"
    _write_jsonl(
        run_dir / "results.jsonl",
        [
            {
                "image_path": "/tmp/empty1.jpg",
                "parse_status": "success",
                "validation_status": "failed",
                "normalized_json": {
                    "scene": {"scene_version": "run_rejection_counts_item_001", "status": "valid"},
                    "target": {"label": "unknown", "target_id": "unknown", "bbox_xyxy": None},
                    "geometry_2d": {"pixel_center": None, "confidence": 0.0},
                    "manipulation_assessment": {"candidate": False, "difficulty": "unknown"},
                    "error": {"code": "E_NO_TARGET"},
                },
            },
            {
                "image_path": "/tmp/empty2.jpg",
                "parse_status": "success",
                "validation_status": "failed",
                "normalized_json": {
                    "scene": {"scene_version": "run_rejection_counts_item_002", "status": "valid"},
                    "target": {"label": "unknown", "target_id": "unknown", "bbox_xyxy": None},
                    "geometry_2d": {"pixel_center": None, "confidence": 0.0},
                    "manipulation_assessment": {"candidate": False, "difficulty": "unknown"},
                    "error": {"code": "E_NO_TARGET"},
                },
            },
        ],
    )
    write_scene_and_replay_indexes(run_dir)

    indexes = inspect_run_indexes(run_dir)

    assert indexes["replay_index"]["rejection_reasons"] == {"E_NO_TARGET": 2}


def test_inspect_run_indexes_detects_missing_indexes_without_crashing(tmp_path):
    run_dir = tmp_path / "run_20260527_120003_missing_indexes"
    run_dir.mkdir()

    indexes = inspect_run_indexes(run_dir)

    assert indexes["scene_index"]["exists"] is False
    assert indexes["replay_index"]["exists"] is False
    assert indexes["consistency"]["status"] == "warning"
    assert indexes["consistency"]["errors"] == []
    assert len(indexes["consistency"]["warnings"]) == 2


def test_inspect_run_indexes_detects_count_mismatch(tmp_path):
    run_dir = tmp_path / "run_20260527_120003_count_mismatch"
    run_dir.mkdir()
    (run_dir / "scene_index.json").write_text(
        json.dumps(
            {
                "scene_index_version": "teto_scene_index.v1",
                "run_id": run_dir.name,
                "total_count": 2,
                "scene_versions": ["scene_001", "scene_002"],
                "scenes": [
                    {"scene_version": "scene_001"},
                    {"scene_version": "scene_002"},
                ],
            }
        ),
        encoding="utf-8",
    )
    (run_dir / "replay_index.json").write_text(
        json.dumps(
            {
                "replay_index_version": "teto_replay_index.v1",
                "run_id": run_dir.name,
                "records": [{"scene_version": "scene_001"}],
            }
        ),
        encoding="utf-8",
    )

    indexes = inspect_run_indexes(run_dir)

    assert indexes["consistency"]["status"] == "error"
    assert "records count 1" in indexes["consistency"]["errors"][0]


def test_inspector_summary_displays_index_paths_and_counts(tmp_path):
    run_dir = tmp_path / "run_20260527_120003_index_display"
    _write_jsonl(
        run_dir / "results.jsonl",
        [
            {
                "image_path": "/tmp/camera.jpg",
                "parse_status": "success",
                "validation_status": "passed",
                "normalized_json": {
                    "scene": {"scene_version": "run_index_display_item_001", "status": "valid"},
                    "target": {"label": "camera", "target_id": "obj_001", "bbox_xyxy": [1, 2, 11, 22]},
                    "geometry_2d": {"pixel_center": [6, 12], "confidence": 0.6},
                    "manipulation_assessment": {"candidate": True, "difficulty": "easy"},
                    "error": {"code": "OK"},
                },
            }
        ],
    )
    write_scene_and_replay_indexes(run_dir)

    text = format_summary(inspect_robot_task_run(run_dir))

    assert "Index summary" in text
    assert "scene_index:" in text
    assert "path: scene_index.json" in text
    assert "scenes: 1" in text
    assert "replay_index:" in text
    assert "path: replay_index.json" in text
    assert "records: 1" in text
    assert "positive_replay_samples: 1" in text
    assert "index_consistency: ok" in text


def test_legacy_results_without_scene_do_not_crash_index_generation(tmp_path):
    run_dir = tmp_path / "run_20260527_120003_legacy_index"
    _write_jsonl(
        run_dir / "results.jsonl",
        [
            {
                "image_path": "/tmp/legacy.jpg",
                "parse_status": "success",
                "validation_status": "passed",
                "normalized_json": {
                    "target": {"label": "box"},
                    "manipulation_assessment": {"candidate": True, "difficulty": "easy"},
                    "error": {"code": "OK"},
                },
            }
        ],
    )

    paths = write_scene_and_replay_indexes(run_dir)
    scene_index = json.loads(Path(paths["scene_index_path"]).read_text(encoding="utf-8"))
    replay_index = json.loads(Path(paths["replay_index_path"]).read_text(encoding="utf-8"))

    assert scene_index["scenes"][0]["scene_version"] == "unknown"
    assert scene_index["scenes"][0]["scene_status"] == "unknown"
    assert scene_index["scenes"][0]["target_id"] == "unknown"
    assert replay_index["records"][0]["hard_negative_sample"] is True


def test_format_items_honors_limit(tmp_path):
    run_dir = tmp_path / "run_20260527_120004"
    _write_jsonl(
        run_dir / "results.jsonl",
        [
            {"image_path": "/tmp/one.jpg", "parse_status": "success", "validation_status": "passed"},
            {"image_path": "/tmp/two.jpg", "parse_status": "success", "validation_status": "passed"},
        ],
    )

    text = format_items(inspect_robot_task_run(run_dir)["items"], limit=1)

    assert "[1]" in text
    assert "[2]" not in text


def test_script_reads_specified_run_dir(tmp_path):
    run_dir = tmp_path / "run_20260527_120005"
    _write_jsonl(
        run_dir / "results.jsonl",
        [{"image_path": "/tmp/one.jpg", "parse_status": "success", "validation_status": "passed"}],
    )

    completed = subprocess.run(
        [
            sys.executable,
            "scripts/inspect_robot_task_json.py",
            "--run-dir",
            str(run_dir),
            "--details",
        ],
        cwd=Path(__file__).resolve().parents[1],
        text=True,
        capture_output=True,
        check=False,
    )

    assert completed.returncode == 0
    assert "TETO robot_task_json inspector" in completed.stdout
    assert "total:              1" in completed.stdout
    assert "image_path: /tmp/one.jpg" in completed.stdout


def test_inspector_does_not_write_results_files(tmp_path):
    run_dir = tmp_path / "run_20260527_120006"
    results_path = run_dir / "results.jsonl"
    _write_jsonl(
        results_path,
        [{"image_path": "/tmp/one.jpg", "parse_status": "success", "validation_status": "passed"}],
    )
    before = {path.name: path.read_text(encoding="utf-8") for path in run_dir.iterdir()}

    inspect_robot_task_run(run_dir)

    after = {path.name: path.read_text(encoding="utf-8") for path in run_dir.iterdir()}
    assert after == before


def test_launcher_does_not_trigger_inspector_for_regular_batch(monkeypatch):
    called = False

    def fake_inspect(run_dir=None):
        nonlocal called
        called = True
        return {"ok": True, "summary": {}, "items": []}

    monkeypatch.setattr(teto_V1, "inspect_robot_task_run", fake_inspect)

    teto_V1.maybe_show_robot_task_inspection({"prompt_type": "describe_image", "run_dir": "/tmp/run"})

    assert called is False


def test_launcher_uses_robot_task_run_dir_for_inspector(monkeypatch, capsys):
    captured_run_dir = ""

    def fake_inspect(run_dir=None):
        nonlocal captured_run_dir
        captured_run_dir = run_dir
        return {
            "ok": True,
            "run_dir": str(run_dir),
            "results_path": str(Path(run_dir) / "results.jsonl"),
            "summary": {
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
                "no_target_count": 0,
            },
            "items": [],
        }

    monkeypatch.setattr(teto_V1, "inspect_robot_task_run", fake_inspect)
    monkeypatch.setattr("builtins.input", lambda prompt="": "")

    teto_V1.maybe_show_robot_task_inspection(
        {"prompt_type": "robot_task_json", "run_dir": "/tmp/correct_run", "output_dir": "/tmp/wrong_run"}
    )

    assert captured_run_dir == "/tmp/correct_run"
    assert "TETO robot_task_json inspector" in capsys.readouterr().out
