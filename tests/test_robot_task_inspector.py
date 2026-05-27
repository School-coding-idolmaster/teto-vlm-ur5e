import json
import subprocess
import sys
from pathlib import Path

from src import robot_task_inspector
from src.robot_task_inspector import (
    format_items,
    inspect_robot_task_run,
    load_results_jsonl,
)
import teto_V1


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
            },
        ],
    )

    inspection = inspect_robot_task_run(run_dir)

    assert inspection["ok"] is True
    assert inspection["summary"] == {
        "total": 3,
        "parse_success": 2,
        "parse_failed": 1,
        "validation_passed": 1,
        "validation_warning": 1,
        "validation_failed": 1,
        "unsafe_count": 1,
    }
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
