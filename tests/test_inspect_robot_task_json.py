import json
import subprocess
import sys
from pathlib import Path

from src.robot_task_inspector import write_scene_and_replay_indexes


def _write_jsonl(path: Path, rows: list[dict]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        "".join(json.dumps(row, ensure_ascii=False) + "\n" for row in rows),
        encoding="utf-8",
    )


def test_inspect_robot_task_json_indexes_option_displays_index_summary(tmp_path):
    run_dir = tmp_path / "run_20260527_120003_cli_indexes"
    _write_jsonl(
        run_dir / "results.jsonl",
        [
            {
                "image_path": "/tmp/camera.jpg",
                "parse_status": "success",
                "validation_status": "passed",
                "normalized_json": {
                    "scene": {"scene_version": "run_cli_indexes_item_001", "status": "valid"},
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
                    "scene": {"scene_version": "run_cli_indexes_item_002", "status": "valid"},
                    "target": {"label": "unknown", "target_id": "unknown", "bbox_xyxy": None},
                    "geometry_2d": {"pixel_center": None, "confidence": 0.0},
                    "manipulation_assessment": {"candidate": False, "difficulty": "unknown"},
                    "error": {"code": "E_NO_TARGET"},
                },
            },
        ],
    )
    write_scene_and_replay_indexes(run_dir)

    completed = subprocess.run(
        [
            sys.executable,
            "scripts/inspect_robot_task_json.py",
            "--run-dir",
            str(run_dir),
            "--indexes",
        ],
        cwd=Path(__file__).resolve().parents[1],
        text=True,
        capture_output=True,
        check=False,
    )

    assert completed.returncode == 0
    assert "scene_index:" in completed.stdout
    assert "replay_index:" in completed.stdout
    assert "positive_replay_samples: 1" in completed.stdout
    assert "hard_negative_samples: 1" in completed.stdout
    assert "E_NO_TARGET: 1" in completed.stdout
    assert "index_consistency: ok" in completed.stdout


def test_inspect_robot_task_json_legacy_run_missing_indexes_does_not_crash(tmp_path):
    run_dir = tmp_path / "run_20260527_120003_cli_legacy"
    _write_jsonl(
        run_dir / "results.jsonl",
        [{"image_path": "/tmp/legacy.jpg", "parse_status": "success", "validation_status": "passed"}],
    )

    completed = subprocess.run(
        [
            sys.executable,
            "scripts/inspect_robot_task_json.py",
            "--run-dir",
            str(run_dir),
            "--indexes",
        ],
        cwd=Path(__file__).resolve().parents[1],
        text=True,
        capture_output=True,
        check=False,
    )

    assert completed.returncode == 0
    assert "scene_index: missing" in completed.stdout
    assert "replay_index: missing" in completed.stdout
    assert "index_consistency: warning" in completed.stdout
