import json

from PIL import Image

from src import batch_recognition


class FakeInferencer:
    def __init__(self, backend="mock"):
        self.backend = backend

    def infer(self, image_path, prompt):
        return {"response": f"seen {image_path.name}"}


class RobotTaskInferencer:
    def __init__(self, backend="mock"):
        self.backend = backend

    def infer(self, image_path, prompt):
        return {
            "response": json.dumps(
                {
                    "schema_version": "teto_robot_task.v1",
                    "task_type": "target_analysis",
                    "user_instruction": "pick the red cup",
                    "target": {
                        "label": "red cup",
                        "approx_position": "center",
                        "visibility": "clear",
                    },
                    "spatial_context": {
                        "surface": "table",
                        "nearby_objects": ["box"],
                        "relations": [],
                        "obstacles": [],
                    },
                    "manipulation_assessment": {
                        "candidate": True,
                        "difficulty": "easy",
                        "reason": "clear target",
                    },
                    "confidence": {
                        "semantic": "high",
                        "spatial": "medium",
                        "overall": "medium",
                    },
                    "error": {
                        "code": "OK",
                        "message": "",
                    },
                }
            )
        }


def test_batch_recognition_creates_run_files_and_updates_index(tmp_path, monkeypatch):
    input_dir = tmp_path / "images"
    input_dir.mkdir()
    Image.new("RGB", (8, 8), (255, 0, 0)).save(input_dir / "one.jpg")
    Image.new("RGB", (8, 8), (0, 255, 0)).save(input_dir / "two.png")

    monkeypatch.setattr(batch_recognition, "VLMInferencer", FakeInferencer)

    output_root = tmp_path / "recognition_runs"
    index_path = tmp_path / "results" / "index.jsonl"
    monkeypatch.setattr(batch_recognition, "results_index_path", lambda: index_path)
    result = batch_recognition.run_batch_recognition(
        input_dir,
        prompt_type="describe_image",
        backend="mock",
        output_root=output_root,
    )

    run_dir = output_root / result["run_name"]
    results_path = run_dir / "results.jsonl"
    summary_path = run_dir / "summary.json"
    errors_path = run_dir / "errors.log"

    assert result["ok"] is True
    assert result["run_dir"] == str(run_dir)
    assert result["output_dir"] == str(run_dir)
    assert result["results_path"] == str(results_path)
    assert result["summary_path"] == str(summary_path)
    assert result["errors_path"] == str(errors_path)
    assert result["index_path"] == str(index_path)
    assert results_path.exists()
    assert summary_path.exists()
    assert errors_path.exists()
    assert index_path.exists()

    result_rows = [json.loads(line) for line in results_path.read_text(encoding="utf-8").splitlines()]
    assert len(result_rows) == 2
    assert {row["status"] for row in result_rows} == {"success"}
    assert all(row["prompt_type"] == "describe_image" for row in result_rows)
    assert all(row["backend"] == "mock" for row in result_rows)
    assert all("prompt" in row for row in result_rows)
    assert errors_path.read_text(encoding="utf-8") == ""

    summary = json.loads(summary_path.read_text(encoding="utf-8"))
    assert summary["run_name"] == result["run_name"]
    assert summary["created_at"] == result["created_at"]
    assert summary["input_dir"] == str(input_dir)
    assert summary["output_dir"] == str(run_dir)
    assert summary["total"] == 2
    assert summary["success"] == 2
    assert summary["failed"] == 0

    index_rows = [json.loads(line) for line in index_path.read_text(encoding="utf-8").splitlines()]
    assert index_rows == [
        {
            "type": "batch_recognition",
            "run_name": result["run_name"],
            "created_at": result["created_at"],
            "input_dir": str(input_dir),
            "output_dir": str(run_dir),
            "prompt_type": "describe_image",
            "backend": "mock",
            "total": 2,
            "success": 2,
            "failed": 0,
        }
    ]


def test_batch_recognition_accepts_custom_prompt(tmp_path, monkeypatch):
    input_dir = tmp_path / "images"
    input_dir.mkdir()
    Image.new("RGB", (8, 8), (0, 0, 255)).save(input_dir / "one.jpg")

    monkeypatch.setattr(batch_recognition, "VLMInferencer", FakeInferencer)

    output_root = tmp_path / "recognition_runs"
    index_path = tmp_path / "results" / "index.jsonl"
    monkeypatch.setattr(batch_recognition, "results_index_path", lambda: index_path)
    result = batch_recognition.run_batch_recognition(
        input_dir,
        prompt_type="freeform",
        prompt="List objects near the center.",
        backend="mock",
        output_root=output_root,
    )

    results_path = output_root / result["run_name"] / "results.jsonl"
    result_rows = [json.loads(line) for line in results_path.read_text(encoding="utf-8").splitlines()]

    assert result["ok"] is True
    assert result["prompt_type"] == "freeform"
    assert result_rows[0]["prompt_type"] == "freeform"
    assert result_rows[0]["prompt"] == "List objects near the center."


def test_batch_recognition_saves_robot_task_json_fields(tmp_path, monkeypatch):
    input_dir = tmp_path / "images"
    input_dir.mkdir()
    Image.new("RGB", (8, 8), (255, 255, 255)).save(input_dir / "one.jpg")

    monkeypatch.setattr(batch_recognition, "VLMInferencer", RobotTaskInferencer)

    output_root = tmp_path / "recognition_runs"
    index_path = tmp_path / "results" / "index.jsonl"
    monkeypatch.setattr(batch_recognition, "results_index_path", lambda: index_path)
    result = batch_recognition.run_batch_recognition(
        input_dir,
        prompt_type="robot_task_json",
        prompt="robot prompt",
        backend="mock",
        output_root=output_root,
    )

    results_path = output_root / result["run_name"] / "results.jsonl"
    result_rows = [json.loads(line) for line in results_path.read_text(encoding="utf-8").splitlines()]

    assert result_rows[0]["raw_response"] == result_rows[0]["response"]
    assert result_rows[0]["parse_status"] == "success"
    assert result_rows[0]["validation_status"] == "warning"
    assert result_rows[0]["validation_errors"] == []
    assert result_rows[0]["validation_warnings"] == ["2D grounding is missing"]
    assert result_rows[0]["parsed_json"]["target"]["label"] == "red cup"
    assert result_rows[0]["normalized_json"]["target"]["label"] == "red cup"
    assert result_rows[0]["normalized_json"]["target"]["target_id"] == "obj_001"
    assert result_rows[0]["normalized_json"]["geometry_2d"]["image_width"] == 8
    assert result_rows[0]["normalized_json"]["geometry_2d"]["image_height"] == 8
    assert result_rows[0]["normalized_json"]["scene"]["scene_version"] == f"{result['run_name']}_item_001"
    assert result_rows[0]["normalized_json"]["scene"]["image_path"] == str(input_dir / "one.jpg")
    assert result_rows[0]["normalized_json"]["scene"]["image_width"] == 8
    assert result_rows[0]["normalized_json"]["scene"]["image_height"] == 8
    assert result_rows[0]["normalized_json"]["scene"]["record_type"] == "legacy_rgb_only_record"
    assert result_rows[0]["normalized_json"]["scene"]["source"] == "legacy_semantic_image"
    assert result_rows[0]["normalized_json"]["scene"]["is_realsense_scene_snapshot"] is False
    assert result_rows[0]["normalized_json"]["scene"]["status"] == "valid"
    assert (output_root / result["run_name"] / "input_manifest.json").exists()
    assert (output_root / result["run_name"] / "smoke_report.md").exists()
    assert (output_root / result["run_name"] / "smoke_report.json").exists()
    assert result["smoke_report_md_path"] == str(output_root / result["run_name"] / "smoke_report.md")
    assert result["smoke_report_json_path"] == str(output_root / result["run_name"] / "smoke_report.json")
    assert (output_root / result["run_name"] / "scene_index.json").exists()
    assert (output_root / result["run_name"] / "replay_index.json").exists()
    assert result["scene_index_path"] == str(output_root / result["run_name"] / "scene_index.json")
    assert result["replay_index_path"] == str(output_root / result["run_name"] / "replay_index.json")


def test_batch_recognition_writes_scene_and_replay_indexes_for_robot_task_json(tmp_path, monkeypatch):
    input_dir = tmp_path / "images"
    input_dir.mkdir()
    Image.new("RGB", (8, 8), (255, 255, 255)).save(input_dir / "one.jpg")

    monkeypatch.setattr(batch_recognition, "VLMInferencer", RobotTaskInferencer)

    output_root = tmp_path / "recognition_runs"
    index_path = tmp_path / "results" / "index.jsonl"
    monkeypatch.setattr(batch_recognition, "results_index_path", lambda: index_path)
    result = batch_recognition.run_batch_recognition(
        input_dir,
        prompt_type="robot_task_json",
        prompt="robot prompt",
        backend="mock",
        output_root=output_root,
    )

    scene_index = json.loads((output_root / result["run_name"] / "scene_index.json").read_text(encoding="utf-8"))
    replay_index = json.loads((output_root / result["run_name"] / "replay_index.json").read_text(encoding="utf-8"))

    assert scene_index["scene_index_version"] == "teto_scene_index.v1"
    assert scene_index["run_id"] == result["run_name"]
    assert scene_index["total_count"] == 1
    assert scene_index["candidate_scene_count"] == 1
    assert scene_index["scenes"][0]["scene_version"] == f"{result['run_name']}_item_001"
    assert scene_index["scenes"][0]["target_id"] == "obj_001"
    assert replay_index["replay_index_version"] == "teto_replay_index.v1"
    assert replay_index["records"][0]["scene_version"] == f"{result['run_name']}_item_001"
    assert replay_index["records"][0]["positive_replay_sample"] is True
    assert replay_index["records"][0]["hard_negative_sample"] is False


def test_robot_task_json_defaults_to_dedicated_output_root(tmp_path, monkeypatch):
    input_dir = tmp_path / "images"
    input_dir.mkdir()
    Image.new("RGB", (8, 8), (255, 255, 255)).save(input_dir / "one.jpg")

    monkeypatch.setattr(batch_recognition, "VLMInferencer", RobotTaskInferencer)

    robot_root = tmp_path / "results" / "robot_task_json"
    index_path = tmp_path / "results" / "index.jsonl"
    monkeypatch.setattr(batch_recognition, "results_index_path", lambda: index_path)
    monkeypatch.setattr(batch_recognition, "create_robot_task_json_dir", lambda output_root=None: batch_recognition.create_batch_recognition_dir(robot_root))

    result = batch_recognition.run_batch_recognition(
        input_dir,
        prompt_type="robot_task_json",
        prompt="robot prompt",
        backend="mock",
    )

    run_dir = robot_root / result["run_name"]
    assert result["output_dir"] == str(run_dir)
    assert result["run_dir"] == str(run_dir)
    assert (run_dir / "input_manifest.json").exists()
    assert (run_dir / "results.jsonl").exists()
    assert (run_dir / "summary.json").exists()
    assert (run_dir / "smoke_report.md").exists()
    assert (run_dir / "smoke_report.json").exists()
    assert (run_dir / "scene_index.json").exists()
    assert (run_dir / "replay_index.json").exists()
