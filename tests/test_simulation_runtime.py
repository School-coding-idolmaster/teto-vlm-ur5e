import json

from src.simulation_runtime import (
    CURRENT_TETO_VERSION,
    REPORT_VERSION,
    build_simulation_execution_result,
    run_first_simulation_execution,
    write_simulation_execution_result,
)


VALID_TASK = {
    "task_type": "hover_to_object",
    "target_label": "camera",
    "target_world_point": [0.2, 0.1, 0.4],
    "scene_version": "run_simulation_item_001",
    "ttl_ms": 500,
}


def test_build_success_report_fields():
    result = build_simulation_execution_result(
        simulation_task=VALID_TASK,
        status="PASS",
        mode="no_isaac",
        steps_requested=3,
        steps_completed=3,
        world_reset=True,
    )

    assert result["report_version"] == REPORT_VERSION
    assert result["teto_version"] == CURRENT_TETO_VERSION
    assert result["status"] == "PASS"
    assert result["ok"] is True
    assert result["allow_robot_motion"] is False
    assert result["consumed_simulation_task"] is True
    assert result["world_reset"] is True
    assert result["steps_completed"] == 3
    assert result["blocking_reasons"] == []
    assert result["error"]["code"] == "OK"


def test_build_failure_report_fields():
    result = build_simulation_execution_result(
        simulation_task=VALID_TASK,
        status="FAIL",
        mode="isaac",
        steps_requested=5,
        error_code="E_ISAAC_RUNTIME_FAILED",
        error_message="missing isaac runtime",
    )

    assert result["ok"] is False
    assert result["isaac_runtime_used"] is True
    assert result["world_reset"] is False
    assert result["steps_completed"] == 0
    assert result["blocking_reasons"] == ["E_ISAAC_RUNTIME_FAILED"]
    assert result["error"]["message"] == "missing isaac runtime"


def test_dry_run_execution_does_not_require_isaac():
    result = run_first_simulation_execution(VALID_TASK, dry_run=True, steps=2)

    assert result["status"] == "PASS"
    assert result["mode"] == "dry_run"
    assert result["dry_run"] is True
    assert result["isaac_runtime_used"] is False
    assert result["world_reset"] is True
    assert result["steps_completed"] == 2


def test_no_isaac_execution_mode_does_not_require_isaac():
    result = run_first_simulation_execution(VALID_TASK, no_isaac=True, steps=4)

    assert result["status"] == "PASS"
    assert result["mode"] == "no_isaac"
    assert result["dry_run"] is False
    assert result["isaac_runtime_used"] is False
    assert result["steps_completed"] == 4


def test_invalid_task_returns_failure_report():
    result = run_first_simulation_execution({"task_type": "hover_to_object"}, dry_run=True)

    assert result["status"] == "FAIL"
    assert result["ok"] is False
    assert result["error"]["code"] == "E_INVALID_SIMULATION_TASK"
    assert "target_world_point" in result["error"]["message"]
    assert result["allow_robot_motion"] is False


def test_write_simulation_execution_result_adds_report_path(tmp_path):
    result = build_simulation_execution_result(
        simulation_task=VALID_TASK,
        status="PASS",
        mode="dry_run",
        steps_requested=1,
        steps_completed=1,
        world_reset=True,
    )

    report_path = write_simulation_execution_result(result, tmp_path)

    assert report_path == tmp_path / "simulation_execution_result.json"
    assert result["report_path"] == str(report_path)
    saved = json.loads(report_path.read_text(encoding="utf-8"))
    assert saved["status"] == "PASS"
    assert saved["report_path"] == str(report_path)


def test_dry_run_can_write_report_from_runtime(tmp_path):
    result = run_first_simulation_execution(
        VALID_TASK,
        dry_run=True,
        steps=3,
        output_dir=tmp_path,
        write_report=True,
    )

    report_path = tmp_path / "simulation_execution_result.json"
    assert result["status"] == "PASS"
    assert result["report_path"] == str(report_path)
    assert report_path.exists()
    saved = json.loads(report_path.read_text(encoding="utf-8"))
    assert saved["steps_completed"] == 3
    assert saved["allow_robot_motion"] is False
