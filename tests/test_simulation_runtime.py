import json

from scripts.run_first_simulation_execution import build_parser
from src.simulation_runtime import (
    CURRENT_TETO_VERSION,
    DEFAULT_CUBE_POSITION,
    DEFAULT_CUBE_PRIM_PATH,
    DEFAULT_CUBE_SIZE,
    DEFAULT_CUBE_TARGET_POSITION,
    REPORT_VERSION,
    SimulationObjectSpec,
    _execute_isaac_world,
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
    assert result["cube_spawned"] is False
    assert result["simulation_object_spawned"] is False
    assert result["simulation_object_moved"] is False
    assert result["cube_moved"] is False
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
    assert result["cube_spawned"] is False
    assert result["cube_moved"] is False
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
    assert result["cube_spawned"] is False
    assert result["cube_moved"] is False


def test_dry_run_spawn_cube_report_does_not_require_isaac():
    result = run_first_simulation_execution(VALID_TASK, dry_run=True, steps=2, spawn_cube=True)

    assert result["status"] == "PASS"
    assert result["mode"] == "dry_run"
    assert result["world_reset"] is True
    assert result["steps_completed"] == 2
    assert result["simulation_object_spawned"] is True
    assert result["object_type"] == "cube"
    assert result["cube_spawned"] is True
    assert result["cube_prim_path"] == DEFAULT_CUBE_PRIM_PATH
    assert result["cube_position"] == DEFAULT_CUBE_POSITION
    assert result["cube_size"] == DEFAULT_CUBE_SIZE
    assert result["cube_moved"] is False
    assert result["simulation_object_type"] == "cube"
    assert result["simulation_object_prim_path"] == DEFAULT_CUBE_PRIM_PATH


def test_dry_run_spawn_cube_metadata_fields_are_complete(tmp_path):
    result = run_first_simulation_execution(
        VALID_TASK,
        dry_run=True,
        steps=3,
        spawn_cube=True,
        output_dir=tmp_path,
        write_report=True,
    )

    report_path = tmp_path / "simulation_execution_result.json"
    saved = json.loads(report_path.read_text(encoding="utf-8"))
    for key in (
        "simulation_object_spawned",
        "object_type",
        "cube_prim_path",
        "cube_position",
        "cube_size",
        "cube_spawned",
    ):
        assert key in result
        assert key in saved
    assert saved["cube_spawned"] is True
    assert saved["cube_prim_path"] == DEFAULT_CUBE_PRIM_PATH
    assert saved["cube_position"] == DEFAULT_CUBE_POSITION
    assert saved["cube_size"] == DEFAULT_CUBE_SIZE


def test_cli_move_object_and_move_cube_alias_parse_to_same_flow():
    parser = build_parser()

    move_object_args = parser.parse_args(["--dry-run", "--steps", "3", "--move-object"])
    move_cube_args = parser.parse_args(["--dry-run", "--steps", "3", "--move-cube"])

    assert move_object_args.move_object is True
    assert move_object_args.move_cube is False
    assert move_cube_args.move_object is False
    assert move_cube_args.move_cube is True


def test_dry_run_move_object_report_does_not_require_isaac():
    result = run_first_simulation_execution(VALID_TASK, dry_run=True, steps=2, move_object=True)

    assert result["status"] == "PASS"
    assert result["mode"] == "dry_run"
    assert result["world_reset"] is True
    assert result["steps_completed"] == 2
    assert result["simulation_object_spawned"] is True
    assert result["simulation_object_move_requested"] is True
    assert result["simulation_object_moved"] is True
    assert result["cube_spawned"] is True
    assert result["cube_move_requested"] is True
    assert result["cube_moved"] is True
    assert result["cube_prim_path"] == DEFAULT_CUBE_PRIM_PATH
    assert result["cube_position"] == DEFAULT_CUBE_POSITION
    assert result["cube_initial_position"] == DEFAULT_CUBE_POSITION
    assert result["cube_target_position"] == DEFAULT_CUBE_TARGET_POSITION
    assert result["cube_final_position"] == DEFAULT_CUBE_TARGET_POSITION
    assert result["cube_displacement"] == [0.3, 0.0, 0.0]


def test_dry_run_move_cube_alias_still_moves_default_fixture():
    result = run_first_simulation_execution(VALID_TASK, dry_run=True, steps=2, move_cube=True)

    assert result["status"] == "PASS"
    assert result["simulation_object_spawned"] is True
    assert result["simulation_object_moved"] is True
    assert result["cube_spawned"] is True
    assert result["cube_moved"] is True
    assert result["cube_initial_position"] == DEFAULT_CUBE_POSITION
    assert result["cube_target_position"] == DEFAULT_CUBE_TARGET_POSITION
    assert result["cube_final_position"] == DEFAULT_CUBE_TARGET_POSITION


def test_dry_run_move_cube_metadata_fields_are_complete(tmp_path):
    result = run_first_simulation_execution(
        VALID_TASK,
        dry_run=True,
        steps=3,
        move_object=True,
        output_dir=tmp_path,
        write_report=True,
    )

    report_path = tmp_path / "simulation_execution_result.json"
    saved = json.loads(report_path.read_text(encoding="utf-8"))
    for key in (
        "simulation_object_spawned",
        "simulation_object_moved",
        "simulation_object_move_requested",
        "simulation_object_type",
        "simulation_object_prim_path",
        "simulation_object_initial_position",
        "simulation_object_target_position",
        "simulation_object_final_position",
        "simulation_object_displacement",
        "cube_spawned",
        "cube_move_requested",
        "cube_moved",
        "cube_initial_position",
        "cube_target_position",
        "cube_final_position",
        "cube_displacement",
    ):
        assert key in result
        assert key in saved
    assert saved["cube_spawned"] is True
    assert saved["cube_moved"] is True
    assert saved["cube_initial_position"] == DEFAULT_CUBE_POSITION
    assert saved["cube_target_position"] == DEFAULT_CUBE_TARGET_POSITION
    assert saved["cube_final_position"] == DEFAULT_CUBE_TARGET_POSITION
    assert saved["cube_displacement"] == [0.3, 0.0, 0.0]


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


def test_spawn_cube_failure_returns_fail_report_without_crashing(tmp_path):
    class FakeSimulationApp:
        closed = False

        def __init__(self, config):
            self.config = config

        def close(self):
            self.closed = True

    class FakeWorld:
        def __init__(self):
            self.reset_called = False
            self.step_calls = 0

        def reset(self):
            self.reset_called = True

        def step(self, render=False):
            self.step_calls += 1

    def failing_spawner(world, *, object_spec):
        raise RuntimeError("cube spawn failed")

    result = _execute_isaac_world(
        simulation_task=VALID_TASK,
        simulation_app_cls=FakeSimulationApp,
        world_cls=FakeWorld,
        steps=3,
        headless=True,
        spawn_object=True,
        move_object=False,
        object_spec=SimulationObjectSpec(
            object_type="cube",
            prim_path=DEFAULT_CUBE_PRIM_PATH,
            initial_position=tuple(DEFAULT_CUBE_POSITION),
            target_position=tuple(DEFAULT_CUBE_TARGET_POSITION),
            size=DEFAULT_CUBE_SIZE,
        ),
        started_at="2026-05-31 00:00:00",
        output_dir=tmp_path,
        write_report=True,
        object_spawner=failing_spawner,
    )

    assert result["status"] == "FAIL"
    assert result["ok"] is False
    assert result["mode"] == "isaac"
    assert result["isaac_runtime_used"] is True
    assert result["world_reset"] is True
    assert result["steps_completed"] == 0
    assert result["simulation_object_spawned"] is False
    assert result["object_type"] == "cube"
    assert result["cube_spawned"] is False
    assert result["cube_prim_path"] == DEFAULT_CUBE_PRIM_PATH
    assert result["cube_position"] == DEFAULT_CUBE_POSITION
    assert result["cube_size"] == DEFAULT_CUBE_SIZE
    assert result["cube_moved"] is False
    assert result["allow_robot_motion"] is False
    assert result["error"]["code"] == "E_CUBE_SPAWN_FAILED"
    assert "cube spawn failed" in result["error"]["message"]
    assert (tmp_path / "simulation_execution_result.json").exists()


def test_move_cube_failure_returns_fail_report_without_crashing(tmp_path):
    class FakeSimulationApp:
        def __init__(self, config):
            self.config = config

        def close(self):
            self.closed = True

    class FakeWorld:
        def reset(self):
            self.reset_called = True

        def step(self, render=False):
            self.step_called = True

    class FakeSimulationObject:
        pass

    object_spec = SimulationObjectSpec(
        object_type="cube",
        prim_path=DEFAULT_CUBE_PRIM_PATH,
        initial_position=tuple(DEFAULT_CUBE_POSITION),
        target_position=tuple(DEFAULT_CUBE_TARGET_POSITION),
        size=DEFAULT_CUBE_SIZE,
    )

    def successful_spawner(world, *, object_spec):
        return FakeSimulationObject(), {
            "simulation_object_spawned": True,
            "simulation_object_moved": False,
            "simulation_object_move_requested": False,
            "simulation_object_type": "cube",
            "simulation_object_prim_path": DEFAULT_CUBE_PRIM_PATH,
            "simulation_object_initial_position": DEFAULT_CUBE_POSITION,
            "simulation_object_target_position": None,
            "simulation_object_final_position": None,
            "simulation_object_displacement": None,
            "simulation_object_size": DEFAULT_CUBE_SIZE,
            "object_type": "cube",
            "cube_prim_path": DEFAULT_CUBE_PRIM_PATH,
            "cube_position": DEFAULT_CUBE_POSITION,
            "cube_size": DEFAULT_CUBE_SIZE,
            "cube_spawned": True,
            "cube_move_requested": False,
            "cube_moved": False,
            "cube_initial_position": DEFAULT_CUBE_POSITION,
            "cube_target_position": None,
            "cube_final_position": None,
            "cube_displacement": None,
        }

    def failing_pose_updater(simulation_object, *, object_spec, current_metadata):
        raise RuntimeError("pose update failed")

    result = _execute_isaac_world(
        simulation_task=VALID_TASK,
        simulation_app_cls=FakeSimulationApp,
        world_cls=FakeWorld,
        steps=3,
        headless=True,
        spawn_object=True,
        move_object=True,
        object_spec=object_spec,
        started_at="2026-05-31 00:00:00",
        output_dir=tmp_path,
        write_report=True,
        object_spawner=successful_spawner,
        object_pose_updater=failing_pose_updater,
    )

    assert result["status"] == "FAIL"
    assert result["ok"] is False
    assert result["mode"] == "isaac"
    assert result["world_reset"] is True
    assert result["steps_completed"] == 0
    assert result["simulation_object_spawned"] is True
    assert result["simulation_object_move_requested"] is True
    assert result["simulation_object_moved"] is False
    assert result["cube_spawned"] is True
    assert result["cube_move_requested"] is True
    assert result["cube_moved"] is False
    assert result["cube_initial_position"] == DEFAULT_CUBE_POSITION
    assert result["cube_target_position"] == DEFAULT_CUBE_TARGET_POSITION
    assert result["cube_final_position"] == DEFAULT_CUBE_POSITION
    assert result["cube_displacement"] == [0.0, 0.0, 0.0]
    assert result["error"]["code"] == "E_SIM_OBJECT_MOVE_FAILED"
    assert "pose update failed" in result["error"]["message"]
    assert (tmp_path / "simulation_execution_result.json").exists()
