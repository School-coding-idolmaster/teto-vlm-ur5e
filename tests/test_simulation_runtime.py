import json

from scripts.harnesses.run_shadow_simulation_contract import build_parser
from src.simulation_runtime import (
    CURRENT_TETO_VERSION,
    DEFAULT_CUBE_POSITION,
    DEFAULT_CUBE_PRIM_PATH,
    DEFAULT_CUBE_SIZE,
    DEFAULT_CUBE_TARGET_POSITION,
    DEFAULT_ROBOT_PRIM_PATH,
    DEFAULT_ROBOT_TYPE,
    REPORT_VERSION,
    RobotAssetSpec,
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
    assert result["robot_asset_check_requested"] is False
    assert result["robot_asset_load_requested"] is False
    assert result["robot_asset_available"] is False
    assert result["robot_asset_loaded"] is False
    assert result["robot_prim_inspection_requested"] is False
    assert result["robot_prim_inspection"]["inspection_status"] == "NOT_REQUESTED"
    assert result["robot_structure_report_generated"] is False
    assert result["robot_structure_report_path"] is None
    assert result["articulation_readiness_requested"] is False
    assert result["articulation_readiness"]["readiness_status"] == "NOT_REQUESTED"
    assert result["articulation_readiness"]["control_enabled"] is False
    assert result["articulation_readiness"]["motion_generated"] is False
    assert result["articulation_readiness"]["command_generated"] is False
    assert result["simulation_motion_precheck_requested"] is False
    assert result["simulation_motion_precheck_status"] == "NOT_REQUESTED"
    assert result["ready_for_simulation_motion"] is False
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


def test_cli_robot_asset_arguments_parse():
    parser = build_parser()

    args = parser.parse_args(
        [
            "--dry-run",
            "--steps",
            "1",
            "--check-robot-asset",
            "--inspect-robot-prim",
            "--check-articulation-readiness",
            "--robot-type",
            "ur5",
            "--robot-prim-path",
            "/World/TestRobot",
            "--robot-asset-path",
            "/tmp/missing.usd",
        ]
    )

    assert args.check_robot_asset is True
    assert args.inspect_robot_prim is True
    assert args.check_articulation_readiness is True
    assert args.load_robot_asset is False
    assert args.robot_type == "ur5"
    assert args.robot_prim_path == "/World/TestRobot"
    assert args.robot_asset_path == "/tmp/missing.usd"


def test_cli_articulation_state_argument_parse():
    parser = build_parser()

    args = parser.parse_args(["--dry-run", "--steps", "1", "--observe-articulation-state"])

    assert args.observe_articulation_state is True


def test_cli_simulation_motion_precheck_argument_parse():
    parser = build_parser()
    args = parser.parse_args(["--check-simulation-motion-precheck"])

    assert args.check_simulation_motion_precheck is True


def test_cli_simulation_micro_motion_arguments_parse():
    parser = build_parser()
    args = parser.parse_args(
        [
            "--execute-simulation-micro-motion",
            "--micro-motion-joint",
            "wrist_3_joint",
            "--micro-motion-delta-rad",
            "0.01",
            "--micro-motion-tolerance-rad",
            "0.005",
        ]
    )

    assert args.execute_simulation_micro_motion is True
    assert args.micro_motion_joint == "wrist_3_joint"
    assert args.micro_motion_delta_rad == 0.01
    assert args.micro_motion_tolerance_rad == 0.005


def test_cli_semantic_bridge_arguments_parse():
    parser = build_parser()
    args = parser.parse_args(
        [
            "--semantic-simulation-bridge",
            "--semantic-task-json",
            "tests/fixtures/semantic_contracts/eligible_hover_to_object.json",
            "--semantic-confidence-threshold",
            "0.7",
        ]
    )

    assert args.semantic_simulation_bridge is True
    assert args.semantic_task_json == "tests/fixtures/semantic_contracts/eligible_hover_to_object.json"
    assert args.semantic_confidence_threshold == 0.7


def test_cli_safe_simulated_task_execution_arguments_parse():
    parser = build_parser()
    args = parser.parse_args(
        [
            "--safe-simulated-task-execution",
            "--semantic-bridge-demo-contract",
            "--execution-attempt-id",
            "attempt_001",
            "--execution-enable-retry-recommendation",
            "--execution-enable-fallback-recommendation",
        ]
    )

    assert args.safe_simulated_task_execution is True
    assert args.semantic_bridge_demo_contract is True
    assert args.execution_attempt_id == "attempt_001"
    assert args.execution_max_attempts == 1
    assert args.execution_enable_retry_recommendation is True
    assert args.execution_enable_fallback_recommendation is True


def test_cli_lab_readiness_arguments_parse():
    parser = build_parser()
    args = parser.parse_args(
        [
            "--check-lab-readiness",
            "--lab-readiness-config",
            "configs/lab_backend.example.yaml",
            "--check-camera-readiness",
            "--check-live-vlm-readiness",
            "--check-shadow-mode-readiness",
        ]
    )

    assert args.check_lab_readiness is True
    assert args.lab_readiness_config == "configs/lab_backend.example.yaml"
    assert args.check_camera_readiness is True
    assert args.check_live_vlm_readiness is True
    assert args.check_shadow_mode_readiness is True


def test_cli_camera_snapshot_arguments_parse():
    parser = build_parser()
    args = parser.parse_args(
        [
            "--check-camera-snapshot",
            "--camera-snapshot-config",
            "configs/camera_snapshot.example.yaml",
            "--camera-snapshot-report",
        ]
    )

    assert args.check_camera_snapshot is True
    assert args.camera_snapshot_config == "configs/camera_snapshot.example.yaml"
    assert args.camera_snapshot_report is True


def test_dry_run_robot_asset_check_passes_with_unavailable_default():
    result = run_first_simulation_execution(VALID_TASK, dry_run=True, steps=1, check_robot_asset=True)

    assert result["status"] == "PASS"
    assert result["ok"] is True
    assert result["error"]["code"] == "OK"
    assert result["world_reset"] is True
    assert result["steps_completed"] == 1
    assert result["robot_asset_check_requested"] is True
    assert result["robot_asset_load_requested"] is False
    assert result["robot_type"] == DEFAULT_ROBOT_TYPE
    assert result["robot_prim_path"] == DEFAULT_ROBOT_PRIM_PATH
    assert result["robot_asset_path"] is None
    assert result["robot_asset_source"] == "dry_run"
    assert result["robot_asset_available"] is False
    assert result["robot_asset_loaded"] is False
    assert result["robot_prim_exists"] is False
    assert result["robot_asset_status"] == "UNAVAILABLE"
    assert result["robot_asset_blocking_reason"] == "E_ROBOT_ASSET_UNAVAILABLE"
    assert result["blocking_reasons"] == []


def test_dry_run_robot_prim_inspection_returns_not_found_diagnostic():
    result = run_first_simulation_execution(VALID_TASK, dry_run=True, steps=1, inspect_robot_prim=True)
    inspection = result["robot_prim_inspection"]

    assert result["status"] == "PASS"
    assert result["error"]["code"] == "OK"
    assert result["robot_prim_inspection_requested"] is True
    assert inspection["requested"] is True
    assert inspection["robot_prim_path"] == DEFAULT_ROBOT_PRIM_PATH
    assert inspection["robot_prim_exists"] is False
    assert inspection["inspection_status"] == "E_ROBOT_PRIM_NOT_FOUND"
    assert inspection["joint_metadata_summary"]["metadata_only"] is True
    assert inspection["joint_metadata_summary"]["control_ready"] is False
    assert inspection["joint_metadata_summary"]["control_targets_generated"] is False
    assert inspection["joint_metadata_table"] == []
    assert result["robot_structure_report_generated"] is False
    assert result["robot_structure_report_path"] is None
    for key in (
        "total_descendant_prim_count",
        "link_like_prim_count",
        "joint_like_prim_count",
        "visual_like_prim_count",
        "collision_like_prim_count",
        "possible_dof_count",
    ):
        assert isinstance(inspection[key], int)
        assert inspection[key] >= 0


def test_dry_run_articulation_readiness_returns_not_ready_diagnostic():
    result = run_first_simulation_execution(
        VALID_TASK,
        dry_run=True,
        steps=1,
        inspect_robot_prim=True,
        check_articulation_readiness=True,
    )
    readiness = result["articulation_readiness"]

    assert result["status"] == "PASS"
    assert result["error"]["code"] == "OK"
    assert result["articulation_readiness_requested"] is True
    assert readiness["requested"] is True
    assert readiness["readiness_status"] == "NOT_READY"
    assert readiness["articulation_ready"] is False
    assert readiness["control_enabled"] is False
    assert readiness["motion_generated"] is False
    assert readiness["command_generated"] is False
    assert "robot_prim" in readiness["missing_requirements"]
    assert "articulation_root" in readiness["missing_requirements"]
    assert "six_standard_ur5e_arm_joints" in readiness["missing_requirements"]
    assert readiness["safety_boundary"]["read_only"] is True


def test_dry_run_articulation_state_observation_returns_not_observable():
    result = run_first_simulation_execution(
        VALID_TASK,
        dry_run=True,
        steps=1,
        observe_articulation_state=True,
    )
    state = result["articulation_state"]

    assert result["status"] == "PASS"
    assert result["error"]["code"] == "OK"
    assert result["articulation_state_observation_requested"] is True
    assert result["articulation_state_observable"] is False
    assert result["articulation_state_status"] == "NOT_OBSERVABLE"
    assert result["control_enabled"] is False
    assert result["motion_generated"] is False
    assert result["command_generated"] is False
    assert result["joint_targets_generated"] is False
    assert result["observed_joint_count"] == 0
    assert result["joint_limits_available"] is False
    assert state["status"] == "NOT_OBSERVABLE"
    assert state["control_enabled"] is False
    assert state["motion_generated"] is False
    assert state["command_generated"] is False
    assert state["joint_targets_generated"] is False
    assert state["joint_state_table"] == []


def test_dry_run_simulation_motion_precheck_returns_not_ready():
    result = run_first_simulation_execution(
        VALID_TASK,
        dry_run=True,
        steps=1,
        check_simulation_motion_precheck=True,
    )
    precheck = result["simulation_motion_precheck"]

    assert result["status"] == "PASS"
    assert result["error"]["code"] == "OK"
    assert result["simulation_motion_precheck_requested"] is True
    assert result["simulation_motion_precheck_status"] == "NOT_READY"
    assert result["ready_for_simulation_motion"] is False
    assert result["control_enabled"] is False
    assert result["motion_generated"] is False
    assert result["command_generated"] is False
    assert result["joint_targets_generated"] is False
    assert result["trajectory_generated"] is False
    assert result["tcp_pose_world_generated"] is False
    assert result["robot_motion_executed"] is False
    assert "robot_prim_exists" in precheck["missing_requirements"]
    assert "articulation_readiness_ready" in precheck["missing_requirements"]
    assert "articulation_state_ok" in precheck["missing_requirements"]


def test_inspect_robot_prim_does_not_add_motion_control_fields():
    result = run_first_simulation_execution(VALID_TASK, dry_run=True, steps=1, inspect_robot_prim=True)
    serialized_keys = " ".join(result.keys()) + " " + " ".join(result["robot_prim_inspection"].keys())

    for forbidden in (
        "joint_target",
        "joint_angles",
        "tcp_pose_world",
        "actual_TCP_pose",
        "URScript",
        "RTDE",
        "MoveIt",
        "ROS2",
    ):
        assert forbidden not in serialized_keys


def test_explicit_invalid_robot_asset_load_fails_without_isaac():
    result = run_first_simulation_execution(
        VALID_TASK,
        dry_run=True,
        steps=1,
        load_robot_asset=True,
        robot_asset_path="/tmp/teto_missing_robot_asset.usd",
    )

    assert result["status"] == "FAIL"
    assert result["ok"] is False
    assert result["error"]["code"] == "E_ROBOT_ASSET_LOAD_FAILED"
    assert result["robot_asset_check_requested"] is True
    assert result["robot_asset_load_requested"] is True
    assert result["robot_asset_available"] is False
    assert result["robot_asset_loaded"] is False
    assert result["robot_prim_exists"] is False
    assert result["robot_asset_status"] == "LOAD_FAILED"
    assert result["robot_asset_blocking_reason"] == "E_ROBOT_ASSET_UNAVAILABLE"


def test_robot_asset_report_fields_are_complete(tmp_path):
    result = run_first_simulation_execution(
        VALID_TASK,
        dry_run=True,
        steps=1,
        check_robot_asset=True,
        output_dir=tmp_path,
        write_report=True,
    )

    saved = json.loads((tmp_path / "simulation_execution_result.json").read_text(encoding="utf-8"))
    for key in (
        "robot_asset_check_requested",
        "robot_asset_load_requested",
        "robot_type",
        "robot_prim_path",
        "robot_asset_path",
        "robot_asset_source",
        "robot_asset_available",
        "robot_asset_loaded",
        "robot_prim_exists",
        "robot_asset_status",
        "robot_asset_blocking_reason",
    ):
        assert key in result
        assert key in saved
    assert saved["robot_asset_available"] is False
    assert saved["robot_asset_loaded"] is False
    assert saved["robot_asset_status"] == "UNAVAILABLE"


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


def test_true_runtime_robot_asset_check_unavailable_passes_without_loading(tmp_path):
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

    def unexpected_loader(world, *, robot_asset_spec):
        raise AssertionError("check mode without a local asset must not load")

    result = _execute_isaac_world(
        simulation_task=VALID_TASK,
        simulation_app_cls=FakeSimulationApp,
        world_cls=FakeWorld,
        steps=1,
        headless=True,
        spawn_object=False,
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
        check_robot_asset=True,
        robot_asset_spec=RobotAssetSpec(),
        robot_asset_loader=unexpected_loader,
    )

    assert result["status"] == "PASS"
    assert result["error"]["code"] == "OK"
    assert result["world_reset"] is True
    assert result["steps_completed"] == 1
    assert result["robot_asset_available"] is False
    assert result["robot_asset_loaded"] is False
    assert result["robot_asset_status"] == "UNAVAILABLE"
    assert result["robot_asset_blocking_reason"] == "E_ROBOT_ASSET_UNAVAILABLE"


def test_true_like_articulation_state_observation_report(tmp_path):
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

    def successful_loader(world, *, robot_asset_spec):
        world.loaded = robot_asset_spec.robot_asset_path

    def prim_exists(world, *, robot_asset_spec):
        return True

    def prim_inspector(world, *, robot_prim_path):
        return {
            "requested": True,
            "robot_prim_path": robot_prim_path,
            "robot_prim_exists": True,
            "robot_root_type_name": "Xform",
            "total_descendant_prim_count": 8,
            "link_like_prim_count": 6,
            "joint_like_prim_count": 6,
            "visual_like_prim_count": 6,
            "collision_like_prim_count": 6,
            "articulation_root_found": True,
            "physics_schema_summary": [],
            "joint_names": [],
            "joint_prim_paths": [],
            "possible_dof_names": [],
            "possible_dof_count": 6,
            "joint_metadata_summary": {
                "arm_joint_count": 6,
                "arm_joint_names": [
                    "shoulder_pan_joint",
                    "shoulder_lift_joint",
                    "elbow_joint",
                    "wrist_1_joint",
                    "wrist_2_joint",
                    "wrist_3_joint",
                ],
            },
            "joint_metadata_table": [],
            "inspection_status": "OK",
            "inspection_warnings": [],
        }

    def state_observer(world, *, robot_prim_path, robot_prim_inspection, articulation_readiness):
        return {
            "requested": True,
            "status": "OK",
            "metadata_only": True,
            "control_enabled": False,
            "motion_generated": False,
            "command_generated": False,
            "joint_targets_generated": False,
            "robot_prim_path": robot_prim_path,
            "articulation_state_observable": True,
            "arm_joint_count": 6,
            "observed_joint_count": 6,
            "expected_arm_joint_names": [
                "shoulder_pan_joint",
                "shoulder_lift_joint",
                "elbow_joint",
                "wrist_1_joint",
                "wrist_2_joint",
                "wrist_3_joint",
            ],
            "observed_arm_joint_names": [
                "shoulder_pan_joint",
                "shoulder_lift_joint",
                "elbow_joint",
                "wrist_1_joint",
                "wrist_2_joint",
                "wrist_3_joint",
            ],
            "missing_arm_joint_names": [],
            "extra_joint_names": [],
            "joint_positions_available": True,
            "joint_velocities_available": True,
            "joint_limits_available": True,
            "joint_state_table": [
                {
                    "joint_name": "shoulder_pan_joint",
                    "category": "arm",
                    "position": 0.0,
                    "velocity": 0.0,
                    "lower_limit": -3.14,
                    "upper_limit": 3.14,
                    "limit_available": True,
                    "within_limit": True,
                    "metadata_only": True,
                    "control_target_generated": False,
                }
            ],
            "warnings": [],
            "errors": [],
            "safety_boundary": {
                "read_only": True,
                "no_robot_motion": True,
                "no_joint_targets": True,
                "no_tcp_pose_world": True,
                "no_trajectory": True,
                "no_ros2_moveit_rtde_urscript": True,
            },
        }

    asset_path = tmp_path / "robot.usd"
    asset_path.write_text("#usda 1.0\n", encoding="utf-8")
    result = _execute_isaac_world(
        simulation_task=VALID_TASK,
        simulation_app_cls=FakeSimulationApp,
        world_cls=FakeWorld,
        steps=1,
        headless=True,
        spawn_object=False,
        move_object=False,
        object_spec=SimulationObjectSpec(
            object_type="cube",
            prim_path=DEFAULT_CUBE_PRIM_PATH,
            initial_position=tuple(DEFAULT_CUBE_POSITION),
            target_position=tuple(DEFAULT_CUBE_TARGET_POSITION),
            size=DEFAULT_CUBE_SIZE,
        ),
        started_at="2026-06-01 00:00:00",
        output_dir=tmp_path,
        write_report=True,
        check_robot_asset=True,
        load_robot_asset=True,
        robot_asset_spec=RobotAssetSpec(robot_asset_path=str(asset_path)),
        inspect_robot_prim=True,
        check_articulation_readiness=True,
        observe_articulation_state=True,
        check_simulation_motion_precheck=True,
        robot_asset_loader=successful_loader,
        robot_prim_verifier=prim_exists,
        robot_prim_inspector=prim_inspector,
        articulation_state_observer=state_observer,
    )

    assert result["status"] == "PASS"
    assert result["robot_asset_loaded"] is True
    assert result["robot_prim_exists"] is True
    assert result["articulation_readiness"]["readiness_status"] == "READY"
    assert result["articulation_state_observation_requested"] is True
    assert result["articulation_state_observable"] is True
    assert result["articulation_state_status"] == "OK"
    assert result["control_enabled"] is False
    assert result["motion_generated"] is False
    assert result["command_generated"] is False
    assert result["joint_targets_generated"] is False
    assert result["arm_joint_count"] == 6
    assert result["observed_joint_count"] == 6
    assert result["missing_arm_joint_names"] == []
    assert result["joint_limits_available"] is True
    assert result["simulation_motion_precheck_requested"] is True
    assert result["simulation_motion_precheck_status"] == "READY_FOR_SIMULATION_MOTION"
    assert result["ready_for_simulation_motion"] is True
    assert result["trajectory_generated"] is False
    assert result["tcp_pose_world_generated"] is False
    assert result["robot_motion_executed"] is False
    assert result["articulation_state"]["control_enabled"] is False
    assert result["articulation_state"]["motion_generated"] is False
    assert result["articulation_state"]["command_generated"] is False
    assert result["articulation_state"]["joint_targets_generated"] is False
