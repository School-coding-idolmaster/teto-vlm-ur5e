import json
import importlib.util
import os
import subprocess
import threading
import time
from pathlib import Path
from types import SimpleNamespace

import pytest

from scripts import text_to_ur5e_real_motion as real_motion_cli
from src.isaac_sim_operator import (
    GATEWAY_SYNTHETIC_FAKE,
    IsaacOperatorConfig,
    IsaacOperatorSafetyError,
    IsaacSimOperator,
    SyntheticFakeGateway,
    _joint_delta_summary,
    load_isaac_operator_config,
    validate_no_real_robot_args,
    validate_no_real_robot_config,
)
from src.isaac_sim_bridge import (
    DEFAULT_ISAAC_HOME_POSE_RAD,
    _missing_local_usd_dependencies,
    _named_joint_target,
    _require_articulation,
    _visual_timing,
)
from src.qwen_motion_parser import QwenMotionParserRequest, evaluate_qwen_motion_parser

CONSOLE_SCRIPT = Path("scripts/teto_isaac_operator_console.py")


VECTOR_QWEN_RESPONSE = json.dumps(
    {
        "schema_version": "teto_motion_semantics.v1",
        "intent_status": "ok",
        "intent_type": "relative_cartesian_motion",
        "motion": {
            "reference": "tcp",
            "mode": "vector_delta",
            "direction_semantic": "unknown",
            "delta": {
                "x": {"value": 30, "unit": "cm", "meters": 0.30, "quality": "explicit"},
                "y": {"value": 10, "unit": "cm", "meters": 0.10, "quality": "explicit"},
                "z": {"value": 0, "unit": "m", "meters": 0.0, "quality": "explicit"},
            },
            "distance": {"value": None, "unit": "unspecified", "meters": None, "quality": "explicit"},
            "fuzzy_magnitude": "unspecified",
            "frame_hint": "base_link",
        },
        "clarification": {"required": False, "reason": ""},
        "unsupported": {"reason": ""},
        "confidence": {"intent": 0.99, "direction": 0.99, "distance": 0.99, "overall": 0.99},
        "language": "en",
        "notes": "explicit vector",
    }
)


def _single_axis_qwen_response(distance_m: float) -> str:
    return json.dumps(
        {
            "schema_version": "teto_motion_semantics.v1",
            "intent_status": "ok",
            "intent_type": "relative_cartesian_motion",
            "motion": {
                "reference": "tcp",
                "mode": "single_axis",
                "direction_semantic": "up",
                "delta": {
                    "z": {
                        "value": distance_m,
                        "unit": "m",
                        "meters": distance_m,
                        "quality": "explicit",
                    }
                },
                "distance": None,
                "frame_hint": "base_link",
            },
            "clarification": {"required": False, "reason": ""},
            "unsupported": {"reason": ""},
            "confidence": {
                "intent": 0.99,
                "direction": 0.99,
                "distance": 0.0,
                "overall": 0.99,
            },
            "language": "en",
        }
    )


def _load_console_module():
    spec = importlib.util.spec_from_file_location("teto_isaac_operator_console_test", CONSOLE_SCRIPT)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def _load_importer_module():
    path = Path("scripts/import_ur5e_urdf_to_isaac_usd.py")
    spec = importlib.util.spec_from_file_location("import_ur5e_urdf_to_isaac_usd_test", path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def _config() -> IsaacOperatorConfig:
    return IsaacOperatorConfig(
        raw={
            "gui_required": True,
            "real_robot_disabled": True,
            "max_substep_distance_m": 0.02,
            "max_total_distance_m": 0.35,
            "position_tolerance_m": 0.001,
            "qwen_endpoint": "http://127.0.0.1:18080/api/generate",
            "workspace_envelope": {"x": [-1, 1], "y": [-1, 1], "z": [0, 2]},
        }
    )


def test_example_config_loads_and_is_fail_closed():
    config = load_isaac_operator_config("configs/isaac_sim_operator.example.yaml")
    assert config.raw["gui_required"] is True
    assert config.raw["real_robot_disabled"] is True
    assert config.raw["allow_moveit_execute"] is False
    assert config.raw["apply_initial_home_pose"] is True
    assert config.raw["visual_demo_slowdown_enabled"] is True
    assert config.raw["visual_markers_enabled"] is True
    assert config.raw["scene_monitor_type"] == "none"
    assert config.raw["scene_monitor_frequency_hz"] == 5.0
    assert config.raw["camera_monitor_unavailable_policy"] == "warn_only"


def test_real_flag_is_refused():
    with pytest.raises(IsaacOperatorSafetyError, match="E_REAL_FLAG_FORBIDDEN"):
        validate_no_real_robot_args(["--real"])


def test_isaac_visual_options_do_not_appear_in_real_motion_script():
    real_script = Path("scripts/text_to_ur5e_real_motion.py").read_text(encoding="utf-8")
    for option in (
        "--motion-duration-sec",
        "--substep-pause-sec",
        "--no-visual-demo-slowdown",
        "--no-visual-markers",
        "visual_demo_slowdown_enabled",
        "isaac_initial_home_pose",
        "scene_monitor_frequency_hz",
        "camera_monitor_unavailable_policy",
        "scene_monitor_callable",
    ):
        assert option not in real_script


def test_real_ur_ip_is_refused():
    with pytest.raises(IsaacOperatorSafetyError, match="E_REAL_UR_IP_FORBIDDEN"):
        validate_no_real_robot_config(
            {
                "gui_required": True,
                "real_robot_disabled": True,
                "robot_ip": "192.168.20.35",
            }
        )


def test_documentation_only_ip_does_not_enable_connection():
    validate_no_real_robot_config(
        {
            "gui_required": True,
            "real_robot_disabled": True,
            "documentation_note": "192.168.20.35",
        }
    )


def test_vector_command_decomposes_executes_and_writes_simulation_evidence(tmp_path):
    operator = IsaacSimOperator(
        config=_config(),
        gateway=SyntheticFakeGateway(),
        headless=True,
        output_dir=tmp_path,
        qwen_callable=lambda _prompt: VECTOR_QWEN_RESPONSE,
    )
    result = operator.execute_text("move forward 30 cm and left 10 cm")

    assert result["status"] == "PASS"
    assert result["gateway_type"] == GATEWAY_SYNTHETIC_FAKE
    assert result["substep_count"] == 16
    assert result["completed_substep_count"] == 16
    assert result["execution_mode"] == "isaac_sim"
    assert result["runtime_mode"] == "headless_smoke_test"
    assert result["isaac_gui_required"] is True
    assert result["simulated_robot_motion_executed"] is True
    assert Path(result["artifact_paths"]["json"]).is_file()
    assert Path(result["artifact_paths"]["markdown"]).is_file()
    assert all(step["verification_result"] == "PASS" for step in result["substeps"])
    assert result["real_robot_motion_executed"] is False
    assert result["real_ur_connection_used"] is False
    assert result["dashboard_used"] is False
    assert result["rtde_write_used"] is False
    assert result["moveit_execute_trajectory_called"] is False
    assert result["trajectory_sent"] is False
    assert result["isaac_visual_timing"]["scope"] == "isaac_sim_only"


@pytest.mark.parametrize(
    "command",
    ["move up 5 cm", "move up 0.05 meters", "raise the tcp by 0.05 meters"],
)
def test_single_axis_explicit_delta_executes_only_in_isaac_sim(command, tmp_path):
    operator = IsaacSimOperator(
        config=_config(),
        gateway=SyntheticFakeGateway(),
        headless=True,
        output_dir=tmp_path,
        qwen_callable=lambda _prompt: _single_axis_qwen_response(0.05),
    )

    result = operator.execute_text(command)

    assert result["status"] == "PASS"
    assert result["delta_vector_m"] == [0.0, 0.0, 0.05]
    assert result["simulated_robot_motion_executed"] is True
    assert result["real_robot_motion_executed"] is False
    assert result["dashboard_used"] is False
    assert result["rtde_write_used"] is False
    assert result["moveit_execute_trajectory_called"] is False
    assert result["trajectory_sent"] is False
    assert result["target_final_tcp_pose"]["position_m"] == [0.4, 0.0, 0.45]
    assert result["measured_delta_vector_m"] == [0.0, 0.0, 0.05]
    assert result["final_position_error_m"] == 0.0
    assert result["direction_check_passed"] is True
    assert result["isaac_visual_timing"]["scope"] == "isaac_sim_only"


def test_substeps_use_low_cost_monitor_without_recalling_qwen_or_vlm(tmp_path):
    calls = {"qwen": 0, "monitor": 0}

    def qwen(_prompt):
        calls["qwen"] += 1
        return _single_axis_qwen_response(0.05)

    def monitor(context):
        calls["monitor"] += 1
        return {
            "monitor_type": "mock",
            "camera_check_status": "PASS",
            "target_visible": None,
            "depth_valid": True,
            "tf_fresh": True,
            "scene_snapshot_id": f"mock-{context['substep_index']}",
            "scene_freshness_status": "fresh",
        }

    result = IsaacSimOperator(
        config=_config(),
        gateway=SyntheticFakeGateway(),
        headless=True,
        output_dir=tmp_path,
        qwen_callable=qwen,
        scene_monitor_callable=monitor,
    ).execute_text("move up 0.05 meters")

    assert result["status"] == "PASS"
    assert calls == {"qwen": 1, "monitor": 3}
    assert result["completed_substep_count"] == 3
    assert result["reobserve_triggered"] is False
    assert result["vlm_reobserve_called"] is False
    assert result["llm_reobserve_called"] is False
    assert result["working_memory_before"]["remaining_delta_m"] == [0.0, 0.0, 0.05]
    assert result["working_memory_after"]["remaining_delta_m"] == [0.0, 0.0, 0.0]
    assert result["working_memory_after"]["completed_substeps"] == 3
    assert result["working_memory_after"]["scene_snapshot_id"] == "mock-3"
    assert all(step["camera_check_status"] == "PASS" for step in result["substeps"])
    assert all(step["vlm_reobserve_called"] is False for step in result["substeps"])
    assert all(step["llm_reobserve_called"] is False for step in result["substeps"])
    assert result["real_robot_motion_executed"] is False
    assert result["trajectory_sent"] is False


def test_scene_stale_stops_after_substep_and_requests_reobservation(tmp_path):
    calls = {"qwen": 0, "monitor": 0}

    def qwen(_prompt):
        calls["qwen"] += 1
        return _single_axis_qwen_response(0.05)

    def stale_monitor(_context):
        calls["monitor"] += 1
        return {
            "monitor_type": "mock",
            "camera_check_status": "WARN",
            "scene_freshness_status": "stale",
        }

    result = IsaacSimOperator(
        config=_config(),
        gateway=SyntheticFakeGateway(),
        headless=True,
        output_dir=tmp_path,
        qwen_callable=qwen,
        scene_monitor_callable=stale_monitor,
    ).execute_text("move up 0.05 meters")

    assert result["status"] == "REOBSERVE_REQUIRED"
    assert calls == {"qwen": 1, "monitor": 1}
    assert result["completed_substep_count"] == 1
    assert result["reobserve_triggered"] is True
    assert result["reobserve_reason"] == "E_SCENE_STALE"
    assert result["replan_required"] is True
    assert result["vlm_reobserve_called"] is False
    assert result["llm_reobserve_called"] is False
    assert result["substeps"][0]["continue_allowed"] is False
    assert result["working_memory_after"]["reobserve_required"] is True
    assert result["working_memory_after"]["remaining_delta_m"] == [0.0, 0.0, 0.03]
    assert result["real_robot_motion_executed"] is False
    assert result["trajectory_sent"] is False


def test_qwen_delta_contract_is_identical_at_isaac_and_real_handoffs(tmp_path):
    command = "move up 0.05 meters"
    raw_response = _single_axis_qwen_response(0.05)
    parser_result = evaluate_qwen_motion_parser(
        QwenMotionParserRequest(
            user_text=command,
            max_distance_m=0.35,
            hard_safety_limit_m=0.35,
            llm_callable=lambda _prompt: raw_response,
        )
    )
    real_parsed = real_motion_cli._parsed_from_qwen_result(
        command,
        parser_result,
        max_distance_m=0.35,
        hard_safety_limit_m=0.35,
    )
    real_metadata = real_motion_cli._metadata_from_parser_result(
        parser_result,
        input_mode="command_line",
        original_user_text=command,
        parser_mode="qwen",
    )
    isaac_result = IsaacSimOperator(
        config=_config(),
        gateway=SyntheticFakeGateway(),
        headless=True,
        output_dir=tmp_path,
        qwen_callable=lambda _prompt: raw_response,
    ).execute_text(command)

    expected_contract = {
        "intent": "relative_cartesian_motion",
        "frame": "base_link",
        "direction_axis": "z",
        "direction_sign": "+",
        "distance_m": 0.05,
        "requested_distance_m": 0.05,
        "requested_distance_norm_m": 0.05,
        "delta_m": [0.0, 0.0, 0.05],
        "vector_delta_m": {"x": 0.0, "y": 0.0, "z": 0.05},
        "motion_contract_type": "single_axis_relative",
        "legacy_axis_compatible": True,
    }
    shared_fields = tuple(expected_contract)
    parser_contract = parser_result["normalized_contract"]
    isaac_contract = isaac_result["parsed_motion"]["normalized_contract"]
    real_contract = real_metadata["normalized_contract"]

    assert {key: parser_contract[key] for key in shared_fields} == expected_contract
    assert {key: isaac_contract[key] for key in shared_fields} == expected_contract
    assert {key: real_contract[key] for key in shared_fields} == expected_contract
    assert parser_result["parser_source"] == "qwen_llm"
    assert real_metadata["parser_source"] == "qwen_llm"
    assert real_parsed.parser_source == "qwen_llm"
    assert real_parsed.delta_m == expected_contract["delta_m"]
    assert real_parsed.distance_m == expected_contract["distance_m"]
    assert real_parsed.natural_language_evidence["direction_axis"] == "z"
    assert real_parsed.natural_language_evidence["direction_sign"] == "+"
    assert real_motion_cli._direction_parse_guard(
        parser_metadata=real_metadata,
        parsed=real_parsed,
    ) is None
    assert isaac_result["real_robot_motion_executed"] is False
    assert isaac_result["real_ur_connection_used"] is False
    assert isaac_result["dashboard_used"] is False
    assert isaac_result["rtde_write_used"] is False
    assert isaac_result["moveit_execute_trajectory_called"] is False
    assert isaac_result["trajectory_sent"] is False


def test_gui_operator_refuses_synthetic_gateway(tmp_path):
    with pytest.raises(IsaacOperatorSafetyError, match="E_GUI_DEMO_REQUIRES"):
        IsaacSimOperator(
            config=_config(),
            gateway=SyntheticFakeGateway(),
            headless=False,
            output_dir=tmp_path,
        )


def test_home_and_reset_are_simulation_only(tmp_path):
    gateway = SyntheticFakeGateway()
    operator = IsaacSimOperator(config=_config(), gateway=gateway, headless=True, output_dir=tmp_path)
    assert operator.home() == {"status": "PASS", "simulated_only": True}
    assert operator.reset() == {"status": "PASS", "simulated_only": True}


def test_natural_home_pose_maps_only_named_isaac_joints():
    names = ["fixed_joint", *DEFAULT_ISAAC_HOME_POSE_RAD]
    current = [0.25] * len(names)
    target, applied_names, applied_positions = _named_joint_target(
        names,
        current,
        DEFAULT_ISAAC_HOME_POSE_RAD,
    )

    assert target[0] == 0.25
    assert applied_names == list(DEFAULT_ISAAC_HOME_POSE_RAD)
    assert applied_positions == list(DEFAULT_ISAAC_HOME_POSE_RAD.values())
    assert target[1:] == list(DEFAULT_ISAAC_HOME_POSE_RAD.values())


def test_natural_home_pose_fails_closed_when_joint_is_missing():
    with pytest.raises(RuntimeError, match="E_ISAAC_HOME_JOINTS_MISSING"):
        _named_joint_target(
            ["shoulder_pan_joint"],
            [0.0],
            DEFAULT_ISAAC_HOME_POSE_RAD,
        )


def test_visual_timing_is_gui_only():
    config = {
        "visual_demo_slowdown_enabled": True,
        "motion_duration_sec": 2.4,
        "substep_pause_sec": 0.25,
        "visual_demo_fps": 60,
        "frames_per_substep": 90,
    }
    gui = _visual_timing(config, headless=False, substep_count=3)
    headless = _visual_timing(config, headless=True, substep_count=3)

    assert gui["visual_demo_slowdown_enabled"] is True
    assert gui["motion_frames_per_substep"] == 48
    assert gui["pause_frames_per_substep"] == 15
    assert headless["visual_demo_slowdown_enabled"] is False
    assert headless["motion_frames_per_substep"] == 90
    assert headless["pause_frames_per_substep"] == 0


def test_joint_delta_summary_is_compact():
    summary = _joint_delta_summary(
        {"names": ["a", "b"], "positions_rad": [0.0, 1.0]},
        {"names": ["a", "b"], "positions_rad": [0.1, 1.0]},
    )
    assert summary == [
        {"joint": "a", "before_rad": 0.0, "after_rad": 0.1, "delta_rad": 0.1}
    ]


def test_clean_no_tool_sanitization_preserves_standard_tool0(tmp_path):
    importer = _load_importer_module()
    urdf = tmp_path / "robot.urdf"
    urdf.write_text(
        """<?xml version="1.0"?>
<robot name="test">
  <link name="world"/>
  <link name="base_link"/>
  <link name="flange"/>
  <link name="tool0"/>
  <link name="End_E"><visual><geometry><mesh filename="tool_12.dae"/></geometry></visual></link>
  <link name="stage_3"><visual><geometry><mesh filename="stage_3.dae"/></geometry></visual></link>
  <link name="camera_link"/>
  <link name="ground_plane"/>
  <joint name="flange-tool0" type="fixed"><parent link="flange"/><child link="tool0"/></joint>
  <joint name="tool0_to_my_tool" type="fixed"><parent link="tool0"/><child link="End_E"/></joint>
  <joint name="stage_3_joint" type="fixed"><parent link="End_E"/><child link="stage_3"/></joint>
</robot>
""",
        encoding="utf-8",
    )

    temp_dir, sanitized, metadata = importer._sanitized_urdf_copy(
        urdf,
        clean_no_tool=True,
    )
    try:
        text = sanitized.read_text(encoding="utf-8")
    finally:
        temp_dir.cleanup()

    assert 'link name="tool0"' in text
    assert 'joint name="flange-tool0"' in text
    assert "tool_12" not in text
    assert "stage_3" not in text
    assert metadata["removed_links"] == ["End_E", "camera_link", "ground_plane", "stage_3"]
    assert metadata["removed_joints"] == ["stage_3_joint", "tool0_to_my_tool"]


def test_console_mode_waits_until_quit(monkeypatch):
    console = _load_console_module()
    release_input = threading.Event()

    class FakeOperator:
        def status(self):
            return {"qwen_health": {"ok": False}, "isaac_connection_status": "MOCK_CONNECTED"}

    class FakeGateway:
        def render_once(self):
            return True

    def blocking_input(prompt):
        assert prompt == "TETO/Isaac> "
        release_input.wait(timeout=2.0)
        return "quit"

    monkeypatch.setattr("builtins.input", blocking_input)
    result = []
    thread = threading.Thread(
        target=lambda: result.append(
            console._run_operator_session(
                SimpleNamespace(cmd=None),
                FakeOperator(),
                FakeGateway(),
                object(),
            )
        )
    )
    thread.start()
    time.sleep(0.05)
    assert thread.is_alive(), "console session returned before receiving quit"
    release_input.set()
    thread.join(timeout=2.0)
    assert not thread.is_alive()
    assert result == [0]


def test_one_shot_command_exits_after_command(capsys):
    console = _load_console_module()
    calls = []

    class FakeOperator:
        def execute_text(self, command):
            calls.append(command)
            return {
                "status": "PASS",
                "completed_substep_count": 1,
                "substep_count": 1,
                "gateway_type": "simulated_measured_gateway",
                "real_robot_motion_executed": False,
            }

    status = console._run_operator_session(
        SimpleNamespace(cmd="move forward 1 cm"),
        FakeOperator(),
        object(),
        object(),
    )
    assert status == 0
    assert calls == ["move forward 1 cm"]
    assert "entering one-shot command branch" in capsys.readouterr().out


def test_qwen_unavailable_is_yellow_and_does_not_abort_banner(capsys):
    console = _load_console_module()
    console._banner(
        {
            "qwen_health": {"ok": False, "error": "connection refused"},
            "isaac_connection_status": "ISAAC_SIM_CONNECTED",
        },
        headless=False,
    )
    output = capsys.readouterr().out
    assert "Qwen: YELLOW" in output
    assert "Isaac connection: ISAAC_SIM_CONNECTED" in output


def test_ur5e_asset_override_selects_usd_reference(monkeypatch, tmp_path):
    console = _load_console_module()
    config = _config()
    config.raw["asset_mode"] = "urdf_import"
    asset = tmp_path / "ur5e.usd"
    asset.write_text("#usda 1.0\n", encoding="utf-8")
    observed = {}

    class FakeSimulationApp:
        def __init__(self, settings):
            observed["settings"] = settings

        def close(self):
            observed["closed"] = True

    class FakeBridge:
        gateway_type = "simulated_measured_gateway"

        def __init__(self, *, simulation_app, config, headless):
            observed["asset_mode"] = config["asset_mode"]
            observed["asset_path"] = config["ur5e_asset_path"]

        def status(self):
            return {
                "connection_status": "ISAAC_SIM_CONNECTED",
                "current_tcp_pose": {
                    "frame": "base_link",
                    "position_m": [0.4, 0.0, 0.4],
                    "orientation_xyzw": [0.0, 0.0, 0.0, 1.0],
                },
                "joint_state": {},
            }

    monkeypatch.setattr(console, "load_isaac_operator_config", lambda _path: config)
    monkeypatch.setattr(console, "_banner", lambda *_args: None)
    monkeypatch.setattr(console, "_run_operator_session", lambda *_args: 0)
    monkeypatch.setitem(
        __import__("sys").modules,
        "isaacsim",
        SimpleNamespace(SimulationApp=FakeSimulationApp),
    )
    monkeypatch.setitem(
        __import__("sys").modules,
        "src.isaac_sim_bridge",
        SimpleNamespace(IsaacSimMeasuredBridge=FakeBridge),
    )

    status = console.main(["--gui", "--console", "--ur5e-asset", str(asset)])
    assert status == 0
    assert observed["asset_mode"] == "usd_reference"
    assert observed["asset_path"] == str(asset)
    assert observed["closed"] is True


def test_console_defaults_to_clean_no_tool_asset_and_applies_visual_options(monkeypatch, tmp_path):
    console = _load_console_module()
    config = _config()
    clean_asset = tmp_path / "outputs/isaac_assets/generated_ur5e/ur5e_clean_no_tool.usd"
    clean_asset.parent.mkdir(parents=True)
    clean_asset.write_text("#usda 1.0\n", encoding="utf-8")
    observed = {}

    class FakeSimulationApp:
        def __init__(self, _settings):
            pass

        def close(self):
            pass

    class FakeBridge:
        gateway_type = "simulated_measured_gateway"

        def __init__(self, *, simulation_app, config, headless):
            observed.update(config)

        def status(self):
            return {"connection_status": "ISAAC_SIM_CONNECTED"}

    monkeypatch.setattr(console, "REPO_ROOT", tmp_path)
    monkeypatch.setattr(console, "load_isaac_operator_config", lambda _path: config)
    monkeypatch.setattr(console, "_banner", lambda *_args: None)
    monkeypatch.setattr(console, "_run_operator_session", lambda *_args: 0)
    monkeypatch.setitem(
        __import__("sys").modules,
        "isaacsim",
        SimpleNamespace(SimulationApp=FakeSimulationApp),
    )
    monkeypatch.setitem(
        __import__("sys").modules,
        "src.isaac_sim_bridge",
        SimpleNamespace(IsaacSimMeasuredBridge=FakeBridge),
    )

    assert console.main(
        [
            "--gui",
            "--console",
            "--motion-duration-sec",
            "3.0",
            "--substep-pause-sec",
            "0.4",
        ]
    ) == 0
    assert observed["asset_mode"] == "usd_reference"
    assert observed["ur5e_asset_path"] == str(clean_asset)
    assert observed["motion_duration_sec"] == 3.0
    assert observed["substep_pause_sec"] == 0.4


def test_missing_usd_dependency_is_reported_before_articulation_init(tmp_path):
    asset = tmp_path / "ur5e.usd"
    asset.write_text("#usda 1.0\n", encoding="utf-8")

    class FakeLayer:
        def GetExternalReferences(self):
            return ["ur5e_isaac.usd", "tool_link.usd"]

    sdf_module = SimpleNamespace(
        Layer=SimpleNamespace(FindOrOpen=lambda _path: FakeLayer()),
    )

    missing = _missing_local_usd_dependencies(asset, sdf_module)
    assert missing == [tmp_path / "tool_link.usd", tmp_path / "ur5e_isaac.usd"]


def test_missing_articulation_has_explicit_error_code():
    class InvalidPrim:
        def IsValid(self):
            return False

    stage = SimpleNamespace(GetPrimAtPath=lambda _path: InvalidPrim())
    usd_module = SimpleNamespace(PrimRange=lambda _prim: ())

    with pytest.raises(RuntimeError, match="E_ISAAC_ARTICULATION_NOT_FOUND"):
        _require_articulation(stage, "/World/TETO_UR5e", usd_module, object())


def test_nested_generated_asset_articulation_path_is_selected():
    class FakePath:
        pathString = "/World/TETO_UR5e/root_joint/root_joint"

    class FakePrim:
        def IsValid(self):
            return True

        def HasAPI(self, _api):
            return True

        def GetPath(self):
            return FakePath()

    prim = FakePrim()
    stage = SimpleNamespace(GetPrimAtPath=lambda _path: prim)
    usd_module = SimpleNamespace(PrimRange=lambda _prim: (prim,))

    assert (
        _require_articulation(stage, "/World/TETO_UR5e", usd_module, object())
        == "/World/TETO_UR5e/root_joint/root_joint"
    )


def test_quit_closes_simulation_app_through_main(monkeypatch, tmp_path):
    console = _load_console_module()
    config = _config()
    observed = {"closed": False}

    class FakeSimulationApp:
        def __init__(self, settings):
            observed["settings"] = settings

        def close(self):
            observed["closed"] = True

    class FakeBridge:
        gateway_type = "simulated_measured_gateway"

        def __init__(self, *, simulation_app, config, headless):
            pass

        def status(self):
            return {
                "connection_status": "ISAAC_SIM_CONNECTED",
                "current_tcp_pose": {
                    "frame": "base_link",
                    "position_m": [0.4, 0.0, 0.4],
                    "orientation_xyzw": [0.0, 0.0, 0.0, 1.0],
                },
                "joint_state": {},
            }

        def render_once(self):
            return True

    monkeypatch.setattr(console, "load_isaac_operator_config", lambda _path: config)
    monkeypatch.setattr(console, "_banner", lambda *_args: None)
    monkeypatch.setattr("builtins.input", lambda _prompt: "quit")
    monkeypatch.setitem(
        __import__("sys").modules,
        "isaacsim",
        SimpleNamespace(SimulationApp=FakeSimulationApp),
    )
    monkeypatch.setitem(
        __import__("sys").modules,
        "src.isaac_sim_bridge",
        SimpleNamespace(IsaacSimMeasuredBridge=FakeBridge),
    )

    assert console.main(["--gui", "--console"]) == 0
    assert observed["closed"] is True


def test_launcher_continues_to_console_when_qwen_is_unavailable(tmp_path):
    fake_isaac = tmp_path / "isaac"
    fake_bin = tmp_path / "bin"
    fake_isaac.mkdir()
    fake_bin.mkdir()
    fake_python = fake_isaac / "python.sh"
    fake_python.write_text(
        "#!/usr/bin/env bash\nprintf 'FAKE_ISAAC_PYTHON_ARGS:'\nprintf ' <%s>' \"$@\"\nprintf '\\n'\n",
        encoding="utf-8",
    )
    fake_python.chmod(0o755)
    fake_app = fake_isaac / "isaac-sim.sh"
    fake_app.write_text("#!/usr/bin/env bash\nexit 0\n", encoding="utf-8")
    fake_app.chmod(0o755)
    fake_curl = fake_bin / "curl"
    fake_curl.write_text("#!/usr/bin/env bash\nexit 7\n", encoding="utf-8")
    fake_curl.chmod(0o755)

    environment = dict(os.environ)
    environment["PATH"] = f"{fake_bin}:{environment['PATH']}"
    completed = subprocess.run(
        [
            "bash",
            "scripts/start_teto_isaac_gui_operator.sh",
            "--gui",
            "--console",
            "--isaac-app",
            str(fake_app),
            "--motion-duration-sec",
            "3.0",
            "--substep-pause-sec",
            "0.4",
            "--no-visual-markers",
        ],
        cwd=Path.cwd(),
        env=environment,
        text=True,
        capture_output=True,
        timeout=5,
        check=False,
    )
    assert completed.returncode == 0
    assert "Qwen endpoint unavailable" in completed.stderr
    assert "Launcher branch: persistent_console" in completed.stdout
    assert "FAKE_ISAAC_PYTHON_ARGS:" in completed.stdout
    assert "<--console>" in completed.stdout
    assert "<--motion-duration-sec>" in completed.stdout
    assert "<3.0>" in completed.stdout
    assert "<--substep-pause-sec>" in completed.stdout
    assert "<0.4>" in completed.stdout
    assert "<--no-visual-markers>" in completed.stdout
    assert "<scripts/teto_isaac_operator_console.py>" in completed.stdout
