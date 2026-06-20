import json
import importlib.util
import os
import subprocess
import threading
import time
from pathlib import Path
from types import SimpleNamespace

import pytest

from src.isaac_sim_operator import (
    GATEWAY_SYNTHETIC_FAKE,
    IsaacOperatorConfig,
    IsaacOperatorSafetyError,
    IsaacSimOperator,
    SyntheticFakeGateway,
    load_isaac_operator_config,
    validate_no_real_robot_args,
    validate_no_real_robot_config,
)

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


def _load_console_module():
    spec = importlib.util.spec_from_file_location("teto_isaac_operator_console_test", CONSOLE_SCRIPT)
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


def test_real_flag_is_refused():
    with pytest.raises(IsaacOperatorSafetyError, match="E_REAL_FLAG_FORBIDDEN"):
        validate_no_real_robot_args(["--real"])


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
    assert "<scripts/teto_isaac_operator_console.py>" in completed.stdout
