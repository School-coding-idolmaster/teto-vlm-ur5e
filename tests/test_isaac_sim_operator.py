import json
from pathlib import Path

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
