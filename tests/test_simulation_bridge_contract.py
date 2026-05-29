import json

import pytest

from src.execution_readiness_contract import evaluate_execution_readiness
from src.simulation_bridge_contract import (
    CONTRACT_VERSION,
    build_simulation_task,
    evaluate_replay_record_for_simulation,
    evaluate_simulation_bridge_eligibility,
)


def _valid_normalized_json() -> dict:
    return {
        "schema_version": "teto_robot_task.v1",
        "scene": {"scene_version": "run_simulation_item_001", "status": "valid", "ttl_ms": 500},
        "target": {
            "target_id": "obj_001",
            "label": "camera",
            "candidate": True,
            "bbox_xyxy": [10, 20, 110, 160],
        },
        "geometry_2d": {
            "pixel_center": [60, 90],
            "image_width": 320,
            "image_height": 240,
            "confidence": 0.75,
        },
        "geometry_3d": {"world_point_m": [0.2, 0.1, 0.4]},
        "manipulation_assessment": {"candidate": True, "difficulty": "easy"},
        "error": {"code": "OK", "message": ""},
    }


def test_simulation_ready_pass():
    normalized = _valid_normalized_json()
    readiness = evaluate_execution_readiness(normalized)

    result = evaluate_simulation_bridge_eligibility(normalized, readiness)

    assert result["contract_version"] == CONTRACT_VERSION
    assert result["simulation_ready"] is True
    assert result["blocking_reasons"] == []
    assert result["missing_runtime_inputs"] == []
    assert result["task_type"] == "hover_to_object"
    assert result["target_label"] == "camera"
    assert result["world_point_m"] == [0.2, 0.1, 0.4]
    assert result["allow_robot_motion"] is False


def test_execution_not_ready():
    normalized = _valid_normalized_json()
    readiness = {"ready": False, "blocking_reasons": ["E_NO_TARGET"]}

    result = evaluate_simulation_bridge_eligibility(normalized, readiness)

    assert result["simulation_ready"] is False
    assert "E_NOT_EXECUTION_READY" in result["blocking_reasons"]
    assert "execution_readiness" in result["missing_runtime_inputs"]


def test_missing_world_point():
    normalized = _valid_normalized_json()
    normalized.pop("geometry_3d")

    result = evaluate_simulation_bridge_eligibility(normalized, evaluate_execution_readiness(normalized))

    assert result["simulation_ready"] is False
    assert "E_NO_WORLD_POINT" in result["blocking_reasons"]
    assert "world_point_m" in result["missing_runtime_inputs"]


def test_missing_scene_version():
    normalized = _valid_normalized_json()
    normalized["scene"].pop("scene_version")

    result = evaluate_simulation_bridge_eligibility(normalized, evaluate_execution_readiness(normalized))

    assert result["simulation_ready"] is False
    assert "E_NO_SCENE_VERSION" in result["blocking_reasons"]
    assert "scene_version" in result["missing_runtime_inputs"]


def test_missing_ttl():
    normalized = _valid_normalized_json()
    normalized["scene"].pop("ttl_ms")

    result = evaluate_simulation_bridge_eligibility(normalized, evaluate_execution_readiness(normalized))

    assert result["simulation_ready"] is False
    assert "E_NO_TTL" in result["blocking_reasons"]
    assert "ttl_ms" in result["missing_runtime_inputs"]


def test_build_simulation_task():
    task = build_simulation_task(_valid_normalized_json())

    assert task == {
        "task_type": "hover_to_object",
        "target_label": "camera",
        "target_world_point": [0.2, 0.1, 0.4],
        "scene_version": "run_simulation_item_001",
        "ttl_ms": 500,
    }


def test_build_simulation_task_rejects_not_ready():
    normalized = _valid_normalized_json()
    normalized["scene"].pop("ttl_ms")

    with pytest.raises(ValueError, match="E_NO_TTL"):
        build_simulation_task(normalized)


def test_evaluate_replay_record_for_simulation_returns_task_when_ready():
    replay_record = {"scene_version": "run_simulation_item_001"}
    result_record = {"normalized_json": _valid_normalized_json()}

    result = evaluate_replay_record_for_simulation(replay_record, result_record)

    assert result["simulation_ready"] is True
    assert result["simulation_task"]["target_world_point"] == [0.2, 0.1, 0.4]
    assert result["allow_robot_motion"] is False


def test_evaluate_replay_record_for_simulation_reports_blocking_reasons():
    normalized = _valid_normalized_json()
    normalized["target"]["candidate"] = False
    replay_record = {"scene_version": "run_simulation_item_001"}
    result_record = {"normalized_json": normalized}

    result = evaluate_replay_record_for_simulation(replay_record, result_record)

    assert result["simulation_ready"] is False
    assert "E_NOT_EXECUTION_READY" in result["blocking_reasons"]
    assert result["simulation_task"] is None


def test_simulation_bridge_does_not_contain_robot_control_fields():
    text = json.dumps(build_simulation_task(_valid_normalized_json()))

    assert "joint_angles" not in text
    assert "trajectory" not in text
    assert "urscript" not in text.lower()
    assert "moveit_goal" not in text
    assert "execution_command" not in text
    assert "tcp_pose_world" not in text
