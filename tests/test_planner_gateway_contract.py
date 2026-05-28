import json

import pytest

from src.planner_gateway_contract import (
    CONTRACT_VERSION,
    build_planner_gateway_input,
    evaluate_planner_gateway_eligibility,
    evaluate_replay_record_for_planner,
)


def _valid_normalized_json() -> dict:
    return {
        "schema_version": "teto_robot_task.v1",
        "task_type": "target_analysis",
        "user_instruction": "hover over the camera",
        "scene": {
            "scene_version": "run_planner_item_001",
            "status": "valid",
        },
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
        "confidence": {
            "semantic": "high",
            "spatial": "medium",
            "overall": "high",
        },
        "manipulation_assessment": {
            "candidate": True,
            "difficulty": "easy",
        },
        "error": {
            "code": "OK",
            "message": "",
        },
    }


def test_planner_gateway_eligibility_accepts_valid_grounded_candidate():
    result = evaluate_planner_gateway_eligibility(_valid_normalized_json())

    assert result == {
        "eligible": True,
        "status": "eligible",
        "reasons": [],
        "warnings": [],
        "required_missing_fields": [],
        "contract_version": CONTRACT_VERSION,
    }


def test_planner_gateway_eligibility_rejects_no_target():
    normalized = _valid_normalized_json()
    normalized["target"].update({"target_id": "unknown", "label": "unknown", "candidate": False})
    normalized["target"]["bbox_xyxy"] = None
    normalized["geometry_2d"]["pixel_center"] = None
    normalized["geometry_2d"]["confidence"] = 0.0
    normalized["error"]["code"] = "E_NO_TARGET"

    result = evaluate_planner_gateway_eligibility(normalized)

    assert result["eligible"] is False
    assert result["status"] == "rejected"
    assert "E_NO_TARGET" in result["reasons"]
    assert "E_NOT_CANDIDATE" in result["reasons"]
    assert "E_UNKNOWN_TARGET" in result["reasons"]
    assert "target.target_id" in result["required_missing_fields"]


def test_planner_gateway_eligibility_rejects_missing_bbox():
    normalized = _valid_normalized_json()
    normalized["target"]["bbox_xyxy"] = None

    result = evaluate_planner_gateway_eligibility(normalized)

    assert result["eligible"] is False
    assert "E_MISSING_BBOX" in result["reasons"]
    assert "E_NOT_GROUNDED" in result["reasons"]
    assert "target.bbox_xyxy" in result["required_missing_fields"]


def test_planner_gateway_eligibility_rejects_missing_pixel_center():
    normalized = _valid_normalized_json()
    normalized["geometry_2d"]["pixel_center"] = None

    result = evaluate_planner_gateway_eligibility(normalized)

    assert result["eligible"] is False
    assert "E_MISSING_PIXEL_CENTER" in result["reasons"]
    assert "E_NOT_GROUNDED" in result["reasons"]
    assert "geometry_2d.pixel_center" in result["required_missing_fields"]


def test_planner_gateway_eligibility_rejects_not_grounded():
    result = evaluate_planner_gateway_eligibility(_valid_normalized_json(), grounded=False)

    assert result["eligible"] is False
    assert "E_NOT_GROUNDED" in result["reasons"]


def test_planner_gateway_eligibility_rejects_scene_invalid():
    normalized = _valid_normalized_json()
    normalized["scene"]["status"] = "invalid"

    result = evaluate_planner_gateway_eligibility(normalized)

    assert result["eligible"] is False
    assert "E_SCENE_INVALID" in result["reasons"]


def test_build_planner_gateway_input_is_dry_run_only():
    planner_input = build_planner_gateway_input(_valid_normalized_json(), task_id="task_001")

    assert planner_input["contract_version"] == CONTRACT_VERSION
    assert planner_input["task_id"] == "task_001"
    assert planner_input["execution_policy"]["dry_run_only"] is True
    assert planner_input["execution_policy"]["allow_robot_motion"] is False
    assert planner_input["planner_requirements"]["requires_depth"] is True
    assert planner_input["planner_requirements"]["requires_tf"] is True


def test_build_planner_gateway_input_does_not_contain_robot_control_fields():
    planner_input = build_planner_gateway_input(_valid_normalized_json())
    text = json.dumps(planner_input)

    assert "URScript" not in text
    assert "joint_angles" not in text
    assert "trajectory" not in text
    assert "tcp_pose_world" not in text


def test_replay_record_planner_eligibility_uses_normalized_fields_only():
    result_record = {
        "parsed_json": {
            "target": {"bbox_xyxy": [1, 2, 3, 4]},
            "geometry_2d": {"pixel_center": [2, 3]},
        },
        "normalized_json": _valid_normalized_json(),
    }
    result_record["normalized_json"]["target"]["bbox_xyxy"] = None
    result_record["normalized_json"]["geometry_2d"]["pixel_center"] = None
    replay_record = {
        "scene_version": "run_planner_item_001",
        "grounded": True,
    }

    result = evaluate_replay_record_for_planner(replay_record, result_record)

    assert result["eligible"] is False
    assert "E_MISSING_BBOX" in result["reasons"]
    assert "E_MISSING_PIXEL_CENTER" in result["reasons"]
    assert result["planner_input"] is None


def test_planner_gateway_input_lists_missing_runtime_inputs():
    planner_input = build_planner_gateway_input(_valid_normalized_json())

    assert "depth_aligned_to_color" in planner_input["missing_runtime_inputs"]
    assert "camera_point_m" in planner_input["missing_runtime_inputs"]
    assert "world_point_m" in planner_input["missing_runtime_inputs"]
    assert "tf_timestamp" in planner_input["missing_runtime_inputs"]
    assert "scene_ttl_ms" in planner_input["missing_runtime_inputs"]
    assert "robot_safety_state" in planner_input["missing_runtime_inputs"]


def test_build_planner_gateway_input_rejects_ineligible_input():
    normalized = _valid_normalized_json()
    normalized["error"]["code"] = "E_NO_TARGET"

    with pytest.raises(ValueError, match="E_NO_TARGET"):
        build_planner_gateway_input(normalized)
