import json

from src.execution_readiness_contract import (
    CONTRACT_VERSION,
    build_execution_readiness_input,
    evaluate_execution_readiness,
)


def _valid_normalized_json() -> dict:
    return {
        "schema_version": "teto_robot_task.v1",
        "scene": {"scene_version": "run_execution_item_001", "status": "valid"},
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
        "confidence": {"semantic": "high", "spatial": "medium", "overall": "high"},
        "manipulation_assessment": {"candidate": True, "difficulty": "easy"},
        "error": {"code": "OK", "message": ""},
    }


def test_execution_readiness_rejects_when_planner_rejected():
    normalized = _valid_normalized_json()
    normalized["target"].update({"target_id": "unknown", "label": "unknown", "candidate": False})
    normalized["target"]["bbox_xyxy"] = None
    normalized["geometry_2d"]["pixel_center"] = None
    normalized["geometry_2d"]["confidence"] = 0.0
    normalized["error"]["code"] = "E_NO_TARGET"

    result = evaluate_execution_readiness(normalized)

    assert result["contract_version"] == CONTRACT_VERSION
    assert result["ready"] is False
    assert result["status"] == "planner_rejected"
    assert result["planner_eligible"] is False
    assert result["projector_eligible"] is False
    assert result["allow_robot_motion"] is False
    assert "E_NO_TARGET" in result["blocking_reasons"]


def test_execution_readiness_rejects_when_projector_rejected():
    normalized = _valid_normalized_json()
    normalized["geometry_2d"]["confidence"] = 0.25

    result = evaluate_execution_readiness(normalized)

    assert result["ready"] is False
    assert result["status"] == "projector_rejected"
    assert result["planner_eligible"] is True
    assert result["projector_eligible"] is False
    assert result["blocking_reasons"] == ["E_LOW_GEOMETRY_CONFIDENCE"]


def test_execution_readiness_reports_dry_run_ready():
    result = evaluate_execution_readiness(_valid_normalized_json())

    assert result["ready"] is True
    assert result["status"] == "dry_run_ready"
    assert result["planner_eligible"] is True
    assert result["projector_eligible"] is True
    assert result["blocking_reasons"] == []
    assert result["allow_robot_motion"] is False


def test_execution_readiness_aggregates_blocking_reasons():
    normalized = _valid_normalized_json()
    normalized["target"].update({"target_id": "unknown", "label": "unknown", "candidate": False})
    normalized["target"]["bbox_xyxy"] = None
    normalized["geometry_2d"]["pixel_center"] = None
    normalized["geometry_2d"]["confidence"] = 0.0
    normalized["error"]["code"] = "E_NO_TARGET"

    result = evaluate_execution_readiness(normalized)

    assert "E_NO_TARGET" in result["blocking_reasons"]
    assert "E_NOT_CANDIDATE" in result["blocking_reasons"]
    assert "E_MISSING_BBOX" in result["blocking_reasons"]
    assert "E_MISSING_PIXEL_CENTER" in result["blocking_reasons"]
    assert "E_NOT_GROUNDED" in result["blocking_reasons"]


def test_execution_readiness_allow_robot_motion_always_false():
    ready = evaluate_execution_readiness(_valid_normalized_json())
    rejected = _valid_normalized_json()
    rejected["target"]["candidate"] = False

    assert ready["allow_robot_motion"] is False
    assert evaluate_execution_readiness(rejected)["allow_robot_motion"] is False
    assert build_execution_readiness_input(_valid_normalized_json())["execution_policy"]["allow_robot_motion"] is False


def test_build_execution_readiness_input_builds_dry_run_summary():
    summary = build_execution_readiness_input(_valid_normalized_json())

    assert summary["contract_version"] == CONTRACT_VERSION
    assert summary["readiness"]["status"] == "dry_run_ready"
    assert summary["scene_version"] == "run_execution_item_001"
    assert summary["target"]["target_id"] == "obj_001"
    assert summary["target"]["label"] == "camera"
    assert summary["target"]["bbox_xyxy"] == [10, 20, 110, 160]
    assert summary["target"]["pixel_center"] == [60, 90]
    assert summary["execution_policy"] == {"dry_run_only": True, "allow_robot_motion": False}


def test_build_execution_readiness_input_lists_runtime_contract_inputs():
    summary = build_execution_readiness_input(_valid_normalized_json())

    assert "depth_aligned_to_color" in summary["missing_runtime_inputs"]
    assert "camera_point_m" in summary["missing_runtime_inputs"]
    assert "world_point_m" in summary["missing_runtime_inputs"]
    assert "depth_sample" in summary["missing_runtime_inputs"]
    assert "camera_info" in summary["missing_runtime_inputs"]
    assert "tf_tree" in summary["missing_runtime_inputs"]


def test_execution_readiness_input_does_not_contain_control_fields():
    text = json.dumps(build_execution_readiness_input(_valid_normalized_json()))

    assert "MoveIt" not in text
    assert "URScript" not in text
    assert "joint_angles" not in text
    assert "trajectory" not in text
    assert "tcp_pose_world" not in text


def test_execution_readiness_accepts_result_record_wrapper():
    result = evaluate_execution_readiness({"normalized_json": _valid_normalized_json()})

    assert result["ready"] is True
    assert result["status"] == "dry_run_ready"
