from src.projector_contract import (
    CONTRACT_VERSION,
    DEFAULT_MISSING_RUNTIME_INPUTS,
    PROJECTOR_ERROR_CODES,
    build_projector_input,
    evaluate_projector_eligibility,
)


def _valid_normalized_json() -> dict:
    return {
        "schema_version": "teto_robot_task.v1",
        "scene": {"scene_version": "run_projector_item_001", "status": "valid"},
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
        "manipulation_assessment": {"candidate": True, "difficulty": "easy"},
        "error": {"code": "OK", "message": ""},
    }


def test_projector_eligibility_accepts_valid_target():
    result = evaluate_projector_eligibility(_valid_normalized_json())

    assert result["contract_version"] == CONTRACT_VERSION
    assert result["projector_status"] == "ready"
    assert result["eligible"] is True
    assert result["errors"] == []
    assert result["camera_point_m"] is None
    assert result["world_point_m"] is None
    assert result["projector_confidence"] == 0.0
    assert result["missing_runtime_inputs"] == DEFAULT_MISSING_RUNTIME_INPUTS
    assert result["allow_robot_motion"] is False


def test_projector_eligibility_rejects_missing_bbox():
    normalized = _valid_normalized_json()
    normalized["target"]["bbox_xyxy"] = None

    result = evaluate_projector_eligibility(normalized)

    assert result["eligible"] is False
    assert result["projector_status"] == "rejected"
    assert "E_MISSING_BBOX" in result["errors"]
    assert "E_NOT_GROUNDED" in result["errors"]


def test_projector_eligibility_rejects_missing_pixel_center():
    normalized = _valid_normalized_json()
    normalized["geometry_2d"]["pixel_center"] = None

    result = evaluate_projector_eligibility(normalized)

    assert result["eligible"] is False
    assert "E_MISSING_PIXEL_CENTER" in result["errors"]
    assert "E_NOT_GROUNDED" in result["errors"]


def test_projector_eligibility_rejects_low_geometry_confidence():
    normalized = _valid_normalized_json()
    normalized["geometry_2d"]["confidence"] = 0.49

    result = evaluate_projector_eligibility(normalized)

    assert result["eligible"] is False
    assert result["errors"] == ["E_LOW_GEOMETRY_CONFIDENCE"]


def test_projector_eligibility_rejects_not_grounded():
    normalized = _valid_normalized_json()
    normalized["grounded"] = False

    result = evaluate_projector_eligibility(normalized)

    assert result["eligible"] is False
    assert "E_NOT_GROUNDED" in result["errors"]
    assert "E_MISSING_BBOX" not in result["errors"]
    assert "E_MISSING_PIXEL_CENTER" not in result["errors"]


def test_projector_eligibility_rejects_not_candidate():
    normalized = _valid_normalized_json()
    normalized["target"]["candidate"] = False
    normalized["manipulation_assessment"]["candidate"] = False

    result = evaluate_projector_eligibility(normalized)

    assert result["eligible"] is False
    assert result["errors"] == ["E_NOT_CANDIDATE"]


def test_build_projector_input_builds_dry_run_skeleton():
    projector_input = build_projector_input(_valid_normalized_json())

    assert projector_input == {
        "contract_version": CONTRACT_VERSION,
        "pixel_center": [60, 90],
        "bbox_xyxy": [10, 20, 110, 160],
        "image_width": 320,
        "image_height": 240,
        "camera_frame": None,
        "depth_sample": None,
        "allow_robot_motion": False,
    }


def test_projector_contract_allow_robot_motion_always_false():
    normalized = _valid_normalized_json()
    rejected = _valid_normalized_json()
    rejected["target"]["candidate"] = False

    assert evaluate_projector_eligibility(normalized)["allow_robot_motion"] is False
    assert evaluate_projector_eligibility(rejected)["allow_robot_motion"] is False
    assert build_projector_input(normalized)["allow_robot_motion"] is False


def test_projector_contract_vocabulary_keeps_future_runtime_errors():
    assert "OK" in PROJECTOR_ERROR_CODES
    assert "E_NO_DEPTH" in PROJECTOR_ERROR_CODES
    assert "E_CAMERA_INFO_MISSING" in PROJECTOR_ERROR_CODES
    assert "E_CAMERA_FRAME_MISSING" in PROJECTOR_ERROR_CODES
    assert "E_TF_UNAVAILABLE" in PROJECTOR_ERROR_CODES
    assert "E_WORLD_TRANSFORM_FAILED" in PROJECTOR_ERROR_CODES
    assert "E_LOW_PROJECTOR_CONFIDENCE" in PROJECTOR_ERROR_CODES


def test_projector_contract_does_not_generate_world_or_robot_control_fields():
    result = evaluate_projector_eligibility(_valid_normalized_json())
    projector_input = build_projector_input(_valid_normalized_json())

    assert result["camera_point_m"] is None
    assert result["world_point_m"] is None
    assert "tcp_pose_world" not in projector_input
    assert "joint_angles" not in projector_input
    assert "trajectory" not in projector_input
