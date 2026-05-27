import json

from src.json_validator import (
    extract_json_from_response,
    normalize_robot_task_json,
    parse_robot_task_response,
    validate_robot_task_json,
)


VALID_ROBOT_TASK = {
    "schema_version": "teto_robot_task.v1",
    "task_type": "target_analysis",
    "user_instruction": "pick the red cup",
    "target": {
        "label": "red cup",
        "approx_position": "center",
        "visibility": "clear",
    },
    "spatial_context": {
        "surface": "table",
        "nearby_objects": ["box"],
        "relations": [
            {
                "object": "red cup",
                "relation": "left_of",
                "target": "box",
                "confidence": "high",
            }
        ],
        "obstacles": [],
    },
    "manipulation_assessment": {
        "candidate": True,
        "difficulty": "easy",
        "reason": "clear target",
    },
    "confidence": {
        "semantic": "high",
        "spatial": "medium",
        "overall": "medium",
    },
    "error": {
        "code": "OK",
        "message": "",
    },
}


def test_extract_json_from_response_parses_strict_object():
    result = extract_json_from_response(json.dumps(VALID_ROBOT_TASK))

    assert result["parse_status"] == "success"
    assert result["data"]["schema_version"] == "teto_robot_task.v1"


def test_extract_json_from_response_rejects_embedded_object():
    result = extract_json_from_response(f"prefix {json.dumps(VALID_ROBOT_TASK)}")

    assert result["parse_status"] == "failed"


def test_validate_robot_task_json_accepts_controlled_vocab():
    assert validate_robot_task_json(VALID_ROBOT_TASK) == []


def test_normalize_robot_task_json_marks_invalid_vocab():
    data = {
        **VALID_ROBOT_TASK,
        "target": {
            "label": "cup",
            "approx_position": "middle-ish",
            "visibility": "clear",
        },
    }

    result = normalize_robot_task_json(data)

    assert result["parse_status"] == "success"
    assert result["validation_status"] == "failed"
    assert result["normalized_json"]["target"]["approx_position"] == "unknown"
    assert result["validation_errors"]


def test_parse_robot_task_response_handles_parse_failure():
    result = parse_robot_task_response("not json")

    assert result["parse_status"] == "failed"
    assert result["validation_status"] == "failed"
    assert result["parsed_json"] is None
    assert result["normalized_json"]["error"]["code"] == "E_PARSE"
    assert result["raw_response"] == "not json"


def test_parse_robot_task_response_forces_living_target_unsafe():
    data = {
        **VALID_ROBOT_TASK,
        "target": {
            "label": "bird",
            "approx_position": "center",
            "visibility": "clear",
        },
        "manipulation_assessment": {
            "candidate": True,
            "difficulty": "medium",
            "reason": "visible target",
        },
        "error": {
            "code": "OK",
            "message": "",
        },
    }

    result = parse_robot_task_response(json.dumps(data))

    assert result["parse_status"] == "success"
    assert result["validation_status"] == "failed"
    assert result["parsed_json"]["manipulation_assessment"]["candidate"] is True
    assert result["normalized_json"]["manipulation_assessment"]["candidate"] is False
    assert result["normalized_json"]["manipulation_assessment"]["difficulty"] == "unsafe"
    assert result["normalized_json"]["error"]["code"] == "E_UNSAFE"
    assert result["validation_errors"]
    assert result["validation_warnings"]


def test_parse_robot_task_response_forces_living_being_unsafe():
    data = {
        **VALID_ROBOT_TASK,
        "target": {
            "label": "living_being",
            "approx_position": "center",
            "visibility": "clear",
        },
    }

    result = parse_robot_task_response(json.dumps(data))

    assert result["parse_status"] == "success"
    assert result["validation_status"] == "failed"
    assert result["normalized_json"]["manipulation_assessment"]["candidate"] is False
    assert result["normalized_json"]["manipulation_assessment"]["difficulty"] == "unsafe"
    assert result["normalized_json"]["error"]["code"] == "E_UNSAFE"


def test_normalize_robot_task_json_marks_unknown_candidate_as_no_target():
    data = {
        **VALID_ROBOT_TASK,
        "target": {
            "label": "unknown",
            "approx_position": "unknown",
            "visibility": "unknown",
        },
        "manipulation_assessment": {
            "candidate": True,
            "difficulty": "unknown",
            "reason": "unknown",
        },
    }

    result = normalize_robot_task_json(data)

    assert result["validation_status"] == "failed"
    assert result["normalized_json"]["manipulation_assessment"]["candidate"] is False
    assert result["normalized_json"]["error"]["code"] == "E_NO_TARGET"


def test_unknown_scene_without_living_terms_stays_no_target():
    data = {
        **VALID_ROBOT_TASK,
        "target": {
            "label": "unknown",
            "approx_position": "unknown",
            "visibility": "unknown",
        },
        "spatial_context": {
            "surface": "unknown",
            "nearby_objects": [],
            "relations": [],
            "obstacles": [],
        },
        "manipulation_assessment": {
            "candidate": False,
            "difficulty": "unknown",
            "reason": "no clear object",
        },
        "error": {
            "code": "E_NO_TARGET",
            "message": "No target found.",
        },
    }

    result = normalize_robot_task_json(data)

    assert result["normalized_json"]["error"]["code"] == "E_NO_TARGET"


def test_living_only_scene_with_unknown_target_normalizes_to_unsafe():
    data = {
        **VALID_ROBOT_TASK,
        "target": {
            "label": "unknown",
            "approx_position": "unknown",
            "visibility": "unknown",
        },
        "spatial_context": {
            "surface": "floor",
            "nearby_objects": ["person"],
            "relations": [],
            "obstacles": [],
        },
        "manipulation_assessment": {
            "candidate": False,
            "difficulty": "unknown",
            "reason": "no manipulation candidate",
        },
        "error": {
            "code": "E_NO_TARGET",
            "message": "",
        },
    }

    result = normalize_robot_task_json(data)

    assert result["normalized_json"]["manipulation_assessment"]["candidate"] is False
    assert result["normalized_json"]["manipulation_assessment"]["difficulty"] == "unsafe"
    assert result["normalized_json"]["error"]["code"] == "E_UNSAFE"
    assert (
        result["normalized_json"]["error"]["message"]
        == "Only living beings or unsafe targets were detected; no manipulation candidate is allowed."
    )


def test_living_only_scene_from_detected_objects_normalizes_to_unsafe():
    data = {
        **VALID_ROBOT_TASK,
        "target": {
            "label": "unknown",
            "approx_position": "unknown",
            "visibility": "unknown",
        },
        "spatial_context": {
            "surface": "unknown",
            "nearby_objects": [],
            "relations": [],
            "obstacles": [],
        },
        "detected_objects": [{"label": "woman"}],
        "manipulation_assessment": {
            "candidate": False,
            "difficulty": "unknown",
            "reason": "no manipulation candidate",
        },
        "error": {
            "code": "E_NO_TARGET",
            "message": "",
        },
    }

    result = normalize_robot_task_json(data)

    assert result["normalized_json"]["error"]["code"] == "E_UNSAFE"


def test_bbox_normalizes_and_infers_pixel_center():
    data = {
        **VALID_ROBOT_TASK,
        "target": {
            **VALID_ROBOT_TASK["target"],
            "bbox_xyxy": [10, 20, 110, 220],
        },
    }

    result = normalize_robot_task_json(data, image_size=(640, 480))

    assert result["validation_errors"] == []
    assert result["normalized_json"]["target"]["bbox_xyxy"] == [10.0, 20.0, 110.0, 220.0]
    assert result["normalized_json"]["geometry_2d"]["pixel_center"] == [60.0, 120.0]
    assert result["normalized_json"]["geometry_2d"]["image_width"] == 640
    assert result["normalized_json"]["geometry_2d"]["image_height"] == 480
    assert "pixel_center was inferred from bbox_xyxy" in result["validation_warnings"]


def test_bbox_out_of_bounds_warns_without_failed_status():
    data = {
        **VALID_ROBOT_TASK,
        "target": {
            **VALID_ROBOT_TASK["target"],
            "bbox_xyxy": [10, 20, 700, 520],
        },
    }

    result = normalize_robot_task_json(data, image_size=(640, 480))

    assert result["validation_errors"] == []
    assert result["validation_status"] == "warning"
    assert "target.bbox_xyxy x coordinates are outside image bounds" in result["validation_warnings"]
    assert "target.bbox_xyxy y coordinates are outside image bounds" in result["validation_warnings"]


def test_bad_bbox_format_reports_validation_error_without_crashing():
    data = {
        **VALID_ROBOT_TASK,
        "target": {
            **VALID_ROBOT_TASK["target"],
            "bbox_xyxy": [10, 20, 110],
        },
    }

    result = normalize_robot_task_json(data, image_size=(640, 480))

    assert result["validation_status"] == "failed"
    assert "target.bbox_xyxy must be null or a list of 4 numbers" in result["validation_errors"]


def test_valid_pixel_center_is_preserved():
    data = {
        **VALID_ROBOT_TASK,
        "geometry_2d": {
            "pixel_center": [100, 200],
            "confidence": 0.7,
        },
    }

    result = normalize_robot_task_json(data, image_size=(640, 480))

    assert result["normalized_json"]["geometry_2d"]["pixel_center"] == [100.0, 200.0]
    assert result["normalized_json"]["geometry_2d"]["confidence"] == 0.7
    assert result["normalized_json"]["geometry_2d"]["image_width"] == 640
    assert result["normalized_json"]["geometry_2d"]["image_height"] == 480


def test_pixel_center_out_of_bounds_warns_without_crashing():
    data = {
        **VALID_ROBOT_TASK,
        "geometry_2d": {
            "pixel_center": [700, 200],
        },
    }

    result = normalize_robot_task_json(data, image_size=(640, 480))

    assert result["validation_errors"] == []
    assert result["validation_status"] == "warning"
    assert "geometry_2d.pixel_center x coordinate is outside image bounds" in result["validation_warnings"]


def test_unsafe_target_with_bbox_keeps_safety_override_and_bbox_for_audit():
    data = {
        **VALID_ROBOT_TASK,
        "target": {
            "label": "person",
            "bbox_xyxy": [10, 20, 110, 220],
            "approx_position": "center",
            "visibility": "clear",
        },
        "manipulation_assessment": {
            "candidate": True,
            "difficulty": "easy",
            "reason": "visible",
        },
        "error": {
            "code": "OK",
            "message": "",
        },
    }

    result = normalize_robot_task_json(data, image_size=(640, 480))

    assert result["normalized_json"]["target"]["bbox_xyxy"] == [10.0, 20.0, 110.0, 220.0]
    assert result["normalized_json"]["target"]["candidate"] is False
    assert result["normalized_json"]["manipulation_assessment"]["candidate"] is False
    assert result["normalized_json"]["manipulation_assessment"]["difficulty"] == "unsafe"
    assert result["normalized_json"]["error"]["code"] == "E_UNSAFE"


def test_bbox_can_be_read_from_common_object_locations():
    data = {
        **VALID_ROBOT_TASK,
        "objects": [{"label": "red cup", "bbox_xyxy": [1, 2, 11, 22]}],
    }

    result = normalize_robot_task_json(data, image_size=(100, 100))

    assert result["normalized_json"]["target"]["bbox_xyxy"] == [1.0, 2.0, 11.0, 22.0]
