import pytest

from src.motion_command_normalizer import (
    DEFAULT_SMALL_STEP_M,
    POLICY_VERSION,
    STATUS_NEEDS_CLARIFICATION,
    STATUS_PASS,
    STATUS_UNSUPPORTED_INTENT,
    normalize_motion_command,
)


@pytest.mark.parametrize(
    ("command", "axis", "sign", "distance"),
    [
        ("raise the tcp by 5 cm", "z", "+", 0.05),
        ("drop the end effector 2 centimeters", "z", "-", 0.02),
        ("shift the arm tip left by 10 mm", "y", "+", 0.01),
        ("move forward 5 centimeters", "x", "+", 0.05),
        ("go backward by 3 cm", "x", "-", 0.03),
        ("raise it by two centimeters", "z", "+", 0.02),
        ("lower it by five millimeters", "z", "-", 0.005),
        ("move the tcp to the right by 1 cm", "y", "-", 0.01),
        ("把 tcp 抬高 5 厘米", "z", "+", 0.05),
        ("工具向左移动 1 厘米", "y", "+", 0.01),
    ],
)
def test_explicit_language_commands_parse_to_relative_cartesian_motion(command, axis, sign, distance):
    result = normalize_motion_command(command)

    assert result["parse_status"] == STATUS_PASS
    assert result["natural_language_coverage_version"] == POLICY_VERSION
    assert result["intent"] == "relative_cartesian_motion"
    assert result["motion_frame"] == "base_link"
    assert result["direction_axis"] == axis
    assert result["direction_sign"] == sign
    assert result["requested_distance_m"] == distance
    assert result["distance_source"] == "explicit"
    assert result["direction_source"] == "explicit_direction_word"
    assert result["requires_confirmation"] is True
    assert result["safety_gate_still_required"] is True
    assert result["execution_permission_decided_by_parser"] is False


@pytest.mark.parametrize(
    ("command", "axis", "sign"),
    [
        ("go down a little", "z", "-"),
        ("move up a bit", "z", "+"),
        ("lift the tool slightly", "z", "+"),
        ("nudge the tcp left", "y", "+"),
        ("shift the end effector forward slightly", "x", "+"),
        ("bring the robot hand slightly higher", "z", "+"),
        ("末端往下移动一点", "z", "-"),
        ("机械臂末端往前一点", "x", "+"),
    ],
)
def test_fuzzy_small_step_commands_infer_default_distance(command, axis, sign):
    result = normalize_motion_command(command)

    assert result["parse_status"] == STATUS_PASS
    assert result["direction_axis"] == axis
    assert result["direction_sign"] == sign
    assert result["requested_distance_m"] == DEFAULT_SMALL_STEP_M
    assert result["distance_source"] == "inferred_default"
    assert result["inferred_default_distance_m"] == DEFAULT_SMALL_STEP_M
    assert result["requires_confirmation"] is True
    assert result["execution_permission_decided_by_parser"] is False


@pytest.mark.parametrize(
    ("command", "reason"),
    [
        ("move it over there", "E_TARGET_LOCATION_UNSPECIFIED"),
        ("move closer", "E_TARGET_LOCATION_UNSPECIFIED"),
        ("move 5 cm", "E_DIRECTION_MISSING"),
        ("move up and down 5 cm", "E_CONFLICTING_DIRECTIONS"),
    ],
)
def test_ambiguous_language_requires_clarification(command, reason):
    result = normalize_motion_command(command)

    assert result["parse_status"] == STATUS_NEEDS_CLARIFICATION
    assert result["clarification_required"] is True
    assert result["clarification_reason"] == reason
    assert result["execution_permission_decided_by_parser"] is False


@pytest.mark.parametrize(
    ("command", "reason"),
    [
        ("move to the mug", "NEEDS_VISION"),
        ("grab the cup", "UNSUPPORTED_VISION_OR_MANIPULATION_INTENT"),
        ("go faster", "UNSUPPORTED_SPEED_INTENT"),
    ],
)
def test_object_and_manipulation_language_is_unsupported(command, reason):
    result = normalize_motion_command(command)

    assert result["parse_status"] == STATUS_UNSUPPORTED_INTENT
    assert result["unsupported_intent_reason"] == reason
    assert result["execution_permission_decided_by_parser"] is False


def test_qwen_semantic_fuzzy_down_beats_rule_distance_gap():
    result = normalize_motion_command(
        "drop the tool a tiny bit",
        qwen_semantic=_semantic_payload(direction="down", quality="fuzzy_small", meters=None, language="en"),
    )

    assert result["parse_status"] == STATUS_PASS
    assert result["parser_source"] == "qwen_llm"
    assert result["direction_axis"] == "z"
    assert result["direction_sign"] == "-"
    assert result["requested_distance_m"] == DEFAULT_SMALL_STEP_M
    assert result["distance_source"] == "inferred_default"
    assert result["qwen_semantic_parse_used"] is True
    assert result["fallback_parse_used"] is False
    assert result["qwen_fallback_conflict"] is True
    assert result["qwen_fallback_conflict_reason"] == "fallback_rejected:E_DISTANCE_MISSING"
    assert result["canonicalization_source"] == "qwen_semantic"
    assert result["execution_permission_decided_by_parser"] is False
    assert result["safety_gate_still_required"] is True


def test_qwen_semantic_chinese_explicit_down_does_not_need_rule_synonym():
    result = normalize_motion_command(
        "把末端降低 2 厘米",
        qwen_semantic=_semantic_payload(direction="down", quality="explicit", meters=0.02, value=2, unit="cm", language="zh"),
    )

    assert result["parse_status"] == STATUS_PASS
    assert result["direction_axis"] == "z"
    assert result["direction_sign"] == "-"
    assert result["requested_distance_m"] == 0.02
    assert result["qwen_direction_semantic"] == "down"
    assert result["qwen_distance_quality"] == "explicit"
    assert result["qwen_distance_m"] == 0.02
    assert result["qwen_language"] == "zh"
    assert result["canonicalization_source"] == "qwen_semantic"


def test_qwen_semantic_vision_target_is_unsupported_not_relative_motion():
    result = normalize_motion_command(
        "move to the mug",
        qwen_semantic=_unsupported_payload("vision_target_motion", "NEEDS_VISION"),
    )

    assert result["parse_status"] == STATUS_UNSUPPORTED_INTENT
    assert result["unsupported_intent_reason"] == "NEEDS_VISION"
    assert result["qwen_semantic_parse_used"] is True
    assert result["canonicalization_source"] == "unsupported"


def test_qwen_semantic_manipulation_is_unsupported():
    result = normalize_motion_command(
        "grab the cup",
        qwen_semantic=_unsupported_payload("manipulation", "UNSUPPORTED_VISION_OR_MANIPULATION_INTENT"),
    )

    assert result["parse_status"] == STATUS_UNSUPPORTED_INTENT
    assert result["unsupported_intent_reason"] == "UNSUPPORTED_VISION_OR_MANIPULATION_INTENT"
    assert result["qwen_semantic_parse_used"] is True


def test_qwen_semantic_direction_missing_requires_clarification():
    result = normalize_motion_command(
        "move 5 cm",
        qwen_semantic=_clarification_payload(direction="missing", distance_quality="explicit", reason="E_DIRECTION_MISSING"),
    )

    assert result["parse_status"] == STATUS_NEEDS_CLARIFICATION
    assert result["clarification_reason"] == "E_DIRECTION_MISSING"
    assert result["qwen_semantic_parse_used"] is True
    assert result["canonicalization_source"] == "clarification"


def test_qwen_semantic_conflicting_direction_requires_clarification():
    result = normalize_motion_command(
        "move up and down 5 cm",
        qwen_semantic=_clarification_payload(direction="conflicting", distance_quality="explicit", reason="E_CONFLICTING_DIRECTIONS"),
    )

    assert result["parse_status"] == STATUS_NEEDS_CLARIFICATION
    assert result["clarification_reason"] == "E_CONFLICTING_DIRECTIONS"
    assert result["qwen_semantic_parse_used"] is True


def test_low_confidence_qwen_semantic_falls_back_to_rule_parse():
    result = normalize_motion_command(
        "move up 5 mm",
        qwen_semantic=_semantic_payload(direction="down", quality="explicit", meters=0.005, confidence=0.5),
    )

    assert result["parse_status"] == STATUS_PASS
    assert result["direction_axis"] == "z"
    assert result["direction_sign"] == "+"
    assert result["qwen_semantic_parse_used"] is False
    assert result["fallback_parse_used"] is True
    assert result["canonicalization_source"] == "fallback_rule"


def test_invalid_qwen_schema_falls_back_to_rule_parse_with_evidence():
    result = normalize_motion_command("move up 5 mm", qwen_semantic={"unexpected": "shape"})

    assert result["parse_status"] == STATUS_PASS
    assert result["direction_axis"] == "z"
    assert result["direction_sign"] == "+"
    assert result["qwen_semantic_parse_used"] is False
    assert result["fallback_parse_used"] is True
    assert result["qwen_fallback_conflict"] is False
    assert result["qwen_fallback_conflict_reason"] == "E_QWEN_SCHEMA_UNRECOGNIZED"


def test_qwen_semantic_long_relative_motion_keeps_safety_gate_handoff():
    result = normalize_motion_command(
        "move forward 20 cm",
        qwen_semantic=_semantic_payload(direction="forward", quality="explicit", meters=0.2, value=20, unit="cm"),
    )

    assert result["parse_status"] == STATUS_PASS
    assert result["direction_axis"] == "x"
    assert result["direction_sign"] == "+"
    assert result["requested_distance_m"] == 0.2
    assert result["execution_permission_decided_by_parser"] is False
    assert result["safety_gate_still_required"] is True


def _semantic_payload(*, direction, quality, meters, value=None, unit="m", language="en", confidence=0.95):
    return {
        "schema_version": "teto_motion_semantics.v1",
        "intent_status": "ok",
        "intent_type": "relative_cartesian_motion",
        "motion": {
            "reference": "tcp",
            "direction_semantic": direction,
            "distance": {"value": value, "unit": unit, "meters": meters, "quality": quality},
            "fuzzy_magnitude": "small" if quality == "fuzzy_small" else "unspecified",
            "frame_hint": "base_link",
        },
        "clarification": {"required": False, "reason": ""},
        "unsupported": {"reason": ""},
        "confidence": {"intent": confidence, "direction": confidence, "distance": confidence, "overall": confidence},
        "language": language,
        "notes": "test payload",
    }


def _unsupported_payload(intent_type, reason):
    payload = _semantic_payload(direction="unknown", quality="missing", meters=None)
    payload["intent_status"] = "unsupported"
    payload["intent_type"] = intent_type
    payload["unsupported"] = {"reason": reason}
    return payload


def _clarification_payload(*, direction, distance_quality, reason):
    payload = _semantic_payload(direction=direction, quality=distance_quality, meters=0.05, value=5, unit="cm")
    payload["intent_status"] = "needs_clarification"
    payload["intent_type"] = "unknown"
    payload["clarification"] = {"required": True, "reason": reason}
    return payload
