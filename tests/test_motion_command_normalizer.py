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
