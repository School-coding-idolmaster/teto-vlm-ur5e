import pytest

from scripts.text_to_ur5e_real_motion import MotionParseError, parse_motion_command


@pytest.mark.parametrize(
    ("command", "delta"),
    [
        ("move up 5 mm", [0.0, 0.0, 0.005]),
        ("move down 5 mm", [0.0, 0.0, -0.005]),
        ("move left 5 mm", [0.0, 0.005, 0.0]),
        ("move right 5 mm", [0.0, -0.005, 0.0]),
        ("move forward 5 mm", [0.005, 0.0, 0.0]),
        ("move backward 5 mm", [-0.005, 0.0, 0.0]),
        ("raise 5 millimeters", [0.0, 0.0, 0.005]),
        ("lower 5 millimeter", [0.0, 0.0, -0.005]),
        ("move +z 5 mm", [0.0, 0.0, 0.005]),
        ("move -z 5 mm", [0.0, 0.0, -0.005]),
        ("move +x 5 mm", [0.005, 0.0, 0.0]),
        ("move -y 5 mm", [0.0, -0.005, 0.0]),
    ],
)
def test_valid_restricted_relative_commands(command, delta):
    parsed = parse_motion_command(command)

    assert parsed.delta_m == delta
    assert parsed.frame == "base_link"
    assert parsed.task_contract(real_robot_motion_requested=True, dry_run=False)["intent"] == "relative_cartesian_motion"
    assert parsed.gateway_task()["task_contract"]["intent"] == "cartesian_offset"


@pytest.mark.parametrize(
    "command",
    [
        "hover above red mug",
        "move above the red mug",
        "run URScript movel upward",
        "move shoulder joint 1 degree",
        "send RTDE write",
        "dashboard play",
    ],
)
def test_rejects_vision_and_direct_robot_control_commands(command):
    with pytest.raises(MotionParseError):
        parse_motion_command(command)


def test_rejects_motion_over_hard_safety_limit():
    with pytest.raises(MotionParseError) as exc:
        parse_motion_command("move up 20 mm", max_step_m=0.01)

    assert str(exc.value) == "E_EXCEEDS_HARD_SAFETY_LIMIT"


def test_rejects_motion_over_default_step_limit():
    with pytest.raises(MotionParseError) as exc:
        parse_motion_command("move up 6 mm")

    assert str(exc.value) == "E_EXCEEDS_MAX_STEP"
