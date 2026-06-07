import pytest

from scripts import text_to_ur5e_real_motion as cli
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


def test_cmd_parses_without_interactive_prompt(monkeypatch, capsys):
    monkeypatch.setattr(cli, "_lookup_current_tcp_pose", lambda timeout_s: _pose())
    monkeypatch.setattr("builtins.input", lambda _prompt: pytest.fail("input prompt should not be used"))

    exit_code = cli.main(["--dry-run", "--cmd", "move up 5 mm"])

    output = capsys.readouterr().out
    assert exit_code == 0
    assert "Enter motion command:" not in output
    assert '"final_status": "PASS"' in output
    assert '"real_robot_motion_executed": false' in output


def test_dry_run_with_cmd_works(monkeypatch, capsys):
    monkeypatch.setattr(cli, "_lookup_current_tcp_pose", lambda timeout_s: _pose())

    exit_code = cli.main(["--cmd", "move left 5 mm"])

    output = capsys.readouterr().out
    assert exit_code == 0
    assert '"dry_run": true' in output
    assert '"final_status": "PASS"' in output


def test_real_confirmation_accepts_exact_lowercase_y(monkeypatch, capsys):
    monkeypatch.setattr(cli, "_lookup_current_tcp_pose", lambda timeout_s: _pose())
    monkeypatch.setattr(cli, "_execution_prereq_blockers", lambda timeout_s: [])
    monkeypatch.setattr(cli, "evaluate_cartesian_motion_execution", _fake_success_execution)
    monkeypatch.setattr("builtins.input", lambda prompt: "y")

    exit_code = cli.main(["--real", "--cmd", "move up 5 mm"])

    output = capsys.readouterr().out
    assert exit_code == 0
    assert "Execute on real UR5e? Type y to continue:" not in output
    assert '"final_status": "PASS"' in output
    assert '"real_robot_motion_executed": true' in output
    assert '"moveit_execute_error_code": 1' in output


def test_confirmation_mismatch_aborts(monkeypatch, capsys):
    monkeypatch.setattr(cli, "_lookup_current_tcp_pose", lambda timeout_s: _pose())
    monkeypatch.setattr(cli, "_execution_prereq_blockers", lambda timeout_s: pytest.fail("prereq check should not run"))
    monkeypatch.setattr(cli, "evaluate_cartesian_motion_execution", lambda request: pytest.fail("execution should not run"))
    monkeypatch.setattr("builtins.input", lambda prompt: "Y")

    exit_code = cli.main(["--real", "--cmd", "move up 5 mm"])

    output = capsys.readouterr().out
    assert exit_code == 2
    assert '"E_CONFIRMATION_MISMATCH"' in output
    assert '"real_robot_motion_executed": false' in output


def test_real_yes_with_cmd_is_allowed(monkeypatch, capsys):
    monkeypatch.setattr(cli, "_lookup_current_tcp_pose", lambda timeout_s: _pose())
    monkeypatch.setattr(cli, "_execution_prereq_blockers", lambda timeout_s: [])
    monkeypatch.setattr(cli, "evaluate_cartesian_motion_execution", _fake_success_execution)
    monkeypatch.setattr("builtins.input", lambda _prompt: pytest.fail("confirmation prompt should not be used"))

    exit_code = cli.main(["--real", "--yes", "--cmd", "move up 5 mm"])

    output = capsys.readouterr().out
    assert exit_code == 0
    assert '"final_status": "PASS"' in output
    assert '"real_robot_motion_executed": true' in output


def test_real_yes_without_cmd_is_rejected(monkeypatch, capsys):
    monkeypatch.setattr("builtins.input", lambda _prompt: pytest.fail("command prompt should not be used"))

    exit_code = cli.main(["--real", "--yes"])

    output = capsys.readouterr().out
    assert exit_code == 2
    assert '"E_YES_REQUIRES_CMD"' in output
    assert '"real_robot_motion_executed": false' in output


def _pose():
    return {
        "frame": "base_link",
        "position_m": [0.4, 0.0, 0.3],
        "orientation_xyzw": [0.0, 0.0, 0.0, 1.0],
    }


def _fake_success_execution(request):
    return {
        "cartesian_motion_execution_status": "PASS",
        "trajectory_sent": True,
        "controller_command_sent": True,
        "real_robot_motion_executed": True,
        "blocking_reasons": [],
        "warnings": [],
        "moveit_pose_executor_result": {
            "goal_accepted": True,
            "execute_success": True,
            "moveit_execute_error_code": 1,
            "moveit_execute_error_code_name": "SUCCESS",
        },
    }
