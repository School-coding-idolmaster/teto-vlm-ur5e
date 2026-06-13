import json

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


def test_exact_default_max_step_is_allowed():
    parsed = parse_motion_command("move up 5 mm", max_step_m=0.005)

    assert parsed.distance_m == 0.005
    assert parsed.delta_m == [0.0, 0.0, 0.005]


def test_hard_safety_limit_is_allowed_when_max_step_allows_it():
    parsed = parse_motion_command("move up 10 mm", max_step_m=0.01)

    assert parsed.distance_m == 0.01
    assert parsed.delta_m == [0.0, 0.0, 0.01]


def test_floating_point_tolerance_allows_limit_equivalent(monkeypatch):
    monkeypatch.setattr(cli, "_extract_distance_m", lambda _normalized: (0.005000000000000004, "mm"))

    parsed = parse_motion_command("move up 5 mm", max_step_m=0.005)

    assert parsed.delta_m == [0.0, 0.0, 0.005]


def test_greater_than_hard_safety_limit_is_blocked():
    with pytest.raises(MotionParseError) as exc:
        parse_motion_command("move up 10.001 mm", max_step_m=0.01)

    assert str(exc.value) == "E_EXCEEDS_HARD_SAFETY_LIMIT"


def test_rejects_motion_over_default_step_limit():
    with pytest.raises(MotionParseError) as exc:
        parse_motion_command("move up 6 mm")

    assert str(exc.value) == "E_EXCEEDS_MAX_STEP"


def test_cmd_parses_without_interactive_prompt(monkeypatch, capsys):
    monkeypatch.setattr(cli, "_lookup_current_tcp_pose", lambda timeout_s: _pose())
    monkeypatch.setattr(cli, "evaluate_qwen_motion_parser", _fake_qwen_success)
    monkeypatch.setattr("builtins.input", lambda _prompt: pytest.fail("input prompt should not be used"))

    exit_code = cli.main(["--dry-run", "--cmd", "raise the tcp by 5 millimeters"])

    output = capsys.readouterr().out
    assert exit_code == 0
    assert "Enter motion command:" not in output
    assert '"input_mode": "cmd"' in output
    assert '"original_user_text": "raise the tcp by 5 millimeters"' in output
    assert '"parser_source": "qwen_llm"' in output
    assert '"llm_called": true' in output
    assert '"final_status": "PASS"' in output
    assert '"real_robot_motion_executed": false' in output


def test_manual_qwen_path_reads_stdin_and_preserves_text(monkeypatch, capsys):
    prompts = []
    monkeypatch.setattr(cli, "_lookup_current_tcp_pose", lambda timeout_s: _pose())
    monkeypatch.setattr(cli, "evaluate_qwen_motion_parser", _fake_qwen_success)
    monkeypatch.setattr("builtins.input", lambda prompt: prompts.append(prompt) or "raise the tcp by 5 millimeters")

    exit_code = cli.main(["--dry-run"])

    output = capsys.readouterr().out
    assert exit_code == 0
    assert prompts == ["Enter motion command: "]
    assert '"input_mode": "manual"' in output
    assert '"original_user_text": "raise the tcp by 5 millimeters"' in output
    assert '"parser_mode": "qwen"' in output
    assert '"parser_source": "qwen_llm"' in output
    assert '"llm_called": true' in output
    assert '"delta_m": [\n      0.0,\n      0.0,\n      0.005\n    ]' in output
    evidence = _final_evidence(output)
    assert evidence["execution_preview"] == {
        "original_command": "raise the tcp by 5 millimeters",
        "input_mode": "manual",
        "parser_mode": "qwen",
        "parser_source": "qwen_llm",
        "llm_called": True,
        "model_name": "qwen2.5vl:3b",
        "endpoint": None,
        "intent": "relative_cartesian_motion",
        "frame": "base_link",
        "axis": "z",
        "direction": "+",
        "distance_m": 0.005,
        "delta_m": [0.0, 0.0, 0.005],
        "max_distance_m": 0.005,
        "hard_safety_limit_m": 0.01,
        "within_safety_limit": True,
        "dry_run": True,
        "real_robot_motion_requested": False,
        "manual_confirmation_required": True,
        "preview_status": "PASS",
    }
    planner = evidence["planner_acceptance"]
    assert planner["status"] == "PASS"
    assert planner["requested_distance_m"] == 0.005
    assert planner["requested_delta_m"] == [0.0, 0.0, 0.005]
    assert planner["plan_only"] is True
    assert planner["execution_allowed"] is False
    assert planner["trajectory_sent"] is False
    assert planner["execute_trajectory_called"] is False
    assert planner["planned_goal_frame"] == "base_link"
    assert planner["reasonableness_check"] == "PASS"
    assert evidence["real_robot_motion_executed"] is False


def test_manual_qwen_down_command_prints_negative_z_preview(monkeypatch, capsys):
    monkeypatch.setattr(cli, "_lookup_current_tcp_pose", lambda timeout_s: _pose())
    monkeypatch.setattr(cli, "evaluate_qwen_motion_parser", _fake_qwen_down_success)
    monkeypatch.setattr("builtins.input", lambda _prompt: "tcp down 2mm")

    exit_code = cli.main(["--dry-run"])

    evidence = _final_evidence(capsys.readouterr().out)
    preview = evidence["execution_preview"]
    assert exit_code == 0
    assert preview["original_command"] == "tcp down 2mm"
    assert preview["axis"] == "z"
    assert preview["direction"] == "-"
    assert preview["distance_m"] == 0.002
    assert preview["delta_m"] == [0.0, 0.0, -0.002]
    assert preview["within_safety_limit"] is True
    assert preview["dry_run"] is True
    assert preview["real_robot_motion_requested"] is False
    planner = evidence["planner_acceptance"]
    assert planner["status"] == "PASS"
    assert planner["requested_delta_m"] == [0.0, 0.0, -0.002]
    assert planner["requested_distance_m"] == 0.002
    assert planner["plan_only"] is True
    assert planner["execution_allowed"] is False
    assert planner["trajectory_sent"] is False
    assert planner["execute_trajectory_called"] is False


def test_suspicious_mocked_joint_trajectory_is_exposed_in_planner_acceptance(monkeypatch, capsys):
    real_gateway = cli.evaluate_cartesian_motion_gateway

    def fake_gateway(request):
        result = real_gateway(request)
        result["moveit_pose_executor_result"] = {
            "trajectory_point_count": 2,
            "joint_trajectory_points": [
                {"positions": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]},
                {"positions": [0.0, 1.25, 0.0, 0.0, 0.0, 0.0]},
            ],
            "trajectory_sent": False,
            "moveit_execute_called": False,
        }
        return result

    monkeypatch.setattr(cli, "_lookup_current_tcp_pose", lambda timeout_s: _pose())
    monkeypatch.setattr(cli, "evaluate_qwen_motion_parser", _fake_qwen_success)
    monkeypatch.setattr(cli, "evaluate_cartesian_motion_gateway", fake_gateway)
    monkeypatch.setattr("builtins.input", lambda _prompt: "raise the tcp by 5 millimeters")

    exit_code = cli.main(["--dry-run"])

    evidence = _final_evidence(capsys.readouterr().out)
    planner = evidence["planner_acceptance"]
    assert exit_code == 0
    assert planner["status"] == "WARNING"
    assert planner["reasonableness_check"] == "WARNING"
    assert planner["planned_waypoint_count"] == 2
    assert planner["max_joint_delta_rad"] == 1.25
    assert "W_SUSPICIOUS_JOINT_DELTA_FOR_TINY_CARTESIAN_MOTION" in planner["warnings"]
    assert planner["execution_allowed"] is False
    assert planner["trajectory_sent"] is False
    assert planner["execute_trajectory_called"] is False
    assert evidence["real_robot_motion_executed"] is False


def test_unsafe_qwen_distance_blocks_before_execution_preview(monkeypatch, capsys):
    monkeypatch.setattr(cli, "_lookup_current_tcp_pose", lambda timeout_s: pytest.fail("pose lookup should not run"))
    monkeypatch.setattr(cli, "evaluate_qwen_motion_parser", _fake_qwen_unsafe_distance)
    monkeypatch.setattr("builtins.input", lambda _prompt: "raise the tcp by 20 millimeters")

    exit_code = cli.main(["--dry-run"])

    evidence = _final_evidence(capsys.readouterr().out)
    assert exit_code == 2
    assert evidence["final_status"] != "PASS"
    assert evidence.get("execution_preview") is None
    assert evidence["planner_acceptance"] is None
    assert "E_EXCEEDS_HARD_SAFETY_LIMIT" in evidence["blocking_reasons"]
    assert evidence["real_robot_motion_executed"] is False


def test_dry_run_with_cmd_works(monkeypatch, capsys):
    monkeypatch.setattr(cli, "_lookup_current_tcp_pose", lambda timeout_s: _pose())

    exit_code = cli.main(["--parser", "rule", "--cmd", "move left 5 mm"])

    output = capsys.readouterr().out
    assert exit_code == 0
    assert '"dry_run": true' in output
    assert '"parser_source": "rule_based"' in output
    assert '"llm_called": false' in output
    assert '"final_status": "PASS"' in output
    assert '"motion_check_source": "moveit_pose_executor"' in output
    assert '"motion_check_distance_m": 0.005' in output


def test_real_confirmation_accepts_exact_lowercase_y(monkeypatch, capsys):
    monkeypatch.setattr(cli, "_lookup_current_tcp_pose", lambda timeout_s: _pose())
    monkeypatch.setattr(cli, "evaluate_qwen_motion_parser", _fake_qwen_success)
    monkeypatch.setattr(cli, "_execution_prereq_blockers", lambda timeout_s: [])
    monkeypatch.setattr(cli, "evaluate_cartesian_motion_execution", _fake_success_execution)
    monkeypatch.setattr("builtins.input", lambda prompt: "y")

    exit_code = cli.main(["--real", "--cmd", "raise the tcp by 5 millimeters"])

    output = capsys.readouterr().out
    assert exit_code == 0
    assert "Execute on real UR5e? Type y to continue:" not in output
    assert '"llm_called": true' in output
    assert '"parser_source": "qwen_llm"' in output
    assert '"final_status": "PASS"' in output
    assert '"real_robot_motion_executed": true' in output
    assert '"moveit_execute_error_code": 1' in output


def test_confirmation_mismatch_aborts(monkeypatch, capsys):
    monkeypatch.setattr(cli, "_lookup_current_tcp_pose", lambda timeout_s: _pose())
    monkeypatch.setattr(cli, "evaluate_qwen_motion_parser", _fake_qwen_success)
    monkeypatch.setattr(cli, "_execution_prereq_blockers", lambda timeout_s: pytest.fail("prereq check should not run"))
    monkeypatch.setattr(cli, "evaluate_cartesian_motion_execution", lambda request: pytest.fail("execution should not run"))
    monkeypatch.setattr("builtins.input", lambda prompt: "Y")

    exit_code = cli.main(["--real", "--cmd", "raise the tcp by 5 millimeters"])

    output = capsys.readouterr().out
    assert exit_code == 2
    assert '"E_CONFIRMATION_MISMATCH"' in output
    assert '"real_robot_motion_executed": false' in output


def test_real_yes_with_cmd_is_allowed(monkeypatch, capsys):
    monkeypatch.setattr(cli, "_lookup_current_tcp_pose", lambda timeout_s: _pose())
    monkeypatch.setattr(cli, "evaluate_qwen_motion_parser", _fake_qwen_success)
    monkeypatch.setattr(cli, "_execution_prereq_blockers", lambda timeout_s: [])
    monkeypatch.setattr(cli, "evaluate_cartesian_motion_execution", _fake_success_execution)
    monkeypatch.setattr("builtins.input", lambda _prompt: pytest.fail("confirmation prompt should not be used"))

    exit_code = cli.main(["--real", "--yes", "--cmd", "raise the tcp by 5 millimeters"])

    output = capsys.readouterr().out
    assert exit_code == 0
    assert '"llm_called": true' in output
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


def _fake_qwen_success(request):
    raw = json.dumps(
        {
            "intent": "relative_cartesian_motion",
            "axis": "z",
            "direction": "+",
            "distance_m": 0.005,
            "confidence": 0.95,
            "reason": "small upward relative motion",
        }
    )
    return {
        "qwen_motion_parser_status": "PASS",
        "parser_source": "qwen_llm",
        "llm_called": True,
        "model_name": request.model_name or "qwen2.5vl:3b",
        "qwen_endpoint": request.endpoint,
        "llm_latency_ms": 1.2,
        "raw_llm_output": raw,
        "normalized_contract": {
            "intent": "relative_cartesian_motion",
            "frame": "base_link",
            "delta_m": [0.0, 0.0, 0.005],
            "max_distance_m": request.max_distance_m,
            "hard_safety_limit_m": request.hard_safety_limit_m,
            "must_confirm": True,
        },
        "axis": "z",
        "direction": "+",
        "distance_m": 0.005,
        "confidence": 0.95,
        "delta_m": [0.0, 0.0, 0.005],
        "parser_blocking_reasons": [],
        "blocking_reasons": [],
    }


def _fake_qwen_down_success(request):
    raw = json.dumps(
        {
            "intent": "relative_cartesian_motion",
            "axis": "z",
            "direction": "-",
            "distance_m": 0.002,
            "confidence": 0.96,
            "reason": "small downward relative motion",
        }
    )
    return {
        "qwen_motion_parser_status": "PASS",
        "parser_source": "qwen_llm",
        "llm_called": True,
        "model_name": request.model_name or "qwen2.5vl:3b",
        "qwen_endpoint": request.endpoint,
        "llm_latency_ms": 1.1,
        "raw_llm_output": raw,
        "normalized_contract": {
            "intent": "relative_cartesian_motion",
            "frame": "base_link",
            "delta_m": [0.0, 0.0, -0.002],
            "max_distance_m": request.max_distance_m,
            "hard_safety_limit_m": request.hard_safety_limit_m,
            "must_confirm": True,
        },
        "axis": "z",
        "direction": "-",
        "distance_m": 0.002,
        "confidence": 0.96,
        "delta_m": [0.0, 0.0, -0.002],
        "parser_blocking_reasons": [],
        "blocking_reasons": [],
    }


def _fake_qwen_unsafe_distance(request):
    return {
        "qwen_motion_parser_status": "BLOCKED",
        "parser_source": "qwen_llm",
        "llm_called": True,
        "model_name": request.model_name or "qwen2.5vl:3b",
        "qwen_endpoint": request.endpoint,
        "llm_latency_ms": 1.0,
        "raw_llm_output": json.dumps(
            {
                "intent": "relative_cartesian_motion",
                "axis": "z",
                "direction": "+",
                "distance_m": 0.02,
                "confidence": 0.95,
            }
        ),
        "normalized_contract": None,
        "parser_blocking_reasons": ["E_EXCEEDS_HARD_SAFETY_LIMIT"],
        "blocking_reasons": ["E_EXCEEDS_HARD_SAFETY_LIMIT"],
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


def _final_evidence(output: str):
    marker = "Final execution evidence:\n"
    assert marker in output
    return json.loads(output.rsplit(marker, 1)[1])
