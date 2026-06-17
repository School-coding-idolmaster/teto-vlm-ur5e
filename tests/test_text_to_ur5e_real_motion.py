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
        parse_motion_command("move up 20 mm", max_distance_m=0.01, hard_safety_limit_m=0.01)

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
        parse_motion_command("move up 10.001 mm", max_distance_m=0.01, hard_safety_limit_m=0.01)

    assert str(exc.value) == "E_EXCEEDS_HARD_SAFETY_LIMIT"


def test_default_lab_policy_allows_five_centimeters():
    parsed = parse_motion_command("move up 50 mm")

    assert parsed.distance_m == 0.05
    assert parsed.max_distance_m == 0.05
    assert parsed.hard_safety_limit_m == 0.05


def test_rejects_motion_over_default_distance_limit():
    with pytest.raises(MotionParseError) as exc:
        parse_motion_command("move up 51 mm", hard_safety_limit_m=0.1)

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
        "max_distance_m": 0.05,
        "hard_safety_limit_m": 0.05,
        "within_safety_limit": True,
        "dry_run": True,
        "real_robot_motion_requested": False,
        "manual_confirmation_required": True,
        "preview_status": "PASS",
    }
    planner = evidence["planner_acceptance"]
    assert planner["status"] == "PASS"
    assert planner["requested_distance_m"] == 0.005
    assert planner["configured_max_distance_m"] == 0.05
    assert planner["hard_safety_limit_m"] == 0.05
    assert planner["requested_distance_within_configured_limit"] is True
    assert planner["safety_policy_name"] == "lab_directional_step_motion_v1"
    assert planner["safety_policy_source"] == "cli_defaults"
    assert planner["motion_frame"] == "base_link"
    assert planner["direction_axis"] == "z"
    assert planner["direction_sign"] == "+"
    assert planner["base_link_direction_mapping"]["forward"]["base_link_direction"] == "+X"
    assert planner["base_link_direction_mapping"]["backward"]["base_link_direction"] == "-X"
    assert planner["base_link_direction_mapping"]["left"]["base_link_direction"] == "+Y"
    assert planner["base_link_direction_mapping"]["right"]["base_link_direction"] == "-Y"
    assert planner["base_link_direction_mapping"]["up"]["base_link_direction"] == "+Z"
    assert planner["base_link_direction_mapping"]["down"]["base_link_direction"] == "-Z"
    assert planner["requested_delta_m"] == [0.0, 0.0, 0.005]
    assert planner["plan_only"] is True
    assert planner["execution_allowed"] is False
    assert planner["trajectory_sent"] is False
    assert planner["execute_trajectory_called"] is False
    assert planner["planned_goal_frame"] == "base_link"
    assert planner["metrics_source"] == "not_available"
    assert planner["planned_waypoint_count"] is None
    assert planner["max_joint_delta_rad"] is None
    assert planner["total_joint_motion_rad"] is None
    assert planner["estimated_cartesian_path_length_m"] is None
    assert planner["orientation_change_rad"] is None
    assert planner["trajectory_duration_s"] is None
    assert planner["reasonableness_check"] == "PASS"
    assert evidence["real_robot_motion_executed"] is False


def test_mocked_normal_plan_only_metrics_are_reported(monkeypatch, capsys):
    monkeypatch.setattr(cli, "_lookup_current_tcp_pose", lambda timeout_s: _pose())
    monkeypatch.setattr(cli, "evaluate_qwen_motion_parser", _fake_qwen_success)
    monkeypatch.setattr(cli, "evaluate_cartesian_motion_gateway", _gateway_with_plan_metrics(_normal_up_plan_metrics()))
    monkeypatch.setattr("builtins.input", lambda _prompt: "raise the tcp by 5 millimeters")

    exit_code = cli.main(["--dry-run"])

    evidence = _final_evidence(capsys.readouterr().out)
    planner = evidence["planner_acceptance"]
    assert exit_code == 0
    assert planner["status"] == "PASS"
    assert planner["plan_only"] is True
    assert planner["execution_allowed"] is False
    assert planner["trajectory_sent"] is False
    assert planner["execute_trajectory_called"] is False
    assert planner["metrics_source"] == "mock_plan_only"
    assert planner["planned_waypoint_count"] == 3
    assert planner["max_joint_delta_rad"] == 0.03
    assert planner["total_joint_motion_rad"] == 0.06
    assert planner["estimated_cartesian_path_length_m"] == 0.005
    assert planner["orientation_change_rad"] == 0.0
    assert planner["trajectory_duration_s"] == 1.2
    assert planner["reasonableness_check"] == "PASS"


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


def test_mocked_downward_plan_only_metrics_are_reported(monkeypatch, capsys):
    monkeypatch.setattr(cli, "_lookup_current_tcp_pose", lambda timeout_s: _pose())
    monkeypatch.setattr(cli, "evaluate_qwen_motion_parser", _fake_qwen_down_success)
    monkeypatch.setattr(cli, "evaluate_cartesian_motion_gateway", _gateway_with_plan_metrics(_normal_down_plan_metrics()))
    monkeypatch.setattr("builtins.input", lambda _prompt: "tcp down 2mm")

    exit_code = cli.main(["--dry-run"])

    evidence = _final_evidence(capsys.readouterr().out)
    planner = evidence["planner_acceptance"]
    assert exit_code == 0
    assert planner["requested_delta_m"] == [0.0, 0.0, -0.002]
    assert planner["requested_distance_m"] == 0.002
    assert planner["metrics_source"] == "mock_plan_only"
    assert planner["planned_waypoint_count"] == 2
    assert planner["max_joint_delta_rad"] == 0.01
    assert planner["total_joint_motion_rad"] == 0.015
    assert planner["estimated_cartesian_path_length_m"] == 0.002
    assert planner["trajectory_duration_s"] == 0.4
    assert planner["execution_allowed"] is False
    assert planner["trajectory_sent"] is False
    assert planner["execute_trajectory_called"] is False


def test_suspicious_mocked_joint_trajectory_is_exposed_in_planner_acceptance(monkeypatch, capsys):
    monkeypatch.setattr(cli, "_lookup_current_tcp_pose", lambda timeout_s: _pose())
    monkeypatch.setattr(cli, "evaluate_qwen_motion_parser", _fake_qwen_success)
    monkeypatch.setattr(cli, "evaluate_cartesian_motion_gateway", _gateway_with_plan_metrics(_suspicious_plan_metrics()))
    monkeypatch.setattr("builtins.input", lambda _prompt: "raise the tcp by 5 millimeters")

    exit_code = cli.main(["--dry-run"])

    evidence = _final_evidence(capsys.readouterr().out)
    planner = evidence["planner_acceptance"]
    assert exit_code == 0
    assert planner["status"] == "WARNING"
    assert planner["reasonableness_check"] == "WARNING"
    assert planner["planned_waypoint_count"] == 2
    assert planner["max_joint_delta_rad"] == 1.25
    assert planner["total_joint_motion_rad"] == 1.25
    assert "W_SUSPICIOUS_JOINT_DELTA_FOR_TINY_CARTESIAN_MOTION" in planner["warnings"]
    assert planner["execution_allowed"] is False
    assert planner["trajectory_sent"] is False
    assert planner["execute_trajectory_called"] is False
    assert evidence["real_robot_motion_executed"] is False


def test_moveit_plan_only_trajectory_object_metrics_are_reported():
    trajectory = _trajectory_object(
        [
            ([0.0, 0.0, 0.0, 0.0, 0.0, 0.0], {"sec": 0, "nanosec": 0}),
            ([0.0, 0.04, 0.02, 0.0, 0.0, 0.0], {"sec": 0, "nanosec": 700000000}),
        ]
    )

    planner = cli._planner_acceptance(
        execution_preview={
            "delta_m": [0.0, 0.0, 0.005],
            "distance_m": 0.005,
            "hard_safety_limit_m": 0.01,
            "frame": "base_link",
        },
        motion={"cartesian_motion_gateway_status": "PASS", "translation_distance_m": 0.005, "frame": "base_link"},
        execution={
            "moveit_pose_executor_result": {
                "planned_trajectory": trajectory,
                "moveit_plan_called": True,
                "trajectory_sent": False,
                "moveit_execute_called": False,
            }
        },
        dry_run=True,
    )

    assert planner["status"] == "PASS"
    assert planner["metrics_source"] == "moveit_plan_only"
    assert planner["planned_waypoint_count"] == 2
    assert planner["max_joint_delta_rad"] == 0.04
    assert planner["total_joint_motion_rad"] == 0.06
    assert planner["trajectory_duration_s"] == 0.7
    assert planner["execution_allowed"] is False
    assert planner["trajectory_sent"] is False
    assert planner["execute_trajectory_called"] is False


def test_plan_only_smoke_path_extracts_metrics_without_execute(monkeypatch, capsys):
    captured_requests = []

    def fake_execution(request):
        captured_requests.append(request)
        assert request.config["enable_moveit_execute"] is False
        assert request.config["enable_real_robot_motion"] is False
        assert request.config["trajectory_send_allowed"] is False
        return {
            "cartesian_motion_execution_status": "PASS",
            "moveit_plan_requested": True,
            "moveit_plan_success": True,
            "manual_confirmation_required": True,
            "manual_confirmation_accepted": False,
            "moveit_execute_called": False,
            "trajectory_send_allowed": False,
            "trajectory_sent": False,
            "controller_command_sent": False,
            "real_robot_motion_executed": False,
            "moveit_pose_executor_result": {
                "moveit_pose_executor_status": "PASS",
                "plan_success": True,
                "moveit_plan_called": True,
                "moveit_execute_called": False,
                "trajectory_send_allowed": False,
                "trajectory_sent": False,
                "controller_command_sent": False,
                "real_robot_motion_executed": False,
                "planned_trajectory": _trajectory_object(
                    [
                        ([0.0, 0.0, 0.0, 0.0, 0.0, 0.0], {"sec": 0, "nanosec": 0}),
                        ([0.0, 0.03, 0.01, 0.0, 0.0, 0.0], {"sec": 1, "nanosec": 100000000}),
                    ]
                ),
                "trajectory_point_count": 2,
                "blocking_reasons": [],
                "warnings": [],
            },
            "blocking_reasons": [],
            "warnings": [],
        }

    monkeypatch.setattr(cli, "_lookup_current_tcp_pose", lambda timeout_s: _pose())
    monkeypatch.setattr(cli, "evaluate_qwen_motion_parser", _fake_qwen_success)
    monkeypatch.setattr(cli, "evaluate_cartesian_motion_execution", fake_execution)
    monkeypatch.setattr(cli, "_execution_prereq_blockers", lambda timeout_s: pytest.fail("execute prereq path should not run"))

    exit_code = cli.main(["--plan-only-smoke", "--cmd", "raise the tcp by 5 millimeters"])

    evidence = _final_evidence(capsys.readouterr().out)
    planner = evidence["planner_acceptance"]
    assert exit_code == 0
    assert captured_requests
    assert planner["status"] == "PASS"
    assert planner["plan_only"] is True
    assert planner["execution_allowed"] is False
    assert planner["trajectory_sent"] is False
    assert planner["controller_command_sent"] is False
    assert planner["execute_trajectory_called"] is False
    assert planner["real_robot_motion_executed"] is False
    assert planner["metrics_source"] == "moveit_plan_only"
    assert planner["planned_waypoint_count"] == 2
    assert planner["max_joint_delta_rad"] == 0.03
    assert planner["total_joint_motion_rad"] == 0.04
    assert planner["trajectory_duration_s"] == 1.1
    assert evidence["trajectory_sent"] is False
    assert evidence["controller_command_sent"] is False
    assert evidence["real_robot_motion_executed"] is False


def test_plan_only_smoke_unavailable_remains_safe(monkeypatch, capsys):
    def fake_execution(request):
        assert request.config["enable_moveit_execute"] is False
        assert request.config["enable_real_robot_motion"] is False
        return {
            "cartesian_motion_execution_status": "BLOCKED",
            "moveit_plan_requested": True,
            "moveit_plan_success": False,
            "moveit_execute_called": False,
            "trajectory_send_allowed": False,
            "trajectory_sent": False,
            "controller_command_sent": False,
            "real_robot_motion_executed": False,
            "moveit_pose_executor_result": {
                "moveit_pose_executor_status": "BLOCKED",
                "plan_success": False,
                "moveit_plan_called": False,
                "moveit_execute_called": False,
                "trajectory_send_allowed": False,
                "trajectory_sent": False,
                "controller_command_sent": False,
                "real_robot_motion_executed": False,
                "blocking_reasons": ["E_MOVEIT_ACTION_SERVER_UNAVAILABLE"],
                "warnings": [],
            },
            "blocking_reasons": ["E_MOVEIT_ACTION_SERVER_UNAVAILABLE"],
            "warnings": [],
        }

    monkeypatch.setattr(cli, "_lookup_current_tcp_pose", lambda timeout_s: _pose())
    monkeypatch.setattr(cli, "evaluate_qwen_motion_parser", _fake_qwen_success)
    monkeypatch.setattr(cli, "evaluate_cartesian_motion_execution", fake_execution)

    exit_code = cli.main(["--plan-only-smoke", "--cmd", "raise the tcp by 5 millimeters"])

    evidence = _final_evidence(capsys.readouterr().out)
    planner = evidence["planner_acceptance"]
    assert exit_code == 2
    assert planner["status"] == "BLOCKED"
    assert planner["metrics_source"] == "not_available"
    assert planner["planned_waypoint_count"] is None
    assert "E_MOVEIT_ACTION_SERVER_UNAVAILABLE" in planner["blocking_reasons"]
    assert planner["execution_allowed"] is False
    assert planner["trajectory_sent"] is False
    assert planner["controller_command_sent"] is False
    assert planner["execute_trajectory_called"] is False
    assert planner["real_robot_motion_executed"] is False
    assert evidence["real_robot_motion_executed"] is False


def test_plan_only_smoke_rejects_real_flag(capsys):
    exit_code = cli.main(["--real", "--plan-only-smoke", "--cmd", "raise the tcp by 5 millimeters"])

    output = capsys.readouterr().out
    assert exit_code == 2
    assert "E_REAL_AND_PLAN_ONLY_SMOKE_CONFLICT" in output
    assert '"real_robot_motion_executed": false' in output


def test_acceptance_dry_run_two_mm_up_has_workflow_evidence(monkeypatch, capsys):
    monkeypatch.setattr(cli, "_lookup_current_tcp_pose", lambda timeout_s: _pose())
    monkeypatch.setattr(cli, "evaluate_qwen_motion_parser", _fake_qwen_two_mm_up_success)

    exit_code = cli.main(["--acceptance", "--dry-run", "--cmd", "raise the tcp by 2 millimeters"])

    evidence = _final_evidence(capsys.readouterr().out)
    workflow = evidence["acceptance_workflow"]
    assert exit_code == 0
    assert workflow["status"] == "PASS"
    assert workflow["mode"] == "dry_run"
    assert workflow["qwen_parser_used"] is True
    assert workflow["execution_preview_status"] == "PASS"
    assert workflow["planner_acceptance_status"] == "PASS"
    assert evidence["execution_preview"]["delta_m"] == [0.0, 0.0, 0.002]
    assert workflow["trajectory_sent"] is False
    assert workflow["execute_trajectory_called"] is False
    assert workflow["real_robot_motion_executed"] is False
    assert evidence["post_motion_verification_status"] == "NOT_RUN"
    assert evidence["post_motion_verification"]["reason"] == "real_robot_motion_executed=false"
    assert evidence["post_motion_verification"]["intended_delta_m"] == [0.0, 0.0, 0.002]
    assert evidence["post_motion_verification"]["tcp_pose_sample_settle_s"] is None
    assert evidence["post_motion_verification"]["tcp_pose_sample_attempts"] is None
    assert evidence["post_motion_verification"]["tcp_pose_stale_check_passed"] is None
    assert evidence["actual_displacement_m"] is None


def test_acceptance_dry_run_with_mock_current_tcp_pose_passes_without_real_state(monkeypatch, capsys):
    monkeypatch.setattr(cli, "_lookup_current_tcp_pose", lambda timeout_s: pytest.fail("mock pose should avoid real pose lookup"))
    monkeypatch.setattr(cli, "evaluate_qwen_motion_parser", _fake_qwen_two_mm_up_success)

    exit_code = cli.main(
        [
            "--acceptance",
            "--dry-run",
            "--mock-current-tcp-pose",
            "--cmd",
            "raise the tcp by 2 millimeters",
        ]
    )

    evidence = _final_evidence(capsys.readouterr().out)
    workflow = evidence["acceptance_workflow"]
    current_pose = evidence["current_tcp_pose"]
    assert exit_code == 0
    assert evidence["final_status"] == "PASS"
    assert workflow["status"] == "PASS"
    assert workflow["mode"] == "dry_run"
    assert evidence["execution_preview"]["preview_status"] == "PASS"
    assert evidence["execution_preview"]["axis"] == "z"
    assert evidence["execution_preview"]["direction"] == "+"
    assert evidence["execution_preview"]["distance_m"] == 0.002
    assert evidence["execution_preview"]["delta_m"] == [0.0, 0.0, 0.002]
    assert evidence["execution_preview"]["within_safety_limit"] is True
    assert current_pose["available"] is True
    assert current_pose["source"] == "mock_current_tcp_pose_for_dry_run_only"
    assert current_pose["allowed_for_real_execution"] is False
    assert evidence["target_pose"]["position_m"] == [-0.154964, 0.312309, 1.048042]
    assert evidence["trajectory_sent"] is False
    assert evidence["execute_trajectory_called"] is False
    assert evidence["controller_command_sent"] is False
    assert evidence["real_robot_motion_executed"] is False
    assert "E_CURRENT_TCP_POSE_MISSING" not in evidence["blocking_reasons"]


def test_acceptance_dry_run_without_current_tcp_pose_remains_blocked(monkeypatch, capsys):
    monkeypatch.setattr(cli, "_lookup_current_tcp_pose", lambda timeout_s: None)
    monkeypatch.setattr(cli, "evaluate_qwen_motion_parser", _fake_qwen_two_mm_up_success)

    exit_code = cli.main(["--acceptance", "--dry-run", "--cmd", "raise the tcp by 2 millimeters"])

    evidence = _final_evidence(capsys.readouterr().out)
    assert exit_code == 2
    assert evidence["final_status"] == "BLOCKED"
    assert evidence["current_tcp_pose"]["available"] is False
    assert evidence["current_tcp_pose"]["source"] == "current_tcp_pose_not_provided_or_available"
    assert evidence["current_tcp_pose"]["tcp_pose_readiness_status"] == "BLOCKED"
    assert evidence["current_tcp_pose"]["tcp_pose_base_frame"] == "base_link"
    assert evidence["current_tcp_pose"]["tcp_pose_tool_frame"] == "tool0"
    assert evidence["current_tcp_pose"]["tcp_pose_lookup_success"] is False
    assert evidence["current_tcp_pose"]["current_tcp_pose_blocking_reason"] == "E_CURRENT_TCP_POSE_MISSING"
    assert "E_CURRENT_TCP_POSE_MISSING" in evidence["blocking_reasons"]
    assert evidence["trajectory_sent"] is False
    assert evidence["controller_command_sent"] is False
    assert evidence["real_robot_motion_executed"] is False


def test_acceptance_dry_run_two_mm_down_has_workflow_evidence(monkeypatch, capsys):
    monkeypatch.setattr(cli, "_lookup_current_tcp_pose", lambda timeout_s: _pose())
    monkeypatch.setattr(cli, "evaluate_qwen_motion_parser", _fake_qwen_down_success)

    exit_code = cli.main(["--acceptance", "--dry-run", "--cmd", "tcp down 2mm"])

    evidence = _final_evidence(capsys.readouterr().out)
    workflow = evidence["acceptance_workflow"]
    assert exit_code == 0
    assert workflow["mode"] == "dry_run"
    assert evidence["execution_preview"]["delta_m"] == [0.0, 0.0, -0.002]
    assert evidence["execution_preview"]["preview_status"] == "PASS"
    assert workflow["real_execution_allowed"] is False
    assert workflow["real_robot_motion_executed"] is False


def test_tcp_pose_readiness_check_passes_with_mock_pose(monkeypatch, capsys):
    monkeypatch.setattr(cli, "_lookup_current_tcp_pose", lambda timeout_s: pytest.fail("mock readiness should avoid TF lookup"))

    exit_code = cli.main(["--check-tcp-pose-readiness", "--mock-current-tcp-pose"])

    evidence = json.loads(capsys.readouterr().out)
    assert exit_code == 0
    assert evidence["final_status"] == "PASS"
    assert evidence["tcp_pose_readiness_status"] == "PASS"
    assert evidence["current_tcp_pose"]["available"] is True
    assert evidence["current_tcp_pose"]["tcp_pose_available"] is True
    assert evidence["current_tcp_pose"]["tcp_pose_base_frame"] == "base_link"
    assert evidence["current_tcp_pose"]["tcp_pose_tool_frame"] == "tool0"
    assert evidence["manual_confirmation_required"] is False
    assert evidence["trajectory_sent"] is False
    assert evidence["execute_trajectory_called"] is False
    assert evidence["real_robot_motion_executed"] is False


def test_tcp_pose_readiness_check_fails_when_pose_unavailable(monkeypatch, capsys):
    monkeypatch.setattr(cli, "_lookup_current_tcp_pose", lambda timeout_s: None)

    exit_code = cli.main(["--check-tcp-pose-readiness"])

    evidence = json.loads(capsys.readouterr().out)
    assert exit_code == 2
    assert evidence["final_status"] == "BLOCKED"
    assert evidence["tcp_pose_readiness_status"] == "BLOCKED"
    assert "E_CURRENT_TCP_POSE_MISSING" in evidence["blocking_reasons"]
    assert evidence["current_tcp_pose"]["tcp_pose_lookup_success"] is False
    assert evidence["trajectory_sent"] is False
    assert evidence["execute_trajectory_called"] is False
    assert evidence["real_robot_motion_executed"] is False


def test_tcp_pose_readiness_check_records_configured_frames(capsys):
    pose = json.dumps(
        {
            "frame": "base",
            "position_m": [0.1, 0.2, 0.3],
            "orientation_xyzw": [0.0, 0.0, 0.0, 1.0],
        }
    )

    exit_code = cli.main(
        [
            "--check-tcp-pose-readiness",
            "--current-tcp-pose-json",
            pose,
            "--tcp-pose-base-frame",
            "base",
            "--tcp-pose-tool-frame",
            "tcp",
            "--moveit-planning-frame",
            "base",
            "--moveit-end-effector-link",
            "tcp",
        ]
    )

    evidence = json.loads(capsys.readouterr().out)
    assert exit_code == 0
    assert evidence["current_tcp_pose"]["tcp_pose_base_frame"] == "base"
    assert evidence["current_tcp_pose"]["tcp_pose_tool_frame"] == "tcp"
    assert evidence["moveit_planning_frame"] == "base"
    assert evidence["moveit_end_effector_link"] == "tcp"


def test_acceptance_plan_only_mode_has_workflow_evidence(monkeypatch, capsys):
    monkeypatch.setattr(cli, "_lookup_current_tcp_pose", lambda timeout_s: _pose())
    monkeypatch.setattr(cli, "evaluate_qwen_motion_parser", _fake_qwen_two_mm_up_success)
    monkeypatch.setattr(cli, "evaluate_cartesian_motion_execution", _fake_plan_only_execution)

    exit_code = cli.main(["--acceptance", "--plan-only-smoke", "--cmd", "raise the tcp by 2 millimeters"])

    evidence = _final_evidence(capsys.readouterr().out)
    workflow = evidence["acceptance_workflow"]
    assert exit_code == 0
    assert workflow["mode"] == "plan_only"
    assert evidence["planner_acceptance"]["status"] == "PASS"
    assert workflow["real_execution_allowed"] is False
    assert workflow["trajectory_sent"] is False
    assert workflow["execute_trajectory_called"] is False
    assert workflow["real_robot_motion_executed"] is False


def test_real_small_motion_acceptance_without_confirmation_blocks(monkeypatch, capsys):
    monkeypatch.setattr(cli, "_lookup_current_tcp_pose", lambda timeout_s: _pose())
    monkeypatch.setattr(cli, "evaluate_qwen_motion_parser", _fake_qwen_two_mm_up_success)
    monkeypatch.setattr(cli, "_execution_prereq_blockers", lambda timeout_s: pytest.fail("execution prereq should not run without confirmation"))
    monkeypatch.setattr("builtins.input", lambda _prompt: (_ for _ in ()).throw(EOFError))

    exit_code = cli.main(["--real-small-motion", "--cmd", "raise the tcp by 2 millimeters"])

    evidence = _final_evidence(capsys.readouterr().out)
    workflow = evidence["acceptance_workflow"]
    assert exit_code == 2
    assert workflow["status"] == "BLOCKED"
    assert workflow["mode"] == "real_small_motion"
    assert workflow["manual_confirmation_required"] is True
    assert workflow["manual_confirmation_received"] is False
    assert workflow["real_robot_motion_executed"] is False
    assert "E_CONFIRMATION_MISMATCH" in workflow["blocking_reasons"]
    _assert_real_small_two_mm_gate_allowed(evidence, direction="+")
    assert evidence["planner_acceptance"]["planner_acceptance_context"] == "plan_only_safety_audit"
    assert evidence["planner_acceptance"]["planner_acceptance_blocks_real_execution"] is False
    assert evidence["target_frame"] == "base_link"
    assert evidence["current_tcp_frame"] == "base_link"
    assert evidence["moveit_end_effector_link"] == "tool0"
    assert evidence["moveit_planning_frame"] == "base_link"
    assert evidence["moveit_group_name"] == "ur_manipulator"


def test_real_small_motion_lower_two_mm_allowed_until_confirmation(monkeypatch, capsys):
    monkeypatch.setattr(cli, "_lookup_current_tcp_pose", lambda timeout_s: _pose())
    monkeypatch.setattr(cli, "evaluate_qwen_motion_parser", _fake_qwen_down_success)
    monkeypatch.setattr(cli, "_execution_prereq_blockers", lambda timeout_s: pytest.fail("execution prereq should not run without confirmation"))
    monkeypatch.setattr("builtins.input", lambda _prompt: (_ for _ in ()).throw(EOFError))

    exit_code = cli.main(["--real-small-motion", "--cmd", "lower the tcp by 2 millimeters"])

    evidence = _final_evidence(capsys.readouterr().out)
    workflow = evidence["acceptance_workflow"]
    assert exit_code == 2
    assert workflow["status"] == "BLOCKED"
    assert workflow["mode"] == "real_small_motion"
    assert "E_CONFIRMATION_MISMATCH" in workflow["blocking_reasons"]
    assert "E_REAL_SMALL_MOTION_COMMAND_NOT_ALLOWED" not in evidence["blocking_reasons"]
    assert evidence["execution_preview"]["delta_m"] == [0.0, 0.0, -0.002]
    _assert_real_small_two_mm_gate_allowed(evidence, direction="-")
    assert evidence["trajectory_sent"] is False
    assert evidence["execute_trajectory_called"] is False
    assert evidence["controller_command_sent"] is False
    assert evidence["real_robot_motion_executed"] is False


def test_real_small_motion_lower_five_cm_allowed_until_confirmation(monkeypatch, capsys):
    monkeypatch.setattr(cli, "_lookup_current_tcp_pose", lambda timeout_s: _pose())
    monkeypatch.setattr(cli, "evaluate_qwen_motion_parser", _fake_qwen_down_five_cm_success)
    monkeypatch.setattr(cli, "_execution_prereq_blockers", lambda timeout_s: pytest.fail("execution prereq should not run without confirmation"))
    monkeypatch.setattr("builtins.input", lambda _prompt: (_ for _ in ()).throw(EOFError))

    exit_code = cli.main(["--real-small-motion", "--cmd", "lower the tcp by 5 centimeters"])

    evidence = _final_evidence(capsys.readouterr().out)
    assert exit_code == 2
    assert evidence["final_status"] == "BLOCKED"
    assert "E_REAL_SMALL_MOTION_COMMAND_NOT_ALLOWED" not in evidence["blocking_reasons"]
    assert "E_CONFIRMATION_MISMATCH" in evidence["blocking_reasons"]
    assert evidence["real_small_motion_command_allowed"] is True
    assert evidence["real_small_motion_gate_basis"] == "normalized_contract"
    assert evidence["real_small_motion_gate"]["normalized_distance_m"] == 0.05
    assert evidence["allowed_distance_m"] == 0.05
    assert evidence["configured_max_distance_m"] == 0.05
    assert evidence["requested_distance_within_configured_limit"] is True
    assert evidence["moveit_position_tolerance_m"] == 0.002
    assert evidence["tolerance_to_requested_distance_ratio"] == 0.04
    assert evidence["trajectory_sent"] is False
    assert evidence["execute_trajectory_called"] is False
    assert evidence["controller_command_sent"] is False
    assert evidence["real_robot_motion_executed"] is False


def test_real_small_motion_over_configured_distance_blocks_before_confirmation(monkeypatch, capsys):
    monkeypatch.setattr(cli, "_lookup_current_tcp_pose", lambda timeout_s: _pose())
    monkeypatch.setattr(cli, "evaluate_qwen_motion_parser", _fake_qwen_down_six_cm_success)
    monkeypatch.setattr(cli, "_execution_prereq_blockers", lambda timeout_s: pytest.fail("execution prereq should not run for blocked gate"))
    monkeypatch.setattr("builtins.input", lambda _prompt: pytest.fail("confirmation prompt should not be reached"))

    exit_code = cli.main(["--real-small-motion", "--cmd", "lower the tcp by 6 centimeters"])

    evidence = _final_evidence(capsys.readouterr().out)
    assert exit_code == 2
    assert evidence["final_status"] == "BLOCKED"
    assert "E_REAL_SMALL_MOTION_DISTANCE_NOT_ALLOWED" in evidence["blocking_reasons"]
    assert evidence["real_small_motion_command_allowed"] is False
    assert evidence["real_small_motion_gate"]["normalized_distance_m"] == 0.06
    assert evidence["real_small_motion_gate"]["configured_max_distance_m"] == 0.05
    assert evidence["real_small_motion_gate"]["requested_distance_within_configured_limit"] is False
    assert evidence["trajectory_sent"] is False
    assert evidence["execute_trajectory_called"] is False
    assert evidence["controller_command_sent"] is False
    assert evidence["real_robot_motion_executed"] is False


def test_real_small_motion_vague_motion_remains_blocked_before_execution(monkeypatch, capsys):
    monkeypatch.setattr(cli, "_lookup_current_tcp_pose", lambda timeout_s: pytest.fail("pose lookup should not run for parser-blocked command"))
    monkeypatch.setattr(cli, "evaluate_qwen_motion_parser", _fake_qwen_vague_motion_blocked)
    monkeypatch.setattr(cli, "_execution_prereq_blockers", lambda timeout_s: pytest.fail("execution prereq should not run for parser-blocked command"))
    monkeypatch.setattr("builtins.input", lambda _prompt: pytest.fail("confirmation prompt should not be reached"))

    exit_code = cli.main(["--real-small-motion", "--cmd", "go down a little"])

    evidence = _final_evidence(capsys.readouterr().out)
    assert exit_code == 2
    assert evidence["final_status"] == "BLOCKED"
    assert "E_DISTANCE_MM_REQUIRED" in evidence["blocking_reasons"]
    assert evidence["trajectory_sent"] is False
    assert evidence["execute_trajectory_called"] is False
    assert evidence["controller_command_sent"] is False
    assert evidence["real_robot_motion_executed"] is False


def test_real_small_motion_rejects_mock_current_tcp_pose(capsys):
    exit_code = cli.main(
        [
            "--real-small-motion",
            "--mock-current-tcp-pose",
            "--cmd",
            "raise the tcp by 2 millimeters",
        ]
    )

    output = capsys.readouterr().out
    evidence = json.loads(output)
    assert exit_code == 2
    assert evidence["final_status"] == "BLOCKED"
    assert "E_MOCK_CURRENT_TCP_POSE_NOT_ALLOWED_FOR_REAL_EXECUTION" in evidence["blocking_reasons"]
    assert evidence["current_tcp_pose"]["available"] is False
    assert evidence["current_tcp_pose"]["source"] == "real_robot_state_required"
    assert evidence["trajectory_sent"] is False
    assert evidence["controller_command_sent"] is False
    assert evidence["real_robot_motion_executed"] is False


def test_real_small_motion_without_real_current_tcp_pose_blocks(monkeypatch, capsys):
    monkeypatch.setattr(cli, "_lookup_current_tcp_pose", lambda timeout_s: None)
    monkeypatch.setattr(cli, "evaluate_qwen_motion_parser", _fake_qwen_two_mm_up_success)
    monkeypatch.setattr(cli, "_execution_prereq_blockers", lambda timeout_s: pytest.fail("prereq should not run without pose"))

    exit_code = cli.main(["--real-small-motion", "--cmd", "raise the tcp by 2 millimeters"])

    evidence = _final_evidence(capsys.readouterr().out)
    assert exit_code == 2
    assert evidence["final_status"] == "BLOCKED"
    assert evidence["current_tcp_pose"]["available"] is False
    assert evidence["current_tcp_pose"]["source"] == "real_robot_state_required"
    assert evidence["current_tcp_pose"]["tcp_pose_readiness_status"] == "BLOCKED"
    assert evidence["current_tcp_pose"]["tcp_pose_base_frame"] == "base_link"
    assert evidence["current_tcp_pose"]["tcp_pose_tool_frame"] == "tool0"
    assert evidence["current_tcp_pose"]["tcp_pose_lookup_success"] is False
    assert evidence["current_tcp_pose"]["current_tcp_pose_blocking_reason"] == "E_CURRENT_TCP_POSE_MISSING"
    assert "E_CURRENT_TCP_POSE_MISSING" in evidence["blocking_reasons"]
    assert evidence["trajectory_sent"] is False
    assert evidence["controller_command_sent"] is False
    assert evidence["real_robot_motion_executed"] is False


def test_qwen_forward_direction_mismatch_blocks_before_pose_lookup(monkeypatch, capsys):
    monkeypatch.setattr(cli, "_lookup_current_tcp_pose", lambda timeout_s: pytest.fail("pose lookup should not run on direction mismatch"))
    monkeypatch.setattr(cli, "evaluate_qwen_motion_parser", _fake_qwen_forward_as_z_success)

    exit_code = cli.main(["--dry-run", "--cmd", "move the tcp forward by 5 centimeters"])

    evidence = _final_evidence(capsys.readouterr().out)
    assert exit_code == 2
    assert evidence["final_status"] == "BLOCKED"
    assert "E_DIRECTION_PARSE_MISMATCH" in evidence["blocking_reasons"]
    assert evidence["expected_axis"] == "x"
    assert evidence["expected_direction"] == "+"
    assert evidence["qwen_axis"] == "z"
    assert evidence["qwen_direction"] == "+"
    assert evidence["trajectory_sent"] is False
    assert evidence["execute_trajectory_called"] is False
    assert evidence["real_robot_motion_executed"] is False


def test_acceptance_real_requires_real_small_motion(capsys):
    exit_code = cli.main(["--acceptance", "--real", "--cmd", "raise the tcp by 2 millimeters"])

    output = capsys.readouterr().out
    assert exit_code == 2
    assert "E_ACCEPTANCE_REAL_REQUIRES_REAL_SMALL_MOTION" in output
    assert '"real_robot_motion_executed": false' in output


def test_acceptance_unsafe_distance_remains_blocked(monkeypatch, capsys):
    monkeypatch.setattr(cli, "_lookup_current_tcp_pose", lambda timeout_s: pytest.fail("pose lookup should not run"))
    monkeypatch.setattr(cli, "evaluate_qwen_motion_parser", _fake_qwen_unsafe_distance)

    exit_code = cli.main(["--acceptance", "--dry-run", "--cmd", "raise the tcp by 20 millimeters"])

    evidence = _final_evidence(capsys.readouterr().out)
    workflow = evidence["acceptance_workflow"]
    assert exit_code == 2
    assert evidence["final_status"] != "PASS"
    assert workflow["status"] == "BLOCKED"
    assert workflow["real_robot_motion_executed"] is False
    assert "E_EXCEEDS_HARD_SAFETY_LIMIT" in workflow["blocking_reasons"]


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
    poses = iter([_pose(), _pose_z_offset(0.005)])
    monkeypatch.setattr(cli, "_lookup_current_tcp_pose", lambda timeout_s: next(poses))
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
    evidence = _final_evidence(output)
    post_motion = evidence["post_motion_verification"]
    assert evidence["post_motion_verification_status"] == "PASS"
    assert post_motion["tcp_pose_before_execution"]["position_m"] == [0.4, 0.0, 0.3]
    assert post_motion["target_tcp_pose"]["position_m"] == [0.4, 0.0, 0.305]
    assert post_motion["tcp_pose_after_execution"]["position_m"] == [0.4, 0.0, 0.305]
    assert post_motion["intended_delta_m"] == [0.0, 0.0, 0.005]
    assert post_motion["actual_displacement_m"] == [0.0, 0.0, 0.005]
    assert post_motion["actual_displacement_distance_m"] == 0.005
    assert post_motion["actual_distance_error_m"] == 0.0
    assert post_motion["intended_direction"] == "z+"
    assert post_motion["actual_direction"] == "z+"
    assert post_motion["direction_check_passed"] is True
    assert post_motion["orientation_change_rad"] == 0.0
    assert post_motion["tcp_pose_sample_settle_s"] == 0.25
    assert post_motion["tcp_pose_sample_attempts"] == 1
    assert post_motion["tcp_pose_stale_check_passed"] is None


def test_direction_verification_fails_opposite_displacement(monkeypatch, capsys):
    poses = iter([_pose(), _pose_z_offset(-0.002)])
    monkeypatch.setattr(cli, "_lookup_current_tcp_pose", lambda timeout_s: next(poses))
    monkeypatch.setattr(cli, "evaluate_qwen_motion_parser", _fake_qwen_two_mm_up_success)
    monkeypatch.setattr(cli, "_execution_prereq_blockers", lambda timeout_s: [])
    monkeypatch.setattr(cli, "evaluate_cartesian_motion_execution", _fake_success_execution)
    monkeypatch.setattr(cli.time, "sleep", lambda _seconds: None)
    monkeypatch.setattr("builtins.input", lambda _prompt: pytest.fail("confirmation prompt should not be used"))

    exit_code = cli.main(["--real", "--yes", "--cmd", "raise the tcp by 2 millimeters"])

    evidence = _final_evidence(capsys.readouterr().out)
    post_motion = evidence["post_motion_verification"]
    assert exit_code == 2
    assert evidence["final_status"] == "FAILED"
    assert post_motion["post_motion_verification_status"] == "FAILED"
    assert post_motion["intended_direction"] == "z+"
    assert post_motion["actual_direction"] == "z-"
    assert post_motion["direction_check_passed"] is False
    assert post_motion["reason"] == "post_motion_direction_distance_or_orientation_check_failed"


def test_post_motion_z_minus_with_negative_actual_z_passes():
    result = cli._post_motion_verification(
        real_robot_motion_executed=True,
        before_tcp_pose=_pose(),
        target_tcp_pose=_pose_z_offset(-0.002),
        after_tcp_pose=_pose_z_offset(-0.002),
        intended_delta_m=[0.0, 0.0, -0.002],
    )

    assert result["post_motion_verification_status"] == "PASS"
    assert result["intended_direction"] == "z-"
    assert result["actual_direction"] == "z-"
    assert result["direction_check_passed"] is True


def test_post_motion_z_minus_with_positive_actual_z_fails():
    result = cli._post_motion_verification(
        real_robot_motion_executed=True,
        before_tcp_pose=_pose(),
        target_tcp_pose=_pose_z_offset(-0.002),
        after_tcp_pose=_pose_z_offset(0.002),
        intended_delta_m=[0.0, 0.0, -0.002],
    )

    assert result["post_motion_verification_status"] == "FAILED"
    assert result["intended_direction"] == "z-"
    assert result["actual_direction"] == "z+"
    assert result["direction_check_passed"] is False


def test_post_motion_z_plus_with_positive_actual_z_passes():
    result = cli._post_motion_verification(
        real_robot_motion_executed=True,
        before_tcp_pose=_pose(),
        target_tcp_pose=_pose_z_offset(0.002),
        after_tcp_pose=_pose_z_offset(0.002),
        intended_delta_m=[0.0, 0.0, 0.002],
    )

    assert result["post_motion_verification_status"] == "PASS"
    assert result["intended_direction"] == "z+"
    assert result["actual_direction"] == "z+"
    assert result["direction_check_passed"] is True


def test_post_motion_z_plus_with_negative_actual_z_fails():
    result = cli._post_motion_verification(
        real_robot_motion_executed=True,
        before_tcp_pose=_pose(),
        target_tcp_pose=_pose_z_offset(0.002),
        after_tcp_pose=_pose_z_offset(-0.002),
        intended_delta_m=[0.0, 0.0, 0.002],
    )

    assert result["post_motion_verification_status"] == "FAILED"
    assert result["intended_direction"] == "z+"
    assert result["actual_direction"] == "z-"
    assert result["direction_check_passed"] is False


def test_recorded_lower_two_mm_evidence_passes_with_cross_axis_drift():
    result = cli._post_motion_verification(
        real_robot_motion_executed=True,
        before_tcp_pose={
            **_pose(),
            "position_m": [0.0, 0.0, 0.0],
            "orientation_xyzw": [0.0, 0.0, 0.0, 1.0],
            "tcp_pose_age_s": 0.001,
        },
        target_tcp_pose={
            **_pose(),
            "position_m": [0.0, 0.0, -0.002],
            "orientation_xyzw": [0.0, 0.0, 0.0, 1.0],
        },
        after_tcp_pose={
            **_pose(),
            "position_m": [-0.000206, -0.000073, -0.002073],
            "orientation_xyzw": [0.0, 0.0, 0.0022595, 0.999997447],
            "tcp_pose_age_s": 0.001,
            "tcp_pose_sample_settle_s": 0.25,
            "tcp_pose_sample_attempts": 1,
        },
        intended_delta_m=[0.0, 0.0, -0.002],
    )

    assert result["post_motion_verification_status"] == "PASS"
    assert result["actual_displacement_m"] == [-0.000206, -7.3e-05, -0.002073]
    assert result["actual_direction"] == "z-"
    assert result["direction_check_passed"] is True
    assert result["actual_displacement_distance_m"] == 0.002084
    assert result["actual_distance_error_m"] == 8.4e-05
    assert result["orientation_change_rad"] == 0.004519
    assert result["orientation_check_passed"] is True
    assert result["post_motion_distance_tolerance_m"] == 0.005
    assert result["post_motion_orientation_tolerance_rad"] == 0.01


def test_post_motion_verification_preserves_configured_tcp_frames():
    result = cli._post_motion_verification(
        real_robot_motion_executed=True,
        before_tcp_pose={
            **_pose(),
            "frame": "base",
            "tcp_pose_base_frame": "base",
            "tcp_pose_tool_frame": "tcp",
            "position_m": [0.0, 0.0, 0.0],
            "tcp_pose_age_s": 0.001,
        },
        target_tcp_pose={
            **_pose(),
            "frame": "base",
            "tcp_pose_base_frame": "base",
            "tcp_pose_tool_frame": "tcp",
            "position_m": [0.0, 0.0, 0.002],
        },
        after_tcp_pose={
            **_pose(),
            "frame": "base",
            "tcp_pose_base_frame": "base",
            "tcp_pose_tool_frame": "tcp",
            "position_m": [0.0, 0.0, 0.002],
            "tcp_pose_age_s": 0.001,
        },
        intended_delta_m=[0.0, 0.0, 0.002],
    )

    assert result["post_motion_verification_status"] == "PASS"
    assert result["tcp_pose_before_execution"]["tcp_pose_base_frame"] == "base"
    assert result["tcp_pose_before_execution"]["tcp_pose_tool_frame"] == "tcp"
    assert result["tcp_pose_after_execution"]["tcp_pose_base_frame"] == "base"
    assert result["tcp_pose_after_execution"]["tcp_pose_tool_frame"] == "tcp"


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
    poses = iter([_pose(), _pose_z_offset(0.005)])
    monkeypatch.setattr(cli, "_lookup_current_tcp_pose", lambda timeout_s: next(poses))
    monkeypatch.setattr(cli, "evaluate_qwen_motion_parser", _fake_qwen_success)
    monkeypatch.setattr(cli, "_execution_prereq_blockers", lambda timeout_s: [])
    monkeypatch.setattr(cli, "evaluate_cartesian_motion_execution", _fake_success_execution)
    monkeypatch.setattr(cli.time, "sleep", lambda _seconds: None)
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


def _pose_z_offset(offset_m):
    pose = _pose()
    pose["position_m"] = [0.4, 0.0, round(0.3 + offset_m, 12)]
    return pose


def _assert_real_small_two_mm_gate_allowed(evidence, *, direction):
    assert evidence["requested_distance_m"] == 0.002
    assert evidence["moveit_position_tolerance_m"] <= 0.0005
    assert evidence["moveit_position_tolerance_m"] < evidence["requested_distance_m"]
    assert evidence["moveit_orientation_tolerance_rad"] == 0.01
    assert evidence["tolerance_to_requested_distance_ratio"] == 0.1
    assert "real_motion_safety_policy_v1" in evidence["small_motion_tolerance_policy"]
    assert evidence["configured_max_distance_m"] == 0.05
    assert evidence["requested_distance_within_configured_limit"] is True
    assert evidence["real_small_motion_gate_policy"].startswith("lab_directional_step_motion_v1")
    assert evidence["real_small_motion_gate_basis"] == "normalized_contract"
    assert evidence["allowed_axis"] == "z"
    assert evidence["allowed_direction"] == direction
    assert evidence["motion_frame"] == "base_link"
    assert evidence["direction_axis"] == "z"
    assert evidence["direction_sign"] == direction
    assert evidence["allowed_distance_m"] == 0.002
    assert evidence["real_small_motion_command_allowed"] is True
    assert evidence["real_small_motion_gate"]["blocking_reasons"] == []
    assert evidence["real_small_motion_gate"]["normalized_axis"] == "z"
    assert evidence["real_small_motion_gate"]["normalized_direction"] == direction
    assert evidence["real_small_motion_gate"]["normalized_distance_m"] == 0.002


def _gateway_with_plan_metrics(metrics):
    real_gateway = cli.evaluate_cartesian_motion_gateway

    def fake_gateway(request):
        result = real_gateway(request)
        result["moveit_pose_executor_result"] = {
            **metrics,
            "trajectory_sent": False,
            "moveit_execute_called": False,
        }
        return result

    return fake_gateway


def _normal_up_plan_metrics():
    return {
        "trajectory_point_count": 3,
        "joint_trajectory_points": [
            {"positions": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], "time_from_start": {"sec": 0, "nanosec": 0}},
            {"positions": [0.0, 0.02, 0.0, 0.0, 0.0, 0.0], "time_from_start": {"sec": 0, "nanosec": 500000000}},
            {"positions": [0.0, 0.05, 0.01, 0.0, 0.0, 0.0], "time_from_start": {"sec": 1, "nanosec": 200000000}},
        ],
        "cartesian_waypoints": [
            {"position_m": [0.4, 0.0, 0.3], "orientation_xyzw": [0.0, 0.0, 0.0, 1.0]},
            {"position_m": [0.4, 0.0, 0.305], "orientation_xyzw": [0.0, 0.0, 0.0, 1.0]},
        ],
    }


def _normal_down_plan_metrics():
    return {
        "trajectory_point_count": 2,
        "joint_trajectory_points": [
            {"positions": [0.0, 0.02, 0.0, 0.0, 0.0, 0.0], "time_from_start": {"sec": 0, "nanosec": 0}},
            {"positions": [0.0, 0.01, -0.005, 0.0, 0.0, 0.0], "time_from_start": {"sec": 0, "nanosec": 400000000}},
        ],
        "cartesian_waypoints": [
            {"position_m": [0.4, 0.0, 0.3], "orientation_xyzw": [0.0, 0.0, 0.0, 1.0]},
            {"position_m": [0.4, 0.0, 0.298], "orientation_xyzw": [0.0, 0.0, 0.0, 1.0]},
        ],
    }


def _suspicious_plan_metrics():
    return {
        "trajectory_point_count": 2,
        "joint_trajectory_points": [
            {"positions": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]},
            {"positions": [0.0, 1.25, 0.0, 0.0, 0.0, 0.0]},
        ],
    }


def _trajectory_object(points):
    class Duration:
        def __init__(self, sec, nanosec):
            self.sec = sec
            self.nanosec = nanosec

    class Point:
        def __init__(self, positions, time_from_start):
            self.positions = positions
            self.time_from_start = Duration(time_from_start["sec"], time_from_start["nanosec"])

    class JointTrajectory:
        def __init__(self, raw_points):
            self.points = [Point(positions, time_from_start) for positions, time_from_start in raw_points]

    class Trajectory:
        def __init__(self, raw_points):
            self.joint_trajectory = JointTrajectory(raw_points)

    return Trajectory(points)


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


def _fake_qwen_two_mm_up_success(request):
    raw = json.dumps(
        {
            "intent": "relative_cartesian_motion",
            "axis": "z",
            "direction": "+",
            "distance_m": 0.002,
            "confidence": 0.96,
            "reason": "small upward acceptance motion",
        }
    )
    return {
        "qwen_motion_parser_status": "PASS",
        "parser_source": "qwen_llm",
        "llm_called": True,
        "model_name": request.model_name or "qwen2.5vl:3b",
        "qwen_endpoint": request.endpoint,
        "llm_latency_ms": 1.0,
        "raw_llm_output": raw,
        "normalized_contract": {
            "intent": "relative_cartesian_motion",
            "frame": "base_link",
            "delta_m": [0.0, 0.0, 0.002],
            "max_distance_m": request.max_distance_m,
            "hard_safety_limit_m": request.hard_safety_limit_m,
            "must_confirm": True,
        },
        "axis": "z",
        "direction": "+",
        "distance_m": 0.002,
        "confidence": 0.96,
        "delta_m": [0.0, 0.0, 0.002],
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


def _fake_qwen_down_five_cm_success(request):
    raw = json.dumps(
        {
            "intent": "relative_cartesian_motion",
            "axis": "z",
            "direction": "-",
            "distance_m": 0.05,
            "confidence": 0.96,
            "reason": "larger downward relative motion",
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
            "delta_m": [0.0, 0.0, -0.05],
            "max_distance_m": request.max_distance_m,
            "hard_safety_limit_m": request.hard_safety_limit_m,
            "must_confirm": True,
        },
        "axis": "z",
        "direction": "-",
        "distance_m": 0.05,
        "confidence": 0.96,
        "delta_m": [0.0, 0.0, -0.05],
        "parser_blocking_reasons": [],
        "blocking_reasons": [],
    }


def _fake_qwen_down_six_cm_success(request):
    raw = json.dumps(
        {
            "intent": "relative_cartesian_motion",
            "axis": "z",
            "direction": "-",
            "distance_m": 0.06,
            "confidence": 0.96,
            "reason": "over configured downward relative motion",
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
            "delta_m": [0.0, 0.0, -0.06],
            "max_distance_m": request.max_distance_m,
            "hard_safety_limit_m": request.hard_safety_limit_m,
            "must_confirm": True,
        },
        "axis": "z",
        "direction": "-",
        "distance_m": 0.06,
        "confidence": 0.96,
        "delta_m": [0.0, 0.0, -0.06],
        "parser_blocking_reasons": [],
        "blocking_reasons": [],
    }


def _fake_qwen_forward_as_z_success(request):
    raw = json.dumps(
        {
            "intent": "relative_cartesian_motion",
            "axis": "z",
            "direction": "+",
            "distance_m": 0.05,
            "confidence": 0.96,
            "reason": "incorrectly mapped forward as upward",
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
            "delta_m": [0.0, 0.0, 0.05],
            "max_distance_m": request.max_distance_m,
            "hard_safety_limit_m": request.hard_safety_limit_m,
            "must_confirm": True,
        },
        "axis": "z",
        "direction": "+",
        "distance_m": 0.05,
        "confidence": 0.96,
        "delta_m": [0.0, 0.0, 0.05],
        "parser_blocking_reasons": [],
        "blocking_reasons": [],
    }


def _fake_qwen_vague_motion_blocked(request):
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
                "direction": "-",
                "confidence": 0.5,
            }
        ),
        "normalized_contract": None,
        "parser_blocking_reasons": ["E_DISTANCE_MM_REQUIRED"],
        "blocking_reasons": ["E_DISTANCE_MM_REQUIRED"],
    }


def _fake_plan_only_execution(request):
    assert request.config["enable_moveit_execute"] is False
    assert request.config["enable_real_robot_motion"] is False
    assert request.config["trajectory_send_allowed"] is False
    assert request.config["requested_distance_m"] == 0.002
    assert request.config["position_tolerance_m"] <= 0.0005
    assert request.config["position_tolerance_m"] < request.config["requested_distance_m"]
    assert request.config["orientation_tolerance_rad"] == 0.01
    return {
        "cartesian_motion_execution_status": "PASS",
        "moveit_plan_requested": True,
        "moveit_plan_success": True,
        "manual_confirmation_required": True,
        "manual_confirmation_accepted": False,
        "moveit_execute_called": False,
        "trajectory_send_allowed": False,
        "trajectory_sent": False,
        "controller_command_sent": False,
        "real_robot_motion_executed": False,
        "moveit_pose_executor_result": {
            "moveit_pose_executor_status": "PASS",
            "plan_success": True,
            "moveit_plan_called": True,
            "moveit_execute_called": False,
            "trajectory_send_allowed": False,
            "trajectory_sent": False,
            "controller_command_sent": False,
            "real_robot_motion_executed": False,
            "planned_trajectory": _trajectory_object(
                [
                    ([0.0, 0.0, 0.0, 0.0, 0.0, 0.0], {"sec": 0, "nanosec": 0}),
                    ([0.0, 0.01, 0.0, 0.0, 0.0, 0.0], {"sec": 0, "nanosec": 500000000}),
                ]
            ),
            "trajectory_point_count": 2,
            "blocking_reasons": [],
            "warnings": [],
        },
        "blocking_reasons": [],
        "warnings": [],
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
