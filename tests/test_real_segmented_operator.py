import io
from pathlib import Path

from scripts import teto_operator_console as real_console
from src import moveit_pose_executor
from src.moveit_pose_executor import (
    MoveItPoseExecutorRequest,
    evaluate_moveit_pose_execute,
)
from src.real_segmented_operator_backend import (
    DashboardTCPClient,
    E_CONTROLLER_MANAGER_UNAVAILABLE,
    E_D455_TOPIC_NOT_FOUND,
    E_DASHBOARD_NOT_REMOTE,
    E_DASHBOARD_UNAVAILABLE,
    E_PROGRAM_NOT_PLAYING,
    E_SAFETY_STATUS_NOT_NORMAL,
    E_SCALED_CONTROLLER_INACTIVE,
    E_TCP_POSE_UNAVAILABLE,
    RealOperatorStateProvider,
    RealSegmentedBackendConfig,
    RealSegmentedOperatorBackend,
)
from src.unified_segmented_operator import (
    UnifiedOperatorConfig,
    UnifiedSegmentedOperator,
)


def _autonomous_config():
    return {
        "manual_confirmation_required": False,
        "autonomous_segmented_execution": True,
        "safety_gate_still_required": True,
        "authoritative_state_check": True,
        "vision_guard_passed": True,
        "controller_active": True,
        "external_control_playing": True,
        "dashboard_robot_mode_running": True,
        "dashboard_safety_mode_ok": True,
        "dashboard_safety_status_ok": True,
        "dashboard_program_state_playing": True,
    }


def test_autonomous_moveit_mode_requires_every_authoritative_gate():
    config = _autonomous_config()
    assert (
        moveit_pose_executor._autonomous_segmented_execution_blockers(config, {})
        == []
    )

    for field in (
        "authoritative_state_check",
        "vision_guard_passed",
        "controller_active",
        "external_control_playing",
        "dashboard_robot_mode_running",
        "dashboard_safety_mode_ok",
        "dashboard_safety_status_ok",
        "dashboard_program_state_playing",
    ):
        blocked = {**config, field: False}
        assert moveit_pose_executor._autonomous_segmented_execution_blockers(
            blocked, {}
        )


def test_autonomous_mode_does_not_accept_missing_safety_gate_contract():
    config = _autonomous_config()
    config["safety_gate_still_required"] = False
    reasons = moveit_pose_executor._autonomous_segmented_execution_blockers(
        config, {}
    )
    assert "E_AUTONOMOUS_SEGMENTED_SAFETY_GATE_REQUIRED" in reasons


def test_autonomous_segment_executes_without_manual_confirmation_when_all_gates_pass(
    monkeypatch,
):
    monkeypatch.setattr(
        "src.moveit_pose_executor._plan_with_move_group_action",
        lambda _target, _config: {
            "action_call_attempted": True,
            "action_server_available": True,
            "goal_accepted": True,
            "success": True,
            "error_code": 1,
            "error_code_name": "SUCCESS",
            "planned_trajectory": object(),
            "trajectory_point_count": 2,
        },
    )
    monkeypatch.setattr(
        "src.moveit_pose_executor._execute_trajectory_action",
        lambda _trajectory, _config: {
            "action_call_attempted": True,
            "action_server_available": True,
            "goal_accepted": True,
            "success": True,
            "error_code": 1,
            "error_code_name": "SUCCESS",
        },
    )
    config = {
        **_autonomous_config(),
        "planning_group": "ur_manipulator",
        "end_effector_link": "tool0",
        "allowed_frames": ["base_link"],
        "max_translation_m": 0.05,
        "hard_safety_limit_m": 0.05,
        "workspace_bounds": {
            "x": [-1.0, 1.0],
            "y": [-1.0, 1.0],
            "z": [0.0, 2.0],
        },
        "robot_state_ok": True,
        "safety_status_ok": True,
        "protective_stop": False,
        "emergency_stop": False,
        "speed_scaling": 0.05,
        "max_speed_scale": 0.10,
        "max_acc_scale": 0.10,
    }
    result = evaluate_moveit_pose_execute(
        MoveItPoseExecutorRequest(
            requested=True,
            target_pose=_pose([0.0, 0.0, 0.51]),
            current_tcp_pose=_pose([0.0, 0.0, 0.50]),
            config=config,
            manual_confirmation_result={},
            robot_state_result=config,
        )
    )

    assert result["moveit_pose_executor_status"] == "PASS"
    assert result["manual_confirmation_required"] is False
    assert result["autonomous_segmented_execution"] is True
    assert result["safety_gate_still_required"] is True
    assert result["real_robot_motion_executed"] is True


def test_real_backend_is_fail_closed_and_requires_vision():
    backend = RealSegmentedOperatorBackend()
    assert backend.execution_mode == "real_ur5e"
    assert backend.autonomous_segmented_execution is True
    assert backend.vision_guard_required is True


def test_real_console_main_path_is_unified_and_legacy_is_explicit():
    console = Path("scripts/qwen_operator_console.sh").read_text(encoding="utf-8")
    startup = Path("scripts/start_teto_qwen_real_operator.sh").read_text(
        encoding="utf-8"
    )

    assert "exec python scripts/teto_operator_console.py --backend real" in console
    assert 'if [[ "${1:-}" == "--legacy-manual" ]]' in console
    assert "--legacy-manual-console" in startup
    unified_prefix = console.split('if [[ "${MODE}" == "unified" ]]', 1)[1].split(
        "interrupted=0", 1
    )[0]
    assert "--real-small-motion" not in unified_prefix
    assert "--yes" not in unified_prefix


def test_real_backend_source_uses_dashboard_controller_tcp_and_d455_guards():
    source = Path("src/real_segmented_operator_backend.py").read_text(
        encoding="utf-8"
    )
    for required in (
        "DashboardTCPClient",
        '"robotmode"',
        '"safetystatus"',
        '"programState"',
        '"is in remote control"',
        "ListControllers",
        "JointState",
        "PoseStamped",
        "E_D455_SNAPSHOT_UNAVAILABLE",
        "E_D455_TOPIC_NOT_FOUND",
        "E_DASHBOARD_UNAVAILABLE",
        "E_MOVEIT_EXECUTOR_UNAVAILABLE",
        "E_TCP_POSE_STALE",
        "evaluate_cartesian_motion_gateway",
        "evaluate_cartesian_motion_execution",
    ):
        assert required in source
    assert "manual_confirmation_result={}" in source
    assert '"manual_confirmation_required": False' in source
    assert "--yes" not in source


def test_real_console_constructs_and_injects_concrete_provider():
    args = real_console.build_parser().parse_args([])
    operator = real_console.build_operator(args)

    assert isinstance(operator.backend, RealSegmentedOperatorBackend)
    assert isinstance(
        operator.backend.state_provider,
        RealOperatorStateProvider,
    )
    assert operator.backend.state_provider.config.robot_ip == "192.168.20.35"
    assert operator.backend.state_provider.config.dashboard_port == 29999
    assert operator.backend.state_provider.config.tcp_pose_topic == "/tcp_pose"
    assert (
        operator.backend.state_provider.config.joint_states_topic
        == "/joint_states"
    )


def test_dashboard_unavailable_has_specific_error():
    def unavailable(*_args, **_kwargs):
        raise OSError("connection refused")

    result = DashboardTCPClient(
        host="192.168.20.35",
        socket_factory=unavailable,
    ).read_status()

    assert result["status"] == "BLOCKED"
    assert result["abort_reason"] == E_DASHBOARD_UNAVAILABLE
    assert result["dashboard_reachable"] is False


def test_dashboard_queries_are_parsed_from_direct_tcp_client():
    class FakeSocket:
        def __init__(self):
            self.responses = iter(
                (
                    b"Connected: Universal Robots Dashboard Server\n",
                    b"Robotmode: RUNNING\n",
                    b"Safetystatus: NORMAL\n",
                    b"PLAYING external_control.urp\n",
                    b"true\n",
                )
            )
            self.commands = []

        def __enter__(self):
            return self

        def __exit__(self, *_args):
            return False

        def settimeout(self, _timeout):
            return None

        def recv(self, _size):
            return next(self.responses)

        def sendall(self, payload):
            self.commands.append(payload.decode("ascii").strip())

    fake = FakeSocket()
    result = DashboardTCPClient(
        host="192.168.20.35",
        socket_factory=lambda *_args, **_kwargs: fake,
    ).read_status()

    assert result["status"] == "PASS"
    assert result["dashboard_reachable"] is True
    assert result["robotmode"] == "RUNNING"
    assert result["safetystatus"] == "NORMAL"
    assert result["programState"] == "PLAYING"
    assert result["remote_control"] is True
    assert fake.commands == [
        "robotmode",
        "safetystatus",
        "programState",
        "is in remote control",
    ]


def test_dashboard_specific_state_errors():
    class FakeSocket:
        def __init__(self, responses):
            self.responses = iter(responses)

        def __enter__(self):
            return self

        def __exit__(self, *_args):
            return False

        def settimeout(self, _timeout):
            return None

        def recv(self, _size):
            return next(self.responses)

        def sendall(self, _payload):
            return None

    cases = (
        (
            (b"welcome\n", b"RUNNING\n", b"NORMAL\n", b"PLAYING\n", b"false\n"),
            E_DASHBOARD_NOT_REMOTE,
        ),
        (
            (b"welcome\n", b"RUNNING\n", b"NORMAL\n", b"STOPPED\n", b"true\n"),
            E_PROGRAM_NOT_PLAYING,
        ),
        (
            (
                b"welcome\n",
                b"RUNNING\n",
                b"PROTECTIVE_STOP\n",
                b"PLAYING\n",
                b"true\n",
            ),
            E_SAFETY_STATUS_NOT_NORMAL,
        ),
    )
    for responses, reason in cases:
        result = DashboardTCPClient(
            host="192.168.20.35",
            socket_factory=lambda *_args, _responses=responses, **_kwargs: FakeSocket(
                _responses
            ),
        ).read_status()
        assert result["abort_reason"] == reason


def test_missing_tcp_pose_returns_specific_error(monkeypatch):
    monkeypatch.setattr(
        "src.real_segmented_operator_backend._read_tcp_pose_topic",
        lambda **_kwargs: {
            "status": "BLOCKED",
            "abort_reason": E_TCP_POSE_UNAVAILABLE,
        },
    )
    provider = RealOperatorStateProvider(RealSegmentedBackendConfig())

    assert provider.read_tcp_pose()["abort_reason"] == E_TCP_POSE_UNAVAILABLE


def test_controller_inactive_and_unavailable_keep_specific_errors(monkeypatch):
    dashboard = type(
        "Dashboard",
        (),
        {"read_status": lambda self: _dashboard_pass()},
    )()
    provider = RealOperatorStateProvider(
        RealSegmentedBackendConfig(),
        dashboard_client=dashboard,
    )
    monkeypatch.setattr(
        provider,
        "read_joint_states",
        lambda: {"status": "PASS"},
    )
    monkeypatch.setattr(
        "src.real_segmented_operator_backend._read_moveit_status",
        lambda **_kwargs: {"status": "PASS"},
    )
    for reason in (
        E_SCALED_CONTROLLER_INACTIVE,
        E_CONTROLLER_MANAGER_UNAVAILABLE,
    ):
        monkeypatch.setattr(
            "src.real_segmented_operator_backend._read_controller_status",
            lambda **_kwargs: {
                "status": "BLOCKED",
                "abort_reason": reason,
            },
        )
        assert provider.check_motion_state()["abort_reason"] == reason


def test_d455_topic_missing_returns_specific_error(monkeypatch):
    monkeypatch.setattr(
        "src.real_segmented_operator_backend._discover_d455_topics",
        lambda **_kwargs: {
            "status": "BLOCKED",
            "abort_reason": E_D455_TOPIC_NOT_FOUND,
        },
    )
    provider = RealOperatorStateProvider(RealSegmentedBackendConfig())

    result = provider.capture_snapshot(phase="status")
    assert result["abort_reason"] == E_D455_TOPIC_NOT_FOUND


def test_console_reader_does_not_concatenate_previous_input():
    stdin = io.StringIO("move up 1cm\nmove down 1cm\n")
    stdout = io.StringIO()

    first = real_console._read_console_command(stdin, stdout)
    second = real_console._read_console_command(stdin, stdout)

    assert first == "move up 1cm"
    assert second == "move down 1cm"
    assert "momove" not in first + second
    assert stdout.getvalue() == "\nTETO/Operator> \nTETO/Operator> "


def test_startup_restores_terminal_fds_before_console():
    startup = Path("scripts/start_teto_qwen_real_operator.sh").read_text(
        encoding="utf-8"
    )
    assert "exec 3>&1 4>&2" in startup
    assert "exec 1>&3 2>&4" in startup


def test_status_has_non_null_real_backend_fields(tmp_path):
    expected_status = {
        "status": "PASS",
        "dashboard_reachable": True,
        "robotmode": "RUNNING",
        "safetystatus": "NORMAL",
        "programState": "PLAYING",
        "remote_control": True,
        "tcp_pose_available": True,
        "tcp_pose_fresh": True,
        "tcp_pose_stamp": 123.0,
        "joint_states_available": True,
        "scaled_joint_trajectory_controller_active": True,
        "d455_color_topic": "/camera/color/image_raw",
        "d455_depth_topic": "/camera/aligned_depth/image_raw",
        "d455_snapshot_fresh": True,
        "qwen_healthy": True,
        "manual_confirmation_required": False,
        "autonomous_segmented_execution": True,
        "safety_gate_still_required": True,
    }

    class StatusProvider:
        def status(self):
            return expected_status

    backend = RealSegmentedOperatorBackend(
        state_provider=StatusProvider(),
    )
    operator = UnifiedSegmentedOperator(
        config=UnifiedOperatorConfig(
            parser_mode="shared",
            output_dir=tmp_path,
        ),
        backend=backend,
    )
    result = operator.handle_command("status")

    assert result["status"] == "PASS"
    assert result["backend_status"] == expected_status
    assert result["backend_status"] is not None
    assert result["manual_confirmation_required"] is False
    assert result["autonomous_segmented_execution"] is True


def test_isaac_sim_files_do_not_import_real_backend_or_ros_control_types():
    source = Path("src/isaac_sim_operator.py").read_text(encoding="utf-8")
    console = Path("scripts/teto_isaac_operator_console.py").read_text(
        encoding="utf-8"
    )
    for forbidden in (
        "real_segmented_operator_backend",
        "ur_dashboard_msgs",
        "controller_manager_msgs",
        "sensor_msgs",
    ):
        assert forbidden not in source
        assert forbidden not in console
    assert "OperatorCommandInterface" in source


def _dashboard_pass():
    return {
        "status": "PASS",
        "dashboard_reachable": True,
        "robotmode": "RUNNING",
        "safetystatus": "NORMAL",
        "programState": "PLAYING",
        "remote_control": True,
    }


def _pose(position):
    return {
        "frame": "base_link",
        "position_m": list(position),
        "orientation_xyzw": [0.0, 0.0, 0.0, 1.0],
    }
