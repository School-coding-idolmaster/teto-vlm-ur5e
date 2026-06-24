from scripts.harnesses.run_shadow_simulation_contract import build_parser
from src.ur5_read_only_state_contract import (
    E_DASHBOARD_COMMAND_NOT_ALLOWED,
    E_READ_ONLY_MODE_REQUIRED,
    E_RTDE_WRITE_NOT_ALLOWED,
    E_REQUIRED_STATE_FIELD_MISSING,
    READ_ONLY_STATE_CONTRACT_READY,
    REQUIRED_STATE_FIELDS,
    STATUS_BLOCKED,
    STATUS_PASS,
    UR5ReadOnlyStateRequest,
    evaluate_ur5_read_only_state,
    format_ur5_read_only_state_report,
)


def test_positive_ur5_read_only_state_contract_passes_without_writes():
    result = _evaluate()

    assert result["ur5_read_only_state_status"] == STATUS_PASS
    assert result["read_only_state_status"] == READ_ONLY_STATE_CONTRACT_READY
    assert result["read_only_state_contract_ready"] is True
    assert result["robot_model"] == "UR5e"
    assert result["robot_ip_declared"] is True
    assert result["read_only_mode"] is True
    assert result["rtde_read_enabled"] == "declared_future_only"
    assert result["rtde_write_enabled"] is False
    assert result["rtde_write_attempted"] is False
    assert result["dashboard_read_enabled"] == "declared_future_only"
    assert result["dashboard_command_enabled"] is False
    assert result["dashboard_command_attempted"] is False
    assert result["required_state_fields_declared"] == list(REQUIRED_STATE_FIELDS)
    assert result["state_ttl_ms"] == 500
    assert result["manual_confirmation_required"] is True
    assert result["execution_allowed"] is False
    assert result["real_robot_enabled"] is False
    assert result["real_robot_motion_executed"] is False
    assert result["blocking_reasons"] == []


def test_rtde_write_request_blocks_but_result_stays_read_only():
    result = _evaluate(config={"rtde_write_enabled": True, "rtde_write_attempted": True})

    assert result["ur5_read_only_state_status"] == STATUS_BLOCKED
    assert E_RTDE_WRITE_NOT_ALLOWED in result["blocking_reasons"]
    assert result["rtde_write_enabled"] is False
    assert result["rtde_write_attempted"] is False
    assert result["execution_allowed"] is False


def test_dashboard_command_request_blocks_but_result_stays_read_only():
    result = _evaluate(config={"dashboard_command_enabled": True, "dashboard_command_attempted": True})

    assert result["ur5_read_only_state_status"] == STATUS_BLOCKED
    assert E_DASHBOARD_COMMAND_NOT_ALLOWED in result["blocking_reasons"]
    assert result["dashboard_command_enabled"] is False
    assert result["dashboard_command_attempted"] is False


def test_missing_required_state_field_blocks():
    result = _evaluate(config={"required_state_fields": ["robot_mode"]})

    assert result["ur5_read_only_state_status"] == STATUS_BLOCKED
    assert E_REQUIRED_STATE_FIELD_MISSING in result["blocking_reasons"]
    assert "safety_status" in result["missing_state_fields"]


def test_read_only_mode_false_blocks():
    result = _evaluate(config={"read_only_mode": False})

    assert result["ur5_read_only_state_status"] == STATUS_BLOCKED
    assert E_READ_ONLY_MODE_REQUIRED in result["blocking_reasons"]
    assert result["real_robot_enabled"] is False
    assert result["real_robot_motion_executed"] is False


def test_ur5_read_only_report_states_disabled_write_surfaces():
    report = format_ur5_read_only_state_report(_evaluate())

    assert "TETO V2.11.0 UR5 Read-Only State Contract Report" in report
    assert "does not connect to RTDE or Dashboard live sockets" in report
    assert "does not write RTDE values" in report
    assert "does not send Dashboard commands" in report
    assert "does not command a real UR5" in report


def test_cli_parser_accepts_ur5_read_only_state_flags():
    args = build_parser().parse_args(
        [
            "--check-ur5-read-only-state",
            "--ur5-read-only-state-config",
            "configs/ur5_read_only_state.example.yaml",
            "--ur5-read-only-state-report",
        ]
    )

    assert args.check_ur5_read_only_state is True
    assert args.ur5_read_only_state_config == "configs/ur5_read_only_state.example.yaml"
    assert args.ur5_read_only_state_report is True


def _evaluate(*, config: dict | None = None) -> dict:
    request = UR5ReadOnlyStateRequest(requested=True, config=_merged_config(config))
    return evaluate_ur5_read_only_state(request)


def _valid_config() -> dict:
    return {
        "read_only_mode": True,
        "robot_model": "UR5e",
        "robot_ip": "unavailable_for_shadow",
        "rtde_read_enabled": "declared_future_only",
        "rtde_write_enabled": False,
        "rtde_write_attempted": False,
        "dashboard_read_enabled": "declared_future_only",
        "dashboard_command_enabled": False,
        "dashboard_command_attempted": False,
        "state_ttl_ms": 500,
        "manual_confirmation_required": True,
        "execution_allowed": False,
        "real_robot_enabled": False,
        "required_state_fields": list(REQUIRED_STATE_FIELDS),
    }


def _merged_config(config: dict | None) -> dict:
    merged = _valid_config()
    if config:
        merged.update(config)
    return merged
