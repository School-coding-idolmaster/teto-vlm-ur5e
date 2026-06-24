import copy
import json

import pytest

from src.unified_segmented_operator import (
    OperatorCommandInterface,
    UnifiedOperatorConfig,
    UnifiedSegmentedOperator,
)


class FakeBackend:
    backend_name = "fake_measured_backend"
    execution_mode = "real_ur5e"
    autonomous_segmented_execution = True
    vision_guard_required = True

    def __init__(
        self,
        *,
        fail_at=None,
        vision_status="PASS",
        tcp_fresh=True,
        state_status="PASS",
    ):
        self.pose = {
            "frame": "base_link",
            "position_m": [0.0, 0.0, 0.5],
            "orientation_xyzw": [0.0, 0.0, 0.0, 1.0],
        }
        self.home_pose = copy.deepcopy(self.pose)
        self.fail_at = fail_at
        self.vision_status = vision_status
        self.tcp_fresh = tcp_fresh
        self.state_status = state_status
        self.calls = 0
        self.snapshot_count = 0

    def status(self):
        return {
            "status": self.state_status,
            "current_tcp_pose": copy.deepcopy(self.pose),
        }

    def read_tcp_pose(self):
        return {
            "status": "PASS" if self.tcp_fresh else "BLOCKED",
            "abort_reason": None if self.tcp_fresh else "E_TCP_POSE_STALE",
            "current_tcp_pose": copy.deepcopy(self.pose),
            "tcp_pose_age_s": 0.01 if self.tcp_fresh else 5.0,
            "tcp_pose_fresh": self.tcp_fresh,
        }

    def check_motion_state(self):
        return {
            "status": self.state_status,
            "abort_reason": (
                None
                if self.state_status == "PASS"
                else "E_SCALED_CONTROLLER_INACTIVE"
            ),
            "authoritative_state_check": True,
            "controller_active": self.state_status == "PASS",
            "external_control_playing": True,
            "dashboard_robot_mode_running": True,
            "dashboard_safety_mode_ok": True,
            "dashboard_program_state_playing": True,
            "robot_state_ok": self.state_status == "PASS",
            "safety_status_ok": True,
            "protective_stop": False,
            "emergency_stop": False,
        }

    def capture_snapshot(self, *, phase, previous_snapshot=None):
        self.snapshot_count += 1
        return {
            "status": self.vision_status,
            "abort_reason": (
                None
                if self.vision_status == "PASS"
                else "E_D455_SNAPSHOT_UNAVAILABLE"
            ),
            "snapshot_id": f"snapshot-{self.snapshot_count}",
            "snapshot_phase": phase,
            "freshness_check_passed": self.vision_status == "PASS",
            "newer_than_previous": True,
        }

    def execute_subgoal(
        self,
        *,
        delta_m,
        current_tcp_pose,
        target_tcp_pose,
        state_evidence,
        vision_evidence,
        substep_index,
        substep_count,
    ):
        self.calls += 1
        if self.fail_at == self.calls:
            return {
                "status": "BLOCKED",
                "abort_reason": "E_FAKE_SEGMENT_FAILURE",
                "real_robot_motion_executed": False,
            }
        self.pose = copy.deepcopy(target_tcp_pose)
        return {
            "status": "PASS",
            "real_robot_motion_executed": True,
            "trajectory_sent": True,
            "manual_confirmation_required": False,
        }

    def begin_relative_motion(self, *, initial_tcp_pose, planned_subgoals):
        return None

    def home_reference_pose(self):
        return copy.deepcopy(self.home_pose)

    def reset_session(self):
        return {"status": "PASS"}


def _operator(tmp_path, backend=None, *, max_substep=0.05):
    return UnifiedSegmentedOperator(
        config=UnifiedOperatorConfig(
            max_total_distance_m=0.50,
            max_substep_distance_m=max_substep,
            position_tolerance_m=0.001,
            orientation_tolerance_rad=0.01,
            workspace_bounds={"x": [-1, 1], "y": [-1, 1], "z": [0, 2]},
            parser_mode="shared",
            output_dir=tmp_path,
        ),
        backend=backend or FakeBackend(),
    )


def test_command_interface_has_shared_builtin_semantics():
    interface = OperatorCommandInterface(UnifiedOperatorConfig())
    assert interface.classify("status") == "status"
    assert interface.classify("home") == "home"
    assert interface.classify("reset") == "reset"
    assert interface.classify("help") == "help"
    assert interface.classify("quit") == "quit"
    assert interface.classify("exit") == "quit"
    assert interface.classify("move up 5 mm") == "motion"


def test_status_help_and_quit_share_result_semantics(tmp_path):
    operator = _operator(tmp_path)
    status = operator.handle_command("status")
    help_result = operator.handle_command("help")
    quit_result = operator.handle_command("quit")

    assert status["status"] == "PASS"
    assert status["operator_command"] == "status"
    assert help_result["commands"][:3] == ["status", "home", "reset"]
    assert quit_result["quit_requested"] is True
    assert status["manual_confirmation_required"] is False
    assert status["safety_gate_still_required"] is True


@pytest.mark.parametrize(
    ("command", "expected_segments"),
    [
        ("move up 10 cm", 2),
        ("move forward 30 cm", 6),
        ("move left 50 cm", 10),
    ],
)
def test_unified_decomposition_executes_each_segment(
    tmp_path,
    command,
    expected_segments,
):
    backend = FakeBackend()
    result = _operator(tmp_path, backend).handle_command(command)

    assert result["status"] == "PASS"
    assert result["substep_count"] == expected_segments
    assert result["completed_substep_count"] == expected_segments
    assert backend.calls == expected_segments
    assert result["manual_confirmation_required"] is False
    assert result["autonomous_segmented_execution"] is True
    assert result["safety_gate_still_required"] is True
    assert all(step["verification_result"] == "PASS" for step in result["substeps"])
    assert all(step["before_snapshot_evidence"] for step in result["substeps"])
    assert all(step["after_snapshot_evidence"] for step in result["substeps"])


def test_real_qwen_ten_centimeters_routes_to_segments_not_legacy_excessive_error(
    tmp_path,
):
    payload = {
        "schema_version": "teto_motion_semantics.v1",
        "intent_status": "ok",
        "intent_type": "relative_cartesian_motion",
        "motion": {
            "reference": "tcp",
            "mode": "single_axis",
            "direction_semantic": "up",
            "delta": {
                "z": {
                    "value": 10,
                    "unit": "cm",
                    "meters": 0.10,
                    "quality": "explicit",
                }
            },
            "distance": None,
            "frame_hint": "base_link",
        },
        "clarification": {"required": False, "reason": ""},
        "unsupported": {"reason": ""},
        "confidence": {
            "intent": 0.99,
            "direction": 0.99,
            "distance": 0.99,
            "overall": 0.99,
        },
        "language": "en",
    }
    backend = FakeBackend()
    operator = UnifiedSegmentedOperator(
        config=UnifiedOperatorConfig(
            max_total_distance_m=0.50,
            max_substep_distance_m=0.05,
            workspace_bounds={"x": [-1, 1], "y": [-1, 1], "z": [0, 2]},
            parser_mode="qwen",
            output_dir=tmp_path,
        ),
        backend=backend,
        qwen_callable=lambda _prompt: json.dumps(payload),
    )
    result = operator.handle_command("move up 10 cm")

    assert result["status"] == "PASS"
    assert result["requested_total_distance_m"] == 0.10
    assert result["substep_count"] == 2
    assert result["parser_result"]["blocking_reasons"] == []
    assert "E_EXCESSIVE_CARTESIAN_MOTION" not in json.dumps(result)


def test_segment_three_failure_aborts_without_starting_later_segments(tmp_path):
    backend = FakeBackend(fail_at=3)
    result = _operator(tmp_path, backend, max_substep=0.02).handle_command(
        "move up 10 cm"
    )

    assert result["status"] == "ABORTED"
    assert result["abort_reason"] == "E_FAKE_SEGMENT_FAILURE"
    assert result["completed_substep_count"] == 2
    assert backend.calls == 3
    assert len(result["substeps"]) == 3
    assert result["substeps"][2]["continue_allowed"] is False


def test_d455_unavailable_blocks_before_moveit_segment(tmp_path):
    backend = FakeBackend(vision_status="BLOCKED")
    result = _operator(tmp_path, backend).handle_command("move up 10 cm")

    assert result["status"] == "ABORTED"
    assert result["abort_reason"] == "E_D455_SNAPSHOT_UNAVAILABLE"
    assert backend.calls == 0
    assert result["real_robot_motion_executed"] is False


def test_tcp_pose_stale_blocks_before_segment_execution(tmp_path):
    backend = FakeBackend(tcp_fresh=False)
    result = _operator(tmp_path, backend).handle_command("move up 10 cm")

    assert result["status"] == "BLOCKED"
    assert result["abort_reason"] == "E_TCP_POSE_STALE"
    assert backend.calls == 0


def test_tcp_pose_stale_after_segment_aborts_before_next_segment(tmp_path):
    class StaleAfterExecutionBackend(FakeBackend):
        def __init__(self):
            super().__init__()
            self.stale_after_execute = False

        def execute_subgoal(self, **kwargs):
            result = super().execute_subgoal(**kwargs)
            self.stale_after_execute = True
            return result

        def read_tcp_pose(self):
            result = super().read_tcp_pose()
            if self.stale_after_execute:
                result.update(
                    {
                        "status": "BLOCKED",
                        "abort_reason": "E_TCP_POSE_STALE",
                        "tcp_pose_age_s": 5.0,
                        "tcp_pose_fresh": False,
                    }
                )
            return result

    backend = StaleAfterExecutionBackend()
    result = _operator(tmp_path, backend).handle_command("move up 10 cm")

    assert result["status"] == "ABORTED"
    assert result["abort_reason"] == "E_TCP_POSE_STALE"
    assert backend.calls == 1
    assert result["completed_substep_count"] == 0
    assert result["substeps"][0]["after_tcp_pose_freshness_status"] == "BLOCKED"
    assert result["substeps"][0]["continue_allowed"] is False


def test_controller_inactive_blocks_before_segment_execution(tmp_path):
    backend = FakeBackend(state_status="BLOCKED")
    result = _operator(tmp_path, backend).handle_command("move up 10 cm")

    assert result["status"] == "ABORTED"
    assert result["abort_reason"] == "E_SCALED_CONTROLLER_INACTIVE"
    assert backend.calls == 0


def test_home_uses_same_segmented_execution_loop(tmp_path):
    backend = FakeBackend()
    operator = _operator(tmp_path, backend)
    moved = operator.handle_command("move up 10 cm")
    returned = operator.handle_command("home")

    assert moved["status"] == "PASS"
    assert returned["status"] == "PASS"
    assert returned["operator_command"] == "home"
    assert backend.pose["position_m"] == backend.home_pose["position_m"]
