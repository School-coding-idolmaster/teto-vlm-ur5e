import json
import math

import pytest

from scripts import run_real_long_motion_test as real_cli
from src.autoregressive_motion_planner import (
    AutoregressiveMotionPlannerRequest,
    plan_offline_autoregressive_motion,
)
from src.cartesian_motion_gateway import CartesianMotionGatewayRequest, evaluate_cartesian_motion_gateway
from src.guarded_vector_motion_executor import (
    GuardedVectorExecutionRequest,
    execute_guarded_vector_motion,
)
from src.motion_command_normalizer import canonicalize_delta_motion
from src.qwen_motion_parser import QwenMotionParserRequest, evaluate_qwen_motion_parser


pytestmark = [pytest.mark.safety, pytest.mark.real_path]


def test_vector_norm_and_straight_line_decomposition():
    plan = _vector_plan()

    assert plan["final_plan_status"] == "PASS"
    assert plan["motion_contract_type"] == "decomposed_relative_motion"
    assert plan["requested_distance_norm_m"] == round(math.sqrt(0.10), 6)
    assert plan["substep_count"] == 16
    assert plan["substeps"][0]["substep_delta_m"] == {"x": 0.01875, "y": 0.00625, "z": 0.0}
    assert all(step["substep_delta_norm_m"] <= 0.02 for step in plan["substeps"])
    assert plan["substeps"][-1]["cumulative_delta_after_m"] == {"x": 0.3, "y": 0.1, "z": 0.0}
    assert all(step["target_generated_from"] == "latest_verified_tcp_pose" for step in plan["substeps"][1:])


def test_single_axis_backward_compatibility_keeps_ten_substeps():
    canonical = canonicalize_delta_motion({"x": 0.20, "y": 0.0, "z": 0.0})
    plan = plan_offline_autoregressive_motion(
        AutoregressiveMotionPlannerRequest(
            canonical_motion_intent=canonical,
            current_tcp_pose=_pose(),
            config=_config(max_total=0.20),
        )
    )

    assert canonical["motion_contract_type"] == "decomposed_relative_motion"
    assert plan["substep_count"] == 10
    assert plan["direction_axis"] == "x"
    assert plan["direction_sign"] == "+"


def test_explicit_delta_json_cli_is_preview_only_without_real_flags(tmp_path, capsys):
    exit_code = real_cli.main(
        [
            "--delta-json",
            '{"x":0.30,"y":0.10,"z":0.0}',
            "--max-total-distance-m",
            "0.35",
            "--max-substep-distance-m",
            "0.02",
            "--mock-current-tcp-pose",
            '{"position_m":[0,0,0.5],"orientation_xyzw":[0,0,0,1]}',
            "--output-dir",
            str(tmp_path),
        ]
    )

    assert exit_code == 0
    summary = json.loads(capsys.readouterr().out)
    report = json.loads(next(tmp_path.glob("*.json")).read_text())
    assert summary["substep_count"] == 16
    assert summary["real_execution_enabled"] is False
    assert report["canonical_motion_intent"]["vector_source"] == "delta_json"
    assert report["real_execution"]["execute_trajectory_called"] is False
    assert report["real_execution"]["trajectory_sent"] is False


def test_mocked_qwen_vector_semantics_canonicalize_without_execution_permission():
    payload = _qwen_vector_payload()
    result = evaluate_qwen_motion_parser(
        QwenMotionParserRequest(
            user_text="move forward 30 cm and left 10 cm",
            max_distance_m=0.35,
            hard_safety_limit_m=0.35,
            llm_callable=lambda _prompt: json.dumps(payload),
        )
    )

    assert result["qwen_motion_parser_status"] == "PASS"
    assert result["delta_m"] == [0.3, 0.1, 0.0]
    assert result["vector_source"] == "qwen_semantic"
    assert result["motion_contract_type"] == "decomposed_relative_motion"
    assert result["execution_permission_decided_by_parser"] is False
    assert result["safety_gate_still_required"] is True


def test_above_vector_envelope_blocks_without_execution():
    plan = _vector_plan(delta={"x": 0.36, "y": 0.0, "z": 0.01}, max_total=0.35)

    assert plan["final_plan_status"] == "BLOCKED"
    assert plan["final_blocking_reason"] == "E_LONG_MOTION_TOTAL_EXCEEDS_LIMIT"
    assert plan["substeps"] == []
    assert plan["execute_trajectory_called"] is False


def test_vector_preview_missing_tcp_needs_current_pose():
    plan = _vector_plan(current_pose=None)

    assert plan["final_plan_status"] == "NEEDS_CURRENT_TCP"
    assert plan["substeps"][0]["current_tcp_pose_available"] is False
    assert plan["substeps"][0]["target_tcp_pose_m"] is None


def test_real_flags_absent_never_call_adapters():
    plan = _vector_plan()
    result = execute_guarded_vector_motion(
        GuardedVectorExecutionRequest(
            autoregressive_plan=plan,
            pose_reader=lambda: (_ for _ in ()).throw(AssertionError("pose reader called")),
            substep_executor=lambda *_args: (_ for _ in ()).throw(AssertionError("executor called")),
        )
    )

    assert result["final_real_execution_status"] == "NOT_REQUESTED"
    assert result["real_autoregressive_execution_enabled"] is False
    assert result["execute_trajectory_called"] is False
    assert result["trajectory_sent"] is False


def test_armed_legacy_adapters_cannot_bypass_authoritative_gateway():
    plan = _vector_plan()
    result = execute_guarded_vector_motion(
        GuardedVectorExecutionRequest(
            autoregressive_plan=plan,
            real_execution_requested=True,
            enable_real_autoregressive_execution=True,
            armed_long_motion_test=True,
            pose_reader=lambda: _pose(),
            substep_executor=lambda *_args: _success_execution(),
        )
    )

    assert result["final_real_execution_status"] == "BLOCKED"
    assert result["final_abort_reason"] == "E_AUTHORITATIVE_SUBSTEP_GATEWAY_UNAVAILABLE"
    assert result["execute_trajectory_called"] is False
    assert result["trajectory_sent"] is False


def test_armed_mock_execution_reads_and_executes_each_substep_sequentially():
    plan = _vector_plan(delta={"x": 0.03, "y": 0.01, "z": 0.0}, max_substep=0.02)
    gateway = _FakeAuthoritativeGateway()

    result = execute_guarded_vector_motion(
        GuardedVectorExecutionRequest(
            autoregressive_plan=plan,
            real_execution_requested=True,
            enable_real_autoregressive_execution=True,
            armed_long_motion_test=True,
            authoritative_substep_gateway=gateway,
        )
    )

    assert result["final_real_execution_status"] == "PASS"
    assert result["real_autoregressive_substeps_attempted"] == plan["substep_count"]
    assert result["real_autoregressive_substeps_completed"] == plan["substep_count"]
    assert gateway.calls == plan["substep_count"]
    assert all(item["substep_gateway_authoritative"] is True for item in result["per_substep_real_execution_evidence"])
    assert all(item["synthetic_safety_state_used"] is False for item in result["per_substep_real_execution_evidence"])
    assert all(item["synthetic_confirmation_used"] is False for item in result["per_substep_real_execution_evidence"])


def test_mock_verification_failure_at_fourth_attempt_aborts_later_steps():
    plan = _vector_plan(delta={"x": 0.09, "y": 0.03, "z": 0.0}, max_substep=0.02)
    gateway = _FakeAuthoritativeGateway(fail_at=4)
    result = _armed_execute(plan, gateway)

    assert result["final_real_execution_status"] == "ABORTED"
    assert result["real_autoregressive_substeps_attempted"] == 4
    assert result["real_autoregressive_substeps_completed"] == 3
    assert gateway.calls == 4
    assert result["execute_trajectory_called"] is True
    assert result["trajectory_sent"] is True
    assert result["real_execution_attempted"] is True
    assert result["real_robot_motion_executed"] is True
    assert result["post_motion_verification_failed_after_motion"] is True
    assert result["per_substep_real_execution_evidence"][3]["post_motion_verification_status"] == "FAILED"


def test_opposite_vector_projection_aborts():
    plan = _vector_plan(delta={"x": 0.03, "y": 0.01, "z": 0.0})
    gateway = _FakeAuthoritativeGateway(fail_at=1, abort_reason="E_VECTOR_DIRECTION_PROJECTION_FAILED")
    result = _armed_execute(plan, gateway)

    assert result["final_real_execution_status"] == "ABORTED"
    first = result["per_substep_real_execution_evidence"][0]
    assert first["abort_reason"] == "E_VECTOR_DIRECTION_PROJECTION_FAILED"


def test_thirty_cm_vector_routes_all_sixteen_substeps_through_authoritative_gateway():
    plan = _vector_plan()
    gateway = _FakeAuthoritativeGateway()

    result = _armed_execute(plan, gateway)

    assert plan["substep_count"] == 16
    assert gateway.calls == 16
    assert result["real_autoregressive_substeps_attempted"] == 16
    assert result["real_autoregressive_substeps_completed"] == 16
    assert all(
        item["substep_current_tcp_pose_source"] == "measured_gateway"
        for item in result["per_substep_real_execution_evidence"]
    )


def test_authoritative_gateway_rejects_synthetic_safety_or_confirmation():
    plan = _vector_plan(delta={"x": 0.03, "y": 0.01, "z": 0.0})
    gateway = _FakeAuthoritativeGateway(synthetic_safety=True)

    result = _armed_execute(plan, gateway)

    assert result["final_real_execution_status"] == "ABORTED"
    assert result["real_autoregressive_substeps_attempted"] == 1
    assert result["final_abort_reason"] == "E_SYNTHETIC_SAFETY_STATE_FORBIDDEN"


def test_real_cli_without_full_arming_blocks_without_gateway_call(tmp_path):
    exit_code = real_cli.main(
        [
            "--real",
            "--delta-json",
            '{"x":0.30,"y":0.10,"z":0.0}',
            "--max-total-distance-m",
            "0.35",
            "--output-dir",
            str(tmp_path),
        ]
    )

    report = json.loads(next(tmp_path.glob("*.json")).read_text())
    assert exit_code == 2
    assert report["real_execution"]["final_real_execution_status"] == "BLOCKED"
    assert report["real_execution"]["final_abort_reason"] == "E_REAL_AUTOREGRESSIVE_EXECUTION_NOT_ENABLED"
    assert report["real_execution"]["execute_trajectory_called"] is False


def test_real_cli_blocks_mock_pose_even_when_fully_armed(tmp_path, monkeypatch):
    gateway = _FakeAuthoritativeGateway()
    monkeypatch.setattr(real_cli, "_authoritative_real_substep_gateway", lambda: gateway)
    exit_code = real_cli.main(
        [
            "--real",
            "--enable-real-autoregressive-execution",
            "--armed-long-motion-test",
            "--delta-json",
            '{"x":0.30,"y":0.10,"z":0.0}',
            "--max-total-distance-m",
            "0.35",
            "--mock-current-tcp-pose",
            '{"position_m":[0,0,0.5],"orientation_xyzw":[0,0,0,1]}',
            "--output-dir",
            str(tmp_path),
        ],
    )

    report = json.loads(next(tmp_path.glob("*.json")).read_text())
    assert exit_code == 2
    assert report["real_execution"]["final_abort_reason"] == "E_MOCK_CURRENT_TCP_POSE_NOT_ALLOWED_FOR_REAL_EXECUTION"
    assert gateway.calls == 0


def test_real_cli_blocks_when_authoritative_gateway_unavailable(tmp_path):
    exit_code = real_cli.main(
        [
            "--real",
            "--enable-real-autoregressive-execution",
            "--armed-long-motion-test",
            "--delta-json",
            '{"x":0.30,"y":0.10,"z":0.0}',
            "--max-total-distance-m",
            "0.35",
            "--output-dir",
            str(tmp_path),
        ]
    )

    report = json.loads(next(tmp_path.glob("*.json")).read_text())
    assert exit_code == 2
    assert report["real_execution"]["final_abort_reason"] == "E_AUTHORITATIVE_SUBSTEP_GATEWAY_UNAVAILABLE"
    assert report["real_execution"]["trajectory_sent"] is False


def test_real_cli_fake_authoritative_gateway_is_called_per_substep(tmp_path, monkeypatch):
    gateway = _FakeAuthoritativeGateway()
    monkeypatch.setattr(real_cli, "_authoritative_real_substep_gateway", lambda: gateway)
    exit_code = real_cli.main(
        [
            "--real",
            "--enable-real-autoregressive-execution",
            "--armed-long-motion-test",
            "--delta-json",
            '{"x":0.30,"y":0.10,"z":0.0}',
            "--max-total-distance-m",
            "0.35",
            "--output-dir",
            str(tmp_path),
        ],
    )

    report = json.loads(next(tmp_path.glob("*.json")).read_text())
    assert exit_code == 0
    assert gateway.calls == 16
    assert report["real_execution"]["real_autoregressive_substeps_completed"] == 16
    assert report["safety_confirmation"]["synthetic_safety_state_used"] is False
    assert report["safety_confirmation"]["synthetic_confirmation_used"] is False


def test_long_vector_one_shot_permission_remains_false():
    plan = _vector_plan()

    assert plan["max_one_shot_distance_m"] == 0.05
    assert plan["one_shot_target_pose_created"] is False
    assert plan["one_shot_real_motion_allowed"] is False
    assert plan["planned_execution_style"] == "decomposed_autoregressive_vector_preview"


def test_gateway_blocks_thirty_cm_vector_as_one_shot():
    result = evaluate_cartesian_motion_gateway(
        CartesianMotionGatewayRequest(
            requested=True,
            config={"max_translation_m": 0.05, "hard_safety_limit_m": 0.05},
            command_to_task_result=_gateway_task([0.30, 0.10, 0.0]),
            current_tcp_pose=_pose(),
        )
    )

    assert result["cartesian_motion_gateway_status"] == "BLOCKED"
    assert result["motion_contract_type"] == "vector_relative"
    assert result["requested_distance_norm_m"] == round(math.sqrt(0.10), 6)
    assert result["one_shot_vector_motion_allowed"] is False
    assert result["target_pose"] is None


def _vector_plan(
    *,
    delta=None,
    current_pose="default",
    max_total=0.35,
    max_substep=0.02,
):
    delta = delta or {"x": 0.30, "y": 0.10, "z": 0.0}
    return plan_offline_autoregressive_motion(
        AutoregressiveMotionPlannerRequest(
            canonical_motion_intent=canonicalize_delta_motion(delta),
            current_tcp_pose=_pose() if current_pose == "default" else current_pose,
            config=_config(max_total=max_total, max_substep=max_substep),
        )
    )


def _config(*, max_total=0.35, max_substep=0.02):
    return {
        "enable_long_step_decomposition": True,
        "max_one_shot_distance_m": 0.05,
        "max_decomposed_total_distance_m": max_total,
        "max_decomposed_substep_distance_m": max_substep,
        "substep_execution_mode": "offline_preview",
        "workspace_bounds": {"x": [-1.0, 1.0], "y": [-1.0, 1.0], "z": [0.0, 2.0]},
    }


def _pose():
    return {
        "frame": "base_link",
        "position_m": [0.0, 0.0, 0.5],
        "orientation_xyzw": [0.0, 0.0, 0.0, 1.0],
    }


def _armed_execute(plan, gateway):
    return execute_guarded_vector_motion(
        GuardedVectorExecutionRequest(
            autoregressive_plan=plan,
            real_execution_requested=True,
            enable_real_autoregressive_execution=True,
            armed_long_motion_test=True,
            authoritative_substep_gateway=gateway,
        )
    )


class _FakeAuthoritativeGateway:
    def __init__(
        self,
        *,
        fail_at=None,
        abort_reason="E_POST_MOTION_VERIFICATION_FAILED",
        synthetic_safety=False,
        synthetic_confirmation=False,
    ):
        self.pose = _pose()
        self.calls = 0
        self.fail_at = fail_at
        self.abort_reason = abort_reason
        self.synthetic_safety = synthetic_safety
        self.synthetic_confirmation = synthetic_confirmation

    def read_current_pose(self):
        return {**self.pose, "position_m": list(self.pose["position_m"])}

    def __call__(self, step):
        self.calls += 1
        pre = self.read_current_pose()
        delta = list(step["target_delta_m"])
        target = {
            **pre,
            "position_m": [
                round(pre["position_m"][index] + delta[index], 6)
                for index in range(3)
            ],
        }
        failed = self.calls == self.fail_at
        if not failed:
            self.pose = target
        return {
            "substep_gateway_called": True,
            "substep_gateway_authoritative": True,
            "substep_current_tcp_pose_source": "measured_gateway",
            "synthetic_safety_state_used": self.synthetic_safety,
            "synthetic_confirmation_used": self.synthetic_confirmation,
            "target_generated_from": (
                "measured_current_tcp_pose"
                if self.calls == 1
                else "latest_verified_measured_tcp_pose"
            ),
            "pre_step_tcp_pose": pre,
            "target_tcp_pose": target,
            "post_step_tcp_pose": pre if failed else target,
            "gateway_result_status": "BLOCKED" if failed else "PASS",
            "gateway_blocking_reason": self.abort_reason if failed else None,
            "execute_trajectory_called": True,
            "trajectory_sent": True,
            "real_execution_attempted": True,
            "real_motion_command_sent": True,
            "real_robot_motion_executed": True,
            "real_robot_motion_executed_evidence_source": "fake_authoritative_gateway",
            "post_motion_verification_status": "FAILED" if failed else "PASS",
            "continue_allowed": not failed,
        }


def _success_execution():
    return {
        "moveit_pose_executor_status": "PASS",
        "execute_success": True,
        "moveit_execute_called": True,
        "trajectory_sent": True,
        "real_robot_motion_executed": True,
    }


def _qwen_vector_payload():
    return {
        "schema_version": "teto_motion_semantics.v1",
        "intent_status": "ok",
        "intent_type": "relative_cartesian_motion",
        "motion": {
            "mode": "vector_delta",
            "direction_semantic": "unknown",
            "distance": {"quality": "missing"},
            "delta": {
                "x": {"value": 30, "unit": "cm", "meters": 0.30, "quality": "explicit"},
                "y": {"value": 10, "unit": "cm", "meters": 0.10, "quality": "explicit"},
                "z": {"value": 0, "unit": "m", "meters": 0.0, "quality": "explicit"},
            },
        },
        "confidence": {"intent": 0.95, "direction": 0.95, "distance": 0.95, "overall": 0.95},
        "language": "en",
    }


def _gateway_task(delta):
    return {
        "command_to_task_status": "PASS",
        "task_contract": {
            "intent": "cartesian_offset",
            "frame": "base_link",
            "cartesian_offset_m": delta,
        },
    }
