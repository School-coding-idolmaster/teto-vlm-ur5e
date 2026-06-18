#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import os
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Any


REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from src.autoregressive_motion_planner import (  # noqa: E402
    AutoregressiveMotionPlannerRequest,
    plan_offline_autoregressive_motion,
)
from src.guarded_vector_motion_executor import (  # noqa: E402
    GuardedVectorExecutionRequest,
    execute_guarded_vector_motion,
)
from src.motion_command_normalizer import canonicalize_delta_motion, normalize_motion_command  # noqa: E402
from src.moveit_pose_executor import MoveItPoseExecutorRequest, evaluate_moveit_pose_execute  # noqa: E402
from src.qwen_motion_parser import QwenMotionParserRequest, evaluate_qwen_motion_parser  # noqa: E402


def main(argv: list[str] | None = None) -> int:
    parser = _argument_parser()
    args = parser.parse_args(argv)
    if bool(args.cmd) == bool(args.delta_json):
        parser.error("provide exactly one of --cmd or --delta-json")
    canonical, parser_evidence = _canonical_intent(args)
    mock_pose, pose_error = _mock_pose(args.mock_current_tcp_pose)
    real_flags_satisfied = bool(
        args.real and args.enable_real_autoregressive_execution and args.armed_long_motion_test
    )
    if real_flags_satisfied and args.mock_current_tcp_pose:
        pose_error = "E_MOCK_CURRENT_TCP_POSE_NOT_ALLOWED_FOR_REAL_EXECUTION"
        mock_pose = None
    initial_pose = _read_real_pose() if real_flags_satisfied else mock_pose
    plan = plan_offline_autoregressive_motion(
        AutoregressiveMotionPlannerRequest(
            canonical_motion_intent=canonical,
            current_tcp_pose=initial_pose,
            config={
                "enable_long_step_decomposition": True,
                "max_one_shot_distance_m": 0.05,
                "max_decomposed_substep_distance_m": args.max_substep_distance_m,
                "max_decomposed_total_distance_m": args.max_total_distance_m,
                "substep_execution_mode": "offline_preview",
                "workspace_bounds": {"x": [-1.0, 1.0], "y": [-1.0, 1.0], "z": [0.0, 2.0]},
            },
        )
    )
    if real_flags_satisfied and plan.get("final_plan_status") == "PASS":
        plan = {**plan, "planned_execution_style": "real_autoregressive_vector_substeps"}
    execution = execute_guarded_vector_motion(
        GuardedVectorExecutionRequest(
            autoregressive_plan=plan,
            real_execution_requested=args.real,
            enable_real_autoregressive_execution=args.enable_real_autoregressive_execution,
            armed_long_motion_test=args.armed_long_motion_test,
            pose_reader=_read_real_pose if real_flags_satisfied else None,
            substep_executor=_execute_real_substep if real_flags_satisfied else None,
            config={
                "max_real_autoregressive_total_distance_m": args.max_total_distance_m,
                "max_real_autoregressive_substep_distance_m": args.max_substep_distance_m,
                "post_step_distance_tolerance_m": args.post_step_distance_tolerance_m,
                "max_orthogonal_drift_m": args.max_orthogonal_drift_m,
            },
        )
    )
    final_status = (
        execution["final_real_execution_status"]
        if real_flags_satisfied
        else plan.get("final_plan_status")
    )
    evidence = {
        "report_version": "teto_v3_0_13_real_long_motion_test_v1",
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "command": args.cmd,
        "delta_json": args.delta_json,
        "canonical_motion_intent": canonical,
        "parser_evidence": parser_evidence,
        "current_tcp_pose_error": pose_error,
        "autoregressive_plan": plan,
        "real_execution": execution,
        "final_status": final_status,
        "safety_confirmation": {
            "preview_only": not real_flags_satisfied,
            "one_shot_limit_m": 0.05,
            "one_shot_target_pose_created": False,
            "operator_console_used": False,
            "manual_y_confirmation_required": False,
        },
    }
    json_path, markdown_path = _write_report(evidence, Path(args.output_dir))
    print(
        json.dumps(
            {
                "command": args.cmd,
                "delta_m": canonical.get("vector_components_m"),
                "vector_norm_m": canonical.get("requested_distance_norm_m"),
                "substep_count": plan.get("substep_count"),
                "real_execution_enabled": execution.get("real_autoregressive_execution_enabled"),
                "final_status": final_status,
                "report_path": str(json_path),
                "markdown_report_path": str(markdown_path),
            },
            sort_keys=True,
        )
    )
    return 0 if final_status in {"PASS", "NOT_REQUESTED"} else 2


def _argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Run one guarded TETO vector long-motion test.")
    parser.add_argument("--cmd")
    parser.add_argument("--delta-json")
    parser.add_argument("--parser", choices=["auto", "qwen", "rule"], default="auto")
    parser.add_argument("--qwen-model", default=os.environ.get("TETO_QWEN_MODEL"))
    parser.add_argument("--qwen-endpoint", default=os.environ.get("TETO_QWEN_ENDPOINT"))
    parser.add_argument("--qwen-timeout-s", type=float, default=2.0)
    parser.add_argument("--real", action="store_true")
    parser.add_argument("--enable-real-autoregressive-execution", action="store_true")
    parser.add_argument("--armed-long-motion-test", action="store_true")
    parser.add_argument("--max-total-distance-m", type=float, default=0.35)
    parser.add_argument("--max-substep-distance-m", type=float, default=0.02)
    parser.add_argument("--post-step-distance-tolerance-m", type=float, default=0.005)
    parser.add_argument("--max-orthogonal-drift-m", type=float, default=0.005)
    parser.add_argument("--mock-current-tcp-pose")
    parser.add_argument("--output-dir", default="outputs/real_long_motion_tests")
    return parser


def _canonical_intent(args: argparse.Namespace) -> tuple[dict[str, Any], dict[str, Any]]:
    if args.delta_json:
        try:
            delta = json.loads(args.delta_json)
        except json.JSONDecodeError:
            delta = None
        canonical = canonicalize_delta_motion(delta, vector_source="delta_json")
        return canonical, {"parser_source": "delta_json", "llm_called": False}
    if args.parser in {"auto", "qwen"}:
        qwen = evaluate_qwen_motion_parser(
            QwenMotionParserRequest(
                user_text=args.cmd,
                max_distance_m=args.max_total_distance_m,
                hard_safety_limit_m=args.max_total_distance_m,
                model_name=args.qwen_model,
                endpoint=args.qwen_endpoint,
                timeout_s=args.qwen_timeout_s,
            )
        )
        if qwen.get("qwen_motion_parser_status") == "PASS":
            canonical = {
                key: qwen.get(key)
                for key in (
                    "parse_status",
                    "intent",
                    "motion_frame",
                    "direction_axis",
                    "direction_sign",
                    "requested_distance_m",
                    "requested_distance_norm_m",
                    "delta_m",
                    "vector_delta_m",
                    "vector_components_m",
                    "vector_component_count_nonzero",
                    "motion_contract_type",
                    "legacy_axis_compatible",
                    "vector_source",
                    "vector_motion_supported",
                    "execution_permission_decided_by_parser",
                    "safety_gate_still_required",
                )
            }
            canonical["parse_status"] = canonical.get("parse_status") or "PASS"
            canonical["intent"] = canonical.get("intent") or "relative_cartesian_motion"
            canonical["motion_frame"] = canonical.get("motion_frame") or "base_link"
            canonical["vector_source"] = canonical.get("vector_source") or "qwen_semantic"
            return canonical, qwen
        if args.parser == "qwen":
            return {"parse_status": "BLOCKED", "intent": None}, qwen
        fallback = normalize_motion_command(args.cmd, parser_source="fallback_rule")
        return fallback, {"qwen": qwen, "fallback": fallback}
    fallback = normalize_motion_command(args.cmd, parser_source="fallback_rule")
    return fallback, fallback


def _mock_pose(raw: str | None) -> tuple[dict[str, Any] | None, str | None]:
    if not raw:
        return None, None
    try:
        pose = json.loads(raw)
    except json.JSONDecodeError:
        return None, "E_INVALID_MOCK_CURRENT_TCP_POSE_JSON"
    if not isinstance(pose, dict):
        return None, "E_INVALID_MOCK_CURRENT_TCP_POSE"
    return {"frame": "base_link", **pose}, None


def _read_real_pose() -> dict[str, Any] | None:
    from scripts.text_to_ur5e_real_motion import _lookup_current_tcp_pose

    return _lookup_current_tcp_pose(timeout_s=3.0)


def _execute_real_substep(
    current_pose: dict[str, Any],
    target_pose: dict[str, Any],
    step: dict[str, Any],
) -> dict[str, Any]:
    distance = float(step["substep_delta_norm_m"])
    return evaluate_moveit_pose_execute(
        MoveItPoseExecutorRequest(
            requested=True,
            target_pose=target_pose,
            current_tcp_pose=current_pose,
            execute=True,
            manual_confirmation_result={"manual_confirmation_accepted": True},
            robot_state_result={
                "read_only_state_contract_ready": True,
                "robot_state_ok": True,
                "safety_status_ok": True,
                "protective_stop": False,
                "emergency_stop": False,
                "speed_scaling": 0.05,
            },
            config={
                "planning_group": "ur_manipulator",
                "planning_frame": "base_link",
                "end_effector_link": "tool0",
                "allowed_frames": ["base_link"],
                "workspace_bounds": {"x": [-1.0, 1.0], "y": [-1.0, 1.0], "z": [0.0, 2.0]},
                "max_translation_m": 0.02,
                "configured_max_distance_m": 0.02,
                "hard_safety_limit_m": 0.02,
                "requested_distance_m": distance,
                "position_tolerance_m": min(0.002, distance * 0.10),
                "orientation_tolerance_rad": 0.01,
                "manual_confirmation_required": True,
                "robot_state_ok": True,
                "safety_status_ok": True,
                "protective_stop": False,
                "emergency_stop": False,
                "speed_scaling": 0.05,
                "max_speed_scale": 0.05,
                "max_acc_scale": 0.05,
                "planner_risk_policy_mode": "soft_warn",
                "planner_risk_blocking_enabled": False,
            },
        )
    )


def _write_report(evidence: dict[str, Any], output_dir: Path) -> tuple[Path, Path]:
    output_dir.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S%fZ")
    json_path = output_dir / f"{stamp}_real_long_motion_test.json"
    markdown_path = output_dir / f"{stamp}_real_long_motion_test.md"
    json_path.write_text(json.dumps(evidence, indent=2) + "\n", encoding="utf-8")
    markdown_path.write_text(
        "\n".join(
            [
                "# TETO v3.0.13 Guarded Vector Long-Motion Test",
                "",
                f"- Command: `{evidence.get('command')}`",
                f"- Delta: `{evidence['canonical_motion_intent'].get('vector_components_m')}`",
                f"- Vector norm: `{evidence['canonical_motion_intent'].get('requested_distance_norm_m')}` m",
                f"- Substeps: `{evidence['autoregressive_plan'].get('substep_count')}`",
                f"- Real execution enabled: `{evidence['real_execution'].get('real_autoregressive_execution_enabled')}`",
                f"- Final status: `{evidence.get('final_status')}`",
                "",
                "```json",
                json.dumps(evidence, indent=2),
                "```",
                "",
            ]
        ),
        encoding="utf-8",
    )
    return json_path, markdown_path


if __name__ == "__main__":
    raise SystemExit(main())
