#!/usr/bin/env python3
from __future__ import annotations

# LEGACY MANUAL REAL PATH ONLY.
# This is not the current default real path. Current real default:
# scripts/start_teto_real_full_stack.sh / scripts/teto_operator_console.py.
# Current Isaac default: scripts/start_teto_isaac_gui_operator.sh.
# Do not use dry-run, plan-only, fake, or Isaac evidence as REAL_PATH success
# evidence. REAL_PATH success from this legacy path requires explicit real
# legacy manual routing plus measured real execution evidence.

import argparse
import json
import math
import os
import re
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any


REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from src.cartesian_motion_gateway import (  # noqa: E402
    CartesianMotionExecutionRequest,
    CartesianMotionGatewayRequest,
    evaluate_cartesian_motion_execution,
    evaluate_cartesian_motion_gateway,
)
from src.moveit_pose_executor import (  # noqa: E402
    DEFAULT_PLANNER_RISK_POLICY_MODE,
    DEFAULT_PLANNER_RISK_POLICY_NAME,
    DEFAULT_WARN_MAX_JOINT_DELTA_RAD,
    DEFAULT_WARN_MAX_WRIST_JOINT_DELTA_RAD,
    DEFAULT_WARN_PATH_LENGTH_RATIO,
    evaluate_planner_audit_risk,
)
from src.motion_command_normalizer import (  # noqa: E402
    DEFAULT_SMALL_STEP_M,
    POLICY_VERSION as MOTION_LANGUAGE_POLICY_VERSION,
    normalize_motion_command,
)
from src.qwen_motion_parser import (  # noqa: E402
    QwenMotionParserRequest,
    evaluate_qwen_motion_parser,
)


DEFAULT_REAL_MAX_DISTANCE_M = 0.05
DEFAULT_REAL_HARD_SAFETY_LIMIT_M = 0.05
DEFAULT_REAL_POSITION_TOLERANCE_M = 0.002
DEFAULT_REAL_ORIENTATION_TOLERANCE_RAD = 0.01
REAL_MOTION_TOLERANCE_DISTANCE_RATIO = 0.10
DEFAULT_MAX_AXIS_STEP_M = DEFAULT_REAL_MAX_DISTANCE_M
DEFAULT_SESSION_RADIUS_LIMIT_M = None
DEFAULT_TCP_POSE_BASE_FRAME = "base_link"
DEFAULT_TCP_POSE_TOOL_FRAME = "tool0"
DEFAULT_MOVEIT_PLANNING_FRAME = DEFAULT_TCP_POSE_BASE_FRAME
DEFAULT_MOVEIT_END_EFFECTOR_LINK = DEFAULT_TCP_POSE_TOOL_FRAME
DEFAULT_SAFETY_POLICY_NAME = "lab_directional_step_motion_v1"
DEFAULT_SAFETY_POLICY_SOURCE = "cli_defaults"
DEFAULT_MOTION_PERMISSION_ENVELOPE_VERSION = "teto_v3_0_9_expanded_decomposed_contract_preview"
EPS = 1e-9
CONFIRMATION_REPLY = "y"
BASE_LINK_DIRECTION_MAPPING = {
    "forward": {"axis": "x", "sign": "+", "delta_index": 0, "base_link_direction": "+X"},
    "backward": {"axis": "x", "sign": "-", "delta_index": 0, "base_link_direction": "-X"},
    "left": {"axis": "y", "sign": "+", "delta_index": 1, "base_link_direction": "+Y"},
    "right": {"axis": "y", "sign": "-", "delta_index": 1, "base_link_direction": "-Y"},
    "up": {"axis": "z", "sign": "+", "delta_index": 2, "base_link_direction": "+Z"},
    "down": {"axis": "z", "sign": "-", "delta_index": 2, "base_link_direction": "-Z"},
}
REAL_SMALL_MOTION_ALLOWED_AXES = {"x", "y", "z"}
REAL_SMALL_MOTION_ALLOWED_DIRECTIONS = {"+", "-"}
REAL_SMALL_MOTION_GATE_POLICY = (
    "lab_directional_step_motion_v1:"
    "intent=relative_cartesian_motion;frame=base_link;axis=x|y|z;direction=+|-;"
    "step_delta_m<=max_step_distance_m;axis_delta_m<=max_axis_step_m;must_confirm=true"
)
MOCK_CURRENT_TCP_POSE_FOR_DRY_RUN_ONLY = {
    "available": True,
    "frame": "base_link",
    "position_m": [-0.154964, 0.312309, 1.046042],
    "orientation_xyzw": [-0.707084555167031, 0.0060696987353325745, 0.005069564207997603, 0.7070847828374228],
    "source": "mock_current_tcp_pose_for_dry_run_only",
    "allowed_for_real_execution": False,
}

STATUS_PASS = "PASS"
STATUS_BLOCKED = "BLOCKED"
STATUS_FAILED = "FAILED"
STATUS_WARNING = "WARNING"

SUSPICIOUS_TINY_MOTION_JOINT_DELTA_RAD = 1.0
SUSPICIOUS_TINY_MOTION_ORIENTATION_CHANGE_RAD = 0.25
POST_EXECUTE_TCP_SETTLE_S = 0.25
POST_EXECUTE_TCP_SAMPLE_ATTEMPTS = 3
TCP_POSE_STALE_AFTER_S = 1.0


class MotionParseError(ValueError):
    pass


@dataclass(frozen=True)
class ParsedMotionCommand:
    command: str
    frame: str
    delta_m: list[float]
    distance_m: float
    max_distance_m: float
    hard_safety_limit_m: float
    direction: str
    unit: str
    parser_source: str = "rule_based"
    llm_called: bool = False
    model_name: str | None = None
    qwen_endpoint: str | None = None
    llm_latency_ms: float | None = None
    raw_llm_output: str | None = None
    parser_blocking_reasons: list[str] | None = None
    natural_language_evidence: dict[str, Any] | None = None

    def task_contract(self, *, real_robot_motion_requested: bool, dry_run: bool) -> dict[str, Any]:
        return {
            "intent": "relative_cartesian_motion",
            "frame": self.frame,
            "delta_m": [round(value, 6) for value in self.delta_m],
            "max_distance_m": self.max_distance_m,
            "hard_safety_limit_m": self.hard_safety_limit_m,
            "must_confirm": True,
            "real_robot_motion_requested": real_robot_motion_requested,
            "dry_run": dry_run,
        }

    def execution_preview(
        self,
        *,
        input_mode: str,
        parser_mode: str,
        real_robot_motion_requested: bool,
        dry_run: bool,
        manual_confirmation_required: bool = True,
    ) -> dict[str, Any]:
        axis, sign = _axis_direction_from_delta(self.delta_m)
        within_safety_limit = (
            self.distance_m <= self.max_distance_m + EPS
            and self.distance_m <= self.hard_safety_limit_m + EPS
        )
        language = self.natural_language_evidence if isinstance(self.natural_language_evidence, dict) else {}
        return {
            "original_command": self.command,
            "input_mode": input_mode,
            "parser_mode": parser_mode,
            "parser_source": self.parser_source,
            "natural_language_coverage_version": language.get("natural_language_coverage_version"),
            "motion_language_policy_version": language.get("motion_language_policy_version"),
            "raw_command": language.get("raw_command", self.command),
            "normalized_command": language.get("normalized_command"),
            "parse_status": language.get("parse_status", STATUS_PASS),
            "distance_source": language.get("distance_source", "explicit"),
            "direction_source": language.get("direction_source", "explicit_direction_word"),
            "inferred_default_distance_m": language.get("inferred_default_distance_m"),
            "motion_parse_confidence": language.get("motion_parse_confidence"),
            "requires_confirmation": language.get("requires_confirmation", True),
            "safety_gate_still_required": language.get("safety_gate_still_required", True),
            "execution_permission_decided_by_parser": language.get("execution_permission_decided_by_parser", False),
            "llm_called": self.llm_called,
            "model_name": self.model_name,
            "endpoint": self.qwen_endpoint,
            "intent": "relative_cartesian_motion",
            "frame": self.frame,
            "axis": axis,
            "direction": sign,
            "distance_m": self.distance_m,
            "delta_m": [round(value, 6) for value in self.delta_m],
            "max_distance_m": self.max_distance_m,
            "hard_safety_limit_m": self.hard_safety_limit_m,
            "within_safety_limit": within_safety_limit,
            "dry_run": dry_run,
            "real_robot_motion_requested": real_robot_motion_requested,
            "manual_confirmation_required": manual_confirmation_required,
            "preview_status": STATUS_PASS if within_safety_limit else STATUS_BLOCKED,
        }

    def gateway_task(self) -> dict[str, Any]:
        dx, dy, dz = self.delta_m
        return {
            "command_to_task_status": STATUS_PASS,
            "intent": "cartesian_offset",
            "frame": self.frame,
            "dx": dx,
            "dy": dy,
            "dz": dz,
            "cartesian_offset_m": list(self.delta_m),
            "task_contract": {
                "intent": "cartesian_offset",
                "frame": self.frame,
                "dx": dx,
                "dy": dy,
                "dz": dz,
                "cartesian_offset_m": list(self.delta_m),
            },
            "blocking_reasons": [],
            "warnings": [],
        }


def parse_motion_command(
    command: str,
    *,
    max_step_m: float | None = None,
    max_distance_m: float | None = None,
    hard_safety_limit_m: float = DEFAULT_REAL_HARD_SAFETY_LIMIT_M,
    default_small_step_m: float = DEFAULT_SMALL_STEP_M,
) -> ParsedMotionCommand:
    configured_max_distance_m = (
        float(max_distance_m)
        if max_distance_m is not None
        else float(max_step_m)
        if max_step_m is not None
        else DEFAULT_REAL_MAX_DISTANCE_M
    )
    if not isinstance(command, str) or not command.strip():
        raise MotionParseError("E_EMPTY_COMMAND")
    if (
        configured_max_distance_m <= 0.0
        or hard_safety_limit_m <= 0.0
        or configured_max_distance_m > hard_safety_limit_m + EPS
    ):
        raise MotionParseError("E_INVALID_MAX_DISTANCE")

    normalized = _normalize(command)

    try:
        _reject_forbidden(normalized)
        distance_m, unit = _extract_distance_m(normalized)
        direction, delta = _extract_direction_delta(normalized, distance_m)
        language_evidence = _rule_language_evidence(
            command=command,
            normalized=normalized,
            distance_m=distance_m,
            unit=unit,
            delta=delta,
            distance_source="explicit",
            direction_source="explicit_direction_word",
            confidence=0.92,
        )
    except MotionParseError:
        language_evidence = normalize_motion_command(
            command,
            default_small_step_m=default_small_step_m,
            parser_source="normalizer",
        )
        if language_evidence.get("parse_status") != STATUS_PASS:
            raise MotionParseError(_motion_parse_error_from_language(language_evidence))
        distance_m = float(language_evidence["requested_distance_m"])
        unit = str(language_evidence.get("unit") or language_evidence.get("distance_source") or "normalized")
        delta = [float(value) for value in language_evidence["delta_m"]]
        direction = f"{language_evidence.get('direction_axis')}{language_evidence.get('direction_sign')}"
    if distance_m > hard_safety_limit_m + EPS:
        raise MotionParseError("E_EXCEEDS_HARD_SAFETY_LIMIT")
    if distance_m > configured_max_distance_m + EPS:
        raise MotionParseError("E_EXCEEDS_MAX_STEP")
    if not any(abs(value) > 0.0 for value in delta):
        raise MotionParseError("E_INVALID_ZERO_MOTION")

    return ParsedMotionCommand(
        command=command.strip(),
        frame="base_link",
        delta_m=[round(value, 6) for value in delta],
        distance_m=round(distance_m, 6),
        max_distance_m=configured_max_distance_m,
        hard_safety_limit_m=hard_safety_limit_m,
        direction=direction,
        unit=unit,
        natural_language_evidence=language_evidence,
    )


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Interactive text-to-UR5e relative Cartesian motion entrypoint.")
    parser.add_argument("--real", action="store_true", help="Enable real MoveIt ExecuteTrajectory.")
    parser.add_argument("--dry-run", action="store_true", help="Parse and validate without executing.")
    parser.add_argument("--plan-only-smoke", action="store_true", help="Request MoveIt planning only; ExecuteTrajectory remains disabled.")
    parser.add_argument("--acceptance", action="store_true", help="Emit v3.0.2 unified Qwen manual acceptance workflow evidence.")
    parser.add_argument("--real-small-motion", action="store_true", help="Guarded future real small-motion acceptance path; requires manual confirmation.")
    parser.add_argument("--check-tcp-pose-readiness", action="store_true", help="Read-only current TCP pose readiness check.")
    parser.add_argument("--mock-current-tcp-pose", action="store_true", help="Use the fixed dry-run-only mock TCP pose; rejected for real motion.")
    parser.add_argument("--current-tcp-pose-json", help="Explicit current TCP pose JSON for dry-run/simulation only; rejected for real motion.")
    parser.add_argument("--cmd", help='One-shot command, for example: --cmd "move up 5 mm".')
    parser.add_argument("--yes", action="store_true", help="Skip real-mode confirmation; only allowed together with --real --cmd.")
    parser.add_argument("--parser", choices=["rule", "qwen", "auto"], default="qwen", help="Text parser mode. Default: qwen.")
    parser.add_argument("--qwen-model", default=os.environ.get("TETO_QWEN_MODEL"), help="Qwen/Ollama model name.")
    parser.add_argument("--qwen-endpoint", default=os.environ.get("TETO_QWEN_ENDPOINT"), help="Optional Ollama-compatible endpoint.")
    parser.add_argument("--qwen-timeout-s", type=float, default=_env_float("TETO_QWEN_TIMEOUT_S"))
    parser.add_argument("--default-small-step-m", type=float, default=DEFAULT_SMALL_STEP_M)
    parser.add_argument(
        "--max-distance-m",
        "--max-step-m",
        "--max-step-distance-m",
        dest="max_distance_m",
        type=float,
        default=DEFAULT_REAL_MAX_DISTANCE_M,
        help="Configured maximum per-step relative TCP translation in meters.",
    )
    parser.add_argument("--max-axis-step-m", type=float, default=DEFAULT_MAX_AXIS_STEP_M)
    parser.add_argument("--hard-safety-limit-m", type=float, default=DEFAULT_REAL_HARD_SAFETY_LIMIT_M)
    parser.add_argument("--session-radius-limit-m", type=float, default=DEFAULT_SESSION_RADIUS_LIMIT_M)
    parser.add_argument("--position-tolerance-m", type=float, default=DEFAULT_REAL_POSITION_TOLERANCE_M)
    parser.add_argument("--orientation-tolerance-rad", type=float, default=DEFAULT_REAL_ORIENTATION_TOLERANCE_RAD)
    parser.add_argument("--safety-policy-name", default=DEFAULT_SAFETY_POLICY_NAME)
    parser.add_argument("--previous-verified-tcp-pose-json", help="Previous verified TCP pose JSON for step-policy evidence.")
    parser.add_argument("--tcp-pose-base-frame", default=DEFAULT_TCP_POSE_BASE_FRAME)
    parser.add_argument("--tcp-pose-tool-frame", default=DEFAULT_TCP_POSE_TOOL_FRAME)
    parser.add_argument("--moveit-planning-frame", default=DEFAULT_MOVEIT_PLANNING_FRAME)
    parser.add_argument("--moveit-end-effector-link", default=DEFAULT_MOVEIT_END_EFFECTOR_LINK)
    parser.add_argument("--planner-risk-policy", default=DEFAULT_PLANNER_RISK_POLICY_NAME)
    parser.add_argument(
        "--planner-risk-mode",
        choices=["evidence_only", "soft_warn", "hard_block"],
        default=DEFAULT_PLANNER_RISK_POLICY_MODE,
    )
    parser.add_argument("--warn-max-joint-delta-rad", type=float, default=DEFAULT_WARN_MAX_JOINT_DELTA_RAD)
    parser.add_argument("--warn-max-wrist-joint-delta-rad", type=float, default=DEFAULT_WARN_MAX_WRIST_JOINT_DELTA_RAD)
    parser.add_argument("--warn-path-length-ratio", type=float, default=DEFAULT_WARN_PATH_LENGTH_RATIO)
    parser.add_argument("--enable-planner-risk-blocking", action="store_true")
    parser.add_argument("--enable-long-step-decomposition", action="store_true")
    parser.add_argument("--motion-permission-envelope-version", default=DEFAULT_MOTION_PERMISSION_ENVELOPE_VERSION)
    parser.add_argument("--max-one-shot-distance-m", type=float, default=DEFAULT_REAL_MAX_DISTANCE_M)
    parser.add_argument("--long-step-threshold-m", type=float, default=0.05)
    parser.add_argument("--max-substep-distance-m", "--max-decomposed-substep-distance-m", type=float, default=0.02)
    parser.add_argument("--min-final-substep-distance-m", type=float, default=0.001)
    parser.add_argument("--long-motion-total-limit-m", "--max-decomposed-total-distance-m", type=float, default=0.50)
    parser.add_argument("--substep-execution-mode", choices=["contract_only"], default="contract_only")
    parser.add_argument("--speed-scale", type=float, default=0.10)
    parser.add_argument("--acc-scale", type=float, default=0.10)
    args = parser.parse_args(argv)
    safety_policy = _safety_policy_from_args(args)
    frame_config = _frame_config_from_args(args)
    planner_risk_policy = _planner_risk_policy_from_args(args)

    real_requested = bool(args.real or args.real_small_motion)
    acceptance_mode = _acceptance_mode(args)

    if real_requested and args.dry_run:
        print(_json({"final_status": STATUS_BLOCKED, "blocking_reasons": ["E_REAL_AND_DRY_RUN_CONFLICT"], "real_robot_motion_executed": False, **_post_motion_not_run_fields()}))
        return 2
    if real_requested and args.plan_only_smoke:
        print(_json({"final_status": STATUS_BLOCKED, "blocking_reasons": ["E_REAL_AND_PLAN_ONLY_SMOKE_CONFLICT"], "real_robot_motion_executed": False, **_post_motion_not_run_fields()}))
        return 2
    if args.real_small_motion and args.yes:
        print(_json({"final_status": STATUS_BLOCKED, "blocking_reasons": ["E_REAL_SMALL_MOTION_REQUIRES_MANUAL_CONFIRMATION"], "real_robot_motion_executed": False, **_post_motion_not_run_fields()}))
        return 2
    if args.acceptance and args.real and not args.real_small_motion:
        print(_json({"final_status": STATUS_BLOCKED, "blocking_reasons": ["E_ACCEPTANCE_REAL_REQUIRES_REAL_SMALL_MOTION"], "real_robot_motion_executed": False, **_post_motion_not_run_fields()}))
        return 2
    if args.check_tcp_pose_readiness:
        pose, evidence, blocker = _resolve_current_tcp_pose(args, real_requested=False)
        readiness_status = STATUS_PASS if pose is not None and blocker is None else STATUS_BLOCKED
        result = {
            "final_status": readiness_status,
            "tcp_pose_readiness_status": readiness_status,
            "current_tcp_pose": evidence,
            "blocking_reasons": [blocker or "E_CURRENT_TCP_POSE_MISSING"] if readiness_status != STATUS_PASS else [],
            "execute_trajectory_called": False,
            "trajectory_sent": False,
            "controller_command_sent": False,
            "real_robot_motion_executed": False,
            "manual_confirmation_required": False,
            "moveit_planning_frame": frame_config["moveit_planning_frame"],
            "moveit_end_effector_link": frame_config["moveit_end_effector_link"],
        }
        print(_json(result))
        return 0 if readiness_status == STATUS_PASS else 2
    if real_requested and (args.mock_current_tcp_pose or args.current_tcp_pose_json):
        print(
            _json(
                {
                    "final_status": STATUS_BLOCKED,
                    "blocking_reasons": ["E_MOCK_CURRENT_TCP_POSE_NOT_ALLOWED_FOR_REAL_EXECUTION"],
                    "current_tcp_pose": _current_tcp_pose_unavailable(real_required=True),
                    "trajectory_sent": False,
                    "execute_trajectory_called": False,
                    "controller_command_sent": False,
                    "real_robot_motion_executed": False,
                    **_post_motion_not_run_fields(),
                }
            )
        )
        return 2
    if real_requested and args.yes and args.cmd is None:
        print(_json({"final_status": STATUS_BLOCKED, "blocking_reasons": ["E_YES_REQUIRES_CMD"], "real_robot_motion_executed": False, **_post_motion_not_run_fields()}))
        return 2
    dry_run = not real_requested or args.dry_run
    parser_max_distance_m = (
        safety_policy["long_motion_total_limit_m"]
        if safety_policy["enable_long_step_decomposition"]
        else safety_policy["configured_max_distance_m"]
    )
    parser_hard_safety_limit_m = (
        safety_policy["long_motion_total_limit_m"]
        if safety_policy["enable_long_step_decomposition"]
        else safety_policy["hard_safety_limit_m"]
    )

    if args.cmd is not None:
        command = args.cmd
        input_mode = "cmd"
    else:
        try:
            command = input("Enter motion command: ")
            input_mode = "manual"
        except EOFError:
            metadata = _parser_metadata(
                input_mode="manual",
                original_user_text="",
                parser_mode=args.parser,
                parser_source=None,
                llm_called=False,
                model_name=args.qwen_model,
                qwen_endpoint=args.qwen_endpoint,
                llm_latency_ms=None,
                raw_llm_output=None,
                normalized_contract=None,
                parser_blocking_reasons=["E_NO_INPUT"],
            )
            print(_json({**metadata, "final_status": STATUS_BLOCKED, "blocking_reasons": ["E_NO_INPUT"], "real_robot_motion_executed": False, **_post_motion_not_run_fields()}))
            return 2

    parsed, parser_result = _parse_command_for_mode(
        command,
        parser_mode=args.parser,
        real=real_requested,
        dry_run=dry_run,
        max_distance_m=parser_max_distance_m,
        hard_safety_limit_m=parser_hard_safety_limit_m,
        default_small_step_m=float(args.default_small_step_m),
        input_mode=input_mode,
        qwen_model=args.qwen_model,
        qwen_endpoint=args.qwen_endpoint,
        qwen_timeout_s=args.qwen_timeout_s,
    )
    parser_metadata = _metadata_from_parser_result(
        parser_result,
        input_mode=input_mode,
        original_user_text=command,
        parser_mode=args.parser,
    )
    _print_parser_summary(parser_metadata)

    if parsed is None:
        post_motion = _post_motion_verification(
            real_robot_motion_executed=False,
            before_tcp_pose=None,
            target_tcp_pose=None,
            after_tcp_pose=None,
            intended_delta_m=None,
            reason="real_robot_motion_executed=false",
        )
        result = {
            **parser_metadata,
            "parsed_contract": None,
            "planner_acceptance": None,
            "acceptance_workflow": _acceptance_workflow(
                status=STATUS_BLOCKED,
                mode=acceptance_mode,
                parser_metadata=parser_metadata,
                parsed_contract=None,
                planner_acceptance=None,
                trajectory_sent=False,
                execute_trajectory_called=False,
                controller_command_sent=False,
                real_robot_motion_executed=False,
                blocking_reasons=parser_metadata["parser_blocking_reasons"],
            ),
            **_empty_motion_check(None),
            "blocking_reasons": parser_metadata["parser_blocking_reasons"],
            "goal_accepted": False,
            "execute_accepted": False,
            "trajectory_sent": False,
            "execute_trajectory_called": False,
            "controller_command_sent": False,
            "real_robot_motion_executed": False,
            **_post_motion_top_level_fields(post_motion),
            "final_status": STATUS_BLOCKED,
        }
        print("Final execution evidence:")
        print(_json(result))
        return 2

    execution_preview = parsed.execution_preview(
        input_mode=input_mode,
        parser_mode=args.parser,
        real_robot_motion_requested=real_requested,
        dry_run=dry_run,
    )
    printed_contract = parsed.task_contract(real_robot_motion_requested=real_requested, dry_run=dry_run)
    printed_contract["execution_preview"] = execution_preview
    parser_metadata["normalized_contract"] = printed_contract
    print("Execution preview:")
    print(_json(execution_preview))
    print("Normalized TETO task contract:")
    print(_json(printed_contract))
    print("Parsed TETO task contract:")
    print(_json(printed_contract))

    direction_guard = _direction_parse_guard(parser_metadata=parser_metadata, parsed=parsed)
    if direction_guard is not None:
        result = _blocked_result(
            "E_DIRECTION_PARSE_MISMATCH",
            printed_contract,
            parser_metadata,
            acceptance_mode=acceptance_mode,
        )
        result.update(direction_guard)
        result["direction_parse_guard"] = direction_guard
        print("Final execution evidence:")
        print(_json(result))
        return 2

    current_pose, current_pose_evidence, current_pose_blocker = _resolve_current_tcp_pose(args, real_requested=real_requested)
    if current_pose_blocker:
        result = _blocked_result(
            current_pose_blocker,
            printed_contract,
            parser_metadata,
            acceptance_mode=acceptance_mode,
            current_tcp_pose=current_pose_evidence,
        )
        print("Final execution evidence:")
        print(_json(result))
        return 2
    if current_pose is None:
        result = _blocked_result(
            "E_CURRENT_TCP_POSE_MISSING",
            printed_contract,
            parser_metadata,
            acceptance_mode=acceptance_mode,
            current_tcp_pose=current_pose_evidence,
        )
        print("Final execution evidence:")
        print(_json(result))
        return 2
    previous_verified_pose, previous_verified_pose_blocker = _previous_verified_tcp_pose_from_args(args)
    if previous_verified_pose_blocker:
        result = _blocked_result(
            previous_verified_pose_blocker,
            printed_contract,
            parser_metadata,
            acceptance_mode=acceptance_mode,
            current_tcp_pose=current_pose_evidence,
        )
        print("Final execution evidence:")
        print(_json(result))
        return 2

    config = _build_gateway_config(
        current_pose=current_pose,
        previous_verified_tcp_pose=previous_verified_pose,
        safety_policy=safety_policy,
        frame_config=frame_config,
        planner_risk_policy=planner_risk_policy,
        speed_scale=float(args.speed_scale),
        acc_scale=float(args.acc_scale),
        real=real_requested,
        dry_run=dry_run,
        requested_distance_m=parsed.distance_m,
    )

    motion = evaluate_cartesian_motion_gateway(
        CartesianMotionGatewayRequest(
            requested=True,
            config=config,
            command_to_task_result=parsed.gateway_task(),
            current_tcp_pose=current_pose,
        )
    )
    planner_acceptance = _planner_acceptance(
        execution_preview=execution_preview,
        motion=motion,
        execution={},
        dry_run=dry_run,
    )
    if motion.get("cartesian_motion_gateway_status") != STATUS_PASS:
        real_small_gate = (
            _real_small_motion_gate(
                parsed=parsed,
                execution_preview=execution_preview,
                planner_acceptance=planner_acceptance,
                parser_metadata=parser_metadata,
            )
            if args.real_small_motion
            else None
        )
        result = _evidence(
            status=STATUS_BLOCKED,
            motion=motion,
            execution={},
            parsed_contract=printed_contract,
            parser_metadata=parser_metadata,
            planner_acceptance=planner_acceptance,
            acceptance_mode=acceptance_mode,
            current_tcp_pose=current_pose_evidence,
            blocking_reasons=_unique(
                _string_list(motion.get("blocking_reasons"))
                + (
                    _string_list(real_small_gate.get("blocking_reasons"))
                    if isinstance(real_small_gate, dict)
                    else []
                )
            ),
            real_small_motion_gate=real_small_gate,
        )
        print("Final execution evidence:")
        print(_json(result))
        return 2

    if args.plan_only_smoke:
        plan_only_config = _build_plan_only_smoke_config(
            current_pose=current_pose,
            previous_verified_tcp_pose=previous_verified_pose,
            safety_policy=safety_policy,
            frame_config=frame_config,
            planner_risk_policy=planner_risk_policy,
            speed_scale=float(args.speed_scale),
            acc_scale=float(args.acc_scale),
            requested_distance_m=parsed.distance_m,
        )
        execution = evaluate_cartesian_motion_execution(
            CartesianMotionExecutionRequest(
                requested=True,
                config=plan_only_config,
                cartesian_motion_result=motion,
                manual_confirmation_result={"manual_confirmation_accepted": False},
                ur5_state_result={"read_only_state_contract_ready": True},
            )
        )
        planner_acceptance = _planner_acceptance(
            execution_preview=execution_preview,
            motion=motion,
            execution=execution,
            dry_run=True,
        )
        status = STATUS_BLOCKED if planner_acceptance.get("status") == STATUS_BLOCKED else STATUS_PASS
        result = _evidence(
            status=status,
            motion=motion,
            execution=execution,
            parsed_contract=printed_contract,
            parser_metadata=parser_metadata,
            planner_acceptance=planner_acceptance,
            acceptance_mode=acceptance_mode,
            current_tcp_pose=current_pose_evidence,
            blocking_reasons=planner_acceptance.get("blocking_reasons", []),
        )
        print("Final execution evidence:")
        print(_json(result))
        return 0 if status == STATUS_PASS else 2

    if dry_run:
        result = _evidence(
            status=STATUS_PASS,
            motion=motion,
            execution={},
            parsed_contract=printed_contract,
            parser_metadata=parser_metadata,
            planner_acceptance=planner_acceptance,
            acceptance_mode=acceptance_mode,
            current_tcp_pose=current_pose_evidence,
        )
        print("Final execution evidence:")
        print(_json(result))
        return 0

    real_small_gate = (
        _real_small_motion_gate(
            parsed=parsed,
            execution_preview=execution_preview,
            planner_acceptance=planner_acceptance,
            parser_metadata=parser_metadata,
        )
        if args.real_small_motion
        else None
    )
    real_small_blockers = real_small_gate.get("blocking_reasons", []) if isinstance(real_small_gate, dict) else []
    if real_small_blockers:
        result = _evidence(
            status=STATUS_BLOCKED,
            motion=motion,
            execution={},
            parsed_contract=printed_contract,
            parser_metadata=parser_metadata,
            planner_acceptance=planner_acceptance,
            acceptance_mode=acceptance_mode,
            current_tcp_pose=current_pose_evidence,
            blocking_reasons=real_small_blockers,
            real_small_motion_gate=real_small_gate,
        )
        print("Final execution evidence:")
        print(_json(result))
        return 2

    if not args.yes:
        try:
            confirmation = input(f"Execute on real UR5e? Type {CONFIRMATION_REPLY} to continue: ")
        except EOFError:
            confirmation = ""
        if confirmation != CONFIRMATION_REPLY:
            result = _evidence(
                status=STATUS_BLOCKED,
                motion=motion,
                execution={},
                parsed_contract=printed_contract,
                parser_metadata=parser_metadata,
                planner_acceptance=planner_acceptance,
                acceptance_mode=acceptance_mode,
                current_tcp_pose=current_pose_evidence,
                manual_confirmation_received=False,
                blocking_reasons=["E_CONFIRMATION_MISMATCH"],
                real_small_motion_gate=real_small_gate,
            )
            print("Final execution evidence:")
            print(_json(result))
            return 2

    prereq_blockers = _execution_prereq_blockers(timeout_s=3.0)
    if prereq_blockers:
        result = _evidence(
            status=STATUS_BLOCKED,
            motion=motion,
            execution={},
            parsed_contract=printed_contract,
            parser_metadata=parser_metadata,
            planner_acceptance=planner_acceptance,
            acceptance_mode=acceptance_mode,
            current_tcp_pose=current_pose_evidence,
            manual_confirmation_received=bool(args.yes),
            blocking_reasons=prereq_blockers,
            real_small_motion_gate=real_small_gate,
        )
        print("Final execution evidence:")
        print(_json(result))
        return 2

    execution = evaluate_cartesian_motion_execution(
        CartesianMotionExecutionRequest(
            requested=True,
            config=config,
            cartesian_motion_result=motion,
            manual_confirmation_result={"manual_confirmation_accepted": True},
            ur5_state_result={
                "read_only_state_contract_ready": True,
                "robot_state_ok": True,
                "safety_status_ok": True,
                "protective_stop": False,
                "emergency_stop": False,
                "speed_scaling": min(float(args.speed_scale), 0.10),
            },
        )
    )
    post_motion_tcp_pose = None
    if execution.get("real_robot_motion_executed") is True:
        post_motion_tcp_pose = _sample_tcp_pose_after_execution(
            timeout_s=3.0,
            settle_s=POST_EXECUTE_TCP_SETTLE_S,
            attempts=POST_EXECUTE_TCP_SAMPLE_ATTEMPTS,
            base_frame=frame_config["tcp_pose_base_frame"],
            tool_frame=frame_config["tcp_pose_tool_frame"],
        )

    status = STATUS_PASS if execution.get("real_robot_motion_executed") is True else STATUS_BLOCKED
    planner_acceptance = _planner_acceptance(
        execution_preview=execution_preview,
        motion=motion,
        execution=execution,
        dry_run=dry_run,
    )
    result = _evidence(
        status=status,
        motion=motion,
        execution=execution,
        parsed_contract=printed_contract,
        parser_metadata=parser_metadata,
        planner_acceptance=planner_acceptance,
        acceptance_mode=acceptance_mode,
        current_tcp_pose=current_pose_evidence,
        post_motion_tcp_pose=post_motion_tcp_pose,
        manual_confirmation_received=True,
        real_small_motion_gate=real_small_gate,
    )
    print("Final execution evidence:")
    print(_json(result))
    return 0 if result.get("final_status") == STATUS_PASS else 2


def _parse_command_for_mode(
    command: str,
    *,
    parser_mode: str,
    real: bool,
    dry_run: bool,
    max_distance_m: float,
    hard_safety_limit_m: float,
    default_small_step_m: float,
    input_mode: str,
    qwen_model: str | None,
    qwen_endpoint: str | None,
    qwen_timeout_s: float | None,
) -> tuple[ParsedMotionCommand | None, dict[str, Any]]:
    if parser_mode == "rule":
        return _parse_rule_result(
            command,
            max_distance_m=max_distance_m,
            hard_safety_limit_m=hard_safety_limit_m,
            default_small_step_m=default_small_step_m,
            input_mode=input_mode,
        )

    qwen_result = evaluate_qwen_motion_parser(
        QwenMotionParserRequest(
            user_text=command,
            max_distance_m=max_distance_m,
            hard_safety_limit_m=hard_safety_limit_m,
            model_name=qwen_model,
            endpoint=qwen_endpoint,
            timeout_s=qwen_timeout_s,
        )
    )
    if qwen_result.get("qwen_motion_parser_status") == STATUS_PASS:
        return _parsed_from_qwen_result(
            command,
            qwen_result,
            max_distance_m=max_distance_m,
            hard_safety_limit_m=hard_safety_limit_m,
        ), qwen_result

    if parser_mode == "auto" and dry_run and not real:
        parsed, rule_result = _parse_rule_result(
            command,
            max_distance_m=max_distance_m,
            hard_safety_limit_m=hard_safety_limit_m,
            default_small_step_m=default_small_step_m,
            input_mode=input_mode,
        )
        if parsed is not None:
            rule_result["warnings"] = _unique(_string_list(rule_result.get("warnings")) + ["qwen_failed_dry_run_rule_fallback"])
            rule_result["raw_llm_output"] = qwen_result.get("raw_llm_output")
            rule_result["qwen_blocking_reasons"] = qwen_result.get("parser_blocking_reasons", [])
            return parsed, rule_result

    return None, qwen_result


def _acceptance_mode(args: argparse.Namespace) -> str | None:
    if args.real_small_motion:
        return "real_small_motion"
    if args.plan_only_smoke:
        return "plan_only"
    if args.acceptance:
        return "dry_run" if args.dry_run or not args.real else "real_small_motion"
    return None


def _parse_rule_result(
    command: str,
    *,
    max_distance_m: float,
    hard_safety_limit_m: float,
    input_mode: str,
    default_small_step_m: float = DEFAULT_SMALL_STEP_M,
) -> tuple[ParsedMotionCommand | None, dict[str, Any]]:
    try:
        parsed = parse_motion_command(
            command,
            max_distance_m=max_distance_m,
            hard_safety_limit_m=hard_safety_limit_m,
            default_small_step_m=default_small_step_m,
        )
    except MotionParseError as exc:
        language_evidence = normalize_motion_command(
            command,
            default_small_step_m=default_small_step_m,
            parser_source="normalizer",
        )
        reasons = [str(exc)]
        return None, {
            "parser_source": "rule_based",
            "llm_called": False,
            "model_name": None,
            "qwen_endpoint": None,
            "llm_latency_ms": None,
            "raw_llm_output": None,
            "normalized_contract": None,
            **_language_metadata_fields(language_evidence),
            "parser_blocking_reasons": reasons,
            "blocking_reasons": reasons,
            "input_mode": input_mode,
        }
    return parsed, {
        "parser_source": "rule_based",
        "llm_called": False,
        "model_name": None,
        "qwen_endpoint": None,
        "llm_latency_ms": None,
        "raw_llm_output": None,
        "normalized_contract": None,
        **_language_metadata_fields(parsed.natural_language_evidence),
        "parser_blocking_reasons": [],
        "blocking_reasons": [],
        "input_mode": input_mode,
    }


def _parsed_from_qwen_result(
    command: str,
    result: dict[str, Any],
    *,
    max_distance_m: float,
    hard_safety_limit_m: float,
) -> ParsedMotionCommand:
    delta = result.get("delta_m")
    if not _vector3(delta):
        raise MotionParseError("E_INVALID_QWEN_DELTA")
    distance_m = float(result.get("distance_m"))
    language_evidence = _language_metadata_fields(result)
    if not language_evidence.get("parse_status"):
        language_evidence = _rule_language_evidence(
            command=command,
            normalized=_normalize(command),
            distance_m=distance_m,
            unit="qwen_json",
            delta=[round(float(value), 6) for value in delta],
            distance_source="explicit",
            direction_source="explicit_direction_word",
            confidence=_optional_number(result.get("confidence")) or 0.90,
            parser_source="qwen_llm",
        )
    return ParsedMotionCommand(
        command=command.strip(),
        frame="base_link",
        delta_m=[round(float(value), 6) for value in delta],
        distance_m=round(distance_m, 6),
        max_distance_m=max_distance_m,
        hard_safety_limit_m=hard_safety_limit_m,
        direction=f"{result.get('axis')}{result.get('direction')}",
        unit="qwen_json",
        parser_source="qwen_llm",
        llm_called=True,
        model_name=result.get("model_name"),
        qwen_endpoint=result.get("qwen_endpoint"),
        llm_latency_ms=result.get("llm_latency_ms"),
        raw_llm_output=result.get("raw_llm_output"),
        parser_blocking_reasons=result.get("parser_blocking_reasons", []),
        natural_language_evidence=language_evidence,
    )


def _rule_language_evidence(
    *,
    command: str,
    normalized: str,
    distance_m: float,
    unit: str,
    delta: list[float],
    distance_source: str,
    direction_source: str,
    confidence: float,
    parser_source: str = "rule_based",
) -> dict[str, Any]:
    axis, sign = _axis_direction_from_delta(delta)
    return {
        "natural_language_coverage_version": MOTION_LANGUAGE_POLICY_VERSION,
        "motion_language_policy_version": MOTION_LANGUAGE_POLICY_VERSION,
        "raw_command": command.strip(),
        "normalized_command": normalized,
        "parser_source": parser_source,
        "parser_confidence": round(float(confidence), 6),
        "motion_parse_confidence": round(float(confidence), 6),
        "parse_status": STATUS_PASS,
        "clarification_required": False,
        "clarification_reason": None,
        "unsupported_intent_reason": None,
        "distance_source": distance_source,
        "direction_source": direction_source,
        "inferred_default_distance_m": None,
        "requested_distance_m": round(float(distance_m), 6),
        "direction_axis": axis,
        "direction_sign": sign,
        "motion_frame": "base_link",
        "requires_confirmation": True,
        "safety_gate_still_required": True,
        "execution_permission_decided_by_parser": False,
        "semantic_alignment_version": MOTION_LANGUAGE_POLICY_VERSION,
        "qwen_semantic_schema_version": None,
        "qwen_intent_status": None,
        "qwen_intent_type": None,
        "qwen_direction_semantic": None,
        "qwen_distance_quality": None,
        "qwen_distance_m": None,
        "qwen_language": None,
        "qwen_confidence_intent": None,
        "qwen_confidence_direction": None,
        "qwen_confidence_distance": None,
        "qwen_confidence_overall": None,
        "qwen_semantic_parse_used": False,
        "fallback_parse_used": parser_source != "qwen_llm",
        "qwen_fallback_conflict": False,
        "qwen_fallback_conflict_reason": None,
        "canonicalization_source": "qwen_semantic" if parser_source == "qwen_llm" else "fallback_rule",
        "unit": unit,
    }


def _motion_parse_error_from_language(language_evidence: dict[str, Any]) -> str:
    if language_evidence.get("parse_status") == "UNSUPPORTED_INTENT":
        return str(language_evidence.get("unsupported_intent_reason") or "E_UNSUPPORTED_INTENT")
    return str(language_evidence.get("clarification_reason") or "E_COMMAND_NEEDS_CLARIFICATION")


def _language_metadata_fields(language_evidence: dict[str, Any] | None) -> dict[str, Any]:
    if not isinstance(language_evidence, dict):
        return {
            "natural_language_coverage_version": MOTION_LANGUAGE_POLICY_VERSION,
            "motion_language_policy_version": MOTION_LANGUAGE_POLICY_VERSION,
            "semantic_alignment_version": MOTION_LANGUAGE_POLICY_VERSION,
            "parse_status": None,
            "clarification_required": None,
            "clarification_reason": None,
            "unsupported_intent_reason": None,
            "distance_source": None,
            "direction_source": None,
            "inferred_default_distance_m": None,
            "requested_distance_m": None,
            "direction_axis": None,
            "direction_sign": None,
            "motion_frame": None,
            "motion_parse_confidence": None,
            "requires_confirmation": True,
            "safety_gate_still_required": True,
            "execution_permission_decided_by_parser": False,
            "qwen_semantic_schema_version": None,
            "qwen_intent_status": None,
            "qwen_intent_type": None,
            "qwen_direction_semantic": None,
            "qwen_distance_quality": None,
            "qwen_distance_m": None,
            "qwen_language": None,
            "qwen_confidence_intent": None,
            "qwen_confidence_direction": None,
            "qwen_confidence_distance": None,
            "qwen_confidence_overall": None,
            "qwen_semantic_parse_used": False,
            "fallback_parse_used": None,
            "qwen_fallback_conflict": False,
            "qwen_fallback_conflict_reason": None,
            "canonicalization_source": None,
            "raw_command": None,
            "normalized_command": None,
        }
    keys = [
        "natural_language_coverage_version",
        "motion_language_policy_version",
        "semantic_alignment_version",
        "raw_command",
        "normalized_command",
        "parse_status",
        "clarification_required",
        "clarification_reason",
        "unsupported_intent_reason",
        "distance_source",
        "direction_source",
        "inferred_default_distance_m",
        "requested_distance_m",
        "direction_axis",
        "direction_sign",
        "motion_frame",
        "parser_confidence",
        "motion_parse_confidence",
        "requires_confirmation",
        "safety_gate_still_required",
        "execution_permission_decided_by_parser",
        "qwen_semantic_schema_version",
        "qwen_intent_status",
        "qwen_intent_type",
        "qwen_direction_semantic",
        "qwen_distance_quality",
        "qwen_distance_m",
        "qwen_language",
        "qwen_confidence_intent",
        "qwen_confidence_direction",
        "qwen_confidence_distance",
        "qwen_confidence_overall",
        "qwen_semantic_parse_used",
        "fallback_parse_used",
        "qwen_fallback_conflict",
        "qwen_fallback_conflict_reason",
        "canonicalization_source",
    ]
    return {key: language_evidence.get(key) for key in keys}


def _metadata_from_parser_result(
    parser_result: dict[str, Any],
    *,
    input_mode: str,
    original_user_text: str,
    parser_mode: str,
) -> dict[str, Any]:
    return _parser_metadata(
        input_mode=input_mode,
        original_user_text=original_user_text,
        parser_mode=parser_mode,
        parser_source=parser_result.get("parser_source"),
        llm_called=parser_result.get("llm_called") is True,
        model_name=parser_result.get("model_name"),
        qwen_endpoint=parser_result.get("qwen_endpoint"),
        llm_latency_ms=parser_result.get("llm_latency_ms"),
        raw_llm_output=parser_result.get("raw_llm_output"),
        normalized_contract=parser_result.get("normalized_contract"),
        parser_blocking_reasons=_string_list(parser_result.get("parser_blocking_reasons") or parser_result.get("blocking_reasons")),
        language_metadata=_language_metadata_fields(parser_result),
    )


def _direction_parse_guard(
    *,
    parser_metadata: dict[str, Any],
    parsed: ParsedMotionCommand,
) -> dict[str, Any] | None:
    if parser_metadata.get("parser_source") != "qwen_llm":
        return None
    expected = _lexical_direction_expectation(str(parser_metadata.get("original_user_text") or ""))
    if expected is None:
        return None
    qwen_axis, qwen_direction = _axis_direction_from_delta(parsed.delta_m)
    if qwen_axis == expected["expected_axis"] and qwen_direction == expected["expected_direction"]:
        return None
    return {
        "expected_axis": expected["expected_axis"],
        "expected_direction": expected["expected_direction"],
        "expected_direction_word": expected["direction_word"],
        "qwen_axis": qwen_axis,
        "qwen_direction": qwen_direction,
        "direction_parse_mismatch": True,
    }


def _lexical_direction_expectation(command: str) -> dict[str, str] | None:
    normalized = _normalize(command)
    expectations = [
        ("forward", "x", "+", [r"\bforward\b", r"\bforwards\b"]),
        ("backward", "x", "-", [r"\bbackward\b", r"\bbackwards\b", r"\bback\b"]),
        ("left", "y", "+", [r"\bleft\b"]),
        ("right", "y", "-", [r"\bright\b"]),
        ("up", "z", "+", [r"\bup\b", r"\bhigher\b", r"\braise\b", r"\blift\b"]),
        ("down", "z", "-", [r"\bdown\b", r"\blower\b", r"\blowering\b", r"\bdrop\b", r"\bdescend\b"]),
    ]
    matches = [
        {"direction_word": word, "expected_axis": axis, "expected_direction": sign}
        for word, axis, sign, patterns in expectations
        if any(re.search(pattern, normalized) for pattern in patterns)
    ]
    return matches[0] if len(matches) == 1 else None


def _parser_metadata(
    *,
    input_mode: str,
    original_user_text: str,
    parser_mode: str,
    parser_source: str | None,
    llm_called: bool,
    model_name: str | None,
    qwen_endpoint: str | None,
    llm_latency_ms: float | None,
    raw_llm_output: str | None,
    normalized_contract: dict[str, Any] | None,
    parser_blocking_reasons: list[str],
    language_metadata: dict[str, Any] | None = None,
) -> dict[str, Any]:
    return {
        "input_mode": input_mode,
        "original_user_text": original_user_text,
        "parser_mode": parser_mode,
        "parser_source": parser_source,
        "llm_called": llm_called,
        "model_name": model_name,
        "qwen_endpoint": qwen_endpoint,
        "llm_latency_ms": llm_latency_ms,
        "raw_llm_output": raw_llm_output,
        "normalized_contract": normalized_contract,
        "parser_blocking_reasons": parser_blocking_reasons,
        **(language_metadata if isinstance(language_metadata, dict) else {}),
    }


def _print_parser_summary(metadata: dict[str, Any]) -> None:
    print(f"Original user text: {metadata.get('original_user_text')}")
    print(f"Parser mode: {metadata.get('parser_mode')}")
    print(f"LLM called: {str(metadata.get('llm_called')).lower()}")
    if metadata.get("raw_llm_output") is not None:
        print("Raw Qwen output:")
        print(metadata["raw_llm_output"])
    if metadata.get("normalized_contract") is not None:
        print("Normalized TETO task contract:")
        print(_json(metadata["normalized_contract"]))


def _normalize(command: str) -> str:
    lowered = command.lower().strip()
    return re.sub(r"\s+", " ", lowered.replace("+", " +").replace("-", " -"))


def _reject_forbidden(normalized: str) -> None:
    forbidden_patterns = [
        r"\bhover\b",
        r"\bmug\b",
        r"\bobject\b",
        r"\bvision\b",
        r"\bcamera\b",
        r"\burscript\b",
        r"\bscript\b",
        r"\brtde\b",
        r"\bdashboard\b",
        r"\bmovej\b",
        r"\bmovel\b",
        r"\bservoj\b",
        r"\bjoint\b",
        r"\btrajectory\b",
    ]
    if any(re.search(pattern, normalized) for pattern in forbidden_patterns):
        raise MotionParseError("E_UNSUPPORTED_OR_FORBIDDEN_COMMAND")


def _extract_distance_m(normalized: str) -> tuple[float, str]:
    match = re.search(r"\b(\d+(?:\.\d+)?)\s*(mm|millimeter|millimeters)\b", normalized)
    if not match:
        raise MotionParseError("E_DISTANCE_MM_REQUIRED")
    value = float(match.group(1))
    if not math.isfinite(value) or value <= 0.0:
        raise MotionParseError("E_INVALID_DISTANCE")
    return value / 1000.0, match.group(2)


def _extract_direction_delta(normalized: str, distance_m: float) -> tuple[str, list[float]]:
    signed_axis_map = [
        ("up", [0.0, 0.0, distance_m], [r"(?:^|\s)\+\s*z\b", r"\bz\s*\+"]),
        ("down", [0.0, 0.0, -distance_m], [r"(?:^|\s)-\s*z\b", r"\bz\s*-"]),
        ("left", [0.0, distance_m, 0.0], [r"(?:^|\s)\+\s*y\b", r"\by\s*\+"]),
        ("right", [0.0, -distance_m, 0.0], [r"(?:^|\s)-\s*y\b", r"\by\s*-"]),
        ("forward", [distance_m, 0.0, 0.0], [r"(?:^|\s)\+\s*x\b", r"\bx\s*\+"]),
        ("backward", [-distance_m, 0.0, 0.0], [r"(?:^|\s)-\s*x\b", r"\bx\s*-"]),
    ]
    signed_matches = [
        (direction, delta)
        for direction, delta, patterns in signed_axis_map
        if any(re.search(pattern, normalized) for pattern in patterns)
    ]
    if len(signed_matches) == 1:
        return signed_matches[0]
    if len(signed_matches) > 1:
        raise MotionParseError("E_DIRECTION_REQUIRED")

    direction_map = [
        ("up", [0.0, 0.0, distance_m], [r"\bup\b", r"\bhigher\b", r"\braise\b", r"\bz\b"]),
        ("down", [0.0, 0.0, -distance_m], [r"\bdown\b", r"\blower\b", r"\blowering\b"]),
        ("left", [0.0, distance_m, 0.0], [r"\bleft\b", r"\by\b"]),
        ("right", [0.0, -distance_m, 0.0], [r"\bright\b"]),
        ("forward", [distance_m, 0.0, 0.0], [r"\bforward\b", r"\bforwards\b", r"\bx\b"]),
        ("backward", [-distance_m, 0.0, 0.0], [r"\bbackward\b", r"\bbackwards\b", r"\bback\b"]),
    ]
    matches: list[tuple[str, list[float]]] = []
    for direction, delta, patterns in direction_map:
        if any(re.search(pattern, normalized) for pattern in patterns):
            matches.append((direction, delta))
    if len(matches) != 1:
        raise MotionParseError("E_DIRECTION_REQUIRED")
    return matches[0]


def _lookup_current_tcp_pose(
    *,
    timeout_s: float,
    base_frame: str = DEFAULT_TCP_POSE_BASE_FRAME,
    tool_frame: str = DEFAULT_TCP_POSE_TOOL_FRAME,
) -> dict[str, Any] | None:
    readiness = _lookup_current_tcp_pose_readiness(
        timeout_s=timeout_s,
        base_frame=base_frame,
        tool_frame=tool_frame,
    )
    if readiness.get("tcp_pose_lookup_success") is not True:
        return None
    return readiness.get("current_tcp_pose")


def _lookup_current_tcp_pose_readiness(
    *,
    timeout_s: float,
    base_frame: str,
    tool_frame: str,
) -> dict[str, Any]:
    base_frame = base_frame or DEFAULT_TCP_POSE_BASE_FRAME
    tool_frame = tool_frame or DEFAULT_TCP_POSE_TOOL_FRAME
    evidence: dict[str, Any] = {
        "tcp_pose_readiness_status": STATUS_BLOCKED,
        "tcp_pose_source": "tf2",
        "tcp_pose_base_frame": base_frame,
        "tcp_pose_tool_frame": tool_frame,
        "tcp_pose_lookup_success": False,
        "tcp_pose_lookup_error": None,
        "tcp_pose_available": False,
        "tcp_pose_stamp": None,
        "tcp_pose_age_s": None,
        "tcp_pose_stale_after_s": TCP_POSE_STALE_AFTER_S,
        "tf_available": False,
        "tf_static_available": None,
        "available_frames_sample": None,
        "robot_state_readiness_status": "UNKNOWN",
        "current_tcp_pose_blocking_reason": "E_CURRENT_TCP_POSE_MISSING",
        "current_tcp_pose": None,
    }
    try:
        import rclpy
        from rclpy.duration import Duration
        from tf2_ros import Buffer, TransformListener
    except Exception as exc:
        evidence["tcp_pose_lookup_error"] = f"E_ROS2_TF_IMPORT_FAILED:{exc}"
        evidence["robot_state_readiness_status"] = STATUS_BLOCKED
        return evidence

    initialized_here = False
    node = None
    try:
        if not rclpy.ok():
            rclpy.init(args=None)
            initialized_here = True
        node = rclpy.create_node("teto_text_to_ur5e_tcp_lookup")
        buffer = Buffer()
        listener = TransformListener(buffer, node)
        end = node.get_clock().now() + Duration(seconds=float(timeout_s))
        last_error = None
        while rclpy.ok() and node.get_clock().now() < end:
            rclpy.spin_once(node, timeout_sec=0.1)
            try:
                frames = buffer.all_frames_as_string()
                if frames:
                    evidence["available_frames_sample"] = str(frames)[:1000]
                    evidence["tf_available"] = True
            except Exception:
                pass
            try:
                tf = buffer.lookup_transform(base_frame, tool_frame, rclpy.time.Time())
                t = tf.transform.translation
                q = tf.transform.rotation
                stamp = _stamp_seconds(getattr(tf, "header", None))
                now_s = _clock_seconds(node)
                age_s = round(now_s - stamp, 6) if stamp is not None and now_s is not None else None
                pose = {
                    "frame": base_frame,
                    "position_m": [t.x, t.y, t.z],
                    "orientation_xyzw": [q.x, q.y, q.z, q.w],
                    "tcp_pose_stamp": stamp,
                    "tcp_pose_age_s": age_s,
                    "tcp_pose_base_frame": base_frame,
                    "tcp_pose_tool_frame": tool_frame,
                    "source": "tf2_lookup",
                }
                evidence.update(
                    {
                        "tcp_pose_readiness_status": STATUS_PASS,
                        "tcp_pose_lookup_success": True,
                        "tcp_pose_lookup_error": None,
                        "tcp_pose_available": True,
                        "tcp_pose_stamp": stamp,
                        "tcp_pose_age_s": age_s,
                        "tf_available": True,
                        "robot_state_readiness_status": STATUS_PASS,
                        "current_tcp_pose_blocking_reason": None,
                        "current_tcp_pose": pose,
                    }
                )
                return evidence
            except Exception as exc:
                last_error = exc
                pass
        evidence["tcp_pose_lookup_error"] = f"E_TF_LOOKUP_FAILED:{last_error}" if last_error else "E_TF_LOOKUP_TIMEOUT"
        evidence["robot_state_readiness_status"] = STATUS_BLOCKED
        return evidence
    except Exception as exc:
        evidence["tcp_pose_lookup_error"] = f"E_TCP_POSE_LOOKUP_FAILED:{exc}"
        evidence["robot_state_readiness_status"] = STATUS_BLOCKED
        return evidence
    finally:
        if node is not None:
            try:
                node.destroy_node()
            except Exception:
                pass
        if initialized_here:
            try:
                rclpy.shutdown()
            except Exception:
                pass


def _sample_tcp_pose_after_execution(
    *,
    timeout_s: float,
    settle_s: float,
    attempts: int,
    base_frame: str = DEFAULT_TCP_POSE_BASE_FRAME,
    tool_frame: str = DEFAULT_TCP_POSE_TOOL_FRAME,
) -> dict[str, Any] | None:
    settle_s = max(0.0, float(settle_s))
    attempts = max(1, int(attempts))
    if settle_s > 0.0:
        time.sleep(settle_s)
    last_pose = None
    for attempt in range(1, attempts + 1):
        if base_frame == DEFAULT_TCP_POSE_BASE_FRAME and tool_frame == DEFAULT_TCP_POSE_TOOL_FRAME:
            last_pose = _lookup_current_tcp_pose(timeout_s=timeout_s)
        else:
            last_pose = _lookup_current_tcp_pose(
                timeout_s=timeout_s,
                base_frame=base_frame,
                tool_frame=tool_frame,
            )
        if last_pose is not None:
            pose = dict(last_pose)
            pose["tcp_pose_sample_settle_s"] = round(settle_s, 6)
            pose["tcp_pose_sample_attempts"] = attempt
            return pose
    if last_pose is not None:
        pose = dict(last_pose)
        pose["tcp_pose_sample_settle_s"] = round(settle_s, 6)
        pose["tcp_pose_sample_attempts"] = attempts
        return pose
    return None


def _stamp_seconds(header: Any) -> float | None:
    stamp = getattr(header, "stamp", None)
    if stamp is None and isinstance(header, dict):
        stamp = header.get("stamp")
    if stamp is None:
        return None
    sec = _optional_number(stamp.get("sec") if isinstance(stamp, dict) else getattr(stamp, "sec", None))
    nanosec = _optional_number(stamp.get("nanosec") if isinstance(stamp, dict) else getattr(stamp, "nanosec", None))
    if sec is None:
        return None
    return round(sec + (nanosec or 0.0) / 1_000_000_000.0, 6)


def _clock_seconds(node: Any) -> float | None:
    try:
        now = node.get_clock().now()
    except Exception:
        return None
    nanoseconds = getattr(now, "nanoseconds", None)
    if isinstance(nanoseconds, int):
        return nanoseconds / 1_000_000_000.0
    try:
        msg = now.to_msg()
    except Exception:
        return None
    sec = _optional_number(getattr(msg, "sec", None))
    nanosec = _optional_number(getattr(msg, "nanosec", None))
    if sec is None:
        return None
    return sec + (nanosec or 0.0) / 1_000_000_000.0


def _resolve_current_tcp_pose(args: argparse.Namespace, *, real_requested: bool) -> tuple[dict[str, Any] | None, dict[str, Any], str | None]:
    frame_config = _frame_config_from_args(args)
    base_frame = frame_config["tcp_pose_base_frame"]
    tool_frame = frame_config["tcp_pose_tool_frame"]
    if args.mock_current_tcp_pose:
        pose = dict(MOCK_CURRENT_TCP_POSE_FOR_DRY_RUN_ONLY)
        pose["frame"] = base_frame
        pose["tcp_pose_base_frame"] = base_frame
        pose["tcp_pose_tool_frame"] = tool_frame
        pose["tcp_pose_readiness_status"] = STATUS_PASS
        pose["tcp_pose_lookup_success"] = True
        pose["tcp_pose_available"] = True
        pose["tcp_pose_source"] = pose.get("source")
        return _pose_for_gateway(pose), pose, None

    if args.current_tcp_pose_json:
        try:
            raw_pose = json.loads(args.current_tcp_pose_json)
        except json.JSONDecodeError:
            return None, _current_tcp_pose_unavailable(real_required=real_requested, base_frame=base_frame, tool_frame=tool_frame, lookup_error="E_INVALID_CURRENT_TCP_POSE_JSON"), "E_INVALID_CURRENT_TCP_POSE_JSON"
        pose = _current_tcp_pose_evidence(
            raw_pose,
            source="provided_current_tcp_pose_for_dry_run_only",
            allowed_for_real_execution=False,
            base_frame=base_frame,
            tool_frame=tool_frame,
        )
        if pose is None:
            return None, _current_tcp_pose_unavailable(real_required=real_requested, base_frame=base_frame, tool_frame=tool_frame, lookup_error="E_INVALID_CURRENT_TCP_POSE"), "E_INVALID_CURRENT_TCP_POSE"
        return _pose_for_gateway(pose), pose, None

    if base_frame == DEFAULT_TCP_POSE_BASE_FRAME and tool_frame == DEFAULT_TCP_POSE_TOOL_FRAME:
        pose = _lookup_current_tcp_pose(timeout_s=3.0)
    else:
        pose = _lookup_current_tcp_pose(
            timeout_s=3.0,
            base_frame=base_frame,
            tool_frame=tool_frame,
        )
    if pose is None:
        return None, _current_tcp_pose_unavailable(
            real_required=real_requested,
            base_frame=base_frame,
            tool_frame=tool_frame,
            lookup_error="E_TF_LOOKUP_UNAVAILABLE",
        ), None
    evidence = _current_tcp_pose_evidence(
        pose,
        source=pose.get("source") or "real_robot_state",
        allowed_for_real_execution=True,
        base_frame=base_frame,
        tool_frame=tool_frame,
    )
    if evidence is None:
        return None, _current_tcp_pose_unavailable(real_required=real_requested, base_frame=base_frame, tool_frame=tool_frame, lookup_error="E_INVALID_CURRENT_TCP_POSE"), "E_INVALID_CURRENT_TCP_POSE"
    return _pose_for_gateway(evidence), evidence, None


def _pose_for_gateway(pose: dict[str, Any]) -> dict[str, Any]:
    return {
        "frame": pose.get("frame") or pose.get("tcp_pose_base_frame") or DEFAULT_TCP_POSE_BASE_FRAME,
        "position_m": list(pose.get("position_m") or []),
        "orientation_xyzw": list(pose.get("orientation_xyzw") or []),
    }


def _current_tcp_pose_evidence(
    pose: dict[str, Any],
    *,
    source: str,
    allowed_for_real_execution: bool,
    base_frame: str = DEFAULT_TCP_POSE_BASE_FRAME,
    tool_frame: str = DEFAULT_TCP_POSE_TOOL_FRAME,
    readiness: dict[str, Any] | None = None,
) -> dict[str, Any] | None:
    if not isinstance(pose, dict):
        return None
    base_frame = str(pose.get("tcp_pose_base_frame") or base_frame or DEFAULT_TCP_POSE_BASE_FRAME)
    tool_frame = str(pose.get("tcp_pose_tool_frame") or tool_frame or DEFAULT_TCP_POSE_TOOL_FRAME)
    position = pose.get("position_m")
    orientation = pose.get("orientation_xyzw")
    if not _vector3(position) or not _quaternion(orientation):
        return None
    readiness = readiness if isinstance(readiness, dict) else {}
    stamp = _optional_number(pose.get("tcp_pose_stamp"))
    age_s = _optional_number(pose.get("tcp_pose_age_s"))
    return {
        "available": True,
        "frame": pose.get("frame") or base_frame,
        "position_m": [round(float(value), 12) for value in position],
        "orientation_xyzw": [round(float(value), 15) for value in orientation],
        "source": source,
        "tcp_pose_readiness_status": STATUS_PASS,
        "tcp_pose_source": source,
        "tcp_pose_base_frame": base_frame,
        "tcp_pose_tool_frame": tool_frame,
        "tcp_pose_lookup_success": True,
        "tcp_pose_lookup_error": None,
        "tcp_pose_available": True,
        "tcp_pose_stamp": stamp,
        "tcp_pose_age_s": age_s,
        "tcp_pose_stale_after_s": TCP_POSE_STALE_AFTER_S,
        "tf_available": readiness.get("tf_available"),
        "tf_static_available": readiness.get("tf_static_available"),
        "available_frames_sample": readiness.get("available_frames_sample"),
        "robot_state_readiness_status": readiness.get("robot_state_readiness_status", STATUS_PASS),
        "current_tcp_pose_blocking_reason": None,
        "allowed_for_real_execution": allowed_for_real_execution,
        "tcp_pose_sample_settle_s": _optional_number(pose.get("tcp_pose_sample_settle_s")),
        "tcp_pose_sample_attempts": int(pose["tcp_pose_sample_attempts"])
        if isinstance(pose.get("tcp_pose_sample_attempts"), int)
        and not isinstance(pose.get("tcp_pose_sample_attempts"), bool)
        else None,
    }


def _current_tcp_pose_unavailable(
    *,
    real_required: bool,
    base_frame: str = DEFAULT_TCP_POSE_BASE_FRAME,
    tool_frame: str = DEFAULT_TCP_POSE_TOOL_FRAME,
    lookup_error: Any = None,
    readiness: dict[str, Any] | None = None,
) -> dict[str, Any]:
    readiness = readiness if isinstance(readiness, dict) else {}
    return {
        "available": False,
        "frame": None,
        "position_m": None,
        "orientation_xyzw": None,
        "source": "real_robot_state_required" if real_required else "current_tcp_pose_not_provided_or_available",
        "tcp_pose_readiness_status": STATUS_BLOCKED,
        "tcp_pose_source": readiness.get("tcp_pose_source", "tf2"),
        "tcp_pose_base_frame": base_frame,
        "tcp_pose_tool_frame": tool_frame,
        "tcp_pose_lookup_success": False,
        "tcp_pose_lookup_error": lookup_error,
        "tcp_pose_available": False,
        "tcp_pose_stamp": None,
        "tcp_pose_age_s": None,
        "tcp_pose_stale_after_s": TCP_POSE_STALE_AFTER_S,
        "tf_available": readiness.get("tf_available", False),
        "tf_static_available": readiness.get("tf_static_available"),
        "available_frames_sample": readiness.get("available_frames_sample"),
        "robot_state_readiness_status": readiness.get("robot_state_readiness_status", STATUS_BLOCKED),
        "current_tcp_pose_blocking_reason": "E_CURRENT_TCP_POSE_MISSING",
        "allowed_for_real_execution": True if real_required else False,
    }


def _safety_policy_from_args(args: argparse.Namespace) -> dict[str, Any]:
    configured_max = _optional_number(getattr(args, "max_distance_m", None))
    max_axis_step = _optional_number(getattr(args, "max_axis_step_m", None))
    hard_limit = _optional_number(getattr(args, "hard_safety_limit_m", None))
    session_radius_limit = _optional_number(getattr(args, "session_radius_limit_m", None))
    configured_position_tolerance = _optional_number(getattr(args, "position_tolerance_m", None))
    orientation_tolerance = _optional_number(getattr(args, "orientation_tolerance_rad", None))
    max_step_distance = configured_max if configured_max is not None else DEFAULT_REAL_MAX_DISTANCE_M
    hard_safety_limit = hard_limit if hard_limit is not None else DEFAULT_REAL_HARD_SAFETY_LIMIT_M
    configured_one_shot = _optional_number(getattr(args, "max_one_shot_distance_m", None))
    max_one_shot_distance = min(
        configured_one_shot if configured_one_shot is not None else max_step_distance,
        max_step_distance,
        hard_safety_limit,
    )
    max_decomposed_substep = _optional_number(getattr(args, "max_substep_distance_m", None)) or 0.02
    max_decomposed_total = _optional_number(getattr(args, "long_motion_total_limit_m", None)) or 0.50
    return {
        "safety_policy_name": str(getattr(args, "safety_policy_name", None) or DEFAULT_SAFETY_POLICY_NAME),
        "safety_policy_source": DEFAULT_SAFETY_POLICY_SOURCE,
        "configured_max_distance_m": max_step_distance,
        "max_step_distance_m": max_step_distance,
        "max_axis_step_m": max_axis_step if max_axis_step is not None else DEFAULT_MAX_AXIS_STEP_M,
        "hard_safety_limit_m": hard_safety_limit,
        "session_radius_limit_m": session_radius_limit,
        "configured_position_tolerance_m": (
            configured_position_tolerance
            if configured_position_tolerance is not None
            else DEFAULT_REAL_POSITION_TOLERANCE_M
        ),
        "configured_orientation_tolerance_rad": (
            orientation_tolerance
            if orientation_tolerance is not None
            else DEFAULT_REAL_ORIENTATION_TOLERANCE_RAD
        ),
        "enable_long_step_decomposition": bool(getattr(args, "enable_long_step_decomposition", False)),
        "motion_permission_envelope_version": str(
            getattr(args, "motion_permission_envelope_version", None)
            or DEFAULT_MOTION_PERMISSION_ENVELOPE_VERSION
        ),
        "long_step_policy_name": "lab_long_step_decomposition_v1",
        "max_one_shot_distance_m": max_one_shot_distance,
        "long_step_threshold_m": _optional_number(getattr(args, "long_step_threshold_m", None)) or 0.05,
        "max_substep_distance_m": max_decomposed_substep,
        "max_decomposed_substep_distance_m": max_decomposed_substep,
        "min_final_substep_distance_m": _optional_number(getattr(args, "min_final_substep_distance_m", None)) or 0.001,
        "long_motion_total_limit_m": max_decomposed_total,
        "max_decomposed_total_distance_m": max_decomposed_total,
        "hard_single_step_safety_limit_m": hard_safety_limit,
        "substep_execution_mode": str(getattr(args, "substep_execution_mode", None) or "contract_only"),
    }


def _frame_config_from_args(args: argparse.Namespace) -> dict[str, str]:
    return {
        "tcp_pose_base_frame": str(getattr(args, "tcp_pose_base_frame", None) or DEFAULT_TCP_POSE_BASE_FRAME),
        "tcp_pose_tool_frame": str(getattr(args, "tcp_pose_tool_frame", None) or DEFAULT_TCP_POSE_TOOL_FRAME),
        "moveit_planning_frame": str(getattr(args, "moveit_planning_frame", None) or DEFAULT_MOVEIT_PLANNING_FRAME),
        "moveit_end_effector_link": str(getattr(args, "moveit_end_effector_link", None) or DEFAULT_MOVEIT_END_EFFECTOR_LINK),
    }


def _planner_risk_policy_from_args(args: argparse.Namespace) -> dict[str, Any]:
    return {
        "planner_risk_policy_name": str(getattr(args, "planner_risk_policy", None) or DEFAULT_PLANNER_RISK_POLICY_NAME),
        "planner_risk_policy_mode": str(getattr(args, "planner_risk_mode", None) or DEFAULT_PLANNER_RISK_POLICY_MODE),
        "planner_risk_blocking_enabled": bool(getattr(args, "enable_planner_risk_blocking", False)),
        "warn_max_joint_delta_rad": (
            _optional_number(getattr(args, "warn_max_joint_delta_rad", None))
            or DEFAULT_WARN_MAX_JOINT_DELTA_RAD
        ),
        "warn_max_wrist_joint_delta_rad": (
            _optional_number(getattr(args, "warn_max_wrist_joint_delta_rad", None))
            or DEFAULT_WARN_MAX_WRIST_JOINT_DELTA_RAD
        ),
        "warn_path_length_ratio": (
            _optional_number(getattr(args, "warn_path_length_ratio", None))
            or DEFAULT_WARN_PATH_LENGTH_RATIO
        ),
        "warn_joint_wrap_suspected": True,
    }


def _previous_verified_tcp_pose_from_args(args: argparse.Namespace) -> tuple[dict[str, Any] | None, str | None]:
    raw = getattr(args, "previous_verified_tcp_pose_json", None)
    if not raw:
        return None, None
    try:
        pose = json.loads(raw)
    except json.JSONDecodeError:
        return None, "E_INVALID_PREVIOUS_VERIFIED_TCP_POSE_JSON"
    evidence = _current_tcp_pose_evidence(
        pose if isinstance(pose, dict) else {},
        source="previous_verified_tcp_pose_json",
        allowed_for_real_execution=True,
    )
    if evidence.get("available") is not True:
        return None, "E_INVALID_PREVIOUS_VERIFIED_TCP_POSE"
    return evidence, None


def _execution_prereq_blockers(*, timeout_s: float) -> list[str]:
    try:
        import rclpy
        from controller_manager_msgs.srv import ListControllers
        from moveit_msgs.action import ExecuteTrajectory
        from rclpy.action import ActionClient
        from rclpy.duration import Duration
    except Exception:
        return ["E_ROS2_EXECUTION_IMPORT_FAILED"]

    blockers: list[str] = []
    initialized_here = False
    try:
        if not rclpy.ok():
            rclpy.init(args=None)
            initialized_here = True
        node = rclpy.create_node("teto_text_to_ur5e_execution_prereq")
        execute_client = ActionClient(node, ExecuteTrajectory, "/execute_trajectory")
        if not execute_client.wait_for_server(timeout_sec=float(timeout_s)):
            blockers.append("E_EXECUTE_TRAJECTORY_ACTION_UNAVAILABLE")

        controller_client = node.create_client(ListControllers, "/controller_manager/list_controllers")
        if not controller_client.wait_for_service(timeout_sec=float(timeout_s)):
            blockers.append("E_CONTROLLER_MANAGER_UNAVAILABLE")
        else:
            future = controller_client.call_async(ListControllers.Request())
            deadline = node.get_clock().now() + Duration(seconds=float(timeout_s))
            while rclpy.ok() and node.get_clock().now() < deadline and not future.done():
                rclpy.spin_once(node, timeout_sec=0.1)
            if not future.done():
                blockers.append("E_CONTROLLER_MANAGER_TIMEOUT")
            else:
                controllers = {item.name: item.state for item in future.result().controller}
                if controllers.get("scaled_joint_trajectory_controller") != "active":
                    blockers.append("E_SCALED_JOINT_TRAJECTORY_CONTROLLER_INACTIVE")
        return blockers
    except Exception as exc:
        return [f"E_EXECUTION_PREREQ_CHECK_FAILED:{exc}"]
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        if initialized_here:
            try:
                rclpy.shutdown()
            except Exception:
                pass


def _build_gateway_config(
    *,
    current_pose: dict[str, Any],
    previous_verified_tcp_pose: dict[str, Any] | None,
    safety_policy: dict[str, Any],
    frame_config: dict[str, str],
    planner_risk_policy: dict[str, Any],
    speed_scale: float,
    acc_scale: float,
    real: bool,
    dry_run: bool,
    requested_distance_m: float | None = None,
) -> dict[str, Any]:
    tolerance = _motion_tolerance_config(
        requested_distance_m=requested_distance_m,
        safety_policy=safety_policy,
    )
    return {
        "moveit_execution_mode": "real" if real and not dry_run else "shadow",
        "enable_ros2_runtime": bool(real and not dry_run),
        "enable_moveit_plan": bool(real and not dry_run),
        "enable_moveit_execute": bool(real and not dry_run),
        "enable_real_robot_motion": bool(real and not dry_run),
        "manual_confirmation_required": True,
        "planning_group": "ur_manipulator",
        "planning_frame": frame_config["moveit_planning_frame"],
        "end_effector_frame": frame_config["moveit_end_effector_link"],
        "end_effector_link": frame_config["moveit_end_effector_link"],
        "move_group_action_name": "/move_action",
        "execute_trajectory_action_name": "/execute_trajectory",
        "pipeline_id": "move_group",
        "planner_id": "ur_manipulator[RRTConnectkConfigDefault]",
        "allowed_frames": [frame_config["tcp_pose_base_frame"], frame_config["moveit_planning_frame"]],
        "max_translation_m": safety_policy["configured_max_distance_m"],
        "max_step_distance_m": safety_policy["max_step_distance_m"],
        "max_axis_step_m": safety_policy["max_axis_step_m"],
        "hard_safety_limit_m": safety_policy["hard_safety_limit_m"],
        "session_radius_limit_m": safety_policy["session_radius_limit_m"],
        "configured_max_distance_m": safety_policy["configured_max_distance_m"],
        "safety_policy_name": safety_policy["safety_policy_name"],
        "safety_policy_source": safety_policy["safety_policy_source"],
        "workspace_bounds": {"x": [-1.0, 1.0], "y": [-1.0, 1.0], "z": [0.0, 2.0]},
        "requested_distance_m": tolerance["requested_distance_m"],
        "position_tolerance_m": tolerance["position_tolerance_m"],
        "orientation_tolerance_rad": tolerance["orientation_tolerance_rad"],
        "tolerance_to_requested_distance_ratio": tolerance["tolerance_to_requested_distance_ratio"],
        "small_motion_tolerance_policy": tolerance["small_motion_tolerance_policy"],
        **planner_risk_policy,
        "enable_long_step_decomposition": safety_policy["enable_long_step_decomposition"],
        "motion_permission_envelope_version": safety_policy["motion_permission_envelope_version"],
        "long_step_policy_name": safety_policy["long_step_policy_name"],
        "max_one_shot_distance_m": safety_policy["max_one_shot_distance_m"],
        "long_step_threshold_m": safety_policy["long_step_threshold_m"],
        "max_substep_distance_m": safety_policy["max_substep_distance_m"],
        "max_decomposed_substep_distance_m": safety_policy["max_decomposed_substep_distance_m"],
        "min_final_substep_distance_m": safety_policy["min_final_substep_distance_m"],
        "long_motion_total_limit_m": safety_policy["long_motion_total_limit_m"],
        "max_decomposed_total_distance_m": safety_policy["max_decomposed_total_distance_m"],
        "hard_single_step_safety_limit_m": safety_policy["hard_single_step_safety_limit_m"],
        "substep_execution_mode": safety_policy["substep_execution_mode"],
        "max_speed_scale": min(float(speed_scale), 0.10),
        "max_acc_scale": min(float(acc_scale), 0.10),
        "current_tcp_pose": current_pose,
        "previous_verified_tcp_pose": previous_verified_tcp_pose,
        "tcp_pose_base_frame": frame_config["tcp_pose_base_frame"],
        "tcp_pose_tool_frame": frame_config["tcp_pose_tool_frame"],
        "base_link_direction_mapping": BASE_LINK_DIRECTION_MAPPING,
        "robot_state_ok": True,
        "safety_status_ok": True,
        "protective_stop": False,
        "emergency_stop": False,
        "speed_scaling": min(float(speed_scale), 0.10),
    }


def _build_plan_only_smoke_config(
    *,
    current_pose: dict[str, Any],
    previous_verified_tcp_pose: dict[str, Any] | None,
    safety_policy: dict[str, Any],
    frame_config: dict[str, str],
    planner_risk_policy: dict[str, Any],
    speed_scale: float,
    acc_scale: float,
    requested_distance_m: float | None = None,
) -> dict[str, Any]:
    return {
        **_build_gateway_config(
            current_pose=current_pose,
            previous_verified_tcp_pose=previous_verified_tcp_pose,
            safety_policy=safety_policy,
            frame_config=frame_config,
            planner_risk_policy=planner_risk_policy,
            speed_scale=speed_scale,
            acc_scale=acc_scale,
            real=True,
            dry_run=False,
            requested_distance_m=requested_distance_m,
        ),
        "moveit_execution_mode": "real",
        "enable_ros2_runtime": True,
        "enable_moveit_plan": True,
        "enable_moveit_execute": False,
        "enable_real_robot_motion": False,
        "moveit_execute_allowed": False,
        "trajectory_send_allowed": False,
        "execution_allowed": False,
        "manual_confirmation_required": True,
    }


def _motion_tolerance_config(*, requested_distance_m: float | None, safety_policy: dict[str, Any]) -> dict[str, Any]:
    requested = _optional_number(requested_distance_m)
    configured_position_tolerance = (
        _optional_number(safety_policy.get("configured_position_tolerance_m"))
        or DEFAULT_REAL_POSITION_TOLERANCE_M
    )
    orientation_tolerance = (
        _optional_number(safety_policy.get("configured_orientation_tolerance_rad"))
        or DEFAULT_REAL_ORIENTATION_TOLERANCE_RAD
    )
    position_tolerance = configured_position_tolerance
    if requested is not None and requested > 0.0:
        position_tolerance = min(
            configured_position_tolerance,
            requested * REAL_MOTION_TOLERANCE_DISTANCE_RATIO,
        )
    policy = (
        "real_motion_safety_policy_v1:"
        "position_tolerance_m=min(configured_position_tolerance_m,"
        "requested_distance_m*0.10);orientation_tolerance_rad=configured"
    )
    ratio = None
    if requested is not None and requested > 0.0:
        ratio = round(position_tolerance / requested, 6)
    return {
        "requested_distance_m": round(requested, 6) if requested is not None else None,
        "position_tolerance_m": round(position_tolerance, 6),
        "orientation_tolerance_rad": round(orientation_tolerance, 6),
        "tolerance_to_requested_distance_ratio": ratio,
        "small_motion_tolerance_policy": policy,
    }


def _planner_acceptance(
    *,
    execution_preview: dict[str, Any],
    motion: dict[str, Any],
    execution: dict[str, Any],
    dry_run: bool,
) -> dict[str, Any]:
    requested_delta = execution_preview.get("delta_m")
    requested_distance = _optional_number(execution_preview.get("distance_m"))
    gateway_distance = _optional_number(motion.get("translation_distance_m"))
    moveit_result = _moveit_result_from(motion, execution)
    moveit_plan_request = motion.get("moveit_plan_request") if isinstance(motion.get("moveit_plan_request"), dict) else {}
    hard_safety_limit = (
        _optional_number(execution_preview.get("hard_safety_limit_m"))
        or _optional_number(motion.get("hard_safety_limit_m"))
        or _optional_number(moveit_result.get("hard_safety_limit_m"))
        or DEFAULT_REAL_HARD_SAFETY_LIMIT_M
    )
    trajectory_metrics = _trajectory_metrics(moveit_result)
    tolerance_evidence = _moveit_tolerance_evidence(
        requested_distance=requested_distance,
        moveit_result=moveit_result,
        moveit_plan_request=moveit_plan_request,
    )

    trajectory_sent = (
        execution.get("trajectory_sent") is True
        or moveit_result.get("trajectory_sent") is True
        or motion.get("trajectory_sent") is True
    )
    controller_command_sent = (
        execution.get("controller_command_sent") is True
        or moveit_result.get("controller_command_sent") is True
        or motion.get("controller_command_sent") is True
    )
    real_robot_motion_executed = (
        execution.get("real_robot_motion_executed") is True
        or moveit_result.get("real_robot_motion_executed") is True
        or motion.get("real_robot_motion_executed") is True
    )
    execute_called = (
        execution.get("moveit_execute_called") is True
        or moveit_result.get("moveit_execute_called") is True
        or motion.get("moveit_execute_called") is True
    )
    execution_allowed = (
        not dry_run
        and execution.get("trajectory_send_allowed") is True
        and execution.get("real_robot_motion_executed") is True
    )
    distance_matches = (
        requested_distance is not None
        and gateway_distance is not None
        and abs(requested_distance - gateway_distance) <= EPS
    )
    within_safety_limit = (
        requested_distance is not None
        and requested_distance <= hard_safety_limit + EPS
    )

    blocking_reasons: list[str] = []
    warnings: list[str] = []
    if motion.get("cartesian_motion_gateway_status") != STATUS_PASS:
        blocking_reasons.extend(_string_list(motion.get("blocking_reasons")))
    if not distance_matches:
        blocking_reasons.append("E_PLANNER_REQUEST_DISTANCE_MISMATCH")
    if not within_safety_limit:
        blocking_reasons.append("E_PLANNER_REQUEST_EXCEEDS_HARD_LIMIT")
    if not dry_run and execution_allowed:
        blocking_reasons.append("E_PLANNER_ACCEPTANCE_NOT_PLAN_ONLY")
    if trajectory_sent:
        blocking_reasons.append("E_PLANNER_ACCEPTANCE_TRAJECTORY_SENT")
    if controller_command_sent:
        blocking_reasons.append("E_PLANNER_ACCEPTANCE_CONTROLLER_COMMAND_SENT")
    if execute_called:
        blocking_reasons.append("E_PLANNER_ACCEPTANCE_EXECUTE_CALLED")
    if real_robot_motion_executed:
        blocking_reasons.append("E_PLANNER_ACCEPTANCE_REAL_ROBOT_MOTION_EXECUTED")
    if execution.get("cartesian_motion_execution_status") == STATUS_BLOCKED:
        blocking_reasons.extend(_string_list(execution.get("blocking_reasons")))

    max_joint_delta = _first_not_none(
        _optional_number(moveit_result.get("max_joint_delta_rad")),
        trajectory_metrics["max_joint_delta_rad"],
    )
    if (
        max_joint_delta is not None
        and requested_distance is not None
        and requested_distance <= hard_safety_limit + EPS
        and max_joint_delta > SUSPICIOUS_TINY_MOTION_JOINT_DELTA_RAD
    ):
        warnings.append("W_SUSPICIOUS_JOINT_DELTA_FOR_TINY_CARTESIAN_MOTION")
    orientation_change = trajectory_metrics["orientation_change_rad"]
    if (
        orientation_change is not None
        and requested_distance is not None
        and requested_distance <= hard_safety_limit + EPS
        and orientation_change > SUSPICIOUS_TINY_MOTION_ORIENTATION_CHANGE_RAD
    ):
        warnings.append("W_SUSPICIOUS_ORIENTATION_CHANGE_FOR_TINY_CARTESIAN_MOTION")

    blocking_reasons = _unique(blocking_reasons)
    warnings = _unique(warnings)
    reasonableness_check = STATUS_BLOCKED if blocking_reasons else STATUS_WARNING if warnings else STATUS_PASS
    status = STATUS_BLOCKED if blocking_reasons else STATUS_WARNING if warnings else STATUS_PASS
    result = {
        "status": status,
        "planner_acceptance_context": "plan_only_safety_audit",
        "planner_acceptance_blocks_real_execution": False,
        "real_execution_audit_status": _real_execution_audit_status(real_robot_motion_executed, execution),
        "plan_only": dry_run and not execution_allowed,
        "execution_allowed": execution_allowed,
        "trajectory_sent": trajectory_sent,
        "controller_command_sent": controller_command_sent,
        "execute_trajectory_called": execute_called,
        "real_robot_motion_executed": real_robot_motion_executed,
        "manual_confirmation_required": True,
        "manual_confirmation_status": (
            "ACCEPTED"
            if execution.get("manual_confirmation_accepted") is True
            else "NOT_RECEIVED"
        ),
        "requested_delta_m": requested_delta if _vector3(requested_delta) else None,
        "requested_distance_m": requested_distance,
        "configured_max_distance_m": _first_not_none(
            _optional_number(execution_preview.get("max_distance_m")),
            _optional_number(motion.get("configured_max_distance_m")),
            _optional_number(moveit_result.get("configured_max_distance_m")),
        ),
        "motion_frame": motion.get("motion_frame") or execution_preview.get("frame"),
        "direction_axis": motion.get("direction_axis") or execution_preview.get("axis"),
        "direction_sign": motion.get("direction_sign") or execution_preview.get("direction"),
        "first_move_bootstrap_used": motion.get("first_move_bootstrap_used"),
        "previous_verified_tcp_pose": motion.get("previous_verified_tcp_pose"),
        "current_measured_tcp_pose": motion.get("current_measured_tcp_pose"),
        "requested_target_tcp_pose": motion.get("requested_target_tcp_pose"),
        "delta_from_current_tcp_m": motion.get("delta_from_current_tcp_m"),
        "delta_from_previous_verified_tcp_m": motion.get("delta_from_previous_verified_tcp_m"),
        "max_step_distance_m": motion.get("max_step_distance_m"),
        "max_axis_step_m": motion.get("max_axis_step_m"),
        "hard_safety_limit_m": hard_safety_limit,
        "session_radius_limit_m": motion.get("session_radius_limit_m"),
        "motion_distance_regime": motion.get("motion_distance_regime"),
        "motion_permission_envelope_version": motion.get("motion_permission_envelope_version"),
        "long_step_decomposition_enabled": motion.get("long_step_decomposition_enabled"),
        "decomposition_enabled": motion.get("decomposition_enabled"),
        "long_step_policy_name": motion.get("long_step_policy_name"),
        "requested_total_distance_m": motion.get("requested_total_distance_m"),
        "intent_name": motion.get("intent_name"),
        "motion_contract_type": motion.get("motion_contract_type"),
        "requested_vector_m": motion.get("requested_vector_m"),
        "normalized_direction_vector": motion.get("normalized_direction_vector"),
        "shared_max_total_distance_m": motion.get("shared_max_total_distance_m"),
        "distance_within_shared_envelope": motion.get("distance_within_shared_envelope"),
        "decomposition_required": motion.get("decomposition_required"),
        "execution_backend": motion.get("execution_backend"),
        "backend_policy_id": motion.get("backend_policy_id"),
        "one_shot_execution_allowed": motion.get("one_shot_execution_allowed"),
        "planned_subgoals": motion.get("planned_subgoals"),
        "one_shot_distance_limit_m": motion.get("one_shot_distance_limit_m"),
        "max_one_shot_distance_m": motion.get("max_one_shot_distance_m"),
        "hard_single_step_safety_limit_m": motion.get("hard_single_step_safety_limit_m"),
        "long_motion_total_limit_m": motion.get("long_motion_total_limit_m"),
        "max_decomposed_total_distance_m": motion.get("max_decomposed_total_distance_m"),
        "max_substep_distance_m": motion.get("max_substep_distance_m"),
        "max_decomposed_substep_distance_m": motion.get("max_decomposed_substep_distance_m"),
        "min_final_substep_distance_m": motion.get("min_final_substep_distance_m"),
        "planned_execution_style": motion.get("planned_execution_style"),
        "substep_execution_mode": motion.get("substep_execution_mode"),
        "real_substep_execution_enabled": motion.get("real_substep_execution_enabled"),
        "planned_substep_count": motion.get("planned_substep_count"),
        "planned_substep_distances_m": motion.get("planned_substep_distances_m"),
        "planned_substep_vectors_m": motion.get("planned_substep_vectors_m"),
        "substep_count": motion.get("substep_count"),
        "decomposed_substeps_m": motion.get("decomposed_substeps_m"),
        "decomposed_total_distance_m": motion.get("decomposed_total_distance_m"),
        "decomposition_remainder_m": motion.get("decomposition_remainder_m"),
        "decomposition_status": motion.get("decomposition_status"),
        "decomposition_blocking_reason": motion.get("decomposition_blocking_reason"),
        "decomposition_does_not_bypass_safety_limits": motion.get("decomposition_does_not_bypass_safety_limits"),
        "safety_gate_scope": motion.get("safety_gate_scope"),
        "one_shot_distance_check_status": motion.get("one_shot_distance_check_status"),
        "substep_distance_check_status": motion.get("substep_distance_check_status"),
        "total_long_motion_check_status": motion.get("total_long_motion_check_status"),
        "workspace_envelope_check_status": motion.get("workspace_envelope_check_status"),
        "decomposed_motion_allowed": motion.get("decomposed_motion_allowed"),
        "decomposed_motion_blocking_reason": motion.get("decomposed_motion_blocking_reason"),
        "autoregressive_update_required": motion.get("autoregressive_update_required"),
        "substep_feedback_required": motion.get("substep_feedback_required"),
        "substep_reobserve_allowed": motion.get("substep_reobserve_allowed"),
        "step_delta_within_limit": motion.get("step_delta_within_limit"),
        "axis_delta_within_limit": motion.get("axis_delta_within_limit"),
        "workspace_envelope_within_limit": motion.get("workspace_envelope_within_limit"),
        "base_link_direction_mapping": motion.get("base_link_direction_mapping"),
        "requested_distance_within_configured_limit": _first_not_none(
            motion.get("requested_distance_within_configured_limit"),
            moveit_result.get("requested_distance_within_configured_limit"),
            execution_preview.get("within_safety_limit"),
        ),
        "safety_policy_source": _first_not_none(
            motion.get("safety_policy_source"),
            moveit_result.get("safety_policy_source"),
        ),
        "safety_policy_name": _first_not_none(
            motion.get("safety_policy_name"),
            moveit_result.get("safety_policy_name"),
        ),
        "planner_mode": _first_not_none(moveit_result.get("planner_mode"), moveit_plan_request.get("planner_mode")),
        "planning_pipeline_id": _first_not_none(moveit_result.get("planning_pipeline_id"), moveit_plan_request.get("planning_pipeline_id")),
        "planner_id": _first_not_none(moveit_result.get("planner_id"), moveit_plan_request.get("planner_id")),
        "moveit_goal_type": _first_not_none(moveit_result.get("moveit_goal_type"), moveit_plan_request.get("moveit_goal_type")),
        "joint_space_pose_goal_used": _first_not_none(moveit_result.get("joint_space_pose_goal_used"), moveit_plan_request.get("joint_space_pose_goal_used")),
        "cartesian_path_used": _first_not_none(moveit_result.get("cartesian_path_used"), moveit_plan_request.get("cartesian_path_used")),
        "cartesian_path_fraction": _first_not_none(moveit_result.get("cartesian_path_fraction"), moveit_plan_request.get("cartesian_path_fraction")),
        "joint_space_fallback_used": _first_not_none(moveit_result.get("joint_space_fallback_used"), moveit_plan_request.get("joint_space_fallback_used")),
        "joint_space_fallback_reason": _first_not_none(moveit_result.get("joint_space_fallback_reason"), moveit_plan_request.get("joint_space_fallback_reason")),
        "start_state_source": _first_not_none(moveit_result.get("start_state_source"), moveit_plan_request.get("start_state_source")),
        "start_state_is_diff": _first_not_none(moveit_result.get("start_state_is_diff"), moveit_plan_request.get("start_state_is_diff")),
        "explicit_start_state_provided": _first_not_none(moveit_result.get("explicit_start_state_provided"), moveit_plan_request.get("explicit_start_state_provided")),
        "current_joint_state_available": _first_not_none(moveit_result.get("current_joint_state_available"), moveit_plan_request.get("current_joint_state_available")),
        "current_joint_state_source": _first_not_none(moveit_result.get("current_joint_state_source"), moveit_plan_request.get("current_joint_state_source")),
        "current_joint_state_age_s": _first_not_none(moveit_result.get("current_joint_state_age_s"), moveit_plan_request.get("current_joint_state_age_s")),
        "target_orientation_source": _first_not_none(
            moveit_result.get("target_orientation_source"),
            moveit_plan_request.get("target_orientation_source"),
            motion.get("target_orientation_source"),
        ),
        "orientation_mode": _first_not_none(
            moveit_result.get("orientation_mode"),
            moveit_plan_request.get("orientation_mode"),
            motion.get("orientation_mode"),
        ),
        "orientation_locked": _first_not_none(
            moveit_result.get("orientation_locked"),
            moveit_plan_request.get("orientation_locked"),
            motion.get("orientation_locked"),
        ),
        "requested_start_tcp_pose": _first_not_none(moveit_result.get("requested_start_tcp_pose"), motion.get("requested_start_tcp_pose")),
        "requested_target_tcp_pose": _first_not_none(
            moveit_result.get("requested_target_tcp_pose"),
            motion.get("requested_target_tcp_pose"),
        ),
        **tolerance_evidence,
        "planned_goal_frame": motion.get("frame") or execution_preview.get("frame"),
        "target_frame": _first_not_none(moveit_result.get("target_frame"), moveit_plan_request.get("planning_frame"), motion.get("frame"), execution_preview.get("frame")),
        "current_tcp_frame": _first_not_none(moveit_result.get("current_tcp_frame"), (motion.get("current_tcp_pose") or {}).get("frame") if isinstance(motion.get("current_tcp_pose"), dict) else None),
        "moveit_end_effector_link": _first_not_none(moveit_result.get("moveit_end_effector_link"), moveit_plan_request.get("end_effector_frame")),
        "moveit_planning_frame": _first_not_none(moveit_result.get("moveit_planning_frame"), moveit_plan_request.get("planning_frame")),
        "moveit_group_name": _first_not_none(moveit_result.get("moveit_group_name"), moveit_plan_request.get("planning_group")),
        "metrics_source": trajectory_metrics["metrics_source"],
        "planned_waypoint_count": trajectory_metrics["planned_waypoint_count"],
        "estimated_cartesian_path_length_m": trajectory_metrics["estimated_cartesian_path_length_m"],
        "path_metric_source": _first_not_none(moveit_result.get("path_metric_source"), trajectory_metrics["metrics_source"]),
        "planned_joint_names": moveit_result.get("planned_joint_names"),
        "planned_start_joint_positions": moveit_result.get("planned_start_joint_positions"),
        "planned_final_joint_positions": moveit_result.get("planned_final_joint_positions"),
        "per_joint_delta_rad": moveit_result.get("per_joint_delta_rad"),
        "max_joint_delta_rad": max_joint_delta,
        "planned_joint_path_length_rad": _first_not_none(
            _optional_number(moveit_result.get("planned_joint_path_length_rad")),
            trajectory_metrics["total_joint_motion_rad"],
        ),
        "path_length_ratio": _first_not_none(
            _optional_number(moveit_result.get("path_length_ratio")),
            (
                round(float(trajectory_metrics["total_joint_motion_rad"]) / requested_distance, 6)
                if trajectory_metrics["total_joint_motion_rad"] is not None and requested_distance is not None and requested_distance > 0.0
                else None
            ),
        ),
        "wrist_joint_names": moveit_result.get("wrist_joint_names"),
        "wrist_joint_delta_rad": moveit_result.get("wrist_joint_delta_rad"),
        "max_wrist_joint_delta_rad": moveit_result.get("max_wrist_joint_delta_rad"),
        "joint_wrap_suspected": moveit_result.get("joint_wrap_suspected"),
        "joint_delta_audit_status": moveit_result.get("joint_delta_audit_status"),
        "joint_delta_audit_reason": moveit_result.get("joint_delta_audit_reason"),
        "planner_audit_warnings": moveit_result.get("planner_audit_warnings"),
        "planner_risk_policy_name": _first_not_none(
            moveit_result.get("planner_risk_policy_name"),
            moveit_plan_request.get("planner_risk_policy_name"),
        ),
        "planner_risk_policy_mode": _first_not_none(
            moveit_result.get("planner_risk_policy_mode"),
            moveit_plan_request.get("planner_risk_policy_mode"),
        ),
        "planner_risk_blocking_enabled": _first_not_none(
            moveit_result.get("planner_risk_blocking_enabled"),
            moveit_plan_request.get("planner_risk_blocking_enabled"),
        ),
        "warn_max_joint_delta_rad": _first_not_none(
            moveit_result.get("warn_max_joint_delta_rad"),
            moveit_plan_request.get("warn_max_joint_delta_rad"),
        ),
        "warn_max_wrist_joint_delta_rad": _first_not_none(
            moveit_result.get("warn_max_wrist_joint_delta_rad"),
            moveit_plan_request.get("warn_max_wrist_joint_delta_rad"),
        ),
        "warn_path_length_ratio": _first_not_none(
            moveit_result.get("warn_path_length_ratio"),
            moveit_plan_request.get("warn_path_length_ratio"),
        ),
        "warn_joint_wrap_suspected": _first_not_none(
            moveit_result.get("warn_joint_wrap_suspected"),
            moveit_plan_request.get("warn_joint_wrap_suspected"),
        ),
        "total_joint_motion_rad": trajectory_metrics["total_joint_motion_rad"],
        "orientation_change_rad": trajectory_metrics["orientation_change_rad"],
        "trajectory_duration_s": trajectory_metrics["trajectory_duration_s"],
        "reasonableness_check": reasonableness_check,
        "blocking_reasons": blocking_reasons,
        "warnings": warnings,
    }
    return {**result, **evaluate_planner_audit_risk(result)}


def _moveit_tolerance_evidence(
    *,
    requested_distance: float | None,
    moveit_result: dict[str, Any],
    moveit_plan_request: dict[str, Any],
) -> dict[str, Any]:
    position_tolerance = _first_not_none(
        _optional_number(moveit_result.get("moveit_position_tolerance_m")),
        _optional_number(moveit_plan_request.get("moveit_position_tolerance_m")),
    )
    orientation_tolerance = _first_not_none(
        _optional_number(moveit_result.get("moveit_orientation_tolerance_rad")),
        _optional_number(moveit_plan_request.get("moveit_orientation_tolerance_rad")),
    )
    ratio = _first_not_none(
        _optional_number(moveit_result.get("tolerance_to_requested_distance_ratio")),
        _optional_number(moveit_plan_request.get("tolerance_to_requested_distance_ratio")),
    )
    if ratio is None and requested_distance is not None and requested_distance > 0.0 and position_tolerance is not None:
        ratio = round(float(position_tolerance) / requested_distance, 6)
    return {
        "moveit_position_tolerance_m": position_tolerance,
        "moveit_orientation_tolerance_rad": orientation_tolerance,
        "tolerance_to_requested_distance_ratio": ratio,
        "small_motion_tolerance_policy": _first_not_none(
            moveit_result.get("small_motion_tolerance_policy"),
            moveit_plan_request.get("small_motion_tolerance_policy"),
        ),
    }


def _real_execution_audit_status(real_robot_motion_executed: bool, execution: dict[str, Any]) -> str:
    if real_robot_motion_executed:
        return STATUS_PASS
    if execution.get("cartesian_motion_execution_status") == STATUS_BLOCKED:
        return STATUS_BLOCKED
    return "NOT_RUN"


def _real_small_motion_gate(
    *,
    parsed: ParsedMotionCommand,
    execution_preview: dict[str, Any],
    planner_acceptance: dict[str, Any],
    parser_metadata: dict[str, Any],
) -> dict[str, Any]:
    blockers: list[str] = []
    axis, sign = _axis_direction_from_delta(parsed.delta_m)
    configured_max_distance = _optional_number(execution_preview.get("max_distance_m")) or DEFAULT_REAL_MAX_DISTANCE_M
    max_axis_step = _optional_number(planner_acceptance.get("max_axis_step_m")) or configured_max_distance
    hard_safety_limit = _optional_number(execution_preview.get("hard_safety_limit_m")) or DEFAULT_REAL_HARD_SAFETY_LIMIT_M
    requested_distance_within_configured_limit = parsed.distance_m <= configured_max_distance + EPS
    requested_distance_within_hard_limit = parsed.distance_m <= hard_safety_limit + EPS
    distance_allowed = requested_distance_within_configured_limit and requested_distance_within_hard_limit
    intent_allowed = execution_preview.get("intent") == "relative_cartesian_motion"
    frame_allowed = parsed.frame == "base_link"
    axis_allowed = axis in REAL_SMALL_MOTION_ALLOWED_AXES
    direction_allowed = sign in REAL_SMALL_MOTION_ALLOWED_DIRECTIONS
    must_confirm = True

    if parser_metadata.get("parser_source") != "qwen_llm":
        blockers.append("E_REAL_SMALL_MOTION_QWEN_REQUIRED")
    if not intent_allowed:
        blockers.append("E_REAL_SMALL_MOTION_INTENT_NOT_ALLOWED")
    if not frame_allowed:
        blockers.append("E_REAL_SMALL_MOTION_FRAME_NOT_ALLOWED")
    if not axis_allowed:
        blockers.append("E_REAL_SMALL_MOTION_AXIS_NOT_ALLOWED")
    if not direction_allowed:
        blockers.append("E_REAL_SMALL_MOTION_DIRECTION_NOT_ALLOWED")
    if not distance_allowed:
        blockers.append("E_REAL_SMALL_MOTION_DISTANCE_NOT_ALLOWED")
    if execution_preview.get("manual_confirmation_required") is not True:
        blockers.append("E_REAL_SMALL_MOTION_CONFIRMATION_NOT_REQUIRED")
    if execution_preview.get("preview_status") != STATUS_PASS:
        blockers.append("E_REAL_SMALL_MOTION_PREVIEW_NOT_PASS")
    if planner_acceptance.get("status") != STATUS_PASS:
        blockers.append("E_REAL_SMALL_MOTION_PLANNER_ACCEPTANCE_NOT_PASS")
    if planner_acceptance.get("trajectory_sent") is True:
        blockers.append("E_REAL_SMALL_MOTION_TRAJECTORY_ALREADY_SENT")
    if planner_acceptance.get("execute_trajectory_called") is True:
        blockers.append("E_REAL_SMALL_MOTION_EXECUTE_ALREADY_CALLED")
    blockers = _unique(blockers)
    return {
        "real_small_motion_gate_policy": REAL_SMALL_MOTION_GATE_POLICY,
        "real_small_motion_gate_basis": "normalized_contract",
        "allowed_axis": axis if axis_allowed else None,
        "allowed_direction": sign if direction_allowed else None,
        "allowed_distance_m": parsed.distance_m if distance_allowed else None,
        "configured_max_distance_m": configured_max_distance,
        "max_step_distance_m": configured_max_distance,
        "max_axis_step_m": max_axis_step,
        "hard_safety_limit_m": hard_safety_limit,
        "requested_distance_within_configured_limit": requested_distance_within_configured_limit,
        "motion_frame": parsed.frame,
        "direction_axis": axis,
        "direction_sign": sign,
        "base_link_direction_mapping": BASE_LINK_DIRECTION_MAPPING,
        "real_small_motion_command_allowed": not blockers,
        "normalized_intent": execution_preview.get("intent"),
        "normalized_frame": parsed.frame,
        "normalized_axis": axis,
        "normalized_direction": sign,
        "normalized_distance_m": parsed.distance_m,
        "normalized_delta_m": list(parsed.delta_m),
        "must_confirm": must_confirm,
        "blocking_reasons": blockers,
    }


def _moveit_result_from(motion: dict[str, Any], execution: dict[str, Any]) -> dict[str, Any]:
    if isinstance(execution.get("moveit_pose_executor_result"), dict):
        return execution["moveit_pose_executor_result"]
    if isinstance(motion.get("moveit_pose_executor_result"), dict):
        return motion["moveit_pose_executor_result"]
    return {}


def _trajectory_metrics(moveit_result: dict[str, Any]) -> dict[str, Any]:
    waypoint_count = _first_not_none(
        moveit_result.get("trajectory_point_count"),
        moveit_result.get("planned_waypoint_count"),
    )
    waypoint_count = int(waypoint_count) if isinstance(waypoint_count, int) and not isinstance(waypoint_count, bool) else None
    joint_points = _joint_trajectory_points(moveit_result)
    max_joint_delta = _max_joint_delta(joint_points)
    total_joint_motion = _total_joint_motion(joint_points)
    if waypoint_count is None and joint_points:
        waypoint_count = len(joint_points)
    cartesian_waypoints = _cartesian_waypoints(moveit_result)
    return {
        "metrics_source": _metrics_source(moveit_result, joint_points, cartesian_waypoints),
        "planned_waypoint_count": waypoint_count,
        "estimated_cartesian_path_length_m": _first_not_none(
            _optional_number(moveit_result.get("estimated_cartesian_path_length_m")),
            _cartesian_path_length(cartesian_waypoints),
        ),
        "max_joint_delta_rad": max_joint_delta,
        "total_joint_motion_rad": total_joint_motion,
        "orientation_change_rad": _first_not_none(
            _optional_number(moveit_result.get("orientation_change_rad")),
            _orientation_change_from_waypoints(cartesian_waypoints),
        ),
        "trajectory_duration_s": _trajectory_duration(moveit_result),
    }


def _metrics_source(moveit_result: dict[str, Any], joint_points: list[list[float]], cartesian_waypoints: list[dict[str, Any]]) -> str:
    if not moveit_result:
        return "not_available"
    if moveit_result.get("planned_trajectory") is not None or moveit_result.get("moveit_plan_called") is True:
        return "moveit_plan_only"
    if joint_points or cartesian_waypoints or moveit_result.get("trajectory_point_count") is not None:
        return "mock_plan_only"
    return "not_available"


def _joint_trajectory_points(moveit_result: dict[str, Any]) -> list[list[float]]:
    raw_points = moveit_result.get("joint_trajectory_points") or moveit_result.get("planned_joint_positions")
    trajectory = moveit_result.get("planned_trajectory")
    if raw_points is None:
        raw_points = _raw_joint_points_from_trajectory(trajectory)
    if not isinstance(raw_points, list):
        return []
    points: list[list[float]] = []
    for raw in raw_points:
        positions = raw.get("positions") if isinstance(raw, dict) else getattr(raw, "positions", raw)
        if not isinstance(positions, list) or not positions:
            continue
        values = [_optional_number(value) for value in positions]
        if any(value is None for value in values):
            continue
        points.append([float(value) for value in values if value is not None])
    return points


def _raw_joint_points_from_trajectory(trajectory: Any) -> list[Any]:
    if isinstance(trajectory, dict):
        joint_trajectory = trajectory.get("joint_trajectory")
        if isinstance(joint_trajectory, dict):
            return joint_trajectory.get("points") if isinstance(joint_trajectory.get("points"), list) else []
    joint_trajectory = getattr(trajectory, "joint_trajectory", None)
    raw_points = getattr(joint_trajectory, "points", None)
    return list(raw_points) if raw_points is not None else []


def _max_joint_delta(points: list[list[float]]) -> float | None:
    if len(points) < 2:
        return None
    max_delta = 0.0
    for left, right in zip(points, points[1:]):
        if len(left) != len(right):
            continue
        max_delta = max(max_delta, max(abs(float(b) - float(a)) for a, b in zip(left, right)))
    return round(max_delta, 6)


def _total_joint_motion(points: list[list[float]]) -> float | None:
    if len(points) < 2:
        return None
    total = 0.0
    for left, right in zip(points, points[1:]):
        if len(left) != len(right):
            continue
        total += sum(abs(float(b) - float(a)) for a, b in zip(left, right))
    return round(total, 6)


def _trajectory_duration(moveit_result: dict[str, Any]) -> float | None:
    explicit = _optional_number(moveit_result.get("trajectory_duration_s"))
    if explicit is not None:
        return explicit
    raw_points = _raw_joint_points_from_trajectory(moveit_result.get("planned_trajectory"))
    if not raw_points:
        raw_points = moveit_result.get("joint_trajectory_points") if isinstance(moveit_result.get("joint_trajectory_points"), list) else []
    if not raw_points:
        return None
    last = raw_points[-1]
    duration = last.get("time_from_start") if isinstance(last, dict) else getattr(last, "time_from_start", None)
    return _duration_seconds(duration)


def _duration_seconds(duration: Any) -> float | None:
    if duration is None:
        return None
    number = _optional_number(duration)
    if number is not None:
        return number
    if isinstance(duration, dict):
        sec = _optional_number(duration.get("sec", duration.get("secs")))
        nanosec = _optional_number(duration.get("nanosec", duration.get("nsecs")))
        if sec is None:
            return None
        return round(sec + (nanosec or 0.0) / 1_000_000_000.0, 6)
    if hasattr(duration, "to_msg"):
        try:
            duration = duration.to_msg()
        except Exception:
            return None
    sec = _optional_number(getattr(duration, "sec", None))
    nanosec = _optional_number(getattr(duration, "nanosec", None))
    if sec is None:
        sec = _optional_number(getattr(duration, "secs", None))
    if nanosec is None:
        nanosec = _optional_number(getattr(duration, "nsecs", None))
    if sec is None:
        return None
    return round(sec + (nanosec or 0.0) / 1_000_000_000.0, 6)


def _cartesian_waypoints(moveit_result: dict[str, Any]) -> list[dict[str, Any]]:
    raw_points = (
        moveit_result.get("cartesian_waypoints")
        or moveit_result.get("tcp_waypoints_m")
        or moveit_result.get("pose_waypoints")
    )
    if isinstance(raw_points, list):
        return [_normalize_cartesian_waypoint(point) for point in raw_points if _normalize_cartesian_waypoint(point)]
    return _multi_dof_waypoints_from_trajectory(moveit_result.get("planned_trajectory"))


def _normalize_cartesian_waypoint(point: Any) -> dict[str, Any] | None:
    if isinstance(point, dict):
        position = point.get("position_m") or point.get("position") or point.get("xyz")
        orientation = point.get("orientation_xyzw") or point.get("orientation") or point.get("quat_xyzw")
        if _vector3(position):
            return {
                "position_m": [float(value) for value in position],
                "orientation_xyzw": list(orientation) if _quaternion(orientation) else None,
            }
    if isinstance(point, list) and len(point) in {3, 7} and _vector3(point[:3]):
        return {
            "position_m": [float(value) for value in point[:3]],
            "orientation_xyzw": list(point[3:7]) if len(point) == 7 and _quaternion(point[3:7]) else None,
        }
    return None


def _multi_dof_waypoints_from_trajectory(trajectory: Any) -> list[dict[str, Any]]:
    if trajectory is None:
        return []
    multi_dof = getattr(trajectory, "multi_dof_joint_trajectory", None)
    raw_points = getattr(multi_dof, "points", None)
    if raw_points is None and isinstance(trajectory, dict):
        raw_multi = trajectory.get("multi_dof_joint_trajectory")
        raw_points = raw_multi.get("points") if isinstance(raw_multi, dict) else None
    waypoints = []
    for point in list(raw_points or []):
        transforms = point.get("transforms") if isinstance(point, dict) else getattr(point, "transforms", None)
        if not transforms:
            continue
        transform = transforms[0]
        translation = transform.get("translation") if isinstance(transform, dict) else getattr(transform, "translation", None)
        rotation = transform.get("rotation") if isinstance(transform, dict) else getattr(transform, "rotation", None)
        position = _xyz_from_object(translation)
        orientation = _xyzw_from_object(rotation)
        if _vector3(position):
            waypoints.append({"position_m": position, "orientation_xyzw": orientation if _quaternion(orientation) else None})
    return waypoints


def _cartesian_path_length(waypoints: list[dict[str, Any]]) -> float | None:
    if len(waypoints) < 2:
        return None
    total = 0.0
    for left, right in zip(waypoints, waypoints[1:]):
        left_position = left.get("position_m")
        right_position = right.get("position_m")
        if not _vector3(left_position) or not _vector3(right_position):
            continue
        total += math.sqrt(sum((float(b) - float(a)) ** 2 for a, b in zip(left_position, right_position)))
    return round(total, 6)


def _orientation_change_from_waypoints(waypoints: list[dict[str, Any]]) -> float | None:
    orientations = [point.get("orientation_xyzw") for point in waypoints if _quaternion(point.get("orientation_xyzw"))]
    if len(orientations) < 2:
        return None
    return _quaternion_angle(orientations[0], orientations[-1])


def _quaternion_angle(left: list[float], right: list[float]) -> float:
    dot = abs(sum(float(a) * float(b) for a, b in zip(left, right)))
    dot = max(0.0, min(1.0, dot))
    return round(2.0 * math.acos(dot), 6)


def _xyz_from_object(value: Any) -> list[float] | None:
    if isinstance(value, dict):
        result = [value.get(axis) for axis in ("x", "y", "z")]
    else:
        result = [getattr(value, axis, None) for axis in ("x", "y", "z")]
    numbers = [_optional_number(item) for item in result]
    if any(item is None for item in numbers):
        return None
    return [float(item) for item in numbers if item is not None]


def _xyzw_from_object(value: Any) -> list[float] | None:
    if isinstance(value, dict):
        result = [value.get(axis) for axis in ("x", "y", "z", "w")]
    else:
        result = [getattr(value, axis, None) for axis in ("x", "y", "z", "w")]
    numbers = [_optional_number(item) for item in result]
    if any(item is None for item in numbers):
        return None
    return [float(item) for item in numbers if item is not None]


def _evidence(
    *,
    status: str,
    motion: dict[str, Any],
    execution: dict[str, Any],
    parsed_contract: dict[str, Any],
    parser_metadata: dict[str, Any],
    planner_acceptance: dict[str, Any] | None = None,
    acceptance_mode: str | None = None,
    current_tcp_pose: dict[str, Any] | None = None,
    post_motion_tcp_pose: dict[str, Any] | None = None,
    manual_confirmation_received: bool = False,
    blocking_reasons: list[str] | None = None,
    real_small_motion_gate: dict[str, Any] | None = None,
) -> dict[str, Any]:
    moveit = execution.get("moveit_pose_executor_result") if isinstance(execution.get("moveit_pose_executor_result"), dict) else {}
    motion_check = _motion_check_fields(motion, moveit, parsed_contract)
    reasons = blocking_reasons if blocking_reasons is not None else execution.get("blocking_reasons", [])
    trajectory_sent = execution.get("trajectory_sent", False)
    controller_command_sent = execution.get("controller_command_sent", False)
    real_robot_motion_executed = execution.get("real_robot_motion_executed", False)
    execute_trajectory_called = (
        execution.get("moveit_execute_called") is True
        or (isinstance(planner_acceptance, dict) and planner_acceptance.get("execute_trajectory_called") is True)
    )
    post_motion = _post_motion_verification(
        real_robot_motion_executed=real_robot_motion_executed is True,
        before_tcp_pose=current_tcp_pose,
        target_tcp_pose=motion.get("target_pose"),
        after_tcp_pose=post_motion_tcp_pose,
        intended_delta_m=_intended_delta_from_contract(parsed_contract),
        reason=None if real_robot_motion_executed is True else "real_robot_motion_executed=false",
    )
    final_status = (
        STATUS_FAILED
        if real_robot_motion_executed is True and post_motion.get("post_motion_verification_status") == STATUS_FAILED
        else status
    )
    return {
        **parser_metadata,
        "parsed_contract": parsed_contract,
        "execution_preview": parsed_contract.get("execution_preview"),
        "planner_acceptance": planner_acceptance,
        "current_tcp_pose": current_tcp_pose or _current_tcp_pose_unavailable(real_required=False),
        "acceptance_workflow": _acceptance_workflow(
            status=status,
            mode=acceptance_mode,
            parser_metadata=parser_metadata,
            parsed_contract=parsed_contract,
            planner_acceptance=planner_acceptance,
            manual_confirmation_received=manual_confirmation_received,
            real_execution_allowed=execution.get("trajectory_send_allowed") is True,
            trajectory_sent=trajectory_sent,
            execute_trajectory_called=execute_trajectory_called,
            controller_command_sent=controller_command_sent,
            real_robot_motion_executed=real_robot_motion_executed,
            blocking_reasons=reasons,
        ),
        "cartesian_motion_gateway_status": motion.get("cartesian_motion_gateway_status"),
        "cartesian_motion_execution_status": execution.get("cartesian_motion_execution_status"),
        "goal_accepted": moveit.get("goal_accepted", False),
        "execute_accepted": moveit.get("execute_success", False),
        "moveit_execute_error_code": moveit.get("moveit_execute_error_code"),
        "moveit_execute_error_code_name": moveit.get("moveit_execute_error_code_name"),
        "trajectory_sent": trajectory_sent,
        "execute_trajectory_called": execute_trajectory_called,
        "controller_command_sent": controller_command_sent,
        "real_robot_motion_executed": real_robot_motion_executed,
        "manual_confirmation_required": True,
        "manual_confirmation_status": (
            "ACCEPTED" if manual_confirmation_received else "NOT_RECEIVED"
        ),
        **_real_small_motion_gate_fields(real_small_motion_gate),
        "requested_distance_m": planner_acceptance.get("requested_distance_m") if isinstance(planner_acceptance, dict) else None,
        "configured_max_distance_m": planner_acceptance.get("configured_max_distance_m") if isinstance(planner_acceptance, dict) else None,
        "motion_frame": planner_acceptance.get("motion_frame") if isinstance(planner_acceptance, dict) else None,
        "direction_axis": planner_acceptance.get("direction_axis") if isinstance(planner_acceptance, dict) else None,
        "direction_sign": planner_acceptance.get("direction_sign") if isinstance(planner_acceptance, dict) else None,
        "first_move_bootstrap_used": planner_acceptance.get("first_move_bootstrap_used") if isinstance(planner_acceptance, dict) else None,
        "previous_verified_tcp_pose": planner_acceptance.get("previous_verified_tcp_pose") if isinstance(planner_acceptance, dict) else None,
        "current_measured_tcp_pose": planner_acceptance.get("current_measured_tcp_pose") if isinstance(planner_acceptance, dict) else None,
        "requested_target_tcp_pose": planner_acceptance.get("requested_target_tcp_pose") if isinstance(planner_acceptance, dict) else None,
        "delta_from_current_tcp_m": planner_acceptance.get("delta_from_current_tcp_m") if isinstance(planner_acceptance, dict) else None,
        "delta_from_previous_verified_tcp_m": planner_acceptance.get("delta_from_previous_verified_tcp_m") if isinstance(planner_acceptance, dict) else None,
        "max_step_distance_m": planner_acceptance.get("max_step_distance_m") if isinstance(planner_acceptance, dict) else None,
        "max_axis_step_m": planner_acceptance.get("max_axis_step_m") if isinstance(planner_acceptance, dict) else None,
        "hard_safety_limit_m": planner_acceptance.get("hard_safety_limit_m") if isinstance(planner_acceptance, dict) else None,
        "session_radius_limit_m": planner_acceptance.get("session_radius_limit_m") if isinstance(planner_acceptance, dict) else None,
        "motion_distance_regime": planner_acceptance.get("motion_distance_regime") if isinstance(planner_acceptance, dict) else None,
        "motion_permission_envelope_version": planner_acceptance.get("motion_permission_envelope_version") if isinstance(planner_acceptance, dict) else None,
        "long_step_decomposition_enabled": planner_acceptance.get("long_step_decomposition_enabled") if isinstance(planner_acceptance, dict) else None,
        "decomposition_enabled": planner_acceptance.get("decomposition_enabled") if isinstance(planner_acceptance, dict) else None,
        "long_step_policy_name": planner_acceptance.get("long_step_policy_name") if isinstance(planner_acceptance, dict) else None,
        "requested_total_distance_m": planner_acceptance.get("requested_total_distance_m") if isinstance(planner_acceptance, dict) else None,
        "intent_name": planner_acceptance.get("intent_name") if isinstance(planner_acceptance, dict) else None,
        "motion_contract_type": planner_acceptance.get("motion_contract_type") if isinstance(planner_acceptance, dict) else None,
        "requested_vector_m": planner_acceptance.get("requested_vector_m") if isinstance(planner_acceptance, dict) else None,
        "normalized_direction_vector": planner_acceptance.get("normalized_direction_vector") if isinstance(planner_acceptance, dict) else None,
        "shared_max_total_distance_m": planner_acceptance.get("shared_max_total_distance_m") if isinstance(planner_acceptance, dict) else None,
        "distance_within_shared_envelope": planner_acceptance.get("distance_within_shared_envelope") if isinstance(planner_acceptance, dict) else None,
        "decomposition_required": planner_acceptance.get("decomposition_required") if isinstance(planner_acceptance, dict) else None,
        "execution_backend": planner_acceptance.get("execution_backend") if isinstance(planner_acceptance, dict) else None,
        "backend_policy_id": planner_acceptance.get("backend_policy_id") if isinstance(planner_acceptance, dict) else None,
        "one_shot_execution_allowed": planner_acceptance.get("one_shot_execution_allowed") if isinstance(planner_acceptance, dict) else None,
        "one_shot_target_pose_created": planner_acceptance.get("one_shot_target_pose_created") if isinstance(planner_acceptance, dict) else None,
        "one_shot_real_motion_allowed": planner_acceptance.get("one_shot_real_motion_allowed") if isinstance(planner_acceptance, dict) else None,
        "planned_subgoals": planner_acceptance.get("planned_subgoals") if isinstance(planner_acceptance, dict) else None,
        "one_shot_distance_limit_m": planner_acceptance.get("one_shot_distance_limit_m") if isinstance(planner_acceptance, dict) else None,
        "max_one_shot_distance_m": planner_acceptance.get("max_one_shot_distance_m") if isinstance(planner_acceptance, dict) else None,
        "hard_single_step_safety_limit_m": planner_acceptance.get("hard_single_step_safety_limit_m") if isinstance(planner_acceptance, dict) else None,
        "long_motion_total_limit_m": planner_acceptance.get("long_motion_total_limit_m") if isinstance(planner_acceptance, dict) else None,
        "max_decomposed_total_distance_m": planner_acceptance.get("max_decomposed_total_distance_m") if isinstance(planner_acceptance, dict) else None,
        "max_substep_distance_m": planner_acceptance.get("max_substep_distance_m") if isinstance(planner_acceptance, dict) else None,
        "max_decomposed_substep_distance_m": planner_acceptance.get("max_decomposed_substep_distance_m") if isinstance(planner_acceptance, dict) else None,
        "min_final_substep_distance_m": planner_acceptance.get("min_final_substep_distance_m") if isinstance(planner_acceptance, dict) else None,
        "planned_execution_style": planner_acceptance.get("planned_execution_style") if isinstance(planner_acceptance, dict) else None,
        "substep_execution_mode": planner_acceptance.get("substep_execution_mode") if isinstance(planner_acceptance, dict) else None,
        "real_substep_execution_enabled": planner_acceptance.get("real_substep_execution_enabled") if isinstance(planner_acceptance, dict) else None,
        "planned_substep_count": planner_acceptance.get("planned_substep_count") if isinstance(planner_acceptance, dict) else None,
        "planned_substep_distances_m": planner_acceptance.get("planned_substep_distances_m") if isinstance(planner_acceptance, dict) else None,
        "planned_substep_vectors_m": planner_acceptance.get("planned_substep_vectors_m") if isinstance(planner_acceptance, dict) else None,
        "substep_count": planner_acceptance.get("substep_count") if isinstance(planner_acceptance, dict) else None,
        "decomposed_substeps_m": planner_acceptance.get("decomposed_substeps_m") if isinstance(planner_acceptance, dict) else None,
        "decomposed_total_distance_m": planner_acceptance.get("decomposed_total_distance_m") if isinstance(planner_acceptance, dict) else None,
        "decomposition_remainder_m": planner_acceptance.get("decomposition_remainder_m") if isinstance(planner_acceptance, dict) else None,
        "decomposition_status": planner_acceptance.get("decomposition_status") if isinstance(planner_acceptance, dict) else None,
        "decomposition_blocking_reason": planner_acceptance.get("decomposition_blocking_reason") if isinstance(planner_acceptance, dict) else None,
        "decomposition_does_not_bypass_safety_limits": planner_acceptance.get("decomposition_does_not_bypass_safety_limits") if isinstance(planner_acceptance, dict) else None,
        "safety_gate_scope": planner_acceptance.get("safety_gate_scope") if isinstance(planner_acceptance, dict) else None,
        "one_shot_distance_check_status": planner_acceptance.get("one_shot_distance_check_status") if isinstance(planner_acceptance, dict) else None,
        "substep_distance_check_status": planner_acceptance.get("substep_distance_check_status") if isinstance(planner_acceptance, dict) else None,
        "total_long_motion_check_status": planner_acceptance.get("total_long_motion_check_status") if isinstance(planner_acceptance, dict) else None,
        "workspace_envelope_check_status": planner_acceptance.get("workspace_envelope_check_status") if isinstance(planner_acceptance, dict) else None,
        "decomposed_motion_allowed": planner_acceptance.get("decomposed_motion_allowed") if isinstance(planner_acceptance, dict) else None,
        "decomposed_motion_blocking_reason": planner_acceptance.get("decomposed_motion_blocking_reason") if isinstance(planner_acceptance, dict) else None,
        "autoregressive_update_required": planner_acceptance.get("autoregressive_update_required") if isinstance(planner_acceptance, dict) else None,
        "substep_feedback_required": planner_acceptance.get("substep_feedback_required") if isinstance(planner_acceptance, dict) else None,
        "substep_reobserve_allowed": planner_acceptance.get("substep_reobserve_allowed") if isinstance(planner_acceptance, dict) else None,
        "step_delta_within_limit": planner_acceptance.get("step_delta_within_limit") if isinstance(planner_acceptance, dict) else None,
        "axis_delta_within_limit": planner_acceptance.get("axis_delta_within_limit") if isinstance(planner_acceptance, dict) else None,
        "workspace_envelope_within_limit": planner_acceptance.get("workspace_envelope_within_limit") if isinstance(planner_acceptance, dict) else None,
        "base_link_direction_mapping": planner_acceptance.get("base_link_direction_mapping") if isinstance(planner_acceptance, dict) else None,
        "requested_distance_within_configured_limit": planner_acceptance.get("requested_distance_within_configured_limit") if isinstance(planner_acceptance, dict) else None,
        "safety_policy_source": planner_acceptance.get("safety_policy_source") if isinstance(planner_acceptance, dict) else None,
        "safety_policy_name": planner_acceptance.get("safety_policy_name") if isinstance(planner_acceptance, dict) else None,
        "moveit_position_tolerance_m": planner_acceptance.get("moveit_position_tolerance_m") if isinstance(planner_acceptance, dict) else None,
        "moveit_orientation_tolerance_rad": planner_acceptance.get("moveit_orientation_tolerance_rad") if isinstance(planner_acceptance, dict) else None,
        "tolerance_to_requested_distance_ratio": planner_acceptance.get("tolerance_to_requested_distance_ratio") if isinstance(planner_acceptance, dict) else None,
        "small_motion_tolerance_policy": planner_acceptance.get("small_motion_tolerance_policy") if isinstance(planner_acceptance, dict) else None,
        "target_frame": planner_acceptance.get("target_frame") if isinstance(planner_acceptance, dict) else None,
        "current_tcp_frame": planner_acceptance.get("current_tcp_frame") if isinstance(planner_acceptance, dict) else None,
        "moveit_end_effector_link": planner_acceptance.get("moveit_end_effector_link") if isinstance(planner_acceptance, dict) else None,
        "moveit_planning_frame": planner_acceptance.get("moveit_planning_frame") if isinstance(planner_acceptance, dict) else None,
        "moveit_group_name": planner_acceptance.get("moveit_group_name") if isinstance(planner_acceptance, dict) else None,
        "planner_mode": planner_acceptance.get("planner_mode") if isinstance(planner_acceptance, dict) else None,
        "planning_pipeline_id": planner_acceptance.get("planning_pipeline_id") if isinstance(planner_acceptance, dict) else None,
        "planner_id": planner_acceptance.get("planner_id") if isinstance(planner_acceptance, dict) else None,
        "moveit_goal_type": planner_acceptance.get("moveit_goal_type") if isinstance(planner_acceptance, dict) else None,
        "joint_space_pose_goal_used": planner_acceptance.get("joint_space_pose_goal_used") if isinstance(planner_acceptance, dict) else None,
        "cartesian_path_used": planner_acceptance.get("cartesian_path_used") if isinstance(planner_acceptance, dict) else None,
        "cartesian_path_fraction": planner_acceptance.get("cartesian_path_fraction") if isinstance(planner_acceptance, dict) else None,
        "joint_space_fallback_used": planner_acceptance.get("joint_space_fallback_used") if isinstance(planner_acceptance, dict) else None,
        "joint_space_fallback_reason": planner_acceptance.get("joint_space_fallback_reason") if isinstance(planner_acceptance, dict) else None,
        "start_state_source": planner_acceptance.get("start_state_source") if isinstance(planner_acceptance, dict) else None,
        "start_state_is_diff": planner_acceptance.get("start_state_is_diff") if isinstance(planner_acceptance, dict) else None,
        "explicit_start_state_provided": planner_acceptance.get("explicit_start_state_provided") if isinstance(planner_acceptance, dict) else None,
        "current_joint_state_available": planner_acceptance.get("current_joint_state_available") if isinstance(planner_acceptance, dict) else None,
        "current_joint_state_source": planner_acceptance.get("current_joint_state_source") if isinstance(planner_acceptance, dict) else None,
        "current_joint_state_age_s": planner_acceptance.get("current_joint_state_age_s") if isinstance(planner_acceptance, dict) else None,
        "target_orientation_source": planner_acceptance.get("target_orientation_source") if isinstance(planner_acceptance, dict) else None,
        "orientation_mode": planner_acceptance.get("orientation_mode") if isinstance(planner_acceptance, dict) else None,
        "orientation_locked": planner_acceptance.get("orientation_locked") if isinstance(planner_acceptance, dict) else None,
        "requested_start_tcp_pose": planner_acceptance.get("requested_start_tcp_pose") if isinstance(planner_acceptance, dict) else None,
        "target_pose": motion.get("target_pose"),
        "moveit_plan_request": motion.get("moveit_plan_request"),
        "planned_joint_names": planner_acceptance.get("planned_joint_names") if isinstance(planner_acceptance, dict) else None,
        "planned_start_joint_positions": planner_acceptance.get("planned_start_joint_positions") if isinstance(planner_acceptance, dict) else None,
        "planned_final_joint_positions": planner_acceptance.get("planned_final_joint_positions") if isinstance(planner_acceptance, dict) else None,
        "per_joint_delta_rad": planner_acceptance.get("per_joint_delta_rad") if isinstance(planner_acceptance, dict) else None,
        "planned_joint_path_length_rad": planner_acceptance.get("planned_joint_path_length_rad") if isinstance(planner_acceptance, dict) else None,
        "path_length_ratio": planner_acceptance.get("path_length_ratio") if isinstance(planner_acceptance, dict) else None,
        "path_metric_source": planner_acceptance.get("path_metric_source") if isinstance(planner_acceptance, dict) else None,
        "wrist_joint_names": planner_acceptance.get("wrist_joint_names") if isinstance(planner_acceptance, dict) else None,
        "wrist_joint_delta_rad": planner_acceptance.get("wrist_joint_delta_rad") if isinstance(planner_acceptance, dict) else None,
        "max_wrist_joint_delta_rad": planner_acceptance.get("max_wrist_joint_delta_rad") if isinstance(planner_acceptance, dict) else None,
        "joint_wrap_suspected": planner_acceptance.get("joint_wrap_suspected") if isinstance(planner_acceptance, dict) else None,
        "joint_delta_audit_status": planner_acceptance.get("joint_delta_audit_status") if isinstance(planner_acceptance, dict) else None,
        "joint_delta_audit_reason": planner_acceptance.get("joint_delta_audit_reason") if isinstance(planner_acceptance, dict) else None,
        "planner_audit_warnings": planner_acceptance.get("planner_audit_warnings") if isinstance(planner_acceptance, dict) else None,
        "planner_risk_status": planner_acceptance.get("planner_risk_status") if isinstance(planner_acceptance, dict) else None,
        "planner_risk_reasons": planner_acceptance.get("planner_risk_reasons") if isinstance(planner_acceptance, dict) else None,
        "planner_risk_warnings": planner_acceptance.get("planner_risk_warnings") if isinstance(planner_acceptance, dict) else None,
        "planner_risk_infos": planner_acceptance.get("planner_risk_infos") if isinstance(planner_acceptance, dict) else None,
        "planner_risk_policy_name": planner_acceptance.get("planner_risk_policy_name") if isinstance(planner_acceptance, dict) else None,
        "planner_risk_policy_mode": planner_acceptance.get("planner_risk_policy_mode") if isinstance(planner_acceptance, dict) else None,
        "planner_risk_blocking_enabled": planner_acceptance.get("planner_risk_blocking_enabled") if isinstance(planner_acceptance, dict) else None,
        "planner_risk_blocking_reason": planner_acceptance.get("planner_risk_blocking_reason") if isinstance(planner_acceptance, dict) else None,
        "planner_risk_thresholds": planner_acceptance.get("planner_risk_thresholds") if isinstance(planner_acceptance, dict) else None,
        **_post_motion_top_level_fields(post_motion),
        **motion_check,
        "blocking_reasons": reasons,
        "warnings": execution.get("warnings", []),
        "final_status": final_status,
    }


def _real_small_motion_gate_fields(gate: dict[str, Any] | None) -> dict[str, Any]:
    if not isinstance(gate, dict):
        return {}
    return {
        "real_small_motion_gate": gate,
        "real_small_motion_gate_policy": gate.get("real_small_motion_gate_policy"),
        "real_small_motion_gate_basis": gate.get("real_small_motion_gate_basis"),
        "allowed_axis": gate.get("allowed_axis"),
        "allowed_direction": gate.get("allowed_direction"),
        "allowed_distance_m": gate.get("allowed_distance_m"),
        "motion_frame": gate.get("motion_frame"),
        "direction_axis": gate.get("direction_axis"),
        "direction_sign": gate.get("direction_sign"),
        "max_step_distance_m": gate.get("max_step_distance_m"),
        "max_axis_step_m": gate.get("max_axis_step_m"),
        "base_link_direction_mapping": gate.get("base_link_direction_mapping"),
        "real_small_motion_command_allowed": gate.get("real_small_motion_command_allowed") is True,
    }


def _blocked_result(
    reason: str,
    parsed_contract: dict[str, Any],
    parser_metadata: dict[str, Any],
    *,
    acceptance_mode: str | None = None,
    current_tcp_pose: dict[str, Any] | None = None,
) -> dict[str, Any]:
    current_tcp_pose = current_tcp_pose or _current_tcp_pose_unavailable(real_required=False)
    post_motion = _post_motion_verification(
        real_robot_motion_executed=False,
        before_tcp_pose=current_tcp_pose,
        target_tcp_pose=None,
        after_tcp_pose=None,
        intended_delta_m=_intended_delta_from_contract(parsed_contract),
        reason="real_robot_motion_executed=false",
    )
    return {
        **parser_metadata,
        "parsed_contract": parsed_contract,
        "execution_preview": parsed_contract.get("execution_preview"),
        "planner_acceptance": None,
        "current_tcp_pose": current_tcp_pose,
        "acceptance_workflow": _acceptance_workflow(
            status=STATUS_BLOCKED,
            mode=acceptance_mode,
            parser_metadata=parser_metadata,
            parsed_contract=parsed_contract,
            planner_acceptance=None,
            trajectory_sent=False,
            execute_trajectory_called=False,
            controller_command_sent=False,
            real_robot_motion_executed=False,
            blocking_reasons=[reason],
        ),
        **_empty_motion_check(parsed_contract),
        "goal_accepted": False,
        "execute_accepted": False,
        "moveit_execute_error_code": None,
        "moveit_execute_error_code_name": None,
        "trajectory_sent": False,
        "execute_trajectory_called": False,
        "controller_command_sent": False,
        "real_robot_motion_executed": False,
        **_post_motion_top_level_fields(post_motion),
        "blocking_reasons": [reason],
        "final_status": STATUS_BLOCKED,
    }


def _post_motion_verification(
    *,
    real_robot_motion_executed: bool,
    before_tcp_pose: dict[str, Any] | None,
    target_tcp_pose: dict[str, Any] | None,
    after_tcp_pose: dict[str, Any] | None,
    intended_delta_m: list[float] | None,
    reason: str | None = None,
    distance_tolerance_m: float = 0.005,
    orientation_tolerance_rad: float = DEFAULT_REAL_ORIENTATION_TOLERANCE_RAD,
) -> dict[str, Any]:
    before = _tcp_pose_for_evidence(before_tcp_pose)
    target = _tcp_pose_for_evidence(target_tcp_pose)
    intended_delta = [round(float(value), 6) for value in intended_delta_m] if _vector3(intended_delta_m) else None
    intended_direction = _direction_label_from_delta(intended_delta)

    result: dict[str, Any] = {
        "tcp_pose_before_execution": before,
        "target_tcp_pose": target,
        "tcp_pose_after_execution": None,
        "tcp_pose_before_stamp": before.get("tcp_pose_stamp") if isinstance(before, dict) else None,
        "tcp_pose_after_stamp": None,
        "tcp_pose_before_age_s": before.get("tcp_pose_age_s") if isinstance(before, dict) else None,
        "tcp_pose_after_age_s": None,
        "tcp_pose_sample_settle_s": None,
        "tcp_pose_sample_attempts": None,
        "tcp_pose_stale_check_passed": None,
        "tcp_pose_stale_after_s": TCP_POSE_STALE_AFTER_S,
        "intended_delta_m": intended_delta,
        "actual_displacement_m": None,
        "actual_displacement_distance_m": None,
        "actual_distance_error_m": None,
        "intended_direction": intended_direction,
        "actual_direction": None,
        "direction_check_passed": False,
        "orientation_change_rad": None,
        "post_motion_distance_tolerance_m": distance_tolerance_m,
        "post_motion_orientation_tolerance_rad": orientation_tolerance_rad,
        "orientation_check_passed": None,
        "post_motion_verification_status": "NOT_RUN",
        "reason": reason,
    }
    if not real_robot_motion_executed:
        result["reason"] = reason or "real_robot_motion_executed=false"
        return result

    after = _current_tcp_pose_evidence(
        after_tcp_pose or {},
        source="real_robot_state_after_execution",
        allowed_for_real_execution=True,
    )
    result["tcp_pose_after_execution"] = after
    result["tcp_pose_after_stamp"] = after.get("tcp_pose_stamp") if isinstance(after, dict) else None
    result["tcp_pose_after_age_s"] = after.get("tcp_pose_age_s") if isinstance(after, dict) else None
    result["tcp_pose_sample_settle_s"] = after.get("tcp_pose_sample_settle_s") if isinstance(after, dict) else None
    result["tcp_pose_sample_attempts"] = after.get("tcp_pose_sample_attempts") if isinstance(after, dict) else None
    if before is None or target is None or after is None:
        result["post_motion_verification_status"] = STATUS_FAILED
        result["reason"] = "tcp_pose_before_target_or_after_unavailable"
        return result

    stale_check = _tcp_pose_stale_check(before, after)
    result["tcp_pose_stale_check_passed"] = stale_check
    if stale_check is False:
        result["post_motion_verification_status"] = STATUS_FAILED
        result["reason"] = "tcp_pose_stamp_stale"
        return result

    before_position = before.get("position_m")
    after_position = after.get("position_m")
    if not _vector3(before_position) or not _vector3(after_position):
        result["post_motion_verification_status"] = STATUS_FAILED
        result["reason"] = "tcp_position_before_or_after_unavailable"
        return result

    actual_displacement = [float(right) - float(left) for left, right in zip(before_position, after_position)]
    actual_distance = math.sqrt(sum(value**2 for value in actual_displacement))
    intended_distance = math.sqrt(sum(float(value) ** 2 for value in intended_delta)) if intended_delta is not None else None
    intended_axis, intended_sign = _axis_direction_from_delta(intended_delta) if intended_delta is not None else (None, None)
    actual_direction = _actual_direction_for_intended_axis(actual_displacement, intended_axis)
    direction_passed = _direction_check_passed(
        actual_displacement,
        intended_axis=intended_axis,
        intended_sign=intended_sign,
    )
    actual_distance_error = abs(actual_distance - intended_distance) if intended_distance is not None else None
    orientation_change = None
    before_orientation = before.get("orientation_xyzw")
    after_orientation = after.get("orientation_xyzw")
    if _quaternion(before_orientation) and _quaternion(after_orientation):
        orientation_change = _quaternion_angle(before_orientation, after_orientation)
    orientation_passed = (
        orientation_change is None
        or orientation_change <= float(orientation_tolerance_rad) + EPS
    )

    result.update(
        {
            "actual_displacement_m": [round(value, 6) for value in actual_displacement],
            "actual_displacement_distance_m": round(actual_distance, 6),
            "actual_distance_error_m": round(actual_distance_error, 6) if actual_distance_error is not None else None,
            "actual_direction": actual_direction,
            "direction_check_passed": direction_passed,
            "orientation_change_rad": orientation_change,
            "orientation_check_passed": orientation_passed,
            "post_motion_verification_status": (
                STATUS_PASS
                if direction_passed
                and actual_distance_error is not None
                and actual_distance_error <= float(distance_tolerance_m) + EPS
                and orientation_passed
                else STATUS_FAILED
            ),
            "reason": None,
        }
    )
    if result["post_motion_verification_status"] != STATUS_PASS:
        result["reason"] = "post_motion_direction_distance_or_orientation_check_failed"
    return result


def _tcp_pose_for_evidence(pose: dict[str, Any] | None) -> dict[str, Any] | None:
    if not isinstance(pose, dict) or pose.get("available") is False:
        return None
    if _vector3(pose.get("position_m")) and _quaternion(pose.get("orientation_xyzw")):
        return {
            "available": True,
            "frame": pose.get("frame") or "base_link",
            "position_m": [round(float(value), 12) for value in pose["position_m"]],
            "orientation_xyzw": [round(float(value), 15) for value in pose["orientation_xyzw"]],
            "source": pose.get("source"),
            "allowed_for_real_execution": pose.get("allowed_for_real_execution"),
            "tcp_pose_base_frame": pose.get("tcp_pose_base_frame"),
            "tcp_pose_tool_frame": pose.get("tcp_pose_tool_frame"),
            "tcp_pose_source": pose.get("tcp_pose_source"),
            "tcp_pose_readiness_status": pose.get("tcp_pose_readiness_status"),
            "tcp_pose_stamp": _optional_number(pose.get("tcp_pose_stamp")),
            "tcp_pose_age_s": _optional_number(pose.get("tcp_pose_age_s")),
            "tcp_pose_stale_after_s": _optional_number(pose.get("tcp_pose_stale_after_s")),
            "tcp_pose_sample_settle_s": _optional_number(pose.get("tcp_pose_sample_settle_s")),
            "tcp_pose_sample_attempts": int(pose["tcp_pose_sample_attempts"])
            if isinstance(pose.get("tcp_pose_sample_attempts"), int)
            and not isinstance(pose.get("tcp_pose_sample_attempts"), bool)
            else None,
        }
    return _current_tcp_pose_evidence(
        pose,
        source=str(pose.get("source") or "target_tcp_pose"),
        allowed_for_real_execution=pose.get("allowed_for_real_execution") is not False,
    )


def _tcp_pose_stale_check(before: dict[str, Any], after: dict[str, Any]) -> bool | None:
    ages = [
        _optional_number(before.get("tcp_pose_age_s")),
        _optional_number(after.get("tcp_pose_age_s")),
    ]
    available = [age for age in ages if age is not None]
    if not available:
        return None
    return all(age <= TCP_POSE_STALE_AFTER_S + EPS for age in available)


def _post_motion_top_level_fields(post_motion: dict[str, Any]) -> dict[str, Any]:
    return {
        "post_motion_verification": post_motion,
        "post_motion_verification_status": post_motion.get("post_motion_verification_status"),
        "actual_displacement_m": post_motion.get("actual_displacement_m"),
        "actual_displacement_distance_m": post_motion.get("actual_displacement_distance_m"),
        "actual_distance_error_m": post_motion.get("actual_distance_error_m"),
        "orientation_change_rad": post_motion.get("orientation_change_rad"),
        "direction_check_passed": post_motion.get("direction_check_passed"),
        "orientation_check_passed": post_motion.get("orientation_check_passed"),
        "post_motion_distance_tolerance_m": post_motion.get("post_motion_distance_tolerance_m"),
        "post_motion_orientation_tolerance_rad": post_motion.get("post_motion_orientation_tolerance_rad"),
        "tcp_pose_before_stamp": post_motion.get("tcp_pose_before_stamp"),
        "tcp_pose_after_stamp": post_motion.get("tcp_pose_after_stamp"),
        "tcp_pose_before_age_s": post_motion.get("tcp_pose_before_age_s"),
        "tcp_pose_after_age_s": post_motion.get("tcp_pose_after_age_s"),
        "tcp_pose_sample_settle_s": post_motion.get("tcp_pose_sample_settle_s"),
        "tcp_pose_sample_attempts": post_motion.get("tcp_pose_sample_attempts"),
        "tcp_pose_stale_check_passed": post_motion.get("tcp_pose_stale_check_passed"),
    }


def _post_motion_not_run_fields() -> dict[str, Any]:
    return _post_motion_top_level_fields(
        _post_motion_verification(
            real_robot_motion_executed=False,
            before_tcp_pose=None,
            target_tcp_pose=None,
            after_tcp_pose=None,
            intended_delta_m=None,
            reason="real_robot_motion_executed=false",
        )
    )


def _intended_delta_from_contract(parsed_contract: dict[str, Any] | None) -> list[float] | None:
    if not isinstance(parsed_contract, dict):
        return None
    delta = parsed_contract.get("delta_m")
    if _vector3(delta):
        return list(delta)
    preview = parsed_contract.get("execution_preview")
    if isinstance(preview, dict) and _vector3(preview.get("delta_m")):
        return list(preview["delta_m"])
    return None


def _direction_label_from_delta(delta_m: list[float] | None) -> str | None:
    if not _vector3(delta_m):
        return None
    axis, sign = _axis_direction_from_delta([float(value) for value in delta_m])
    if axis is None or sign is None:
        return None
    return f"{axis}{sign}"


def _actual_direction_for_intended_axis(delta_m: list[float], intended_axis: str | None) -> str | None:
    if not _vector3(delta_m) or intended_axis not in {"x", "y", "z"}:
        return _direction_label_from_delta(delta_m)
    index = {"x": 0, "y": 1, "z": 2}[intended_axis]
    value = float(delta_m[index])
    if value > 0.0:
        return f"{intended_axis}+"
    if value < 0.0:
        return f"{intended_axis}-"
    return None


def _direction_check_passed(
    delta_m: list[float],
    *,
    intended_axis: str | None,
    intended_sign: str | None,
) -> bool:
    if not _vector3(delta_m) or intended_axis not in {"x", "y", "z"} or intended_sign not in {"+", "-"}:
        return False
    index = {"x": 0, "y": 1, "z": 2}[intended_axis]
    value = float(delta_m[index])
    return value > 0.0 if intended_sign == "+" else value < 0.0


def _acceptance_workflow(
    *,
    status: str,
    mode: str | None,
    parser_metadata: dict[str, Any],
    parsed_contract: dict[str, Any] | None,
    planner_acceptance: dict[str, Any] | None,
    trajectory_sent: bool,
    execute_trajectory_called: bool,
    controller_command_sent: bool,
    real_robot_motion_executed: bool,
    blocking_reasons: list[str] | None,
    manual_confirmation_received: bool = False,
    real_execution_allowed: bool = False,
) -> dict[str, Any] | None:
    if mode is None:
        return None
    parsed_contract = parsed_contract if isinstance(parsed_contract, dict) else {}
    execution_preview = parsed_contract.get("execution_preview") if isinstance(parsed_contract.get("execution_preview"), dict) else {}
    planner_acceptance = planner_acceptance if isinstance(planner_acceptance, dict) else {}
    return {
        "status": status,
        "mode": mode,
        "original_command": parser_metadata.get("original_user_text"),
        "qwen_parser_used": parser_metadata.get("parser_source") == "qwen_llm",
        "execution_preview_status": execution_preview.get("preview_status"),
        "planner_acceptance_status": planner_acceptance.get("status"),
        "manual_confirmation_required": True,
        "manual_confirmation_received": manual_confirmation_received,
        "real_execution_allowed": real_execution_allowed,
        "trajectory_sent": trajectory_sent,
        "execute_trajectory_called": execute_trajectory_called,
        "controller_command_sent": controller_command_sent,
        "real_robot_motion_executed": real_robot_motion_executed,
        "blocking_reasons": list(blocking_reasons or []),
    }


def _motion_check_fields(motion: dict[str, Any], moveit: dict[str, Any], parsed_contract: dict[str, Any]) -> dict[str, Any]:
    current = moveit.get("motion_check_current_position_m")
    target = moveit.get("motion_check_target_position_m")
    distance = moveit.get("motion_check_distance_m")
    if current is None and isinstance(motion.get("current_tcp_pose"), dict):
        current = motion["current_tcp_pose"].get("position_m")
    if target is None and isinstance(motion.get("target_pose"), dict):
        target = motion["target_pose"].get("position_m")
    if distance is None and _vector3(current) and _vector3(target):
        distance = round(math.sqrt(sum((float(right) - float(left)) ** 2 for left, right in zip(current, target))), 6)
    return {
        "motion_check_source": moveit.get("motion_check_source") or "moveit_pose_executor",
        "motion_check_current_position_m": list(current) if _vector3(current) else current,
        "motion_check_target_position_m": list(target) if _vector3(target) else target,
        "motion_check_distance_m": distance,
        "motion_check_max_distance_m": moveit.get("motion_check_max_distance_m", parsed_contract.get("max_distance_m")),
        "motion_check_hard_limit_m": moveit.get("motion_check_hard_limit_m", parsed_contract.get("hard_safety_limit_m")),
        "motion_check_eps": moveit.get("motion_check_eps", EPS),
    }


def _axis_direction_from_delta(delta_m: list[float]) -> tuple[str | None, str | None]:
    axes = ["x", "y", "z"]
    active = [(axis, float(value)) for axis, value in zip(axes, delta_m) if abs(float(value)) > EPS]
    if len(active) != 1:
        return None, None
    axis, value = active[0]
    return axis, "+" if value > 0.0 else "-"


def _empty_motion_check(parsed_contract: dict[str, Any] | None) -> dict[str, Any]:
    parsed_contract = parsed_contract if isinstance(parsed_contract, dict) else {}
    return {
        "motion_check_source": "moveit_pose_executor",
        "motion_check_current_position_m": None,
        "motion_check_target_position_m": None,
        "motion_check_distance_m": None,
        "motion_check_max_distance_m": parsed_contract.get("max_distance_m"),
        "motion_check_hard_limit_m": parsed_contract.get("hard_safety_limit_m"),
        "motion_check_eps": EPS,
    }


def _vector3(value: Any) -> bool:
    return (
        isinstance(value, list)
        and len(value) == 3
        and all(isinstance(item, (int, float)) and not isinstance(item, bool) and math.isfinite(float(item)) for item in value)
    )


def _quaternion(value: Any) -> bool:
    return (
        isinstance(value, list)
        and len(value) == 4
        and all(isinstance(item, (int, float)) and not isinstance(item, bool) and math.isfinite(float(item)) for item in value)
    )


def _env_float(name: str) -> float | None:
    value = os.environ.get(name)
    if not value:
        return None
    try:
        number = float(value)
    except ValueError:
        return None
    return number if math.isfinite(number) else None


def _optional_number(value: Any) -> float | None:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    number = float(value)
    return number if math.isfinite(number) else None


def _first_not_none(*values: Any) -> Any:
    for value in values:
        if value is not None:
            return value
    return None


def _string_list(value: Any) -> list[str]:
    if isinstance(value, list):
        return [str(item) for item in value if item]
    if isinstance(value, str) and value:
        return [value]
    return []


def _unique(values: list[str]) -> list[str]:
    result = []
    for value in values:
        if value and value not in result:
            result.append(value)
    return result


def _json(data: dict[str, Any]) -> str:
    return json.dumps(data, indent=2, sort_keys=True)


if __name__ == "__main__":
    raise SystemExit(main())
