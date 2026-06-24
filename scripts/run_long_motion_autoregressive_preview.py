#!/usr/bin/env python3
from __future__ import annotations

# HISTORICAL/DEBUG long-motion preview entrypoint.
# This is not the current default real path. Current real default:
# scripts/start_teto_real_full_stack.sh / scripts/teto_operator_console.py.
# Current Isaac default: scripts/start_teto_isaac_gui_operator.sh.
# Dry-run, plan-only, fake, or Isaac preview evidence is not REAL_PATH success
# evidence.

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
from src.motion_command_normalizer import normalize_motion_command  # noqa: E402
from src.qwen_motion_parser import (  # noqa: E402
    QwenMotionParserRequest,
    evaluate_qwen_motion_parser,
)


DEFAULT_MOCK_POSE = {
    "frame": "base_link",
    "position_m": [0.40, 0.0, 0.30],
    "orientation_xyzw": [0.0, 0.0, 0.0, 1.0],
}
DEFAULT_COMMANDS = [
    "move forward 10 cm",
    "move forward 20 cm",
    "lower the tcp by 10 centimeters",
    "raise the tool by 20 centimeters",
    "shift the end effector left by 15 cm",
    "drop the tool a tiny bit",
    "go up 5 cm and right 2 cm",
]


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Generate TETO v3.0.13 offline autoregressive long-motion preview evidence."
    )
    parser.add_argument("--cmd", action="append", help="Natural-language command. Repeat for multiple previews.")
    parser.add_argument("--examples", action="store_true", help="Generate the built-in safe offline examples.")
    parser.add_argument("--parser", choices=["rule", "qwen", "auto"], default="auto")
    parser.add_argument("--qwen-model", default=os.environ.get("TETO_QWEN_MODEL"))
    parser.add_argument("--qwen-endpoint", default=os.environ.get("TETO_QWEN_ENDPOINT"))
    parser.add_argument("--qwen-timeout-s", type=float, default=2.0)
    parser.add_argument("--mock-current-tcp-pose", action="store_true")
    parser.add_argument("--current-tcp-pose-json")
    parser.add_argument("--max-one-shot-distance-m", type=float, default=0.05)
    parser.add_argument("--max-decomposed-substep-distance-m", type=float, default=0.02)
    parser.add_argument("--max-decomposed-total-distance-m", type=float, default=0.20)
    parser.add_argument("--session-radius-limit-m", type=float)
    parser.add_argument("--output-dir", default="outputs/autoregressive_motion_previews")
    args = parser.parse_args(argv)

    commands = DEFAULT_COMMANDS if args.examples else args.cmd or []
    if not commands:
        parser.error("provide --cmd or --examples")
    current_pose, pose_error = _current_pose(args)
    exit_code = 0
    for command in commands:
        evidence = _build_evidence(command, args=args, current_pose=current_pose, pose_error=pose_error)
        json_path, markdown_path = _write_report(evidence, Path(args.output_dir))
        summary = {
            "command": command,
            "parse_status": evidence["canonical_motion_intent"].get("parse_status"),
            "requested_distance_m": evidence["autoregressive_plan"].get("requested_distance_m"),
            "substep_count": evidence["autoregressive_plan"].get("substep_count"),
            "final_plan_status": evidence["autoregressive_plan"].get("final_plan_status"),
            "any_execution_attempted": False,
            "report_path": str(json_path),
            "markdown_report_path": str(markdown_path),
        }
        print(json.dumps(summary, ensure_ascii=False, sort_keys=True))
        if evidence["autoregressive_plan"].get("final_plan_status") not in {"PASS", "INVALID_REQUEST"}:
            exit_code = 2
    return exit_code


def _build_evidence(
    command: str,
    *,
    args: argparse.Namespace,
    current_pose: dict[str, Any] | None,
    pose_error: str | None,
) -> dict[str, Any]:
    canonical, parser_evidence = _canonical_motion_intent(command, args)
    if pose_error:
        current_pose = None
    plan = plan_offline_autoregressive_motion(
        AutoregressiveMotionPlannerRequest(
            canonical_motion_intent=canonical,
            current_tcp_pose=current_pose,
            config={
                "enable_long_step_decomposition": True,
                "max_one_shot_distance_m": args.max_one_shot_distance_m,
                "max_decomposed_substep_distance_m": args.max_decomposed_substep_distance_m,
                "max_decomposed_total_distance_m": args.max_decomposed_total_distance_m,
                "substep_execution_mode": "offline_preview",
                "session_radius_limit_m": args.session_radius_limit_m,
                "workspace_bounds": {"x": [-1.0, 1.0], "y": [-1.0, 1.0], "z": [0.0, 2.0]},
            },
        )
    )
    return {
        "report_version": "teto_v3_0_13_autoregressive_preview_report_v1",
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "command": command,
        "parser_mode": args.parser,
        "parser_evidence": parser_evidence,
        "canonical_motion_intent": canonical,
        "current_tcp_pose": current_pose,
        "current_tcp_pose_error": pose_error,
        "autoregressive_plan": plan,
        "safety_confirmation": {
            "robot_launch": False,
            "ur5_connection": False,
            "moveit_launch": False,
            "execute_trajectory_called": False,
            "trajectory_sent": False,
            "real_substep_execution": False,
            "operator_console_used": False,
            "manual_confirmation_required": False,
        },
    }


def _canonical_motion_intent(
    command: str,
    args: argparse.Namespace,
) -> tuple[dict[str, Any], dict[str, Any]]:
    if args.parser in {"qwen", "auto"}:
        qwen = evaluate_qwen_motion_parser(
            QwenMotionParserRequest(
                user_text=command,
                max_distance_m=args.max_decomposed_total_distance_m,
                hard_safety_limit_m=args.max_decomposed_total_distance_m,
                model_name=args.qwen_model,
                endpoint=args.qwen_endpoint,
                timeout_s=args.qwen_timeout_s,
            )
        )
        if qwen.get("qwen_motion_parser_status") == "PASS":
            return _canonical_from_parser_evidence(qwen), qwen
        if args.parser == "qwen":
            return _blocked_canonical(qwen), qwen
        fallback = normalize_motion_command(command, parser_source="fallback_rule")
        fallback["qwen_fallback_evidence"] = qwen
        return fallback, {"qwen": qwen, "fallback": fallback}
    fallback = normalize_motion_command(command, parser_source="normalizer")
    return fallback, fallback


def _canonical_from_parser_evidence(evidence: dict[str, Any]) -> dict[str, Any]:
    return {
        "parse_status": evidence.get("parse_status") or "PASS",
        "intent": "relative_cartesian_motion",
        "motion_frame": evidence.get("motion_frame") or "base_link",
        "direction_axis": evidence.get("direction_axis") or evidence.get("axis"),
        "direction_sign": evidence.get("direction_sign") or evidence.get("direction"),
        "requested_distance_m": evidence.get("requested_distance_m") or evidence.get("distance_m"),
        "requested_distance_norm_m": evidence.get("requested_distance_norm_m") or evidence.get("distance_m"),
        "delta_m": evidence.get("delta_m"),
        "vector_delta_m": evidence.get("vector_delta_m"),
        "vector_components_m": evidence.get("vector_components_m"),
        "vector_component_count_nonzero": evidence.get("vector_component_count_nonzero"),
        "motion_contract_type": evidence.get("motion_contract_type"),
        "legacy_axis_compatible": evidence.get("legacy_axis_compatible"),
        "vector_source": evidence.get("vector_source"),
        "vector_motion_supported": evidence.get("vector_motion_supported", True),
        "parser_source": evidence.get("parser_source"),
        "execution_permission_decided_by_parser": False,
        "safety_gate_still_required": True,
    }


def _blocked_canonical(evidence: dict[str, Any]) -> dict[str, Any]:
    return {
        "parse_status": evidence.get("parse_status") or "BLOCKED",
        "intent": None,
        "motion_frame": "base_link",
        "direction_axis": evidence.get("direction_axis") or evidence.get("axis"),
        "direction_sign": evidence.get("direction_sign") or evidence.get("direction"),
        "requested_distance_m": evidence.get("requested_distance_m") or evidence.get("distance_m"),
        "delta_m": evidence.get("delta_m"),
        "execution_permission_decided_by_parser": False,
        "safety_gate_still_required": True,
    }


def _current_pose(args: argparse.Namespace) -> tuple[dict[str, Any] | None, str | None]:
    if args.current_tcp_pose_json:
        try:
            value = json.loads(args.current_tcp_pose_json)
        except json.JSONDecodeError:
            return None, "E_INVALID_CURRENT_TCP_POSE_JSON"
        return value if isinstance(value, dict) else None, None if isinstance(value, dict) else "E_INVALID_CURRENT_TCP_POSE"
    if args.mock_current_tcp_pose or not args.current_tcp_pose_json:
        return dict(DEFAULT_MOCK_POSE), None
    return None, "E_CURRENT_TCP_POSE_MISSING"


def _write_report(evidence: dict[str, Any], output_dir: Path) -> tuple[Path, Path]:
    output_dir.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S%fZ")
    slug = _slug(str(evidence["command"]))
    json_path = output_dir / f"{stamp}_{slug}.json"
    markdown_path = output_dir / f"{stamp}_{slug}.md"
    json_path.write_text(json.dumps(evidence, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    plan = evidence["autoregressive_plan"]
    markdown_path.write_text(
        "\n".join(
            [
                "# TETO v3.0.13 Offline Autoregressive Motion Preview",
                "",
                f"- Command: `{evidence['command']}`",
                f"- Parse status: `{evidence['canonical_motion_intent'].get('parse_status')}`",
                f"- Requested distance: `{plan.get('requested_distance_m')}` m",
                f"- Substeps: `{plan.get('substep_count')}`",
                f"- Final plan status: `{plan.get('final_plan_status')}`",
                f"- Blocking reason: `{plan.get('final_blocking_reason')}`",
                f"- Abort reason: `{plan.get('final_abort_reason')}`",
                "- Execution attempted: `false`",
                "",
                "```json",
                json.dumps(plan, indent=2, ensure_ascii=False),
                "```",
                "",
            ]
        ),
        encoding="utf-8",
    )
    return json_path, markdown_path


def _slug(command: str) -> str:
    slug = "".join(character.lower() if character.isalnum() else "_" for character in command)
    return "_".join(part for part in slug.split("_") if part)[:60] or "preview"


if __name__ == "__main__":
    raise SystemExit(main())
