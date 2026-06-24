#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import os
import sys
from pathlib import Path
from typing import Any


REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from src.bounded_relative_motion import SHARED_MAX_RELATIVE_MOTION_DISTANCE_M
from src.real_segmented_operator_backend import (
    RealOperatorStateProvider,
    RealSegmentedBackendConfig,
    RealSegmentedOperatorBackend,
)
from src.unified_segmented_operator import (
    UnifiedOperatorConfig,
    UnifiedSegmentedOperator,
)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Unified TETO segmented natural-language operator console."
    )
    parser.add_argument("--backend", choices=["real"], default="real")
    parser.add_argument("--cmd", help="Run one command and exit.")
    parser.add_argument(
        "--qwen-endpoint",
        default=os.environ.get(
            "TETO_QWEN_ENDPOINT",
            "http://127.0.0.1:18080/api/generate",
        ),
    )
    parser.add_argument(
        "--max-total-distance-m",
        type=_bounded_total,
        default=SHARED_MAX_RELATIVE_MOTION_DISTANCE_M,
    )
    parser.add_argument(
        "--max-substep-distance-m",
        type=_bounded_substep,
        default=0.05,
    )
    parser.add_argument("--position-tolerance-m", type=float, default=0.005)
    parser.add_argument("--orientation-tolerance-rad", type=float, default=0.01)
    parser.add_argument("--tcp-pose-stale-after-s", type=float, default=1.0)
    parser.add_argument("--snapshot-max-age-s", type=float, default=1.0)
    parser.add_argument("--snapshot-sync-tolerance-s", type=float, default=0.25)
    parser.add_argument(
        "--robot-ip",
        default=os.environ.get("TETO_ROBOT_IP", "192.168.20.35"),
    )
    parser.add_argument(
        "--dashboard-port",
        type=int,
        default=int(os.environ.get("TETO_DASHBOARD_PORT", "29999")),
    )
    parser.add_argument(
        "--tcp-pose-topic",
        default=os.environ.get("TETO_TCP_POSE_TOPIC", "/tcp_pose"),
    )
    parser.add_argument(
        "--joint-states-topic",
        default=os.environ.get("TETO_JOINT_STATES_TOPIC", "/joint_states"),
    )
    parser.add_argument(
        "--d455-color-topic",
        default=os.environ.get("TETO_D455_COLOR_TOPIC"),
        help="Optional explicit RGB topic. Omit to auto-discover.",
    )
    parser.add_argument(
        "--d455-depth-topic",
        default=os.environ.get("TETO_D455_DEPTH_TOPIC"),
        help="Optional explicit aligned-depth topic. Omit to auto-discover.",
    )
    parser.add_argument(
        "--workspace-json",
        default=os.environ.get("TETO_REAL_WORKSPACE_JSON"),
        help='Workspace bounds, for example {"x":[-1,1],"y":[-1,1],"z":[0,2]}.',
    )
    parser.add_argument(
        "--output-dir",
        default="outputs/real_segmented_operator_runs",
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    operator = build_operator(args)
    if args.cmd:
        result = operator.handle_command(args.cmd.strip())
        print(
            json.dumps(_summary(result), indent=2, ensure_ascii=False),
            flush=True,
        )
        return 0 if result.get("status") == "PASS" else 2

    print("TETO unified segmented operator", flush=True)
    print(
        "Backend: real UR5e / MoveIt / measured TCP / Dashboard / D455",
        flush=True,
    )
    print(
        "Commands: status, home, reset, help, quit/exit, or relative Cartesian language.",
        flush=True,
    )
    print(
        "Manual y confirmation: disabled. Per-segment safety and vision gates: required.",
        flush=True,
    )
    while True:
        try:
            command = _read_console_command(sys.stdin, sys.stdout)
        except KeyboardInterrupt:
            print(
                "\nCommand input interrupted; no command submitted.",
                flush=True,
            )
            continue
        if command is None:
            print(flush=True)
            break
        if not command:
            continue
        try:
            result = operator.handle_command(command)
        except KeyboardInterrupt:
            print(
                "\nActive command interrupted; later segments were not started.",
                flush=True,
            )
            continue
        print(
            json.dumps(_summary(result), indent=2, ensure_ascii=False),
            flush=True,
        )
        if result.get("quit_requested") is True:
            break
    print("TETO unified operator console closed.", flush=True)
    return 0


def build_operator(args: argparse.Namespace) -> UnifiedSegmentedOperator:
    workspace = _workspace(args.workspace_json)
    health_base = args.qwen_endpoint.removesuffix("/api/generate").rstrip("/")
    backend_config = RealSegmentedBackendConfig(
        max_substep_distance_m=args.max_substep_distance_m,
        position_tolerance_m=args.position_tolerance_m,
        orientation_tolerance_rad=args.orientation_tolerance_rad,
        tcp_pose_stale_after_s=args.tcp_pose_stale_after_s,
        snapshot_max_age_s=args.snapshot_max_age_s,
        snapshot_sync_tolerance_s=args.snapshot_sync_tolerance_s,
        robot_ip=args.robot_ip,
        dashboard_port=args.dashboard_port,
        tcp_pose_topic=args.tcp_pose_topic,
        joint_states_topic=args.joint_states_topic,
        color_topic=args.d455_color_topic,
        depth_topic=args.d455_depth_topic,
        qwen_health_url=f"{health_base}/health",
        workspace_bounds=workspace,
    )
    provider = RealOperatorStateProvider(backend_config)
    return UnifiedSegmentedOperator(
        config=UnifiedOperatorConfig(
            max_total_distance_m=args.max_total_distance_m,
            max_substep_distance_m=args.max_substep_distance_m,
            position_tolerance_m=args.position_tolerance_m,
            orientation_tolerance_rad=args.orientation_tolerance_rad,
            tcp_pose_stale_after_s=args.tcp_pose_stale_after_s,
            workspace_bounds=workspace,
            qwen_endpoint=args.qwen_endpoint,
            parser_mode="qwen",
            output_dir=args.output_dir,
        ),
        backend=RealSegmentedOperatorBackend(
            backend_config,
            state_provider=provider,
        ),
    )


def _read_console_command(stdin: Any, stdout: Any) -> str | None:
    stdout.write("\nTETO/Operator> ")
    stdout.flush()
    raw = stdin.readline()
    if raw == "":
        return None
    return raw.strip()


def _summary(result: dict[str, Any]) -> dict[str, Any]:
    return {
        "status": result.get("status"),
        "operator_command": result.get("operator_command"),
        "execution_mode": result.get("execution_mode"),
        "input_text": result.get("input_text"),
        "requested_total_distance_m": result.get("requested_total_distance_m"),
        "substeps": (
            f"{result.get('completed_substep_count', 0)}/"
            f"{result.get('substep_count', 0)}"
        ),
        "abort_reason": result.get("abort_reason"),
        "manual_confirmation_required": result.get(
            "manual_confirmation_required"
        ),
        "autonomous_segmented_execution": result.get(
            "autonomous_segmented_execution"
        ),
        "safety_gate_still_required": result.get("safety_gate_still_required"),
        "vision_guard_required": result.get("vision_guard_required"),
        "real_robot_motion_executed": result.get("real_robot_motion_executed"),
        "artifact_paths": result.get("artifact_paths"),
        "backend_status": result.get("backend_status")
        or {
            "status": result.get("status"),
            "abort_reason": result.get("abort_reason"),
        },
        "commands": result.get("commands"),
        "quit_requested": result.get("quit_requested"),
    }


def _bounded_total(value: str) -> float:
    number = float(value)
    if number <= 0.0 or number > SHARED_MAX_RELATIVE_MOTION_DISTANCE_M:
        raise argparse.ArgumentTypeError(
            f"must be > 0 and <= {SHARED_MAX_RELATIVE_MOTION_DISTANCE_M}"
        )
    return number


def _bounded_substep(value: str) -> float:
    number = float(value)
    if number <= 0.0 or number > 0.05:
        raise argparse.ArgumentTypeError("must be > 0 and <= 0.05")
    return number


def _workspace(raw: str | None) -> dict[str, list[float]]:
    if not raw:
        return {"x": [-1.0, 1.0], "y": [-1.0, 1.0], "z": [0.0, 2.0]}
    try:
        value = json.loads(raw)
    except json.JSONDecodeError as exc:
        raise SystemExit(f"E_INVALID_WORKSPACE_JSON: {exc}") from exc
    if not isinstance(value, dict):
        raise SystemExit("E_INVALID_WORKSPACE_JSON: expected object")
    result = {}
    for axis in ("x", "y", "z"):
        bounds = value.get(axis)
        if (
            not isinstance(bounds, list)
            or len(bounds) != 2
            or not all(isinstance(item, (int, float)) for item in bounds)
            or float(bounds[0]) > float(bounds[1])
        ):
            raise SystemExit(f"E_INVALID_WORKSPACE_JSON: invalid {axis} bounds")
        result[axis] = [float(bounds[0]), float(bounds[1])]
    return result


if __name__ == "__main__":
    raise SystemExit(main())
