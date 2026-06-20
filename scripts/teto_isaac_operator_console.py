#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import queue
import sys
import threading
import time
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from src.isaac_sim_operator import (  # noqa: E402
    IsaacOperatorSafetyError,
    IsaacSimOperator,
    SyntheticFakeGateway,
    load_isaac_operator_config,
    validate_no_real_robot_args,
)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="TETO LLM-to-Isaac Sim GUI operator console.")
    parser.add_argument("--world-config", default="configs/isaac_sim_operator.example.yaml")
    parser.add_argument("--qwen-endpoint")
    parser.add_argument("--ur5e-asset")
    parser.add_argument("--gui", action="store_true", default=True, help="Require visible Isaac GUI (default).")
    parser.add_argument("--headless", action="store_true", help="CI smoke only; evidence is marked headless_smoke_test.")
    parser.add_argument("--console", action="store_true", help="Open the TETO/Isaac interactive prompt.")
    parser.add_argument("--cmd", help="Execute one natural-language command and exit.")
    parser.add_argument("--output-dir", default="outputs/isaac_sim_operator_runs")
    parser.add_argument("--no-real-robot", action="store_true", default=True, help="Mandatory fail-closed safety mode.")
    parser.add_argument(
        "--synthetic-fake-gateway",
        action="store_true",
        help="Unit-test/mock smoke only. Forbidden for formal GUI demo evidence.",
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    raw_argv = list(sys.argv[1:] if argv is None else argv)
    try:
        validate_no_real_robot_args(raw_argv)
        args = build_parser().parse_args(raw_argv)
        config = load_isaac_operator_config(args.world_config)
    except IsaacOperatorSafetyError as exc:
        print(f"SAFETY BLOCK: {exc}", file=sys.stderr)
        return 2
    if args.headless:
        args.gui = False
    if not args.no_real_robot:
        print("SAFETY BLOCK: E_NO_REAL_ROBOT_FLAG_REQUIRED", file=sys.stderr)
        return 2
    if args.qwen_endpoint:
        config.raw["qwen_endpoint"] = args.qwen_endpoint
    if args.ur5e_asset:
        config.raw["ur5e_asset_path"] = args.ur5e_asset
    if args.synthetic_fake_gateway and not args.headless:
        print("SAFETY BLOCK: E_SYNTHETIC_FAKE_GATEWAY_REQUIRES_HEADLESS", file=sys.stderr)
        return 2

    simulation_app = None
    if args.synthetic_fake_gateway:
        gateway = SyntheticFakeGateway()
    else:
        try:
            from isaacsim import SimulationApp

            simulation_app = SimulationApp({"headless": args.headless})
            from src.isaac_sim_bridge import IsaacSimMeasuredBridge

            gateway = IsaacSimMeasuredBridge(
                simulation_app=simulation_app,
                config=config.raw,
                headless=args.headless,
            )
        except Exception as exc:
            if simulation_app is not None:
                simulation_app.close()
            print(f"Isaac startup failed: {exc}", file=sys.stderr)
            return 3

    operator = IsaacSimOperator(
        config=config,
        gateway=gateway,
        headless=args.headless,
        output_dir=args.output_dir,
    )
    _banner(operator.status(), args.headless)
    try:
        if args.cmd:
            result = operator.execute_text(args.cmd)
            print(json.dumps(_summary(result), ensure_ascii=False, indent=2))
            return 0 if result.get("status") == "PASS" else 4
        if args.console or not args.cmd:
            return _interactive_loop(operator, gateway, simulation_app)
        return 0
    finally:
        if simulation_app is not None:
            simulation_app.close()


def _interactive_loop(operator: IsaacSimOperator, gateway, simulation_app) -> int:
    commands: queue.Queue[str] = queue.Queue()
    stopped = threading.Event()

    def reader() -> None:
        while not stopped.is_set():
            try:
                command = input("TETO/Isaac> ")
            except EOFError:
                command = "quit"
            commands.put(command)
            if command.strip().lower() in {"quit", "exit"}:
                return

    threading.Thread(target=reader, daemon=True).start()
    while not stopped.is_set():
        if simulation_app is not None and not gateway.render_once():
            break
        try:
            command = commands.get(timeout=0.01)
        except queue.Empty:
            time.sleep(0.01)
            continue
        normalized = command.strip().lower()
        if normalized in {"quit", "exit"}:
            stopped.set()
            break
        if normalized == "status":
            print(json.dumps(operator.status(), ensure_ascii=False, indent=2))
        elif normalized == "home":
            print(json.dumps(operator.home(), ensure_ascii=False, indent=2))
        elif normalized == "reset":
            print(json.dumps(operator.reset(), ensure_ascii=False, indent=2))
        elif normalized:
            result = operator.execute_text(command)
            for step in result.get("substeps", []):
                print(
                    f"substep {step['substep_index']}/{step['substep_count']}: "
                    f"{step['verification_result']} measured={step['simulated_measured_tcp_after']['position_m'] if step.get('simulated_measured_tcp_after') else None}"
                )
            print(json.dumps(_summary(result), ensure_ascii=False, indent=2))
    return 0


def _banner(status: dict, headless: bool) -> None:
    print("TETO Isaac Sim GUI Operator")
    print(f"Mode: {'HEADLESS_SMOKE_TEST' if headless else 'ISAAC_SIM_ONLY'}")
    print("Real robot: DISABLED")
    print(f"Qwen: {'OK' if status['qwen_health'].get('ok') else 'YELLOW'}")
    print(f"Isaac GUI: {'NOT REQUIRED (CI ONLY)' if headless else 'REQUIRED'}")
    print(f"Isaac connection: {status['isaac_connection_status']}")
    print("Window checklist: viewport visible; UR5e visible; timeline/rendering active.")


def _summary(result: dict) -> dict:
    return {
        "status": result.get("status"),
        "abort_reason": result.get("abort_reason"),
        "substeps": f"{result.get('completed_substep_count')}/{result.get('substep_count')}",
        "final_simulated_tcp_pose": result.get("final_simulated_tcp_pose"),
        "gateway_type": result.get("gateway_type"),
        "evidence": result.get("artifact_paths"),
        "real_robot_motion_executed": False,
    }


if __name__ == "__main__":
    raise SystemExit(main())
