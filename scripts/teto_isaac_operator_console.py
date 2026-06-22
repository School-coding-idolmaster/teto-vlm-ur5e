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
    parser.add_argument("--motion-duration-sec", type=float)
    parser.add_argument("--substep-pause-sec", type=float)
    parser.add_argument("--no-visual-demo-slowdown", action="store_true")
    parser.add_argument("--no-visual-markers", action="store_true")
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
    print(f"[TETO Isaac] operator main entered argv={raw_argv!r}", flush=True)
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
    if args.motion_duration_sec is not None:
        config.raw["motion_duration_sec"] = max(float(args.motion_duration_sec), 0.0)
    if args.substep_pause_sec is not None:
        config.raw["substep_pause_sec"] = max(float(args.substep_pause_sec), 0.0)
    if args.no_visual_demo_slowdown:
        config.raw["visual_demo_slowdown_enabled"] = False
    if args.no_visual_markers:
        config.raw["visual_markers_enabled"] = False
    if args.ur5e_asset:
        config.raw["ur5e_asset_path"] = args.ur5e_asset
        config.raw["asset_mode"] = "usd_reference"
        print(
            f"[TETO Isaac] --ur5e-asset selects usd_reference: {args.ur5e_asset}",
            flush=True,
        )
    else:
        clean_asset = REPO_ROOT / "outputs/isaac_assets/generated_ur5e/ur5e_clean_no_tool.usd"
        if clean_asset.is_file():
            config.raw["ur5e_asset_path"] = str(clean_asset)
            config.raw["asset_mode"] = "usd_reference"
            print(f"[TETO Isaac] defaulting to clean no-tool asset: {clean_asset}", flush=True)
    if args.synthetic_fake_gateway and not args.headless:
        print("SAFETY BLOCK: E_SYNTHETIC_FAKE_GATEWAY_REQUIRES_HEADLESS", file=sys.stderr)
        return 2

    simulation_app = None
    if args.synthetic_fake_gateway:
        print("[TETO Isaac] using synthetic fake gateway for headless smoke", flush=True)
        gateway = SyntheticFakeGateway()
    else:
        try:
            print(
                f"[TETO Isaac] creating SimulationApp headless={args.headless}",
                flush=True,
            )
            from isaacsim import SimulationApp

            simulation_app = SimulationApp({"headless": args.headless})
            print("[TETO Isaac] SimulationApp created; initializing measured bridge", flush=True)
            from src.isaac_sim_bridge import IsaacSimMeasuredBridge

            gateway = IsaacSimMeasuredBridge(
                simulation_app=simulation_app,
                config=config.raw,
                headless=args.headless,
            )
        except Exception as exc:
            print(
                f"Isaac startup failed before console: {type(exc).__name__}: {exc}",
                file=sys.stderr,
                flush=True,
            )
            if simulation_app is not None:
                print("[TETO Isaac] closing SimulationApp after startup failure", flush=True)
                simulation_app.close()
            return 3

    operator = IsaacSimOperator(
        config=config,
        gateway=gateway,
        headless=args.headless,
        output_dir=args.output_dir,
    )
    _banner(operator.status(), args.headless)
    try:
        return _run_operator_session(args, operator, gateway, simulation_app)
    finally:
        if simulation_app is not None:
            print("[TETO Isaac] session ended; closing SimulationApp", flush=True)
            simulation_app.close()


def _run_operator_session(args, operator: IsaacSimOperator, gateway, simulation_app) -> int:
    if args.cmd:
        print("[TETO Isaac] entering one-shot command branch", flush=True)
        result = operator.execute_text(args.cmd)
        print(json.dumps(_summary(result), ensure_ascii=False, indent=2), flush=True)
        return 0 if result.get("status") == "PASS" else 4
    print("[TETO Isaac] entering persistent REPL; type quit or Ctrl-D to close", flush=True)
    return _interactive_loop(operator, gateway, simulation_app)


def _interactive_loop(operator: IsaacSimOperator, gateway, simulation_app) -> int:
    commands: queue.Queue[str] = queue.Queue()
    stopped = threading.Event()

    def reader() -> None:
        while not stopped.is_set():
            try:
                command = input("TETO/Isaac> ")
            except EOFError:
                print("[TETO Isaac] Ctrl-D received; requesting clean shutdown", flush=True)
                command = "quit"
            commands.put(command)
            if command.strip().lower() in {"quit", "exit"}:
                return

    reader_thread = threading.Thread(target=reader, name="teto-isaac-console-input", daemon=True)
    reader_thread.start()
    while not stopped.is_set():
        if simulation_app is not None and not gateway.render_once():
            print("[TETO Isaac] SimulationApp is no longer running; leaving REPL", flush=True)
            break
        try:
            command = commands.get(timeout=0.01)
        except queue.Empty:
            time.sleep(0.01)
            continue
        normalized = command.strip().lower()
        if normalized in {"quit", "exit"}:
            print("[TETO Isaac] quit received; closing console session", flush=True)
            stopped.set()
            break
        if normalized == "status":
            print(json.dumps(_status_summary(operator.status()), ensure_ascii=False, indent=2), flush=True)
        elif normalized in {"home", "reset home", "demo_center", "demo center"}:
            print(json.dumps(operator.demo_center(), ensure_ascii=False, indent=2), flush=True)
        elif normalized == "reset":
            print(json.dumps(operator.reset(), ensure_ascii=False, indent=2), flush=True)
        elif normalized:
            result = operator.execute_text(command)
            for step in result.get("substeps", []):
                print(
                    f"substep {step['substep_index']}/{step['substep_count']}: "
                    f"{step['verification_result']} measured={step['simulated_measured_tcp_after']['position_m'] if step.get('simulated_measured_tcp_after') else None}"
                )
            print(json.dumps(_summary(result), ensure_ascii=False, indent=2), flush=True)
    return 0


def _banner(status: dict, headless: bool) -> None:
    print("TETO Isaac Sim GUI Operator", flush=True)
    print(f"Mode: {'HEADLESS_SMOKE_TEST' if headless else 'ISAAC_SIM_ONLY'}", flush=True)
    print("Real robot: DISABLED", flush=True)
    print(f"Qwen: {'OK' if status['qwen_health'].get('ok') else 'YELLOW'}", flush=True)
    print(f"Isaac GUI: {'NOT REQUIRED (CI ONLY)' if headless else 'REQUIRED'}", flush=True)
    print(f"Isaac connection: {status['isaac_connection_status']}", flush=True)
    print("Window checklist: viewport visible; UR5e visible; timeline/rendering active.", flush=True)


def _summary(result: dict) -> dict:
    return {
        "status": result.get("status"),
        "abort_reason": result.get("abort_reason"),
        "substeps": f"{result.get('completed_substep_count')}/{result.get('substep_count')}",
        "subgoal_count": result.get("subgoal_count"),
        "requested_delta_m": result.get("delta_vector_m"),
        "motion_contract_type": result.get("motion_contract_type"),
        "requested_total_distance_m": result.get("requested_total_distance_m"),
        "max_substep_m": result.get("max_substep_m"),
        "measured_delta_m": result.get("measured_delta_vector_m"),
        "final_displacement_m": result.get("final_displacement_m"),
        "target_final_tcp": result.get("target_final_tcp_pose"),
        "measured_final_tcp": result.get("final_simulated_tcp_pose"),
        "position_error_m": result.get("final_position_error_m"),
        "workspace_check_status": result.get("workspace_check_status"),
        "direction_check_passed": result.get("direction_check_passed"),
        "joint_delta_summary": result.get("joint_delta_summary"),
        "visible_motion_hint": result.get("visible_motion_hint"),
        "gateway_type": result.get("gateway_type"),
        "evidence": result.get("artifact_paths"),
        "real_robot_motion_executed": False,
    }


def _status_summary(status: dict) -> dict:
    return {
        "execution_mode": status.get("execution_mode"),
        "runtime_mode": status.get("runtime_mode"),
        "isaac_connection_status": status.get("isaac_connection_status"),
        "qwen_ok": (status.get("qwen_health") or {}).get("ok"),
        "current_simulated_tcp_pose": status.get("current_simulated_tcp_pose"),
        "current_simulated_joint_state": status.get("current_simulated_joint_state"),
        "isaac_initial_home_pose_applied": status.get("isaac_initial_home_pose_applied"),
        "isaac_initial_home_pose_source": status.get("isaac_initial_home_pose_source"),
        "isaac_visual_markers_enabled": status.get("isaac_visual_markers_enabled"),
        "isaac_trajectory_trace_enabled": status.get("isaac_trajectory_trace_enabled"),
        "isaac_visual_timing": status.get("isaac_visual_timing"),
        "last_command_summary": _summary(status["last_command_result"]) if status.get("last_command_result") else None,
        "real_robot_motion_executed": False,
        "dashboard_used": False,
        "rtde_write_used": False,
        "moveit_execute_trajectory_called": False,
    }


if __name__ == "__main__":
    raise SystemExit(main())
