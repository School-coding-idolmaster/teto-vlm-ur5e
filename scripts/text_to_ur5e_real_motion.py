#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import math
import os
import re
import sys
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
from src.qwen_motion_parser import (  # noqa: E402
    QwenMotionParserRequest,
    evaluate_qwen_motion_parser,
)


DEFAULT_MAX_STEP_M = 0.005
HARD_SAFETY_LIMIT_M = 0.01
EPS = 1e-9
CONFIRMATION_REPLY = "y"

STATUS_PASS = "PASS"
STATUS_BLOCKED = "BLOCKED"
STATUS_FAILED = "FAILED"
STATUS_WARNING = "WARNING"

SUSPICIOUS_TINY_MOTION_JOINT_DELTA_RAD = 1.0
SUSPICIOUS_TINY_MOTION_ORIENTATION_CHANGE_RAD = 0.25


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
        return {
            "original_command": self.command,
            "input_mode": input_mode,
            "parser_mode": parser_mode,
            "parser_source": self.parser_source,
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


def parse_motion_command(command: str, *, max_step_m: float = DEFAULT_MAX_STEP_M) -> ParsedMotionCommand:
    if not isinstance(command, str) or not command.strip():
        raise MotionParseError("E_EMPTY_COMMAND")
    if max_step_m <= 0.0 or max_step_m > HARD_SAFETY_LIMIT_M + EPS:
        raise MotionParseError("E_INVALID_MAX_STEP")

    normalized = _normalize(command)
    _reject_forbidden(normalized)

    distance_m, unit = _extract_distance_m(normalized)
    if distance_m > HARD_SAFETY_LIMIT_M + EPS:
        raise MotionParseError("E_EXCEEDS_HARD_SAFETY_LIMIT")
    if distance_m > max_step_m + EPS:
        raise MotionParseError("E_EXCEEDS_MAX_STEP")

    direction, delta = _extract_direction_delta(normalized, distance_m)
    if not any(abs(value) > 0.0 for value in delta):
        raise MotionParseError("E_INVALID_ZERO_MOTION")

    return ParsedMotionCommand(
        command=command.strip(),
        frame="base_link",
        delta_m=[round(value, 6) for value in delta],
        distance_m=round(distance_m, 6),
        max_distance_m=max_step_m,
        hard_safety_limit_m=HARD_SAFETY_LIMIT_M,
        direction=direction,
        unit=unit,
    )


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Interactive text-to-UR5e relative Cartesian motion entrypoint.")
    parser.add_argument("--real", action="store_true", help="Enable real MoveIt ExecuteTrajectory.")
    parser.add_argument("--dry-run", action="store_true", help="Parse and validate without executing.")
    parser.add_argument("--cmd", help='One-shot command, for example: --cmd "move up 5 mm".')
    parser.add_argument("--yes", action="store_true", help="Skip real-mode confirmation; only allowed together with --real --cmd.")
    parser.add_argument("--parser", choices=["rule", "qwen", "auto"], default="qwen", help="Text parser mode. Default: qwen.")
    parser.add_argument("--qwen-model", default=os.environ.get("TETO_QWEN_MODEL"), help="Qwen/Ollama model name.")
    parser.add_argument("--qwen-endpoint", default=os.environ.get("TETO_QWEN_ENDPOINT"), help="Optional Ollama-compatible endpoint.")
    parser.add_argument("--qwen-timeout-s", type=float, default=_env_float("TETO_QWEN_TIMEOUT_S"))
    parser.add_argument("--max-step-m", type=float, default=DEFAULT_MAX_STEP_M)
    parser.add_argument("--speed-scale", type=float, default=0.10)
    parser.add_argument("--acc-scale", type=float, default=0.10)
    args = parser.parse_args(argv)

    if args.real and args.dry_run:
        print(_json({"final_status": STATUS_BLOCKED, "blocking_reasons": ["E_REAL_AND_DRY_RUN_CONFLICT"]}))
        return 2
    if args.real and args.yes and args.cmd is None:
        print(_json({"final_status": STATUS_BLOCKED, "blocking_reasons": ["E_YES_REQUIRES_CMD"], "real_robot_motion_executed": False}))
        return 2
    dry_run = not args.real or args.dry_run

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
            print(_json({**metadata, "final_status": STATUS_BLOCKED, "blocking_reasons": ["E_NO_INPUT"], "real_robot_motion_executed": False}))
            return 2

    parsed, parser_result = _parse_command_for_mode(
        command,
        parser_mode=args.parser,
        real=bool(args.real),
        dry_run=dry_run,
        max_step_m=float(args.max_step_m),
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
        result = {
            **parser_metadata,
            "parsed_contract": None,
            "planner_acceptance": None,
            **_empty_motion_check(None),
            "blocking_reasons": parser_metadata["parser_blocking_reasons"],
            "goal_accepted": False,
            "execute_accepted": False,
            "trajectory_sent": False,
            "controller_command_sent": False,
            "real_robot_motion_executed": False,
            "final_status": STATUS_BLOCKED,
        }
        print("Final execution evidence:")
        print(_json(result))
        return 2

    execution_preview = parsed.execution_preview(
        input_mode=input_mode,
        parser_mode=args.parser,
        real_robot_motion_requested=bool(args.real),
        dry_run=dry_run,
    )
    printed_contract = parsed.task_contract(real_robot_motion_requested=bool(args.real), dry_run=dry_run)
    printed_contract["execution_preview"] = execution_preview
    parser_metadata["normalized_contract"] = printed_contract
    print("Execution preview:")
    print(_json(execution_preview))
    print("Normalized TETO task contract:")
    print(_json(printed_contract))
    print("Parsed TETO task contract:")
    print(_json(printed_contract))

    current_pose = _lookup_current_tcp_pose(timeout_s=3.0)
    if current_pose is None:
        result = _blocked_result("E_CURRENT_TCP_POSE_MISSING", printed_contract, parser_metadata)
        print("Final execution evidence:")
        print(_json(result))
        return 2

    config = _build_gateway_config(
        current_pose=current_pose,
        max_step_m=float(args.max_step_m),
        speed_scale=float(args.speed_scale),
        acc_scale=float(args.acc_scale),
        real=bool(args.real),
        dry_run=dry_run,
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
        result = _evidence(
            status=STATUS_BLOCKED,
            motion=motion,
            execution={},
            parsed_contract=printed_contract,
            parser_metadata=parser_metadata,
            planner_acceptance=planner_acceptance,
            blocking_reasons=motion.get("blocking_reasons", []),
        )
        print("Final execution evidence:")
        print(_json(result))
        return 2

    if dry_run:
        result = _evidence(
            status=STATUS_PASS,
            motion=motion,
            execution={},
            parsed_contract=printed_contract,
            parser_metadata=parser_metadata,
            planner_acceptance=planner_acceptance,
        )
        print("Final execution evidence:")
        print(_json(result))
        return 0

    if not args.yes:
        confirmation = input(f"Execute on real UR5e? Type {CONFIRMATION_REPLY} to continue: ")
        if confirmation != CONFIRMATION_REPLY:
            result = _evidence(
                status=STATUS_BLOCKED,
                motion=motion,
                execution={},
                parsed_contract=printed_contract,
                parser_metadata=parser_metadata,
                planner_acceptance=planner_acceptance,
                blocking_reasons=["E_CONFIRMATION_MISMATCH"],
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
            blocking_reasons=prereq_blockers,
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
    )
    print("Final execution evidence:")
    print(_json(result))
    return 0 if status == STATUS_PASS else 2


def _parse_command_for_mode(
    command: str,
    *,
    parser_mode: str,
    real: bool,
    dry_run: bool,
    max_step_m: float,
    input_mode: str,
    qwen_model: str | None,
    qwen_endpoint: str | None,
    qwen_timeout_s: float | None,
) -> tuple[ParsedMotionCommand | None, dict[str, Any]]:
    if parser_mode == "rule":
        return _parse_rule_result(command, max_step_m=max_step_m, input_mode=input_mode)

    qwen_result = evaluate_qwen_motion_parser(
        QwenMotionParserRequest(
            user_text=command,
            max_distance_m=max_step_m,
            hard_safety_limit_m=HARD_SAFETY_LIMIT_M,
            model_name=qwen_model,
            endpoint=qwen_endpoint,
            timeout_s=qwen_timeout_s,
        )
    )
    if qwen_result.get("qwen_motion_parser_status") == STATUS_PASS:
        return _parsed_from_qwen_result(command, qwen_result, max_step_m=max_step_m), qwen_result

    if parser_mode == "auto" and dry_run and not real:
        parsed, rule_result = _parse_rule_result(command, max_step_m=max_step_m, input_mode=input_mode)
        if parsed is not None:
            rule_result["warnings"] = _unique(_string_list(rule_result.get("warnings")) + ["qwen_failed_dry_run_rule_fallback"])
            rule_result["raw_llm_output"] = qwen_result.get("raw_llm_output")
            rule_result["qwen_blocking_reasons"] = qwen_result.get("parser_blocking_reasons", [])
            return parsed, rule_result

    return None, qwen_result


def _parse_rule_result(command: str, *, max_step_m: float, input_mode: str) -> tuple[ParsedMotionCommand | None, dict[str, Any]]:
    try:
        parsed = parse_motion_command(command, max_step_m=max_step_m)
    except MotionParseError as exc:
        reasons = [str(exc)]
        return None, {
            "parser_source": "rule_based",
            "llm_called": False,
            "model_name": None,
            "qwen_endpoint": None,
            "llm_latency_ms": None,
            "raw_llm_output": None,
            "normalized_contract": None,
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
        "parser_blocking_reasons": [],
        "blocking_reasons": [],
        "input_mode": input_mode,
    }


def _parsed_from_qwen_result(command: str, result: dict[str, Any], *, max_step_m: float) -> ParsedMotionCommand:
    delta = result.get("delta_m")
    if not _vector3(delta):
        raise MotionParseError("E_INVALID_QWEN_DELTA")
    distance_m = float(result.get("distance_m"))
    return ParsedMotionCommand(
        command=command.strip(),
        frame="base_link",
        delta_m=[round(float(value), 6) for value in delta],
        distance_m=round(distance_m, 6),
        max_distance_m=max_step_m,
        hard_safety_limit_m=HARD_SAFETY_LIMIT_M,
        direction=f"{result.get('axis')}{result.get('direction')}",
        unit="qwen_json",
        parser_source="qwen_llm",
        llm_called=True,
        model_name=result.get("model_name"),
        qwen_endpoint=result.get("qwen_endpoint"),
        llm_latency_ms=result.get("llm_latency_ms"),
        raw_llm_output=result.get("raw_llm_output"),
        parser_blocking_reasons=result.get("parser_blocking_reasons", []),
    )


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
    )


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


def _lookup_current_tcp_pose(*, timeout_s: float) -> dict[str, Any] | None:
    try:
        import rclpy
        from rclpy.duration import Duration
        from tf2_ros import Buffer, TransformListener
    except Exception:
        return None

    initialized_here = False
    try:
        if not rclpy.ok():
            rclpy.init(args=None)
            initialized_here = True
        node = rclpy.create_node("teto_text_to_ur5e_tcp_lookup")
        buffer = Buffer()
        listener = TransformListener(buffer, node)
        end = node.get_clock().now() + Duration(seconds=float(timeout_s))
        while rclpy.ok() and node.get_clock().now() < end:
            rclpy.spin_once(node, timeout_sec=0.1)
            try:
                tf = buffer.lookup_transform("base_link", "tool0", rclpy.time.Time())
                t = tf.transform.translation
                q = tf.transform.rotation
                return {
                    "frame": "base_link",
                    "position_m": [t.x, t.y, t.z],
                    "orientation_xyzw": [q.x, q.y, q.z, q.w],
                }
            except Exception:
                pass
        return None
    except Exception:
        return None
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
    max_step_m: float,
    speed_scale: float,
    acc_scale: float,
    real: bool,
    dry_run: bool,
) -> dict[str, Any]:
    return {
        "moveit_execution_mode": "real" if real and not dry_run else "shadow",
        "enable_ros2_runtime": bool(real and not dry_run),
        "enable_moveit_plan": bool(real and not dry_run),
        "enable_moveit_execute": bool(real and not dry_run),
        "enable_real_robot_motion": bool(real and not dry_run),
        "manual_confirmation_required": True,
        "planning_group": "ur_manipulator",
        "planning_frame": "base_link",
        "end_effector_frame": "tool0",
        "end_effector_link": "tool0",
        "move_group_action_name": "/move_action",
        "execute_trajectory_action_name": "/execute_trajectory",
        "pipeline_id": "move_group",
        "planner_id": "ur_manipulator[RRTConnectkConfigDefault]",
        "allowed_frames": ["base_link"],
        "max_translation_m": max_step_m,
        "hard_safety_limit_m": HARD_SAFETY_LIMIT_M,
        "workspace_bounds": {"x": [-1.0, 1.0], "y": [-1.0, 1.0], "z": [0.0, 2.0]},
        "position_tolerance_m": 0.005,
        "orientation_tolerance_rad": 0.05,
        "max_speed_scale": min(float(speed_scale), 0.10),
        "max_acc_scale": min(float(acc_scale), 0.10),
        "current_tcp_pose": current_pose,
        "robot_state_ok": True,
        "safety_status_ok": True,
        "protective_stop": False,
        "emergency_stop": False,
        "speed_scaling": min(float(speed_scale), 0.10),
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
    trajectory_metrics = _trajectory_metrics(moveit_result)

    trajectory_sent = (
        execution.get("trajectory_sent") is True
        or moveit_result.get("trajectory_sent") is True
        or motion.get("trajectory_sent") is True
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
        and requested_distance <= float(execution_preview.get("hard_safety_limit_m", HARD_SAFETY_LIMIT_M)) + EPS
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
    if execute_called:
        blocking_reasons.append("E_PLANNER_ACCEPTANCE_EXECUTE_CALLED")

    max_joint_delta = trajectory_metrics["max_joint_delta_rad"]
    if (
        max_joint_delta is not None
        and requested_distance is not None
        and requested_distance <= HARD_SAFETY_LIMIT_M + EPS
        and max_joint_delta > SUSPICIOUS_TINY_MOTION_JOINT_DELTA_RAD
    ):
        warnings.append("W_SUSPICIOUS_JOINT_DELTA_FOR_TINY_CARTESIAN_MOTION")
    orientation_change = trajectory_metrics["orientation_change_rad"]
    if (
        orientation_change is not None
        and requested_distance is not None
        and requested_distance <= HARD_SAFETY_LIMIT_M + EPS
        and orientation_change > SUSPICIOUS_TINY_MOTION_ORIENTATION_CHANGE_RAD
    ):
        warnings.append("W_SUSPICIOUS_ORIENTATION_CHANGE_FOR_TINY_CARTESIAN_MOTION")

    blocking_reasons = _unique(blocking_reasons)
    warnings = _unique(warnings)
    reasonableness_check = STATUS_BLOCKED if blocking_reasons else STATUS_WARNING if warnings else STATUS_PASS
    status = STATUS_BLOCKED if blocking_reasons else STATUS_WARNING if warnings else STATUS_PASS
    return {
        "status": status,
        "plan_only": dry_run and not execution_allowed,
        "execution_allowed": execution_allowed,
        "trajectory_sent": trajectory_sent,
        "execute_trajectory_called": execute_called,
        "requested_delta_m": requested_delta if _vector3(requested_delta) else None,
        "requested_distance_m": requested_distance,
        "planned_goal_frame": motion.get("frame") or execution_preview.get("frame"),
        "metrics_source": trajectory_metrics["metrics_source"],
        "planned_waypoint_count": trajectory_metrics["planned_waypoint_count"],
        "estimated_cartesian_path_length_m": trajectory_metrics["estimated_cartesian_path_length_m"],
        "max_joint_delta_rad": max_joint_delta,
        "total_joint_motion_rad": trajectory_metrics["total_joint_motion_rad"],
        "orientation_change_rad": trajectory_metrics["orientation_change_rad"],
        "trajectory_duration_s": trajectory_metrics["trajectory_duration_s"],
        "reasonableness_check": reasonableness_check,
        "blocking_reasons": blocking_reasons,
        "warnings": warnings,
    }


def _moveit_result_from(motion: dict[str, Any], execution: dict[str, Any]) -> dict[str, Any]:
    if isinstance(execution.get("moveit_pose_executor_result"), dict):
        return execution["moveit_pose_executor_result"]
    if isinstance(motion.get("moveit_pose_executor_result"), dict):
        return motion["moveit_pose_executor_result"]
    return {}


def _trajectory_metrics(moveit_result: dict[str, Any]) -> dict[str, Any]:
    waypoint_count = moveit_result.get("trajectory_point_count")
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
    blocking_reasons: list[str] | None = None,
) -> dict[str, Any]:
    moveit = execution.get("moveit_pose_executor_result") if isinstance(execution.get("moveit_pose_executor_result"), dict) else {}
    motion_check = _motion_check_fields(motion, moveit, parsed_contract)
    return {
        **parser_metadata,
        "parsed_contract": parsed_contract,
        "execution_preview": parsed_contract.get("execution_preview"),
        "planner_acceptance": planner_acceptance,
        "cartesian_motion_gateway_status": motion.get("cartesian_motion_gateway_status"),
        "cartesian_motion_execution_status": execution.get("cartesian_motion_execution_status"),
        "goal_accepted": moveit.get("goal_accepted", False),
        "execute_accepted": moveit.get("execute_success", False),
        "moveit_execute_error_code": moveit.get("moveit_execute_error_code"),
        "moveit_execute_error_code_name": moveit.get("moveit_execute_error_code_name"),
        "trajectory_sent": execution.get("trajectory_sent", False),
        "controller_command_sent": execution.get("controller_command_sent", False),
        "real_robot_motion_executed": execution.get("real_robot_motion_executed", False),
        "target_pose": motion.get("target_pose"),
        **motion_check,
        "blocking_reasons": blocking_reasons if blocking_reasons is not None else execution.get("blocking_reasons", []),
        "warnings": execution.get("warnings", []),
        "final_status": status,
    }


def _blocked_result(reason: str, parsed_contract: dict[str, Any], parser_metadata: dict[str, Any]) -> dict[str, Any]:
    return {
        **parser_metadata,
        "parsed_contract": parsed_contract,
        "execution_preview": parsed_contract.get("execution_preview"),
        "planner_acceptance": None,
        **_empty_motion_check(parsed_contract),
        "goal_accepted": False,
        "execute_accepted": False,
        "moveit_execute_error_code": None,
        "moveit_execute_error_code_name": None,
        "trajectory_sent": False,
        "controller_command_sent": False,
        "real_robot_motion_executed": False,
        "blocking_reasons": [reason],
        "final_status": STATUS_BLOCKED,
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
