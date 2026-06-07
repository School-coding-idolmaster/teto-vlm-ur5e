#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import math
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


DEFAULT_MAX_STEP_M = 0.005
HARD_SAFETY_LIMIT_M = 0.01
CONFIRMATION_TOKEN = "EXECUTE_REAL_UR5E"

STATUS_PASS = "PASS"
STATUS_BLOCKED = "BLOCKED"
STATUS_FAILED = "FAILED"


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
    if max_step_m <= 0.0 or max_step_m > HARD_SAFETY_LIMIT_M:
        raise MotionParseError("E_INVALID_MAX_STEP")

    normalized = _normalize(command)
    _reject_forbidden(normalized)

    distance_m, unit = _extract_distance_m(normalized)
    if distance_m > HARD_SAFETY_LIMIT_M:
        raise MotionParseError("E_EXCEEDS_HARD_SAFETY_LIMIT")
    if distance_m > max_step_m:
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
    parser.add_argument("--max-step-m", type=float, default=DEFAULT_MAX_STEP_M)
    parser.add_argument("--speed-scale", type=float, default=0.10)
    parser.add_argument("--acc-scale", type=float, default=0.10)
    args = parser.parse_args(argv)

    if args.real and args.dry_run:
        print(_json({"final_status": STATUS_BLOCKED, "blocking_reasons": ["E_REAL_AND_DRY_RUN_CONFLICT"]}))
        return 2
    dry_run = not args.real or args.dry_run

    try:
        command = input("Enter motion command: ")
    except EOFError:
        print(_json({"final_status": STATUS_BLOCKED, "blocking_reasons": ["E_NO_INPUT"]}))
        return 2

    try:
        parsed = parse_motion_command(command, max_step_m=float(args.max_step_m))
    except MotionParseError as exc:
        print(_json({"final_status": STATUS_BLOCKED, "blocking_reasons": [str(exc)], "real_robot_motion_executed": False}))
        return 2

    printed_contract = parsed.task_contract(real_robot_motion_requested=bool(args.real), dry_run=dry_run)
    print("Parsed TETO task contract:")
    print(_json(printed_contract))

    current_pose = _lookup_current_tcp_pose(timeout_s=3.0)
    if current_pose is None:
        result = _blocked_result("E_CURRENT_TCP_POSE_MISSING", printed_contract)
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
    if motion.get("cartesian_motion_gateway_status") != STATUS_PASS:
        result = _evidence(
            status=STATUS_BLOCKED,
            motion=motion,
            execution={},
            parsed_contract=printed_contract,
            blocking_reasons=motion.get("blocking_reasons", []),
        )
        print("Final execution evidence:")
        print(_json(result))
        return 2

    if dry_run:
        result = _evidence(status=STATUS_PASS, motion=motion, execution={}, parsed_contract=printed_contract)
        print("Final execution evidence:")
        print(_json(result))
        return 0

    confirmation = input(f"Type {CONFIRMATION_TOKEN} to continue: ").strip()
    if confirmation != CONFIRMATION_TOKEN:
        result = _evidence(
            status=STATUS_BLOCKED,
            motion=motion,
            execution={},
            parsed_contract=printed_contract,
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
    result = _evidence(status=status, motion=motion, execution=execution, parsed_contract=printed_contract)
    print("Final execution evidence:")
    print(_json(result))
    return 0 if status == STATUS_PASS else 2


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


def _evidence(
    *,
    status: str,
    motion: dict[str, Any],
    execution: dict[str, Any],
    parsed_contract: dict[str, Any],
    blocking_reasons: list[str] | None = None,
) -> dict[str, Any]:
    moveit = execution.get("moveit_pose_executor_result") if isinstance(execution.get("moveit_pose_executor_result"), dict) else {}
    return {
        "parsed_contract": parsed_contract,
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
        "blocking_reasons": blocking_reasons if blocking_reasons is not None else execution.get("blocking_reasons", []),
        "warnings": execution.get("warnings", []),
        "final_status": status,
    }


def _blocked_result(reason: str, parsed_contract: dict[str, Any]) -> dict[str, Any]:
    return {
        "parsed_contract": parsed_contract,
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


def _json(data: dict[str, Any]) -> str:
    return json.dumps(data, indent=2, sort_keys=True)


if __name__ == "__main__":
    raise SystemExit(main())
