from __future__ import annotations

import hashlib
import math
import socket
import time
import urllib.error
import urllib.request
from dataclasses import dataclass
from datetime import datetime, timezone
from typing import Any, Callable

from src.cartesian_motion_gateway import (
    CartesianMotionExecutionRequest,
    CartesianMotionGatewayRequest,
    evaluate_cartesian_motion_execution,
    evaluate_cartesian_motion_gateway,
)


STATUS_PASS = "PASS"
STATUS_BLOCKED = "BLOCKED"
EPS = 1e-9
E_DASHBOARD_UNAVAILABLE = "E_DASHBOARD_UNAVAILABLE"
E_DASHBOARD_NOT_REMOTE = "E_DASHBOARD_NOT_REMOTE"
E_PROGRAM_NOT_PLAYING = "E_PROGRAM_NOT_PLAYING"
E_SAFETY_STATUS_NOT_NORMAL = "E_SAFETY_STATUS_NOT_NORMAL"
E_TCP_POSE_UNAVAILABLE = "E_TCP_POSE_UNAVAILABLE"
E_TCP_POSE_STALE = "E_TCP_POSE_STALE"
E_JOINT_STATES_UNAVAILABLE = "E_JOINT_STATES_UNAVAILABLE"
E_CONTROLLER_MANAGER_UNAVAILABLE = "E_CONTROLLER_MANAGER_UNAVAILABLE"
E_SCALED_CONTROLLER_INACTIVE = "E_SCALED_CONTROLLER_INACTIVE"
E_D455_TOPIC_NOT_FOUND = "E_D455_TOPIC_NOT_FOUND"
E_D455_SNAPSHOT_UNAVAILABLE = "E_D455_SNAPSHOT_UNAVAILABLE"
E_MOVEIT_EXECUTOR_UNAVAILABLE = "E_MOVEIT_EXECUTOR_UNAVAILABLE"


@dataclass(frozen=True)
class RealSegmentedBackendConfig:
    max_substep_distance_m: float = 0.05
    position_tolerance_m: float = 0.005
    orientation_tolerance_rad: float = 0.01
    tcp_pose_stale_after_s: float = 1.0
    snapshot_max_age_s: float = 1.0
    snapshot_sync_tolerance_s: float = 0.25
    ros_timeout_s: float = 3.0
    post_motion_settle_s: float = 0.25
    speed_scale: float = 0.10
    acc_scale: float = 0.10
    planning_group: str = "ur_manipulator"
    planning_frame: str = "base_link"
    end_effector_link: str = "tool0"
    robot_ip: str = "192.168.20.35"
    dashboard_port: int = 29999
    dashboard_timeout_s: float = 2.0
    tcp_pose_topic: str = "/tcp_pose"
    joint_states_topic: str = "/joint_states"
    color_topic: str | None = None
    depth_topic: str | None = None
    qwen_health_url: str = "http://127.0.0.1:18080/health"
    workspace_bounds: dict[str, list[float]] | None = None


class DashboardTCPClient:
    """Read-only Universal Robots Dashboard client."""

    def __init__(
        self,
        *,
        host: str,
        port: int = 29999,
        timeout_s: float = 2.0,
        socket_factory: Callable[..., Any] = socket.create_connection,
    ) -> None:
        self.host = host
        self.port = int(port)
        self.timeout_s = float(timeout_s)
        self.socket_factory = socket_factory

    def read_status(self) -> dict[str, Any]:
        try:
            with self.socket_factory(
                (self.host, self.port),
                timeout=self.timeout_s,
            ) as connection:
                connection.settimeout(self.timeout_s)
                welcome = _recv_dashboard_line(connection)
                responses = {
                    "robotmode": self._query(connection, "robotmode"),
                    "safetystatus": self._query(connection, "safetystatus"),
                    "programState": self._query(connection, "programState"),
                    "remote_control": self._query(
                        connection,
                        "is in remote control",
                    ),
                }
        except (OSError, TimeoutError) as exc:
            return _blocked(
                E_DASHBOARD_UNAVAILABLE,
                dashboard_reachable=False,
                dashboard_host=self.host,
                dashboard_port=self.port,
                dashboard_error=str(exc),
            )

        robotmode = _dashboard_value(responses["robotmode"]).upper()
        safetystatus = _dashboard_value(responses["safetystatus"]).upper()
        program_state = _dashboard_program_state(
            responses["programState"]
        ).upper()
        remote_control = _dashboard_bool(responses["remote_control"])
        blockers: list[str] = []
        if remote_control is not True:
            blockers.append(E_DASHBOARD_NOT_REMOTE)
        if robotmode != "RUNNING" or program_state != "PLAYING":
            blockers.append(E_PROGRAM_NOT_PLAYING)
        if safetystatus != "NORMAL":
            blockers.append(E_SAFETY_STATUS_NOT_NORMAL)
        return {
            "status": STATUS_PASS if not blockers else STATUS_BLOCKED,
            "abort_reason": blockers[0] if blockers else None,
            "blocking_reasons": blockers,
            "dashboard_reachable": True,
            "dashboard_host": self.host,
            "dashboard_port": self.port,
            "dashboard_welcome": welcome,
            "robotmode": robotmode or None,
            "safetystatus": safetystatus or None,
            "programState": program_state or None,
            "remote_control": remote_control,
            "dashboard_raw_responses": responses,
            "dashboard_command_attempted": False,
        }

    def _query(self, connection: Any, command: str) -> str:
        connection.sendall(f"{command}\n".encode("ascii"))
        return _recv_dashboard_line(connection)


class RealOperatorStateProvider:
    """Concrete provider wired to the lab Dashboard and ROS graph."""

    def __init__(
        self,
        config: RealSegmentedBackendConfig,
        *,
        dashboard_client: DashboardTCPClient | None = None,
    ) -> None:
        self.config = config
        self.dashboard_client = dashboard_client or DashboardTCPClient(
            host=config.robot_ip,
            port=config.dashboard_port,
            timeout_s=config.dashboard_timeout_s,
        )
        self._d455_topics: dict[str, str] | None = None

    def read_tcp_pose(self) -> dict[str, Any]:
        return _read_tcp_pose_topic(
            topic=self.config.tcp_pose_topic,
            timeout_s=self.config.ros_timeout_s,
            planning_frame=self.config.planning_frame,
            stale_after_s=self.config.tcp_pose_stale_after_s,
        )

    def read_joint_states(self) -> dict[str, Any]:
        return _read_joint_states_topic(
            topic=self.config.joint_states_topic,
            timeout_s=self.config.ros_timeout_s,
        )

    def check_motion_state(self) -> dict[str, Any]:
        dashboard = self.dashboard_client.read_status()
        controller = _read_controller_status(
            timeout_s=self.config.ros_timeout_s
        )
        joints = self.read_joint_states()
        moveit = _read_moveit_status(timeout_s=self.config.ros_timeout_s)
        ordered = (dashboard, controller, joints, moveit)
        blocker = next(
            (
                str(item.get("abort_reason"))
                for item in ordered
                if item.get("status") != STATUS_PASS
                and item.get("abort_reason")
            ),
            None,
        )
        passed = blocker is None
        return {
            "status": STATUS_PASS if passed else STATUS_BLOCKED,
            "abort_reason": blocker,
            "blocking_reasons": [
                str(item["abort_reason"])
                for item in ordered
                if item.get("status") != STATUS_PASS
                and item.get("abort_reason")
            ],
            "authoritative_state_check": True,
            "synthetic_safety_state_used": False,
            "dashboard": dashboard,
            "controller": controller,
            "joint_states": joints,
            "moveit": moveit,
            "dashboard_reachable": dashboard.get("dashboard_reachable") is True,
            "dashboard_robot_mode_running": dashboard.get("robotmode")
            == "RUNNING",
            "dashboard_safety_mode_ok": dashboard.get("safetystatus")
            == "NORMAL",
            "dashboard_safety_status_ok": dashboard.get("safetystatus")
            == "NORMAL",
            "dashboard_program_state_playing": dashboard.get("programState")
            == "PLAYING",
            "remote_control": dashboard.get("remote_control"),
            "external_control_playing": dashboard.get("programState")
            == "PLAYING",
            "controller_active": controller.get("controller_active") is True,
            "robot_state_ok": passed,
            "safety_status_ok": dashboard.get("safetystatus") == "NORMAL",
            "protective_stop": dashboard.get("safetystatus")
            == "PROTECTIVE_STOP",
            "emergency_stop": "EMERGENCY_STOP"
            in str(dashboard.get("safetystatus") or ""),
        }

    def discover_d455_topics(self) -> dict[str, Any]:
        result = _discover_d455_topics(
            configured_color=self.config.color_topic,
            configured_depth=self.config.depth_topic,
            timeout_s=self.config.ros_timeout_s,
        )
        if result.get("status") == STATUS_PASS:
            self._d455_topics = {
                "color_topic": str(result["color_topic"]),
                "depth_topic": str(result["depth_topic"]),
            }
        return result

    def capture_snapshot(
        self,
        *,
        phase: str,
        previous_snapshot: dict[str, Any] | None = None,
    ) -> dict[str, Any]:
        topics = self._d455_topics or {}
        if not topics:
            discovery = self.discover_d455_topics()
            if discovery.get("status") != STATUS_PASS:
                return {**discovery, "snapshot_phase": phase}
            topics = self._d455_topics or {}
        return _capture_d455_snapshot(
            color_topic=str(topics["color_topic"]),
            depth_topic=str(topics["depth_topic"]),
            timeout_s=self.config.ros_timeout_s,
            max_age_s=self.config.snapshot_max_age_s,
            sync_tolerance_s=self.config.snapshot_sync_tolerance_s,
            phase=phase,
            previous_snapshot=previous_snapshot,
        )

    def status(self) -> dict[str, Any]:
        state = self.check_motion_state()
        pose = self.read_tcp_pose()
        d455_topics = self.discover_d455_topics()
        snapshot = (
            self.capture_snapshot(phase="status")
            if d455_topics.get("status") == STATUS_PASS
            else {**d455_topics, "snapshot_phase": "status"}
        )
        qwen = _read_qwen_health(
            self.config.qwen_health_url,
            timeout_s=self.config.ros_timeout_s,
        )
        ordered = (state, pose, snapshot, qwen)
        blocker = next(
            (
                str(item.get("abort_reason"))
                for item in ordered
                if item.get("status") != STATUS_PASS
                and item.get("abort_reason")
            ),
            None,
        )
        return {
            "status": STATUS_PASS if blocker is None else STATUS_BLOCKED,
            "abort_reason": blocker,
            "dashboard_reachable": state.get("dashboard_reachable"),
            "robotmode": (state.get("dashboard") or {}).get("robotmode"),
            "safetystatus": (state.get("dashboard") or {}).get(
                "safetystatus"
            ),
            "programState": (state.get("dashboard") or {}).get(
                "programState"
            ),
            "remote_control": state.get("remote_control"),
            "tcp_pose_available": pose.get("tcp_pose_available") is True,
            "tcp_pose_fresh": pose.get("tcp_pose_fresh"),
            "tcp_pose_stamp": pose.get("tcp_pose_stamp"),
            "tcp_pose_topic": pose.get("tcp_pose_topic")
            or self.config.tcp_pose_topic,
            "joint_states_available": (
                (state.get("joint_states") or {}).get("status") == STATUS_PASS
            ),
            "joint_states_topic": self.config.joint_states_topic,
            "scaled_joint_trajectory_controller_active": (
                (state.get("controller") or {}).get("controller_active")
                is True
            ),
            "d455_color_topic": snapshot.get("color_topic")
            or d455_topics.get("color_topic"),
            "d455_depth_topic": snapshot.get("depth_topic")
            or d455_topics.get("depth_topic"),
            "d455_snapshot_fresh": snapshot.get(
                "freshness_check_passed"
            ),
            "d455_status": snapshot,
            "moveit_executor_available": (
                (state.get("moveit") or {}).get("status") == STATUS_PASS
            ),
            "qwen_healthy": qwen.get("healthy") is True,
            "qwen": qwen,
            "state": state,
            "current_tcp_pose": pose,
            "vision": snapshot,
            "manual_confirmation_required": False,
            "autonomous_segmented_execution": True,
            "safety_gate_still_required": True,
        }


class RealSegmentedOperatorBackend:
    backend_name = "ur5e_moveit_measured"
    execution_mode = "real_ur5e"
    autonomous_segmented_execution = True
    vision_guard_required = True

    def __init__(
        self,
        config: RealSegmentedBackendConfig | None = None,
        *,
        state_provider: RealOperatorStateProvider | None = None,
    ) -> None:
        self.config = config or RealSegmentedBackendConfig()
        self.state_provider = state_provider or RealOperatorStateProvider(
            self.config
        )
        self._home_pose: dict[str, Any] | None = None
        self._last_snapshot: dict[str, Any] | None = None

    def status(self) -> dict[str, Any]:
        return self.state_provider.status()

    def read_tcp_pose(self) -> dict[str, Any]:
        result = self.state_provider.read_tcp_pose()
        pose = _pose(result)
        if pose is not None and self._home_pose is None:
            self._home_pose = pose
        return result

    def check_motion_state(self) -> dict[str, Any]:
        return self.state_provider.check_motion_state()

    def capture_snapshot(
        self,
        *,
        phase: str,
        previous_snapshot: dict[str, Any] | None = None,
    ) -> dict[str, Any]:
        snapshot = self.state_provider.capture_snapshot(
            phase=phase,
            previous_snapshot=previous_snapshot,
        )
        if snapshot.get("status") == STATUS_PASS:
            self._last_snapshot = snapshot
        return snapshot

    def execute_subgoal(
        self,
        *,
        delta_m: list[float],
        current_tcp_pose: dict[str, Any],
        target_tcp_pose: dict[str, Any],
        state_evidence: dict[str, Any],
        vision_evidence: dict[str, Any],
        substep_index: int,
        substep_count: int,
    ) -> dict[str, Any]:
        if state_evidence.get("status") != STATUS_PASS:
            return {
                "status": STATUS_BLOCKED,
                "abort_reason": state_evidence.get("abort_reason")
                or "E_REAL_STATE_CHECK_BLOCKED",
                "real_robot_motion_executed": False,
            }
        if vision_evidence.get("status") != STATUS_PASS:
            return {
                "status": STATUS_BLOCKED,
                "abort_reason": vision_evidence.get("abort_reason")
                or "E_VISION_GUARD_BLOCKED",
                "real_robot_motion_executed": False,
            }

        gateway_config = self._gateway_config(
            current_tcp_pose=current_tcp_pose,
            state_evidence=state_evidence,
            vision_evidence=vision_evidence,
            requested_distance_m=_norm(delta_m),
        )
        gateway = evaluate_cartesian_motion_gateway(
            CartesianMotionGatewayRequest(
                requested=True,
                config=gateway_config,
                command_to_task_result={
                    "command_to_task_status": STATUS_PASS,
                    "task_id": f"unified-real-{substep_index}-of-{substep_count}",
                    "task_contract": {
                        "intent": "cartesian_offset",
                        "frame": self.config.planning_frame,
                        "cartesian_offset_m": list(delta_m),
                    },
                },
                current_tcp_pose=current_tcp_pose,
            )
        )
        if gateway.get("cartesian_motion_gateway_status") != STATUS_PASS:
            return {
                "status": STATUS_BLOCKED,
                "abort_reason": (
                    gateway.get("blocking_reasons") or ["E_CARTESIAN_GATEWAY_BLOCKED"]
                )[0],
                "gateway_result": gateway,
                "real_robot_motion_executed": False,
            }
        generated_target = _pose(gateway.get("target_pose"))
        if generated_target is None or not _poses_close(generated_target, target_tcp_pose):
            return {
                "status": STATUS_BLOCKED,
                "abort_reason": "E_GATEWAY_TARGET_MISMATCH",
                "gateway_result": gateway,
                "real_robot_motion_executed": False,
            }

        execution = evaluate_cartesian_motion_execution(
            CartesianMotionExecutionRequest(
                requested=True,
                config=gateway_config,
                cartesian_motion_result=gateway,
                manual_confirmation_result={},
                ur5_state_result=state_evidence,
            )
        )
        passed = (
            execution.get("cartesian_motion_execution_status") == STATUS_PASS
            and execution.get("real_robot_motion_executed") is True
        )
        if passed and self.config.post_motion_settle_s > 0.0:
            time.sleep(self.config.post_motion_settle_s)
        return {
            "status": STATUS_PASS if passed else STATUS_BLOCKED,
            "abort_reason": (
                None
                if passed
                else (execution.get("blocking_reasons") or ["E_MOVEIT_SUBGOAL_FAILED"])[0]
            ),
            "gateway_result": gateway,
            "cartesian_motion_execution_result": execution,
            "moveit_execute_called": execution.get("moveit_execute_called") is True,
            "trajectory_sent": execution.get("trajectory_sent") is True,
            "real_robot_motion_executed": execution.get("real_robot_motion_executed")
            is True,
            "manual_confirmation_required": False,
            "autonomous_segmented_execution": True,
            "safety_gate_still_required": True,
        }

    def begin_relative_motion(
        self,
        *,
        initial_tcp_pose: dict[str, Any],
        planned_subgoals: list[dict[str, Any]],
    ) -> None:
        if self._home_pose is None:
            self._home_pose = _pose(initial_tcp_pose)

    def home_reference_pose(self) -> dict[str, Any] | None:
        return _copy_pose(self._home_pose)

    def reset_session(self) -> dict[str, Any]:
        return {
            "status": STATUS_PASS,
            "session_home_preserved": self._home_pose is not None,
            "automatic_retry_motion": False,
        }

    def _gateway_config(
        self,
        *,
        current_tcp_pose: dict[str, Any],
        state_evidence: dict[str, Any],
        vision_evidence: dict[str, Any],
        requested_distance_m: float,
    ) -> dict[str, Any]:
        max_step = float(self.config.max_substep_distance_m)
        return {
            "moveit_execution_mode": "real",
            "enable_ros2_runtime": True,
            "enable_moveit_plan": True,
            "enable_moveit_execute": True,
            "enable_real_robot_motion": True,
            "manual_confirmation_required": False,
            "autonomous_segmented_execution": True,
            "safety_gate_still_required": True,
            "authoritative_state_check": state_evidence.get(
                "authoritative_state_check"
            )
            is True,
            "vision_guard_passed": vision_evidence.get("status") == STATUS_PASS,
            "controller_active": state_evidence.get("controller_active") is True,
            "external_control_playing": state_evidence.get(
                "external_control_playing"
            )
            is True,
            "dashboard_robot_mode_running": state_evidence.get(
                "dashboard_robot_mode_running"
            )
            is True,
            "dashboard_safety_mode_ok": state_evidence.get(
                "dashboard_safety_mode_ok"
            )
            is True,
            "dashboard_safety_status_ok": state_evidence.get(
                "dashboard_safety_status_ok"
            )
            is True,
            "dashboard_program_state_playing": state_evidence.get(
                "dashboard_program_state_playing"
            )
            is True,
            "robot_state_ok": state_evidence.get("robot_state_ok") is True,
            "safety_status_ok": state_evidence.get("safety_status_ok") is True,
            "protective_stop": state_evidence.get("protective_stop") is True,
            "emergency_stop": state_evidence.get("emergency_stop") is True,
            "speed_scaling": min(float(self.config.speed_scale), 0.10),
            "max_speed_scale": min(float(self.config.speed_scale), 0.10),
            "max_acc_scale": min(float(self.config.acc_scale), 0.10),
            "planning_group": self.config.planning_group,
            "planning_frame": self.config.planning_frame,
            "end_effector_frame": self.config.end_effector_link,
            "end_effector_link": self.config.end_effector_link,
            "move_group_action_name": "/move_action",
            "execute_trajectory_action_name": "/execute_trajectory",
            "pipeline_id": "move_group",
            "planner_id": "ur_manipulator[RRTConnectkConfigDefault]",
            "allowed_frames": [self.config.planning_frame],
            "workspace_bounds": self.config.workspace_bounds
            or {"x": [-1.0, 1.0], "y": [-1.0, 1.0], "z": [0.0, 2.0]},
            "max_translation_m": max_step,
            "configured_max_distance_m": max_step,
            "max_step_distance_m": max_step,
            "max_axis_step_m": max_step,
            "hard_safety_limit_m": max_step,
            "max_one_shot_distance_m": max_step,
            "enable_long_step_decomposition": False,
            "requested_distance_m": requested_distance_m,
            "position_tolerance_m": min(
                float(self.config.position_tolerance_m),
                max(requested_distance_m * 0.10, 0.0001),
            ),
            "orientation_tolerance_rad": float(
                self.config.orientation_tolerance_rad
            ),
            "small_motion_tolerance_policy": "real_motion_safety_policy_v1",
            "safety_policy_name": "unified_real_segmented_motion_v1",
            "safety_policy_source": "measured_dashboard_controller_tcp_d455",
            "current_tcp_pose": current_tcp_pose,
            "target_pose_generated_by_llm": False,
            "urscript_generated": False,
            "rtde_write_attempted": False,
            "dashboard_command_attempted": False,
            "raw_joint_targets_generated": False,
        }


def _read_tcp_pose_topic(
    *,
    topic: str,
    timeout_s: float,
    planning_frame: str,
    stale_after_s: float,
) -> dict[str, Any]:
    try:
        import rclpy
        from geometry_msgs.msg import PoseStamped
        from rclpy.qos import qos_profile_sensor_data
    except Exception as exc:
        return _blocked(
            E_TCP_POSE_UNAVAILABLE,
            tcp_pose_topic=topic,
            tcp_pose_error=str(exc),
        )

    initialized_here = False
    node = None
    try:
        if not rclpy.ok():
            rclpy.init(args=None)
            initialized_here = True
        node = rclpy.create_node("teto_unified_tcp_pose_provider")
        resolved_topic = _resolve_topic(
            node,
            configured=topic,
            accepted_types={"geometry_msgs/msg/PoseStamped"},
            fallback_predicate=lambda name: "tcp_pose" in name.lower()
            and name.endswith("/pose"),
        )
        if resolved_topic is None:
            return _blocked(
                E_TCP_POSE_UNAVAILABLE,
                tcp_pose_topic=topic,
                available_topics=_topic_names(node),
            )
        messages: list[Any] = []
        subscription = node.create_subscription(
            PoseStamped,
            resolved_topic,
            messages.append,
            qos_profile_sensor_data,
        )
        _spin_until(node, lambda: bool(messages), timeout_s)
        node.destroy_subscription(subscription)
        if not messages:
            return _blocked(
                E_TCP_POSE_UNAVAILABLE,
                tcp_pose_topic=resolved_topic,
            )
        message = messages[-1]
        stamp = _stamp_seconds(message.header.stamp)
        now_s = node.get_clock().now().nanoseconds / 1_000_000_000.0
        age_s = max(0.0, now_s - stamp) if stamp is not None else None
        fresh = age_s is not None and age_s <= stale_after_s + EPS
        pose = message.pose
        return {
            "status": STATUS_PASS if fresh else STATUS_BLOCKED,
            "abort_reason": None if fresh else E_TCP_POSE_STALE,
            "blocking_reasons": [] if fresh else [E_TCP_POSE_STALE],
            "current_tcp_pose": {
                "frame": planning_frame,
                "position_m": [
                    pose.position.x,
                    pose.position.y,
                    pose.position.z,
                ],
                "orientation_xyzw": [
                    pose.orientation.x,
                    pose.orientation.y,
                    pose.orientation.z,
                    pose.orientation.w,
                ],
            },
            "tcp_pose_available": True,
            "tcp_pose_fresh": fresh,
            "tcp_pose_stamp": stamp,
            "tcp_pose_age_s": round(age_s, 6)
            if age_s is not None
            else None,
            "tcp_pose_stale_after_s": stale_after_s,
            "tcp_pose_topic": resolved_topic,
            "tcp_pose_source_frame": message.header.frame_id,
            "tcp_pose_source": "ros_pose_stamped",
            "synthetic_tcp_pose_used": False,
        }
    except Exception as exc:
        return _blocked(
            E_TCP_POSE_UNAVAILABLE,
            tcp_pose_topic=topic,
            tcp_pose_error=str(exc),
        )
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


def _read_joint_states_topic(*, topic: str, timeout_s: float) -> dict[str, Any]:
    try:
        import rclpy
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import JointState
    except Exception as exc:
        return _blocked(
            E_JOINT_STATES_UNAVAILABLE,
            joint_states_topic=topic,
            joint_states_error=str(exc),
        )

    initialized_here = False
    node = None
    try:
        if not rclpy.ok():
            rclpy.init(args=None)
            initialized_here = True
        node = rclpy.create_node("teto_unified_joint_state_provider")
        resolved_topic = _resolve_topic(
            node,
            configured=topic,
            accepted_types={"sensor_msgs/msg/JointState"},
            fallback_predicate=lambda name: name.endswith("/joint_states"),
        )
        if resolved_topic is None:
            return _blocked(
                E_JOINT_STATES_UNAVAILABLE,
                joint_states_topic=topic,
            )
        messages: list[Any] = []
        subscription = node.create_subscription(
            JointState,
            resolved_topic,
            messages.append,
            qos_profile_sensor_data,
        )
        _spin_until(node, lambda: bool(messages), timeout_s)
        node.destroy_subscription(subscription)
        if not messages:
            return _blocked(
                E_JOINT_STATES_UNAVAILABLE,
                joint_states_topic=resolved_topic,
            )
        message = messages[-1]
        return {
            "status": STATUS_PASS,
            "abort_reason": None,
            "joint_states_available": True,
            "joint_states_topic": resolved_topic,
            "joint_state_stamp": _stamp_seconds(message.header.stamp),
            "joint_names": list(message.name),
            "joint_positions": list(message.position),
        }
    except Exception as exc:
        return _blocked(
            E_JOINT_STATES_UNAVAILABLE,
            joint_states_topic=topic,
            joint_states_error=str(exc),
        )
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


def _read_controller_status(*, timeout_s: float) -> dict[str, Any]:
    try:
        import rclpy
        from controller_manager_msgs.srv import ListControllers
    except Exception as exc:
        return _blocked(
            E_CONTROLLER_MANAGER_UNAVAILABLE,
            controller_manager_error=str(exc),
        )

    initialized_here = False
    node = None
    try:
        if not rclpy.ok():
            rclpy.init(args=None)
            initialized_here = True
        node = rclpy.create_node("teto_unified_controller_provider")
        response = _call_service(
            node,
            ListControllers,
            "/controller_manager/list_controllers",
            timeout_s,
        )
        if response is None:
            return _blocked(E_CONTROLLER_MANAGER_UNAVAILABLE)
        controllers = {
            item.name: item.state for item in response.controller
        }
        active = (
            controllers.get("scaled_joint_trajectory_controller") == "active"
        )
        return {
            "status": STATUS_PASS if active else STATUS_BLOCKED,
            "abort_reason": None
            if active
            else E_SCALED_CONTROLLER_INACTIVE,
            "blocking_reasons": []
            if active
            else [E_SCALED_CONTROLLER_INACTIVE],
            "controller_manager_available": True,
            "controller_active": active,
            "scaled_joint_trajectory_controller_state": controllers.get(
                "scaled_joint_trajectory_controller"
            ),
            "controllers": controllers,
        }
    except Exception as exc:
        return _blocked(
            E_CONTROLLER_MANAGER_UNAVAILABLE,
            controller_manager_error=str(exc),
        )
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


def _read_moveit_status(*, timeout_s: float) -> dict[str, Any]:
    try:
        import rclpy
        from moveit_msgs.action import ExecuteTrajectory, MoveGroup
        from rclpy.action import ActionClient
    except Exception as exc:
        return _blocked(
            E_MOVEIT_EXECUTOR_UNAVAILABLE,
            moveit_error=str(exc),
        )

    initialized_here = False
    node = None
    try:
        if not rclpy.ok():
            rclpy.init(args=None)
            initialized_here = True
        node = rclpy.create_node("teto_unified_moveit_provider")
        move_client = ActionClient(node, MoveGroup, "/move_action")
        execute_client = ActionClient(
            node,
            ExecuteTrajectory,
            "/execute_trajectory",
        )
        move_ready = move_client.wait_for_server(timeout_sec=float(timeout_s))
        execute_ready = execute_client.wait_for_server(
            timeout_sec=float(timeout_s)
        )
        available = move_ready and execute_ready
        return {
            "status": STATUS_PASS if available else STATUS_BLOCKED,
            "abort_reason": None
            if available
            else E_MOVEIT_EXECUTOR_UNAVAILABLE,
            "blocking_reasons": []
            if available
            else [E_MOVEIT_EXECUTOR_UNAVAILABLE],
            "move_action_available": move_ready,
            "execute_trajectory_available": execute_ready,
            "moveit_executor_available": available,
        }
    except Exception as exc:
        return _blocked(
            E_MOVEIT_EXECUTOR_UNAVAILABLE,
            moveit_error=str(exc),
        )
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


def _discover_d455_topics(
    *,
    configured_color: str | None,
    configured_depth: str | None,
    timeout_s: float,
) -> dict[str, Any]:
    try:
        import rclpy
    except Exception as exc:
        return _blocked(
            E_D455_TOPIC_NOT_FOUND,
            d455_discovery_error=str(exc),
        )

    initialized_here = False
    node = None
    try:
        if not rclpy.ok():
            rclpy.init(args=None)
            initialized_here = True
        node = rclpy.create_node("teto_unified_d455_topic_discovery")
        _spin_until(node, lambda: bool(node.get_topic_names_and_types()), timeout_s)
        image_topics = {
            name
            for name, types in node.get_topic_names_and_types()
            if "sensor_msgs/msg/Image" in types
        }
        color = _select_image_topic(
            image_topics,
            configured=configured_color,
            preferred=(
                "color/image_raw",
                "color/image_rect_raw",
                "rgb/image_raw",
            ),
            predicate=lambda name: (
                ("color" in name.lower() or "rgb" in name.lower())
                and "depth" not in name.lower()
            ),
        )
        depth = _select_image_topic(
            image_topics,
            configured=configured_depth,
            preferred=(
                "aligned_depth_to_color/image_raw",
                "aligned_depth/image_raw",
                "depth/image_rect_raw",
                "depth/image_raw",
            ),
            predicate=lambda name: "depth" in name.lower(),
        )
        if color is None or depth is None:
            return _blocked(
                E_D455_TOPIC_NOT_FOUND,
                color_topic=color or configured_color,
                depth_topic=depth or configured_depth,
                available_image_topics=sorted(image_topics),
            )
        return {
            "status": STATUS_PASS,
            "abort_reason": None,
            "color_topic": color,
            "depth_topic": depth,
            "available_image_topics": sorted(image_topics),
            "topics_auto_discovered": not (
                configured_color and configured_depth
            ),
        }
    except Exception as exc:
        return _blocked(
            E_D455_TOPIC_NOT_FOUND,
            d455_discovery_error=str(exc),
        )
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


def _read_qwen_health(url: str, *, timeout_s: float) -> dict[str, Any]:
    try:
        with urllib.request.urlopen(url, timeout=float(timeout_s)) as response:
            payload = response.read().decode("utf-8", errors="replace")
        healthy = response.status == 200
        return {
            "status": STATUS_PASS if healthy else STATUS_BLOCKED,
            "abort_reason": None if healthy else "E_QWEN_UNAVAILABLE",
            "healthy": healthy,
            "url": url,
            "response": payload[:1000],
        }
    except (OSError, urllib.error.URLError) as exc:
        return _blocked(
            "E_QWEN_UNAVAILABLE",
            healthy=False,
            url=url,
            qwen_error=str(exc),
        )


def _capture_d455_snapshot(
    *,
    color_topic: str,
    depth_topic: str,
    timeout_s: float,
    max_age_s: float,
    sync_tolerance_s: float,
    phase: str,
    previous_snapshot: dict[str, Any] | None,
) -> dict[str, Any]:
    try:
        import rclpy
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import Image
    except Exception as exc:
        return _blocked(
            E_D455_SNAPSHOT_UNAVAILABLE,
            snapshot_phase=phase,
            d455_snapshot_error=str(exc),
        )

    initialized_here = False
    node = None
    latest: dict[str, Any] = {"color": None, "depth": None}
    try:
        if not rclpy.ok():
            rclpy.init(args=None)
            initialized_here = True
        node = rclpy.create_node("teto_unified_d455_snapshot_guard")
        color_subscription = node.create_subscription(
            Image,
            color_topic,
            lambda message: latest.__setitem__("color", message),
            qos_profile_sensor_data,
        )
        depth_subscription = node.create_subscription(
            Image,
            depth_topic,
            lambda message: latest.__setitem__("depth", message),
            qos_profile_sensor_data,
        )
        deadline = time.monotonic() + float(timeout_s)
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if latest["color"] is not None and latest["depth"] is not None:
                break
        if latest["color"] is None or latest["depth"] is None:
            return _blocked(
                "E_D455_SNAPSHOT_UNAVAILABLE",
                snapshot_phase=phase,
                color_topic=color_topic,
                depth_topic=depth_topic,
                color_available=latest["color"] is not None,
                depth_available=latest["depth"] is not None,
            )

        color_stamp = _stamp_seconds(latest["color"].header.stamp)
        depth_stamp = _stamp_seconds(latest["depth"].header.stamp)
        now_s = node.get_clock().now().nanoseconds / 1_000_000_000.0
        color_age = now_s - color_stamp if color_stamp is not None else None
        depth_age = now_s - depth_stamp if depth_stamp is not None else None
        synchronized = (
            color_stamp is not None
            and depth_stamp is not None
            and abs(color_stamp - depth_stamp) <= sync_tolerance_s + EPS
        )
        fresh = (
            color_age is not None
            and depth_age is not None
            and color_age <= max_age_s + EPS
            and depth_age <= max_age_s + EPS
        )
        latest_stamp = max(color_stamp or 0.0, depth_stamp or 0.0)
        previous_stamp = _number(
            (previous_snapshot or {}).get("latest_capture_stamp")
        )
        newer_than_previous = (
            previous_stamp is None or latest_stamp > previous_stamp + EPS
        )
        passed = synchronized and fresh and newer_than_previous
        blocker = None
        if not fresh:
            blocker = "E_D455_SNAPSHOT_STALE"
        elif not synchronized:
            blocker = "E_D455_RGB_DEPTH_NOT_SYNCHRONIZED"
        elif not newer_than_previous:
            blocker = "E_D455_AFTER_SNAPSHOT_NOT_NEW"
        snapshot_id = hashlib.sha256(
            b"|".join(
                (
                    bytes(latest["color"].data),
                    bytes(latest["depth"].data),
                    f"{color_stamp}:{depth_stamp}".encode("utf-8"),
                )
            )
        ).hexdigest()[:24]
        return {
            "status": STATUS_PASS if passed else STATUS_BLOCKED,
            "abort_reason": blocker,
            "snapshot_phase": phase,
            "snapshot_id": f"d455-{snapshot_id}",
            "source": "realsense_d455_ros",
            "capture_timestamp": datetime.now(timezone.utc).isoformat(),
            "color_topic": color_topic,
            "depth_topic": depth_topic,
            "color_stamp": color_stamp,
            "depth_stamp": depth_stamp,
            "latest_capture_stamp": latest_stamp,
            "color_age_s": round(color_age, 6)
            if color_age is not None
            else None,
            "depth_age_s": round(depth_age, 6)
            if depth_age is not None
            else None,
            "max_age_s": max_age_s,
            "rgb_depth_skew_s": round(abs(color_stamp - depth_stamp), 6)
            if color_stamp is not None and depth_stamp is not None
            else None,
            "sync_tolerance_s": sync_tolerance_s,
            "freshness_check_passed": fresh,
            "synchronization_check_passed": synchronized,
            "newer_than_previous": newer_than_previous,
            "color_frame_id": latest["color"].header.frame_id,
            "depth_frame_id": latest["depth"].header.frame_id,
            "color_width": latest["color"].width,
            "color_height": latest["color"].height,
            "depth_width": latest["depth"].width,
            "depth_height": latest["depth"].height,
            "color_encoding": latest["color"].encoding,
            "depth_encoding": latest["depth"].encoding,
            "live_vlm_called": False,
            "future_scene_check_hook": "adaptive_reobservation_or_vlm_scene_guard",
        }
    except Exception as exc:
        return _blocked(
            E_D455_SNAPSHOT_UNAVAILABLE,
            snapshot_phase=phase,
            d455_snapshot_error=str(exc),
        )
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


def _call_service(
    node: Any,
    service_type: Any,
    service_name: str,
    timeout_s: float,
) -> Any:
    client = node.create_client(service_type, service_name)
    if not client.wait_for_service(timeout_sec=float(timeout_s)):
        return None
    future = client.call_async(service_type.Request())
    deadline = time.monotonic() + float(timeout_s)
    while time.monotonic() < deadline and not future.done():
        import rclpy

        rclpy.spin_once(node, timeout_sec=0.1)
    return future.result() if future.done() else None


def _read_bool_topic(
    node: Any,
    message_type: Any,
    topic: str,
    timeout_s: float,
) -> bool | None:
    result: list[bool] = []
    subscription = node.create_subscription(
        message_type,
        topic,
        lambda message: result.append(bool(message.data)),
        1,
    )
    deadline = time.monotonic() + float(timeout_s)
    while time.monotonic() < deadline and not result:
        import rclpy

        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_subscription(subscription)
    return result[-1] if result else None


def _recv_dashboard_line(connection: Any) -> str:
    chunks: list[bytes] = []
    while True:
        chunk = connection.recv(4096)
        if not chunk:
            break
        chunks.append(chunk)
        if b"\n" in chunk:
            break
    return b"".join(chunks).decode("utf-8", errors="replace").strip()


def _dashboard_value(response: str) -> str:
    text = str(response or "").strip()
    return text.split(":", 1)[1].strip() if ":" in text else text


def _dashboard_program_state(response: str) -> str:
    value = _dashboard_value(response)
    return value.split()[0] if value else ""


def _dashboard_bool(response: str) -> bool | None:
    value = _dashboard_value(response).strip().lower()
    if value in {"true", "yes", "1"}:
        return True
    if value in {"false", "no", "0"}:
        return False
    return None


def _spin_until(
    node: Any,
    predicate: Callable[[], bool],
    timeout_s: float,
) -> bool:
    import rclpy

    deadline = time.monotonic() + float(timeout_s)
    while rclpy.ok() and time.monotonic() < deadline:
        if predicate():
            return True
        rclpy.spin_once(node, timeout_sec=0.1)
    return predicate()


def _topic_names(node: Any) -> list[str]:
    try:
        return sorted(name for name, _types in node.get_topic_names_and_types())
    except Exception:
        return []


def _resolve_topic(
    node: Any,
    *,
    configured: str,
    accepted_types: set[str],
    fallback_predicate: Callable[[str], bool],
) -> str | None:
    topics = {
        name: set(types) for name, types in node.get_topic_names_and_types()
    }
    if configured in topics and topics[configured] & accepted_types:
        return configured
    candidates = sorted(
        name
        for name, types in topics.items()
        if types & accepted_types and fallback_predicate(name)
    )
    return candidates[0] if candidates else None


def _select_image_topic(
    topics: set[str],
    *,
    configured: str | None,
    preferred: tuple[str, ...],
    predicate: Callable[[str], bool],
) -> str | None:
    if configured:
        return configured if configured in topics else None
    candidates = [name for name in topics if predicate(name)]
    for suffix in preferred:
        matches = sorted(name for name in candidates if name.endswith(suffix))
        if matches:
            return matches[0]
    return sorted(candidates)[0] if candidates else None


def _stamp_seconds(stamp: Any) -> float | None:
    sec = _number(getattr(stamp, "sec", None))
    nanosec = _number(getattr(stamp, "nanosec", None))
    if sec is None:
        return None
    return sec + (nanosec or 0.0) / 1_000_000_000.0


def _pose(value: Any) -> dict[str, Any] | None:
    if not isinstance(value, dict):
        return None
    if isinstance(value.get("current_tcp_pose"), dict):
        value = value["current_tcp_pose"]
    position = value.get("position_m")
    orientation = value.get("orientation_xyzw")
    if not _vector(position, 3) or not _vector(orientation, 4):
        return None
    return {
        "frame": str(value.get("frame") or "base_link"),
        "position_m": [float(item) for item in position],
        "orientation_xyzw": [float(item) for item in orientation],
    }


def _copy_pose(value: Any) -> dict[str, Any] | None:
    pose = _pose(value)
    if pose is None:
        return None
    return {
        "frame": pose["frame"],
        "position_m": list(pose["position_m"]),
        "orientation_xyzw": list(pose["orientation_xyzw"]),
    }


def _poses_close(left: dict[str, Any], right: dict[str, Any]) -> bool:
    left_pose = _pose(left)
    right_pose = _pose(right)
    if left_pose is None or right_pose is None:
        return False
    return (
        left_pose["frame"] == right_pose["frame"]
        and all(
            abs(a - b) <= 1e-6
            for a, b in zip(
                left_pose["position_m"],
                right_pose["position_m"],
            )
        )
        and all(
            abs(a - b) <= 1e-6
            for a, b in zip(
                left_pose["orientation_xyzw"],
                right_pose["orientation_xyzw"],
            )
        )
    )


def _vector(value: Any, length: int) -> bool:
    return (
        isinstance(value, (list, tuple))
        and len(value) == length
        and all(_number(item) is not None for item in value)
    )


def _number(value: Any) -> float | None:
    if isinstance(value, bool):
        return None
    try:
        result = float(value)
    except (TypeError, ValueError):
        return None
    return result if math.isfinite(result) else None


def _norm(vector: list[float]) -> float:
    return math.sqrt(sum(float(value) ** 2 for value in vector))


def _blocked(reason: str, **evidence: Any) -> dict[str, Any]:
    return {
        "status": STATUS_BLOCKED,
        "abort_reason": reason,
        "blocking_reasons": [reason],
        **evidence,
    }
