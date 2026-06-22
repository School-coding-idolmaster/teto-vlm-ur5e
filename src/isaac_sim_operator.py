from __future__ import annotations

import ipaddress
import json
import math
import re
import urllib.error
import urllib.request
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Protocol

from src.adaptive_reobservation_policy import (
    STABLE_SUBGOAL_EXECUTION,
    AdaptiveReobservationPolicyRequest,
    evaluate_adaptive_reobservation_policy,
)
from src.autoregressive_motion_planner import (
    AutoregressiveMotionPlannerRequest,
    plan_offline_autoregressive_motion,
)
from src.memory_guided_execution import (
    ReobservationPolicyRequest,
    build_working_memory,
    evaluate_event_triggered_reobservation,
    make_scene_monitor_result,
    update_working_memory,
)
from src.qwen_motion_parser import QwenMotionParserRequest, evaluate_qwen_motion_parser


CONTRACT_VERSION = "teto_isaac_sim_operator.v1"
EVIDENCE_SCHEMA_VERSION = "teto_isaac_sim_operator_run.v1"
GATEWAY_SIMULATED_MEASURED = "simulated_measured_gateway"
GATEWAY_SYNTHETIC_FAKE = "synthetic_fake_gateway"
DEFAULT_OUTPUT_DIR = "outputs/isaac_sim_operator_runs"
EPS = 1e-9


class IsaacOperatorSafetyError(ValueError):
    pass


class SimulatedMeasuredGateway(Protocol):
    gateway_type: str

    def status(self) -> dict[str, Any]: ...

    def execute_relative_substep(
        self,
        *,
        delta_m: list[float],
        target_tcp_pose: dict[str, Any],
        substep_index: int,
        substep_count: int,
    ) -> dict[str, Any]: ...

    def home(self) -> dict[str, Any]: ...

    def reset(self) -> dict[str, Any]: ...

    def begin_relative_motion(
        self,
        *,
        initial_tcp_pose: dict[str, Any],
        planned_subgoals: list[dict[str, Any]],
    ) -> None: ...


@dataclass(frozen=True)
class IsaacOperatorConfig:
    raw: dict[str, Any]
    path: Path | None = None

    @property
    def qwen_endpoint(self) -> str:
        value = str(self.raw.get("qwen_endpoint") or "http://127.0.0.1:18080/api/generate")
        value = value.rstrip("/")
        return value if value.endswith("/api/generate") else f"{value}/api/generate"

    @property
    def max_substep_distance_m(self) -> float:
        return _positive(self.raw.get("max_substep_distance_m"), 0.05)

    @property
    def max_total_distance_m(self) -> float:
        return _positive(self.raw.get("max_total_distance_m"), 0.50)

    @property
    def position_tolerance_m(self) -> float:
        return _positive(self.raw.get("position_tolerance_m"), 0.008)

    @property
    def scene_monitor_frequency_hz(self) -> float:
        return _positive(self.raw.get("scene_monitor_frequency_hz"), 5.0)

    @property
    def camera_monitor_unavailable_policy(self) -> str:
        value = str(self.raw.get("camera_monitor_unavailable_policy") or "warn_only")
        return value if value in {"warn_only", "block"} else "warn_only"

    @property
    def workspace_bounds(self) -> dict[str, list[float]]:
        value = self.raw.get("workspace_envelope")
        if not isinstance(value, dict):
            value = {}
        result = {"x": [-1.0, 1.0], "y": [-1.0, 1.0], "z": [0.0, 2.0]}
        for axis in ("x", "y", "z"):
            bounds = value.get(axis)
            if isinstance(bounds, (list, tuple)) and len(bounds) == 2:
                result[axis] = [float(bounds[0]), float(bounds[1])]
        return result


def load_isaac_operator_config(path: str | Path) -> IsaacOperatorConfig:
    config_path = Path(path).expanduser().resolve()
    if not config_path.is_file():
        raise IsaacOperatorSafetyError(f"E_ISAAC_CONFIG_NOT_FOUND: {config_path}")
    try:
        import yaml
    except ImportError as exc:
        raise IsaacOperatorSafetyError("E_PYYAML_REQUIRED") from exc
    payload = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    if not isinstance(payload, dict):
        raise IsaacOperatorSafetyError("E_ISAAC_CONFIG_INVALID")
    raw = payload.get("isaac_sim_operator", payload)
    if not isinstance(raw, dict):
        raise IsaacOperatorSafetyError("E_ISAAC_CONFIG_INVALID")
    validate_no_real_robot_config(raw)
    return IsaacOperatorConfig(raw=raw, path=config_path)


def validate_no_real_robot_args(argv: list[str]) -> None:
    for arg in argv:
        if arg == "--real" or arg.startswith("--real="):
            raise IsaacOperatorSafetyError("E_REAL_FLAG_FORBIDDEN_IN_ISAAC_OPERATOR")


def validate_no_real_robot_config(config: dict[str, Any]) -> None:
    required_false = {
        "real_robot_enabled",
        "allow_real_robot",
        "allow_moveit_execute",
        "dashboard_enabled",
        "rtde_write_enabled",
    }
    for key in required_false:
        if config.get(key) not in {None, False}:
            raise IsaacOperatorSafetyError(f"E_REAL_CAPABILITY_MUST_BE_DISABLED: {key}")
    if config.get("real_robot_disabled") is not True:
        raise IsaacOperatorSafetyError("E_REAL_ROBOT_DISABLED_MUST_BE_TRUE")
    if config.get("gui_required") is not True:
        raise IsaacOperatorSafetyError("E_ISAAC_GUI_REQUIRED_MUST_BE_TRUE")
    for key_path, value in _walk_scalars(config):
        if _is_real_robot_ip(value) and not _documentation_only_key(key_path):
            raise IsaacOperatorSafetyError(f"E_REAL_UR_IP_FORBIDDEN: {key_path}")


def qwen_health(endpoint: str, timeout_s: float = 2.0) -> dict[str, Any]:
    generate_suffix = "/api/generate"
    base = endpoint[:-len(generate_suffix)] if endpoint.endswith(generate_suffix) else endpoint.rstrip("/")
    url = f"{base}/health"
    try:
        with urllib.request.urlopen(url, timeout=timeout_s) as response:
            payload = json.loads(response.read().decode("utf-8"))
        return {"ok": payload.get("status") == "ok", "url": url, "payload": payload}
    except (OSError, ValueError, urllib.error.URLError) as exc:
        return {"ok": False, "url": url, "error": str(exc)}


class IsaacSimOperator:
    def __init__(
        self,
        *,
        config: IsaacOperatorConfig,
        gateway: SimulatedMeasuredGateway,
        headless: bool = False,
        output_dir: str | Path = DEFAULT_OUTPUT_DIR,
        qwen_callable=None,
        scene_monitor_callable=None,
    ) -> None:
        self.config = config
        self.gateway = gateway
        self.headless = bool(headless)
        self.output_dir = Path(output_dir)
        self.qwen_callable = qwen_callable
        self.scene_monitor_callable = scene_monitor_callable
        self.last_result: dict[str, Any] | None = None
        self.last_evidence_path: str | None = None
        if gateway.gateway_type not in {GATEWAY_SIMULATED_MEASURED, GATEWAY_SYNTHETIC_FAKE}:
            raise IsaacOperatorSafetyError("E_UNKNOWN_GATEWAY_TYPE")
        if not self.headless and gateway.gateway_type != GATEWAY_SIMULATED_MEASURED:
            raise IsaacOperatorSafetyError("E_GUI_DEMO_REQUIRES_SIMULATED_MEASURED_GATEWAY")

    def status(self) -> dict[str, Any]:
        gateway_status = self.gateway.status()
        return {
            "contract_version": CONTRACT_VERSION,
            "execution_mode": "isaac_sim",
            "runtime_mode": "headless_smoke_test" if self.headless else "gui",
            "gateway_type": self.gateway.gateway_type,
            "isaac_connection_status": gateway_status.get("connection_status", "UNKNOWN"),
            "qwen_health": qwen_health(self.config.qwen_endpoint),
            "current_simulated_tcp_pose": gateway_status.get("current_tcp_pose"),
            "current_simulated_joint_state": gateway_status.get("joint_state"),
            "isaac_initial_home_pose_applied": gateway_status.get(
                "isaac_initial_home_pose_applied",
                bool(self.config.raw.get("apply_initial_home_pose", True)),
            ),
            "isaac_initial_home_pose_source": gateway_status.get(
                "isaac_initial_home_pose_source",
                "isaac_config_pending_gateway_confirmation",
            ),
            "isaac_initial_home_joint_names": gateway_status.get("isaac_initial_home_joint_names"),
            "isaac_initial_home_joint_positions_rad": gateway_status.get(
                "isaac_initial_home_joint_positions_rad"
            ),
            "isaac_visual_markers_enabled": gateway_status.get(
                "isaac_visual_markers_enabled",
                bool(self.config.raw.get("visual_markers_enabled", True)) and not self.headless,
            ),
            "isaac_trajectory_trace_enabled": gateway_status.get(
                "isaac_trajectory_trace_enabled",
                bool(self.config.raw.get("trajectory_trace_enabled", True)) and not self.headless,
            ),
            "isaac_visual_timing": gateway_status.get("isaac_visual_timing")
            or _configured_visual_timing(self.config.raw, headless=self.headless),
            "last_command_result": self.last_result,
            "last_evidence_path": self.last_evidence_path,
            **_safety_flags(),
        }

    def execute_text(self, text: str) -> dict[str, Any]:
        text = str(text or "").strip()
        if not text:
            return {"status": "BLOCKED", "abort_reason": "E_EMPTY_COMMAND", **_safety_flags()}
        before_status = self.gateway.status()
        current_pose = _pose(before_status.get("current_tcp_pose"))
        parser_result = (
            _parse_isaac_demo_motion(
                text,
                max_distance_m=self.config.max_total_distance_m,
            )
            if self.qwen_callable is None
            else None
        )
        if parser_result is None:
            parser_result = evaluate_qwen_motion_parser(
                QwenMotionParserRequest(
                    user_text=text,
                    max_distance_m=self.config.max_total_distance_m,
                    hard_safety_limit_m=self.config.max_total_distance_m,
                    endpoint=self.config.qwen_endpoint,
                    llm_callable=self.qwen_callable,
                )
            )
        evidence = self._base_evidence(text, parser_result, before_status)
        if parser_result.get("qwen_motion_parser_status") != "PASS":
            evidence["status"] = "BLOCKED"
            evidence["abort_reason"] = (
                (parser_result.get("blocking_reasons") or ["E_QWEN_PARSE_BLOCKED"])[0]
            )
            return self._finish(evidence)
        delta = _vector(parser_result.get("delta_m"))
        if delta is None or _norm(delta) <= EPS:
            evidence["status"] = "BLOCKED"
            evidence["abort_reason"] = "E_INVALID_ZERO_MOTION"
            return self._finish(evidence)
        if current_pose is None:
            evidence["status"] = "BLOCKED"
            evidence["abort_reason"] = "E_SIMULATED_TCP_POSE_UNAVAILABLE"
            return self._finish(evidence)

        step_deltas, plan = self._decompose(parser_result, current_pose, delta)
        evidence["autoregressive_plan"] = plan
        evidence["delta_vector_m"] = delta
        evidence["motion_contract_type"] = (
            "long_range_approach"
            if _norm(delta) > self.config.max_substep_distance_m + EPS
            else parser_result.get("motion_contract_type") or "single_axis_relative"
        )
        evidence["requested_total_distance_m"] = round(
            float(parser_result.get("requested_distance_m") or _norm(delta)),
            6,
        )
        evidence["requested_distance_norm_m"] = round(_norm(delta), 6)
        evidence["requested_vector_m"] = delta
        evidence["normalized_direction_vector"] = _normalized(delta)
        evidence["max_substep_m"] = self.config.max_substep_distance_m
        evidence["substep_count"] = len(step_deltas)
        evidence["subgoal_count"] = len(step_deltas)
        evidence["planned_subgoals"] = plan.get("planned_subgoals", [])
        evidence["workspace_check_status"] = plan.get("workspace_check_status", "BLOCKED")
        working_memory = build_working_memory(
            task_goal=text,
            goal_type="relative_motion",
            target_delta_m=delta,
            latest_verified_tcp_m=current_pose["position_m"],
            motion_mode=str(
                parser_result.get("motion_contract_type") or "single_axis_relative"
            ),
        )
        evidence["working_memory_before"] = json.loads(json.dumps(working_memory))
        evidence["working_memory_after"] = json.loads(json.dumps(working_memory))
        if not step_deltas:
            evidence["status"] = "BLOCKED"
            evidence["abort_reason"] = plan.get("final_blocking_reason") or "E_DECOMPOSITION_FAILED"
            return self._finish(evidence)

        begin_relative_motion = getattr(self.gateway, "begin_relative_motion", None)
        if callable(begin_relative_motion):
            begin_relative_motion(
                initial_tcp_pose=current_pose,
                planned_subgoals=evidence["planned_subgoals"],
            )

        latest = current_pose
        completed = 0
        stable_substep_count = 0
        subgoal_failure_count = 0
        for index, step_delta in enumerate(step_deltas, start=1):
            before = _pose(self.gateway.status().get("current_tcp_pose"))
            if before is None:
                evidence["abort_reason"] = "E_SIMULATED_TCP_POSE_UNAVAILABLE"
                break
            target = {
                "frame": before["frame"],
                "position_m": _add(before["position_m"], step_delta),
                "orientation_xyzw": list(before["orientation_xyzw"]),
            }
            if not _in_workspace(target["position_m"], self.config.workspace_bounds):
                evidence["workspace_check_status"] = "BLOCKED"
                evidence["abort_reason"] = "E_OUT_OF_SIM_WORKSPACE"
                break
            result = self.gateway.execute_relative_substep(
                delta_m=step_delta,
                target_tcp_pose=target,
                substep_index=index,
                substep_count=len(step_deltas),
            )
            after = _pose(result.get("measured_tcp_after"))
            error_m = _distance(after["position_m"], target["position_m"]) if after else None
            direction_ok = (
                after is not None
                and _dot(
                    [after["position_m"][i] - before["position_m"][i] for i in range(3)],
                    step_delta,
                )
                > EPS
            )
            verified = bool(
                result.get("execution_status") == "PASS"
                and after is not None
                and error_m is not None
                and error_m <= self.config.position_tolerance_m + EPS
                and direction_ok
            )
            subgoal_failure_count = 0 if verified else subgoal_failure_count + 1
            scene_monitor_result = self._run_scene_monitor(
                {
                    "task_goal": text,
                    "goal_type": "relative_motion",
                    "substep_index": index,
                    "substep_count": len(step_deltas),
                    "target_tcp_pose": target,
                    "measured_tcp_before": before,
                    "measured_tcp_after": after,
                }
            )
            policy_result = evaluate_event_triggered_reobservation(
                ReobservationPolicyRequest(
                    working_memory=working_memory,
                    latest_measured_tcp_m=after["position_m"] if after else None,
                    subgoal_target_tcp_m=target["position_m"],
                    position_error_m=error_m,
                    position_error_limit_m=self.config.position_tolerance_m,
                    direction_check_passed=direction_ok,
                    scene_monitor_result=scene_monitor_result,
                    subgoal_failed=not verified,
                    subgoal_failure_count=subgoal_failure_count,
                    camera_unavailable_policy=self.config.camera_monitor_unavailable_policy,
                )
            )
            adaptive_policy_result = evaluate_adaptive_reobservation_policy(
                AdaptiveReobservationPolicyRequest(
                    working_memory=working_memory,
                    current_execution_phase=STABLE_SUBGOAL_EXECUTION,
                    subgoal_index=index,
                    position_error_m=error_m,
                    position_error_limit_m=self.config.position_tolerance_m,
                    direction_check_passed=direction_ok,
                    subgoal_failure_count=subgoal_failure_count,
                    scene_monitor_result=scene_monitor_result,
                    target_task_type="relative_motion",
                    camera_monitor_available=scene_monitor_result.get(
                        "camera_check_status"
                    )
                    not in {"NOT_AVAILABLE", "FAIL"},
                    stable_substep_count=stable_substep_count,
                    subgoal_failed=not verified,
                    config={
                        "camera_unavailable_policy": (
                            self.config.camera_monitor_unavailable_policy
                        ),
                    },
                )
            )
            scene_monitor_result["frequency_mode"] = adaptive_policy_result[
                "camera_monitor_frequency_mode"
            ]
            continue_allowed = bool(
                verified
                and policy_result["continue_allowed"] is True
                and adaptive_policy_result["reobserve_required"] is False
                and adaptive_policy_result["abort_required"] is False
            )
            evidence["substeps"].append(
                {
                    "substep_index": index,
                    "substep_count": len(step_deltas),
                    "delta_m": step_delta,
                    "target_tcp_pose": target,
                    "simulated_measured_tcp_before": before,
                    "simulated_measured_tcp_after": after,
                    "simulated_joint_state_after": result.get("joint_state_after"),
                    "position_error_m": round(error_m, 6) if error_m is not None else None,
                    "position_tolerance_m": self.config.position_tolerance_m,
                    "direction_check_passed": direction_ok,
                    "verification_result": "PASS" if verified else "FAILED",
                    "camera_check_status": scene_monitor_result.get("camera_check_status"),
                    "monitor_frequency_hz": scene_monitor_result.get("monitor_frequency_hz"),
                    "scene_monitor_result": scene_monitor_result,
                    "reobservation_policy_result": policy_result,
                    "adaptive_reobservation_policy_result": adaptive_policy_result,
                    "execution_load_mode": adaptive_policy_result["execution_load_mode"],
                    "llm_call_policy": adaptive_policy_result["llm_call_policy"],
                    "vlm_call_policy": adaptive_policy_result["vlm_call_policy"],
                    "camera_monitor_frequency_mode": adaptive_policy_result[
                        "camera_monitor_frequency_mode"
                    ],
                    "load_reduction_active": adaptive_policy_result[
                        "load_reduction_active"
                    ],
                    "reobserve_triggered": adaptive_policy_result["reobserve_required"],
                    "reobserve_reason": adaptive_policy_result["reobserve_reason"],
                    "replan_required": adaptive_policy_result["replan_required"],
                    "vlm_reobserve_called": False,
                    "llm_reobserve_called": False,
                    "continue_allowed": continue_allowed,
                    "gateway_result": result,
                    "isaac_visual_timing": result.get("isaac_visual_timing"),
                    "isaac_visual_markers_enabled": result.get("isaac_visual_markers_enabled"),
                    "isaac_trajectory_trace_enabled": result.get("isaac_trajectory_trace_enabled"),
                }
            )
            evidence["measured_subgoals"].append(
                {
                    "subgoal_index": index,
                    "planned_pose": target,
                    "measured_pose": after,
                    "position_error_m": round(error_m, 6) if error_m is not None else None,
                    "verification_result": "PASS" if verified else "FAILED",
                }
            )
            if verified:
                completed += 1
                latest = after
            stable_substep_count = stable_substep_count + 1 if continue_allowed else 0
            measured_total_delta = (
                [
                    round(after["position_m"][component] - current_pose["position_m"][component], 6)
                    for component in range(3)
                ]
                if after is not None
                else None
            )
            working_memory = update_working_memory(
                working_memory,
                latest_verified_tcp_m=after["position_m"] if verified and after else latest["position_m"],
                measured_total_delta_m=measured_total_delta,
                completed_substeps=completed,
                last_error_m=error_m,
                scene_monitor_result=scene_monitor_result,
                reobservation_policy_result=policy_result,
                adaptive_policy_result=adaptive_policy_result,
                stable_substep_count=stable_substep_count,
                last_direction_check_passed=direction_ok,
            )
            evidence["working_memory_after"] = json.loads(json.dumps(working_memory))
            evidence["scene_monitor_result"] = scene_monitor_result
            evidence["reobservation_policy_result"] = policy_result
            evidence["adaptive_reobservation_policy_result"] = adaptive_policy_result
            evidence["execution_load_mode"] = adaptive_policy_result["execution_load_mode"]
            evidence["llm_call_policy"] = adaptive_policy_result["llm_call_policy"]
            evidence["vlm_call_policy"] = adaptive_policy_result["vlm_call_policy"]
            evidence["camera_monitor_frequency_mode"] = adaptive_policy_result[
                "camera_monitor_frequency_mode"
            ]
            evidence["load_reduction_active"] = adaptive_policy_result[
                "load_reduction_active"
            ]
            if adaptive_policy_result["reobserve_required"] is True:
                evidence["reobserve_triggered"] = True
                evidence["reobserve_reason"] = adaptive_policy_result["reobserve_reason"]
                evidence["replan_required"] = adaptive_policy_result["replan_required"]
            if not continue_allowed:
                evidence["abort_reason"] = (
                    result.get("abort_reason")
                    or adaptive_policy_result.get("reobserve_reason")
                    or (
                        "E_SIMULATED_POST_STEP_DIRECTION_MISMATCH"
                        if not direction_ok
                        else "E_SIMULATED_POST_STEP_VERIFICATION_FAILED"
                    )
                )
                break

        evidence["completed_substep_count"] = completed
        evidence["simulated_robot_motion_executed"] = completed > 0
        evidence["final_simulated_tcp_pose"] = latest
        final_status = self.gateway.status()
        final_pose = _pose(final_status.get("current_tcp_pose")) or latest
        initial_position = current_pose["position_m"]
        final_position = final_pose["position_m"]
        measured_delta = [
            round(float(final_position[index]) - float(initial_position[index]), 6)
            for index in range(3)
        ]
        target_final = (
            evidence["substeps"][-1].get("target_tcp_pose")
            if evidence["substeps"]
            else None
        )
        final_error = (
            _distance(final_position, target_final["position_m"])
            if target_final is not None
            else None
        )
        evidence["final_simulated_tcp_pose"] = final_pose
        evidence["final_simulated_joint_state"] = final_status.get("joint_state")
        evidence["target_final_tcp_pose"] = target_final
        evidence["measured_delta_vector_m"] = measured_delta
        evidence["final_displacement_m"] = round(_norm(measured_delta), 6)
        evidence["final_position_error_m"] = round(final_error, 6) if final_error is not None else None
        evidence["direction_check_passed"] = bool(
            completed > 0
            and _dot(measured_delta, delta) > EPS
            and all(step.get("direction_check_passed") for step in evidence["substeps"])
        )
        evidence["joint_delta_summary"] = _joint_delta_summary(
            before_status.get("joint_state"),
            final_status.get("joint_state"),
        )
        evidence["isaac_initial_home_pose_applied"] = final_status.get(
            "isaac_initial_home_pose_applied",
            evidence.get("isaac_initial_home_pose_applied"),
        )
        evidence["isaac_initial_home_pose_source"] = final_status.get(
            "isaac_initial_home_pose_source",
            evidence.get("isaac_initial_home_pose_source"),
        )
        evidence["isaac_initial_home_joint_names"] = final_status.get(
            "isaac_initial_home_joint_names"
        )
        evidence["isaac_initial_home_joint_positions_rad"] = final_status.get(
            "isaac_initial_home_joint_positions_rad"
        )
        evidence["isaac_visual_markers_enabled"] = final_status.get(
            "isaac_visual_markers_enabled",
            evidence.get("isaac_visual_markers_enabled"),
        )
        evidence["isaac_trajectory_trace_enabled"] = final_status.get(
            "isaac_trajectory_trace_enabled",
            evidence.get("isaac_trajectory_trace_enabled"),
        )
        evidence["isaac_visual_timing"] = final_status.get(
            "isaac_visual_timing"
        ) or evidence.get("isaac_visual_timing")
        evidence["visible_motion_hint"] = (
            "cyan=current TCP, yellow=final target, blue=full planned path, green=measured path"
            if evidence["isaac_visual_markers_enabled"]
            else "visual markers disabled; compare requested/measured delta in this summary"
        )
        evidence["status"] = (
            "PASS"
            if completed == len(step_deltas) and not evidence["reobserve_triggered"]
            else "ABORTED"
            if (evidence.get("adaptive_reobservation_policy_result") or {}).get(
                "abort_required"
            )
            is True
            else "REOBSERVE_REQUIRED"
            if evidence["reobserve_triggered"]
            else "ABORTED"
        )
        return self._finish(evidence)

    def home(self) -> dict[str, Any]:
        return self.gateway.home()

    def reset(self) -> dict[str, Any]:
        return self.gateway.reset()

    def demo_center(self) -> dict[str, Any]:
        result = self.gateway.home()
        return {**result, "operator_command": "demo_center"}

    def _decompose(
        self,
        parser_result: dict[str, Any],
        current_pose: dict[str, Any],
        delta: list[float],
    ) -> tuple[list[list[float]], dict[str, Any]]:
        norm = _norm(delta)
        final_position = _add(current_pose["position_m"], delta)
        if norm > self.config.max_total_distance_m + EPS:
            return [], {
                "final_plan_status": "BLOCKED",
                "final_blocking_reason": "E_ISAAC_LONG_RANGE_LIMIT",
                "workspace_check_status": "NOT_RUN",
                "planned_subgoals": [],
            }
        if not _in_workspace(final_position, self.config.workspace_bounds):
            return [], {
                "final_plan_status": "BLOCKED",
                "final_blocking_reason": "E_OUT_OF_SIM_WORKSPACE",
                "workspace_check_status": "BLOCKED",
                "planned_subgoals": [],
            }
        if (
            parser_result.get("parser_source") != "isaac_sim_demo_local"
            and norm > self.config.max_substep_distance_m + EPS
        ):
            canonical = {
                "parse_status": "PASS",
                "intent": "relative_cartesian_motion",
                "motion_frame": parser_result.get("motion_frame") or "base_link",
                "requested_distance_m": norm,
                "requested_distance_norm_m": norm,
                "delta_m": delta,
                "vector_delta_m": delta,
                "motion_contract_type": parser_result.get("motion_contract_type"),
                "direction_axis": parser_result.get("direction_axis"),
                "direction_sign": parser_result.get("direction_sign"),
            }
            plan = plan_offline_autoregressive_motion(
                AutoregressiveMotionPlannerRequest(
                    canonical_motion_intent=canonical,
                    current_tcp_pose=current_pose,
                    config={
                        "enable_long_step_decomposition": True,
                        "max_one_shot_distance_m": self.config.max_substep_distance_m,
                        "max_decomposed_substep_distance_m": self.config.max_substep_distance_m,
                        "max_decomposed_total_distance_m": self.config.max_total_distance_m,
                        "substep_execution_mode": "offline_preview",
                        "workspace_bounds": self.config.workspace_bounds,
                    },
                )
            )
            if plan.get("final_plan_status") != "PASS":
                return [], plan
            vectors = [
                vector
                for vector in (
                    _vector(item)
                    for item in (
                        plan.get("decomposed_substeps_m")
                        or plan.get("planned_substep_vectors_m")
                        or []
                    )
                )
                if vector is not None
            ]
            plan["planned_subgoals"] = _planned_subgoals(current_pose, vectors)
            plan["workspace_check_status"] = "PASS"
            return vectors, plan
        if norm <= self.config.max_substep_distance_m + EPS:
            target = _target_pose(current_pose, final_position)
            return [delta], {
                "final_plan_status": "PASS",
                "planned_execution_style": "isaac_sim_one_substep",
                "substep_count": 1,
                "decomposed_substeps_m": [delta],
                "workspace_check_status": "PASS",
                "planned_subgoals": [{"subgoal_index": 1, "planned_pose": target}],
            }
        count = max(int(math.ceil(norm / self.config.max_substep_distance_m)), 1)
        regular_step = [round(component / count, 6) for component in delta]
        vectors = [list(regular_step) for _ in range(count - 1)]
        vectors.append(
            [
                round(delta[axis] - sum(vector[axis] for vector in vectors), 6)
                for axis in range(3)
            ]
        )
        planned_subgoals = _planned_subgoals(current_pose, vectors)
        return vectors, {
            "final_plan_status": "PASS",
            "planned_execution_style": "isaac_sim_long_range_approach",
            "motion_contract_type": "long_range_approach",
            "safety_gate_scope": "isaac_sim_decomposed_contract",
            "requested_total_distance_m": round(norm, 6),
            "max_substep_m": self.config.max_substep_distance_m,
            "substep_count": count,
            "decomposed_substeps_m": vectors,
            "planned_subgoals": planned_subgoals,
            "workspace_check_status": "PASS",
        }

    def _base_evidence(
        self,
        text: str,
        parser_result: dict[str, Any],
        before_status: dict[str, Any],
    ) -> dict[str, Any]:
        return {
            "schema_version": EVIDENCE_SCHEMA_VERSION,
            "generated_at": datetime.now(timezone.utc).isoformat(),
            "status": "BLOCKED",
            "execution_mode": "isaac_sim",
            "runtime_mode": "headless_smoke_test" if self.headless else "gui",
            "input_text": text,
            "qwen_raw_response": parser_result.get("raw_llm_output"),
            "parsed_motion": parser_result,
            "gateway_type": self.gateway.gateway_type,
            "initial_simulated_tcp_pose": before_status.get("current_tcp_pose"),
            "initial_simulated_joint_state": before_status.get("joint_state"),
            "delta_vector_m": None,
            "motion_contract_type": None,
            "requested_total_distance_m": None,
            "requested_vector_m": None,
            "normalized_direction_vector": None,
            "requested_distance_norm_m": None,
            "max_substep_m": self.config.max_substep_distance_m,
            "substep_count": 0,
            "subgoal_count": 0,
            "completed_substep_count": 0,
            "planned_subgoals": [],
            "measured_subgoals": [],
            "substeps": [],
            "final_displacement_m": 0.0,
            "workspace_check_status": "NOT_RUN",
            "working_memory_before": None,
            "working_memory_after": None,
            "scene_monitor_result": make_scene_monitor_result(
                monitor_frequency_hz=self.config.scene_monitor_frequency_hz
            ),
            "reobservation_policy_result": None,
            "adaptive_reobservation_policy_result": None,
            "execution_load_mode": "full_observation",
            "llm_call_policy": "initial_only",
            "vlm_call_policy": "initial_only",
            "camera_monitor_frequency_mode": "unavailable",
            "load_reduction_active": False,
            "reobserve_triggered": False,
            "reobserve_reason": None,
            "replan_required": False,
            "vlm_reobserve_called": False,
            "llm_reobserve_called": False,
            "monitor_frequency_hz": self.config.scene_monitor_frequency_hz,
            "abort_reason": None,
            "final_simulated_tcp_pose": before_status.get("current_tcp_pose"),
            "final_simulated_joint_state": before_status.get("joint_state"),
            "target_final_tcp_pose": None,
            "measured_delta_vector_m": None,
            "final_position_error_m": None,
            "direction_check_passed": False,
            "joint_delta_summary": [],
            "isaac_initial_home_pose_applied": before_status.get("isaac_initial_home_pose_applied"),
            "isaac_initial_home_pose_source": before_status.get("isaac_initial_home_pose_source"),
            "isaac_initial_home_joint_names": before_status.get("isaac_initial_home_joint_names"),
            "isaac_initial_home_joint_positions_rad": before_status.get(
                "isaac_initial_home_joint_positions_rad"
            ),
            "isaac_visual_markers_enabled": before_status.get("isaac_visual_markers_enabled"),
            "isaac_trajectory_trace_enabled": before_status.get("isaac_trajectory_trace_enabled"),
            "isaac_visual_timing": before_status.get("isaac_visual_timing")
            or _configured_visual_timing(self.config.raw, headless=self.headless),
            "visible_motion_hint": None,
            "gui_mode_confirmation": {
                "isaac_gui_required": not self.headless,
                "headless_smoke_test": self.headless,
                "viewport_expected_visible": not self.headless,
            },
            "simulated_robot_motion_executed": False,
            "artifact_paths": {},
            **_safety_flags(),
        }

    def _run_scene_monitor(self, context: dict[str, Any]) -> dict[str, Any]:
        if self.scene_monitor_callable is None:
            return make_scene_monitor_result(
                monitor_frequency_hz=self.config.scene_monitor_frequency_hz
            )
        try:
            value = self.scene_monitor_callable(context)
        except Exception as exc:
            value = {
                "monitor_type": "mock",
                "camera_check_status": "NOT_AVAILABLE",
                "scene_freshness_status": "unknown",
                "monitor_error": str(exc),
            }
        return make_scene_monitor_result(
            value,
            monitor_frequency_hz=self.config.scene_monitor_frequency_hz,
        )

    def _finish(self, evidence: dict[str, Any]) -> dict[str, Any]:
        json_path, markdown_path = write_isaac_operator_evidence(evidence, self.output_dir)
        evidence["artifact_paths"] = {
            "json": str(json_path),
            "markdown": str(markdown_path),
        }
        json_path.write_text(json.dumps(evidence, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
        self.last_result = evidence
        self.last_evidence_path = str(json_path)
        return evidence


class SyntheticFakeGateway:
    gateway_type = GATEWAY_SYNTHETIC_FAKE

    def __init__(self, pose: dict[str, Any] | None = None) -> None:
        self.pose = pose or {
            "frame": "base_link",
            "position_m": [0.4, 0.0, 0.4],
            "orientation_xyzw": [0.0, 0.0, 0.0, 1.0],
        }
        self.home_pose = json.loads(json.dumps(self.pose))
        self.joints = [0.0] * 6

    def status(self) -> dict[str, Any]:
        return {
            "connection_status": "SYNTHETIC_FAKE_TEST_ONLY",
            "current_tcp_pose": json.loads(json.dumps(self.pose)),
            "joint_state": {"names": [f"joint_{i}" for i in range(6)], "positions_rad": list(self.joints)},
        }

    def execute_relative_substep(self, *, delta_m, target_tcp_pose, substep_index, substep_count):
        before = json.loads(json.dumps(self.pose))
        self.pose = json.loads(json.dumps(target_tcp_pose))
        return {
            "execution_status": "PASS",
            "measured_tcp_before": before,
            "measured_tcp_after": json.loads(json.dumps(self.pose)),
            "joint_state_after": self.status()["joint_state"],
            "synthetic_fake_test_only": True,
        }

    def begin_relative_motion(self, *, initial_tcp_pose, planned_subgoals) -> None:
        return None

    def home(self) -> dict[str, Any]:
        self.pose = json.loads(json.dumps(self.home_pose))
        return {"status": "PASS", "simulated_only": True}

    def reset(self) -> dict[str, Any]:
        return self.home()


def write_isaac_operator_evidence(evidence: dict[str, Any], output_dir: str | Path) -> tuple[Path, Path]:
    directory = Path(output_dir)
    directory.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S%fZ")
    json_path = directory / f"{stamp}_isaac_sim_operator_run.json"
    markdown_path = directory / f"{stamp}_isaac_sim_operator_run.md"
    json_path.write_text(json.dumps(evidence, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    markdown_path.write_text(
        "\n".join(
            [
                "# TETO Isaac Sim Operator Run",
                "",
                "This artifact is Isaac/simulation evidence, not real-lab validation.",
                "",
                f"- Status: `{evidence.get('status')}`",
                f"- Execution mode: `{evidence.get('execution_mode')}`",
                f"- Gateway: `{evidence.get('gateway_type')}`",
                f"- Input: `{evidence.get('input_text')}`",
                f"- Requested norm: `{evidence.get('requested_distance_norm_m')}` m",
                f"- Substeps: `{evidence.get('completed_substep_count')}/{evidence.get('substep_count')}`",
                f"- Abort reason: `{evidence.get('abort_reason')}`",
                "- Real robot motion executed: `false`",
                "- Real UR connection used: `false`",
                "- Dashboard used: `false`",
                "- RTDE write used: `false`",
                "",
                "```json",
                json.dumps(evidence, indent=2, ensure_ascii=False),
                "```",
                "",
            ]
        ),
        encoding="utf-8",
    )
    return json_path, markdown_path


def _safety_flags() -> dict[str, bool]:
    return {
        "isaac_gui_required": True,
        "real_robot_motion_executed": False,
        "real_ur_connection_used": False,
        "dashboard_used": False,
        "rtde_write_used": False,
        "moveit_execute_trajectory_called": False,
        "trajectory_sent": False,
    }


_ISAAC_DEMO_COMMAND = re.compile(
    r"^move\s+(forward|backward|left|right|up|down)"
    r"(?:\s+and\s+(forward|backward|left|right|up|down))?"
    r"\s+([0-9]+(?:\.[0-9]+)?)\s*(?:m|meter|meters)$",
    re.IGNORECASE,
)
_ISAAC_DIRECTION_VECTORS = {
    "forward": [1.0, 0.0, 0.0],
    "backward": [-1.0, 0.0, 0.0],
    "left": [0.0, 1.0, 0.0],
    "right": [0.0, -1.0, 0.0],
    "up": [0.0, 0.0, 1.0],
    "down": [0.0, 0.0, -1.0],
}


def _parse_isaac_demo_motion(text: str, *, max_distance_m: float) -> dict[str, Any] | None:
    match = _ISAAC_DEMO_COMMAND.fullmatch(text.strip())
    if match is None:
        return None
    first, second, raw_distance = match.groups()
    distance = float(raw_distance)
    names = [first.lower(), second.lower() if second else None]
    vectors = [_ISAAC_DIRECTION_VECTORS[name] for name in names if name]
    combined = [sum(vector[axis] for vector in vectors) for axis in range(3)]
    combined_norm = _norm(combined)
    if combined_norm <= EPS:
        blocking_reasons = ["E_CONFLICTING_DIRECTION"]
        delta = None
    else:
        unit = [component / combined_norm for component in combined]
        delta = [round(component * distance, 6) for component in unit]
        blocking_reasons = (
            ["E_ISAAC_LONG_RANGE_LIMIT"]
            if distance > float(max_distance_m) + EPS
            else []
        )
    return {
        "contract_version": "teto_isaac_demo_parser.v1",
        "qwen_motion_parser_status": "BLOCKED" if blocking_reasons else "PASS",
        "parser_source": "isaac_sim_demo_local",
        "llm_called": False,
        "raw_llm_output": None,
        "blocking_reasons": blocking_reasons,
        "parser_blocking_reasons": blocking_reasons,
        "delta_m": delta,
        "distance_m": round(distance, 6),
        "requested_distance_m": round(distance, 6),
        "requested_distance_norm_m": round(distance, 6),
        "motion_contract_type": "decomposed_relative_motion",
        "motion_frame": "base_link",
        "normalized_contract": {
            "intent": "relative_cartesian_motion",
            "frame": "base_link",
            "delta_m": delta,
            "requested_distance_m": round(distance, 6),
            "requested_distance_norm_m": round(distance, 6),
            "motion_contract_type": "decomposed_relative_motion",
            "isaac_sim_only": True,
        }
        if delta is not None
        else None,
    }


def _normalized(vector: list[float]) -> list[float]:
    length = _norm(vector)
    if length <= EPS:
        return [0.0, 0.0, 0.0]
    return [round(component / length, 6) for component in vector]


def _target_pose(source_pose: dict[str, Any], position: list[float]) -> dict[str, Any]:
    return {
        "frame": source_pose["frame"],
        "position_m": [round(float(value), 6) for value in position],
        "orientation_xyzw": list(source_pose["orientation_xyzw"]),
    }


def _planned_subgoals(
    current_pose: dict[str, Any],
    vectors: list[list[float]],
) -> list[dict[str, Any]]:
    planned = []
    accumulated = list(current_pose["position_m"])
    for index, vector in enumerate(vectors, start=1):
        accumulated = _add(accumulated, vector)
        planned.append(
            {
                "subgoal_index": index,
                "planned_pose": _target_pose(current_pose, accumulated),
            }
        )
    return planned


def _joint_delta_summary(before: Any, after: Any, *, threshold_rad: float = 1e-5) -> list[dict[str, Any]]:
    if not isinstance(before, dict) or not isinstance(after, dict):
        return []
    before_names = before.get("names")
    after_names = after.get("names")
    before_positions = before.get("positions_rad")
    after_positions = after.get("positions_rad")
    if not all(isinstance(value, list) for value in (before_names, after_names, before_positions, after_positions)):
        return []
    before_map = {
        str(name): float(position)
        for name, position in zip(before_names, before_positions)
    }
    summary = []
    for name, position in zip(after_names, after_positions):
        name_text = str(name)
        if name_text not in before_map:
            continue
        delta = float(position) - before_map[name_text]
        if abs(delta) <= threshold_rad:
            continue
        summary.append(
            {
                "joint": name_text,
                "before_rad": round(before_map[name_text], 6),
                "after_rad": round(float(position), 6),
                "delta_rad": round(delta, 6),
            }
        )
    return summary


def _configured_visual_timing(config: dict[str, Any], *, headless: bool) -> dict[str, Any]:
    return {
        "visual_demo_slowdown_enabled": bool(
            config.get("visual_demo_slowdown_enabled", True)
        )
        and not headless,
        "motion_duration_sec": float(config.get("motion_duration_sec") or 2.4),
        "substep_pause_sec": float(config.get("substep_pause_sec") or 0.25),
        "visual_demo_fps": float(config.get("visual_demo_fps") or 60.0),
        "scope": "isaac_sim_only",
    }


def _walk_scalars(value: Any, prefix: str = ""):
    if isinstance(value, dict):
        for key, child in value.items():
            key_path = f"{prefix}.{key}" if prefix else str(key)
            yield from _walk_scalars(child, key_path)
    elif isinstance(value, list):
        for index, child in enumerate(value):
            yield from _walk_scalars(child, f"{prefix}[{index}]")
    else:
        yield prefix, value


def _documentation_only_key(key_path: str) -> bool:
    lowered = key_path.lower()
    return any(token in lowered for token in ("documentation", "example_text", "comment", "note"))


def _is_real_robot_ip(value: Any) -> bool:
    if not isinstance(value, str):
        return False
    try:
        ip = ipaddress.ip_address(value.strip())
    except ValueError:
        return False
    return ip.is_private and not ip.is_loopback


def _pose(value: Any) -> dict[str, Any] | None:
    if not isinstance(value, dict):
        return None
    position = _vector(value.get("position_m"))
    orientation = value.get("orientation_xyzw")
    if position is None or not isinstance(orientation, (list, tuple)) or len(orientation) != 4:
        return None
    return {
        "frame": str(value.get("frame") or "base_link"),
        "position_m": position,
        "orientation_xyzw": [float(item) for item in orientation],
    }


def _vector(value: Any) -> list[float] | None:
    if isinstance(value, dict):
        value = [value.get("x"), value.get("y"), value.get("z")]
    if not isinstance(value, (list, tuple)) or len(value) != 3:
        return None
    try:
        result = [float(item) for item in value]
    except (TypeError, ValueError):
        return None
    return result if all(math.isfinite(item) for item in result) else None


def _positive(value: Any, default: float) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError):
        return default
    return result if math.isfinite(result) and result > 0.0 else default


def _norm(vector: list[float]) -> float:
    return math.sqrt(sum(item * item for item in vector))


def _distance(left: list[float], right: list[float]) -> float:
    return _norm([left[index] - right[index] for index in range(3)])


def _add(left: list[float], right: list[float]) -> list[float]:
    return [round(left[index] + right[index], 6) for index in range(3)]


def _dot(left: list[float], right: list[float]) -> float:
    return sum(left[index] * right[index] for index in range(3))


def _in_workspace(point: list[float], bounds: dict[str, list[float]]) -> bool:
    return all(bounds[axis][0] - EPS <= point[index] <= bounds[axis][1] + EPS for index, axis in enumerate(("x", "y", "z")))
