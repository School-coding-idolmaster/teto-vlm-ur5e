from __future__ import annotations

import json
import math
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Callable, Protocol

from src.bounded_relative_motion import (
    SHARED_MAX_RELATIVE_MOTION_DISTANCE_M,
    build_bounded_relative_motion_contract,
)
from src.qwen_motion_parser import (
    QwenMotionParserRequest,
    evaluate_qwen_motion_parser,
    evaluate_shared_motion_parser,
)


CONTRACT_VERSION = "teto_unified_segmented_operator.v1"
EVIDENCE_SCHEMA_VERSION = "teto_unified_segmented_operator_run.v1"
STATUS_PASS = "PASS"
STATUS_BLOCKED = "BLOCKED"
STATUS_ABORTED = "ABORTED"
EPS = 1e-9


class UnifiedOperatorBackend(Protocol):
    backend_name: str
    execution_mode: str
    autonomous_segmented_execution: bool
    vision_guard_required: bool

    def status(self) -> dict[str, Any]: ...

    def read_tcp_pose(self) -> dict[str, Any]: ...

    def check_motion_state(self) -> dict[str, Any]: ...

    def capture_snapshot(
        self,
        *,
        phase: str,
        previous_snapshot: dict[str, Any] | None = None,
    ) -> dict[str, Any]: ...

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
    ) -> dict[str, Any]: ...

    def begin_relative_motion(
        self,
        *,
        initial_tcp_pose: dict[str, Any],
        planned_subgoals: list[dict[str, Any]],
    ) -> None: ...

    def home_reference_pose(self) -> dict[str, Any] | None: ...

    def reset_session(self) -> dict[str, Any]: ...


@dataclass(frozen=True)
class UnifiedOperatorConfig:
    max_total_distance_m: float = SHARED_MAX_RELATIVE_MOTION_DISTANCE_M
    max_substep_distance_m: float = 0.05
    position_tolerance_m: float = 0.005
    orientation_tolerance_rad: float = 0.01
    tcp_pose_stale_after_s: float = 1.0
    workspace_bounds: dict[str, list[float]] | None = None
    qwen_endpoint: str | None = None
    parser_mode: str = "qwen"
    output_dir: str | Path = "outputs/unified_operator_runs"

    def normalized_workspace_bounds(self) -> dict[str, list[float]]:
        raw = self.workspace_bounds if isinstance(self.workspace_bounds, dict) else {}
        return {
            "x": _bounds(raw.get("x"), [-1.0, 1.0]),
            "y": _bounds(raw.get("y"), [-1.0, 1.0]),
            "z": _bounds(raw.get("z"), [0.0, 2.0]),
        }


class OperatorCommandInterface:
    """Backend-neutral command semantics and bounded-motion contract builder."""

    def __init__(
        self,
        config: UnifiedOperatorConfig,
        *,
        qwen_callable: Callable[[str], str] | None = None,
    ) -> None:
        self.config = config
        self.qwen_callable = qwen_callable

    @staticmethod
    def classify(text: str) -> str:
        normalized = " ".join(str(text or "").strip().lower().split())
        if normalized in {"quit", "exit"}:
            return "quit"
        if normalized == "status":
            return "status"
        if normalized in {"home", "demo center", "demo_center"}:
            return "home"
        if normalized in {"reset", "reset home"}:
            return "reset"
        if normalized in {"help", "?"}:
            return "help"
        return "motion"

    def parse_motion(self, text: str) -> dict[str, Any]:
        if self.config.parser_mode == "shared":
            return evaluate_shared_motion_parser(
                text,
                max_distance_m=self.config.max_total_distance_m,
            )
        return evaluate_qwen_motion_parser(
            QwenMotionParserRequest(
                user_text=text,
                max_distance_m=self.config.max_total_distance_m,
                hard_safety_limit_m=self.config.max_total_distance_m,
                endpoint=self.config.qwen_endpoint,
                llm_callable=self.qwen_callable,
            )
        )

    def build_motion_contract(
        self,
        *,
        delta_m: list[float],
        current_tcp_pose: dict[str, Any],
        execution_backend: str,
        backend_policy_id: str,
    ) -> dict[str, Any]:
        distance = _norm(delta_m)
        if distance > min(
            float(self.config.max_total_distance_m),
            SHARED_MAX_RELATIVE_MOTION_DISTANCE_M,
        ) + EPS:
            return {
                "final_plan_status": STATUS_BLOCKED,
                "final_blocking_reason": "E_RELATIVE_MOTION_RANGE_EXCEEDED",
                "decomposed_substeps_m": [],
                "planned_subgoals": [],
            }
        final_position = _add(current_tcp_pose["position_m"], delta_m)
        if not _in_workspace(final_position, self.config.normalized_workspace_bounds()):
            return {
                "final_plan_status": STATUS_BLOCKED,
                "final_blocking_reason": "E_OUT_OF_WORKSPACE",
                "decomposed_substeps_m": [],
                "planned_subgoals": [],
            }
        contract = build_bounded_relative_motion_contract(
            delta_m,
            max_substep_m=self.config.max_substep_distance_m,
            execution_backend=execution_backend,
            backend_policy_id=backend_policy_id,
            start_pose=current_tcp_pose,
        )
        return {
            **contract,
            "final_plan_status": STATUS_PASS,
            "planned_execution_style": "unified_segmented_relative_motion",
            "safety_gate_scope": "per_segment_backend_authority",
            "workspace_check_status": STATUS_PASS,
        }


class UnifiedSegmentedOperator:
    def __init__(
        self,
        *,
        config: UnifiedOperatorConfig,
        backend: UnifiedOperatorBackend,
        qwen_callable: Callable[[str], str] | None = None,
    ) -> None:
        self.config = config
        self.backend = backend
        self.command_interface = OperatorCommandInterface(
            config,
            qwen_callable=qwen_callable,
        )
        self.last_result: dict[str, Any] | None = None
        self.last_evidence_path: str | None = None

    def handle_command(self, text: str) -> dict[str, Any]:
        command = self.command_interface.classify(text)
        if command == "quit":
            return {
                "status": STATUS_PASS,
                "operator_command": "quit",
                "quit_requested": True,
                **self._safety_semantics(),
            }
        if command == "help":
            return {
                "status": STATUS_PASS,
                "operator_command": "help",
                "commands": [
                    "status",
                    "home",
                    "reset",
                    "help",
                    "quit/exit",
                    "natural-language relative Cartesian motion",
                ],
                **self._safety_semantics(),
            }
        if command == "status":
            backend_status = self.backend.status()
            return {
                "status": backend_status.get("status", STATUS_BLOCKED),
                "operator_command": "status",
                "abort_reason": backend_status.get("abort_reason"),
                "backend_status": backend_status,
                "last_command_result": self.last_result,
                "last_evidence_path": self.last_evidence_path,
                **self._safety_semantics(),
            }
        if command in {"home", "reset"}:
            if command == "reset":
                reset_result = self.backend.reset_session()
                if reset_result.get("status") not in {None, STATUS_PASS}:
                    return {
                        "status": STATUS_BLOCKED,
                        "operator_command": "reset",
                        "abort_reason": reset_result.get("abort_reason")
                        or "E_BACKEND_RESET_FAILED",
                        "backend_reset_result": reset_result,
                        **self._safety_semantics(),
                    }
            return self._execute_home(command)
        return self.execute_text(text)

    def execute_text(self, text: str) -> dict[str, Any]:
        normalized = str(text or "").strip()
        if not normalized:
            return self._blocked("E_EMPTY_COMMAND", input_text=normalized)
        parser_result = self.command_interface.parse_motion(normalized)
        if parser_result.get("qwen_motion_parser_status") != STATUS_PASS:
            reasons = parser_result.get("blocking_reasons") or ["E_MOTION_PARSE_BLOCKED"]
            return self._blocked(
                str(reasons[0]),
                input_text=normalized,
                parser_result=parser_result,
            )
        delta = _vector(parser_result.get("delta_m"))
        if delta is None or _norm(delta) <= EPS:
            return self._blocked(
                "E_INVALID_ZERO_MOTION",
                input_text=normalized,
                parser_result=parser_result,
            )
        return self._execute_delta(
            input_text=normalized,
            delta_m=delta,
            parser_result=parser_result,
            operator_command="motion",
        )

    def _execute_home(self, operator_command: str) -> dict[str, Any]:
        current = self.backend.read_tcp_pose()
        current_pose = _pose(current)
        home_pose = _pose(self.backend.home_reference_pose())
        if current_pose is None:
            return self._blocked(
                current.get("abort_reason") or "E_CURRENT_TCP_POSE_MISSING",
                input_text=operator_command,
            )
        if home_pose is None:
            return self._blocked(
                "E_HOME_REFERENCE_NOT_AVAILABLE",
                input_text=operator_command,
            )
        delta = [
            round(float(home_pose["position_m"][axis]) - float(current_pose["position_m"][axis]), 6)
            for axis in range(3)
        ]
        if _norm(delta) <= EPS:
            return {
                "status": STATUS_PASS,
                "operator_command": operator_command,
                "already_at_home": True,
                "current_tcp_pose": current_pose,
                **self._safety_semantics(),
            }
        return self._execute_delta(
            input_text=operator_command,
            delta_m=delta,
            parser_result={
                "qwen_motion_parser_status": STATUS_PASS,
                "parser_source": "operator_builtin_home",
                "delta_m": delta,
                "requested_distance_m": round(_norm(delta), 6),
            },
            operator_command=operator_command,
        )

    def _execute_delta(
        self,
        *,
        input_text: str,
        delta_m: list[float],
        parser_result: dict[str, Any],
        operator_command: str,
    ) -> dict[str, Any]:
        initial_evidence = self.backend.read_tcp_pose()
        initial_pose = _pose(initial_evidence)
        if initial_pose is None:
            return self._blocked(
                initial_evidence.get("abort_reason") or "E_CURRENT_TCP_POSE_MISSING",
                input_text=input_text,
                parser_result=parser_result,
            )
        pose_blocker = self._pose_blocker(initial_evidence)
        if pose_blocker:
            return self._blocked(
                pose_blocker,
                input_text=input_text,
                parser_result=parser_result,
            )
        plan = self.command_interface.build_motion_contract(
            delta_m=delta_m,
            current_tcp_pose=initial_pose,
            execution_backend=self.backend.execution_mode,
            backend_policy_id=f"{self.backend.backend_name}_unified_segmented_v1",
        )
        if plan.get("final_plan_status") != STATUS_PASS:
            return self._blocked(
                plan.get("final_blocking_reason") or "E_DECOMPOSITION_FAILED",
                input_text=input_text,
                parser_result=parser_result,
                plan=plan,
            )

        steps = plan.get("decomposed_substeps_m") or []
        evidence = self._base_evidence(
            input_text=input_text,
            operator_command=operator_command,
            parser_result=parser_result,
            initial_pose_evidence=initial_evidence,
            delta_m=delta_m,
            plan=plan,
        )
        try:
            self.backend.begin_relative_motion(
                initial_tcp_pose=initial_pose,
                planned_subgoals=plan.get("planned_subgoals") or [],
            )
        except Exception as exc:
            evidence["abort_reason"] = f"E_BACKEND_BEGIN_MOTION_FAILED:{exc}"
            return self._finish(evidence)

        completed = 0
        any_motion = False
        latest_verified_pose = initial_pose
        for index, step_delta, substep_count in iter_segmented_subgoals(steps):
            before_evidence = self.backend.read_tcp_pose()
            before = _pose(before_evidence)
            if before is None:
                evidence["abort_reason"] = before_evidence.get("abort_reason") or "E_CURRENT_TCP_POSE_MISSING"
                break
            pose_blocker = self._pose_blocker(before_evidence)
            if pose_blocker:
                evidence["abort_reason"] = pose_blocker
                break

            state = self.backend.check_motion_state()
            evidence["backend_status"] = {
                "status": state.get("status"),
                "abort_reason": state.get("abort_reason"),
                "state": state,
                "current_tcp_pose": before_evidence,
            }
            if state.get("status") != STATUS_PASS:
                evidence["abort_reason"] = state.get("abort_reason") or "E_REAL_STATE_CHECK_BLOCKED"
                evidence["last_state_evidence"] = state
                break

            before_snapshot = self.backend.capture_snapshot(phase="before")
            evidence["backend_status"]["vision"] = before_snapshot
            if self.backend.vision_guard_required and before_snapshot.get("status") != STATUS_PASS:
                evidence["abort_reason"] = before_snapshot.get("abort_reason") or "E_VISION_GUARD_BLOCKED"
                evidence["last_vision_evidence"] = before_snapshot
                break

            target = {
                "frame": before["frame"],
                "position_m": _add(before["position_m"], step_delta),
                "orientation_xyzw": list(before["orientation_xyzw"]),
            }
            if not _in_workspace(
                target["position_m"],
                self.config.normalized_workspace_bounds(),
            ):
                evidence["abort_reason"] = "E_OUT_OF_WORKSPACE"
                break

            execution = self.backend.execute_subgoal(
                delta_m=step_delta,
                current_tcp_pose=before,
                target_tcp_pose=target,
                state_evidence=state,
                vision_evidence=before_snapshot,
                substep_index=index,
                substep_count=substep_count,
            )
            execution_attempted = any(
                execution.get(field) is True
                for field in (
                    "real_robot_motion_executed",
                    "trajectory_sent",
                    "moveit_execute_called",
                    "real_execution_attempted",
                )
            )
            any_motion = any_motion or execution.get("real_robot_motion_executed") is True
            if execution.get("status") != STATUS_PASS:
                after_evidence = (
                    self.backend.read_tcp_pose() if execution_attempted else None
                )
                after = _pose(after_evidence)
                after_snapshot = (
                    self.backend.capture_snapshot(
                        phase="after_failed_execution",
                        previous_snapshot=before_snapshot,
                    )
                    if execution_attempted
                    else None
                )
                verification = (
                    verify_segment_motion(
                        before_tcp_pose=before,
                        target_tcp_pose=target,
                        after_tcp_pose=after,
                        intended_delta_m=step_delta,
                        position_tolerance_m=self.config.position_tolerance_m,
                        orientation_tolerance_rad=self.config.orientation_tolerance_rad,
                    )
                    if execution_attempted
                    else {"status": "NOT_RUN"}
                )
                evidence["substeps"].append(
                    self._failed_substep(
                        index=index,
                        count=substep_count,
                        delta_m=step_delta,
                        before=before,
                        target=target,
                        state=state,
                        before_snapshot=before_snapshot,
                        after=after,
                        after_snapshot=after_snapshot,
                        verification=verification,
                        execution=execution,
                    )
                )
                evidence["abort_reason"] = execution.get("abort_reason") or "E_SUBSTEP_EXECUTION_BLOCKED"
                break

            after_evidence = self.backend.read_tcp_pose()
            after = _pose(after_evidence)
            after_pose_blocker = self._pose_blocker(after_evidence)
            after_snapshot = self.backend.capture_snapshot(
                phase="after",
                previous_snapshot=before_snapshot,
            )
            verification = verify_segment_motion(
                before_tcp_pose=before,
                target_tcp_pose=target,
                after_tcp_pose=after,
                intended_delta_m=step_delta,
                position_tolerance_m=self.config.position_tolerance_m,
                orientation_tolerance_rad=self.config.orientation_tolerance_rad,
            )
            snapshot_ok = (
                not self.backend.vision_guard_required
                or after_snapshot.get("status") == STATUS_PASS
            )
            continue_allowed = (
                verification.get("status") == STATUS_PASS
                and snapshot_ok
                and after_pose_blocker is None
                and after is not None
            )
            step_evidence = {
                "substep_index": index,
                "substep_count": substep_count,
                "delta_m": step_delta,
                "target_tcp_pose": target,
                "measured_tcp_before": before,
                "measured_tcp_after": after,
                "state_evidence": state,
                "before_snapshot_evidence": before_snapshot,
                "after_snapshot_evidence": after_snapshot,
                "execution_result": execution,
                "verification": verification,
                "verification_result": verification.get("status"),
                "after_tcp_pose_freshness_status": (
                    STATUS_PASS if after_pose_blocker is None else STATUS_BLOCKED
                ),
                "continue_allowed": continue_allowed,
            }
            evidence["substeps"].append(step_evidence)
            if not continue_allowed:
                evidence["abort_reason"] = (
                    after_snapshot.get("abort_reason")
                    if not snapshot_ok
                    else after_pose_blocker
                    if after_pose_blocker is not None
                    else verification.get("abort_reason")
                    or "E_POST_MOTION_VERIFICATION_FAILED"
                )
                break
            completed += 1
            latest_verified_pose = after

        evidence["completed_substep_count"] = completed
        evidence["final_tcp_pose"] = latest_verified_pose
        evidence["real_robot_motion_executed"] = any_motion
        evidence["simulated_robot_motion_executed"] = (
            completed > 0 and self.backend.execution_mode == "isaac_sim"
        )
        evidence["status"] = STATUS_PASS if completed == len(steps) else STATUS_ABORTED
        evidence["safety_gate_status"] = STATUS_PASS if evidence["status"] == STATUS_PASS else STATUS_BLOCKED
        return self._finish(evidence)

    def _pose_blocker(self, evidence: dict[str, Any]) -> str | None:
        if evidence.get("status") == STATUS_BLOCKED:
            return str(evidence.get("abort_reason") or "E_CURRENT_TCP_POSE_MISSING")
        age = _number(evidence.get("tcp_pose_age_s"))
        fresh = evidence.get("tcp_pose_fresh")
        if fresh is False:
            return "E_TCP_POSE_STALE"
        if age is not None and age > self.config.tcp_pose_stale_after_s + EPS:
            return "E_TCP_POSE_STALE"
        return None

    def _base_evidence(
        self,
        *,
        input_text: str,
        operator_command: str,
        parser_result: dict[str, Any],
        initial_pose_evidence: dict[str, Any],
        delta_m: list[float],
        plan: dict[str, Any],
    ) -> dict[str, Any]:
        return {
            "contract_version": CONTRACT_VERSION,
            "schema_version": EVIDENCE_SCHEMA_VERSION,
            "generated_at": datetime.now(timezone.utc).isoformat(),
            "status": STATUS_BLOCKED,
            "operator_command": operator_command,
            "input_text": input_text,
            "execution_mode": self.backend.execution_mode,
            "backend_name": self.backend.backend_name,
            "parser_result": parser_result,
            "parser_source": parser_result.get("parser_source"),
            "delta_vector_m": delta_m,
            "requested_total_distance_m": round(_norm(delta_m), 6),
            "requested_distance_norm_m": round(_norm(delta_m), 6),
            "motion_contract_type": "decomposed_relative_motion",
            "segmentation_contract": plan,
            "max_total_distance_m": self.config.max_total_distance_m,
            "max_substep_distance_m": self.config.max_substep_distance_m,
            "substep_count": len(plan.get("decomposed_substeps_m") or []),
            "completed_substep_count": 0,
            "planned_subgoals": plan.get("planned_subgoals") or [],
            "initial_tcp_pose": initial_pose_evidence,
            "final_tcp_pose": initial_pose_evidence,
            "backend_status": {
                "status": initial_pose_evidence.get("status"),
                "abort_reason": initial_pose_evidence.get("abort_reason"),
                "current_tcp_pose": initial_pose_evidence,
            },
            "substeps": [],
            "abort_reason": None,
            "safety_gate_status": "NOT_RUN",
            "real_robot_motion_executed": False,
            "simulated_robot_motion_executed": False,
            "artifact_paths": {},
            **self._safety_semantics(),
        }

    def _failed_substep(
        self,
        *,
        index: int,
        count: int,
        delta_m: list[float],
        before: dict[str, Any],
        target: dict[str, Any],
        state: dict[str, Any],
        before_snapshot: dict[str, Any],
        after: dict[str, Any] | None,
        after_snapshot: dict[str, Any] | None,
        verification: dict[str, Any],
        execution: dict[str, Any],
    ) -> dict[str, Any]:
        return {
            "substep_index": index,
            "substep_count": count,
            "delta_m": delta_m,
            "target_tcp_pose": target,
            "measured_tcp_before": before,
            "measured_tcp_after": after,
            "state_evidence": state,
            "before_snapshot_evidence": before_snapshot,
            "after_snapshot_evidence": after_snapshot,
            "execution_result": execution,
            "verification": verification,
            "verification_result": verification.get("status"),
            "continue_allowed": False,
        }

    def _blocked(
        self,
        reason: str,
        *,
        input_text: str,
        parser_result: dict[str, Any] | None = None,
        plan: dict[str, Any] | None = None,
    ) -> dict[str, Any]:
        result = {
            "contract_version": CONTRACT_VERSION,
            "schema_version": EVIDENCE_SCHEMA_VERSION,
            "status": STATUS_BLOCKED,
            "input_text": input_text,
            "execution_mode": self.backend.execution_mode,
            "backend_name": self.backend.backend_name,
            "parser_result": parser_result,
            "segmentation_contract": plan,
            "substep_count": 0,
            "completed_substep_count": 0,
            "substeps": [],
            "abort_reason": reason,
            "real_robot_motion_executed": False,
            "simulated_robot_motion_executed": False,
            "artifact_paths": {},
            **self._safety_semantics(),
        }
        return self._finish(result)

    def _finish(self, evidence: dict[str, Any]) -> dict[str, Any]:
        output_dir = Path(self.config.output_dir)
        output_dir.mkdir(parents=True, exist_ok=True)
        stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S%fZ")
        path = output_dir / f"{stamp}_{self.backend.execution_mode}_operator_run.json"
        evidence["artifact_paths"] = {"json": str(path)}
        path.write_text(
            json.dumps(evidence, indent=2, ensure_ascii=False) + "\n",
            encoding="utf-8",
        )
        self.last_result = evidence
        self.last_evidence_path = str(path)
        return evidence

    def _safety_semantics(self) -> dict[str, Any]:
        return {
            "manual_confirmation_required": False,
            "autonomous_segmented_execution": bool(
                self.backend.autonomous_segmented_execution
            ),
            "safety_gate_still_required": True,
            "vision_guard_required": bool(self.backend.vision_guard_required),
            "automatic_retry_motion": False,
            "moveit_long_distance_one_shot_allowed": False,
        }


def verify_segment_motion(
    *,
    before_tcp_pose: dict[str, Any],
    target_tcp_pose: dict[str, Any],
    after_tcp_pose: dict[str, Any] | None,
    intended_delta_m: list[float],
    position_tolerance_m: float,
    orientation_tolerance_rad: float,
) -> dict[str, Any]:
    before = _pose(before_tcp_pose)
    target = _pose(target_tcp_pose)
    after = _pose(after_tcp_pose)
    if before is None or target is None or after is None:
        return {
            "status": STATUS_BLOCKED,
            "abort_reason": "E_TCP_POSE_BEFORE_TARGET_OR_AFTER_UNAVAILABLE",
            "continue_allowed": False,
        }
    actual_delta = [
        float(after["position_m"][axis]) - float(before["position_m"][axis])
        for axis in range(3)
    ]
    target_error = _distance(after["position_m"], target["position_m"])
    distance_error = abs(_norm(actual_delta) - _norm(intended_delta_m))
    direction_projection = _dot(actual_delta, intended_delta_m)
    orientation_change = _quaternion_angle(
        before["orientation_xyzw"],
        after["orientation_xyzw"],
    )
    direction_ok = direction_projection > EPS
    passed = (
        direction_ok
        and target_error <= float(position_tolerance_m) + EPS
        and distance_error <= float(position_tolerance_m) + EPS
        and orientation_change <= float(orientation_tolerance_rad) + EPS
    )
    return {
        "status": STATUS_PASS if passed else STATUS_BLOCKED,
        "abort_reason": None if passed else "E_POST_MOTION_VERIFICATION_FAILED",
        "actual_delta_m": [round(value, 6) for value in actual_delta],
        "actual_distance_m": round(_norm(actual_delta), 6),
        "distance_error_m": round(distance_error, 6),
        "target_position_error_m": round(target_error, 6),
        "direction_projection": round(direction_projection, 9),
        "direction_check_passed": direction_ok,
        "orientation_change_rad": round(orientation_change, 9),
        "orientation_check_passed": orientation_change
        <= float(orientation_tolerance_rad) + EPS,
        "position_tolerance_m": float(position_tolerance_m),
        "orientation_tolerance_rad": float(orientation_tolerance_rad),
        "continue_allowed": passed,
    }


def iter_segmented_subgoals(
    step_deltas: list[list[float]],
):
    """Yield the shared ordered segment sequence used by every backend."""
    count = len(step_deltas)
    for index, delta in enumerate(step_deltas, start=1):
        yield index, [float(value) for value in delta], count


def _pose(value: Any) -> dict[str, Any] | None:
    if not isinstance(value, dict):
        return None
    if isinstance(value.get("current_tcp_pose"), dict):
        value = value["current_tcp_pose"]
    position = _vector(value.get("position_m"))
    orientation = _quaternion(value.get("orientation_xyzw"))
    if position is None or orientation is None:
        return None
    return {
        "frame": str(value.get("frame") or "base_link"),
        "position_m": position,
        "orientation_xyzw": orientation,
    }


def _vector(value: Any) -> list[float] | None:
    if not isinstance(value, (list, tuple)) or len(value) != 3:
        return None
    result = [_number(item) for item in value]
    if any(item is None for item in result):
        return None
    return [float(item) for item in result if item is not None]


def _quaternion(value: Any) -> list[float] | None:
    if not isinstance(value, (list, tuple)) or len(value) != 4:
        return None
    result = [_number(item) for item in value]
    if any(item is None for item in result):
        return None
    quaternion = [float(item) for item in result if item is not None]
    if _norm(quaternion) <= EPS:
        return None
    return quaternion


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


def _dot(left: list[float], right: list[float]) -> float:
    return sum(float(a) * float(b) for a, b in zip(left, right))


def _distance(left: list[float], right: list[float]) -> float:
    return _norm([float(a) - float(b) for a, b in zip(left, right)])


def _add(left: list[float], right: list[float]) -> list[float]:
    return [round(float(a) + float(b), 6) for a, b in zip(left, right)]


def _quaternion_angle(left: list[float], right: list[float]) -> float:
    left_norm = _norm(left)
    right_norm = _norm(right)
    if left_norm <= EPS or right_norm <= EPS:
        return math.inf
    dot = abs(
        sum(float(a) * float(b) for a, b in zip(left, right))
        / (left_norm * right_norm)
    )
    return 2.0 * math.acos(max(-1.0, min(1.0, dot)))


def _bounds(value: Any, default: list[float]) -> list[float]:
    if isinstance(value, (list, tuple)) and len(value) == 2:
        low = _number(value[0])
        high = _number(value[1])
        if low is not None and high is not None and low <= high:
            return [low, high]
    return list(default)


def _in_workspace(
    point: list[float],
    workspace: dict[str, list[float]],
) -> bool:
    return all(
        workspace[axis][0] <= float(point[index]) <= workspace[axis][1]
        for index, axis in enumerate(("x", "y", "z"))
    )
