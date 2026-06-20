from __future__ import annotations

import ipaddress
import json
import math
import urllib.error
import urllib.request
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Protocol

from src.autoregressive_motion_planner import (
    AutoregressiveMotionPlannerRequest,
    plan_offline_autoregressive_motion,
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
        return _positive(self.raw.get("max_substep_distance_m"), 0.02)

    @property
    def max_total_distance_m(self) -> float:
        return _positive(self.raw.get("max_total_distance_m"), 0.35)

    @property
    def position_tolerance_m(self) -> float:
        return _positive(self.raw.get("position_tolerance_m"), 0.008)

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
    ) -> None:
        self.config = config
        self.gateway = gateway
        self.headless = bool(headless)
        self.output_dir = Path(output_dir)
        self.qwen_callable = qwen_callable
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
        evidence["requested_distance_norm_m"] = round(_norm(delta), 6)
        evidence["substep_count"] = len(step_deltas)
        if not step_deltas:
            evidence["status"] = "BLOCKED"
            evidence["abort_reason"] = plan.get("final_blocking_reason") or "E_DECOMPOSITION_FAILED"
            return self._finish(evidence)

        latest = current_pose
        completed = 0
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
                evidence["abort_reason"] = "E_DECOMPOSED_WORKSPACE_ENVELOPE_EXCEEDED"
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
                    "continue_allowed": verified,
                    "gateway_result": result,
                }
            )
            if not verified:
                evidence["abort_reason"] = (
                    result.get("abort_reason")
                    or ("E_SIMULATED_POST_STEP_DIRECTION_MISMATCH" if not direction_ok else "E_SIMULATED_POST_STEP_VERIFICATION_FAILED")
                )
                break
            completed += 1
            latest = after

        evidence["completed_substep_count"] = completed
        evidence["simulated_robot_motion_executed"] = completed > 0
        evidence["final_simulated_tcp_pose"] = latest
        evidence["status"] = "PASS" if completed == len(step_deltas) else "ABORTED"
        return self._finish(evidence)

    def home(self) -> dict[str, Any]:
        return self.gateway.home()

    def reset(self) -> dict[str, Any]:
        return self.gateway.reset()

    def _decompose(
        self,
        parser_result: dict[str, Any],
        current_pose: dict[str, Any],
        delta: list[float],
    ) -> tuple[list[list[float]], dict[str, Any]]:
        norm = _norm(delta)
        if norm <= self.config.max_substep_distance_m + EPS:
            return [delta], {
                "final_plan_status": "PASS",
                "planned_execution_style": "isaac_sim_one_substep",
                "substep_count": 1,
                "decomposed_substeps_m": [delta],
            }
        canonical = {
            "parse_status": "PASS",
            "intent": "relative_cartesian_motion",
            "motion_frame": parser_result.get("motion_frame") or "base_link",
            "requested_distance_m": norm,
            "requested_distance_norm_m": norm,
            "delta_m": delta,
            "vector_delta_m": delta,
            "motion_contract_type": parser_result.get("motion_contract_type"),
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
        vectors = plan.get("decomposed_substeps_m") or plan.get("planned_substep_vectors_m") or []
        return [vector for vector in (_vector(item) for item in vectors) if vector is not None], plan

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
            "requested_distance_norm_m": None,
            "substep_count": 0,
            "completed_substep_count": 0,
            "substeps": [],
            "abort_reason": None,
            "final_simulated_tcp_pose": before_status.get("current_tcp_pose"),
            "gui_mode_confirmation": {
                "isaac_gui_required": not self.headless,
                "headless_smoke_test": self.headless,
                "viewport_expected_visible": not self.headless,
            },
            "simulated_robot_motion_executed": False,
            "artifact_paths": {},
            **_safety_flags(),
        }

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
