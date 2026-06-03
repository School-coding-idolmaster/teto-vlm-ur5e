from __future__ import annotations

import json
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict

import yaml

from src.manual_confirmation_gate import (
    DEFAULT_CONFIRMATION_TIMEOUT_S,
    DEFAULT_CONFIRMATION_TOKEN,
    ManualConfirmationRequest,
    evaluate_manual_confirmation_gate,
)
from src.moveit_plan_only_contract import MoveItPlanOnlyRequest, evaluate_moveit_plan_only
from src.output_paths import PROJECT_ROOT
from src.perception_shadow_pipeline import PerceptionShadowPipelineRequest, evaluate_perception_shadow_pipeline
from src.planner_gateway_shadow import PlannerGatewayShadowRequest, evaluate_planner_gateway_shadow
from src.real_ur5_hover_executor import RealUR5HoverExecutionRequest, evaluate_real_ur5_hover_execution
from src.ros2_interface_readiness import ROS2InterfaceReadinessRequest, evaluate_ros2_interface_readiness
from src.ros2_message_exporter import ROS2MessageExportRequest, evaluate_ros2_message_export
from src.robot_system_shadow_bridge import RobotSystemShadowBridgeRequest, evaluate_robot_system_shadow_bridge
from src.ur5_read_only_state_contract import UR5ReadOnlyStateRequest, evaluate_ur5_read_only_state
from src.v3_command_normalizer import INTENT_HOVER_TO_OBJECT, normalize_v3_command


CURRENT_V3_HOVER_DEMO_VERSION = "TETO V3.0.0"
CONTRACT_VERSION = "teto_v3_hover_demo.v1"

STATUS_PASS = "PASS"
STATUS_BLOCKED = "BLOCKED"

DEFAULT_OUTPUT_ROOT = PROJECT_ROOT / "outputs" / "simulation_runs"

V3_HOVER_DEMO_FIELDS = (
    "v3_hover_demo_evidence_available",
    "v3_hover_demo_status",
    "v3_demo_mode",
    "user_command",
    "normalized_intent",
    "target_label",
    "snapshot_id",
    "grounding_id",
    "scene_version",
    "semantic_gate_passed",
    "geometry_validity_status",
    "projector_status",
    "world_point_m",
    "bounded_target_point_m",
    "hover_offset_m",
    "planner_request_ready",
    "ros2_interface_ready",
    "moveit_plan_ready",
    "ur5_state_ok",
    "manual_confirmation_required",
    "manual_confirmation_accepted",
    "enable_live_camera",
    "enable_live_vlm",
    "enable_ros2_runtime",
    "enable_moveit_plan",
    "enable_moveit_execute",
    "enable_real_robot_motion",
    "moveit_execute_called",
    "trajectory_send_allowed",
    "trajectory_sent",
    "controller_command_sent",
    "real_robot_motion_executed",
    "return_home_requested",
    "return_home_executed",
    "return_home_skipped",
    "urscript_generated",
    "rtde_write_attempted",
    "dashboard_command_attempted",
    "raw_joint_targets_generated",
    "tcp_pose_world_generated_by_llm",
    "blocking_reasons",
    "warnings",
)


@dataclass(frozen=True)
class V3HoverDemoRequest:
    requested: bool = False
    user_command: str | None = None
    config_path: str | None = None
    config: Dict[str, Any] | None = None
    output_dir: str | None = None
    write_evidence: bool = False
    manual_confirmation_token: str | None = None
    enable_live_camera: bool = False
    enable_live_vlm: bool = False
    enable_ros2_runtime: bool = False
    enable_moveit_plan: bool = False
    enable_moveit_execute: bool = False
    enable_real_robot_motion: bool = False


def load_v3_hover_demo_config(path: str | Path | None) -> Dict[str, Any]:
    if not path:
        return {}
    resolved = Path(path).expanduser()
    if not resolved.is_file():
        return {}
    with resolved.open("r", encoding="utf-8") as config_file:
        data = json.load(config_file) if resolved.suffix.lower() == ".json" else yaml.safe_load(config_file)
    if not isinstance(data, dict):
        return {}
    config = data.get("v3_hover_demo")
    return config if isinstance(config, dict) else data


def evaluate_v3_hover_demo(request: V3HoverDemoRequest | None = None) -> Dict[str, Any]:
    request = request or V3HoverDemoRequest()
    if not request.requested:
        return _not_requested_result()

    config = _merge_config(load_v3_hover_demo_config(request.config_path), request.config)
    config = _apply_runtime_flags(config, request)
    user_command = request.user_command or _string(config.get("user_command")) or "hover over the red mug"
    stages: list[str] = []
    blocking_reasons: list[str] = []
    warnings: list[str] = []

    normalized = normalize_v3_command(user_command)
    stages.append("COMMAND_NORMALIZED")
    if normalized.get("accepted") is not True:
        blocking_reasons.append(str(normalized.get("error_code") or "E_UNSUPPORTED_COMMAND"))

    perception = {}
    planner = {}
    ros2 = {}
    message_export = {}
    moveit = {}
    ur5_state = {}
    shadow_bridge = {}
    manual_confirmation = {}
    executor = {}

    if normalized.get("intent_name") == INTENT_HOVER_TO_OBJECT and not blocking_reasons:
        perception = evaluate_perception_shadow_pipeline(
            PerceptionShadowPipelineRequest(
                requested=True,
                config_path=request.config_path,
                user_command=user_command,
                config=_perception_config(config, user_command),
            )
        )
        _append_stages(
            stages,
            perception,
            [
                ("camera_snapshot_validity_status", STATUS_PASS, "CAMERA_SNAPSHOT_READY"),
                ("vlm_grounding_status", STATUS_PASS, "VLM_GROUNDING_READY"),
                ("semantic_gate_passed", True, "SEMANTIC_GATE_PASSED"),
                ("geometry_validity_status", STATUS_PASS, "GEOMETRY_VALID"),
                ("projector_status", STATUS_PASS, "PROJECTOR_READY"),
            ],
        )
        blocking_reasons.extend(_stage_blockers(perception))
        warnings.extend(_list(perception.get("warnings")))

        planner = evaluate_planner_gateway_shadow(
            PlannerGatewayShadowRequest(
                requested=True,
                config_path=request.config_path,
                perception_shadow_result=perception,
                config=_planner_config(config),
            )
        )
        if planner.get("planner_input_ready") is True:
            stages.append("PLANNER_REQUEST_READY")
        blocking_reasons.extend(_stage_blockers(planner))
        warnings.extend(_list(planner.get("warnings")))

        ros2 = evaluate_ros2_interface_readiness(
            ROS2InterfaceReadinessRequest(requested=True, config_path=request.config_path, config=_ros2_config(config))
        )
        if ros2.get("ros2_interface_readiness_status") == "READY_FOR_SHADOW_BRIDGE":
            stages.append("ROS2_INTERFACE_READY")
        blocking_reasons.extend(_stage_blockers(ros2))
        warnings.extend(_list(ros2.get("warnings")))

        message_export = evaluate_ros2_message_export(
            ROS2MessageExportRequest(
                requested=True,
                config_path=request.config_path,
                config=_ros2_message_export_config(config),
                planner_gateway_shadow_result=planner,
                ros2_interface_readiness_result=ros2,
            )
        )
        blocking_reasons.extend(_stage_blockers(message_export))
        warnings.extend(_list(message_export.get("warnings")))

        moveit = evaluate_moveit_plan_only(
            MoveItPlanOnlyRequest(
                requested=True,
                config_path=request.config_path,
                config=_moveit_plan_config(config),
                ros2_message_export_result=message_export,
            )
        )
        if moveit.get("plan_only_ready") is True:
            stages.append("MOVEIT_PLAN_READY")
        blocking_reasons.extend(_stage_blockers(moveit))
        warnings.extend(_list(moveit.get("warnings")))

        ur5_state = evaluate_ur5_read_only_state(
            UR5ReadOnlyStateRequest(requested=True, config_path=request.config_path, config=_ur5_state_config(config))
        )
        if ur5_state.get("read_only_state_contract_ready") is True:
            stages.append("UR5_STATE_OK")
        blocking_reasons.extend(_stage_blockers(ur5_state))
        warnings.extend(_list(ur5_state.get("warnings")))

        shadow_bridge = evaluate_robot_system_shadow_bridge(
            RobotSystemShadowBridgeRequest(
                requested=True,
                config_path=request.config_path,
                config=_shadow_bridge_config(config),
                ros2_message_export_result=message_export,
                moveit_plan_only_result=moveit,
                ur5_read_only_state_result=ur5_state,
            )
        )
        blocking_reasons.extend(_stage_blockers(shadow_bridge))
        warnings.extend(_list(shadow_bridge.get("warnings")))

        stages.append("MANUAL_CONFIRMATION_REQUIRED")
        manual_confirmation = evaluate_manual_confirmation_gate(
            ManualConfirmationRequest(
                manual_confirmation_required=config.get("manual_confirmation_required", True) is True,
                expected_token=_string(config.get("manual_confirmation_token")) or DEFAULT_CONFIRMATION_TOKEN,
                expected_task_id=_string(planner.get("task_id")),
                expected_target_label=_string(planner.get("target_label")),
                expected_bounded_target_point_m=planner.get("bounded_target_point_m")
                if isinstance(planner.get("bounded_target_point_m"), list)
                else None,
                timeout_s=int(config.get("manual_confirmation_timeout_s", DEFAULT_CONFIRMATION_TIMEOUT_S)),
                confirmation=_confirmation_from_request(config, request, planner),
            )
        )
        if manual_confirmation.get("manual_confirmation_accepted") is True:
            stages.append("MANUAL_CONFIRMATION_ACCEPTED")
        elif config.get("enable_real_robot_motion") is True:
            blocking_reasons.extend(_stage_blockers(manual_confirmation))

        executor = evaluate_real_ur5_hover_execution(
            RealUR5HoverExecutionRequest(
                config=_executor_config(config, planner, perception),
                planner_gateway_result=planner,
                ros2_interface_result=ros2,
                moveit_plan_result=moveit,
                ur5_state_result=_runtime_ur5_state(config, ur5_state),
                manual_confirmation_result=manual_confirmation,
            )
        )
        if executor.get("real_robot_motion_executed") is True:
            stages.append("REAL_HOVER_EXECUTED")
        if executor.get("return_home_executed") is True:
            stages.append("RETURN_HOME_EXECUTED")
        else:
            stages.append("RETURN_HOME_SKIPPED")
        if config.get("enable_real_robot_motion") is True or executor.get("real_robot_motion_executed") is True:
            blocking_reasons.extend(_stage_blockers(executor))
        warnings.extend(_list(executor.get("warnings")))

    stages.append("EVIDENCE_EXPORTED")
    blocking_reasons = _unique(blocking_reasons)
    warnings = _unique(warnings)
    status = STATUS_PASS if not blocking_reasons else STATUS_BLOCKED
    result = _build_result(
        status=status,
        user_command=user_command,
        config=config,
        stages=stages,
        blocking_reasons=blocking_reasons,
        warnings=warnings,
        normalized=normalized,
        perception=perception,
        planner=planner,
        ros2=ros2,
        message_export=message_export,
        moveit=moveit,
        ur5_state=ur5_state,
        shadow_bridge=shadow_bridge,
        manual_confirmation=manual_confirmation,
        executor=executor,
    )
    if request.write_evidence:
        return export_v3_hover_demo_evidence(result, request.output_dir)
    return result


def export_v3_hover_demo_evidence(result: Dict[str, Any], output_dir: str | Path | None = None) -> Dict[str, Any]:
    destination = Path(output_dir).expanduser() if output_dir else _timestamped_output_dir()
    destination.mkdir(parents=True, exist_ok=True)
    result_path = destination / "v3_hover_demo_result.json"
    report_path = destination / "v3_hover_demo_report.md"
    summary_path = destination / "summary.md"
    manifest_path = destination / "evidence_manifest.json"

    payload = dict(result)
    payload["v3_hover_demo_result_path"] = str(result_path)
    payload["v3_hover_demo_report_path"] = str(report_path)
    payload["summary_path"] = str(summary_path)
    payload["evidence_manifest_path"] = str(manifest_path)
    payload["v3_hover_demo_evidence_available"] = True

    result_path.write_text(json.dumps(payload, ensure_ascii=False, indent=2, sort_keys=True), encoding="utf-8")
    report_path.write_text(format_v3_hover_demo_report(payload), encoding="utf-8")
    summary_path.write_text(_format_summary(payload), encoding="utf-8")
    manifest_path.write_text(
        json.dumps(_evidence_manifest(payload), ensure_ascii=False, indent=2, sort_keys=True),
        encoding="utf-8",
    )
    return payload


def format_v3_hover_demo_report(result: Dict[str, Any]) -> str:
    return "\n".join(
        [
            "# TETO V3.0.0 First Real UR5 Hover Demo Report",
            "",
            "## Overall Status",
            "",
            f"- v3_hover_demo_status: {_format_value(result.get('v3_hover_demo_status'))}",
            f"- v3_demo_mode: {_format_value(result.get('v3_demo_mode'))}",
            f"- user_command: {_format_value(result.get('user_command'))}",
            f"- normalized_intent: {_format_value(result.get('normalized_intent'))}",
            f"- target_label: {_format_value(result.get('target_label'))}",
            f"- real_robot_motion_executed: {_format_value(result.get('real_robot_motion_executed'))}",
            f"- blocking_reasons: {_format_value(result.get('blocking_reasons'))}",
            "",
            "## Stages",
            "",
            *[f"- {stage}" for stage in result.get("stages", [])],
            "",
            "## Evidence Fields",
            "",
            "| Field | Value |",
            "| --- | --- |",
            *[f"| {field} | {_format_value(result.get(field))} |" for field in V3_HOVER_DEMO_FIELDS],
            "",
            "## Safety Boundary",
            "",
            "V3.0.0 allows semantic target selection only. Execution remains blocked unless live camera/VLM allowance, ROS2 runtime, MoveIt plan/execute, robot state checks, bounded target validation, and manual confirmation all pass.",
            "",
        ]
    )


def _build_result(
    *,
    status: str,
    user_command: str,
    config: Dict[str, Any],
    stages: list[str],
    blocking_reasons: list[str],
    warnings: list[str],
    normalized: Dict[str, Any],
    perception: Dict[str, Any],
    planner: Dict[str, Any],
    ros2: Dict[str, Any],
    message_export: Dict[str, Any],
    moveit: Dict[str, Any],
    ur5_state: Dict[str, Any],
    shadow_bridge: Dict[str, Any],
    manual_confirmation: Dict[str, Any],
    executor: Dict[str, Any],
) -> Dict[str, Any]:
    return {
        "contract_version": CONTRACT_VERSION,
        "schema_version": CONTRACT_VERSION,
        "teto_version": CURRENT_V3_HOVER_DEMO_VERSION,
        "version": CURRENT_V3_HOVER_DEMO_VERSION,
        "v3_hover_demo_requested": True,
        "v3_hover_demo_evidence_available": False,
        "v3_hover_demo_status": status,
        "ok": status == STATUS_PASS,
        "v3_demo_mode": "lab_real_hover" if executor.get("real_robot_motion_executed") is True else "software_no_robot",
        "user_command": user_command,
        "normalized_intent": normalized.get("intent_name"),
        "normalized_command": normalized.get("normalized_command"),
        "target_label": planner.get("target_label") or normalized.get("target_query"),
        "target_label_hint": normalized.get("target_label_hint"),
        "snapshot_id": perception.get("snapshot_id"),
        "grounding_id": perception.get("grounding_id"),
        "scene_version": perception.get("scene_version"),
        "semantic_gate_passed": perception.get("semantic_gate_passed") is True,
        "geometry_validity_status": perception.get("geometry_validity_status"),
        "projector_status": perception.get("projector_status"),
        "world_point_m": perception.get("world_point_m"),
        "bounded_target_point_m": planner.get("bounded_target_point_m") or executor.get("bounded_target_point_m"),
        "hover_offset_m": planner.get("hover_offset_m", config.get("hover_offset_m")),
        "planner_request_ready": planner.get("planner_input_ready") is True,
        "ros2_interface_ready": ros2.get("ros2_interface_readiness_status") == "READY_FOR_SHADOW_BRIDGE",
        "moveit_plan_ready": moveit.get("plan_only_ready") is True,
        "ur5_state_ok": ur5_state.get("read_only_state_contract_ready") is True,
        "manual_confirmation_required": config.get("manual_confirmation_required", True) is True,
        "manual_confirmation_accepted": manual_confirmation.get("manual_confirmation_accepted") is True,
        "enable_live_camera": config.get("enable_live_camera") is True,
        "enable_live_vlm": config.get("enable_live_vlm") is True,
        "enable_ros2_runtime": config.get("enable_ros2_runtime") is True,
        "enable_moveit_plan": config.get("enable_moveit_plan") is True,
        "enable_moveit_execute": config.get("enable_moveit_execute") is True,
        "enable_real_robot_motion": config.get("enable_real_robot_motion") is True,
        "moveit_execute_called": executor.get("moveit_execute_called") is True,
        "trajectory_send_allowed": executor.get("trajectory_send_allowed") is True,
        "trajectory_sent": executor.get("trajectory_sent") is True,
        "controller_command_sent": executor.get("controller_command_sent") is True,
        "real_robot_motion_executed": executor.get("real_robot_motion_executed") is True,
        "return_home_requested": executor.get("return_home_requested") is True,
        "return_home_executed": executor.get("return_home_executed") is True,
        "return_home_skipped": executor.get("return_home_skipped") is True,
        "urscript_generated": False,
        "rtde_write_attempted": False,
        "dashboard_command_attempted": False,
        "raw_joint_targets_generated": False,
        "tcp_pose_world_generated_by_llm": False,
        "blocking_reasons": blocking_reasons,
        "warnings": warnings,
        "stages": stages,
        "normalizer_result": normalized,
        "perception_shadow_result": perception,
        "planner_gateway_shadow_result": planner,
        "ros2_interface_readiness_result": ros2,
        "ros2_message_export_result": message_export,
        "moveit_plan_result": moveit,
        "ur5_state_result": ur5_state,
        "robot_system_shadow_bridge_result": shadow_bridge,
        "manual_confirmation_result": manual_confirmation,
        "real_ur5_hover_executor_result": executor,
    }


def _not_requested_result() -> Dict[str, Any]:
    return {
        "contract_version": CONTRACT_VERSION,
        "schema_version": CONTRACT_VERSION,
        "teto_version": CURRENT_V3_HOVER_DEMO_VERSION,
        "v3_hover_demo_requested": False,
        "v3_hover_demo_evidence_available": False,
        "v3_hover_demo_status": "NOT_REQUESTED",
        "ok": False,
        "real_robot_motion_executed": False,
        "blocking_reasons": [],
        "warnings": [],
    }


def _perception_config(config: Dict[str, Any], user_command: str) -> Dict[str, Any]:
    perception = dict(_nested(config, "perception_shadow_pipeline") or _nested(config, "perception_shadow") or {})
    perception["user_command"] = user_command
    return perception


def _planner_config(config: Dict[str, Any]) -> Dict[str, Any]:
    planner = dict(_nested(config, "planner_gateway_shadow") or {})
    planner.setdefault("allowed_intents", ["hover_to_object"])
    planner.setdefault("intent_name", "hover_to_object")
    planner.setdefault("hover_offset_m", config.get("hover_offset_m", 0.08))
    planner.setdefault("max_speed_scale", config.get("max_speed_scale", 0.10))
    planner.setdefault("max_acc_scale", config.get("max_acc_scale", 0.10))
    planner.setdefault("confidence_threshold", config.get("confidence_threshold", 0.70))
    planner.setdefault("manual_confirmation_required", config.get("manual_confirmation_required", True))
    planner.setdefault("execution_allowed", False)
    planner.setdefault("workspace_bounds", _axis_workspace_bounds(config))
    return planner


def _ros2_config(config: Dict[str, Any]) -> Dict[str, Any]:
    ros2 = dict(_nested(config, "ros2_interface_readiness") or _nested(config, "ros2_interface") or {})
    ros2.setdefault("ros2_environment_declared", True)
    ros2.setdefault("ros_distro", "humble")
    ros2.setdefault("ros_domain_id", "unavailable")
    ros2.setdefault("allow_missing_ros2_runtime", config.get("enable_ros2_runtime") is not True)
    ros2.setdefault("require_ros2_runtime", config.get("enable_ros2_runtime") is True)
    ros2.setdefault(
        "planner_gateway_interface",
        {
            "mode": "topic",
            "topic_name": "/teto/planner_gateway/shadow_request",
            "message_schema": "teto_planner_gateway_shadow.v1",
        },
    )
    ros2.setdefault("frames", {"world_frame": "mock_world", "robot_base_frame": "base_link", "camera_frame": "camera_color_optical_frame"})
    ros2.setdefault("shadow_only", True)
    ros2.setdefault("ros2_publish_enabled", False)
    ros2.setdefault("moveit_enabled", False)
    ros2.setdefault("real_robot_enabled", False)
    ros2.setdefault("execution_allowed", False)
    return ros2


def _ros2_message_export_config(config: Dict[str, Any]) -> Dict[str, Any]:
    message = dict(_nested(config, "ros2_message_export") or {})
    message.setdefault("fake_publish_only", True)
    message.setdefault("message_schema", "teto_planner_gateway_shadow.v1")
    message.setdefault("robot_base_frame", "base_link")
    message.setdefault("camera_frame", "camera_color_optical_frame")
    return message


def _moveit_plan_config(config: Dict[str, Any]) -> Dict[str, Any]:
    moveit = dict(_nested(config, "moveit_plan_only") or _nested(config, "moveit_plan") or {})
    moveit.setdefault("plan_only", True)
    moveit.setdefault("planning_group", "ur_manipulator")
    moveit.setdefault("planning_frame", "base_link")
    moveit.setdefault("end_effector_frame", "tool0")
    moveit.setdefault("workspace_bounds", _axis_workspace_bounds(config))
    moveit.setdefault("moveit_execute_allowed", False)
    moveit.setdefault("trajectory_send_allowed", False)
    moveit.setdefault("execution_allowed", False)
    moveit.setdefault("real_robot_enabled", False)
    return moveit


def _ur5_state_config(config: Dict[str, Any]) -> Dict[str, Any]:
    state = dict(_nested(config, "ur5_read_only_state") or _nested(config, "ur5_state") or {})
    state.setdefault("read_only_mode", True)
    state.setdefault("robot_model", "UR5e")
    state.setdefault("robot_ip", "unavailable_for_shadow")
    state.setdefault("rtde_read_enabled", "declared_future_only")
    state.setdefault("dashboard_read_enabled", "declared_future_only")
    state.setdefault("state_ttl_ms", 500)
    state.setdefault("manual_confirmation_required", True)
    state.setdefault(
        "required_state_fields",
        [
            "robot_mode",
            "safety_status",
            "program_state",
            "speed_scaling",
            "protective_stop",
            "emergency_stop",
            "teach_mode",
            "remote_control_mode",
            "calibration_status",
        ],
    )
    return state


def _shadow_bridge_config(config: Dict[str, Any]) -> Dict[str, Any]:
    bridge = dict(_nested(config, "robot_system_shadow_bridge") or {})
    bridge.setdefault("shadow_bridge_only", True)
    bridge.setdefault("require_moveit_plan_only_ready", True)
    bridge.setdefault("require_ur5_read_only_contract_ready", True)
    bridge.setdefault("execution_allowed", False)
    return bridge


def _executor_config(config: Dict[str, Any], planner: Dict[str, Any], perception: Dict[str, Any]) -> Dict[str, Any]:
    executor = dict(_nested(config, "real_ur5_hover_executor") or {})
    for key in (
        "enable_ros2_runtime",
        "enable_live_camera",
        "enable_live_vlm",
        "enable_moveit_plan",
        "enable_moveit_execute",
        "enable_real_robot_motion",
        "manual_confirmation_required",
        "max_speed_scale",
        "max_acc_scale",
        "confidence_threshold",
        "workspace_bounds",
        "allow_return_home",
        "return_home_named_target",
    ):
        if key in config:
            executor.setdefault(key, config[key])
    executor.setdefault("workspace_check_passed", planner.get("workspace_check_passed"))
    executor.setdefault("target_depth_valid", perception.get("depth_value_m") is not None)
    executor.setdefault("tf_available", perception.get("world_point_m") is not None)
    executor.setdefault("scene_ttl_valid", planner.get("ttl_check_passed", True))
    executor.setdefault("confidence_overall", perception.get("overall_confidence"))
    executor.setdefault("bounded_target_point_m", planner.get("bounded_target_point_m"))
    executor.setdefault("moveit_plan_success", True)
    executor.setdefault("moveit_runtime_available", config.get("enable_moveit_plan") is True)
    executor.setdefault("ros2_runtime_available", config.get("enable_ros2_runtime") is True)
    executor.setdefault("moveit_execute_allowed", config.get("enable_moveit_execute") is True)
    executor.setdefault("robot_state_ok", config.get("robot_state_ok", False))
    executor.setdefault("safety_status_ok", config.get("safety_status_ok", False))
    executor.setdefault("protective_stop", config.get("protective_stop", False))
    executor.setdefault("emergency_stop", config.get("emergency_stop", False))
    executor.setdefault("speed_scaling", config.get("speed_scaling", 0.0))
    executor.setdefault("return_home_requested", config.get("return_home_requested", False))
    return executor


def _runtime_ur5_state(config: Dict[str, Any], ur5_state: Dict[str, Any]) -> Dict[str, Any]:
    state = dict(ur5_state)
    for key in ("robot_state_ok", "safety_status_ok", "protective_stop", "emergency_stop", "speed_scaling"):
        if key in config:
            state[key] = config[key]
    return state


def _confirmation_from_request(config: Dict[str, Any], request: V3HoverDemoRequest, planner: Dict[str, Any]) -> Dict[str, Any] | None:
    confirmation = config.get("manual_confirmation")
    if isinstance(confirmation, dict):
        return confirmation
    token = request.manual_confirmation_token
    if not token:
        return None
    return {
        "confirmation_token": token,
        "task_id": planner.get("task_id"),
        "target_label": planner.get("target_label"),
        "bounded_target_point_m": planner.get("bounded_target_point_m"),
        "understanding": config.get("manual_confirmation_phrase", "I understand this will move the real UR5"),
        "confirmed_at_epoch_s": datetime.now(timezone.utc).timestamp(),
    }


def _apply_runtime_flags(config: Dict[str, Any], request: V3HoverDemoRequest) -> Dict[str, Any]:
    updated = dict(config)
    for key in (
        "enable_live_camera",
        "enable_live_vlm",
        "enable_ros2_runtime",
        "enable_moveit_plan",
        "enable_moveit_execute",
        "enable_real_robot_motion",
    ):
        if getattr(request, key) is True:
            updated[key] = True
        else:
            updated.setdefault(key, False)
    return updated


def _merge_config(base: Dict[str, Any], override: Dict[str, Any] | None) -> Dict[str, Any]:
    merged = dict(base)
    if isinstance(override, dict):
        merged.update(override)
    return merged


def _nested(config: Dict[str, Any], key: str) -> Dict[str, Any]:
    value = config.get(key)
    return value if isinstance(value, dict) else {}


def _axis_workspace_bounds(config: Dict[str, Any]) -> Dict[str, list[float]]:
    raw = config.get("workspace_bounds") if isinstance(config.get("workspace_bounds"), dict) else {}
    if {"x_min", "x_max", "y_min", "y_max", "z_min", "z_max"}.issubset(raw):
        return {
            "x": [float(raw["x_min"]), float(raw["x_max"])],
            "y": [float(raw["y_min"]), float(raw["y_max"])],
            "z": [float(raw["z_min"]), float(raw["z_max"])],
        }
    return raw or {"x": [-1.0, 1.0], "y": [-1.0, 1.0], "z": [0.0, 2.0]}


def _append_stages(stages: list[str], result: Dict[str, Any], checks: list[tuple[str, Any, str]]) -> None:
    for field, expected, stage in checks:
        if result.get(field) == expected:
            stages.append(stage)


def _stage_blockers(result: Dict[str, Any]) -> list[str]:
    return [str(reason) for reason in _list(result.get("blocking_reasons")) if reason]


def _list(value: Any) -> list[Any]:
    if isinstance(value, list):
        return value
    if isinstance(value, tuple):
        return list(value)
    if value:
        return [value]
    return []


def _unique(values: list[str]) -> list[str]:
    output: list[str] = []
    seen: set[str] = set()
    for value in values:
        if value and value not in seen:
            seen.add(value)
            output.append(value)
    return output


def _timestamped_output_dir() -> Path:
    stamp = datetime.now().strftime("v3_hover_demo_%Y%m%d_%H%M%S")
    return DEFAULT_OUTPUT_ROOT / stamp


def _evidence_manifest(result: Dict[str, Any]) -> Dict[str, Any]:
    return {field: result.get(field) for field in V3_HOVER_DEMO_FIELDS} | {
        "v3_hover_demo_result_path": result.get("v3_hover_demo_result_path"),
        "v3_hover_demo_report_path": result.get("v3_hover_demo_report_path"),
        "summary_path": result.get("summary_path"),
    }


def _format_summary(result: Dict[str, Any]) -> str:
    return "\n".join(
        [
            "# V3.0.0 First Real UR5 Hover Demo Summary",
            "",
            f"- status: {_format_value(result.get('v3_hover_demo_status'))}",
            f"- mode: {_format_value(result.get('v3_demo_mode'))}",
            f"- user_command: {_format_value(result.get('user_command'))}",
            f"- normalized_intent: {_format_value(result.get('normalized_intent'))}",
            f"- target_label: {_format_value(result.get('target_label'))}",
            f"- real_robot_motion_executed: {_format_value(result.get('real_robot_motion_executed'))}",
            f"- blocking_reasons: {_format_value(result.get('blocking_reasons'))}",
            "",
        ]
    )


def _format_value(value: Any) -> str:
    if value is None:
        return "null"
    if isinstance(value, bool):
        return "true" if value else "false"
    if isinstance(value, (list, dict)):
        return json.dumps(value, ensure_ascii=False, sort_keys=True)
    return str(value)


def _string(value: Any) -> str | None:
    if isinstance(value, str):
        return value.strip() or None
    if value is None:
        return None
    return str(value).strip() or None
