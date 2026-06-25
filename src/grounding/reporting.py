from __future__ import annotations

import json
from typing import Any, Dict


VLM_GROUNDING_REPORT_FIELDS = (
    "vlm_grounding_requested",
    "vlm_grounding_status",
    "grounding_id",
    "snapshot_id",
    "scene_version",
    "user_command",
    "normalized_command",
    "adapter_mode",
    "target_label",
    "target_object_id",
    "bbox_xyxy",
    "pixel_center",
    "mask_ref",
    "semantic_confidence",
    "grounding_confidence",
    "overall_confidence",
    "grounded",
    "rejected",
    "rejection_reason",
    "error_code",
    "blocking_reasons",
    "warnings",
    "next_safe_action",
    "no_motion_grounding_passed",
    "live_vlm_called",
    "live_camera_used",
    "real_robot_motion_executed",
    "real_robot_command_enabled",
    "robot_command_generated",
    "trajectory_generated",
    "joint_targets_generated",
    "tcp_pose_world_generated",
)


def format_vlm_grounding_report(result: Dict[str, Any]) -> str:
    return "\n".join(
        [
            "# TETO V2.9.4 VLM Grounding Adapter Report",
            "",
            "## Overall Status",
            "",
            f"- vlm_grounding_status: {_format_value(result.get('vlm_grounding_status'))}",
            f"- adapter_mode: {_format_value(result.get('adapter_mode'))}",
            f"- grounding_id: {_format_value(result.get('grounding_id'))}",
            f"- snapshot_id: {_format_value(result.get('snapshot_id'))}",
            f"- scene_version: {_format_value(result.get('scene_version'))}",
            f"- user_command: {_format_value(result.get('user_command'))}",
            f"- normalized_command: {_format_value(result.get('normalized_command'))}",
            f"- no_motion_grounding_passed: {_format_value(result.get('no_motion_grounding_passed'))}",
            f"- blocking_reasons: {_format_value(result.get('blocking_reasons'))}",
            f"- warnings: {_format_value(result.get('warnings'))}",
            f"- next_safe_action: {_format_value(result.get('next_safe_action'))}",
            "",
            "## Grounding Result Fields",
            "",
            "| Field | Value |",
            "| --- | --- |",
            *[f"| {field} | {_format_value(result.get(field))} |" for field in VLM_GROUNDING_REPORT_FIELDS],
            "",
            "## No-Motion Safety Boundary",
            "",
            "V2.9.4 converts a text command plus a declared camera snapshot into a grounding result contract using mock, offline, manual, disabled, or future local Qwen declaration modes only. It is no-motion, no-live-VLM, no-real-robot, no-ROS2, and no-MoveIt evidence only. It does not open a live camera, does not call live Qwen or any live VLM, does not connect to a real UR5, does not use ROS2, MoveIt, RTDE, URScript, Dashboard, a trajectory planner, or tcp_pose_world execution, and does not generate joint targets, trajectories, robot commands, or real execution requests.",
            "",
            "| Safety Flag | Value |",
            "| --- | --- |",
            *[
                f"| {key} | {_format_value(value)} |"
                for key, value in sorted((result.get("safety_boundary") or {}).items())
            ],
            "",
        ]
    )


def _format_value(value: Any) -> str:
    if value is None:
        return "null"
    if isinstance(value, (dict, list)):
        return json.dumps(value, ensure_ascii=False, sort_keys=True)
    return str(value)


__all__ = ["format_vlm_grounding_report"]
