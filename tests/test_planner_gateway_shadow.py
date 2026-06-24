import json
import math

from scripts.harnesses.run_shadow_simulation_contract import build_parser
from src.planner_gateway_shadow import (
    E_EXECUTION_NOT_ALLOWED_IN_SHADOW,
    E_INVALID_WORLD_POINT,
    E_LIVE_CAMERA_DISABLED,
    E_LIVE_VLM_DISABLED,
    E_LOW_CONFIDENCE,
    E_MANUAL_CONFIRMATION_REQUIRED,
    E_MOVEIT_DISABLED,
    E_OUT_OF_WORKSPACE,
    E_PERCEPTION_NOT_READY,
    E_REAL_ROBOT_MOTION_NOT_ALLOWED,
    E_ROBOT_COMMAND_NOT_ALLOWED,
    E_ROS2_PUBLISH_DISABLED,
    E_SCENE_VERSION_MISSING,
    E_SNAPSHOT_ID_MISSING,
    E_UNSUPPORTED_INTENT,
    E_WORLD_FRAME_MISSING,
    E_WORLD_POINT_MISSING,
    PLANNER_INPUT_READY,
    STATUS_BLOCKED,
    STATUS_PASS,
    PlannerGatewayShadowRequest,
    build_planner_gateway_shadow_request,
    evaluate_planner_gateway_shadow,
    format_planner_gateway_shadow_report,
)
from src.simulation_runtime import run_first_simulation_execution


VALID_TASK = {
    "task_type": "hover_to_object",
    "target_label": "camera",
    "target_world_point": [0.2, 0.1, 0.4],
    "scene_version": "run_planner_shadow_item_001",
    "ttl_ms": 500,
}


def test_positive_perception_result_passes_and_is_planner_input_ready():
    result = _evaluate_example("examples/planner_gateway_shadow/perception_positive_result.json")

    assert result["planner_gateway_shadow_status"] == STATUS_PASS
    assert result["planner_input_status"] == PLANNER_INPUT_READY
    assert result["planner_input_ready"] is True
    assert result["intent_name"] == "hover_to_object"
    assert result["world_point_m"] == [-0.073333, -0.12, 1.0]
    assert result["bounded_target_point_m"] == [-0.073333, -0.12, 1.08]
    assert result["hover_offset_m"] == 0.08
    assert result["workspace_check_passed"] is True
    assert result["confidence_check_passed"] is True
    assert result["manual_confirmation_required"] is True
    assert result["execution_allowed"] is False
    assert result["ros2_publish_enabled"] is False
    assert result["ros2_publish_attempted"] is False
    assert result["moveit_called"] is False
    assert result["trajectory_generated"] is False
    assert result["tcp_pose_world_generated"] is False
    assert result["joint_targets_generated"] is False
    assert result["robot_command_generated"] is False
    assert result["real_robot_motion_executed"] is False
    assert result["shadow_request"]["mode"] == "shadow_only"
    assert result["shadow_request"]["execution_allowed"] is False
    assert "trajectory" not in json.dumps(result["shadow_request"])
    assert "tcp_pose_world" not in json.dumps(result["shadow_request"])


def test_perception_not_pass_blocks():
    perception = _positive_perception()
    perception["perception_shadow_status"] = "BLOCKED"

    result = _evaluate(perception=perception)

    assert result["planner_gateway_shadow_status"] == STATUS_BLOCKED
    assert E_PERCEPTION_NOT_READY in result["blocking_reasons"]


def test_missing_world_point_blocks():
    result = _evaluate_example("examples/planner_gateway_shadow/perception_missing_world_point.json")

    assert result["planner_gateway_shadow_status"] == STATUS_BLOCKED
    assert E_WORLD_POINT_MISSING in result["blocking_reasons"]


def test_invalid_world_point_blocks():
    for world_point in ([1.0, 2.0], [1.0, math.inf, 0.0], ["bad", 0.0, 0.0]):
        perception = _positive_perception()
        perception["world_point_m"] = world_point

        result = _evaluate(perception=perception)

        assert result["planner_gateway_shadow_status"] == STATUS_BLOCKED
        assert E_INVALID_WORLD_POINT in result["blocking_reasons"]


def test_missing_world_frame_blocks():
    perception = _positive_perception()
    perception.pop("world_frame")

    result = _evaluate(perception=perception, config={"world_frame": None})

    assert result["planner_gateway_shadow_status"] == STATUS_BLOCKED
    assert E_WORLD_FRAME_MISSING in result["blocking_reasons"]


def test_unsupported_intent_blocks():
    result = _evaluate_example(
        "examples/planner_gateway_shadow/perception_positive_result.json",
        config_path="configs/planner_gateway_shadow_unsupported_intent.example.yaml",
    )

    assert result["planner_gateway_shadow_status"] == STATUS_BLOCKED
    assert E_UNSUPPORTED_INTENT in result["blocking_reasons"]


def test_out_of_workspace_blocks():
    result = _evaluate_example("examples/planner_gateway_shadow/perception_out_of_workspace.json")

    assert result["planner_gateway_shadow_status"] == STATUS_BLOCKED
    assert E_OUT_OF_WORKSPACE in result["blocking_reasons"]


def test_low_confidence_blocks():
    result = _evaluate_example("examples/planner_gateway_shadow/perception_low_confidence.json")

    assert result["planner_gateway_shadow_status"] == STATUS_BLOCKED
    assert E_LOW_CONFIDENCE in result["blocking_reasons"]


def test_scene_version_and_snapshot_id_required():
    perception = _positive_perception()
    perception.pop("scene_version")
    perception.pop("snapshot_id")

    result = _evaluate(perception=perception)

    assert result["planner_gateway_shadow_status"] == STATUS_BLOCKED
    assert E_SCENE_VERSION_MISSING in result["blocking_reasons"]
    assert E_SNAPSHOT_ID_MISSING in result["blocking_reasons"]


def test_manual_confirmation_false_blocks():
    result = _evaluate(config={"manual_confirmation_required": False})

    assert result["planner_gateway_shadow_status"] == STATUS_BLOCKED
    assert E_MANUAL_CONFIRMATION_REQUIRED in result["blocking_reasons"]


def test_execution_allowed_true_blocks():
    result = _evaluate_example(
        "examples/planner_gateway_shadow/perception_positive_result.json",
        config_path="configs/planner_gateway_shadow_execution_requested.example.yaml",
    )

    assert result["planner_gateway_shadow_status"] == STATUS_BLOCKED
    assert E_EXECUTION_NOT_ALLOWED_IN_SHADOW in result["blocking_reasons"]
    assert result["execution_allowed"] is False


def test_ros2_publish_enabled_blocks():
    result = _evaluate_example(
        "examples/planner_gateway_shadow/perception_positive_result.json",
        config_path="configs/planner_gateway_shadow_ros2_publish_requested.example.yaml",
    )

    assert result["planner_gateway_shadow_status"] == STATUS_BLOCKED
    assert E_ROS2_PUBLISH_DISABLED in result["blocking_reasons"]
    assert result["ros2_publish_enabled"] is False
    assert result["ros2_publish_attempted"] is False


def test_moveit_called_blocks():
    result = _evaluate(config={"moveit_called": True})

    assert result["planner_gateway_shadow_status"] == STATUS_BLOCKED
    assert E_MOVEIT_DISABLED in result["blocking_reasons"]
    assert result["moveit_called"] is False


def test_trajectory_tcp_pose_joint_target_and_robot_command_fields_block():
    for field_name in ["trajectory", "tcp_pose_world", "joint_targets", "robot_command"]:
        perception = _positive_perception()
        perception[field_name] = [0.0]

        result = _evaluate(perception=perception)

        assert result["planner_gateway_shadow_status"] == STATUS_BLOCKED
        assert E_ROBOT_COMMAND_NOT_ALLOWED in result["blocking_reasons"]
        assert result["trajectory_generated"] is False
        assert result["tcp_pose_world_generated"] is False
        assert result["joint_targets_generated"] is False
        assert result["robot_command_generated"] is False


def test_live_camera_live_vlm_and_real_motion_flags_block():
    cases = [
        ("live_camera_used", E_LIVE_CAMERA_DISABLED),
        ("live_vlm_called", E_LIVE_VLM_DISABLED),
        ("real_robot_motion_executed", E_REAL_ROBOT_MOTION_NOT_ALLOWED),
    ]
    for flag, reason in cases:
        perception = _positive_perception()
        perception[flag] = True

        result = _evaluate(perception=perception)

        assert result["planner_gateway_shadow_status"] == STATUS_BLOCKED
        assert reason in result["blocking_reasons"]
        assert result[flag] is False


def test_runtime_manifest_contains_planner_gateway_shadow_evidence_fields(tmp_path):
    run_first_simulation_execution(
        VALID_TASK,
        steps=1,
        check_planner_gateway_shadow=True,
        planner_gateway_shadow_config="configs/planner_gateway_shadow_positive.example.yaml",
        planner_gateway_shadow_report=True,
        perception_shadow_result="examples/planner_gateway_shadow/perception_positive_result.json",
        output_dir=tmp_path,
        write_report=True,
    )

    manifest = json.loads((tmp_path / "evidence_manifest.json").read_text(encoding="utf-8"))
    summary = (tmp_path / "summary.md").read_text(encoding="utf-8")
    report = (tmp_path / "planner_gateway_shadow_report.md").read_text(encoding="utf-8")

    assert (tmp_path / "planner_gateway_shadow_result.json").exists()
    assert manifest["planner_gateway_shadow_evidence_available"] is True
    assert manifest["planner_gateway_shadow_status"] == STATUS_PASS
    assert manifest["gateway_request_id"] == "planner_shadow_positive_001"
    assert manifest["task_id"] == "planner_shadow_task_red_mug_001"
    assert manifest["user_command"] == "hover over the red mug"
    assert manifest["intent_name"] == "hover_to_object"
    assert manifest["target_label"] == "red_mug"
    assert manifest["snapshot_id"] == "perception_fixture_snapshot_001"
    assert manifest["grounding_id"] == "mock_grounding_red_mug_001"
    assert manifest["scene_version"] == "perception_fixture_scene_v1"
    assert manifest["world_frame"] == "base_link"
    assert manifest["world_point_m"] == [-0.073333, -0.12, 1.0]
    assert manifest["bounded_target_point_m"] == [-0.073333, -0.12, 1.08]
    assert manifest["hover_offset_m"] == 0.08
    assert manifest["workspace_check_passed"] is True
    assert manifest["confidence_check_passed"] is True
    assert manifest["planner_input_ready"] is True
    assert manifest["manual_confirmation_required"] is True
    assert manifest["execution_allowed"] is False
    assert manifest["ros2_publish_enabled"] is False
    assert manifest["ros2_publish_attempted"] is False
    assert manifest["moveit_called"] is False
    assert manifest["trajectory_generated"] is False
    assert manifest["tcp_pose_world_generated"] is False
    assert manifest["joint_targets_generated"] is False
    assert manifest["robot_command_generated"] is False
    assert manifest["real_robot_motion_executed"] is False
    assert manifest["blocking_reasons"] == []
    assert manifest["warnings"] == []
    assert manifest["replay_ready"] is True
    assert "planner_gateway_shadow_report.md" in [
        item["name"] for item in manifest["planner_gateway_shadow_evidence_files"]
    ]
    assert "## Planner Gateway Shadow Summary" in summary
    assert "planner_gateway_shadow_status: PASS" in summary
    assert "bounded_target_point_m: [-0.073333, -0.12, 1.08]" in summary
    assert "no-ROS2-publish" in report
    assert "no-MoveIt" in report
    assert "no-real-robot" in report
    assert "no-trajectory" in report


def test_report_safety_statement_mentions_disabled_execution_surfaces():
    report = format_planner_gateway_shadow_report(_evaluate())

    assert "no-ROS2-publish" in report
    assert "no-MoveIt" in report
    assert "no-real-robot" in report
    assert "no-trajectory" in report
    assert "tcp_pose_world" in report
    assert "joint targets" in report
    assert "robot commands" in report


def test_cli_parser_accepts_planner_gateway_shadow_flags():
    args = build_parser().parse_args(
        [
            "--check-planner-gateway-shadow",
            "--planner-gateway-shadow-config",
            "configs/planner_gateway_shadow_positive.example.yaml",
            "--planner-gateway-shadow-report",
            "--perception-shadow-result",
            "examples/planner_gateway_shadow/perception_positive_result.json",
        ]
    )

    assert args.check_planner_gateway_shadow is True
    assert args.planner_gateway_shadow_config == "configs/planner_gateway_shadow_positive.example.yaml"
    assert args.planner_gateway_shadow_report is True
    assert args.perception_shadow_result == "examples/planner_gateway_shadow/perception_positive_result.json"


def _evaluate_example(
    perception_path: str,
    *,
    config_path: str = "configs/planner_gateway_shadow_positive.example.yaml",
) -> dict:
    request = build_planner_gateway_shadow_request(
        requested=True,
        config_path=config_path,
        perception_shadow_result_path=perception_path,
    )
    return evaluate_planner_gateway_shadow(request)


def _evaluate(*, perception: dict | None = None, config: dict | None = None) -> dict:
    request = PlannerGatewayShadowRequest(
        requested=True,
        perception_shadow_result=perception or _positive_perception(),
        config=config or {
            "gateway_request_id": "planner_shadow_test_001",
            "task_id": "planner_shadow_task_test_001",
            "intent_name": "hover_to_object",
            "allowed_intents": ["hover_to_object"],
            "world_frame": "base_link",
            "hover_offset_m": 0.08,
            "confidence_threshold": 0.5,
            "manual_confirmation_required": True,
            "execution_allowed": False,
            "ros2_publish_enabled": False,
            "moveit_called": False,
        },
    )
    return evaluate_planner_gateway_shadow(request)


def _positive_perception() -> dict:
    return {
        "perception_shadow_requested": True,
        "requested": True,
        "perception_shadow_status": "PASS",
        "user_command": "hover over the red mug",
        "normalized_command": "hover over the red mug",
        "snapshot_id": "perception_fixture_snapshot_001",
        "grounding_id": "mock_grounding_red_mug_001",
        "scene_version": "perception_fixture_scene_v1",
        "target_label": "red_mug",
        "target_object_id": "mock_red_mug_001",
        "overall_confidence": 0.89,
        "semantic_gate_passed": True,
        "geometry_validity_status": "PASS",
        "projector_status": "PASS",
        "world_frame": "base_link",
        "camera_frame": "camera_color_optical_frame",
        "world_point_m": [-0.073333, -0.12, 1.0],
        "workspace_check_passed": True,
        "replay_ready": True,
        "live_camera_used": False,
        "live_vlm_called": False,
        "real_robot_motion_executed": False,
        "blocking_reasons": [],
        "warnings": [],
    }
