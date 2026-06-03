import json

from scripts.run_first_simulation_execution import build_parser
from src.planner_gateway_shadow import (
    PLANNER_INPUT_READY,
    build_planner_gateway_shadow_request,
    evaluate_planner_gateway_shadow,
)
from src.ros2_interface_readiness import (
    STATUS_READY_FOR_SHADOW_BRIDGE,
    ROS2InterfaceReadinessRequest,
    evaluate_ros2_interface_readiness,
)
from src.ros2_message_exporter import (
    E_BOUNDED_TARGET_POINT_MISSING,
    E_EXECUTION_NOT_ALLOWED_IN_FAKE_PUBLISH,
    E_FAKE_PUBLISH_ONLY_REQUIRED,
    E_INVALID_BOUNDED_TARGET_POINT,
    E_MESSAGE_SCHEMA_MISSING,
    E_MOVEIT_NOT_ALLOWED_IN_FAKE_PUBLISH,
    E_PLANNER_INPUT_MISSING,
    E_PLANNER_INPUT_NOT_READY,
    E_REAL_ROBOT_NOT_ALLOWED,
    E_ROS2_INTERFACE_NOT_READY,
    E_ROS2_PUBLISH_NOT_ALLOWED,
    MESSAGE_EXPORTED,
    STATUS_BLOCKED,
    STATUS_PASS,
    ROS2MessageExportRequest,
    evaluate_ros2_message_export,
    format_ros2_message_export_report,
)
from src.simulation_runtime import run_first_simulation_execution


VALID_TASK = {
    "task_type": "hover_to_object",
    "target_label": "camera",
    "target_world_point": [0.2, 0.1, 0.4],
    "scene_version": "run_ros2_message_export_item_001",
    "ttl_ms": 500,
}


def test_positive_fake_publish_export_passes_and_exports_message():
    result = _evaluate()

    assert result["ros2_message_export_status"] == STATUS_PASS
    assert result["message_export_status"] == MESSAGE_EXPORTED
    assert result["message_id"] == "ros2_fake_publish_planner_shadow_task_red_mug_001"
    assert result["message_schema"] == "teto_planner_gateway/PlannerRequest.v1"
    assert result["fake_publish_only"] is True
    assert result["ros2_publish_enabled"] is False
    assert result["ros2_publish_attempted"] is False
    assert result["execution_allowed"] is False
    assert result["moveit_called"] is False
    assert result["trajectory_generated"] is False
    assert result["tcp_pose_world_generated"] is False
    assert result["joint_targets_generated"] is False
    assert result["robot_command_generated"] is False
    assert result["real_robot_motion_executed"] is False
    assert result["blocking_reasons"] == []

    message = result["exported_message"]
    assert message["schema"] == "teto_planner_gateway/PlannerRequest.v1"
    assert message["task_id"] == "planner_shadow_task_red_mug_001"
    assert message["scene_version"] == "perception_fixture_scene_v1"
    assert message["intent_name"] == "hover_to_object"
    assert message["target_label"] == "red_mug"
    assert message["target_object_id"] == "mock_red_mug_001"
    assert message["world_frame"] == "base_link"
    assert message["robot_base_frame"] == "base_link"
    assert message["camera_frame"] == "camera_color_optical_frame"
    assert message["world_point_m"] == [-0.073333, -0.12, 1.0]
    assert message["bounded_target_point_m"] == [-0.073333, -0.12, 1.08]
    assert message["hover_offset_m"] == 0.08
    assert message["confidence_overall"] == 0.89
    assert message["ttl_ms"] == 500
    assert message["manual_confirmation_required"] is True
    assert message["execution_allowed"] is False
    assert message["fake_publish_only"] is True
    assert message["ros2_publish_enabled"] is False
    assert message["ros2_publish_attempted"] is False
    assert message["planner_gateway_interface_mode"] == "topic"
    assert message["planner_gateway_endpoint"] == "/teto/planner_gateway/shadow_request"
    assert message["created_from_version"] == "TETO V2.10.2"


def test_missing_planner_input_blocks():
    result = _evaluate(planner={})

    assert result["ros2_message_export_status"] == STATUS_BLOCKED
    assert E_PLANNER_INPUT_MISSING in result["blocking_reasons"]


def test_planner_input_not_ready_blocks():
    planner = _positive_planner()
    planner["planner_input_status"] = STATUS_BLOCKED
    planner["planner_input_ready"] = False

    result = _evaluate(planner=planner)

    assert result["ros2_message_export_status"] == STATUS_BLOCKED
    assert E_PLANNER_INPUT_NOT_READY in result["blocking_reasons"]


def test_missing_bounded_target_point_blocks():
    planner = _positive_planner()
    planner.pop("bounded_target_point_m")

    result = _evaluate(planner=planner)

    assert result["ros2_message_export_status"] == STATUS_BLOCKED
    assert E_BOUNDED_TARGET_POINT_MISSING in result["blocking_reasons"]


def test_invalid_bounded_target_point_blocks():
    for point in ([1.0, 2.0], [1.0, float("inf"), 0.0], ["bad", 0.0, 0.0]):
        planner = _positive_planner()
        planner["bounded_target_point_m"] = point

        result = _evaluate(planner=planner)

        assert result["ros2_message_export_status"] == STATUS_BLOCKED
        assert E_INVALID_BOUNDED_TARGET_POINT in result["blocking_reasons"]


def test_ros2_interface_not_ready_blocks():
    readiness = _positive_readiness()
    readiness["ros2_interface_readiness_status"] = STATUS_BLOCKED

    result = _evaluate(readiness=readiness)

    assert result["ros2_message_export_status"] == STATUS_BLOCKED
    assert E_ROS2_INTERFACE_NOT_READY in result["blocking_reasons"]


def test_missing_message_schema_blocks():
    readiness = _positive_readiness()
    readiness.pop("message_schema")
    config = _valid_config()
    config.pop("message_schema")

    result = _evaluate(readiness=readiness, config=config)

    assert result["ros2_message_export_status"] == STATUS_BLOCKED
    assert E_MESSAGE_SCHEMA_MISSING in result["blocking_reasons"]


def test_ros2_publish_enabled_blocks_and_result_stays_false():
    result = _evaluate(config={"ros2_publish_enabled": True})

    assert result["ros2_message_export_status"] == STATUS_BLOCKED
    assert E_ROS2_PUBLISH_NOT_ALLOWED in result["blocking_reasons"]
    assert result["ros2_publish_enabled"] is False
    assert result["ros2_publish_attempted"] is False


def test_fake_publish_only_false_blocks_and_result_stays_true():
    result = _evaluate(config={"fake_publish_only": False})

    assert result["ros2_message_export_status"] == STATUS_BLOCKED
    assert E_FAKE_PUBLISH_ONLY_REQUIRED in result["blocking_reasons"]
    assert result["fake_publish_only"] is True


def test_execution_allowed_true_blocks_and_result_stays_false():
    result = _evaluate(config={"execution_allowed": True})

    assert result["ros2_message_export_status"] == STATUS_BLOCKED
    assert E_EXECUTION_NOT_ALLOWED_IN_FAKE_PUBLISH in result["blocking_reasons"]
    assert result["execution_allowed"] is False


def test_moveit_enabled_true_blocks_and_result_stays_false():
    result = _evaluate(config={"moveit_enabled": True})

    assert result["ros2_message_export_status"] == STATUS_BLOCKED
    assert E_MOVEIT_NOT_ALLOWED_IN_FAKE_PUBLISH in result["blocking_reasons"]
    assert result["moveit_enabled"] is False
    assert result["moveit_called"] is False


def test_real_robot_enabled_true_blocks_and_result_stays_no_motion():
    result = _evaluate(config={"real_robot_enabled": True})

    assert result["ros2_message_export_status"] == STATUS_BLOCKED
    assert E_REAL_ROBOT_NOT_ALLOWED in result["blocking_reasons"]
    assert result["real_robot_motion_executed"] is False


def test_runtime_manifest_contains_ros2_message_export_evidence_fields(tmp_path):
    run_first_simulation_execution(
        VALID_TASK,
        steps=1,
        check_planner_gateway_shadow=True,
        planner_gateway_shadow_config="configs/planner_gateway_shadow_positive.example.yaml",
        planner_gateway_shadow_report=True,
        perception_shadow_result="examples/planner_gateway_shadow/perception_positive_result.json",
        check_ros2_interface_readiness=True,
        ros2_interface_config="configs/ros2_interface.example.yaml",
        ros2_interface_report=True,
        check_ros2_message_export=True,
        ros2_message_export_config="configs/ros2_message_export.example.yaml",
        ros2_message_export_report=True,
        output_dir=tmp_path,
        write_report=True,
    )

    manifest = json.loads((tmp_path / "evidence_manifest.json").read_text(encoding="utf-8"))
    summary = (tmp_path / "summary.md").read_text(encoding="utf-8")
    report = (tmp_path / "ros2_message_export_report.md").read_text(encoding="utf-8")
    result = json.loads((tmp_path / "ros2_message_export_result.json").read_text(encoding="utf-8"))

    assert manifest["ros2_message_export_evidence_available"] is True
    assert manifest["ros2_message_export_status"] == STATUS_PASS
    assert manifest["message_export_status"] == MESSAGE_EXPORTED
    assert manifest["message_id"] == "ros2_fake_publish_planner_shadow_task_red_mug_001"
    assert manifest["message_schema"] == "teto_planner_gateway/PlannerRequest.v1"
    assert manifest["fake_publish_only"] is True
    assert manifest["ros2_publish_enabled"] is False
    assert manifest["ros2_publish_attempted"] is False
    assert manifest["planner_gateway_interface_mode"] == "topic"
    assert manifest["planner_gateway_endpoint"] == "/teto/planner_gateway/shadow_request"
    assert manifest["bounded_target_point_m"] == [-0.073333, -0.12, 1.08]
    assert manifest["world_frame"] == "base_link"
    assert manifest["robot_base_frame"] == "base_link"
    assert manifest["camera_frame"] == "camera_color_optical_frame"
    assert manifest["execution_allowed"] is False
    assert manifest["moveit_called"] is False
    assert manifest["trajectory_generated"] is False
    assert manifest["tcp_pose_world_generated"] is False
    assert manifest["joint_targets_generated"] is False
    assert manifest["robot_command_generated"] is False
    assert manifest["real_robot_motion_executed"] is False
    assert manifest["blocking_reasons"] == []
    assert result["exported_message"]["fake_publish_only"] is True
    assert "## ROS2 Message Export / Fake Publish Summary" in summary
    assert "message_export_status: MESSAGE_EXPORTED" in summary
    assert "does not publish ROS2 messages" in report
    assert "does not call MoveIt" in report
    assert "does not control a real UR5" in report


def test_report_safety_statement_mentions_disabled_execution_surfaces():
    report = format_ros2_message_export_report(_evaluate())

    assert "does not publish ROS2 messages" in report
    assert "does not call rclpy publish" in report
    assert "does not call MoveIt" in report
    assert "does not generate trajectory" in report
    assert "does not control a real UR5" in report


def test_cli_parser_accepts_ros2_message_export_flags():
    args = build_parser().parse_args(
        [
            "--check-ros2-message-export",
            "--ros2-message-export-config",
            "configs/ros2_message_export.example.yaml",
            "--ros2-message-export-report",
        ]
    )

    assert args.check_ros2_message_export is True
    assert args.ros2_message_export_config == "configs/ros2_message_export.example.yaml"
    assert args.ros2_message_export_report is True


def _evaluate(
    *,
    planner: dict | None = None,
    readiness: dict | None = None,
    config: dict | None = None,
) -> dict:
    request = ROS2MessageExportRequest(
        requested=True,
        planner_gateway_shadow_result=_positive_planner() if planner is None else planner,
        ros2_interface_readiness_result=_positive_readiness() if readiness is None else readiness,
        config=_merged_config(config),
    )
    return evaluate_ros2_message_export(request)


def _positive_planner() -> dict:
    request = build_planner_gateway_shadow_request(
        requested=True,
        config_path="configs/planner_gateway_shadow_positive.example.yaml",
        perception_shadow_result_path="examples/planner_gateway_shadow/perception_positive_result.json",
    )
    result = evaluate_planner_gateway_shadow(request)
    assert result["planner_input_status"] == PLANNER_INPUT_READY
    return result


def _positive_readiness() -> dict:
    request = ROS2InterfaceReadinessRequest(
        requested=True,
        config={
            "ros2_environment_declared": True,
            "ros_distro": "humble",
            "ros_domain_id": "unavailable",
            "allow_missing_ros2_runtime": True,
            "planner_gateway_interface": {
                "mode": "topic",
                "topic_name": "/teto/planner_gateway/shadow_request",
                "message_schema": "teto_planner_gateway_shadow.v1",
            },
            "frames": {
                "world_frame": "base_link",
                "robot_base_frame": "base_link",
                "camera_frame": "camera_color_optical_frame",
            },
            "shadow_only": True,
            "ros2_publish_enabled": False,
            "ros2_publish_attempted": False,
            "moveit_enabled": False,
            "moveit_called": False,
            "real_robot_enabled": False,
            "execution_allowed": False,
        },
        environ={"ROS_DISTRO": "humble"},
    )
    result = evaluate_ros2_interface_readiness(request)
    assert result["ros2_interface_readiness_status"] == STATUS_READY_FOR_SHADOW_BRIDGE
    return result


def _valid_config() -> dict:
    return {
        "message_schema": "teto_planner_gateway/PlannerRequest.v1",
        "ttl_ms": 500,
        "fake_publish_only": True,
        "ros2_publish_enabled": False,
        "ros2_publish_attempted": False,
        "execution_allowed": False,
        "moveit_enabled": False,
        "real_robot_enabled": False,
    }


def _merged_config(config: dict | None) -> dict:
    if config is None:
        return _valid_config()
    if "fake_publish_only" in config:
        return config
    merged = _valid_config()
    merged.update(config)
    return merged
