import json

from scripts.harnesses.run_shadow_simulation_contract import build_parser
from src.ros2_interface_readiness import (
    E_FRAME_DECLARATION_MISSING,
    E_MOVEIT_NOT_ALLOWED_IN_READINESS,
    E_PLANNER_GATEWAY_ENDPOINT_MISSING,
    E_REAL_ROBOT_NOT_ALLOWED,
    E_ROS2_PUBLISH_NOT_ALLOWED,
    E_ROS_DISTRO_UNDECLARED,
    STATUS_BLOCKED,
    STATUS_READY_FOR_SHADOW_BRIDGE,
    W_ROS2_RUNTIME_UNAVAILABLE,
    ROS2InterfaceReadinessRequest,
    evaluate_ros2_interface_readiness,
    format_ros2_interface_readiness_report,
)
from src.simulation_runtime import run_first_simulation_execution


VALID_TASK = {
    "task_type": "hover_to_object",
    "target_label": "camera",
    "target_world_point": [0.2, 0.1, 0.4],
    "scene_version": "run_ros2_interface_item_001",
    "ttl_ms": 500,
}


def test_positive_shadow_only_config_ready_for_shadow_bridge():
    result = _evaluate()

    assert result["ros2_interface_readiness_status"] == STATUS_READY_FOR_SHADOW_BRIDGE
    assert result["ros2_environment_declared"] is True
    assert result["ros_distro"] == "humble"
    assert result["ros_domain_id"] == "unavailable"
    assert result["planner_gateway_interface_mode"] == "topic"
    assert result["planner_gateway_endpoint"] == "/teto/planner_gateway/shadow_request"
    assert result["message_schema"] == "teto_planner_gateway_shadow.v1"
    assert result["world_frame"] == "world"
    assert result["robot_base_frame"] == "base_link"
    assert result["camera_frame"] == "camera_color_optical_frame"
    assert result["shadow_only"] is True
    assert result["ros2_publish_enabled"] is False
    assert result["ros2_publish_attempted"] is False
    assert result["moveit_enabled"] is False
    assert result["moveit_called"] is False
    assert result["execution_allowed"] is False
    assert result["trajectory_generated"] is False
    assert result["tcp_pose_world_generated"] is False
    assert result["joint_targets_generated"] is False
    assert result["robot_command_generated"] is False
    assert result["real_robot_motion_executed"] is False
    assert result["blocking_reasons"] == []


def test_missing_ros_distro_blocks():
    config = _valid_config()
    config.pop("ros_distro")

    result = _evaluate(config)

    assert result["ros2_interface_readiness_status"] == STATUS_BLOCKED
    assert E_ROS_DISTRO_UNDECLARED in result["blocking_reasons"]


def test_missing_planner_gateway_endpoint_blocks():
    config = _valid_config()
    config["planner_gateway_interface"].pop("topic_name")

    result = _evaluate(config)

    assert result["ros2_interface_readiness_status"] == STATUS_BLOCKED
    assert E_PLANNER_GATEWAY_ENDPOINT_MISSING in result["blocking_reasons"]


def test_ros2_publish_enabled_blocks_and_result_stays_false():
    result = _evaluate({"ros2_publish_enabled": True})

    assert result["ros2_interface_readiness_status"] == STATUS_BLOCKED
    assert E_ROS2_PUBLISH_NOT_ALLOWED in result["blocking_reasons"]
    assert result["ros2_publish_enabled"] is False
    assert result["ros2_publish_attempted"] is False


def test_moveit_enabled_blocks_and_result_stays_false():
    result = _evaluate({"moveit_enabled": True})

    assert result["ros2_interface_readiness_status"] == STATUS_BLOCKED
    assert E_MOVEIT_NOT_ALLOWED_IN_READINESS in result["blocking_reasons"]
    assert result["moveit_enabled"] is False
    assert result["moveit_called"] is False


def test_real_robot_enabled_blocks_and_result_stays_no_motion():
    result = _evaluate({"real_robot_enabled": True})

    assert result["ros2_interface_readiness_status"] == STATUS_BLOCKED
    assert E_REAL_ROBOT_NOT_ALLOWED in result["blocking_reasons"]
    assert result["real_robot_enabled"] is False
    assert result["real_robot_motion_executed"] is False


def test_missing_required_frame_names_block():
    config = _valid_config()
    config["frames"].pop("world_frame")

    result = _evaluate(config)

    assert result["ros2_interface_readiness_status"] == STATUS_BLOCKED
    assert E_FRAME_DECLARATION_MISSING in result["blocking_reasons"]
    assert "world_frame" in result["missing_frames"]


def test_ros2_runtime_unavailable_allowed_warns_without_blocking():
    result = _evaluate(environ={})

    assert result["ros2_interface_readiness_status"] == STATUS_READY_FOR_SHADOW_BRIDGE
    assert W_ROS2_RUNTIME_UNAVAILABLE in result["warnings"]
    assert result["ros2_publish_attempted"] is False
    assert result["real_robot_motion_executed"] is False


def test_runtime_manifest_contains_ros2_interface_readiness_evidence_fields(tmp_path):
    run_first_simulation_execution(
        VALID_TASK,
        steps=1,
        check_ros2_interface_readiness=True,
        ros2_interface_config="configs/ros2_interface.example.yaml",
        ros2_interface_report=True,
        output_dir=tmp_path,
        write_report=True,
    )

    manifest = json.loads((tmp_path / "evidence_manifest.json").read_text(encoding="utf-8"))
    summary = (tmp_path / "summary.md").read_text(encoding="utf-8")
    report = (tmp_path / "ros2_interface_readiness_report.md").read_text(encoding="utf-8")

    assert (tmp_path / "ros2_interface_readiness_result.json").exists()
    assert manifest["ros2_interface_readiness_evidence_available"] is True
    assert manifest["ros2_interface_readiness_status"] == STATUS_READY_FOR_SHADOW_BRIDGE
    assert manifest["ros2_environment_declared"] is True
    assert manifest["ros_distro"] == "humble"
    assert manifest["ros_domain_id"] == "unavailable"
    assert manifest["planner_gateway_interface_mode"] == "topic"
    assert manifest["planner_gateway_endpoint"] == "/teto/planner_gateway/shadow_request"
    assert manifest["message_schema"] == "teto_planner_gateway_shadow.v1"
    assert manifest["world_frame"] == "world"
    assert manifest["robot_base_frame"] == "base_link"
    assert manifest["camera_frame"] == "camera_color_optical_frame"
    assert manifest["shadow_only"] is True
    assert manifest["ros2_publish_enabled"] is False
    assert manifest["ros2_publish_attempted"] is False
    assert manifest["moveit_enabled"] is False
    assert manifest["moveit_called"] is False
    assert manifest["execution_allowed"] is False
    assert manifest["trajectory_generated"] is False
    assert manifest["tcp_pose_world_generated"] is False
    assert manifest["joint_targets_generated"] is False
    assert manifest["robot_command_generated"] is False
    assert manifest["real_robot_motion_executed"] is False
    assert manifest["blocking_reasons"] == []
    assert "## ROS2 Interface Readiness Summary" in summary
    assert "ros2_interface_readiness_status: READY_FOR_SHADOW_BRIDGE" in summary
    assert "does not publish ROS2 messages" in report
    assert "does not call MoveIt" in report
    assert "does not connect to a real UR5" in report


def test_report_safety_statement_mentions_disabled_execution_surfaces():
    report = format_ros2_interface_readiness_report(_evaluate())

    assert "does not publish ROS2 topics" in report
    assert "does not call MoveIt" in report
    assert "does not connect to a real UR5" in report
    assert "tcp_pose_world" in report
    assert "joint targets" in report
    assert "robot commands" in report


def test_cli_parser_accepts_ros2_interface_readiness_flags():
    args = build_parser().parse_args(
        [
            "--check-ros2-interface-readiness",
            "--ros2-interface-config",
            "configs/ros2_interface.example.yaml",
            "--ros2-interface-report",
        ]
    )

    assert args.check_ros2_interface_readiness is True
    assert args.ros2_interface_config == "configs/ros2_interface.example.yaml"
    assert args.ros2_interface_report is True


def _evaluate(config: dict | None = None, *, environ: dict | None = None) -> dict:
    if config and "frames" in config:
        base_config = config
    else:
        base_config = _valid_config()
    if config and "frames" not in config:
        base_config.update(config)
    request = ROS2InterfaceReadinessRequest(
        requested=True,
        config=base_config,
        environ={"ROS_DISTRO": "humble"} if environ is None else environ,
    )
    return evaluate_ros2_interface_readiness(request)


def _valid_config() -> dict:
    return {
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
            "world_frame": "world",
            "robot_base_frame": "base_link",
            "camera_frame": "camera_color_optical_frame",
            "target_frame": "target_object",
        },
        "shadow_only": True,
        "ros2_publish_enabled": False,
        "ros2_publish_attempted": False,
        "moveit_enabled": False,
        "moveit_called": False,
        "real_robot_enabled": False,
        "execution_allowed": False,
    }
