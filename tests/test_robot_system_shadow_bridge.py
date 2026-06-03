import json

from scripts.run_first_simulation_execution import build_parser
from src.moveit_plan_only_contract import PLAN_ONLY_READY, MoveItPlanOnlyRequest, evaluate_moveit_plan_only
from src.planner_gateway_shadow import (
    PLANNER_INPUT_READY,
    build_planner_gateway_shadow_request,
    evaluate_planner_gateway_shadow,
)
from src.robot_system_shadow_bridge import (
    E_MOVEIT_PLAN_ONLY_NOT_READY,
    E_ROS2_PUBLISH_ATTEMPTED,
    E_TRAJECTORY_SEND_ATTEMPTED,
    ROBOT_SYSTEM_SHADOW_READY,
    STATUS_BLOCKED,
    STATUS_PASS,
    RobotSystemShadowBridgeRequest,
    evaluate_robot_system_shadow_bridge,
    format_robot_system_shadow_bridge_report,
)
from src.ros2_interface_readiness import (
    ROS2InterfaceReadinessRequest,
    STATUS_READY_FOR_SHADOW_BRIDGE,
    evaluate_ros2_interface_readiness,
)
from src.ros2_message_exporter import (
    MESSAGE_EXPORTED,
    ROS2MessageExportRequest,
    evaluate_ros2_message_export,
)
from src.simulation_runtime import CURRENT_TETO_VERSION, run_first_simulation_execution
from src.ur5_read_only_state_contract import (
    READ_ONLY_STATE_CONTRACT_READY,
    REQUIRED_STATE_FIELDS,
    UR5ReadOnlyStateRequest,
    evaluate_ur5_read_only_state,
)


VALID_TASK = {
    "task_type": "hover_to_object",
    "target_label": "camera",
    "target_world_point": [0.2, 0.1, 0.4],
    "scene_version": "run_robot_system_shadow_bridge_001",
    "ttl_ms": 500,
}


REQUIRED_FALSE_FLAGS = (
    "execution_allowed",
    "ros2_publish_enabled",
    "ros2_publish_attempted",
    "moveit_execute_allowed",
    "moveit_execute_called",
    "trajectory_generated",
    "trajectory_send_allowed",
    "trajectory_sent",
    "controller_command_sent",
    "tcp_pose_world_generated",
    "joint_targets_generated",
    "robot_command_generated",
    "real_robot_enabled",
    "real_robot_motion_executed",
    "rtde_write_enabled",
    "rtde_write_attempted",
    "dashboard_command_enabled",
    "dashboard_command_attempted",
    "urscript_generated",
    "automatic_retry_motion",
)


def test_positive_robot_system_shadow_bridge_passes_without_motion_surfaces():
    result = _evaluate()

    assert result["robot_system_shadow_bridge_status"] == STATUS_PASS
    assert result["robot_system_shadow_status"] == ROBOT_SYSTEM_SHADOW_READY
    assert result["robot_system_shadow_ready"] is True
    assert result["ros2_message_export_ready"] is True
    assert result["moveit_plan_only_ready"] is True
    assert result["ur5_read_only_state_ready"] is True
    assert result["message_id"] == "ros2_fake_publish_planner_shadow_task_red_mug_001"
    assert result["planning_group"] == "ur_manipulator"
    assert result["robot_model"] == "UR5e"
    assert result["manual_confirmation_required"] is True
    assert result["blocking_reasons"] == []
    for flag in REQUIRED_FALSE_FLAGS:
        assert result[flag] is False


def test_bridge_blocks_if_moveit_plan_only_not_ready():
    moveit = _positive_moveit_plan_only()
    moveit["plan_only_ready"] = False

    result = _evaluate(moveit_plan_only=moveit)

    assert result["robot_system_shadow_bridge_status"] == STATUS_BLOCKED
    assert E_MOVEIT_PLAN_ONLY_NOT_READY in result["blocking_reasons"]
    assert result["moveit_execute_called"] is False
    assert result["trajectory_sent"] is False


def test_bridge_blocks_attempted_ros2_publish_but_result_stays_false():
    result = _evaluate(config={"ros2_publish_attempted": True})

    assert result["robot_system_shadow_bridge_status"] == STATUS_BLOCKED
    assert E_ROS2_PUBLISH_ATTEMPTED in result["blocking_reasons"]
    assert result["ros2_publish_enabled"] is False
    assert result["ros2_publish_attempted"] is False


def test_bridge_blocks_trajectory_send_attempt_but_result_stays_false():
    result = _evaluate(config={"trajectory_sent": True})

    assert result["robot_system_shadow_bridge_status"] == STATUS_BLOCKED
    assert E_TRAJECTORY_SEND_ATTEMPTED in result["blocking_reasons"]
    assert result["trajectory_send_allowed"] is False
    assert result["trajectory_sent"] is False
    assert result["controller_command_sent"] is False


def test_robot_system_shadow_bridge_report_states_disabled_execution_surfaces():
    report = format_robot_system_shadow_bridge_report(_evaluate())

    assert "TETO V2.11.0 Full Robot-System Shadow Bridge Report" in report
    assert "does not publish ROS2 commands" in report
    assert "execute MoveIt plans" in report
    assert "send controller trajectories" in report
    assert "UR driver/RTDE/Dashboard/URScript commands" in report
    assert "move a real UR5" in report


def test_runtime_manifest_contains_full_robot_system_shadow_bridge_evidence(tmp_path):
    result = run_first_simulation_execution(
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
        check_moveit_plan_only=True,
        moveit_plan_only_config="configs/moveit_plan_only.example.yaml",
        moveit_plan_only_report=True,
        check_ur5_read_only_state=True,
        ur5_read_only_state_config="configs/ur5_read_only_state.example.yaml",
        ur5_read_only_state_report=True,
        check_robot_system_shadow_bridge=True,
        robot_system_shadow_bridge_config="configs/robot_system_shadow_bridge.example.yaml",
        robot_system_shadow_bridge_report=True,
        output_dir=tmp_path,
        write_report=True,
    )

    manifest = json.loads((tmp_path / "evidence_manifest.json").read_text(encoding="utf-8"))
    summary = (tmp_path / "summary.md").read_text(encoding="utf-8")
    moveit_result = json.loads((tmp_path / "moveit_plan_only_result.json").read_text(encoding="utf-8"))
    ur5_result = json.loads((tmp_path / "ur5_read_only_state_result.json").read_text(encoding="utf-8"))
    bridge_result = json.loads(
        (tmp_path / "robot_system_shadow_bridge_result.json").read_text(encoding="utf-8")
    )
    bridge_report = (tmp_path / "robot_system_shadow_bridge_report.md").read_text(encoding="utf-8")

    assert result["teto_version"] == CURRENT_TETO_VERSION
    assert result["mode"] == "no_isaac"
    assert manifest["teto_version"] == CURRENT_TETO_VERSION
    assert manifest["moveit_plan_only_evidence_available"] is True
    assert manifest["moveit_plan_only_status"] == STATUS_PASS
    assert manifest["plan_only_ready"] is True
    assert manifest["ur5_read_only_state_evidence_available"] is True
    assert manifest["ur5_read_only_state_status"] == STATUS_PASS
    assert manifest["read_only_state_contract_ready"] is True
    assert manifest["robot_system_shadow_bridge_evidence_available"] is True
    assert manifest["robot_system_shadow_bridge_status"] == STATUS_PASS
    assert manifest["robot_system_shadow_ready"] is True
    assert manifest["ros2_message_export_ready"] is True
    assert manifest["moveit_plan_only_ready"] is True
    assert manifest["ur5_read_only_state_ready"] is True
    assert manifest["moveit_plan_requested"] is True
    assert manifest["moveit_plan_only"] is True
    for flag in REQUIRED_FALSE_FLAGS:
        assert manifest[flag] is False
        assert bridge_result[flag] is False
    assert manifest["blocking_reasons"] == []
    assert manifest["manual_confirmation_required"] is True
    assert moveit_result["plan_only_status"] == PLAN_ONLY_READY
    assert ur5_result["read_only_state_status"] == READ_ONLY_STATE_CONTRACT_READY
    assert bridge_result["robot_system_shadow_status"] == ROBOT_SYSTEM_SHADOW_READY
    assert "## MoveIt Plan-Only Contract Summary" in summary
    assert "## UR5 Read-Only State Contract Summary" in summary
    assert "## Full Robot-System Shadow Bridge Summary" in summary
    assert "does not publish ROS2 commands" in bridge_report


def test_cli_parser_accepts_robot_system_shadow_bridge_flags():
    args = build_parser().parse_args(
        [
            "--check-robot-system-shadow-bridge",
            "--robot-system-shadow-bridge-config",
            "configs/robot_system_shadow_bridge.example.yaml",
            "--robot-system-shadow-bridge-report",
        ]
    )

    assert args.check_robot_system_shadow_bridge is True
    assert args.robot_system_shadow_bridge_config == "configs/robot_system_shadow_bridge.example.yaml"
    assert args.robot_system_shadow_bridge_report is True


def _evaluate(
    *,
    message_export: dict | None = None,
    moveit_plan_only: dict | None = None,
    ur5_read_only_state: dict | None = None,
    config: dict | None = None,
) -> dict:
    request = RobotSystemShadowBridgeRequest(
        requested=True,
        ros2_message_export_result=_positive_message_export() if message_export is None else message_export,
        moveit_plan_only_result=_positive_moveit_plan_only()
        if moveit_plan_only is None
        else moveit_plan_only,
        ur5_read_only_state_result=_positive_ur5_read_only_state()
        if ur5_read_only_state is None
        else ur5_read_only_state,
        config=_merged_config(config),
    )
    return evaluate_robot_system_shadow_bridge(request)


def _positive_message_export() -> dict:
    planner = evaluate_planner_gateway_shadow(
        build_planner_gateway_shadow_request(
            requested=True,
            config_path="configs/planner_gateway_shadow_positive.example.yaml",
            perception_shadow_result_path="examples/planner_gateway_shadow/perception_positive_result.json",
        )
    )
    assert planner["planner_input_status"] == PLANNER_INPUT_READY

    readiness = evaluate_ros2_interface_readiness(
        ROS2InterfaceReadinessRequest(
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
    )
    assert readiness["ros2_interface_readiness_status"] == STATUS_READY_FOR_SHADOW_BRIDGE

    export = evaluate_ros2_message_export(
        ROS2MessageExportRequest(
            requested=True,
            planner_gateway_shadow_result=planner,
            ros2_interface_readiness_result=readiness,
            config={
                "message_schema": "teto_planner_gateway/PlannerRequest.v1",
                "ttl_ms": 500,
                "fake_publish_only": True,
                "ros2_publish_enabled": False,
                "ros2_publish_attempted": False,
                "execution_allowed": False,
                "moveit_enabled": False,
                "real_robot_enabled": False,
            },
        )
    )
    assert export["message_export_status"] == MESSAGE_EXPORTED
    return export


def _positive_moveit_plan_only() -> dict:
    result = evaluate_moveit_plan_only(
        MoveItPlanOnlyRequest(
            requested=True,
            ros2_message_export_result=_positive_message_export(),
            config={
                "plan_only": True,
                "planning_group": "ur_manipulator",
                "planning_frame": "base_link",
                "end_effector_frame": "tool0",
                "moveit_execute_allowed": False,
                "moveit_execute_called": False,
                "trajectory_send_allowed": False,
                "trajectory_sent": False,
                "controller_command_sent": False,
                "execution_allowed": False,
                "real_robot_enabled": False,
                "workspace_bounds": {
                    "x": [-1.0, 1.0],
                    "y": [-1.0, 1.0],
                    "z": [0.0, 2.0],
                },
            },
        )
    )
    assert result["plan_only_status"] == PLAN_ONLY_READY
    return result


def _positive_ur5_read_only_state() -> dict:
    result = evaluate_ur5_read_only_state(
        UR5ReadOnlyStateRequest(
            requested=True,
            config={
                "read_only_mode": True,
                "robot_model": "UR5e",
                "robot_ip": "unavailable_for_shadow",
                "rtde_read_enabled": "declared_future_only",
                "rtde_write_enabled": False,
                "rtde_write_attempted": False,
                "dashboard_read_enabled": "declared_future_only",
                "dashboard_command_enabled": False,
                "dashboard_command_attempted": False,
                "state_ttl_ms": 500,
                "manual_confirmation_required": True,
                "execution_allowed": False,
                "real_robot_enabled": False,
                "required_state_fields": list(REQUIRED_STATE_FIELDS),
            },
        )
    )
    assert result["read_only_state_status"] == READ_ONLY_STATE_CONTRACT_READY
    return result


def _valid_config() -> dict:
    return {
        "shadow_bridge_only": True,
        "require_moveit_plan_only_ready": True,
        "require_ur5_read_only_contract_ready": True,
        "execution_allowed": False,
        "ros2_publish_enabled": False,
        "ros2_publish_attempted": False,
        "moveit_execute_allowed": False,
        "moveit_execute_called": False,
        "trajectory_send_allowed": False,
        "trajectory_sent": False,
        "controller_command_sent": False,
        "robot_command_generated": False,
        "tcp_pose_world_generated": False,
        "joint_targets_generated": False,
        "real_robot_enabled": False,
        "real_robot_motion_allowed": False,
        "real_robot_motion_executed": False,
        "rtde_write_attempted": False,
        "dashboard_command_attempted": False,
        "automatic_retry_motion": False,
    }


def _merged_config(config: dict | None) -> dict:
    merged = _valid_config()
    if config:
        merged.update(config)
    return merged
