from scripts.harnesses.run_shadow_simulation_contract import build_parser
from src.moveit_plan_only_contract import (
    E_MOVEIT_EXECUTION_NOT_ALLOWED,
    E_OUT_OF_WORKSPACE,
    E_ROS2_MESSAGE_EXPORT_NOT_READY,
    PLAN_ONLY_READY,
    STATUS_BLOCKED,
    STATUS_PASS,
    MoveItPlanOnlyRequest,
    evaluate_moveit_plan_only,
    format_moveit_plan_only_report,
)
from src.planner_gateway_shadow import (
    PLANNER_INPUT_READY,
    build_planner_gateway_shadow_request,
    evaluate_planner_gateway_shadow,
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


def test_positive_moveit_plan_only_contract_passes_without_motion():
    result = _evaluate()

    assert result["moveit_plan_only_status"] == STATUS_PASS
    assert result["plan_only_status"] == PLAN_ONLY_READY
    assert result["plan_only_ready"] is True
    assert result["planning_group"] == "ur_manipulator"
    assert result["planning_frame"] == "base_link"
    assert result["end_effector_frame"] == "tool0"
    assert result["bounded_target_point_m"] == [-0.073333, -0.12, 1.08]
    assert result["moveit_plan_requested"] is True
    assert result["moveit_plan_only"] is True
    assert result["moveit_execute_allowed"] is False
    assert result["moveit_execute_called"] is False
    assert result["trajectory_generated"] is False
    assert result["trajectory_send_allowed"] is False
    assert result["trajectory_sent"] is False
    assert result["controller_command_sent"] is False
    assert result["execution_allowed"] is False
    assert result["tcp_pose_world_generated"] is False
    assert result["joint_targets_generated"] is False
    assert result["robot_command_generated"] is False
    assert result["real_robot_enabled"] is False
    assert result["real_robot_motion_executed"] is False
    assert result["blocking_reasons"] == []
    assert result["bounded_plan_goal"]["non_executable"] is True


def test_moveit_execute_request_blocks_but_result_stays_no_motion():
    result = _evaluate(config={"moveit_execute_allowed": True, "moveit_execute_called": True})

    assert result["moveit_plan_only_status"] == STATUS_BLOCKED
    assert E_MOVEIT_EXECUTION_NOT_ALLOWED in result["blocking_reasons"]
    assert result["moveit_execute_allowed"] is False
    assert result["moveit_execute_called"] is False
    assert result["trajectory_sent"] is False
    assert result["execution_allowed"] is False


def test_out_of_workspace_target_blocks_without_generating_trajectory():
    export = _positive_message_export()
    export["bounded_target_point_m"] = [5.0, 0.0, 1.0]

    result = _evaluate(message_export=export)

    assert result["moveit_plan_only_status"] == STATUS_BLOCKED
    assert E_OUT_OF_WORKSPACE in result["blocking_reasons"]
    assert result["trajectory_generated"] is False
    assert result["controller_command_sent"] is False


def test_message_export_not_ready_blocks():
    export = _positive_message_export()
    export["ros2_message_export_status"] = STATUS_BLOCKED

    result = _evaluate(message_export=export)

    assert result["moveit_plan_only_status"] == STATUS_BLOCKED
    assert E_ROS2_MESSAGE_EXPORT_NOT_READY in result["blocking_reasons"]


def test_moveit_plan_only_report_states_disabled_execution_surfaces():
    report = format_moveit_plan_only_report(_evaluate())

    assert "TETO V2.11.0 MoveIt Plan-Only Contract Report" in report
    assert "does not execute a MoveIt plan" in report
    assert "send a trajectory" in report
    assert "generate tcp_pose_world" in report
    assert "command a real UR5" in report


def test_cli_parser_accepts_moveit_plan_only_flags():
    args = build_parser().parse_args(
        [
            "--check-moveit-plan-only",
            "--moveit-plan-only-config",
            "configs/moveit_plan_only.example.yaml",
            "--moveit-plan-only-report",
        ]
    )

    assert args.check_moveit_plan_only is True
    assert args.moveit_plan_only_config == "configs/moveit_plan_only.example.yaml"
    assert args.moveit_plan_only_report is True


def _evaluate(*, message_export: dict | None = None, config: dict | None = None) -> dict:
    request = MoveItPlanOnlyRequest(
        requested=True,
        ros2_message_export_result=_positive_message_export() if message_export is None else message_export,
        config=_merged_config(config),
    )
    return evaluate_moveit_plan_only(request)


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


def _valid_config() -> dict:
    return {
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
    }


def _merged_config(config: dict | None) -> dict:
    merged = _valid_config()
    if config:
        merged.update(config)
    return merged
