from src.cartesian_motion_gateway import (
    DEFAULT_CONFIRMATION_TOKEN,
    E_EXCESSIVE_CARTESIAN_MOTION,
    E_MANUAL_CONFIRMATION_REQUIRED,
    E_OUT_OF_WORKSPACE,
    CartesianMotionExecutionRequest,
    CartesianMotionGatewayRequest,
    CartesianMotionPipelineRequest,
    evaluate_cartesian_motion_execution,
    evaluate_cartesian_motion_gateway,
    evaluate_cartesian_motion_pipeline,
)


def test_gateway_generates_target_pose_from_current_tcp_pose_and_offset():
    result = evaluate_cartesian_motion_gateway(
        CartesianMotionGatewayRequest(
            requested=True,
            command_to_task_result=_task([0.0, 0.0, 0.10]),
            current_tcp_pose={
                "frame": "base_link",
                "position_m": [0.40, 0.0, 0.30],
                "orientation_xyzw": [0.0, 0.0, 0.0, 1.0],
            },
        )
    )

    assert result["cartesian_motion_gateway_status"] == "PASS"
    assert result["target_pose"] == {
        "frame": "base_link",
        "position_m": [0.4, 0.0, 0.4],
        "orientation_xyzw": [0.0, 0.0, 0.0, 1.0],
    }
    assert result["moveit_plan_request"]["target_pose"] == result["target_pose"]
    assert result["target_pose_generated_by_teto"] is True
    assert result["target_pose_generated_by_llm"] is False
    assert result["real_robot_motion_executed"] is False


def test_gateway_blocks_excessive_motion_and_workspace_violation():
    excessive = evaluate_cartesian_motion_gateway(
        CartesianMotionGatewayRequest(
            requested=True,
            command_to_task_result=_task([0.21, 0.0, 0.0]),
            current_tcp_pose=[0.0, 0.0, 0.5],
        )
    )

    assert excessive["cartesian_motion_gateway_status"] == "BLOCKED"
    assert E_EXCESSIVE_CARTESIAN_MOTION in excessive["blocking_reasons"]
    assert excessive["target_pose"] is None

    out_of_workspace = evaluate_cartesian_motion_gateway(
        CartesianMotionGatewayRequest(
            requested=True,
            command_to_task_result=_task([0.0, 0.0, -0.10]),
            current_tcp_pose=[0.0, 0.0, 0.05],
            config={"workspace_bounds": {"x": [-1.0, 1.0], "y": [-1.0, 1.0], "z": [0.0, 2.0]}},
        )
    )

    assert out_of_workspace["cartesian_motion_gateway_status"] == "BLOCKED"
    assert E_OUT_OF_WORKSPACE in out_of_workspace["blocking_reasons"]
    assert out_of_workspace["target_pose"] is None


def test_execution_requires_manual_confirmation_before_moveit_execute():
    motion = evaluate_cartesian_motion_gateway(
        CartesianMotionGatewayRequest(
            requested=True,
            command_to_task_result=_task([0.0, 0.0, 0.10]),
            current_tcp_pose=[0.0, 0.0, 0.5],
        )
    )

    result = evaluate_cartesian_motion_execution(
        CartesianMotionExecutionRequest(
            requested=True,
            config=_enabled_execution_config(),
            cartesian_motion_result=motion,
            manual_confirmation_result={"manual_confirmation_accepted": False},
            ur5_state_result={"read_only_state_contract_ready": True},
        )
    )

    assert result["cartesian_motion_execution_status"] == "BLOCKED"
    assert E_MANUAL_CONFIRMATION_REQUIRED in result["blocking_reasons"]
    assert result["moveit_execute_called"] is False
    assert result["real_robot_motion_executed"] is False


def test_pipeline_confirmed_cartesian_command_reaches_moveit_execute_gate():
    result = evaluate_cartesian_motion_pipeline(
        CartesianMotionPipelineRequest(
            requested=True,
            user_command="move 10 cm higher",
            config={
                **_enabled_execution_config(),
                "current_tcp_pose": {
                    "frame": "base_link",
                    "position_m": [0.40, 0.0, 0.30],
                    "orientation_xyzw": [0.0, 0.0, 0.0, 1.0],
                },
                "command_to_task_adapter": {"adapter_mode": "qwen_llm"},
            },
            llm_callable=lambda _prompt: (
                '{"intent":"cartesian_offset","frame":"base_link","dx":0.0,'
                '"dy":0.0,"dz":0.10,"confidence":0.94,"error_code":"OK"}'
            ),
            manual_confirmation_token=DEFAULT_CONFIRMATION_TOKEN,
        )
    )

    assert result["cartesian_motion_pipeline_status"] == "PASS"
    assert result["intent"] == "cartesian_offset"
    assert result["cartesian_offset_m"] == [0.0, 0.0, 0.10]
    assert result["target_pose"]["position_m"] == [0.4, 0.0, 0.4]
    assert result["manual_confirmation_accepted"] is True
    assert result["moveit_execute_called"] is True
    assert result["real_robot_motion_executed"] is True
    assert result["cartesian_motion_execution_result"]["urscript_generated"] is False
    assert result["cartesian_motion_execution_result"]["rtde_write_attempted"] is False
    assert result["cartesian_motion_execution_result"]["dashboard_command_attempted"] is False


def test_pipeline_without_real_motion_enabled_validates_but_does_not_execute():
    result = evaluate_cartesian_motion_pipeline(
        CartesianMotionPipelineRequest(
            requested=True,
            user_command="move 10 cm higher",
            config={
                "current_tcp_pose": [0.40, 0.0, 0.30],
                "command_to_task_adapter": {"adapter_mode": "qwen_llm"},
            },
            llm_callable=lambda _prompt: (
                '{"intent":"cartesian_offset","frame":"base_link","dx":0.0,'
                '"dy":0.0,"dz":0.10,"confidence":0.94,"error_code":"OK"}'
            ),
        )
    )

    assert result["cartesian_motion_pipeline_status"] == "PASS"
    assert result["target_pose"]["position_m"] == [0.4, 0.0, 0.4]
    assert result["moveit_execute_called"] is False
    assert result["real_robot_motion_executed"] is False


def test_real_moveit_mode_routes_plan_only_through_pose_executor(monkeypatch):
    calls = []

    def fake_plan(request):
        calls.append(request)
        return {
            "moveit_pose_executor_status": "PASS",
            "plan_success": True,
            "moveit_plan_called": True,
            "moveit_execute_called": False,
            "trajectory_send_allowed": False,
            "trajectory_sent": False,
            "controller_command_sent": False,
            "real_robot_motion_executed": False,
            "blocking_reasons": [],
            "warnings": [],
        }

    monkeypatch.setattr("src.cartesian_motion_gateway.evaluate_moveit_pose_plan", fake_plan)

    motion = evaluate_cartesian_motion_gateway(
        CartesianMotionGatewayRequest(
            requested=True,
            command_to_task_result=_task([0.0, 0.0, 0.02]),
            current_tcp_pose=[0.40, 0.0, 0.30],
        )
    )
    result = evaluate_cartesian_motion_execution(
        CartesianMotionExecutionRequest(
            requested=True,
            config={
                "moveit_execution_mode": "real",
                "enable_ros2_runtime": True,
                "enable_moveit_plan": True,
                "enable_moveit_execute": False,
                "enable_real_robot_motion": False,
                "manual_confirmation_required": True,
            },
            cartesian_motion_result=motion,
            manual_confirmation_result={"manual_confirmation_accepted": False},
        )
    )

    assert result["cartesian_motion_execution_status"] == "PASS"
    assert result["real_moveit_mode"] is True
    assert result["moveit_plan_success"] is True
    assert result["moveit_execute_called"] is False
    assert calls[0].target_pose == motion["target_pose"]
    assert calls[0].current_tcp_pose == motion["current_tcp_pose"]


def _task(offset):
    return {
        "command_to_task_status": "PASS",
        "intent": "cartesian_offset",
        "frame": "base_link",
        "dx": offset[0],
        "dy": offset[1],
        "dz": offset[2],
        "cartesian_offset_m": list(offset),
        "task_contract": {
            "intent": "cartesian_offset",
            "frame": "base_link",
            "dx": offset[0],
            "dy": offset[1],
            "dz": offset[2],
            "cartesian_offset_m": list(offset),
        },
        "blocking_reasons": [],
        "warnings": [],
    }


def _enabled_execution_config():
    return {
        "enable_ros2_runtime": True,
        "enable_moveit_plan": True,
        "enable_moveit_execute": True,
        "enable_real_robot_motion": True,
        "manual_confirmation_required": True,
        "ros2_runtime_available": True,
        "moveit_runtime_available": True,
        "moveit_plan_success": True,
        "moveit_execute_allowed": True,
        "robot_state_ok": True,
        "safety_status_ok": True,
        "protective_stop": False,
        "emergency_stop": False,
        "speed_scaling": 0.05,
        "max_speed_scale": 0.10,
        "max_acc_scale": 0.10,
        "ur5_state": {"read_only_state_contract_ready": True},
    }
