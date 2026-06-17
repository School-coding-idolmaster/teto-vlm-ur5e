import pytest

from src.cartesian_motion_gateway import (
    DEFAULT_CONFIRMATION_TOKEN,
    E_AXIS_STEP_EXCEEDS_LIMIT,
    E_EXCESSIVE_CARTESIAN_MOTION,
    E_MANUAL_CONFIRMATION_REQUIRED,
    E_OUT_OF_WORKSPACE,
    E_STEP_DELTA_EXCEEDS_LIMIT,
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


def test_gateway_allows_exact_relative_max_motion_from_nonzero_tcp_pose():
    result = evaluate_cartesian_motion_gateway(
        CartesianMotionGatewayRequest(
            requested=True,
            config={"max_translation_m": 0.005},
            command_to_task_result=_task([0.0, 0.0, 0.005000000000000004]),
            current_tcp_pose={
                "frame": "base_link",
                "position_m": [-0.154964, 0.312309, 1.041042],
                "orientation_xyzw": [0.0, 0.0, 0.0, 1.0],
            },
        )
    )

    assert result["cartesian_motion_gateway_status"] == "PASS"
    assert E_EXCESSIVE_CARTESIAN_MOTION not in result["blocking_reasons"]
    assert result["translation_distance_m"] == 0.005


def test_directional_step_policy_first_move_bootstraps_from_current_tcp_pose():
    result = evaluate_cartesian_motion_gateway(
        CartesianMotionGatewayRequest(
            requested=True,
            config=_step_policy_config(),
            command_to_task_result=_task([0.05, 0.0, 0.0]),
            current_tcp_pose=_pose([0.40, 0.0, 0.30]),
        )
    )

    assert result["cartesian_motion_gateway_status"] == "PASS"
    assert result["safety_policy_name"] == "lab_directional_step_motion_v1"
    assert result["motion_frame"] == "base_link"
    assert result["direction_axis"] == "x"
    assert result["direction_sign"] == "+"
    assert result["first_move_bootstrap_used"] is True
    assert result["previous_verified_tcp_pose"] is None
    assert result["current_measured_tcp_pose"]["position_m"] == [0.4, 0.0, 0.3]
    assert result["requested_target_tcp_pose"]["position_m"] == [0.45, 0.0, 0.3]
    assert result["delta_from_current_tcp_m"] == [0.05, 0.0, 0.0]
    assert result["delta_from_previous_verified_tcp_m"] is None
    assert result["max_step_distance_m"] == 0.05
    assert result["max_axis_step_m"] == 0.05
    assert result["hard_safety_limit_m"] == 0.1
    assert result["session_radius_limit_m"] is None
    assert result["step_delta_within_limit"] is True
    assert result["axis_delta_within_limit"] is True
    assert result["workspace_envelope_within_limit"] is True


def test_directional_step_policy_second_move_uses_previous_verified_tcp_pose():
    result = evaluate_cartesian_motion_gateway(
        CartesianMotionGatewayRequest(
            requested=True,
            config={
                **_step_policy_config(),
                "previous_verified_tcp_pose": _pose([0.40, 0.0, 0.30]),
            },
            command_to_task_result=_task([0.01, 0.0, 0.0]),
            current_tcp_pose=_pose([0.44, 0.0, 0.30]),
        )
    )

    assert result["cartesian_motion_gateway_status"] == "PASS"
    assert result["first_move_bootstrap_used"] is False
    assert result["requested_target_tcp_pose"]["position_m"] == [0.45, 0.0, 0.3]
    assert result["delta_from_current_tcp_m"] == [0.01, 0.0, 0.0]
    assert result["delta_from_previous_verified_tcp_m"] == [0.05, 0.0, 0.0]
    assert result["step_delta_within_limit"] is True


@pytest.mark.parametrize(
    ("offset", "axis", "sign"),
    [
        ([0.01, 0.0, 0.0], "x", "+"),
        ([-0.01, 0.0, 0.0], "x", "-"),
        ([0.0, 0.01, 0.0], "y", "+"),
        ([0.0, -0.01, 0.0], "y", "-"),
        ([0.0, 0.0, 0.01], "z", "+"),
        ([0.0, 0.0, -0.01], "z", "-"),
    ],
)
def test_directional_step_policy_reports_axis_and_sign(offset, axis, sign):
    result = evaluate_cartesian_motion_gateway(
        CartesianMotionGatewayRequest(
            requested=True,
            config=_step_policy_config(),
            command_to_task_result=_task(offset),
            current_tcp_pose=_pose([0.40, 0.0, 0.30]),
        )
    )

    assert result["cartesian_motion_gateway_status"] == "PASS"
    assert result["direction_axis"] == axis
    assert result["direction_sign"] == sign


def test_directional_step_policy_rejects_over_step_delta():
    result = evaluate_cartesian_motion_gateway(
        CartesianMotionGatewayRequest(
            requested=True,
            config=_step_policy_config(),
            command_to_task_result=_task([0.051, 0.0, 0.0]),
            current_tcp_pose=_pose([0.40, 0.0, 0.30]),
        )
    )

    assert result["cartesian_motion_gateway_status"] == "BLOCKED"
    assert E_STEP_DELTA_EXCEEDS_LIMIT in result["blocking_reasons"]
    assert E_EXCESSIVE_CARTESIAN_MOTION in result["blocking_reasons"]
    assert result["step_delta_within_limit"] is False


def test_directional_step_policy_rejects_axis_step_delta():
    result = evaluate_cartesian_motion_gateway(
        CartesianMotionGatewayRequest(
            requested=True,
            config={**_step_policy_config(), "max_axis_step_m": 0.01},
            command_to_task_result=_task([0.02, 0.0, 0.0]),
            current_tcp_pose=_pose([0.40, 0.0, 0.30]),
        )
    )

    assert result["cartesian_motion_gateway_status"] == "BLOCKED"
    assert E_AXIS_STEP_EXCEEDS_LIMIT in result["blocking_reasons"]
    assert result["axis_delta_within_limit"] is False


def test_directional_step_policy_rejects_workspace_envelope_violation():
    result = evaluate_cartesian_motion_gateway(
        CartesianMotionGatewayRequest(
            requested=True,
            config={**_step_policy_config(), "workspace_bounds": {"x": [0.0, 0.42], "y": [-1.0, 1.0], "z": [0.0, 2.0]}},
            command_to_task_result=_task([0.03, 0.0, 0.0]),
            current_tcp_pose=_pose([0.40, 0.0, 0.30]),
        )
    )

    assert result["cartesian_motion_gateway_status"] == "BLOCKED"
    assert E_OUT_OF_WORKSPACE in result["blocking_reasons"]
    assert result["workspace_envelope_within_limit"] is False


def test_directional_step_policy_rejects_hard_safety_limit_violation():
    result = evaluate_cartesian_motion_gateway(
        CartesianMotionGatewayRequest(
            requested=True,
            config={**_step_policy_config(), "max_step_distance_m": 0.10, "max_translation_m": 0.10, "hard_safety_limit_m": 0.05},
            command_to_task_result=_task([0.06, 0.0, 0.0]),
            current_tcp_pose=_pose([0.40, 0.0, 0.30]),
        )
    )

    assert result["cartesian_motion_gateway_status"] == "BLOCKED"
    assert E_EXCESSIVE_CARTESIAN_MOTION in result["blocking_reasons"]
    assert result["hard_safety_limit_m"] == 0.05


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


def _pose(position):
    return {
        "frame": "base_link",
        "position_m": list(position),
        "orientation_xyzw": [0.0, 0.0, 0.0, 1.0],
    }


def _step_policy_config():
    return {
        "safety_policy_name": "lab_directional_step_motion_v1",
        "max_translation_m": 0.05,
        "configured_max_distance_m": 0.05,
        "max_step_distance_m": 0.05,
        "max_axis_step_m": 0.05,
        "hard_safety_limit_m": 0.1,
        "workspace_bounds": {"x": [-1.0, 1.0], "y": [-1.0, 1.0], "z": [0.0, 2.0]},
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
