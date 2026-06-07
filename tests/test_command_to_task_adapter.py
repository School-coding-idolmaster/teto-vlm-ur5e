from src.command_to_task_adapter import (
    MODE_LLM_QWEN,
    MODE_RULE_BASED,
    STATUS_BLOCKED,
    STATUS_PASS,
    CommandToTaskAdapterRequest,
    evaluate_command_to_task_adapter,
)
from src.v3_hover_demo_orchestrator import V3HoverDemoRequest, evaluate_v3_hover_demo


def test_qwen_llm_callable_hover_passes_validation_without_robot_side_effects():
    result = evaluate_command_to_task_adapter(
        CommandToTaskAdapterRequest(
            requested=True,
            user_command="move above red mug",
            config={"adapter_mode": MODE_LLM_QWEN},
            llm_callable=lambda _prompt: (
                '{"intent":"hover_to_object","target_label":"red_mug",'
                '"waypoint":null,"confidence":0.93,"error_code":"OK"}'
            ),
        )
    )

    assert result["command_to_task_status"] == STATUS_PASS
    assert result["llm_called"] is True
    assert result["intent"] == "hover_to_object"
    assert result["target_label"] == "red_mug"
    assert result["confidence"] == 0.93
    assert result["task_contract"]["execution_policy"]["allow_ros_publish"] is False
    assert result["task_contract"]["execution_policy"]["allow_moveit_execute"] is False
    assert result["task_contract"]["execution_policy"]["allow_robot_motion"] is False
    assert result["ros2_publish_attempted"] is False
    assert result["moveit_called"] is False
    assert result["real_robot_motion_executed"] is False
    assert result["trajectory_generated"] is False
    assert result["tcp_pose_world_generated"] is False
    assert result["joint_targets_generated"] is False


def test_rule_based_examples_match_task_contracts():
    examples = [
        ("move above the red mug", "hover_to_object", "red_mug", None),
        ("go home", "go_home", None, None),
        ("move to inspection pose", "move_to_waypoint", None, "inspection_pose"),
    ]

    for command, intent, target_label, waypoint in examples:
        result = evaluate_command_to_task_adapter(
            CommandToTaskAdapterRequest(
                requested=True,
                user_command=command,
                config={"adapter_mode": MODE_RULE_BASED},
            )
        )

        assert result["command_to_task_status"] == STATUS_PASS
        assert result["intent"] == intent
        assert result["target_label"] == target_label
        assert result["waypoint"] == waypoint
        assert result["task_contract"]["intent"] == intent


def test_low_confidence_llm_output_blocks_before_planner_gateway():
    result = evaluate_command_to_task_adapter(
        CommandToTaskAdapterRequest(
            requested=True,
            user_command="move above red mug",
            config={"adapter_mode": MODE_LLM_QWEN, "confidence_threshold": 0.60},
            llm_callable=lambda _prompt: (
                '{"intent":"hover_to_object","target_label":"red_mug",'
                '"waypoint":null,"confidence":0.20,"error_code":"OK"}'
            ),
        )
    )

    assert result["command_to_task_status"] == STATUS_BLOCKED
    assert "E_LOW_CONFIDENCE" in result["blocking_reasons"]
    assert result["ros2_publish_attempted"] is False
    assert result["moveit_called"] is False
    assert result["real_robot_motion_executed"] is False


def test_forbidden_robot_fields_block_even_when_intent_is_valid():
    result = evaluate_command_to_task_adapter(
        CommandToTaskAdapterRequest(
            requested=True,
            user_command="move above red mug",
            config={
                "adapter_mode": MODE_LLM_QWEN,
                "llm_response": {
                    "intent": "hover_to_object",
                    "target_label": "red_mug",
                    "confidence": 0.95,
                    "trajectory": [[0.0, 0.0, 0.0]],
                },
            },
            llm_callable=lambda _prompt: (
                '{"intent":"hover_to_object","target_label":"red_mug",'
                '"confidence":0.95,"trajectory":[[0,0,0]],"error_code":"OK"}'
            ),
        )
    )

    assert result["command_to_task_status"] == STATUS_BLOCKED
    assert "E_ROBOT_COMMAND_NOT_ALLOWED" in result["blocking_reasons"]
    assert any(field.endswith("trajectory") for field in result["forbidden_robot_control_fields"])
    assert result["trajectory_generated"] is False
    assert result["real_robot_motion_executed"] is False


def test_invalid_llm_json_blocks_safely():
    result = evaluate_command_to_task_adapter(
        CommandToTaskAdapterRequest(
            requested=True,
            user_command="move above red mug",
            config={"adapter_mode": MODE_LLM_QWEN},
            llm_callable=lambda _prompt: "not json",
        )
    )

    assert result["command_to_task_status"] == STATUS_BLOCKED
    assert "E_LLM_CALL_FAILED" in result["blocking_reasons"]
    assert result["ros2_publish_attempted"] is False
    assert result["moveit_called"] is False
    assert result["real_robot_motion_executed"] is False


def test_v3_hover_demo_uses_command_to_task_before_perception_and_stays_shadow_only():
    result = evaluate_v3_hover_demo(
        V3HoverDemoRequest(
            requested=True,
            user_command="move above red mug",
            config_path="configs/v3_hover_demo.example.yaml",
            config={
                "command_to_task_adapter": {
                    "adapter_mode": "offline_llm_json",
                    "llm_response": {
                        "intent": "hover_to_object",
                        "target_label": "red_mug",
                        "waypoint": None,
                        "confidence": 0.94,
                        "error_code": "OK",
                    },
                }
            },
        )
    )

    assert result["v3_hover_demo_status"] == STATUS_PASS
    assert result["normalized_intent"] == "hover_to_object"
    assert result["target_label"] == "red_mug"
    assert "COMMAND_TO_TASK_READY" in result["stages"]
    assert result["command_to_task_result"]["command_to_task_status"] == STATUS_PASS
    assert result["planner_request_ready"] is True
    assert result["moveit_execute_called"] is False
    assert result["controller_command_sent"] is False
    assert result["real_robot_motion_executed"] is False
    assert result["urscript_generated"] is False
    assert result["rtde_write_attempted"] is False
    assert result["dashboard_command_attempted"] is False
