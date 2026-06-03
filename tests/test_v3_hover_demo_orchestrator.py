from pathlib import Path

from src.v3_hover_demo_orchestrator import V3HoverDemoRequest, evaluate_v3_hover_demo


def test_positive_software_no_robot_mode_passes_and_exports_evidence(tmp_path):
    result = evaluate_v3_hover_demo(
        V3HoverDemoRequest(
            requested=True,
            user_command="hover over the red mug",
            config_path="configs/v3_hover_demo.example.yaml",
            output_dir=str(tmp_path),
            write_evidence=True,
        )
    )

    assert result["v3_hover_demo_status"] == "PASS"
    assert result["v3_demo_mode"] == "software_no_robot"
    assert result["normalized_intent"] == "hover_to_object"
    assert result["planner_request_ready"] is True
    assert result["real_robot_motion_executed"] is False
    assert result["moveit_execute_called"] is False
    assert result["controller_command_sent"] is False
    assert result["v3_hover_demo_evidence_available"] is True
    assert Path(result["v3_hover_demo_result_path"]).is_file()
    assert Path(result["v3_hover_demo_report_path"]).is_file()
    assert Path(result["evidence_manifest_path"]).is_file()


def test_unsupported_grasp_command_blocks_before_robot_command_generation():
    result = evaluate_v3_hover_demo(
        V3HoverDemoRequest(
            requested=True,
            user_command="grasp the red mug",
            config_path="configs/v3_hover_demo.example.yaml",
        )
    )

    assert result["v3_hover_demo_status"] == "BLOCKED"
    assert "E_UNSUPPORTED_CONTACT_TASK" in result["blocking_reasons"]
    assert result["real_robot_motion_executed"] is False
    assert result["urscript_generated"] is False
    assert result["raw_joint_targets_generated"] is False


def test_missing_target_blocks():
    result = evaluate_v3_hover_demo(
        V3HoverDemoRequest(
            requested=True,
            user_command="hover over",
            config_path="configs/v3_hover_demo.example.yaml",
        )
    )

    assert result["v3_hover_demo_status"] == "BLOCKED"
    assert "E_TARGET_DESCRIPTION_MISSING" in result["blocking_reasons"]
    assert result["real_robot_motion_executed"] is False


def test_enabling_real_motion_without_manual_confirmation_blocks():
    result = evaluate_v3_hover_demo(
        V3HoverDemoRequest(
            requested=True,
            user_command="hover over the red mug",
            config_path="configs/v3_hover_demo.example.yaml",
            enable_ros2_runtime=True,
            enable_moveit_plan=True,
            enable_moveit_execute=True,
            enable_real_robot_motion=True,
        )
    )

    assert result["v3_hover_demo_status"] == "BLOCKED"
    assert "E_MANUAL_CONFIRMATION_REQUIRED" in result["blocking_reasons"]
    assert result["real_robot_motion_executed"] is False


def test_negative_robot_command_fixture_before_confirmation_blocks():
    result = evaluate_v3_hover_demo(
        V3HoverDemoRequest(
            requested=True,
            user_command="hover over the red mug",
            config_path="configs/v3_hover_demo.example.yaml",
            config={"real_ur5_hover_executor": {"urscript_generated": True}},
            enable_real_robot_motion=True,
        )
    )

    assert result["v3_hover_demo_status"] == "BLOCKED"
    assert "E_EXECUTION_FAILED" in result["blocking_reasons"]
    assert result["real_robot_motion_executed"] is False
