import argparse
import json
import shlex
import sys
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from src.semantic_simulation_bridge import (
    DEFAULT_SEMANTIC_CONFIDENCE_THRESHOLD,
    build_demo_semantic_task_contract,
    load_semantic_task_contract,
)
from src.simulation_runtime import DEFAULT_SIMULATION_TASK, CURRENT_TETO_VERSION, run_first_simulation_execution
from src.v3_hover_demo_orchestrator import V3HoverDemoRequest, evaluate_v3_hover_demo


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Run TETO V3.0.0 hover-demo evidence or legacy no-motion simulation evidence smoke tests."
    )
    parser.add_argument("--dry-run", action="store_true", help="Do not import Isaac; produce a test execution report.")
    parser.add_argument("--no-isaac", action="store_true", help="Pure Python test mode without Isaac imports.")
    parser.add_argument("--spawn-cube", action="store_true", help="Spawn a visible cube in the Isaac World.")
    parser.add_argument(
        "--move-object",
        action="store_true",
        help="Run the default simulation object pose update smoke test.",
    )
    parser.add_argument(
        "--move-cube",
        action="store_true",
        help="Alias for --move-object using the default cube fixture.",
    )
    parser.add_argument("--task-json", help="Path to a simulation_task JSON file.")
    parser.add_argument("--steps", type=int, default=5, help="Number of simulation steps to run.")
    parser.add_argument("--gui", action="store_true", help="Run Isaac with GUI instead of headless mode.")
    parser.add_argument("--output-dir", help="Directory where simulation_execution_result.json is written.")
    parser.add_argument(
        "--check-robot-asset",
        action="store_true",
        help="Check robot asset availability without requiring a successful load.",
    )
    parser.add_argument(
        "--load-robot-asset",
        action="store_true",
        help="Load the specified local robot asset and verify the robot prim exists.",
    )
    parser.add_argument("--robot-asset-path", help="Local USD/USDA/USDC robot asset path to check or load.")
    parser.add_argument("--robot-type", default="ur5", help="Robot type label for report metadata.")
    parser.add_argument("--robot-prim-path", default="/World/TETO_Robot", help="Prim path for a loaded robot asset.")
    parser.add_argument(
        "--inspect-robot-prim",
        action="store_true",
        help="Read robot prim structure and metadata without commanding robot motion.",
    )
    parser.add_argument(
        "--check-articulation-readiness",
        action="store_true",
        help="Evaluate read-only articulation readiness without enabling control or motion.",
    )
    parser.add_argument(
        "--observe-articulation-state",
        action="store_true",
        help="Observe read-only articulation joint state metadata without generating control targets.",
    )
    parser.add_argument(
        "--check-simulation-motion-precheck",
        action="store_true",
        help="Evaluate the simulation-only robot motion precheck gate without moving the robot.",
    )
    parser.add_argument(
        "--execute-simulation-micro-motion",
        action="store_true",
        help="Execute one tiny simulation-only Isaac joint delta after the precheck gate passes.",
    )
    parser.add_argument(
        "--micro-motion-joint",
        default="wrist_3_joint",
        help="UR5e arm joint name for the simulation-only micro-motion.",
    )
    parser.add_argument(
        "--micro-motion-delta-rad",
        type=float,
        default=0.01,
        help="Tiny requested simulation joint delta in radians.",
    )
    parser.add_argument(
        "--micro-motion-tolerance-rad",
        type=float,
        default=0.005,
        help="Allowed absolute error for the observed simulation joint delta.",
    )
    parser.add_argument(
        "--semantic-simulation-bridge",
        action="store_true",
        help="Gate an existing semantic task contract into the simulation-only micro-motion proof path.",
    )
    parser.add_argument("--semantic-task-json", help="Path to a semantic task contract JSON file.")
    parser.add_argument(
        "--semantic-bridge-demo-contract",
        action="store_true",
        help="Use the built-in eligible semantic bridge demo contract.",
    )
    parser.add_argument(
        "--semantic-confidence-threshold",
        type=float,
        default=DEFAULT_SEMANTIC_CONFIDENCE_THRESHOLD,
        help="Minimum semantic and overall confidence for the bridge gate.",
    )
    parser.add_argument(
        "--safe-simulated-task-execution",
        action="store_true",
        help="Run the safe simulated task execution lifecycle after the semantic bridge gate.",
    )
    parser.add_argument("--execution-attempt-id", help="Optional stable execution attempt id.")
    parser.add_argument(
        "--execution-max-attempts",
        type=int,
        default=1,
        help="Maximum attempts metadata. The legacy simulated execution path supports 1 and does not auto-retry.",
    )
    parser.add_argument(
        "--execution-enable-retry-recommendation",
        action="store_true",
        help="Generate retry recommendation metadata without executing an automatic retry.",
    )
    parser.add_argument(
        "--execution-enable-fallback-recommendation",
        action="store_true",
        help="Generate fallback recommendation metadata.",
    )
    parser.add_argument(
        "--check-lab-readiness",
        action="store_true",
        help="Evaluate config-only lab backend readiness without connecting to a real robot.",
    )
    parser.add_argument(
        "--lab-readiness-config",
        help="Path to a local or example lab readiness YAML config. Only fields are inspected.",
    )
    parser.add_argument(
        "--check-camera-readiness",
        action="store_true",
        help="Evaluate config-only camera readiness without opening a camera stream.",
    )
    parser.add_argument(
        "--check-live-vlm-readiness",
        action="store_true",
        help="Evaluate config-only live VLM readiness without calling a live model.",
    )
    parser.add_argument(
        "--check-shadow-mode-readiness",
        action="store_true",
        help="Evaluate no-motion shadow-mode readiness.",
    )
    parser.add_argument(
        "--check-camera-snapshot",
        action="store_true",
        help="Validate a camera snapshot manifest without live capture, VLM calls, or robot motion.",
    )
    parser.add_argument(
        "--camera-snapshot-config",
        help="Path to a camera snapshot YAML/JSON manifest. Only declared fields are validated.",
    )
    parser.add_argument(
        "--camera-snapshot-report",
        action="store_true",
        help="Generate camera snapshot evidence report from the supplied manifest.",
    )
    parser.add_argument(
        "--check-vlm-grounding-adapter",
        action="store_true",
        help="Generate a no-motion VLM grounding result contract from mock/offline/manual/disabled declarations.",
    )
    parser.add_argument(
        "--vlm-grounding-config",
        help="Path to VLM grounding adapter YAML/JSON config.",
    )
    parser.add_argument(
        "--vlm-grounding-report",
        action="store_true",
        help="Generate VLM grounding adapter evidence report.",
    )
    parser.add_argument("--user-command", help="Text command to ground against the declared snapshot.")
    parser.add_argument(
        "--allow-live-vlm",
        action="store_true",
        help="Declare live VLM allowance metadata only; V2.9.5 still never calls a live model.",
    )
    parser.add_argument(
        "--check-planner-gateway-shadow",
        action="store_true",
        help="Generate bounded Planner Gateway shadow input evidence from perception shadow result JSON.",
    )
    parser.add_argument(
        "--planner-gateway-shadow-config",
        help="Path to Planner Gateway shadow YAML/JSON config.",
    )
    parser.add_argument(
        "--planner-gateway-shadow-report",
        action="store_true",
        help="Generate Planner Gateway shadow contract evidence report.",
    )
    parser.add_argument(
        "--perception-shadow-result",
        help="Path to an offline perception_shadow_result JSON/YAML file.",
    )
    parser.add_argument(
        "--check-ros2-interface-readiness",
        action="store_true",
        help="Validate ROS2 Planner Gateway interface declarations without publishing or moving.",
    )
    parser.add_argument(
        "--ros2-interface-config",
        help="Path to ROS2 interface readiness YAML/JSON config.",
    )
    parser.add_argument(
        "--ros2-interface-report",
        action="store_true",
        help="Generate ROS2 interface readiness evidence report.",
    )
    parser.add_argument(
        "--check-ros2-message-export",
        action="store_true",
        help="Export a deterministic fake-publish ROS2 PlannerRequest JSON artifact without publishing.",
    )
    parser.add_argument(
        "--ros2-message-export-config",
        help="Path to ROS2 message export YAML/JSON config.",
    )
    parser.add_argument(
        "--ros2-message-export-report",
        action="store_true",
        help="Generate ROS2 message export fake-publish evidence report.",
    )
    parser.add_argument(
        "--check-moveit-plan-only",
        action="store_true",
        help="Validate MoveIt plan-only declarations without planning execution.",
    )
    parser.add_argument("--moveit-plan-only-config", help="Path to MoveIt plan-only YAML/JSON config.")
    parser.add_argument(
        "--moveit-plan-only-report",
        action="store_true",
        help="Generate MoveIt plan-only evidence report.",
    )
    parser.add_argument(
        "--check-ur5-read-only-state",
        action="store_true",
        help="Validate UR5 read-only state monitor declarations without live sockets.",
    )
    parser.add_argument("--ur5-read-only-state-config", help="Path to UR5 read-only state YAML/JSON config.")
    parser.add_argument(
        "--ur5-read-only-state-report",
        action="store_true",
        help="Generate UR5 read-only state evidence report.",
    )
    parser.add_argument(
        "--check-robot-system-shadow-bridge",
        action="store_true",
        help="Validate the full robot-system shadow bridge rehearsal boundary.",
    )
    parser.add_argument(
        "--robot-system-shadow-bridge-config",
        help="Path to robot-system shadow bridge YAML/JSON config.",
    )
    parser.add_argument(
        "--robot-system-shadow-bridge-report",
        action="store_true",
        help="Generate full robot-system shadow bridge evidence report.",
    )
    parser.add_argument(
        "--check-camera-source-adapter",
        action="store_true",
        help="Validate camera source adapter evidence and generate a no-motion snapshot contract.",
    )
    parser.add_argument(
        "--camera-source-config",
        help="Path to camera source adapter YAML/JSON config.",
    )
    parser.add_argument(
        "--camera-source-report",
        action="store_true",
        help="Generate camera source adapter evidence report.",
    )
    parser.add_argument(
        "--allow-live-camera-capture",
        action="store_true",
        help="Explicitly allow optional one-shot camera capture if a safe backend is available.",
    )
    parser.add_argument(
        "--camera-source-mode",
        choices=[
            "realsense_replay",
            "offline_file",
            "manual_snapshot",
            "live_disabled",
            "optional_realsense_one_shot",
        ],
        help="Override the camera source mode declared in the adapter config.",
    )
    parser.add_argument(
        "--check-geometry-validity",
        action="store_true",
        help="Validate snapshot plus offline/mock grounding geometry before projector handoff.",
    )
    parser.add_argument(
        "--geometry-validity-config",
        help="Path to geometry validity YAML/JSON config. Only declared evidence references are validated.",
    )
    parser.add_argument(
        "--geometry-validity-report",
        action="store_true",
        help="Generate geometry validity evidence report without live camera, live VLM, or robot motion.",
    )
    parser.add_argument(
        "--check-projector-shadow",
        action="store_true",
        help="Run offline 2D-to-3D projector shadow validation without ROS2 TF or robot motion.",
    )
    parser.add_argument(
        "--projector-shadow-config",
        help="Path to projector shadow YAML/JSON config. Only declared offline evidence is used.",
    )
    parser.add_argument(
        "--projector-shadow-report",
        action="store_true",
        help="Generate 2D-to-3D projector shadow evidence report.",
    )
    parser.add_argument(
        "--run-real-scene-shadow",
        action="store_true",
        help="Validate offline/manual camera snapshot plus offline/mock grounding evidence without motion.",
    )
    parser.add_argument(
        "--real-scene-shadow-config",
        help="Path to real-scene shadow YAML/JSON config. Only declared evidence references are validated.",
    )
    parser.add_argument(
        "--grounding-result",
        help="Path to an offline/mock grounding result JSON/YAML file.",
    )
    parser.add_argument(
        "--real-scene-shadow-report",
        action="store_true",
        help="Generate real-scene no-motion shadow evidence report.",
    )
    parser.add_argument(
        "--run-perception-shadow-pipeline",
        action="store_true",
        help="Run the full text + camera + grounding + geometry + projector no-motion shadow pipeline.",
    )
    parser.add_argument(
        "--perception-shadow-config",
        help="Path to full perception shadow pipeline YAML/JSON config.",
    )
    parser.add_argument(
        "--perception-shadow-report",
        action="store_true",
        help="Generate full perception shadow pipeline evidence report.",
    )
    parser.add_argument(
        "--run-v3-hover-demo",
        action="store_true",
        help="Run the TETO V3.0.0 first real UR5 hover demo pipeline. Real motion remains disabled by default.",
    )
    parser.add_argument("--v3-user-command", help="Limited natural-language V3 command, e.g. 'hover over the red mug'.")
    parser.add_argument("--v3-hover-config", help="Path to a V3 hover demo YAML/JSON config.")
    parser.add_argument(
        "--v3-hover-report",
        action="store_true",
        help="Write V3 hover demo JSON, Markdown report, summary, and evidence manifest.",
    )
    parser.add_argument("--enable-live-camera", action="store_true", help="Allow the V3 live camera stage when config also permits it.")
    parser.add_argument("--enable-live-vlm", action="store_true", help="Allow the V3 live VLM stage when config also permits it.")
    parser.add_argument("--enable-ros2-runtime", action="store_true", help="Require/enable ROS2 runtime checks for V3.")
    parser.add_argument("--enable-moveit-plan", action="store_true", help="Enable V3 MoveIt planning checks.")
    parser.add_argument("--enable-moveit-execute", action="store_true", help="Enable V3 MoveIt execution gate.")
    parser.add_argument(
        "--enable-real-robot-motion",
        action="store_true",
        help="Request the final V3 real UR5 motion gate; still blocked unless every safety gate passes.",
    )
    parser.add_argument("--manual-confirmation-token", help="Manual confirmation token for V3 real-motion gate.")
    return parser


def main() -> int:
    args = build_parser().parse_args()
    if args.run_v3_hover_demo:
        result = evaluate_v3_hover_demo(
            V3HoverDemoRequest(
                requested=True,
                user_command=args.v3_user_command or args.user_command,
                config_path=args.v3_hover_config or "configs/v3_hover_demo.example.yaml",
                output_dir=args.output_dir,
                write_evidence=args.v3_hover_report,
                manual_confirmation_token=args.manual_confirmation_token,
                enable_live_camera=args.enable_live_camera,
                enable_live_vlm=args.enable_live_vlm,
                enable_ros2_runtime=args.enable_ros2_runtime,
                enable_moveit_plan=args.enable_moveit_plan,
                enable_moveit_execute=args.enable_moveit_execute,
                enable_real_robot_motion=args.enable_real_robot_motion,
            )
        )
        print_v3_hover_summary(result)
        return 0 if result.get("ok") else 1
    if args.execution_max_attempts != 1:
        raise ValueError("--execution-max-attempts currently supports only 1 in the legacy simulated execution path")
    simulation_task = _load_simulation_task(args.task_json)
    semantic_contract = None
    semantic_contract_path = None
    effective_semantic_bridge = args.semantic_simulation_bridge or args.safe_simulated_task_execution
    if effective_semantic_bridge:
        if args.semantic_task_json and args.semantic_bridge_demo_contract:
            raise ValueError("use either --semantic-task-json or --semantic-bridge-demo-contract, not both")
        if args.semantic_task_json:
            semantic_contract_path = str(Path(args.semantic_task_json).expanduser())
            semantic_contract = load_semantic_task_contract(semantic_contract_path)
        elif args.semantic_bridge_demo_contract:
            semantic_contract = build_demo_semantic_task_contract()
            semantic_contract_path = "builtin:eligible_demo_semantic_bridge_contract"
        else:
            raise ValueError(
                "--semantic-simulation-bridge or --safe-simulated-task-execution requires "
                "--semantic-task-json or --semantic-bridge-demo-contract"
            )
    result = run_first_simulation_execution(
        simulation_task,
        dry_run=args.dry_run,
        no_isaac=args.no_isaac,
        steps=args.steps,
        headless=not args.gui,
        spawn_cube=args.spawn_cube,
        move_object=args.move_object,
        move_cube=args.move_cube,
        check_robot_asset=args.check_robot_asset,
        load_robot_asset=args.load_robot_asset,
        robot_type=args.robot_type,
        robot_prim_path=args.robot_prim_path,
        robot_asset_path=args.robot_asset_path,
        inspect_robot_prim=args.inspect_robot_prim,
        check_articulation_readiness=args.check_articulation_readiness,
        observe_articulation_state=args.observe_articulation_state,
        check_simulation_motion_precheck=args.check_simulation_motion_precheck,
        execute_simulation_micro_motion=args.execute_simulation_micro_motion,
        micro_motion_joint=args.micro_motion_joint,
        micro_motion_delta_rad=args.micro_motion_delta_rad,
        micro_motion_tolerance_rad=args.micro_motion_tolerance_rad,
        semantic_simulation_bridge=effective_semantic_bridge,
        semantic_task_contract=semantic_contract,
        semantic_task_contract_path=semantic_contract_path,
        semantic_confidence_threshold=args.semantic_confidence_threshold,
        safe_simulated_task_execution=args.safe_simulated_task_execution,
        execution_attempt_id=args.execution_attempt_id,
        execution_max_attempts=args.execution_max_attempts,
        execution_enable_retry_recommendation=args.execution_enable_retry_recommendation,
        execution_enable_fallback_recommendation=args.execution_enable_fallback_recommendation,
        check_lab_readiness=args.check_lab_readiness,
        lab_readiness_config=args.lab_readiness_config,
        check_camera_readiness=args.check_camera_readiness,
        check_live_vlm_readiness=args.check_live_vlm_readiness,
        check_shadow_mode_readiness=args.check_shadow_mode_readiness,
        check_camera_source_adapter=args.check_camera_source_adapter,
        camera_source_config=args.camera_source_config,
        camera_source_report=args.camera_source_report,
        allow_live_camera_capture=args.allow_live_camera_capture,
        camera_source_mode=args.camera_source_mode,
        check_camera_snapshot=args.check_camera_snapshot,
        camera_snapshot_config=args.camera_snapshot_config,
        camera_snapshot_report=args.camera_snapshot_report,
        check_vlm_grounding_adapter=args.check_vlm_grounding_adapter,
        vlm_grounding_config=args.vlm_grounding_config,
        vlm_grounding_report=args.vlm_grounding_report,
        user_command=args.user_command,
        allow_live_vlm=args.allow_live_vlm,
        check_geometry_validity=args.check_geometry_validity,
        geometry_validity_config=args.geometry_validity_config,
        geometry_validity_report=args.geometry_validity_report,
        check_projector_shadow=args.check_projector_shadow,
        projector_shadow_config=args.projector_shadow_config,
        projector_shadow_report=args.projector_shadow_report,
        run_real_scene_shadow=args.run_real_scene_shadow,
        real_scene_shadow_config=args.real_scene_shadow_config,
        grounding_result=args.grounding_result,
        real_scene_shadow_report=args.real_scene_shadow_report,
        run_perception_shadow_pipeline=args.run_perception_shadow_pipeline,
        perception_shadow_config=args.perception_shadow_config,
        perception_shadow_report=args.perception_shadow_report,
        check_planner_gateway_shadow=args.check_planner_gateway_shadow,
        planner_gateway_shadow_config=args.planner_gateway_shadow_config,
        planner_gateway_shadow_report=args.planner_gateway_shadow_report,
        perception_shadow_result=args.perception_shadow_result,
        check_ros2_interface_readiness=args.check_ros2_interface_readiness,
        ros2_interface_config=args.ros2_interface_config,
        ros2_interface_report=args.ros2_interface_report,
        check_ros2_message_export=args.check_ros2_message_export,
        ros2_message_export_config=args.ros2_message_export_config,
        ros2_message_export_report=args.ros2_message_export_report,
        check_moveit_plan_only=args.check_moveit_plan_only,
        moveit_plan_only_config=args.moveit_plan_only_config,
        moveit_plan_only_report=args.moveit_plan_only_report,
        check_ur5_read_only_state=args.check_ur5_read_only_state,
        ur5_read_only_state_config=args.ur5_read_only_state_config,
        ur5_read_only_state_report=args.ur5_read_only_state_report,
        check_robot_system_shadow_bridge=args.check_robot_system_shadow_bridge,
        robot_system_shadow_bridge_config=args.robot_system_shadow_bridge_config,
        robot_system_shadow_bridge_report=args.robot_system_shadow_bridge_report,
        output_dir=args.output_dir,
        write_report=True,
        demo_command=shlex.join([sys.executable, *sys.argv]),
    )
    report_path = Path(str(result["report_path"]))
    print_summary(result, report_path)
    return 0 if result.get("ok") else 1


def print_v3_hover_summary(result: dict) -> None:
    print("=" * 50)
    print("TETO V3.0.0 FIRST REAL UR5 HOVER DEMO")
    print("=" * 50)
    print(f"Status: {result.get('v3_hover_demo_status')}")
    print(f"Mode: {result.get('v3_demo_mode')}")
    print(f"user_command: {result.get('user_command')}")
    print(f"normalized_intent: {result.get('normalized_intent')}")
    print(f"target_label: {result.get('target_label')}")
    print(f"planner_request_ready: {result.get('planner_request_ready')}")
    print(f"ros2_interface_ready: {result.get('ros2_interface_ready')}")
    print(f"moveit_plan_ready: {result.get('moveit_plan_ready')}")
    print(f"ur5_state_ok: {result.get('ur5_state_ok')}")
    print(f"manual_confirmation_required: {result.get('manual_confirmation_required')}")
    print(f"manual_confirmation_accepted: {result.get('manual_confirmation_accepted')}")
    print(f"enable_real_robot_motion: {result.get('enable_real_robot_motion')}")
    print(f"trajectory_send_allowed: {result.get('trajectory_send_allowed')}")
    print(f"controller_command_sent: {result.get('controller_command_sent')}")
    print(f"real_robot_motion_executed: {result.get('real_robot_motion_executed')}")
    print(f"blocking_reasons: {result.get('blocking_reasons')}")
    print(f"v3_hover_demo_result_path: {result.get('v3_hover_demo_result_path')}")
    print(f"v3_hover_demo_report_path: {result.get('v3_hover_demo_report_path')}")


def print_summary(result: dict, report_path: Path) -> None:
    print("=" * 50)
    print(f"{CURRENT_TETO_VERSION} REAL-SCENE SHADOW / SIMULATION EVIDENCE")
    print("=" * 50)
    print(f"Status: {result['status']}")
    print(f"Mode: {result['mode']}")
    print(f"World reset: {result['world_reset']}")
    print(f"Steps: {result['steps_completed']}/{result['steps_requested']}")
    print(f"allow_robot_motion: {result['allow_robot_motion']}")
    print(f"cube_spawned: {result.get('cube_spawned')}")
    print(f"cube_prim_path: {result.get('cube_prim_path')}")
    print(f"cube_position: {result.get('cube_position')}")
    print(f"cube_size: {result.get('cube_size')}")
    print(f"cube_moved: {result.get('cube_moved')}")
    print(f"cube_initial_position: {result.get('cube_initial_position')}")
    print(f"cube_target_position: {result.get('cube_target_position')}")
    print(f"cube_final_position: {result.get('cube_final_position')}")
    print(f"cube_displacement: {result.get('cube_displacement')}")
    print(f"robot_asset_check_requested: {result.get('robot_asset_check_requested')}")
    print(f"robot_asset_load_requested: {result.get('robot_asset_load_requested')}")
    print(f"robot_type: {result.get('robot_type')}")
    print(f"robot_prim_path: {result.get('robot_prim_path')}")
    print(f"robot_asset_path: {result.get('robot_asset_path')}")
    print(f"robot_asset_source: {result.get('robot_asset_source')}")
    print(f"robot_asset_available: {result.get('robot_asset_available')}")
    print(f"robot_asset_loaded: {result.get('robot_asset_loaded')}")
    print(f"robot_prim_exists: {result.get('robot_prim_exists')}")
    print(f"robot_asset_status: {result.get('robot_asset_status')}")
    print(f"robot_asset_blocking_reason: {result.get('robot_asset_blocking_reason')}")
    inspection = result.get("robot_prim_inspection") or {}
    print(f"robot_prim_inspection_requested: {result.get('robot_prim_inspection_requested')}")
    print(f"robot_prim_inspection_status: {inspection.get('inspection_status')}")
    print(f"robot_root_type_name: {inspection.get('robot_root_type_name')}")
    print(f"total_descendant_prim_count: {inspection.get('total_descendant_prim_count')}")
    print(f"link_like_prim_count: {inspection.get('link_like_prim_count')}")
    print(f"joint_like_prim_count: {inspection.get('joint_like_prim_count')}")
    print(f"visual_like_prim_count: {inspection.get('visual_like_prim_count')}")
    print(f"collision_like_prim_count: {inspection.get('collision_like_prim_count')}")
    print(f"articulation_root_found: {inspection.get('articulation_root_found')}")
    joint_summary = inspection.get("joint_metadata_summary") or {}
    print(f"arm_joint_count: {joint_summary.get('arm_joint_count')}")
    print(f"arm_joint_names: {joint_summary.get('arm_joint_names')}")
    print(f"structural_joint_count: {joint_summary.get('structural_joint_count')}")
    print(f"structural_joint_names: {joint_summary.get('structural_joint_names')}")
    print(f"gripper_or_tool_joint_count: {joint_summary.get('gripper_or_tool_joint_count')}")
    print(f"gripper_or_tool_joint_names: {joint_summary.get('gripper_or_tool_joint_names')}")
    print(f"unknown_joint_count: {joint_summary.get('unknown_joint_count')}")
    print(f"unknown_joint_names: {joint_summary.get('unknown_joint_names')}")
    print(f"robot_structure_report_generated: {result.get('robot_structure_report_generated')}")
    print(f"robot_structure_report_path: {result.get('robot_structure_report_path')}")
    readiness = result.get("articulation_readiness") or {}
    print(f"articulation_readiness_requested: {result.get('articulation_readiness_requested')}")
    print(f"readiness_status: {readiness.get('readiness_status')}")
    print(f"articulation_ready: {readiness.get('articulation_ready')}")
    print(f"control_enabled: {readiness.get('control_enabled')}")
    print(f"motion_generated: {readiness.get('motion_generated')}")
    print(f"command_generated: {readiness.get('command_generated')}")
    print(f"missing_requirements: {readiness.get('missing_requirements')}")
    print(f"readiness_warnings: {readiness.get('warnings')}")
    print(f"articulation_readiness_path: {result.get('articulation_readiness_path')}")
    state = result.get("articulation_state") or {}
    print(f"articulation_state_observation_requested: {result.get('articulation_state_observation_requested')}")
    print(f"articulation_state_observable: {result.get('articulation_state_observable')}")
    print(f"articulation_state_status: {state.get('status')}")
    print(f"joint_targets_generated: {state.get('joint_targets_generated')}")
    print(f"observed_joint_count: {state.get('observed_joint_count')}")
    print(f"observed_arm_joint_names: {state.get('observed_arm_joint_names')}")
    print(f"missing_arm_joint_names: {state.get('missing_arm_joint_names')}")
    print(f"extra_joint_names: {state.get('extra_joint_names')}")
    print(f"joint_limits_available: {state.get('joint_limits_available')}")
    print(f"state_warnings: {state.get('warnings')}")
    print(f"state_errors: {state.get('errors')}")
    print(f"articulation_state_path: {result.get('articulation_state_path')}")
    print(f"articulation_state_report_path: {result.get('articulation_state_report_path')}")
    precheck = result.get("simulation_motion_precheck") or {}
    print(f"simulation_motion_precheck_requested: {result.get('simulation_motion_precheck_requested')}")
    print(f"simulation_motion_precheck_status: {result.get('simulation_motion_precheck_status')}")
    print(f"ready_for_simulation_motion: {result.get('ready_for_simulation_motion')}")
    print(f"trajectory_generated: {precheck.get('trajectory_generated')}")
    print(f"tcp_pose_world_generated: {precheck.get('tcp_pose_world_generated')}")
    print(f"robot_motion_executed: {precheck.get('robot_motion_executed')}")
    print(f"precheck_blocking_reasons: {precheck.get('blocking_reasons')}")
    print(f"precheck_warnings: {precheck.get('warnings')}")
    print(f"precheck_errors: {precheck.get('errors')}")
    print(f"simulation_motion_precheck_path: {result.get('simulation_motion_precheck_path')}")
    print(f"simulation_motion_precheck_report_path: {result.get('simulation_motion_precheck_report_path')}")
    motion = result.get("motion") or {}
    print(f"simulation_micro_motion_requested: {result.get('simulation_micro_motion_requested')}")
    print(f"simulation_micro_motion_status: {result.get('simulation_micro_motion_status')}")
    print(f"simulation_only: {result.get('simulation_only')}")
    print(f"real_robot_allowed: {result.get('real_robot_allowed')}")
    print(f"real_robot_motion_executed: {result.get('real_robot_motion_executed')}")
    print(f"micro_motion_joint_name: {motion.get('joint_name')}")
    print(f"requested_delta_rad: {motion.get('requested_delta_rad')}")
    print(f"actual_delta_rad: {motion.get('actual_delta_rad')}")
    print(f"tolerance_rad: {motion.get('tolerance_rad')}")
    print(f"delta_within_tolerance: {motion.get('delta_within_tolerance')}")
    print(f"motion_evidence_available: {result.get('motion_evidence_available')}")
    print("Motion evidence files:")
    for item in result.get("motion_evidence_files") or []:
        if isinstance(item, dict):
            print(f"- {item.get('name')}: {item.get('path')}")
    print(f"before_joint_state_path: {motion.get('before_joint_state_path')}")
    print(f"after_joint_state_path: {motion.get('after_joint_state_path')}")
    print(f"simulation_motion_result_path: {motion.get('simulation_motion_result_path')}")
    print(f"simulation_motion_report_path: {motion.get('simulation_motion_report_path')}")
    bridge = result.get("semantic_bridge") or {}
    print(f"semantic_simulation_bridge_requested: {result.get('semantic_simulation_bridge_requested')}")
    print(f"semantic_bridge_status: {result.get('semantic_bridge_status')}")
    print(f"semantic_gate_passed: {result.get('semantic_gate_passed')}")
    print(f"semantic_task_id: {result.get('semantic_task_id')}")
    print(f"semantic_intent: {result.get('semantic_intent')}")
    print(f"semantic_target_label: {result.get('semantic_target_label')}")
    print(f"semantic_bridge_blocking_reasons: {result.get('semantic_bridge_blocking_reasons')}")
    print(f"triggered_simulation_micro_motion: {result.get('triggered_simulation_micro_motion')}")
    print(f"semantic_simulation_bridge_result_path: {bridge.get('semantic_simulation_bridge_result_path')}")
    print(f"semantic_simulation_bridge_report_path: {bridge.get('semantic_simulation_bridge_report_path')}")
    execution = result.get("simulated_task_execution") or {}
    post_check = result.get("post_motion_state_check") or {}
    print(f"safe_simulated_task_execution_requested: {result.get('safe_simulated_task_execution_requested')}")
    print(f"execution_attempt_id: {result.get('execution_attempt_id')}")
    print(f"simulated_task_status: {result.get('simulated_task_status')}")
    print(f"execution_feedback_status: {result.get('execution_feedback_status')}")
    print(f"failure_reason: {result.get('failure_reason')}")
    print(f"retry_recommended: {result.get('retry_recommended')}")
    print(f"fallback_recommended: {result.get('fallback_recommended')}")
    print(f"fallback_type: {result.get('fallback_type')}")
    print(f"post_motion_state_check_status: {post_check.get('post_motion_state_check_status')}")
    print(f"simulated_task_execution_result_path: {execution.get('simulated_task_execution_result_path')}")
    print(f"simulated_task_execution_report_path: {execution.get('simulated_task_execution_report_path')}")
    print(f"lab_readiness_requested: {result.get('lab_readiness_requested')}")
    print(f"lab_backend_readiness_status: {result.get('lab_backend_readiness_status')}")
    print(f"camera_readiness_status: {result.get('camera_readiness_status')}")
    print(f"live_vlm_readiness_status: {result.get('live_vlm_readiness_status')}")
    print(f"shadow_mode_readiness_status: {result.get('shadow_mode_readiness_status')}")
    print(f"no_motion_readiness_passed: {result.get('no_motion_readiness_passed')}")
    print(f"allow_live_camera: {result.get('allow_live_camera')}")
    print(f"allow_live_vlm: {result.get('allow_live_vlm')}")
    print(f"real_robot_command_enabled: {result.get('real_robot_command_enabled')}")
    print(f"readiness_blocking_reasons: {result.get('readiness_blocking_reasons')}")
    print(f"next_safe_action: {result.get('next_safe_action')}")
    print(f"camera_source_requested: {result.get('camera_source_requested')}")
    print(f"camera_source_status: {result.get('camera_source_status')}")
    print(f"camera_source_mode: {result.get('camera_source_mode')}")
    print(f"camera_source_snapshot_id: {result.get('camera_source_snapshot_id')}")
    print(f"no_motion_camera_adapter_passed: {result.get('no_motion_camera_adapter_passed')}")
    print(f"camera_source_blocking_reasons: {result.get('camera_source_blocking_reasons')}")
    print(f"camera_snapshot_requested: {result.get('camera_snapshot_requested')}")
    print(f"camera_snapshot_id: {result.get('camera_snapshot_id')}")
    print(f"camera_snapshot_validity_status: {result.get('camera_snapshot_validity_status')}")
    print(f"camera_snapshot_blocking_reasons: {result.get('camera_snapshot_blocking_reasons')}")
    print(f"no_motion_snapshot_passed: {result.get('no_motion_snapshot_passed')}")
    print(f"live_capture_used: {result.get('live_capture_used')}")
    print(f"live_camera_enabled: {result.get('live_camera_enabled')}")
    vlm_grounding = result.get("vlm_grounding") or {}
    print(f"vlm_grounding_requested: {result.get('vlm_grounding_requested')}")
    print(f"vlm_grounding_status: {result.get('vlm_grounding_status')}")
    print(f"vlm_grounding_id: {result.get('vlm_grounding_id')}")
    print(f"vlm_grounding_snapshot_id: {result.get('vlm_grounding_snapshot_id')}")
    print(f"vlm_grounding_scene_version: {result.get('vlm_grounding_scene_version')}")
    print(f"vlm_grounding_user_command: {result.get('vlm_grounding_user_command')}")
    print(f"vlm_grounding_normalized_command: {result.get('vlm_grounding_normalized_command')}")
    print(f"vlm_grounding_adapter_mode: {result.get('vlm_grounding_adapter_mode')}")
    print(f"vlm_grounding_target_label: {result.get('vlm_grounding_target_label')}")
    print(f"vlm_grounding_bbox_xyxy: {vlm_grounding.get('bbox_xyxy')}")
    print(f"vlm_grounding_pixel_center: {vlm_grounding.get('pixel_center')}")
    print(f"no_motion_grounding_passed: {result.get('no_motion_grounding_passed')}")
    print(f"vlm_grounding_blocking_reasons: {result.get('vlm_grounding_blocking_reasons')}")
    print(f"vlm_grounding_warnings: {result.get('vlm_grounding_warnings')}")
    print(f"vlm_grounding_result_path: {result.get('vlm_grounding_result_path')}")
    print(f"vlm_grounding_report_path: {result.get('vlm_grounding_report_path')}")
    print(f"geometry_validity_requested: {result.get('geometry_validity_requested')}")
    print(f"geometry_validity_snapshot_id: {result.get('geometry_validity_snapshot_id')}")
    print(f"geometry_validity_grounding_id: {result.get('geometry_validity_grounding_id')}")
    print(f"geometry_validity_status: {result.get('geometry_validity_status')}")
    print(f"no_motion_geometry_passed: {result.get('no_motion_geometry_passed')}")
    print(f"geometry_validity_blocking_reasons: {result.get('geometry_validity_blocking_reasons')}")
    print(f"projector_shadow_requested: {result.get('projector_shadow_requested')}")
    print(f"projector_requested: {result.get('projector_requested')}")
    print(f"projector_snapshot_id: {result.get('projector_snapshot_id')}")
    print(f"projector_grounding_id: {result.get('projector_grounding_id')}")
    print(f"projector_status: {result.get('projector_status')}")
    print(f"no_motion_projector_passed: {result.get('no_motion_projector_passed')}")
    print(f"projector_blocking_reasons: {result.get('projector_blocking_reasons')}")
    print(f"real_scene_shadow_requested: {result.get('real_scene_shadow_requested')}")
    print(f"real_scene_shadow_snapshot_id: {result.get('real_scene_shadow_snapshot_id')}")
    print(f"real_scene_shadow_grounding_id: {result.get('real_scene_shadow_grounding_id')}")
    print(f"real_scene_shadow_status: {result.get('real_scene_shadow_status')}")
    print(f"real_scene_shadow_semantic_gate_passed: {result.get('semantic_gate_passed')}")
    print(f"no_motion_shadow_passed: {result.get('no_motion_shadow_passed')}")
    print(f"real_scene_shadow_replay_ready: {result.get('real_scene_shadow_replay_ready')}")
    print(f"real_scene_shadow_blocking_reasons: {result.get('real_scene_shadow_blocking_reasons')}")
    perception = result.get("perception_shadow") or {}
    print(f"perception_shadow_requested: {result.get('perception_shadow_requested')}")
    print(f"perception_shadow_status: {result.get('perception_shadow_status')}")
    print(f"perception_shadow_snapshot_id: {result.get('perception_shadow_snapshot_id')}")
    print(f"perception_shadow_grounding_id: {result.get('perception_shadow_grounding_id')}")
    print(f"perception_shadow_scene_version: {result.get('perception_shadow_scene_version')}")
    print(f"perception_shadow_user_command: {result.get('perception_shadow_user_command')}")
    print(f"perception_shadow_normalized_command: {result.get('perception_shadow_normalized_command')}")
    print(f"perception_shadow_camera_source_status: {result.get('perception_shadow_camera_source_status')}")
    print(f"perception_shadow_vlm_grounding_status: {result.get('perception_shadow_vlm_grounding_status')}")
    print(f"perception_shadow_real_scene_shadow_status: {result.get('perception_shadow_real_scene_shadow_status')}")
    print(f"perception_shadow_geometry_validity_status: {result.get('perception_shadow_geometry_validity_status')}")
    print(f"perception_shadow_projector_status: {result.get('perception_shadow_projector_status')}")
    print(f"perception_shadow_target_label: {result.get('perception_shadow_target_label')}")
    print(f"perception_shadow_world_point_m: {perception.get('world_point_m')}")
    print(f"no_motion_perception_passed: {result.get('no_motion_perception_passed')}")
    print(f"perception_shadow_replay_ready: {result.get('perception_shadow_replay_ready')}")
    print(f"perception_shadow_blocking_reasons: {result.get('perception_shadow_blocking_reasons')}")
    print(f"perception_shadow_result_path: {result.get('perception_shadow_result_path')}")
    print(f"perception_shadow_report_path: {result.get('perception_shadow_report_path')}")
    gateway = result.get("planner_gateway_shadow") or {}
    print(f"planner_gateway_shadow_requested: {result.get('planner_gateway_shadow_requested')}")
    print(f"planner_gateway_shadow_status: {result.get('planner_gateway_shadow_status')}")
    print(f"planner_gateway_shadow_gateway_request_id: {result.get('planner_gateway_shadow_gateway_request_id')}")
    print(f"planner_gateway_shadow_task_id: {result.get('planner_gateway_shadow_task_id')}")
    print(f"planner_gateway_shadow_intent_name: {result.get('planner_gateway_shadow_intent_name')}")
    print(f"planner_gateway_shadow_target_label: {result.get('planner_gateway_shadow_target_label')}")
    print(f"planner_gateway_shadow_snapshot_id: {result.get('planner_gateway_shadow_snapshot_id')}")
    print(f"planner_gateway_shadow_grounding_id: {result.get('planner_gateway_shadow_grounding_id')}")
    print(f"planner_gateway_shadow_scene_version: {result.get('planner_gateway_shadow_scene_version')}")
    print(f"planner_gateway_shadow_world_frame: {result.get('planner_gateway_shadow_world_frame')}")
    print(f"planner_gateway_shadow_world_point_m: {gateway.get('world_point_m')}")
    print(f"planner_gateway_shadow_bounded_target_point_m: {gateway.get('bounded_target_point_m')}")
    print(f"planner_input_ready: {result.get('planner_input_ready')}")
    print(f"planner_gateway_shadow_blocking_reasons: {result.get('planner_gateway_shadow_blocking_reasons')}")
    print(f"planner_gateway_shadow_result_path: {result.get('planner_gateway_shadow_result_path')}")
    print(f"planner_gateway_shadow_report_path: {result.get('planner_gateway_shadow_report_path')}")
    ros2_interface = result.get("ros2_interface_readiness") or {}
    print(f"ros2_interface_readiness_requested: {result.get('ros2_interface_readiness_requested')}")
    print(f"ros2_interface_readiness_status: {result.get('ros2_interface_readiness_status')}")
    print(f"ros2_environment_declared: {result.get('ros2_environment_declared')}")
    print(f"ros_distro: {result.get('ros_distro')}")
    print(f"ros_domain_id: {result.get('ros_domain_id')}")
    print(f"planner_gateway_interface_mode: {result.get('planner_gateway_interface_mode')}")
    print(f"planner_gateway_endpoint: {result.get('planner_gateway_endpoint')}")
    print(f"message_schema: {result.get('message_schema')}")
    print(f"ros2_interface_world_frame: {result.get('ros2_interface_world_frame')}")
    print(f"robot_base_frame: {result.get('robot_base_frame')}")
    print(f"ros2_interface_camera_frame: {result.get('ros2_interface_camera_frame')}")
    print(f"shadow_only: {result.get('shadow_only')}")
    print(f"ros2_interface_blocking_reasons: {result.get('ros2_interface_blocking_reasons')}")
    print(f"ros2_interface_warnings: {result.get('ros2_interface_warnings')}")
    print(f"ros2_interface_readiness_result_path: {result.get('ros2_interface_readiness_result_path')}")
    print(f"ros2_interface_readiness_report_path: {result.get('ros2_interface_readiness_report_path')}")
    print(f"ros2_interface_safety_boundary: {ros2_interface.get('safety_boundary')}")
    message_export = result.get("ros2_message_export") or {}
    print(f"ros2_message_export_requested: {result.get('ros2_message_export_requested')}")
    print(f"ros2_message_export_status: {result.get('ros2_message_export_status')}")
    print(f"message_export_status: {result.get('message_export_status')}")
    print(f"ros2_message_id: {result.get('ros2_message_id')}")
    print(f"ros2_message_schema: {result.get('ros2_message_schema')}")
    print(f"fake_publish_only: {result.get('fake_publish_only')}")
    print(f"ros2_message_export_blocking_reasons: {result.get('ros2_message_export_blocking_reasons')}")
    print(f"ros2_message_export_warnings: {result.get('ros2_message_export_warnings')}")
    print(f"ros2_message_export_result_path: {result.get('ros2_message_export_result_path')}")
    print(f"ros2_message_export_report_path: {result.get('ros2_message_export_report_path')}")
    print(f"ros2_message_export_safety_boundary: {message_export.get('safety_boundary')}")
    moveit_plan_only = result.get("moveit_plan_only") or {}
    print(f"moveit_plan_only_requested: {result.get('moveit_plan_only_requested')}")
    print(f"moveit_plan_only_status: {result.get('moveit_plan_only_status')}")
    print(f"plan_only_status: {result.get('plan_only_status')}")
    print(f"plan_only_ready: {result.get('plan_only_ready')}")
    print(f"planning_group: {result.get('planning_group')}")
    print(f"planning_frame: {result.get('planning_frame')}")
    print(f"end_effector_frame: {result.get('end_effector_frame')}")
    print(f"moveit_plan_only_blocking_reasons: {result.get('moveit_plan_only_blocking_reasons')}")
    print(f"moveit_plan_only_result_path: {result.get('moveit_plan_only_result_path')}")
    print(f"moveit_plan_only_report_path: {result.get('moveit_plan_only_report_path')}")
    print(f"moveit_plan_only_safety_boundary: {moveit_plan_only.get('safety_boundary')}")
    ur5_state = result.get("ur5_read_only_state") or {}
    print(f"ur5_read_only_state_requested: {result.get('ur5_read_only_state_requested')}")
    print(f"ur5_read_only_state_status: {result.get('ur5_read_only_state_status')}")
    print(f"read_only_state_status: {result.get('read_only_state_status')}")
    print(f"read_only_state_contract_ready: {result.get('read_only_state_contract_ready')}")
    print(f"ur5_read_only_state_blocking_reasons: {result.get('ur5_read_only_state_blocking_reasons')}")
    print(f"ur5_read_only_state_result_path: {result.get('ur5_read_only_state_result_path')}")
    print(f"ur5_read_only_state_report_path: {result.get('ur5_read_only_state_report_path')}")
    print(f"ur5_read_only_state_safety_boundary: {ur5_state.get('safety_boundary')}")
    bridge = result.get("robot_system_shadow_bridge") or {}
    print(f"robot_system_shadow_bridge_requested: {result.get('robot_system_shadow_bridge_requested')}")
    print(f"robot_system_shadow_bridge_status: {result.get('robot_system_shadow_bridge_status')}")
    print(f"robot_system_shadow_status: {result.get('robot_system_shadow_status')}")
    print(f"robot_system_shadow_ready: {result.get('robot_system_shadow_ready')}")
    print(f"robot_system_shadow_bridge_blocking_reasons: {result.get('robot_system_shadow_bridge_blocking_reasons')}")
    print(f"robot_system_shadow_bridge_result_path: {result.get('robot_system_shadow_bridge_result_path')}")
    print(f"robot_system_shadow_bridge_report_path: {result.get('robot_system_shadow_bridge_report_path')}")
    print(f"robot_system_shadow_bridge_safety_boundary: {bridge.get('safety_boundary')}")
    print(f"Report: {report_path}")
    if result.get("blocking_reasons"):
        print(f"Blocking reasons: {', '.join(result['blocking_reasons'])}")
    error = result.get("error", {})
    if error.get("code") != "OK":
        print(f"Error: {error.get('code')}: {error.get('message')}")
    print("=" * 50)


def _load_simulation_task(task_json: str | None) -> dict:
    if not task_json:
        return dict(DEFAULT_SIMULATION_TASK)
    path = Path(task_json).expanduser()
    with path.open("r", encoding="utf-8") as task_file:
        data = json.load(task_file)
    if isinstance(data, dict) and isinstance(data.get("simulation_task"), dict):
        return data["simulation_task"]
    if isinstance(data, dict):
        return data
    raise ValueError("simulation task JSON must contain an object")


if __name__ == "__main__":
    raise SystemExit(main())
