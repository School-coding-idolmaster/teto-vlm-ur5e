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


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Run TETO V2.9.2 projector shadow, geometry validity, real-scene no-motion shadow, and safe simulation evidence smoke test."
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
        help="Maximum attempts metadata. V2.9.2 supports 1 and does not auto-retry.",
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
    return parser


def main() -> int:
    args = build_parser().parse_args()
    if args.execution_max_attempts != 1:
        raise ValueError("--execution-max-attempts currently supports only 1 in TETO V2.9.2")
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
        check_camera_snapshot=args.check_camera_snapshot,
        camera_snapshot_config=args.camera_snapshot_config,
        camera_snapshot_report=args.camera_snapshot_report,
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
        output_dir=args.output_dir,
        write_report=True,
        demo_command=shlex.join([sys.executable, *sys.argv]),
    )
    report_path = Path(str(result["report_path"]))
    print_summary(result, report_path)
    return 0 if result.get("ok") else 1


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
    print(f"camera_snapshot_requested: {result.get('camera_snapshot_requested')}")
    print(f"camera_snapshot_id: {result.get('camera_snapshot_id')}")
    print(f"camera_snapshot_validity_status: {result.get('camera_snapshot_validity_status')}")
    print(f"camera_snapshot_blocking_reasons: {result.get('camera_snapshot_blocking_reasons')}")
    print(f"no_motion_snapshot_passed: {result.get('no_motion_snapshot_passed')}")
    print(f"live_capture_used: {result.get('live_capture_used')}")
    print(f"live_camera_enabled: {result.get('live_camera_enabled')}")
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
