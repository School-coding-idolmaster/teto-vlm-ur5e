# H8 Module Boundaries

H8 exists to make the next research phases easier to reason about without
moving any current implementation code. The future work includes Camera-to-Base
TF calibration, formal D455 evidence, Qwen grounding, projected 3D target
records, and red mug hover validation. H8 creates names for those boundaries
while preserving every current runtime import path and safety behavior.

## Package Boundaries

- `src/calibration/`: future camera-to-base TF, D455 extrinsics, calibration
  snapshots, and projection metadata.
- `src/vision/`: future D455 capture, snapshot replay, camera sources, image
  preprocessing, and model-safe visual inputs.
- `src/grounding/`: future Qwen grounding, target selection, 2D grounding
  evidence, projected 3D target evidence, and red mug hover target records.
- `src/contracts/`: future shared schemas, evidence contracts, readiness
  contracts, and validation helpers.
- `src/planning/`: future task intent, planner gateway, bounded target, command
  normalization, and motion planning contract logic.
- `src/execution/`: future real and simulation execution adapters, backend
  interfaces, MoveIt routing, Isaac routing, and measured execution evidence.
- `src/replay/`: future replay manifests, scene records, snapshot replay,
  semantic replay, and evidence lookup utilities.
- `src/safety/`: future shared safety policies, gate wrappers, and safety
  evidence semantics.
- `src/entrypoints/`: future Python entrypoint helpers and operator launch
  boundary notes.

## Existing Files Kept In Place

H8 keeps current runtime files in their existing locations. Files that logically
map to the future boundaries but remain in place include:

- Calibration and vision: `src/camera_snapshot.py`,
  `src/camera_source_adapter.py`, `src/realsense_snapshot_builder.py`,
  `src/projector_shadow.py`, `src/image_utils.py`, and `src/vlm_infer.py`.
- Grounding: `src/vlm_grounding_adapter.py`, `src/grounding_result.py`,
  `src/geometry_validity.py`, `src/real_scene_shadow_pipeline.py`, and
  `src/perception_shadow_pipeline.py`.
- Contracts: `src/bounded_relative_motion.py`, `src/json_validator.py`,
  `src/execution_readiness_contract.py`, `src/projector_contract.py`,
  `src/planner_gateway_contract.py`, `src/moveit_plan_only_contract.py`,
  `src/ur5_read_only_state_contract.py`, `src/simulation_bridge_contract.py`,
  `src/ros2_interface_readiness.py`, `src/ros2_message_exporter.py`, and
  `src/robot_system_shadow_bridge.py`.
- Planning: `src/planner_gateway_shadow.py`, `src/command_to_task_adapter.py`,
  `src/motion_command_normalizer.py`, `src/qwen_motion_parser.py`,
  `src/autoregressive_motion_planner.py`, and
  `src/vector_autoregressive_motion_planner.py`.
- Execution: `src/unified_segmented_operator.py`,
  `src/real_segmented_operator_backend.py`, `src/cartesian_motion_gateway.py`,
  `src/moveit_pose_executor.py`, `src/isaac_sim_operator.py`,
  `src/isaac_sim_bridge.py`, `src/simulation_micro_motion.py`, and
  `src/simulated_task_execution.py`.
- Replay: `src/robot_task_inspector.py`, `src/semantic_simulation_bridge.py`,
  `src/evidence_exporter.py`, `src/output_paths.py`, and
  `src/simulation_runtime.py`.
- Safety and entrypoints: `src/manual_confirmation_gate.py`,
  `scripts/teto_operator_console.py`, `scripts/teto_isaac_operator_console.py`,
  `scripts/start_teto_real_full_stack.sh`,
  `scripts/start_teto_qwen_real_operator.sh`,
  `scripts/start_teto_isaac_gui_operator.sh`,
  `scripts/qwen_operator_console.sh`, and
  `scripts/safety_harnesses/run_real_long_motion_safety_check.py`.

Keeping these files in place avoids import churn around real execution,
Isaac SIM_ONLY execution, replay evidence, and safety tests.

## H8 Prohibitions

H8 does not:

- Move current `src` files.
- Change imports or re-export existing runtime modules.
- Change runtime behavior.
- Change shell startup script behavior.
- Change safety semantics.
- Change the real backend.
- Change the Isaac backend.
- Delete or rewrite tests.
- Start hardware, Isaac Sim, ROS, MoveIt, RealSense, Qwen, or any model.

## Future Migration Candidates

H9 or H10 may audit and migrate code into these package boundaries after
dedicated test coverage and safety review. Candidate migrations include:

- Camera-to-base calibration schemas and replay-safe TF metadata.
- Shared schema definitions for snapshot, grounding, projection, planner, and
  replay evidence.
- A shared no-robot-control field policy helper, if import cycles can be
  avoided.
- Replay manifest utilities that unify legacy robot-task replay with the newer
  snapshot and evidence export paths.
- Backend-neutral execution adapter interfaces, without mixing real and Isaac
  safety surfaces.

Any such migration should happen in small commits with focused safety tests and
without changing real or Isaac behavior unless explicitly approved.

## H8 Acceptance Criteria

- The nine package boundary directories exist.
- Each package has a minimal `__init__.py` with only a docstring and empty
  `__all__`.
- Each package has a README that describes future responsibility and why
  existing files stay in place.
- Schema README files exist only for `calibration`, `grounding`, `contracts`,
  and `replay`.
- No existing runtime file is moved.
- No existing runtime import is changed.
- No runtime behavior, startup behavior, real backend behavior, Isaac backend
  behavior, or safety semantics changes.
- Tests are not deleted or rewritten.
- Compile, pytest, and `git diff --check` pass.
