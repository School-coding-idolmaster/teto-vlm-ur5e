# Projector Module Guide

This guide records the H10 projector boundary policy for future Codex, GPT,
and human audits. The concrete 2D-to-3D metric projector implementation now
lives in `src/projector/shadow.py`. The former root-level
`src/projector_shadow.py` compatibility shim was removed in H10-A5.

## Current Responsibility

The projector is the 2D-to-3D metric layer between grounding/geometry evidence
and planner-gateway input evidence.

It is responsible for:

- Consuming a `geometry_validity` result that already passed grounding,
  snapshot identity, bbox, pixel-center, confidence, depth-availability, and
  TTL checks.
- Loading declared projector config from `projector_shadow` config data.
- Reading inline or referenced `camera_info`, `depth_sample`, and `mock_tf`
  payloads.
- Converting `pixel_center`, depth, and pinhole intrinsics into
  `camera_point_m`.
- Applying a mock/config transform into `world_point_m` when world projection
  is required.
- Reporting `projection_confidence`, `projection_method`, TF availability,
  workspace check status, blocking reasons, warnings, and no-motion safety
  evidence.

The current formal implementation is:

- `src/projector/shadow.py`: current V2.9.2 projector shadow implementation.
- `src/projector_contract.py`: older semantic dry-run eligibility contract
  used by replay/readiness tooling. It does not compute metric points and is
  not migrated into the projector package yet.

## Inputs

`ProjectorShadowRequest` currently accepts:

- `requested`
- `config_path`
- `projector_config`
- `camera_snapshot_config`
- `grounding_result_path`
- `geometry_validity_config`

`evaluate_projector_shadow_from_contracts` consumes:

- `geometry_validity`: the upstream geometry validity result.
- `projector_config`: inline config dictionary.
- Optional path metadata: `config_path`, `camera_snapshot_config`,
  `grounding_result_path`, and `geometry_validity_config`.

The projector config may provide:

- `camera_info` or `camera_info_ref`
- `depth_sample`, `depth_sample_ref`, or `depth_value_m`
- `mock_tf` or `mock_tf_ref`
- `camera_frame`
- `world_frame`
- `projection_method`
- `projection_confidence`
- `require_world_projection`
- `min_depth_m`, `max_depth_m`, or `depth_range_m`
- `workspace_m`
- no-motion audit flags such as `real_tf_used` or `ros2_tf_used`

The geometry result must provide, or carry through nested contracts:

- `geometry_validity_status`
- `pixel_center`
- `snapshot_id`
- `grounding_id`
- `scene_version`
- nested `camera_snapshot`
- nested `grounding_result`
- live-camera/live-VLM flags
- forbidden robot-control field evidence

## Outputs

The current projector shadow output includes:

- `contract_version`
- `teto_version`
- `projector_requested`
- `requested`
- `config_path`
- `camera_snapshot_config`
- `grounding_result_path`
- `geometry_validity_config`
- `geometry_validity`
- `projector_status`
- `snapshot_id`
- `grounding_id`
- `scene_version`
- `pixel_center`
- `depth_value_m`
- `depth_valid`
- `camera_intrinsics_available`
- `camera_frame`
- `world_frame`
- `camera_point_m`
- `world_point_m`
- `projection_confidence`
- `projection_method`
- `tf_available`
- `tf_source`
- `real_tf_used`
- `ros2_tf_used`
- `workspace_check_passed`
- `blocking_reasons`
- `warnings`
- `next_safe_action`
- `no_motion_projector_passed`
- `live_camera_used`
- `live_vlm_called`
- `real_robot_motion_executed`
- `real_robot_command_enabled`
- `robot_command_generated`
- `trajectory_generated`
- `joint_targets_generated`
- `tcp_pose_world_generated`
- `forbidden_robot_control_fields`
- `safety_boundary`

Do not rename, remove, reorder behavior around, or reinterpret these fields
without a dedicated migration plan.

## Error Codes

Current `src/projector/shadow.py` blocking codes:

- `E_GEOMETRY_NOT_VALID`
- `E_PIXEL_CENTER_MISSING`
- `E_CAMERA_INFO_MISSING`
- `E_INVALID_CAMERA_INTRINSICS`
- `E_NO_DEPTH`
- `E_INVALID_DEPTH`
- `E_DEPTH_OUT_OF_RANGE`
- `E_CAMERA_FRAME_MISSING`
- `E_WORLD_FRAME_MISSING`
- `E_TF_UNAVAILABLE`
- `E_INVALID_PROJECTION`
- `E_OUT_OF_WORKSPACE`
- `E_LIVE_CAMERA_DISABLED`
- `E_LIVE_VLM_DISABLED`
- `E_ROBOT_COMMAND_NOT_ALLOWED`

Current `src/projector_contract.py` vocabulary also includes future dry-run
runtime codes such as `E_WORLD_TRANSFORM_FAILED` and
`E_LOW_PROJECTOR_CONFIDENCE`. Preserve this vocabulary unless a future task
explicitly audits compatibility.

## Projection Semantics

The current projection method defaults to `pinhole_mock_tf`.

Metric camera projection uses the standard pinhole form:

```text
x = (u - cx) * z / fx
y = (v - cy) * z / fy
z = depth_value_m
```

Where:

- `u, v` come from `pixel_center`.
- `fx, fy, cx, cy` come from `camera_info.intrinsics` or direct camera info.
- `z` comes from `depth_value_m` or `depth_sample.depth_value_m`.

Depth is valid only when it is a positive finite number inside the configured
range. The default range is `0.05 m` to `5.0 m`.

## TF Semantics

The projector currently accepts only mock/config transform evidence.

- `mock_tf` can carry `translation_m` and optional `rotation_matrix`.
- If `rotation_matrix` is omitted, the camera point is translated directly.
- `tf_source` defaults to `mock_or_config`.
- `real_tf_used` and `ros2_tf_used` are always returned as `False`.
- If config or mock TF declares real or ROS2 TF usage, projector blocks with
  `E_TF_UNAVAILABLE` and emits `real_or_ros2_tf_requested_but_disabled`.

Do not introduce real ROS2 tf2, live TF lookup, RealSense TF access, or
camera-to-base calibration execution in projector documentation-only or
package-boundary work.

## Workspace Semantics

`workspace_m` is optional. If it is present, it must provide valid `[min, max]`
limits for `x`, `y`, and `z`.

The workspace check uses `world_point_m` when available, otherwise
`camera_point_m`. If the projected point is valid but outside configured
workspace bounds, the projector blocks with `E_OUT_OF_WORKSPACE`.

Workspace checking here is projector evidence only. It is not the final
execution safety gate and must not be treated as permission to move a robot.

## Confidence Semantics

`projection_confidence` is selected in this order:

- explicit `projector_config["projection_confidence"]`
- nested grounding `overall_confidence`
- `1.0` when geometry validity passed
- `0.0` when geometry validity did not pass

This confidence is evidence for projection quality. It does not authorize
planning or execution.

## No-Motion Safety Boundary

The projector must keep these output fields false:

- `live_camera_used`
- `live_vlm_called`
- `real_robot_motion_executed`
- `real_robot_command_enabled`
- `robot_command_generated`
- `trajectory_generated`
- `joint_targets_generated`
- `tcp_pose_world_generated`
- `real_tf_used`
- `ros2_tf_used`

The `safety_boundary` must continue to state no live camera capture, no live
VLM call, no real UR5 connection, no ROS2, no ROS2 TF, no MoveIt, no RTDE, no
URScript, no Dashboard, no trajectory, no `tcp_pose_world`, no joint targets,
no robot command, no real execution request, and no automatic retry motion.

## Boundaries

Grounding owns:

- command normalization
- target label/object id
- bbox and pixel center evidence
- semantic, grounding, and overall confidence fields
- grounding rejection state
- `snapshot_id` and `scene_version` carried with grounding evidence

Scene snapshot and camera source code own:

- image/depth/camera-info/metadata/TF refs
- capture timestamp and TTL
- formal RealSense replay source validation
- live capture blocking and no-live-camera source evidence

Geometry validity owns:

- snapshot/grounding identity match checks
- image size, bbox, and pixel-center validity
- depth availability requirement
- camera frame and frame id availability
- grounding confidence threshold
- TTL freshness
- grounded/rejected state before projection

Projector owns:

- depth value extraction
- camera intrinsics validation
- pinhole pixel/depth to `camera_point_m`
- mock/config transform to `world_point_m`
- projector-local workspace check
- projector-local no-motion/TF audit evidence

Perception shadow pipeline owns:

- orchestration across camera source, camera snapshot, grounding, real-scene
  semantic gate, geometry validity, and projector shadow
- pipeline-level stage statuses and identity mismatch aggregation
- replay-ready perception evidence

Planner gateway owns:

- consuming perception `world_point_m`
- validating planner intent and planner input readiness
- bounding the hover target point
- preserving manual-confirmation, ROS2, MoveIt, and execution-disabled shadow
  semantics

Execution and safety own:

- MoveIt execute behavior
- real or Isaac execution backends
- measured robot readiness and post-motion verification
- Cartesian safety gateway behavior
- any safety-critical execution gate

## Import And Packaging Policy

Current imports should remain explicit:

- `from src.projector.shadow import ...`
- `from src.projector_contract import ...`

The `src.projector` package root intentionally does not collect or re-export
public API. Import concrete projector symbols from `src.projector.shadow`.

The historical root-level Python shim has been removed:

- `src/projector_shadow.py`

Do not restore the old shim or add an import hack for the removed root-level
module. Runtime code, tests, and new utilities should import from
`src.projector.shadow` directly.

Artifact, config, and evidence names such as `projector_shadow_config`,
`projector_shadow_result.json`, and `projector_shadow_report.md` remain data
contract names. Do not rename those fields as part of Python import cleanup.

`src/projector_contract.py` remains at the root level for now. It belongs to
the older semantic dry-run readiness/contract line and must not be migrated
alongside projector shadow without a separate audit.

Known current import consumers include:

- `src/perception_shadow_pipeline.py`
- `src/simulation_runtime.py`
- `src/evidence_exporter.py`
- `src/robot_task_inspector.py`
- `src/execution_readiness_contract.py`
- projector, geometry, perception, and replay-oriented tests

## Focused Tests

Recommended focused checks after projector documentation or behavior-neutral
boundary work:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONPYCACHEPREFIX=/tmp/teto_codex_pycache .venv_lab/bin/python -m pytest -p no:cacheprovider -q \
  tests/test_projector_shadow.py \
  tests/test_projector_import_compatibility.py \
  tests/test_projector_contract.py \
  tests/test_geometry_validity.py \
  tests/test_perception_shadow_pipeline.py \
  tests/test_real_scene_shadow_pipeline.py
```

Startup script syntax checks only:

```bash
bash -n scripts/start_teto_real_full_stack.sh
bash -n scripts/start_teto_isaac_gui_operator.sh
```

Do not start hardware, Isaac Sim, ROS, MoveIt, RealSense, Qwen, VLM, LLM, or
model services for projector module maintenance.

## Future Work

Low-risk next steps:

- Add more contract-only documentation for projector fixtures and evidence
  manifests.
- Audit duplicate forbidden robot-control field handling outside grounding.

Higher-risk future steps:

- Splitting projector math helpers into a package.
- Migrating `src/projector_contract.py` into a package.
- Replacing mock/config TF with formal calibrated TF evidence.
- Integrating formal RealSense depth sampling.

Those higher-risk steps need separate audits because they touch visual-motion
contracts, calibration semantics, and downstream planner/execution boundaries.
