# Contracts And Readiness Module Guide

This guide records the H13 contracts/readiness boundary policy for future
Codex, GPT, and human audits. H13-A completed a read-only boundary audit.
H13-B is documentation-only: no implementation files are moved, no imports are
changed, no runtime APIs are created, no `src/readiness/` package is created,
and no launch behavior is modified.

## Boundary Principles

Contracts define stable evidence, schema, request/result, and compatibility
shapes. They describe what evidence means and which fields must remain stable.

Readiness checks decide whether required preconditions are satisfied. They may
block missing, stale, unsafe, inconsistent, or incomplete evidence, but they
must not grant robot execution permission.

Safety gates retain veto power. A contract or readiness result can make
downstream work eligible for further review, simulation, replay, or evidence
export, but it cannot bypass real-path safety checks or execution gateways.

Contracts and readiness modules must keep fail-closed no-motion semantics
explicit. Fields such as `allow_robot_motion=False`, `dry_run_only=True`,
`fake_publish_only=True`, `plan_only=True`, and read-only UR5 declarations are
safety and evidence semantics, not formatting details.

## H13-A Classification

Pure/shared contract candidates:

- `src/projector_contract.py`
- `src/planner_gateway_contract.py`
- `src/execution_readiness_contract.py`
- `src/simulation_bridge_contract.py`

Schema/normalization candidate:

- `src/json_validator.py`

Safety envelope contract:

- `src/bounded_relative_motion.py`

Readiness/safety contracts:

- `src/lab_readiness.py`
- `src/ros2_interface_readiness.py`
- `src/moveit_plan_only_contract.py`
- `src/ur5_read_only_state_contract.py`
- `src/robot_system_shadow_bridge.py`
- `src/articulation_readiness_contract.py`

Artifact/export contract:

- `src/ros2_message_exporter.py`

Mixed pipeline or implementation-adjacent files:

- `src/geometry_validity.py`
- `src/planner_gateway_shadow.py`
- `src/projector/shadow.py`

Already vision-owned:

- `src/vision/snapshot/camera_snapshot.py`

The mixed files should not be treated as pure contracts without another audit.
They coordinate current package outputs, projection behavior, planner-facing
shadow evidence, artifact paths, or implementation-adjacent checks.

## Sensitivity Classification

REAL_PATH-sensitive files:

- `src/bounded_relative_motion.py`
- `src/moveit_plan_only_contract.py`
- `src/ur5_read_only_state_contract.py`
- `src/ros2_interface_readiness.py`
- `src/ros2_message_exporter.py`
- `src/robot_system_shadow_bridge.py`
- `src/lab_readiness.py`

SIM_ONLY files:

- `src/articulation_readiness_contract.py`
- `src/simulation_bridge_contract.py`

SHARED_BUT_SAFE files:

- `src/projector_contract.py`
- `src/planner_gateway_contract.py`
- `src/execution_readiness_contract.py`
- `src/json_validator.py`

Replay/artifact-sensitive files:

- `src/geometry_validity.py`
- `src/vision/snapshot/camera_snapshot.py`
- `src/ros2_message_exporter.py`
- `src/simulation_bridge_contract.py`

CLI/evidence-sensitive surfaces:

- evidence exporter consumers
- report formatter functions
- files whose output is consumed by replay, reports, or offline acceptance

If a future task spans more than one sensitivity class, split the task or stop
for another boundary audit.

## Public APIs And Invariants

These public APIs and evidence semantics must not break:

- contract version strings
- public dataclasses
- `load_*`, `build_*`, `evaluate_*`, and `format_*` functions
- status and error constants
- report fields
- root import compatibility until an audited migration exists
- `allow_robot_motion=False`
- `dry_run_only=True`
- `fake_publish_only=True`
- `plan_only=True`
- read-only UR5 semantics
- no ROS2 publish
- no MoveIt execution
- no RTDE write
- no Dashboard command
- no robot command
- bounded relative motion limit `0.50 m`
- real one-shot cap `0.05 m`
- scene version semantics
- TTL and stale-state blocking semantics
- snapshot ID and grounding ID matching semantics
- confidence, depth, camera information, and TF blocking semantics

Any future change that weakens these invariants is not a documentation cleanup
and needs a dedicated safety review.

## Do Not Move Yet

Do not move these in H13 cleanup without a compatibility plan, focused tests,
and safety review where appropriate:

- `src/cartesian_motion_gateway.py`
- `src/bounded_relative_motion.py`
- `src/moveit_plan_only_contract.py`
- `src/ur5_read_only_state_contract.py`
- `src/ros2_interface_readiness.py`
- `src/ros2_message_exporter.py`
- `src/robot_system_shadow_bridge.py`
- `src/geometry_validity.py`
- `src/planner_gateway_shadow.py`
- `src/projector/shadow.py`
- canonical launch scripts

The canonical startup commands remain documented in
`docs/current_entrypoints.md`. H13 work must not change those scripts, their
arguments, default behavior, path semantics, or operator expectations. Use
`bash -n` only unless a task explicitly permits startup.

## Future `src/contracts/` Policy

`src/contracts/` is suitable as a future canonical namespace for stable,
no-motion shared evidence contracts, schema contracts, and validation helpers.

It must not become a dumping ground for safety, execution, simulation runtime,
projector implementation, artifact generation, or real-path semantics. Mixed
responsibility files should remain where they are until a later audit can
isolate a stable contract surface.

Package-root re-exports should remain conservative unless a later task
explicitly justifies them. Keep `src/contracts/__init__.py` minimal by default.

Any future migration requires:

- compatibility and import plan
- focused tests for the exact surface moved
- clean scans for old and new import paths
- preserved root import compatibility during the migration window
- explicit proof that no-motion and fail-closed semantics are unchanged

Current root modules remain canonical production imports until a later audited
task creates compatibility adapters, migrates imports in stages, and proves
behavior is unchanged.

## Future `src/readiness/` Policy

`src/readiness/` is conceptually plausible because many files are readiness
gates rather than pure schemas. It is premature now.

Do not create `src/readiness/` until readiness files are clearly separable from
safety, execution, simulation runtime, artifact, and real-path behavior. If a
future audit cannot separate those responsibilities cleanly, the correct
implementation choice is no-op.

## Recommended H13-C

The safest H13-C is documentation-only compatibility/import planning if a
future contracts migration is needed.

Do not perform an adapter, helper extraction, import rewrite, or file move yet.
No-op is acceptable if responsibility remains mixed or risk remains high.

## Focused Checks

Documentation-only H13 work should finish with:

```bash
git diff --check
bash -n scripts/start_teto_real_full_stack.sh
bash -n scripts/start_teto_isaac_gui_operator.sh
```

If a future task touches code or tests, choose focused tests for the exact
surface changed. Do not run or start hardware, Isaac Sim, ROS, MoveIt, the UR
driver, RealSense, Qwen, a VLM, an LLM, or model services during contracts
cleanup unless a task explicitly authorizes that startup.
