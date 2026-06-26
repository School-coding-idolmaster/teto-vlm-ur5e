# Safety Module Guide

This guide records the H14 safety boundary policy for future Codex, GPT, and
human audits. H14-A completed a read-only safety boundary audit. H14-B is
documentation-only: no implementation files are moved, no imports are changed,
no runtime APIs are created, no broad `src/safety/` expansion is made, and no
launch behavior is modified.

## Boundary Principles

Safety code must preserve fail-closed behavior. A missing, stale, ambiguous,
synthetic, or unsafe input should block rather than silently grant progress
toward real robot execution.

Parser, planner, contract, readiness, replay, and shadow output must not grant
robot execution permission. They may produce intent, evidence, preview
contracts, no-motion declarations, or downstream inputs, but execution
permission remains gated near measured state and execution.

Safety gates retain veto power. Dry-run, plan-only, fake-publish, read-only,
shadow, replay, and Isaac evidence must remain explicit no-motion or SIM_ONLY
evidence and must not be counted as REAL_PATH success.

`src/safety/` should remain narrow until pure safety helpers are identified and
tested. It is currently a future boundary marker, not the active runtime home
for real execution safety authority.

## H14-A Classification

True safety envelope:

- `src/bounded_relative_motion.py`

Mixed planning, validation, safety, and execution gateway:

- `src/cartesian_motion_gateway.py`

Execution guard:

- `src/guarded_vector_motion_executor.py`

Human confirmation gate:

- `src/manual_confirmation_gate.py`

Real MoveIt plan/execute boundary:

- `src/moveit_pose_executor.py`

Measured real backend safety authority:

- `src/real_segmented_operator_backend.py`

Shared real/sim segmented orchestration:

- `src/unified_segmented_operator.py`

SIM_ONLY safety isolation:

- `src/isaac_sim_operator.py`

Readiness/no-motion contracts:

- `src/moveit_plan_only_contract.py`
- `src/ur5_read_only_state_contract.py`
- `src/ros2_interface_readiness.py`
- `src/robot_system_shadow_bridge.py`
- `src/lab_readiness.py`

Planner/replay shadow guard:

- `src/planner_gateway_shadow.py`

Some responsibilities are intentionally mixed today. In particular,
`src/cartesian_motion_gateway.py` spans planner-facing validation, safety
limits, target generation, manual-confirmation handoff, and execution
authorization. Treat actual responsibility as authoritative, not the filename.

## Sensitivity Classification

REAL_PATH files:

- `src/real_segmented_operator_backend.py`
- `src/moveit_pose_executor.py`
- `src/cartesian_motion_gateway.py`
- `src/manual_confirmation_gate.py`

SIM_ONLY files:

- `src/isaac_sim_operator.py`

SHARED_BUT_SAFE files and surfaces:

- `src/bounded_relative_motion.py`
- `src/unified_segmented_operator.py`
- parser/normalizer handoff safety flags

Replay, artifact, and CLI-sensitive surfaces:

- readiness reports
- evidence exporter consumers
- safety harnesses
- operator consoles

Execution-sensitive surfaces:

- Cartesian gateway
- MoveIt executor
- real backend
- guarded vector executor

If a future task crosses more than one sensitivity class, split the work or
stop for another audit. Safety envelope changes can silently alter real robot
behavior even when the change looks like a small refactor.

## Public APIs And Invariants

These public APIs and evidence semantics must not break:

- contract and version strings
- public dataclasses
- `evaluate_*`, `build_*`, `load_*`, and `format_*` functions
- status and error constants
- root import compatibility until an audited migration exists
- `SHARED_MAX_RELATIVE_MOTION_DISTANCE_M = 0.50`
- `REAL_ONE_SHOT_CAP_M = 0.05`
- `E_RELATIVE_MOTION_RANGE_EXCEEDED`
- `safety_gate_still_required=True`
- parser output never grants execution permission
- dry-run evidence remains no-motion
- plan-only evidence remains no-motion
- fake-publish evidence remains no-motion
- read-only evidence remains no-motion
- Dashboard, RTDE, MoveIt, and UR driver actions remain gated

Real execution evidence must continue to mean:

- real backend involvement
- attempted MoveIt execution
- measured verification semantics

Any future change that weakens these invariants is not a safety documentation
cleanup and needs a dedicated real-path safety review.

## Do Not Move Yet

Do not move these in H14 cleanup without a compatibility plan, focused tests,
and explicit safety review:

- `src/cartesian_motion_gateway.py`
- `src/bounded_relative_motion.py`
- `src/moveit_pose_executor.py`
- `src/real_segmented_operator_backend.py`
- `src/unified_segmented_operator.py`
- `src/guarded_vector_motion_executor.py`
- `scripts/teto_operator_console.py`
- safety harnesses
- canonical launch scripts

The canonical startup commands remain documented in
`docs/current_entrypoints.md`. H14 work must not change those scripts, their
arguments, default behavior, path semantics, or operator expectations. Use
`bash -n` only unless a task explicitly permits startup.

## Future `src/safety/` Policy

`src/safety/` is appropriate as a future narrow namespace for:

- shared safety policy notes
- pure gate helpers
- small evidence semantics that can be audited independently

It is not appropriate as a broad home for:

- execution backends
- MoveIt action clients
- real operator orchestration
- readiness contracts
- mixed gateways

Keep `src/safety/__init__.py` conservative. Do not add package-root re-exports
unless a later task explicitly justifies them.

Any future migration requires:

- compatibility and import plan
- focused tests for the exact safety surface moved
- clean scans for old and new import paths
- preserved root import compatibility during the migration window
- proof that fail-closed behavior and real-path evidence semantics are
  unchanged

## Files Better Left Elsewhere

Readiness contracts should remain under contracts/readiness policy for now.
They preserve no-motion boundaries, but they are not live execution safety
gates.

The real backend and MoveIt execution code belong closer to execution because
they own measured runtime state, ROS/MoveIt action client behavior, and real
execution evidence.

`src/cartesian_motion_gateway.py` should stay at the root until a dedicated
split plan exists. It is too mixed to be absorbed wholesale by `src/safety/`,
`src/planning/`, or `src/execution/`.

`src/bounded_relative_motion.py` could eventually be a contracts or safety
candidate, but only with a safety-envelope compatibility plan. Its shared
limits are depended on by parser, planner, operator, real, sim, tests, and
documentation surfaces.

## Recommended H14-C

The safest H14-C is a documentation-only safety compatibility/import plan if
future safety migration is still desired.

Do not perform an adapter, helper extraction, import rewrite, or file move yet.
No-op is acceptable if responsibility remains mixed or risk remains high.

## Focused Checks

Documentation-only H14 work should finish with:

```bash
git diff --check
bash -n scripts/start_teto_real_full_stack.sh
bash -n scripts/start_teto_isaac_gui_operator.sh
```

If a future task touches code or tests, choose focused tests for the exact
surface changed. Do not run or start hardware, Isaac Sim, ROS, MoveIt, the UR
driver, RealSense, Qwen, a VLM, an LLM, or model services during safety
cleanup unless a task explicitly authorizes that startup.
