# Execution Module Guide

This guide records the H15 execution and operator boundary policy for future
Codex, GPT, and human audits. H15-A completed a read-only execution/operator
boundary audit. H15-B is documentation-only. H15-C adds the compatibility and
import plan in `docs/h15_execution_import_plan.md`. H15 still does not move
implementation files, change imports, create runtime APIs, expand
`src/execution/` broadly, add package-root re-exports, or modify launch
behavior.

## Boundary Principles

Execution authority must stay close to measured state and verification. A
parser, planner, contract, readiness check, replay record, or shadow result may
produce evidence for downstream review, but it must not grant robot execution
permission.

Manual confirmation remains an execution gate, not a UI-only helper. Dry-run,
plan-only, fake-publish, read-only, replay, and Isaac evidence must remain
fail-closed no-motion or SIM_ONLY evidence and must not be counted as REAL_PATH
success.

Real-path execution evidence must continue to mean real backend involvement,
attempted MoveIt execution, and measured verification semantics. Moving code
too early can hide where authority lives, especially when a small helper is
actually deciding whether motion is allowed.

`src/execution/` should remain narrow until implementation boundaries are
proven by import scans, focused tests, and compatibility planning.

## H15-A Classification

REAL_PATH measured backend authority:

- `src/real_segmented_operator_backend.py`

REAL_PATH MoveIt plan/execute wrapper and action evidence boundary:

- `src/moveit_pose_executor.py`

SHARED_BUT_SAFE real/sim orchestration and measured per-segment verification:

- `src/unified_segmented_operator.py`

HIGH-risk mixed planning, validation, safety, and execution gateway:

- `src/cartesian_motion_gateway.py`

Execution guards:

- `src/manual_confirmation_gate.py`
- `src/guarded_vector_motion_executor.py`

SIM_ONLY execution and measured simulation bridge:

- `src/isaac_sim_operator.py`
- `src/isaac_sim_bridge.py`

SIM_ONLY, replay, and artifact helpers:

- `src/simulation_micro_motion.py`
- `src/simulated_task_execution.py`

Legacy or special real execution surface:

- `src/real_ur5_hover_executor.py`

CLI and launch-adjacent real operator entrypoint:

- `scripts/teto_operator_console.py`

Protected canonical launch surfaces:

- `scripts/start_teto_real_full_stack.sh`
- `scripts/start_teto_isaac_gui_operator.sh`

Some responsibilities are intentionally mixed today. In particular,
`src/cartesian_motion_gateway.py` spans planner-facing validation, safety
limits, target generation, manual-confirmation handoff, and execution
authorization. Treat actual responsibility as authoritative, not the filename.

## Import And Dependency Boundary

Current import relationships are part of the compatibility surface:

- `scripts/teto_operator_console.py` imports the real backend plus the unified
  segmented operator.
- `src/real_segmented_operator_backend.py` imports Cartesian gateway execution.
- `src/cartesian_motion_gateway.py` imports manual confirmation, the MoveIt
  executor, command-to-task adaptation, and bounded relative motion.
- `src/isaac_sim_operator.py` imports unified command semantics plus memory and
  re-observation helpers.
- `scripts/teto_isaac_operator_console.py` imports the Isaac operator and
  creates the Isaac bridge only inside the Isaac startup path.
- Tests depend on current root import paths.
- `src/execution/` currently has no active broad runtime authority and should
  stay conservative.

Do not add package-root re-exports to hide these dependencies. If a later task
creates adapters or compatibility shims, it needs a compatibility plan and
focused tests first.

## Sensitivity Classification

REAL_PATH files and surfaces:

- `src/real_segmented_operator_backend.py`
- `src/moveit_pose_executor.py`
- `src/cartesian_motion_gateway.py`
- `src/manual_confirmation_gate.py`
- real operator scripts and consoles

SIM_ONLY files and surfaces:

- `src/isaac_sim_operator.py`
- `src/isaac_sim_bridge.py`
- simulation helpers

SHARED_BUT_SAFE files and surfaces:

- `src/unified_segmented_operator.py`
- bounded motion handoff semantics

Replay, artifact, and CLI-sensitive surfaces:

- simulated task execution
- execution evidence formatters and exporters
- operator consoles
- safety harnesses

Execution-sensitive surfaces:

- Cartesian gateway
- MoveIt executor
- real backend
- guarded vector executor
- manual confirmation gate

If a future task crosses more than one sensitivity class, split the work or
stop for another audit.

## Public APIs And Invariants

These public APIs and evidence semantics must not break:

- root imports until an audited migration exists
- public dataclasses
- `evaluate_*`, `build_*`, `load_*`, and `format_*` functions
- status and error constants
- `safety_gate_still_required=True`
- parser and planner outputs never grant execution permission
- dry-run evidence remains no-motion
- plan-only evidence remains no-motion
- fake-publish evidence remains no-motion
- read-only evidence remains no-motion
- `SHARED_MAX_RELATIVE_MOTION_DISTANCE_M = 0.50`
- `REAL_ONE_SHOT_CAP_M = 0.05`
- Dashboard, RTDE, MoveIt, and UR driver actions remain gated
- D455 freshness checks remain meaningful
- measured post-motion verification evidence remains tied to actual execution
  attempts and measured state

Any future change that weakens these invariants is not a documentation cleanup
and needs a dedicated real-path execution review.

## Do Not Move Yet

Do not move these without a compatibility plan, focused tests, and explicit
execution/safety review:

- `src/cartesian_motion_gateway.py`
- `src/bounded_relative_motion.py`
- `src/moveit_pose_executor.py`
- `src/real_segmented_operator_backend.py`
- `src/unified_segmented_operator.py`
- `src/guarded_vector_motion_executor.py`
- `src/manual_confirmation_gate.py`
- `scripts/teto_operator_console.py`
- safety harnesses
- canonical launch scripts
- readiness/contracts files

The canonical startup commands remain documented in
`docs/current_entrypoints.md`. H15 work must not change those scripts, their
arguments, default behavior, path semantics, or operator expectations. Use
`bash -n` only unless a task explicitly permits startup.

## Future `src/execution/` Policy

`src/execution/` is appropriate later as a narrow canonical namespace for:

- execution adapters
- backend interfaces
- measured execution evidence helpers
- small pure execution abstractions that do not hide authority boundaries

It is not appropriate as a broad home for:

- mixed Cartesian gateway behavior
- bounded motion safety envelopes
- parser or planner contracts
- safety or readiness contracts
- launch scripts
- real operator consoles

Keep `src/execution/__init__.py` conservative. Avoid package-root re-exports
unless a later audited task explicitly justifies them.

For staged import migration rules, required scans, and shim policy, read
`docs/h15_execution_import_plan.md`.

Any future migration requires:

- compatibility and import plan
- focused tests for the exact surface moved
- clean scans for old and new import paths
- preserved root import compatibility during the migration window
- proof that fail-closed behavior, real-path evidence semantics, and measured
  verification are unchanged

## Potential Future Safe Work

Lowest-risk future candidates:

- documentation-only compatibility/import plan for execution imports
- possible tiny `UnifiedOperatorBackend` protocol extraction only after import
  scans and focused real/unified/Isaac tests
- adapter or shim preparation only after a plan
- no-op if responsibility remains mixed or risk remains high

Implementation moves are not recommended yet.

## Recommended H15-D

The safest H15-D is no-op or documentation-only compatibility refinement if
migration is still desired.

Do not perform an adapter, helper extraction, import rewrite, or file move yet.
No-op is acceptable if responsibility remains mixed or risk remains high.

## Focused Checks

Documentation-only H15 work should finish with:

```bash
git diff --check
bash -n scripts/start_teto_real_full_stack.sh
bash -n scripts/start_teto_isaac_gui_operator.sh
```

If a future task touches code or tests, choose focused tests for the exact
surface changed. Do not run or start hardware, Isaac Sim, ROS, MoveIt, the UR
driver, RealSense, Qwen, a VLM, an LLM, or model services during execution
cleanup unless a task explicitly authorizes that startup.
