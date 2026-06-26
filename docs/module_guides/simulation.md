# Simulation And Isaac Module Guide

This guide records the H16 simulation and Isaac boundary policy for future
Codex, GPT, and human audits. H16-A completed a read-only simulation / Isaac
boundary audit. H16-B is documentation-only: no implementation files are moved,
no imports are changed, no runtime APIs are created, no adapters or helpers are
created, and no launch behavior is modified.

H16-B also does not create `src/simulation/` or `src/isaac/`. Those namespaces
may be useful later, but the current root import paths remain the canonical
production imports until a future audited compatibility plan proves otherwise.

## Boundary Principles

Isaac remains SIM_ONLY. SIM_ONLY code must not import or trigger REAL_PATH
execution authority. REAL_PATH code must not depend on Isaac-only operator
state.

Isaac runtime imports must remain guarded and runtime-safe. In particular,
Isaac API imports belong inside the Isaac startup/runtime path, after
`SimulationApp` exists, rather than at broad package import time.

USD asset paths, generated UR5e assets, Isaac app paths, and run artifacts are
artifact-sensitive. Moving code or wrapping imports too early can break demo
workflows even when behavior appears unchanged.

Replay and artifact helpers must preserve evidence compatibility. Dry-run,
no-Isaac, Isaac, synthetic, and simulation-only evidence must not become
REAL_PATH success evidence.

Package-root re-exports must not hide SIM_ONLY versus REAL_PATH boundaries.
Shared real/sim orchestration should not be treated as simulation-owned.

## H16-A Classification

SIM_ONLY operator:

- `src/isaac_sim_operator.py`

Isaac-only measured bridge:

- `src/isaac_sim_bridge.py`

Broad artifact/runtime hub:

- `src/simulation_runtime.py`

SIM_ONLY, replay, and artifact helpers:

- `src/simulation_micro_motion.py`
- `src/simulation_motion_precheck.py`
- `src/semantic_simulation_bridge.py`
- `src/simulated_task_execution.py`

Simulation bridge contract:

- `src/simulation_bridge_contract.py`

SHARED_BUT_SAFE real/sim orchestration:

- `src/unified_segmented_operator.py`

Memory and re-observation policy logic:

- `src/adaptive_reobservation_policy.py`
- `src/memory_guided_execution.py`

CLI and launch-adjacent Isaac surfaces:

- `scripts/teto_isaac_operator_console.py`
- `scripts/start_teto_isaac_gui_operator.sh`

USD and artifact tool:

- `scripts/import_ur5e_urdf_to_isaac_usd.py`

Config surface:

- `configs/isaac_sim_operator.example.yaml`

Some responsibilities are intentionally mixed today. Treat actual
responsibility as authoritative, not the filename.

## Import And Dependency Boundary

Current import relationships are part of the compatibility surface:

- `scripts/teto_isaac_operator_console.py` imports `src.isaac_sim_operator`.
- `scripts/teto_isaac_operator_console.py` imports `src.isaac_sim_bridge` only
  inside the Isaac startup path.
- `src.isaac_sim_bridge` imports only `GATEWAY_SIMULATED_MEASURED` from
  `src.isaac_sim_operator`.
- `src.isaac_sim_operator` imports `src.unified_segmented_operator`,
  `src.memory_guided_execution`, and `src.adaptive_reobservation_policy`.
- `src.simulation_runtime` imports many readiness, shadow, artifact, and
  simulation helper modules.
- `src.evidence_exporter` consumes simulation micro-motion, semantic bridge,
  and simulated task execution formatters.
- Tests and harnesses depend on current root import paths.

Do not rewrite these imports in H16-B. Any future import change needs a
compatibility/import plan, focused tests, and clean scans.

## Sensitivity Classification

REAL_PATH files and surfaces:

- real segmented backend
- MoveIt executor
- Cartesian gateway
- real launch stack

These are not H16 migration targets.

SIM_ONLY files and surfaces:

- Isaac operator
- Isaac bridge
- Isaac console and startup script
- simulation micro-motion
- simulation motion precheck

SHARED_BUT_SAFE files and surfaces:

- unified segmented operator
- bounded relative motion semantics

Replay, artifact, and contracts surfaces:

- simulation runtime
- semantic simulation bridge
- simulated task execution
- simulation bridge contract
- evidence exporter

Memory and re-observation surfaces:

- memory-guided execution
- adaptive re-observation policy

CLI and launch-adjacent surfaces:

- Isaac operator console
- Isaac GUI launcher
- URDF-to-USD importer

If a future task crosses more than one sensitivity class, split the work or
stop for another boundary audit.

## Public APIs And Invariants

These public APIs and evidence semantics must not break:

- root imports until an audited migration exists
- public dataclasses, constants, and functions
- evidence schemas
- artifact filenames and paths
- `real_robot_motion_executed=False` for simulation evidence
- no Dashboard, RTDE, MoveIt, or URScript in SIM_ONLY paths
- `SimulationApp` and Isaac imports only in runtime-safe paths
- no package-root re-export hiding REAL_PATH versus SIM_ONLY
- SIM_ONLY code must not accidentally become a real execution path
- REAL_PATH code must not depend on Isaac-only operator state

Any future change that weakens these invariants is not a documentation cleanup
and needs a dedicated safety and execution review.

## Do Not Move Yet

Do not move these in H16 cleanup without a compatibility plan, focused tests,
and explicit review:

- `src/isaac_sim_operator.py`
- `src/isaac_sim_bridge.py`
- `src/simulation_runtime.py`
- `src/unified_segmented_operator.py`
- `src/adaptive_reobservation_policy.py`
- `src/memory_guided_execution.py`
- canonical launch scripts
- Isaac console
- USD importer
- artifact and replay helpers
- REAL_PATH execution files

The canonical startup commands remain documented in
`docs/current_entrypoints.md`. H16 work must not change those scripts, their
arguments, default behavior, path semantics, or operator expectations. Use
`bash -n` only unless a task explicitly permits startup.

## Future Namespace Policy

`src/isaac/` may be appropriate later for Isaac-specific bridge and operator
internals.

`src/simulation/` may be appropriate later for pure simulation artifact and
contract helpers.

Neither namespace should be created in H16-B. Future namespaces must remain
conservative and avoid package-root re-exports unless a later audited task
explicitly justifies them.

Any future migration requires:

- compatibility and import plan
- focused tests for the exact surface moved
- clean scans for old and new import paths
- preserved root import compatibility during the migration window
- proof that SIM_ONLY, replay, artifact, and REAL_PATH evidence semantics are
  unchanged

## Potential Future Safe Work

Lowest-risk future candidates:

- documentation-only simulation / Isaac import inventory and compatibility plan
- possible tiny safety/config validation helper extraction only after focused
  tests
- possible Isaac bridge namespace preparation only after import timing and
  USD/runtime safety planning
- no-op if responsibility remains mixed or risk remains high

Implementation moves are not recommended yet.

## Recommended H16-C

The safest H16-C is a documentation-only simulation / Isaac compatibility and
import plan if migration is still desired.

Do not perform an adapter, helper extraction, import rewrite, package
creation, or file move yet. No-op is acceptable if responsibility remains
mixed or risk remains high.

## Focused Checks

Documentation-only H16 work should finish with:

```bash
git diff --check
bash -n scripts/start_teto_real_full_stack.sh
bash -n scripts/start_teto_isaac_gui_operator.sh
```

If a future task touches code or tests, choose focused tests for the exact
surface changed. Do not run or start hardware, Isaac Sim, ROS, MoveIt, the UR
driver, RealSense, Qwen, a VLM, an LLM, or model services during simulation
cleanup unless a task explicitly authorizes that startup.
