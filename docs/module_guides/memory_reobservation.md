# Memory / Re-Observation Module Guide

This guide records the H17 memory and re-observation boundary policy for
future Codex, GPT, and human audits. H17-A completed a read-only audit. H17-B
is documentation-only: no implementation files are moved, no imports are
changed, no packages are created, no compatibility shims are added, and no
runtime behavior is changed.

H17-C adds the import compatibility plan in
`docs/h17_memory_reobservation_import_plan.md`. H17-C is also
documentation-only and does not approve package creation, shim creation, import
rewrites, file moves, or runtime API changes.

## Boundary Principles

The memory / re-observation boundary owns deterministic working-memory and
re-observation policy state. It records what the operator currently knows about
the task, scene monitor state, verified progress, remaining motion, re-observe
needs, replan needs, and abort needs.

This boundary does not own execution authority. Execution, simulation, safety,
planning, grounding, vision, contracts, and replay may consume its evidence,
but they must not treat it as permission to move a real robot. Fail-closed
execution gates and measured verification stay near the execution and safety
boundaries.

Memory / re-observation policy must not start or import hardware, Isaac Sim,
ROS, MoveIt, RealSense, Qwen, VLM, LLM, or model services. It should stay
offline-testable and deterministic.

## Current Files

Current implementation files:

- `src/memory_guided_execution.py`: MEMORY_CORE, REOBSERVATION_POLICY, and
  SHARED_BUT_SAFE. It owns the working-memory record, scene monitor result
  normalization, event-triggered re-observation decisions, and update logic for
  verified progress and remaining delta.
- `src/adaptive_reobservation_policy.py`: REOBSERVATION_POLICY,
  EXECUTION_ADJACENT, and SHARED_BUT_SAFE. It owns adaptive execution-phase
  policy, lightweight monitoring mode, model-call suppression policy, recovery
  re-observation decisions, replan decisions, and abort handoff fields.

Current consumers:

- `src/isaac_sim_operator.py`: SIM_ONLY production consumer. It imports both
  current root modules and writes their results into Isaac operator evidence.
- `scripts/teto_isaac_operator_console.py`: CLI and launch-adjacent consumer
  indirectly through `src.isaac_sim_operator`.
- `tests/test_memory_guided_execution.py`: focused unit coverage for working
  memory and event-triggered re-observation policy.
- `tests/test_adaptive_reobservation_policy.py`: focused unit coverage for
  adaptive re-observation behavior.
- `tests/test_isaac_sim_operator.py`: integration coverage for Isaac operator
  evidence fields, working-memory updates, and stale-scene re-observation.

H17-A found no direct REAL_PATH runtime import of these modules. The real
backend currently has only a future-hook string reference; future code that
turns that into a runtime dependency must be audited carefully.

## Import And Packaging Policy

There is currently no `src/memory/` package and no `src/reobservation/`
package. H17-B does not create either package.

The current root modules are implementations, not compatibility shims:

- `src.memory_guided_execution`
- `src.adaptive_reobservation_policy`

Do not add package-root re-exports, compatibility shims, import rewrites, or
new runtime API surfaces during H17-B. Future package names are not decided
yet; do not treat `src/memory/` or `src/reobservation/` as mandatory future
targets.

Any future migration requires a dedicated compatibility/import plan first,
including import inventory, staged import policy, focused tests, clean scans,
and explicit protection for canonical Real and Isaac startup behavior.
For the current plan and no-go conditions, read
`docs/h17_memory_reobservation_import_plan.md`.

## Relationship To Neighbor Boundaries

TETO contract and validation surfaces may consume memory and re-observation
fields as evidence, but they do not own the policy decisions.

Replay and failure analysis may preserve or inspect working-memory,
scene-monitor, re-observation, and adaptive-policy fields. Replay must preserve
evidence compatibility and must not reinterpret SIM_ONLY or no-motion evidence
as REAL_PATH success.

Execution and safety consume the fail-closed meaning of policy outputs such as
`continue_allowed`, `reobserve_required`, `replan_required`, `abort_required`,
`reobserve_reason`, and `execution_load_mode`. They retain authority over real
or simulated motion gates.

Simulation currently consumes this boundary through `src/isaac_sim_operator.py`.
That consumption is SIM_ONLY and evidence-sensitive. The Isaac operator records
working-memory and adaptive-policy outputs in run artifacts, so moving these
modules without compatibility can break the canonical Isaac path or evidence
schema even if the policy logic itself is unchanged.

Vision and grounding own visual snapshots, camera-source declarations, target
labels, bbox/pixel-center evidence, confidence, and semantic rejection. Memory
/ re-observation may refer to scene monitor status, target visibility, or
snapshot freshness as inputs, but it does not own live capture, grounding, or
metric projection.

## What Belongs Here

- Working-memory version and state shape.
- Scene monitor result normalization used by re-observation policy.
- Event-triggered re-observation reasons and warning/abort decisions.
- Adaptive re-observation phases and execution-load policy.
- Deterministic policy outputs that say whether to continue, re-observe,
  replan, or abort.
- Evidence fields that describe model-call suppression and re-observation
  decisions without actually calling models.

## What Does Not Belong Here

- Real robot execution, MoveIt action clients, Dashboard, RTDE, URScript, or UR
  driver behavior.
- Isaac runtime startup, `SimulationApp`, USD asset loading, or measured Isaac
  bridge behavior.
- Live camera capture, RealSense access, visual snapshot construction, or
  camera-source selection.
- Qwen, VLM, LLM, or model runtime calls.
- Grounding target selection, confidence gates, bbox/pixel-center evidence, or
  semantic rejection.
- Planning contracts, Cartesian target generation, bounded motion envelopes, or
  manual confirmation gates.
- Replay artifact writing, evidence export formatting, or operator launch
  scripts.

## Risk Notes

The SIM_ONLY Isaac path is the current production consumer. Its canonical
launcher reaches these modules through `scripts/teto_isaac_operator_console.py`
and `src.isaac_sim_operator`. Import movement can therefore break canonical
Isaac startup or evidence generation.

REAL_PATH has no direct runtime dependency from H17-A, but future hooks must be
treated as real-path sensitive. A future real backend dependency on
re-observation policy would need a safety and execution review, not a simple
module cleanup.

Root imports are part of the current compatibility surface. A future migration
should preserve root import compatibility until all production, script, test,
documentation, and config consumers are intentionally migrated and verified.

## Do Not Move Yet

Do not move these in H17 work without a compatibility/import plan, focused
tests, and explicit review:

- `src/memory_guided_execution.py`
- `src/adaptive_reobservation_policy.py`
- `src/isaac_sim_operator.py`
- `scripts/teto_isaac_operator_console.py`
- canonical launch scripts
- REAL_PATH execution files

## Recommended H17-C

The safest H17-C is a documentation-only memory / re-observation import
compatibility plan if migration is still desired.

Do not perform a package creation, adapter extraction, import rewrite, shim
creation, file move, or low-risk migration yet. The boundary is small, but its
current consumer is canonical SIM_ONLY operator behavior and evidence.

## Focused Checks

Documentation-only H17 work should finish with:

```bash
git diff --check
```

For future implementation work, use focused scans and tests for the exact
surface touched:

```bash
rg -n "from src\.memory_guided_execution|import src\.memory_guided_execution|from src\.adaptive_reobservation_policy|import src\.adaptive_reobservation_policy" src scripts tests docs configs -S
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONPYCACHEPREFIX=/tmp/teto_codex_pycache .venv_lab/bin/python -m pytest -p no:cacheprovider -q \
  tests/test_memory_guided_execution.py \
  tests/test_adaptive_reobservation_policy.py \
  tests/test_isaac_sim_operator.py
```

Startup script syntax checks only:

```bash
bash -n scripts/start_teto_real_full_stack.sh
bash -n scripts/start_teto_isaac_gui_operator.sh
```

Do not start hardware, Isaac Sim, ROS, MoveIt, the UR driver, RealSense, Qwen,
a VLM, an LLM, or model services during memory / re-observation cleanup unless
a task explicitly authorizes that startup.
