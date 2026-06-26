# H16 Simulation / Isaac Import Compatibility Plan

H16-C is documentation-only. It considers possible future simulation and Isaac
namespaces, but it does not create packages, move implementation files, change
imports, create adapters, extract helpers, create runtime APIs, add
package-root re-exports, or modify launch behavior.

H16-A and H16-B established that the simulation / Isaac surface is mixed across
SIM_ONLY operator behavior, guarded Isaac runtime imports, USD and generated
asset paths, broad simulation runtime artifacts, replay evidence, shared
real/sim orchestration, and memory/re-observation policy logic. Direct
implementation movement remains premature.

## Migration Being Considered

Possible future namespace targets:

- `src/isaac/` for Isaac-specific bridge and operator internals.
- `src/simulation/` for pure simulation artifact and contract helpers.

H16-C does not approve:

- package creation
- implementation moves
- import migration
- production import rewrites
- runtime API creation
- adapter or shim creation
- helper extraction
- package-root re-exports
- launch script changes

Current root modules remain canonical production imports until a later audited
task creates compatibility adapters, migrates imports in stages, and proves
behavior is unchanged.

## Candidate Future Modules

Potential candidates for future consideration only:

- Isaac-specific internals from `src/isaac_sim_bridge.py`, but only after an
  import-timing and USD/runtime safety plan proves Isaac API availability stays
  guarded behind the Isaac startup path.
- A narrow SIM_ONLY helper from `src/simulation_motion_precheck.py`, or pure
  config validation logic, only if it is independently tested and does not
  alter safety or evidence semantics.
- A pure artifact formatter helper from simulation evidence paths, only if it
  does not alter evidence schemas, filenames, paths, replay fields, or report
  compatibility.
- Marker-only planning for future `src/isaac/` or `src/simulation/`
  namespaces, with no runtime imports.

These candidates are future-only. They do not authorize a direct move of the
active Isaac operator or the broad simulation runtime hub.

## Do Not Move In The First Implementation Pass

These files and surfaces must not be moved in the first implementation pass:

- `src/isaac_sim_operator.py`: SIM_ONLY operator with shared command semantics,
  memory/re-observation integration, Qwen health/parser surface, and evidence
  writing.
- `src/isaac_sim_bridge.py`: Isaac-only measured bridge with guarded runtime
  imports, USD/articulation behavior, and Isaac API dependency.
- `src/simulation_runtime.py`: broad artifact/runtime hub that imports many
  readiness, shadow, artifact, and simulation helper modules.
- `src/unified_segmented_operator.py`: SHARED_BUT_SAFE real/sim orchestration,
  not simulation-owned.
- `src/adaptive_reobservation_policy.py`: shared memory/re-observation policy
  logic, not Isaac-only.
- `src/memory_guided_execution.py`: shared working-memory and event-triggered
  re-observation logic, not Isaac-only.
- `scripts/teto_isaac_operator_console.py`: launch-adjacent Isaac operator
  console.
- `scripts/start_teto_isaac_gui_operator.sh`: canonical Isaac SIM_ONLY launch
  script.
- `scripts/import_ur5e_urdf_to_isaac_usd.py`: USD / generated-asset tool that
  can start Isaac when run.
- `configs/isaac_sim_operator.example.yaml`: config and local path surface.
- artifact and replay helpers.
- REAL_PATH execution files.
- canonical launch scripts.

If a later task cannot isolate a tiny pure helper without touching these
surfaces, the correct implementation choice is no-op.

## Compatibility Strategy Options

Acceptable strategy options, from safest to riskiest:

- Documentation only: record boundaries and defer implementation.
- No-op: leave current imports and files in place if responsibility remains
  mixed or compatibility risk remains high.
- Marker-only plan: refine future namespace documentation without creating
  runtime imports or package-root re-exports.
- Root shim compatibility adapter: only after a future implementation plan,
  import scans, and focused tests.
- New canonical module plus old import shim: only after import scans and
  focused tests prove behavior and evidence compatibility.
- Staged import migration: migrate tests/docs first, then scripts and offline
  consumers, then production consumers only after focused checks.
- Deletion only after all production, tests, docs, CLI, launch-adjacent, replay,
  artifact, and config consumers are migrated and ripgrep scans are clean.

Compatibility rules:

- If `src/isaac/` is ever created, keep `src/isaac/__init__.py` conservative.
- If `src/simulation/` is ever created, keep `src/simulation/__init__.py`
  conservative.
- Do not add package-root re-exports unless a later task explicitly justifies
  them.
- Preserve current public symbols and evidence fields.
- Preserve root import compatibility during any migration window.
- Delete old shims only in a dedicated task after clean scans, focused tests,
  artifact compatibility checks, and human approval.

## Required Invariants For Any Future Move

Any future move must preserve:

- root imports until an audited migration exists
- SIM_ONLY code must not import or trigger REAL_PATH execution authority
- REAL_PATH code must not depend on Isaac-only operator state
- Isaac runtime imports must remain guarded and runtime-safe
- `SimulationApp` and Isaac imports only in runtime-safe paths
- USD asset paths and generated assets remain compatible
- artifact filenames and evidence schemas remain compatible
- `real_robot_motion_executed=False` remains true for simulation evidence
- no Dashboard, RTDE, MoveIt, or URScript in SIM_ONLY paths
- package-root re-exports must not hide REAL_PATH versus SIM_ONLY boundaries
- memory/re-observation policy files remain shared or paper-core unless
  explicitly reclassified by a later audit
- unified real/sim orchestration remains shared, not simulation-owned

These invariants are safety and evidence semantics, not formatting details.
Breaking them would require a dedicated safety and execution task, not a
simulation namespace cleanup.

## Proposed Future H16 Stages

- H16-D: no-op or documentation-only compatibility refinement if risk remains
  high.
- H16-E: optional namespace preparation design, not implementation.
- H16-F: if approved later, migrate only a tiny pure SIM_ONLY helper with old
  root import compatibility.
- H16-G: migrate tests/docs first, then production imports only after focused
  tests.
- H16-H: delete old shims only after clean scans, focused tests, artifact
  compatibility checks, and human approval.

If compatibility risk remains high at any stage, choose no-op and leave
current imports in place.

## Required Future Scans And Checks

Before any future implementation move, run direct import scans such as:

```bash
rg -n "from src\.isaac_sim_operator|import src\.isaac_sim_operator|from src\.isaac_sim_bridge|import src\.isaac_sim_bridge" src tests docs scripts configs -S
rg -n "from src\.simulation_runtime|import src\.simulation_runtime|from src\.simulation_micro_motion|import src\.simulation_micro_motion" src tests docs scripts configs -S
rg -n "from src\.simulation_motion_precheck|import src\.simulation_motion_precheck|from src\.semantic_simulation_bridge|import src\.semantic_simulation_bridge|from src\.simulated_task_execution|import src\.simulated_task_execution" src tests docs scripts configs -S
rg -n "from src\.isaac|import src\.isaac|from src\.simulation|import src\.simulation" src tests docs scripts configs -S
```

Also scan protected and artifact-sensitive surfaces:

```bash
rg -n "start_teto_real_full_stack|start_teto_isaac_gui_operator|teto_isaac_operator_console|teto_operator_console" src tests docs scripts configs -S
rg -n "generated_ur5e|ur5e_clean_no_tool|ur5e_asset|ur5e_urdf|USD|usd|SimulationApp|isaacsim|omni|pxr" src tests docs scripts configs -S
rg -n "simulation_execution_result|simulation_motion_result|simulation_motion_report|semantic_simulation_bridge|simulated_task_execution|evidence_schema|schema_version|real_robot_motion_executed" src tests docs scripts configs -S
```

Focused tests for future implementation work should include the exact touched
surface and likely consumers:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONPYCACHEPREFIX=/tmp/teto_codex_pycache .venv_lab/bin/python -m pytest -p no:cacheprovider -q \
  tests/test_isaac_sim_operator.py \
  tests/test_simulation_runtime.py \
  tests/test_simulation_micro_motion.py \
  tests/test_simulation_motion_precheck.py \
  tests/test_semantic_simulation_bridge.py \
  tests/test_simulated_task_execution.py
```

Add shared-policy tests if the touched surface crosses those boundaries:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONPYCACHEPREFIX=/tmp/teto_codex_pycache .venv_lab/bin/python -m pytest -p no:cacheprovider -q \
  tests/test_unified_operator_interface.py \
  tests/test_memory_guided_execution.py \
  tests/test_adaptive_reobservation_policy.py
```

Always run:

```bash
git diff --check
bash -n scripts/start_teto_real_full_stack.sh
bash -n scripts/start_teto_isaac_gui_operator.sh
```

Launch script syntax checks are allowed; starting launch scripts is not.

## Non-Goals

H16 simulation modularization is not permission for:

- real/sim behavior changes
- SIM_ONLY / REAL_PATH boundary changes
- Isaac runtime startup
- USD asset regeneration
- evidence schema or path changes
- memory/re-observation policy movement
- unified operator movement
- execution authority relocation
- launch script modification
- operator console behavior changes
- model service startup
- hardware, simulation, ROS, MoveIt, UR driver, RealSense, Qwen, VLM, LLM, or
  model-service startup

## Potential Future Safe Work

- Documentation-only compatibility/import plan: safe now.
- Tiny SIM_ONLY config or precheck helper extraction: possible later, but
  requires xhigh review, import scans, old root compatibility, and focused
  tests.
- Isaac bridge namespace preparation: possible later, but high risk because of
  import timing, USD/runtime behavior, generated assets, and Isaac API
  availability.
- Pure artifact formatter helper: possible later only if evidence schemas and
  paths remain unchanged.
- Adapter or shim preparation: possible later only after a plan and focused
  tests.
- Implementation move: not recommended now.

## Recommendation

Prefer no-op or documentation-only compatibility refinement until a future task
can isolate a tiny helper without touching Isaac runtime import timing, USD or
generated-asset paths, replay/artifact compatibility, launch-adjacent consoles,
shared real/sim orchestration, or memory/re-observation policy logic.
