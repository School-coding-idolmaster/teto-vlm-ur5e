# H12 Planning Import Compatibility Plan

H12-C is documentation-only. It considers a possible future canonical planning
namespace under `src/planning/`, but it does not move implementation files,
change imports, create runtime APIs, or modify launch behavior.

H12-A and H12-B established that the planning-adjacent surface is mixed across
planning, contracts, parser handoff, safety gates, real operator orchestration,
and execution gateways. Direct implementation movement remains premature.

## Migration Being Considered

The only plausible future migration target is a staged package under
`src/planning/` for planner-facing contracts and offline planning helpers.

H12-C does not approve:

- implementation moves
- import migration
- production import rewrites
- package-root re-exports
- safety-envelope migration
- execution-gateway migration
- parser/model-service migration

Current root modules remain canonical production imports until a later task
creates compatibility adapters, migrates imports in stages, and proves behavior
is unchanged.

## Candidate Future Planning Modules

Potential candidates for future planning namespace consideration:

- `src/autoregressive_motion_planner.py`: offline/contract-only planner
  preview. It produces sequential substep evidence and keeps execution
  disabled.
- `src/vector_autoregressive_motion_planner.py`: vector offline decomposition
  helper. It performs straight-line vector decomposition in TCP position space,
  not X-then-Y sequencing.
- Isolated planning-only helpers, if a later audit identifies helpers that do
  not carry safety envelope, parser/model-service, execution gateway, or real
  operator responsibilities.

These candidates are still safety-sensitive because their evidence feeds
downstream safety and execution checks. They should move only with shims and
focused tests.

## Do Not Move In The First Implementation Pass

These files must not be moved in the first implementation pass:

- `src/cartesian_motion_gateway.py`: mixed planning, validation, safety, and
  execution gateway. HIGH risk.
- `src/bounded_relative_motion.py`: shared contract and safety envelope. It is
  `SHARED_BUT_SAFE`, REAL_PATH-sensitive, and not pure planning. Move it only
  with a compatibility and safety-envelope plan.
- `src/unified_segmented_operator.py`: current segmented real/sim operator
  orchestration. REAL_PATH-sensitive.
- `scripts/teto_operator_console.py`: launch-adjacent real operator console
  import surface. Do not change in planning cleanup.
- Parser/model-service related files such as `src/qwen_motion_parser.py`:
  parser evidence must not grant execution permission, and model-service paths
  must not be started or casually migrated.
- Safety harness assumptions, including
  `scripts/safety_harnesses/run_real_long_motion_safety_check.py`: retained
  safety regression harness, not current mainline, but its fail-closed
  assumptions must be preserved.

If a later task cannot isolate a pure planning helper without touching these
surfaces, the correct implementation choice is no-op.

## Compatibility Strategy Options

Acceptable strategy options, from safest to riskiest:

- Documentation only: record boundaries and defer implementation.
- Root shim compatibility adapter: add a new canonical module while keeping the
  old root import path as a compatibility shim.
- New canonical module plus old import shim: move one implementation at a time
  and preserve public root imports during migration.
- Staged import migration: migrate tests/docs first, then scripts/offline
  consumers, then production consumers only after focused checks.
- Deletion only after all production, test, script, and docs consumers are
  migrated and ripgrep scans are clean.

Compatibility rules:

- Keep `src/planning/__init__.py` conservative.
- Do not add package-root re-exports unless a later task explicitly justifies
  them.
- Preserve current public symbols and evidence fields.
- Keep root shims during any compatibility period.
- Delete old shims only in a dedicated task after clean scans and tests.

## Required Invariants For Any Future Move

Any future move must preserve:

- `SHARED_MAX_RELATIVE_MOTION_DISTANCE_M = 0.50`
- a `0.51 m` bounded relative-motion request must block
- `REAL_ONE_SHOT_CAP_M = 0.05`
- long motion must not silently become one-shot real motion
- `E_RELATIVE_MOTION_RANGE_EXCEEDED`

Parser evidence must keep:

- `execution_permission_decided_by_parser=false`
- `safety_gate_still_required=true`

Offline planner evidence must keep:

- `execute_trajectory_called=false`
- `trajectory_sent=false`
- `real_robot_motion_executed=false`

These invariants are safety and evidence semantics, not formatting details.
Breaking them would require a dedicated safety task, not a planning namespace
cleanup.

## Proposed Future H12 Stages

- H12-D: optional no-op marker or compatibility adapter design. Prefer no-op if
  a pure adapter shape is unclear.
- H12-E: migrate only offline planner preview modules if safe:
  `src/autoregressive_motion_planner.py` and
  `src/vector_autoregressive_motion_planner.py`.
- H12-F: migrate imports in tests and docs first while root compatibility
  shims remain.
- H12-G: migrate production imports only after focused tests prove no behavior
  change.
- H12-H: delete old shims only after production, scripts, tests, and docs scans
  are clean.

If compatibility risk remains high at any stage, choose no-op and leave current
imports in place.

## Required Future Scans And Checks

Before any future implementation move, run import scans such as:

```bash
rg -n "from src\.autoregressive_motion_planner|import src\.autoregressive_motion_planner|from src\.vector_autoregressive_motion_planner|import src\.vector_autoregressive_motion_planner" src tests docs scripts -S
rg -n "from src\.bounded_relative_motion|import src\.bounded_relative_motion|from src\.cartesian_motion_gateway|import src\.cartesian_motion_gateway" src tests docs scripts -S
rg -n "from src\.planning|import src\.planning" src tests docs scripts -S
```

Focused tests for future implementation work should include the exact touched
surface and likely consumers:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONPYCACHEPREFIX=/tmp/teto_codex_pycache .venv_lab/bin/python -m pytest -p no:cacheprovider -q \
  tests/test_shared_bounded_relative_motion.py \
  tests/test_autoregressive_motion_planner.py \
  tests/test_vector_long_motion.py \
  tests/test_cartesian_motion_gateway.py \
  tests/test_motion_command_normalizer.py \
  tests/test_qwen_motion_parser.py \
  tests/test_unified_operator_interface.py \
  tests/test_real_segmented_operator.py
```

Always run:

```bash
git diff --check
bash -n scripts/start_teto_real_full_stack.sh
bash -n scripts/start_teto_isaac_gui_operator.sh
```

Launch script syntax checks are allowed; starting launch scripts is not.

## Non-Goals

H12 planning modularization is not permission for:

- real/sim behavior changes
- safety envelope changes
- MoveIt, ROS, UR driver, or Dashboard behavior changes
- parser permission changes
- execution gateway migration
- launch script modification
- model service startup
- treating legacy or safety harnesses as current mainline

## Recommendation

Proceed only with a no-op marker or compatibility adapter design if a later
task needs code. Do not perform a direct implementation move until the offline
planner preview modules can be isolated behind root shims and the required
focused tests are explicitly part of the task.
