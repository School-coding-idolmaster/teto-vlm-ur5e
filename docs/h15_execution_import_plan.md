# H15 Execution Import Compatibility Plan

H15-C is documentation-only. It considers a possible future narrow canonical
namespace under `src/execution/`, but it does not move implementation files,
change imports, create adapters, extract helpers, create runtime APIs, or
modify launch behavior.

H15-A and H15-B established that active execution authority is split across
measured real backend state, MoveIt action evidence, shared segmented
orchestration, mixed Cartesian gateway behavior, execution guards, and SIM_ONLY
Isaac code. Direct implementation movement remains premature.

## Migration Being Considered

The only plausible future target is a staged, narrow package under
`src/execution/` for execution adapters, backend interfaces, measured execution
evidence helpers, and small pure execution abstractions that do not hide
authority boundaries.

H15-C does not approve:

- implementation moves
- import migration
- production import rewrites
- runtime API creation
- adapter or shim creation
- helper extraction
- package-root re-exports
- execution authority relocation
- launch script changes

Current root modules remain canonical production imports until a later audited
task creates compatibility adapters, migrates imports in stages, and proves
behavior is unchanged.

## Candidate Future Modules

Potential candidates for future execution namespace consideration:

- A tiny `UnifiedOperatorBackend` protocol/interface extraction from
  `src/unified_segmented_operator.py`, if import scans prove it can be isolated
  without changing real or Isaac behavior.
- A measured execution evidence helper, only if it is pure, small,
  independently tested, and does not decide whether execution is allowed.
- A narrow execution adapter marker, only after compatibility planning and
  focused tests define what it may import.

These are future candidates only. They do not authorize a direct move of active
REAL_PATH authority.

## Do Not Move In The First Implementation Pass

These files must not be moved in the first implementation pass:

- `src/cartesian_motion_gateway.py`: HIGH-risk mixed planning, validation,
  safety, and execution gateway.
- `src/bounded_relative_motion.py`: shared contract and safety envelope.
- `src/moveit_pose_executor.py`: REAL_PATH MoveIt plan/execute wrapper and
  action evidence boundary.
- `src/real_segmented_operator_backend.py`: REAL_PATH measured backend
  authority.
- `src/unified_segmented_operator.py`: shared real/sim segmented orchestration.
- `src/guarded_vector_motion_executor.py`: execution guard for vector
  long-motion paths.
- `src/manual_confirmation_gate.py`: execution gate, not UI-only helper.
- `scripts/teto_operator_console.py`: launch-adjacent real operator console.
- canonical launch scripts.
- safety harnesses.
- readiness/contracts files.

If a later task cannot isolate a pure helper without touching these surfaces,
the correct implementation choice is no-op.

## Compatibility Strategy Options

Acceptable strategy options, from safest to riskiest:

- Documentation only: record boundaries and defer implementation.
- No-op: leave current imports and files in place if authority boundaries
  remain mixed.
- Marker-only plan: refine `src/execution/` documentation without runtime
  imports.
- Root shim compatibility adapter: only after a future implementation plan.
- New canonical module plus old import shim: only after import scans and
  focused tests.
- Staged import migration: migrate tests/docs first, then scripts or offline
  consumers, then production consumers only after focused checks.
- Deletion only after all production, test, docs, CLI, and launch-adjacent
  consumers are migrated and ripgrep scans are clean.

Compatibility rules:

- Keep `src/execution/__init__.py` conservative.
- Do not add package-root re-exports unless a later task explicitly justifies
  them.
- Preserve current public symbols and evidence fields.
- Keep root compatibility shims during any migration window.
- Delete old shims only in a dedicated task after clean scans, focused tests,
  and human approval.

## Required Invariants For Any Future Move

Any future move must preserve:

- root imports until an audited migration exists
- parser, planner, and contract outputs never grant execution permission
- `safety_gate_still_required=True`
- manual confirmation remains an execution gate
- dry-run evidence remains no-motion
- plan-only evidence remains no-motion
- fake-publish evidence remains no-motion
- read-only evidence remains no-motion
- `SHARED_MAX_RELATIVE_MOTION_DISTANCE_M = 0.50`
- `REAL_ONE_SHOT_CAP_M = 0.05`
- Dashboard, RTDE, MoveIt, and UR driver actions remain gated
- D455 freshness checks remain meaningful
- SIM_ONLY code must not import REAL_PATH authority accidentally
- REAL_PATH code must not depend on Isaac-only operator state

Real execution evidence must remain tied to:

- real backend involvement
- attempted MoveIt execution
- measured post-motion verification semantics

These invariants are safety and evidence semantics, not formatting details.
Breaking them would require a dedicated real-path safety and execution task,
not an execution namespace cleanup.

## Proposed Future H15 Stages

- H15-D: no-op or documentation-only compatibility refinement if risk remains
  high.
- H15-E: optional tiny protocol extraction plan, not implementation.
- H15-F: if approved later, extract only `UnifiedOperatorBackend`
  protocol/interface with old root import compatibility.
- H15-G: migrate tests/docs first, then production imports only after focused
  tests.
- H15-H: delete old shims only after clean scans, full focused tests, and
  human approval.

If compatibility risk remains high at any stage, choose no-op and leave
current imports in place.

## Required Future Scans And Checks

Before any future implementation move, run direct import scans such as:

```bash
rg -n "from src\.real_segmented_operator_backend|import src\.real_segmented_operator_backend|from src\.moveit_pose_executor|import src\.moveit_pose_executor" src tests docs scripts -S
rg -n "from src\.unified_segmented_operator|import src\.unified_segmented_operator|from src\.cartesian_motion_gateway|import src\.cartesian_motion_gateway" src tests docs scripts -S
rg -n "from src\.manual_confirmation_gate|import src\.manual_confirmation_gate|from src\.guarded_vector_motion_executor|import src\.guarded_vector_motion_executor" src tests docs scripts -S
rg -n "from src\.execution|import src\.execution" src tests docs scripts -S
```

Also scan protected and launch-adjacent surfaces:

```bash
rg -n "start_teto_real_full_stack|start_teto_isaac_gui_operator|teto_operator_console|teto_isaac_operator_console" src tests docs scripts -S
rg -n "Dashboard|RTDE|MoveIt|ExecuteTrajectory|execute_trajectory|UR driver|D455|real_robot_motion_executed|safety_gate_still_required" src tests docs scripts -S
```

Focused tests for future implementation work should include the exact touched
surface and likely consumers:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONPYCACHEPREFIX=/tmp/teto_codex_pycache .venv_lab/bin/python -m pytest -p no:cacheprovider -q \
  tests/test_real_segmented_operator.py \
  tests/test_unified_operator_interface.py \
  tests/test_isaac_sim_operator.py \
  tests/test_cartesian_motion_gateway.py \
  tests/test_moveit_pose_executor.py \
  tests/test_manual_confirmation_gate.py \
  tests/test_vector_long_motion.py
```

Add safety or regression harness tests only if the future implementation
touches those surfaces.

Always run:

```bash
git diff --check
bash -n scripts/start_teto_real_full_stack.sh
bash -n scripts/start_teto_isaac_gui_operator.sh
```

Launch script syntax checks are allowed; starting launch scripts is not.

## Non-Goals

H15 execution modularization is not permission for:

- real/sim behavior changes
- execution authority relocation
- MoveIt, ROS, UR driver, RTDE, or Dashboard behavior changes
- parser permission changes
- safety envelope changes
- manual confirmation weakening
- launch script modification
- operator console behavior changes
- model service startup
- hardware, simulation, ROS, MoveIt, UR driver, RealSense, Qwen, VLM, LLM, or
  model-service startup

## Potential Future Safe Work

- Documentation-only compatibility/import plan: safe now.
- `UnifiedOperatorBackend` protocol extraction: possible later, but requires
  xhigh review, import scans, root compatibility, and focused real/sim tests.
- Measured execution evidence helper: possible later only if pure and no
  authority relocation is involved.
- Adapter or shim preparation: possible later only after a plan and focused
  tests.
- Implementation move: not recommended now.

## Recommendation

Prefer no-op or documentation-only compatibility refinement until a future task
can isolate a tiny helper without touching REAL_PATH authority, SIM_ONLY
isolation, launch-adjacent consoles, or mixed gateway behavior.
