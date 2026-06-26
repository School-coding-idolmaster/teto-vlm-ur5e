# H17 Memory / Re-Observation Import Compatibility Plan

H17-C is documentation-only. It creates an import compatibility plan for a
possible future memory / re-observation migration, but it does not create
packages, move implementation files, change imports, create compatibility
shims, add runtime APIs, add package-root re-exports, or modify launch
behavior.

H17-A found the boundary to be small but behavior-sensitive. H17-B documented
the ownership boundary in `docs/module_guides/memory_reobservation.md`. H17-C
keeps the current root modules in place and treats all package layouts below
as future candidates only.

## Current Implementation Files

- `src/memory_guided_execution.py`: current implementation for working-memory
  state, scene monitor normalization, event-triggered re-observation policy,
  and working-memory updates.
- `src/adaptive_reobservation_policy.py`: current implementation for adaptive
  re-observation phases, model-call suppression policy, recovery
  re-observation, replan, and abort handoff fields.

These root modules are implementations, not compatibility shims.

## Current Import Map

Direct imports found by H17-C:

| Consumer | Import | Classification |
| --- | --- | --- |
| `src/adaptive_reobservation_policy.py` | `from src.memory_guided_execution import ...` | production `src/`, policy-to-policy dependency |
| `src/isaac_sim_operator.py` | `from src.adaptive_reobservation_policy import ...` | production `src/`, SIM_ONLY canonical Isaac consumer |
| `src/isaac_sim_operator.py` | `from src.memory_guided_execution import ...` | production `src/`, SIM_ONLY canonical Isaac consumer |
| `tests/test_memory_guided_execution.py` | `from src.memory_guided_execution import ...` | focused test |
| `tests/test_adaptive_reobservation_policy.py` | `from src.adaptive_reobservation_policy import ...` | focused test |
| `tests/test_adaptive_reobservation_policy.py` | `from src.memory_guided_execution import ...` | focused test |

Indirect canonical consumer:

- `scripts/teto_isaac_operator_console.py` imports `src.isaac_sim_operator`,
  so the canonical Isaac console depends on both root modules indirectly.

References without direct runtime imports:

- `docs/module_guides/memory_reobservation.md`
- `docs/module_guides/simulation.md`
- `docs/h8_module_boundaries.md`
- `docs/h16_simulation_import_plan.md`
- focused tests and configs that mention re-observation fields
- `src/real_segmented_operator_backend.py` contains only the future-hook string
  `adaptive_reobservation_or_vlm_scene_guard`
- `src/cartesian_motion_gateway.py` emits `substep_reobserve_allowed` fields,
  but does not import either memory / re-observation module

H17-C reconfirmed that REAL_PATH files do not directly import
`src.memory_guided_execution` or `src.adaptive_reobservation_policy`.

## Current Canonical Consumers

- `src/isaac_sim_operator.py`: SIM_ONLY production consumer. It uses the policy
  outputs to populate working-memory evidence, scene monitor evidence,
  `continue_allowed`, `reobserve_triggered`, `reobserve_reason`,
  `replan_required`, `abort_reason`, and execution-load fields.
- `scripts/teto_isaac_operator_console.py`: launch-adjacent consumer through
  `src.isaac_sim_operator`.
- `tests/test_memory_guided_execution.py`: focused memory policy coverage.
- `tests/test_adaptive_reobservation_policy.py`: focused adaptive policy
  coverage.
- `tests/test_isaac_sim_operator.py`: integration coverage for Isaac evidence
  and stale-scene re-observation behavior.

The canonical Real startup path is not a direct consumer today.

## Why Direct Migration Is Not Allowed Yet

Direct movement is not safe in the next step because:

- the canonical Isaac path imports the current root modules indirectly through
  `src.isaac_sim_operator`;
- Isaac evidence records the exact policy outputs, so import movement can break
  artifact generation even if behavior is unchanged;
- no future package layout has been chosen;
- no compatibility shim exists yet;
- no staged import order has been approved;
- root imports are the current public compatibility surface for production
  code and tests;
- future REAL_PATH hooks could become safety-sensitive if converted into
  runtime dependencies.

## Candidate Future Package Layouts

These are examples only. H17-C does not choose or create any package.

Combined package candidate:

- `src/execution_memory/`
- Possible fit: keeps working-memory and re-observation policy together as
  execution-adjacent state.
- Concern: the name may imply execution authority, even though these modules
  must not grant motion permission.

Split package candidate:

- `src/memory/`
- `src/reobservation/`
- Possible fit: separates persistent working-memory state from policy
  decisions.
- Concern: can create premature package boundaries and import churn for a very
  small surface.

Policy package candidate:

- `src/execution_policy/`
- Possible fit: describes deterministic execution-adjacent policies without
  owning the execution backend.
- Concern: can blur safety, planning, and execution authority unless the
  package stays narrow and avoids root re-exports.

No package candidate:

- Keep `src/memory_guided_execution.py`.
- Keep `src/adaptive_reobservation_policy.py`.
- Possible fit: lowest compatibility risk while the canonical consumer remains
  SIM_ONLY Isaac evidence generation.

## Compatibility Strategy Options

| Strategy | Description | Risk |
| --- | --- | --- |
| Keep current root modules and defer migration | Leave imports and files exactly where they are. | LOW. This preserves canonical Isaac behavior and avoids import churn. |
| Add new package modules and keep root compatibility shims temporarily | Move or copy implementation into a chosen package, then make old root modules import from the new location. | MEDIUM to HIGH. Requires package choice, shim policy, focused tests, and proof that startup/evidence semantics are unchanged. |
| Full import migration after compatibility window | Migrate production, scripts, tests, docs, and configs to the new package path, then remove root shims only after clean scans. | HIGH. The canonical Isaac path, evidence fields, and future REAL_PATH hooks make deletion risky without a staged audit. |

The default strategy after H17-C should be to keep the current root modules and
defer migration.

## Required Checks Before Any Future Migration

Before package creation, shim creation, import rewrites, or file moves:

```bash
git status -sb
git rev-list --left-right --count origin/master...HEAD
rg -n "from src\.memory_guided_execution|import src\.memory_guided_execution|from src\.adaptive_reobservation_policy|import src\.adaptive_reobservation_policy" src scripts tests docs configs -S
rg -n "memory_guided_execution|adaptive_reobservation_policy|working_memory|scene_monitor|reobserve|re-observation|adaptive_reobservation" src scripts tests docs configs -S
rg -n "memory_guided_execution|adaptive_reobservation_policy" src/real_segmented_operator_backend.py src/unified_segmented_operator.py src/cartesian_motion_gateway.py src/moveit_pose_executor.py scripts/teto_operator_console.py scripts/start_teto_real_full_stack.sh scripts/start_teto_isaac_gui_operator.sh scripts/teto_isaac_operator_console.py -S
```

Focused tests should be planned before code changes:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONPYCACHEPREFIX=/tmp/teto_codex_pycache .venv_lab/bin/python -m pytest -p no:cacheprovider -q \
  tests/test_memory_guided_execution.py \
  tests/test_adaptive_reobservation_policy.py \
  tests/test_isaac_sim_operator.py
```

Syntax-only launch checks:

```bash
bash -n scripts/start_teto_real_full_stack.sh
bash -n scripts/start_teto_isaac_gui_operator.sh
```

Always include:

```bash
git diff --check
```

Do not start hardware, Isaac Sim, ROS, MoveIt, the UR driver, RealSense, Qwen,
a VLM, an LLM, or model services during these checks.

## Required Checks After Any Future Migration

After any future package creation, shim creation, import rewrite, or file move:

- rerun all before-migration scans;
- scan for old root imports and new package imports explicitly;
- verify root compatibility shims still work if they exist;
- verify `src/isaac_sim_operator.py` imports from the intended path;
- verify `scripts/teto_isaac_operator_console.py` still reaches the Isaac
  operator without broad Isaac startup during import inspection;
- run focused memory, adaptive policy, and Isaac operator tests;
- run startup script syntax checks only;
- run `git diff --check`;
- inspect evidence field names in the touched tests for compatibility.

Root shim deletion must be a separate task after clean scans prove no
production, script, test, config, or documentation consumer needs the old
paths.

## No-Go Conditions

Do not proceed with implementation migration if any of these are true:

- canonical Isaac startup risk is unresolved;
- a REAL_PATH direct dependency appears;
- the import graph is unclear;
- required tests are hardware, simulation, camera, robot, GPU model service, or
  network dependent;
- docs disagree with `docs/current_entrypoints.md`;
- package naming is still disputed;
- migration would alter evidence schemas or fail-closed policy fields;
- compatibility shim deletion is bundled with the first move.

## Documentation Updates Required For Future Migration

If a future migration happens, update:

- `docs/module_guides/memory_reobservation.md`
- `docs/h17_memory_reobservation_import_plan.md`
- `docs/h8_module_boundaries.md`
- `docs/module_guides/README.md` if a new package guide or local README is
  created
- `docs/module_guides/execution.md` if execution imports or consumer status
  changes
- `docs/module_guides/simulation.md` if Isaac imports or consumer status
  changes
- any future `src/<chosen-package>/README.md`
- focused tests and docs that mention old root import paths

Do not update `CODEX_RULES.md` unless a future task changes a hard safety rule.
Do not update `docs/current_entrypoints.md` unless the user-facing startup
surface changes; H17 migration should preserve it.

## Recommendation

Recommended next step after H17-C:

- H17-D keep modules in place and stop H17 migration.

Rationale: the current root modules are small, direct, and already documented.
The only production consumer is the canonical SIM_ONLY Isaac operator, where
evidence compatibility matters. A package move would create compatibility work
without clear boundary payoff yet. Reopen migration only after a new feature or
consumer makes the root-module layout an actual maintenance problem.
