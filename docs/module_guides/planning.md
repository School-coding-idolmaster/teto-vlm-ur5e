# Planning Module Guide

This guide records the H12 planning boundary policy for future Codex, GPT, and
human audits. H12-A completed a read-only boundary audit. H12-B and H12-C are
documentation-only: no implementation files are moved, no imports are changed,
no runtime APIs are created, and no launch behavior is modified. H12-C adds an
import compatibility plan in `docs/h12_planning_import_plan.md`; it still does
not approve migration.

## Boundary Principles

Planning code may propose feasible motion intent, decompose accepted motion
contracts, or prepare planner-facing evidence. Planning code must not grant
execution permission.

Parser output must also not grant execution permission. Natural-language or
Qwen-derived motion semantics are upstream contract evidence only; downstream
safety gates and execution gateways retain veto power.

Real and sim shared behavior must remain fail-closed. Dry-run, plan-only,
offline preview, Isaac, fake, or synthetic evidence must not become REAL_PATH
success evidence.

## Current File Classification

- `src/bounded_relative_motion.py`: shared contract and safety envelope helper.
  It defines the shared bounded relative-motion contract, decomposition helper,
  planned subgoals, and `0.50 m` envelope evidence. Classification:
  `SHARED_BUT_SAFE`, REAL_PATH-sensitive, not pure planning.
- `src/autoregressive_motion_planner.py`: offline/contract-only planner
  preview. It turns accepted canonical relative motion into sequential substep
  evidence while keeping execution disabled.
- `src/vector_autoregressive_motion_planner.py`: vector offline decomposition
  helper. It uses straight-line vector decomposition in TCP position space, not
  X-then-Y sequencing.
- `src/cartesian_motion_gateway.py`: mixed planning, validation, safety, and
  execution gateway. It is HIGH risk and must not be moved in a planning
  cleanup pass.
- `src/command_to_task_adapter.py`: task-intent adapter that converts command
  evidence into task contracts and applies no-robot-control validation.
- `src/motion_command_normalizer.py`: natural-language and structured delta
  normalization for relative motion semantics. It may add bounded-envelope
  evidence but must not authorize execution.
- `src/qwen_motion_parser.py`: Qwen/model-service-capable parser boundary. It
  can call a model when explicitly used by allowed runtime paths, so import
  migration around it must be careful and must not start model services.
- `src/unified_segmented_operator.py`: current real/sim shared segmented
  operator orchestration. It consumes bounded relative-motion contracts and is
  REAL_PATH-sensitive.
- `src/guarded_vector_motion_executor.py`: guarded execution helper for vector
  long-motion safety checks. It is safety-sensitive and fail-closed by default.
- `scripts/safety_harnesses/run_real_long_motion_safety_check.py`: retained
  safety regression harness, not current mainline. Preserve its fail-closed
  assumptions and do not treat it as the canonical real entrypoint.

Planner gateway shadow and planner gateway contract files remain related but
separate from the long-motion planner cleanup surface. They bridge perception
or replay evidence into planner-facing metadata and must preserve no-motion
shadow semantics unless a future task explicitly expands their scope.

## Public APIs And Invariants

These public symbols and evidence semantics must not break:

- `SHARED_MAX_RELATIVE_MOTION_DISTANCE_M = 0.50`
- a `0.51 m` bounded relative-motion request must block
- `REAL_ONE_SHOT_CAP_M = 0.05`
- long motion must not silently become one-shot real motion
- `E_RELATIVE_MOTION_RANGE_EXCEEDED`
- `build_bounded_relative_motion_contract`
- `decompose_relative_motion`
- `planned_subgoals`
- `AutoregressiveMotionPlannerRequest`
- `plan_offline_autoregressive_motion`

Offline planner evidence must remain fail-closed:

- `execute_trajectory_called=false`
- `trajectory_sent=false`
- `real_robot_motion_executed=false`

Parser evidence must keep:

- `execution_permission_decided_by_parser=false`
- `safety_gate_still_required=true`

Any future change that weakens these invariants is not a documentation cleanup
and needs a dedicated safety review.

## Do Not Move Yet

Do not move these in H12 cleanup without a compatibility plan and focused
tests:

- `src/cartesian_motion_gateway.py`
- `src/bounded_relative_motion.py`
- `src/unified_segmented_operator.py`
- `scripts/teto_operator_console.py`
- `src/qwen_motion_parser.py` and parser/model-service-adjacent paths
- safety harness assumptions, including
  `scripts/safety_harnesses/run_real_long_motion_safety_check.py`

`src/bounded_relative_motion.py` may eventually be a shared contracts or safety
candidate, but it is not pure planning. `src/cartesian_motion_gateway.py`
spans planner-facing validation, safety gates, and execution authorization, so
it must not be absorbed into `src/planning/` wholesale.

## Future `src/planning/` Policy

`src/planning/` may become a future canonical namespace for planner-facing
contracts and offline planning helpers. It must not absorb safety envelope,
execution gateway, parser, model-service, real operator, or launch-adjacent code
wholesale.

Any future migration should be staged:

1. documentation
2. compatibility adapter plan
3. helper extraction if a pure planning helper is isolated
4. import migration
5. implementation move only after tests prove no behavior change

Until that plan exists, current root modules remain the canonical production
imports.

## Recommended H12-C Candidates

Lowest-risk next steps:

- documentation-only compatibility plan for future planning imports
- possible package marker or adapter marker with no runtime behavior
- possible tiny helper extraction only if an isolated pure-planning helper is
  identified

Do not perform a direct implementation move yet.

For import migration staging, required scans, and future shim policy, read
`docs/h12_planning_import_plan.md`.

## Startup Script Protection

The canonical startup commands remain documented in
`docs/current_entrypoints.md`:

```bash
bash scripts/start_teto_real_full_stack.sh
```

```bash
bash scripts/start_teto_isaac_gui_operator.sh \
  --gui --console \
  --isaac-app /home/genlab/isaac-sim/isaac-sim.sh \
  --ur5e-asset outputs/isaac_assets/generated_ur5e/ur5e_clean_no_tool.usd \
  --motion-duration-sec 3.0 \
  --substep-pause-sec 0.35
```

Do not modify these launch scripts, their arguments, default behavior, path
semantics, or operator expectations during planning boundary work. Use
`bash -n` only unless a task explicitly permits startup.

## Focused Checks

Documentation-only H12 work should finish with:

```bash
git diff --check
bash -n scripts/start_teto_real_full_stack.sh
bash -n scripts/start_teto_isaac_gui_operator.sh
```

If a future task touches code or tests, choose focused tests for the exact
surface changed, such as bounded relative motion, autoregressive planning,
vector long motion, Cartesian gateway, parser/normalizer, and unified operator
tests.
