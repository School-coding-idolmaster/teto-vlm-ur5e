# Planning Boundary

This package is reserved for future task intent, planner gateway, bounded
target, command normalization, and motion plan contract logic. Current H12
guidance lives in `docs/module_guides/planning.md`. The H12-C compatibility
and import migration plan lives in `docs/h12_planning_import_plan.md`.

H12-A completed a read-only planning boundary audit. H12-B is
documentation-only. No implementation has been migrated into this package yet,
and no imports should target this package for production planning behavior.

Planning code may propose or decompose feasible motion intent, but it must not
grant execution permission. Parser output must not grant execution permission.
Safety gates and execution gateways retain veto power, and real/sim shared
behavior must remain fail-closed.

Existing files that already carry related responsibilities include:

- `src/planner_gateway_shadow.py`
- `src/planner_gateway_contract.py`
- `src/command_to_task_adapter.py`
- `src/motion_command_normalizer.py`
- `src/qwen_motion_parser.py`
- `src/autoregressive_motion_planner.py`
- `src/vector_autoregressive_motion_planner.py`
- `src/bounded_relative_motion.py`
- `src/cartesian_motion_gateway.py`
- `src/unified_segmented_operator.py`
- `src/guarded_vector_motion_executor.py`

H8 intentionally left those files in place. H12 confirms they remain coupled to
shared bounded motion, real and Isaac command semantics, safety gates, parser
handoff, execution gateways, and existing safety tests.

This package is only a future boundary. H12-B and H12-C do not change runtime
behavior, startup behavior, real robot behavior, Isaac behavior,
parser/model-service behavior, imports, or safety semantics.

Do not move `src/cartesian_motion_gateway.py`, `src/bounded_relative_motion.py`,
`src/unified_segmented_operator.py`, `scripts/teto_operator_console.py`,
`src/qwen_motion_parser.py`, or safety harness assumptions without a dedicated
compatibility plan and focused tests.

If a future task creates package-side planning adapters, keep
`src/planning/__init__.py` conservative and avoid package-root re-exports unless
explicitly justified.
