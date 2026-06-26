# Execution Boundary

For the current H15 boundary policy, read `docs/module_guides/execution.md`.
H15-A completed a read-only execution/operator audit. H15-B is
documentation-only. H15-C adds the compatibility plan in
`docs/h15_execution_import_plan.md`. H15 does not migrate files, change
imports, create runtime APIs, expand this package broadly, or add package-root
re-exports.

This package is reserved for future execution adapters, backend interfaces,
MoveIt execution routing, Isaac execution routing, and measured execution
evidence.

Existing files that already carry related responsibilities include:

- `src/unified_segmented_operator.py`
- `src/real_segmented_operator_backend.py`
- `src/cartesian_motion_gateway.py`
- `src/moveit_pose_executor.py`
- `src/isaac_sim_operator.py`
- `src/isaac_sim_bridge.py`
- `src/simulation_micro_motion.py`
- `src/simulated_task_execution.py`
- `src/manual_confirmation_gate.py`
- `src/guarded_vector_motion_executor.py`

H8 intentionally leaves those files in place. They are safety-critical and
currently separate real execution, Isaac SIM_ONLY execution, and shared command
semantics.

This package is only a future boundary. H8 does not change runtime behavior,
startup behavior, real robot behavior, Isaac behavior, or safety semantics.

H15 keeps the same conservative policy. `src/execution/` is appropriate later
as a narrow namespace for execution adapters, backend interfaces, measured
execution evidence helpers, and small pure execution abstractions that do not
hide authority boundaries. It is not a broad home for the mixed Cartesian
gateway, bounded motion safety envelope, parser/planner contracts,
safety/readiness contracts, launch scripts, or real operator consoles. Keep
`src/execution/__init__.py` conservative.
