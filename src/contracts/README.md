# Contracts Boundary

This package is reserved for future shared schemas, evidence contracts,
readiness contracts, and safety-neutral validation helpers.

For the current H13 boundary policy, read
`docs/module_guides/contracts.md`. H13-A completed a read-only audit. H13-B is
documentation-only and does not migrate files, change imports, create runtime
APIs, or create `src/readiness/`.

Existing files that already carry related responsibilities include:

- `src/bounded_relative_motion.py`
- `src/json_validator.py`
- `src/execution_readiness_contract.py`
- `src/projector_contract.py`
- `src/planner_gateway_contract.py`
- `src/moveit_plan_only_contract.py`
- `src/ur5_read_only_state_contract.py`
- `src/simulation_bridge_contract.py`
- `src/ros2_interface_readiness.py`
- `src/ros2_message_exporter.py`
- `src/robot_system_shadow_bridge.py`

H8 intentionally leaves those files in place because their import paths are
shared by replay, simulation, real-path safety checks, and evidence export.

This package is only a future boundary. H8 does not change runtime behavior,
startup behavior, real robot behavior, Isaac behavior, or safety semantics.

H13 keeps the same conservative policy. Future use of this package should be
limited to stable, no-motion shared evidence contracts, schema contracts, and
validation helpers. Do not use it as a dumping ground for safety, execution,
simulation runtime, projector implementation, artifact generation, or
real-path semantics. Package-root re-exports should remain conservative unless
a later audited task explicitly justifies them.
