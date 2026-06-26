# Safety Boundary

This package is reserved for future shared safety policy notes, gate wrappers,
and evidence semantics that can be audited independently from runtime backends.

For the current H14 boundary policy, read `docs/module_guides/safety.md`.
H14-A completed a read-only audit. H14-B is documentation-only and does not
migrate files, change imports, create runtime APIs, expand this package
broadly, or add package-root re-exports.

Existing files that already carry related responsibilities include:

- `src/manual_confirmation_gate.py`
- `src/bounded_relative_motion.py`
- `src/cartesian_motion_gateway.py`
- `src/moveit_pose_executor.py`
- `src/ur5_read_only_state_contract.py`
- `src/moveit_plan_only_contract.py`
- `src/real_segmented_operator_backend.py`
- `scripts/safety_harnesses/run_real_long_motion_safety_check.py`

H8 intentionally leaves those files in place. They protect real execution,
plan-only evidence, read-only state evidence, bounded motion, and fail-closed
behavior through current tests.

This package is only a future boundary. H8 does not change runtime behavior,
startup behavior, real robot behavior, Isaac behavior, or safety semantics.

H14 keeps the same conservative policy. `src/safety/` is appropriate as a
future narrow namespace for pure gate helpers, shared safety policy notes, and
small evidence semantics that can be audited independently. It is not a broad
home for execution backends, MoveIt action clients, real operator
orchestration, readiness contracts, or mixed gateways. Keep
`src/safety/__init__.py` conservative.
