# Contracts Boundary

This package is reserved for future shared schemas, evidence contracts,
readiness contracts, and safety-neutral validation helpers.

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
