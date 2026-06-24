# Execution Boundary

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

H8 intentionally leaves those files in place. They are safety-critical and
currently separate real execution, Isaac SIM_ONLY execution, and shared command
semantics.

This package is only a future boundary. H8 does not change runtime behavior,
startup behavior, real robot behavior, Isaac behavior, or safety semantics.
