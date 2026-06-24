# Planning Boundary

This package is reserved for future task intent, planner gateway, bounded
target, command normalization, and motion plan contract logic.

Existing files that already carry related responsibilities include:

- `src/planner_gateway_shadow.py`
- `src/planner_gateway_contract.py`
- `src/command_to_task_adapter.py`
- `src/motion_command_normalizer.py`
- `src/qwen_motion_parser.py`
- `src/autoregressive_motion_planner.py`
- `src/vector_autoregressive_motion_planner.py`

H8 intentionally leaves those files in place. They are coupled to shared
bounded motion, real and Isaac command semantics, and existing safety tests.

This package is only a future boundary. H8 does not change runtime behavior,
startup behavior, real robot behavior, Isaac behavior, or safety semantics.
