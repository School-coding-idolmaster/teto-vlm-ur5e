# TETO v3.0.1

Historical release note: this document preserves historical evidence from the
v3.0.1 snapshot. Commands or paths inside may reference legacy behavior. For
current Real and Isaac entrypoints, use `docs/current_entrypoints.md`.

This is a working text-motion pipeline snapshot.

## Verified Real Robot Path

Rule-based text command -> TETO contract -> Cartesian Motion Gateway -> MoveIt -> ExecuteTrajectory -> UR5e physical motion

## Verified Qwen Path

Local HuggingFace Qwen2.5-VL-3B server -> manual natural language -> strict JSON -> TETO contract -> dry-run PASS

## Main Real Evidence

- final_status PASS
- cartesian_motion_gateway_status PASS
- cartesian_motion_execution_status PASS
- moveit_execute_error_code 1 / SUCCESS
- trajectory_sent true
- controller_command_sent true
- real_robot_motion_executed true

## Main Qwen Dry-Run Evidence

- llm_called true
- parser_source qwen_llm
- model_name Qwen/Qwen2.5-VL-3B-Instruct
- normalized_contract.delta_m [0.0, 0.0, 0.005]
- final_status PASS

## Known Limitations

- Qwen manual real execution not yet fully accepted
- fenced JSON handling pending
- joint-level trajectory safety gate pending
- execution preview pending
- vision/D455 grounding pending
- camera->base TF missing
- world_point_m missing

## Safety Note

Qwen/LLM only generates structured intent. Execution remains gated by TETO validation, Cartesian Motion Gateway, MoveIt, and UR5e driver.
