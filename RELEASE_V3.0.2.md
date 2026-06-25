# TETO v3.0.2

Historical release note: this document preserves historical evidence from the
v3.0.2 Qwen manual real execution chain. Commands or paths inside may reference
legacy behavior. For current Real and Isaac entrypoints, use
`docs/current_entrypoints.md`.

TETO v3.0.2 is the first accepted Qwen manual real execution chain release.

## Summary

The v3.0.2 milestone connects the full guarded natural-language execution path:

```text
Natural language
-> Qwen parser
-> TETO task contract
-> execution preview / safety gate
-> manual confirmation
-> MoveIt execution
-> UR5e real motion
-> post-motion verification evidence
```

This release means the Qwen/TETO/MoveIt/UR5e real execution chain is connected and verified through the existing guarded workflow. It does not claim that all motion semantics, frame conventions, or direction correctness are fully solved.

## Final Real Acceptance Evidence

Final guarded command:

```bash
bash scripts/run_qwen_manual_acceptance.sh --cmd "raise the tcp by 2 millimeters" --real-small-motion --auto-start-qwen
```

Observed acceptance summary:

- Qwen parse completed.
- TETO normalized a relative Cartesian motion command.
- Execution preview passed the tiny-motion safety gate.
- Manual confirmation was required before real execution.
- MoveIt execution path was used through the existing TETO gated real-small-motion workflow.
- A trajectory/controller command was sent by the guarded execution path after manual confirmation.
- `real_robot_motion_executed` became true.
- Post-motion verification evidence was recorded.

Post-motion verification found a direction/frame mismatch:

- Requested direction: `z+`
- Requested distance: `0.002 m`
- Measured displacement: approximately `z- 0.002433 m`
- Result interpretation: real motion occurred, but measured motion direction did not match the requested semantic direction.

This mismatch is preserved as a known issue for the next version.

## Post-Motion Verification Evidence

`scripts/text_to_ur5e_real_motion.py` now records post-motion evidence after a successful real execution by reading the current TCP pose again. The final evidence JSON includes:

- `post_motion_verification`
- `post_motion_verification_status`
- `tcp_pose_before_execution`
- `target_tcp_pose`
- `tcp_pose_after_execution`
- `intended_delta_m`
- `actual_displacement_m`
- `actual_displacement_distance_m`
- `actual_distance_error_m`
- `intended_direction`
- `actual_direction`
- `direction_check_passed`
- `orientation_change_rad`

Top-level convenience fields are also exported:

- `actual_displacement_m`
- `actual_displacement_distance_m`
- `actual_distance_error_m`
- `orientation_change_rad`
- `post_motion_verification_status`

Blocked or no-motion paths record `post_motion_verification_status: "NOT_RUN"` with reason `real_robot_motion_executed=false`.

## Startup And Bringup

Qwen server startup now runs in the `qwen_vl` conda environment, which provides `torch`, `transformers`, and `qwen_vl_utils`. The TETO client path remains in `.venv_lab`.

Bring up dependencies without opening the operator console:

```bash
bash scripts/start_teto_qwen_real_operator.sh --bringup-only
```

Open the guarded natural-language operator console:

```bash
bash scripts/start_teto_qwen_real_operator.sh --console
```

Check status without starting Qwen, launching MoveIt, switching controllers, or opening a console:

```bash
bash scripts/start_teto_qwen_real_operator.sh --status
```

The operator console runs:

```bash
bash scripts/qwen_operator_console.sh
```

Each natural-language command is passed to the existing guarded workflow without `--yes`:

```bash
bash scripts/run_qwen_manual_acceptance.sh --cmd "$USER_CMD" --real-small-motion --auto-start-qwen
```

Manual confirmation remains required for real execution.

## Validation Results

Release preparation validation:

- `PYTHONPATH=. .venv_lab/bin/python -m pytest`: `565 passed`, 2 pytest cache warnings
- `PYTHONPYCACHEPREFIX=/tmp/teto_pycache_v302 python3 -m py_compile src/*.py scripts/*.py`: passed
- `bash -n scripts/*.sh`: passed

The pytest warnings were cache write warnings from the managed read-only `.pytest_cache` path and were not test failures.

No robot motion, `ExecuteTrajectory`, trajectory send, or confirmation input was run during release preparation.

## Known Issues

- Real-motion direction/frame mismatch remains open: the final acceptance command requested `z+ 0.002 m`, while post-motion verification measured approximately `z- 0.002433 m`.
- Motion semantics and frame convention correctness need a follow-up fix before claiming direction-level correctness.
- Calibration mismatch warning may still be present in the UR driver startup logs. Treat it as a known warning until the loaded `kinematics_params_file` is explicitly proven to match the lab UR5e calibration.

## Safety Notes

- The guarded real-small-motion path still requires manual confirmation.
- `--yes` remains blocked for `--real-small-motion`.
- Command and distance whitelists remain restricted.
- Startup and bringup scripts do not execute robot motion automatically.
- No raw URScript path was added.
- No direct raw trajectory publishing path was added.
