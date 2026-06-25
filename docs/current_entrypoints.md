# Current Entrypoints

This file is the canonical source of truth for the current user-facing TETO
Real and Isaac startup commands. If README sections, release notes, historical
harness docs, or script comments conflict with this file, follow this file and
`CODEX_RULES.md`.

## Real Mainline

Current canonical Real mainline entrypoint:

```bash
bash scripts/start_teto_real_full_stack.sh
```

This command can start real-lab services. Do not run it unless the task
explicitly authorizes Real startup and the lab has a qualified human operator
present.

## Isaac Mainline

Current canonical Isaac SIM_ONLY operator entrypoint:

```bash
bash scripts/start_teto_isaac_gui_operator.sh \
  --gui --console \
  --isaac-app /home/genlab/isaac-sim/isaac-sim.sh \
  --ur5e-asset outputs/isaac_assets/generated_ur5e/ur5e_clean_no_tool.usd \
  --motion-duration-sec 3.0 \
  --substep-pause-sec 0.35
```

Isaac is SIM_ONLY. This entrypoint must not be treated as real robot evidence
and must not be mixed with the Real backend.

## Non-Mainline Paths

Legacy, compatibility, harness, and safety-harness scripts are not current
mainline operator entrypoints. They may exist for historical evidence,
regression coverage, dry-run checks, or explicit manual fallback, but they must
not be promoted to the default Real or Isaac user path.

Examples of non-mainline paths include:

- `scripts/legacy/*`
- `scripts/harnesses/*`
- `scripts/safety_harnesses/*`
- `scripts/run_text_to_ur5e_real.sh`
- `scripts/run_text_to_ur5e_dry_run.sh`
- `scripts/run_qwen_acceptance_dry_run.sh`
- `scripts/run_qwen_acceptance_plan_only.sh`

## Startup Restrictions

Unless a task explicitly authorizes it, do not start:

- Real hardware or UR driver.
- Isaac Sim.
- ROS or MoveIt.
- RealSense.
- Qwen, VLM, LLM, or model services.

Module refactors, package moves, compatibility shims, and documentation updates
must preserve the two current user-facing startup commands above. Internal
module encapsulation must not change those command lines, their path semantics,
or their operator expectations.
