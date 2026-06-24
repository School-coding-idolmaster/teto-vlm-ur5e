# CODEX Rules for TETO

## Golden Rule

Do not change real robot execution, safety gates, startup behavior, or evidence semantics unless the task explicitly asks for it.

## Before Any Change

Run:

```bash
pwd
git status --short
git status -sb
git log --oneline -5
```

Confirm the current branch and every dirty file before editing.

## Change Classification

Classify each requested change as one of:

- REAL_PATH
- SIM_ONLY
- SHARED_BUT_SAFE
- RISKY_MIXED
- LEGACY_OR_DEPRECATED
- OUTPUT_OR_ARTIFACT

If a change spans categories, split the commit or stop for audit.

## Legacy / Debug Entrypoint Rules

- Before using any script with a LEGACY / DEBUG / HISTORICAL warning, Codex must ask for explicit user confirmation.
- Do not treat legacy/debug scripts as the current architecture.
- Do not delete legacy/debug scripts without a dedicated cleanup audit.
- Do not infer REAL_PATH behavior from legacy/debug scripts.

## Hard Prohibitions

- Do not run UR5e hardware unless explicitly requested.
- Do not start UR driver unless explicitly requested.
- Do not start MoveIt execute unless explicitly requested.
- Do not start Isaac unless explicitly requested.
- Do not access RealSense unless explicitly requested.
- Do not start Qwen/VLM service unless explicitly requested.
- Do not use `git add .`.
- Do not force push.
- Do not delete outputs unless explicitly requested.
- Do not claim real execution from fake / dry-run / Isaac / plan-only evidence.

## Real Path Rules

- Real default mode is autonomous segmented execution with measured per-segment gates.
- Old manual `y` confirmation is legacy-only.
- Real execution requires measured readiness and post-motion verification.
- MoveIt ExecuteTrajectory must not be called without safety gates.
- Protective stop recovery must not be automated by LLM.

## Isaac Rules

- Isaac is SIM_ONLY.
- Isaac must not import real backend or call Dashboard / RTDE / UR driver / ExecuteTrajectory.
- Isaac may use shared bounded contracts but must preserve SIM_ONLY evidence.

## Evidence Rules

- `real_robot_motion_executed=true` must only mean real backend + MoveIt execute + measured verification.
- `simulated_only=true` cannot be used as real evidence.
- plan-only success cannot be execution success.
- fake/synthetic gateway output cannot be real success.

## Testing Rules

Default offline test command:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONPYCACHEPREFIX=/tmp/teto_codex_pycache .venv_lab/bin/python -m pytest -p no:cacheprovider -q
```

Default compile check:

```bash
PYTHONPYCACHEPREFIX=/tmp/teto_codex_compile_pycache .venv_lab/bin/python -m compileall src scripts tests
```

## Commit Rules

- Keep commits small and themed.
- Do not mix docs cleanup, real safety changes, Isaac changes, and legacy cleanup in one commit.
- Stage only intended files.
- Always show `git diff --cached --name-only` before commit.
