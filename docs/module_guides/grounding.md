# Grounding Module Guide

This guide records the H9 grounding module policy for future Codex, GPT, and
human audits. The current grounding code is split into small modules under
`src/grounding/`; the former root-level grounding compatibility shims were
removed in H9-A8B.

## Public Import Policy

Prefer explicit imports from the current package:

- `src.grounding.result`
- `src.grounding.vlm_adapter`
- `src.grounding.command_normalization`
- `src.grounding.reporting`
- `src.grounding.forbidden_fields`
- `src.grounding.scene_binding`

The package root intentionally does not collect public symbols. Keep
`src/grounding/__init__.py` with an empty `__all__` unless a future task
explicitly approves a package-level API.

## Removed Legacy Paths

The legacy root-level shims have been removed:

- `src/grounding_result.py`
- `src/vlm_grounding_adapter.py`

Do not reintroduce alternate root-level shims or import hacks for those paths.
Runtime code, tests, and new utilities should import from the current
`src.grounding.*` modules directly.

Do not rename or remove public classes, functions, constants, modes, status
values, or error codes without a dedicated migration plan.

## Forbidden Dependencies

Grounding modules must not import or start:

- Hardware drivers.
- Isaac Sim.
- ROS or MoveIt.
- RealSense.
- Qwen, VLM, LLM, or model runtimes.
- Real or Isaac execution backends.

Grounding code should remain file/config/contract oriented and no-motion by
default.

## Startup Script Protection

The user-facing startup commands must remain stable:

```bash
bash scripts/start_teto_real_full_stack.sh
```

```bash
bash scripts/start_teto_isaac_gui_operator.sh \
  --gui --console \
  --isaac-app /home/genlab/isaac-sim/isaac-sim.sh \
  --ur5e-asset outputs/isaac_assets/generated_ur5e/ur5e_clean_no_tool.usd \
  --motion-duration-sec 3.0 \
  --substep-pause-sec 0.35
```

Do not change these scripts, their arguments, default behavior, path semantics,
or operator import paths during grounding-only work. For grounding maintenance,
use `bash -n` only unless a task explicitly permits startup.

## What Not To Move

Do not move these into grounding:

- `src/geometry_validity.py`
- `src/real_scene_shadow_pipeline.py`
- `src/perception_shadow_pipeline.py`
- `src/projector/shadow.py`
- Real backend files.
- Isaac backend files.
- Execution, planning, and shared safety behavior.

Those files consume grounding evidence or handle downstream geometry,
projection, replay, planning, or execution boundaries.

## Current Test List

Focused grounding-related checks:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONPYCACHEPREFIX=/tmp/teto_codex_pycache .venv_lab/bin/python -m pytest -p no:cacheprovider -q \
  tests/test_grounding_import_compatibility.py \
  tests/test_vlm_grounding_adapter.py \
  tests/test_geometry_validity.py \
  tests/test_real_scene_shadow_pipeline.py \
  tests/test_perception_shadow_pipeline.py \
  tests/test_projector_shadow.py
```

Full validation:

```bash
PYTHONPYCACHEPREFIX=/tmp/teto_codex_compile_pycache .venv_lab/bin/python -m compileall src scripts tests
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONPYCACHEPREFIX=/tmp/teto_codex_pycache .venv_lab/bin/python -m pytest -p no:cacheprovider -q
git diff --check
```

Startup script syntax checks only:

```bash
bash -n scripts/start_teto_real_full_stack.sh
bash -n scripts/start_teto_isaac_gui_operator.sh
```

## Future Split Plan

Completed H9 splits:

- `forbidden_fields.py`: centralize local forbidden robot-control field names
  and recursive detection.
- `scene_binding.py`: isolate `snapshot_id` and `scene_version` matching.

Possible future splits, in increasing risk order:

- `confidence_gate.py`: isolate confidence threshold and low-confidence
  rejection logic.
- `target_selection.py`: only after the project has real multi-target or
  ambiguity logic. The current mock/manual/offline target handling is small and
  should stay in `vlm_adapter.py`.

Avoid splitting constants, modes, status values, or error codes unless a future
task explicitly accepts the higher compatibility risk.
