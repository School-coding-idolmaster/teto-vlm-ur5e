# Grounding Module

The grounding module converts declared VLM/Qwen target evidence into structured
target candidates that downstream vision, contract, replay, and planning code
can consume. It is intentionally no-motion and evidence-oriented.

Grounding is responsible for:

- Loading and normalizing grounding result contracts.
- Adapting mock, offline, manual, disabled, and future-Qwen grounding modes.
- Normalizing user text commands used by the grounding adapter.
- Preserving target fields such as label, object id, bbox, pixel center,
  confidence fields, snapshot id, and scene version.
- Formatting VLM grounding evidence reports.

Grounding is not responsible for:

- 2D-to-3D projection.
- Camera-to-base TF.
- Camera capture or RealSense operation.
- MoveIt planning.
- Real or Isaac execution.
- Safety gate orchestration outside the local no-motion grounding boundary.

## Current Files

- `result.py`: Grounding result request dataclass, result file loading, and
  grounding result contract normalization.
- `vlm_adapter.py`: Public VLM grounding adapter API, adapter modes, status and
  error codes, confidence checks, no-motion flags, and adapter orchestration.
- `command_normalization.py`: `normalize_command` implementation.
- `forbidden_fields.py`: forbidden robot-control field constants and recursive
  detection helper.
- `reporting.py`: `format_vlm_grounding_report` implementation.
- `scene_binding.py`: `snapshot_id` and `scene_version` binding mismatch
  helper.

## Import Policy

New code should prefer explicit module imports:

- `from src.grounding.result import ...`
- `from src.grounding.vlm_adapter import ...`
- `from src.grounding.command_normalization import normalize_command`
- `from src.grounding.forbidden_fields import ...`
- `from src.grounding.reporting import format_vlm_grounding_report`
- `from src.grounding.scene_binding import ...`

The legacy root-level compatibility shims were removed in H9-A8B:

- `src/grounding_result.py`
- `src/vlm_grounding_adapter.py`

Do not restore those shims or add import hacks for the old paths. Use explicit
`src.grounding.*` submodule imports instead.

`src/grounding/__init__.py` intentionally keeps an empty `__all__`. Use explicit
submodule imports instead of adding a broad package-level public API.

## Tests

Recommended focused checks after grounding changes:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONPYCACHEPREFIX=/tmp/teto_codex_pycache .venv_lab/bin/python -m pytest -p no:cacheprovider -q \
  tests/test_grounding_import_compatibility.py \
  tests/test_vlm_grounding_adapter.py \
  tests/test_geometry_validity.py \
  tests/test_real_scene_shadow_pipeline.py \
  tests/test_perception_shadow_pipeline.py \
  tests/test_projector_shadow.py
```

For full validation, also run compileall, full pytest, and `git diff --check`.
Use only `bash -n` for Real and Isaac startup scripts unless a task explicitly
allows startup.

## Future Split Candidates

H9 completed these grounding helper splits:

- `forbidden_fields.py`
- `scene_binding.py`

Later audits may consider small, behavior-preserving splits for:

- `confidence_gate.py`
- `target_selection.py`

Any future split must preserve status/error constants and avoid changing
runtime, startup, or safety behavior.
