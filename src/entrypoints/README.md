# Entrypoints Boundary

This package is reserved for future Python entrypoint helpers and operator
launch boundary notes. It is named `entrypoints` to avoid confusing package
code with the Python standard library `operator` module.

Existing files that already carry related responsibilities include:

- `scripts/teto_operator_console.py`
- `scripts/teto_isaac_operator_console.py`
- `scripts/start_teto_real_full_stack.sh`
- `scripts/start_teto_qwen_real_operator.sh`
- `scripts/start_teto_isaac_gui_operator.sh`
- `scripts/qwen_operator_console.sh`
- `src/unified_segmented_operator.py`

H8 intentionally leaves those files in place. Current real and Isaac
entrypoints were recently separated, and startup behavior must not change in
this phase.

This package is only a future boundary. H8 does not change runtime behavior,
startup behavior, real robot behavior, Isaac behavior, or safety semantics.
