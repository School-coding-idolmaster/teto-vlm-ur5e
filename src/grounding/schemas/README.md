# Grounding Schema Boundary

This directory is reserved for future schema notes for Qwen grounding results,
2D target evidence, projected 3D targets, confidence fields, and red mug hover
target records.

H8 did not add runtime dataclasses, validators, imports, or call sites here.
After H9-A1 and H9-A3, grounding schema-adjacent logic now lives in:

- `src/grounding/result.py`
- `src/grounding/vlm_adapter.py`
- `src/grounding/command_normalization.py`
- `src/grounding/reporting.py`
- `src/grounding/forbidden_fields.py`
- `src/grounding/scene_binding.py`

The legacy root-level grounding shims were removed in H9-A8B:

- `src/grounding_result.py`
- `src/vlm_grounding_adapter.py`

Do not treat this directory as proof that grounding contracts live in the old
`src` root. Current grounding guidance is in `docs/module_guides/grounding.md`
and `src/grounding/README.md`.
