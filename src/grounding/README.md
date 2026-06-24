# Grounding Boundary

This package is reserved for future Qwen grounding, target selection, 2D
grounding contracts, 3D target construction, and red mug hover evidence.

Existing files that already carry related responsibilities include:

- `src/vlm_grounding_adapter.py`
- `src/grounding_result.py`
- `src/geometry_validity.py`
- `src/projector_shadow.py`
- `src/real_scene_shadow_pipeline.py`
- `src/perception_shadow_pipeline.py`

H8 intentionally leaves those files in place. They enforce snapshot identity,
confidence, geometry, projection, and no-robot-command rules through existing
tests. Moving them now would risk changing those safety boundaries.

This package is only a future boundary. H8 does not change runtime behavior,
startup behavior, real robot behavior, Isaac behavior, or safety semantics.
