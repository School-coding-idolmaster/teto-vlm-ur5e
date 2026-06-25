# Projector Package

This package contains the current concrete projector shadow implementation for
the 2D-to-3D metric layer.

Current public import path:

- `src.projector.shadow`

Removed compatibility shim file:

- `src/projector_shadow.py`

The root-level compatibility shim was removed in H10-A5. New runtime code and
ordinary tests must import from `src.projector.shadow`. Do not add an import
hack for the removed root-level module.

Artifact, config, and evidence names such as `projector_shadow_config`,
`projector_shadow_result.json`, and `projector_shadow_report.md` remain data
contract names, not Python import paths.

## Current Files

- `shadow.py`: V2.9.2 projector shadow implementation. It converts accepted
  geometry evidence, pixel center, depth, camera intrinsics, and mock/config TF
  into `camera_point_m` and `world_point_m` evidence.
- `__init__.py`: conservative package root with an empty `__all__`.

`src/projector_contract.py` is intentionally not migrated yet. It remains the
older semantic dry-run eligibility contract used by replay/readiness tooling
and does not compute metric points.

## Boundary

The projector package is responsible for:

- depth value extraction
- camera intrinsics validation
- pinhole pixel/depth to `camera_point_m`
- mock/config transform to `world_point_m`
- projector-local workspace checks
- projector-local no-motion and TF audit evidence

The projector package is not responsible for:

- VLM grounding
- scene snapshot capture
- live RealSense operation
- MoveIt planning
- robot execution
- safety-critical execution gates

Do not change projection math, depth/intrinsics semantics, TF semantics,
workspace checks, no-motion safety fields, error ordering, or output field
names without a dedicated behavior-change audit.
