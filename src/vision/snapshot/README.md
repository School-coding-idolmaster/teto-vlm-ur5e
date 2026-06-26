# Vision Snapshot Namespace

This package is the home for scene/camera snapshot implementation.

H11-A4 added package-side compatibility adapter modules. H11-A5 moved only the
`camera_snapshot` implementation here. H11-A6 moved only the
`camera_source_adapter` implementation here. H11-A7 moved the
`realsense_snapshot_builder` implementation here. H11-A8-1 migrated focused
tests to package imports, and H11-A8-2 migrated the RealSense snapshot bundle
CLI import. H11-A8-3 migrated the first production `src/` import batch:
`src/geometry_validity.py` and `src/real_scene_shadow_pipeline.py`. H11-A8-4
migrated the second production `src/` import batch:
`src/perception_shadow_pipeline.py` and `src/simulation_runtime.py`. H11-A8-5
migrated the final production `src/` import group: `src/cli.py` and
`src/evidence_exporter.py`. H11-A9 removed the root snapshot compatibility
shims after readiness scans found no real `src/` or `scripts/` consumers.

Current implementation status:

- `src/vision/snapshot/camera_snapshot.py`: current implementation.
- `src/vision/snapshot/camera_source_adapter.py`: current implementation.
- `src/vision/snapshot/realsense_snapshot_builder.py`: current implementation.
- `src/camera_snapshot.py`: removed root compatibility shim.
- `src/camera_source_adapter.py`: removed root compatibility shim.
- `src/realsense_snapshot_builder.py`: removed root compatibility shim.

The current CLI entrypoint remains in:

- `scripts/build_realsense_snapshot_bundle.py`

Current package-side adapter modules are:

- `src.vision.snapshot.camera_snapshot`
- `src.vision.snapshot.camera_source_adapter`
- `src.vision.snapshot.realsense_snapshot_builder`

The `camera_snapshot`, `camera_source_adapter`, and
`realsense_snapshot_builder` modules now own their implementations. The old
root modules no longer exist. None of these modules add behavior, mutate
constants, hide errors, or start services.

`src/vision/snapshot/__init__.py` remains conservative and does not re-export
APIs from the package root. Import concrete adapter modules directly.

Canonical imports are:

- `src.vision.snapshot.camera_snapshot`
- `src.vision.snapshot.camera_source_adapter`
- `src.vision.snapshot.realsense_snapshot_builder`
