# Vision Snapshot Namespace

This package is a future home for scene/camera snapshot implementation.

H11-A4 added package-side compatibility adapter modules. H11-A5 moved only the
`camera_snapshot` implementation here. H11-A6 moved only the
`camera_source_adapter` implementation here. Production imports have not
migrated yet.

Current implementation status:

- `src/vision/snapshot/camera_snapshot.py`: current implementation.
- `src/camera_snapshot.py`: temporary root compatibility shim.
- `src/vision/snapshot/camera_source_adapter.py`: current implementation.
- `src/camera_source_adapter.py`: temporary root compatibility shim.
- `src/realsense_snapshot_builder.py`: current root implementation, not moved
  yet.

The current CLI entrypoint remains in:

- `scripts/build_realsense_snapshot_bundle.py`

Current package-side adapter modules are:

- `src.vision.snapshot.camera_snapshot`
- `src.vision.snapshot.camera_source_adapter`
- `src.vision.snapshot.realsense_snapshot_builder`

The `realsense_snapshot_builder` adapter explicitly imports public symbols from
the current root module so future code can test the package path before
implementation migration. The `camera_snapshot` and `camera_source_adapter`
modules now own their implementations. None of these modules add behavior,
mutate constants, hide errors, or start services.

`src/vision/snapshot/__init__.py` remains conservative and does not re-export
APIs from the package root. Import concrete adapter modules directly.

Future H11-A7 should consider moving `realsense_snapshot_builder.py` only with
extra CLI and artifact-path care, while keeping root modules as compatibility
shims until production imports are migrated in a later step.
